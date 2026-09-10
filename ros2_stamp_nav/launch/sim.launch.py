
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():

    pkg_share = get_package_share_directory("ros2_stamp_nav")
    # package locations
    tb3_gazebo_dir = get_package_share_directory("turtlebot3_gazebo")
    # define robot initial pose 
    x_pose = LaunchConfiguration("x_pose", default="0.0")
    y_pose = LaunchConfiguration("y_pose", default="0.0")
    z_pose = LaunchConfiguration("z_pose", default="0.0")
    yaw_pose = LaunchConfiguration("yaw_pose", default="0.0")

    # launch Gazebo world
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                tb3_gazebo_dir,
                "launch",
                "empty_world.launch.py"
            )
        ), 
        launch_arguments={
            "x_pose": x_pose,
            "y_pose": y_pose,
            "z_pose": z_pose,
            "yaw_pose": yaw_pose
        }.items()
    )

    # map handling
    map_yaml_path = os.path.join(pkg_share, "maps", "nav_map.yaml")
    map_odom_tf = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_odom_tf",
            arguments=[
                "--x", x_pose,
                "--y", y_pose,
                "--z", z_pose,
                "--yaw", yaw_pose,
                "--frame-id", "map",
                "--child-frame-id", "odom"
            ]
    )
    map_server = Node(
        package="nav2_map_server",
        executable="map_server", 
        name="map_server", 
        output="screen",
        parameters=[
            {"yaml_filename": map_yaml_path}, 
            {"use_bond": True}
        ]
    )
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager", 
        executable="lifecycle_manager", 
        name="lifecycle_manager", 
        output="screen",
        parameters=[
            {"node_names": ["map_server"]},
            {"autostart": True},
            {"bond_timeout": 10.0}
        ]
    )

    # RViz
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        parameters=[
            {
                "use_sim_time": True
            }
        ]
    )

    # nav node handles A* path search, motion profiling, and goal pose nav 
    navigator = Node(
        package="ros2_stamp_nav",
        executable="stamp_nav",
        name="nav_node",
        output="screen",                
        emulate_tty=True,               
        parameters=[                    
            {"use_sim_time": True}
        ]
    )
    # creates and optimizes quintic spline paths for navigator
    spline = Node(
        package="ros2_stamp_nav",
        executable="stamp_spline", 
        name="spline_node", 
        output="screen",
        emulate_tty=True,
        parameters=[
            {"use_sim_time": True}
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument("x_pose", default_value=x_pose, description="Robot X initial position"),
        DeclareLaunchArgument("y_pose", default_value=y_pose, description="Robot Y initial position"),
        DeclareLaunchArgument("z_pose", default_value=z_pose, description="Robot Z initial position"),
        DeclareLaunchArgument("yaw_pose", default_value=yaw_pose, description="Robot Yaw initial orientation"),

        world_launch,
        lifecycle_manager,
        map_server,
        map_odom_tf,
        rviz, 
        navigator, 
        spline
    ])