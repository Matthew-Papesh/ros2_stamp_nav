#!/usr/bin/env python3
import rclpy # base libs
from rclpy.node import Node 
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
# message libs 
from geometry_msgs.msg import TwistStamped, PoseStamped, PoseWithCovariance, Point
from nav_msgs.msg import Path, Odometry, OccupancyGrid, GridCells
from visualization_msgs.msg import Marker, MarkerArray
from ros2_stamp_nav_interfaces.srv import Spline
# program handlers 
import ros2_stamp_nav.handler as handler
import ros2_stamp_nav.PID as PID
import threading
import numpy as np
import math
import heapq 

class Navigator(Node):
    """
    Represents a navigation node for a TurtleBot3 robot model for spline-path trajectory driving. 
    """
    def __init__(self):
        """
        Creates a Navigator class instance
        """
        super().__init__("navigator")
        self.odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        self.map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        # thread for driving
        self.drive_thread = None
        self.drive_running = False

        # navigation way points:
        self.waypoints = []
        # create publishers
        self.speed_pub_ = self.create_publisher(TwistStamped, "/cmd_vel", 10)
        self.cspace_pub_ = self.create_publisher(OccupancyGrid, "/navigator/cspace_map", self.map_qos)
        self.simple_path_pub_ = self.create_publisher(GridCells, "/navigator/simple_path", self.map_qos)
        self.inflated_path_pub_ = self.create_publisher(GridCells, "/navigator/inflated_path", self.map_qos)
        self.waypoint_marker_pub_ = self.create_publisher(MarkerArray, "/navigator/waypoint_markers", self.map_qos)
        # create subscribers 
        self.odom_sub_ = self.create_subscription(Odometry, "/odom", self.odom_callback, self.odom_qos)
        self.map_sub_ = self.create_subscription(OccupancyGrid, "/map", self.map_callback, self.map_qos)
        self.goal_pose_sub_ = self.create_subscription(PoseStamped, "/goal_pose", self.goal_pose_callback, 10)

        # create timer
        self.timer = self.create_timer(1, self.loop)
        # create service clients 
        self.spline_called = False
        self.spline_srv = self.create_client(
            Spline,
            "/spline_path/spline_plan"
        )

        # class positions fields
        self.curr_position = handler.get_pose_stamped(0,0,0)
        self.goal_position = None
        self.curr_speed = TwistStamped()
        # class map fields 
        self.map_data = None
        self.cspace_data = None
        self.map_header = None
        self.map_dims = (0,0)
        self.map_pos = (0,0)
        self.map_res = 0

        # absolute maximum centripetal acceleration given simulation physics
        coeff_static_friction = 1.0
        centripetal_acceleration = coeff_static_friction * 9.81 # [m/sec^2]
        # percentage of centripetal acceleration to consider when specifying 
        # max centripetal acceleration for spline path driving
        scaler = 0.8
        
        # motion profiling criteria:
        self.MAX_CENTRIPETAL_ACCELERATION = centripetal_acceleration * scaler # [m/sec^2]
        self.ACCELERATION = 0.1 # [m/sec^2]
        self.MAX_ANGULAR_SPEED = 0.8 # [radians/sec]
        self.MAX_LINEAR_SPEED = 1.5 # [m/sec]
        # other robot constraints: 
        self.MAX_SPLINE_TURN = 0.5 * math.pi # [radians]; the max amount the robot should turn when driving a spline
        self.TURTLEBOT3_RADIUS = 0.105 # [m]
        # pid feedback coefficients 
        self.ANG_KP, self.ANG_KI, self.ANG_KD = 15.152344, 0.000083, 15.357032

        # spline srv responses 
        self.spline_path = None
        self.sd_steps = [] # used for motion profiling 
        self.cumulative_sd_steps = [] 
        # stall for srv
        while not self.spline_srv.wait_for_service(timeout_sec=10):
            self.set_speed(0,0)
            self.get_logger().info("Waiting for spline service")

    def odom_callback(self, msg: PoseWithCovariance):
        """
        Callback function for odometry listener
        """
        heading = handler.get_heading(msg.pose.pose.orientation)
        self.curr_position.pose.position.x = float(msg.pose.pose.position.x)
        self.curr_position.pose.position.y = float(msg.pose.pose.position.y)
        self.curr_position.pose.orientation = handler.get_orientation(heading)
        self.curr_position.header.frame_id = str(msg.header.frame_id)

    def map_callback(self, msg: OccupancyGrid):
        """
        Callback function for map listener
        """
        self.map_dims = (msg.info.width, msg.info.height)
        self.map_pos = (msg.info.origin.position.x, msg.info.origin.position.y)
        self.map_res = msg.info.resolution
        self.map_data = msg.data
        self.map_header = msg.header
        self.set_cspace_data(inflation_radius=self.map_res) 

    def goal_pose_callback(self, msg: PoseStamped): 
        """
        Callback function for goal pose listener
        """
        heading = handler.get_heading(msg.pose.orientation)
        self.goal_position = PoseStamped()
        self.goal_position.pose.position.x = float(msg.pose.position.x)
        self.goal_position.pose.position.y = float(msg.pose.position.y)
        self.goal_position.pose.orientation = handler.get_orientation(heading)
        self.goal_position.header.frame_id = str(msg.header.frame_id)

        # run spline driving with waypoints to goal position 
        self.waypoints = self.get_spline_path_waypoints(self.curr_position, self.goal_position)
        if self.waypoints is None: 
            self.goal_position = None
            self.waypoints = [] # reset 
            return
        self.set_rviz_waypoint_markers(self.waypoints)
        self.request_spline(self.waypoints)

    def driver_thread_handler(self):
        """
        Handler that is called upon opening a thread for motion-profiled spline driving. 
        """
        if self.goal_position is None:
            return # do nothing without goal
        try: 
            self.point_turn_drive(
                rotate=self.waypoints[0][2] - handler.get_heading(self.curr_position))
            self.drive_spline_path()
            self.point_turn_drive(
                rotate=handler.get_heading(self.goal_position) - self.waypoints[-1][2])
            self.reset_spline()
        finally:
            # reset driving and navigation
            self.drive_running = False
            self.goal_position = None 

    def get_markers(self, markers: list[tuple[float, float, float]]) -> MarkerArray:
        """
        Creates a `MarkerArray` instance from a list of 3D tuple points (x,y,heading).
        The instance will be mapped for alignment with the map data and cspace occupancy grid.
        :param markers [list[tuple[float, float, float]]] The specified list 
        """
        marker_array = MarkerArray()
        for i in range(0, len(markers)):
            marker = Marker()
            marker.header = self.map_header
            marker.id = i
            marker.type = Marker.ARROW
            marker.action = Marker.ADD

            marker.pose.position.x = float(markers[i][0])
            marker.pose.position.y = float(markers[i][1])
            marker.pose.position.z = 0.0
            marker.pose.orientation = handler.get_orientation(markers[i][2])

            marker.scale.x = 0.5
            marker.scale.y = 0.1
            marker.scale.z = 0.15
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 1.0

            marker_array.markers.append(marker)
        return marker_array

    def get_gridcells(self, grid: list[tuple[float, float]], resolution: float, x_offset: float=None, y_offset: float=None) -> GridCells:
        """
        Creates a `GridCells` instance from a list of 2D tuple points with a square length of the specified resolution. 
        The instance will be mapped for alignment with the map data and cspace occupancy grid. 
        :param grid [list[tuple[float, float]]] The specified list of points 
        :param resolution [float] The specified cell square length
        :returns The instance 
        """
        x_offset = 0.5*self.map_res if x_offset is None else x_offset
        y_offset = 0.5*self.map_res if y_offset is None else y_offset
        gridcells = GridCells()
        gridcells.header = self.map_header
        gridcells.cell_width = resolution
        gridcells.cell_height = resolution

        for node in grid:
            point = Point()
            point.x = node[0] + x_offset
            point.y = node[1] + y_offset
            gridcells.cells.append(point)
        return gridcells

    def loop(self):
        """ Main node loop. """
        if self.goal_position is None:
            return # do nothing without a goal
        if self.spline_path == None and not self.spline_called:
            # add initial robot position to the front of the waypoints and request interpolated spline path. 
            temp_waypoints = [(
                self.curr_position.pose.position.x, 
                self.curr_position.pose.position.y, 
                self.waypoints[0][2] # point to next node
            )] + self.waypoints

            self.request_spline(waypoints=temp_waypoints)
        elif self.spline_path is not None and not self.drive_running: 
            self.drive_running = True
            self.drive_thread = threading.Thread(target=self.driver_thread_handler)
            self.drive_thread.start()
    
    def request_spline(self, waypoints: list):
        """
        Requests a interpolated spline path of poses given specified waypoints. 
        :param waypoints [list] The specified PoseStamped list of poses to constraint a spline onto
        :returns a simple spline Path object
        """
        self.spline_called = True
        request = Spline.Request()
        obstacles = OccupancyGrid()
        path = Path()

        obstacles.header = self.map_header
        obstacles.info.width = int(self.map_dims[0])
        obstacles.info.height = int(self.map_dims[1])
        obstacles.info.origin.position.x = float(self.map_pos[0])
        obstacles.info.origin.position.y = float(self.map_pos[1])
        obstacles.info.origin.orientation = handler.get_orientation(0)
        obstacles.info.resolution = float(self.map_res)

        for waypoint in waypoints:
            pose = PoseStamped()
            pose.pose.position.x = float(waypoint[0])
            pose.pose.position.y = float(waypoint[1])
            pose.pose.orientation = handler.get_orientation(float(waypoint[2]))
            path.poses.append(pose)

        if self.map_data != None: 
            obstacles.data = self.map_data
        request.waypoints_path = path
        future = self.spline_srv.call_async(request)
        future.add_done_callback(self.response_spline)

    def reset_spline(self):
        """
        Generate an interpolated spline path. 
        """
        self.spline_path = None 
        self.sd_steps = []
        self.cumulative_sd_steps = []

    def response_spline(self, future):
        """
        Retrieve service interpolated spline path
        """
        try: 
            response = future.result()
            self.get_logger().info("Spline path received!")
            self.spline_path = response.spline_path
            self.sd_steps = response.sd_steps 
            self.cumulative_sd_steps = response.cumulative_sd_steps
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
        self.spline_called = False
        
    def set_speed(self, linear: float, angular: float): 
        """
        Published robot command velocity. 
        """
        self.curr_speed.header.stamp = self.get_clock().now().to_msg()
        self.curr_speed.header.frame_id = "base_link"

        self.curr_speed.twist.linear.x = float(linear)
        self.curr_speed.twist.linear.y = 0.0
        self.curr_speed.twist.linear.z = 0.0
        self.curr_speed.twist.angular.z = float(angular)
        self.curr_speed.twist.angular.x = 0.0
        self.curr_speed.twist.angular.y = 0.0

        self.speed_pub_.publish(self.curr_speed)

    def set_rviz_simple_path(self, path: list):
        """
        Visualizes a specified path in rviz. 
        :param path [list] The specified list of 2D tuples for a simple path
        """
        msg = self.get_gridcells(grid=path, resolution=self.map_res)
        self.simple_path_pub_.publish(msg)

    def set_rviz_waypoint_markers(self, waypoints: list[tuple[float, float, float]]):
        """
        Visualizes specified waypoints in rviz.
        :param waypoints [list] The specified waypoints 
        """
        msg = self.get_markers(markers=waypoints)
        self.waypoint_marker_pub_.publish(msg)

    def get_map_cell_data(self, cell_x: int, cell_y: int) -> int | None:
        """
        Determines the occupancy value at given cell in the map data. 
        :param cell_x [int] The specified cell x
        :param cell_y [int] The specified cell y
        :returns Map occupancy value
        """
        width = self.map_dims[0]
        if self.map_data is None or width is None:
            return None
        i = width*cell_y + cell_x
        return self.map_data[i] if i < len(self.map_data) else None

    def get_map_data(self, x: float, y: float) -> int | None:
        """
        Determines te occupancy value closest associated with a specified c-space position. 
        :param x [float] The specified position x
        :param y [float] The specified position y
        :returns The occupancy value 
        """
        if self.map_data is None:
            return None
        origin_x = self.map_pos[0] 
        origin_y = self.map_pos[1]
        cell_x = int(math.floor((x-origin_x)/self.map_res))
        cell_y = int(math.floor((y-origin_y)/self.map_res))
        return self.get_map_cell_data(cell_x=cell_x, cell_y=cell_y)

    def set_cspace_data(self, inflation_radius: float):
        """
        Defines a configuration space (c-space) based on known `self.map_data`. 
        :param inflation_radius [float] The specified radius to dialate to from obstacles in the map 
        """
        if self.map_data is None or self.map_res <= 0 or inflation_radius < 0:
            return # ignore setting if you cant 
        # intialize space
        self.cspace_data = np.zeros((self.map_dims[1], self.map_dims[0]), dtype=np.int8)
        # intialize grid cspace
        cspace_occupancy_grid = OccupancyGrid()
        cspace_occupancy_grid.header.stamp = self.get_clock().now().to_msg()
        cspace_occupancy_grid.header = self.map_header
        cspace_occupancy_grid.info.width = self.map_dims[0]
        cspace_occupancy_grid.info.height = self.map_dims[1]
        cspace_occupancy_grid.info.resolution = self.map_res
        cspace_occupancy_grid.info.origin.position.x = self.map_pos[0]
        cspace_occupancy_grid.info.origin.position.y = self.map_pos[1]
        cspace_occupancy_grid.data = [0]*(self.map_dims[0]*self.map_dims[1])

        # find cell dialation based on resolution 
        cell_radius = int(inflation_radius / self.map_res)
        
        # handler for updating cspace by kernel rooted at (x,y) with given radius
        def set_occupancy(x: int, y: int, radius: int, occupancy: int):
            for xi in range(-radius, radius+1): 
                for yi in range(-radius, radius+1):
                    if 0 <= x+xi < self.map_dims[0] and 0 <= y+yi < self.map_dims[1]:
                        self.cspace_data[y+yi, x+xi] = occupancy

        # step through map data
        for x in range(0, self.map_dims[0]):
            for y in range(0, self.map_dims[1]):
                   # check kernel position for occupancy obstacle
                   occupancy = self.get_map_cell_data(x, y)
                   if occupancy >= 50: # inflate from occupied cell
                       set_occupancy(x, y, cell_radius, occupancy)

        # update rviz cspace
        cspace_occupancy_grid.data = self.cspace_data.flatten().tolist()
        self.cspace_pub_.publish(cspace_occupancy_grid)
                    
    def get_cspace_cell_data(self, cell_x: int, cell_y: int) -> int | None:
        """
        Retrieves the c-space occupancy value. 
        :param cell_x [int] The specified cell x 
        :param cell_y [int] The specified cell y
        :returns c-space occupancy value at cell 
        """
        if 0 <= cell_x < self.map_dims[0] and 0 <= cell_y < self.map_dims[1]:
            return self.cspace_data[cell_y, cell_x]
        return None

    def get_cspace_data(self, x: float, y: float) -> int | None:
        """
        Retrieves the c-space occupancy value closest to global coordinate specified. 
        :param x [float] The specified global x 
        :param y [float] The specified global y
        :returns The occupancy value
        """
        cell_x = int(math.floor((x-self.map_pos[0])/self.map_res))
        cell_y = int(math.floor((y-self.map_pos[1])/self.map_res))
        return self.get_cspace_cell_data(cell_x, cell_y)

    def get_spline_path_waypoints(self, pose0: PoseStamped, pose1: PoseStamped) -> list | None:
        """
        Retrives waypoints for interpolating splines for a path starting and ending at the specified poses. 
        :param pose0 [PoseStamped] The specified initial pose 
        :param pose1 [PoseStamped] The spedicied final pose 
        :returns List of waypoints 
        """
        # parse in path
        x0 = pose0.pose.position.x 
        y0 = pose0.pose.position.y 
        x1 = pose1.pose.position.x
        y1 = pose1.pose.position.y
        path = self.get_simple_path(x0, y0, x1, y1)
        # ignore bad paths 
        if path is None:
            return None
        
        self.set_rviz_simple_path(path) # visualize path
        # takes the difference between two points 
        def diff(p0: tuple[float, float], p1: tuple[float, float]) -> np.ndarray: 
            return np.array(p1, dtype=np.float64) - np.array(p0, dtype=np.float64)

        # define intitial/final waypoint params 
        heading_0 = math.atan2(path[1][1]-y0, path[1][0]-x0)
        heading_1 = math.atan2(y1-path[-2][1], x1-path[-2][0])
        cardinality_0 = diff(path[0], path[1])
        # define waypoints (x, y, heading), cardinality, and corners 
        waypoints = [(x0, y0, heading_0)]
        cardinalities = [cardinality_0]
        path_corners = [(x0, y0)]

        # step through path found via A* search
        for i in range(1, len(path)-1):
            prev_p = path[i-1]
            curr_p = path[i]   
            next_p = path[i+1]
            curr_cardinality = diff(curr_p, next_p)

            # ignore (by continuing): no direction change, same cardinality, adjacent path corners   
            if (np.all(diff(prev_p, curr_p) == curr_cardinality)
                or np.all(cardinalities[-1] == curr_cardinality)
                or (i+2 < len(path) and np.all(diff(next_p, path[i+2]) != curr_cardinality))): 
                continue

            # found corner 
            cardinalities.append(curr_cardinality)
            path_corners.append(curr_p)
        path_corners.append((x1, y1))

        cell_padding = 3
        waypoint_radius = self.map_res*2.0
        # retreive mid-points between path corners from A* search
        for i in range(0, len(path_corners)-1):
            curr_p = path_corners[i]
            next_p = path_corners[i+1]
            # mid point waypoint position
            xi = int((0.5*(curr_p[0]+(0.5*(curr_p[0] + next_p[0]))) - self.map_pos[0])/self.map_res)
            yi = int((0.5*(curr_p[1]+(0.5*(curr_p[1] + next_p[1]))) - self.map_pos[1])/self.map_res)
            ci = cardinalities[i]
            heading_i = float(np.arctan2(ci[1], ci[0]))

            # this section slices a kernel of cspace locally near  mid point
            left_ang = heading_i + 0.5*math.pi
            right_ang = heading_i - 0.5*math.pi

            left_dx = cell_padding*math.cos(left_ang)
            left_dy = cell_padding*math.sin(left_ang)
            right_dx = cell_padding*math.cos(right_ang)
            right_dy = cell_padding*math.sin(right_ang)

            left_x = xi + left_dx
            left_y = yi + left_dy
            right_x = xi + right_dx
            right_y = yi + right_dy

            min_x, max_x = int(max(0, min(left_x, right_x))), int(min(self.map_dims[0], max(left_x, right_x)+1))
            min_y, max_y = int(max(0, min(left_y, right_y))), int(min(self.map_dims[1], max(left_y, right_y)+1))
            
            # slice to get local free space in cspace (filter out obsticles)
            sqr_kernel = self.cspace_data[min_y:max_y, min_x:max_x]
            y_idxs, x_idxs = np.ogrid[min_y:max_y, min_x:max_x]
            circle_mask = (y_idxs-yi)**2.0 + (x_idxs-xi)**2.0 <= cell_padding**2.0

            free_cells = (sqr_kernel == 0) & circle_mask
            total_free_cells = np.sum(free_cells)
            
            # calculate centroids in free space
            if total_free_cells > 0:
                free_idxs = np.argwhere(free_cells) # get free cell indeces
                # all free cell global indeces 
                global_free_x = free_idxs[:, 1] + min_x
                global_free_y = free_idxs[:, 0] + min_y
                # find centroid 
                centroid_x = np.sum(global_free_x) / total_free_cells
                centroid_y = np.sum(global_free_y) / total_free_cells
                # record waypoint as mid-point centroid between corners of A* search
                waypoints.append((
                    centroid_x*self.map_res + self.map_pos[0],
                    centroid_y*self.map_res + self.map_pos[1],
                    heading_i
                ))
            else:
                waypoints.append((
                    xi, yi, heading_i
                ))

        # add final waypoint 
        waypoints.append((x1, y1, heading_1))
        # filter out unnecessary waypoints to close to others  
        filtered_waypoints = []
        for i in range(0, len(waypoints)-1):
            curr_x, curr_y, curr_heading = waypoints[i]
            next_x, next_y, _ = waypoints[i+1]
            if (next_x-curr_x)**2 + (next_y-curr_y)**2 <= waypoint_radius**2:
                continue
            # adjust headings
            filtered_waypoints.append((
                curr_x, curr_y, 
                curr_heading
            ))
        # infer final heading from A* search
        heading_1 = math.atan2(
            y1-path[-2][1],
            x1-path[-2][0]
        )

        filtered_waypoints.append((x1, y1, heading_1))
        return filtered_waypoints
        
    def get_simple_path(self, x0: float, y0: float, x1: float, y1: float) -> list | None:
        """
        Searches the c-space and retrieves the best path from position (x0,y0) to position (x1,y1) with A*.
        :param x0 [float] The specified initial x 
        :param y0 [float] The specified initial y
        :param x1 [float] The specified final x
        :param y1 [float] The specified final y
        :returns a found c-space path of tuple cell positions 
        """
        if self.map_data is None or self.map_res <= 0:
            return None 
        # transform (x0,y0)
        cell_x0 = int(math.floor((x0 - self.map_pos[0])/self.map_res))
        cell_y0 = int(math.floor((y0 - self.map_pos[1])/self.map_res))
        # transform (x1,y1)
        cell_x1 = int(math.floor((x1 - self.map_pos[0])/self.map_res))
        cell_y1 = int(math.floor((y1 - self.map_pos[1])/self.map_res))

        # retrives neigbors to cell (x,y)
        def get_neighbors(cell_x: int, cell_y: int) -> list:
            offsets = [(1,0), (0,-1), (-1,0), (0,1)]
            neighbors = []

            for offset in offsets:
                dx, dy = offset
                occup = self.get_cspace_cell_data(cell_x+dx, cell_y+dy)
                if occup is not None and occup == 0:
                    neighbors.append((cell_x+dx, cell_y+dy))
            return neighbors

        frontier = [] # nodes just visited 
        explored = {(cell_x0, cell_y0): None} # nodes prev visited and left frontier 
        costs = {(cell_x0, cell_y0): 0} # cost for position (xi,yi) from pose0
        heapq.heappush(frontier, (0, (cell_x0, cell_y0)))

        while frontier:
            # retrieve next best from frontier 
            _, curr_pos = heapq.heappop(frontier)
            # check for pose1 
            if curr_pos == (cell_x1, cell_y1):
                path = []
                while curr_pos is not None: # reconstruct found path
                    tf_x = curr_pos[0]*self.map_res + self.map_pos[0]
                    tf_y = curr_pos[1]*self.map_res + self.map_pos[1]
                    path.append((tf_x, tf_y))
                    curr_pos = explored[curr_pos]
                path.reverse()
                return path

            # step through neighbors 
            for neighbor in get_neighbors(curr_pos[0], curr_pos[1]):
                new_neigh_cost = costs[curr_pos] + 1 # cost from best node 
                
                #  updated neighbor cost if unexplored or if new cost is better
                if neighbor not in explored or new_neigh_cost < costs[neighbor]:
                    heuristic = (cell_x1-neighbor[0])**2.0 + (cell_y1-neighbor[1])**2.0
                    explored[neighbor] = curr_pos
                    # update cost (G) and predicted cost (F)
                    costs[neighbor] = new_neigh_cost 
                    pred_cost = new_neigh_cost + heuristic
                    heapq.heappush(frontier, (pred_cost, neighbor))
        return None # no path found

    def get_pose_index(self, spline_path: Path, pose: PoseStamped, kernel_index: int, padding: int) -> tuple[int]:
        """
        Calculates the pose index along a path that most closely approximates a specified pose. 
        :param path [Path] The specified path
        :param pose [PoseStamped] The specified pose to consider
        :param kernel_index [int] The specified position of the kernel or scope to evaluate 
        :param padding [int] The specified radius of the kernel or scope to evaluate
        :returns The most similar path position to that of the specified pose as an index
        """
        min_loss = None
        ideal_pose_index = 0
        for i in range(max(0, kernel_index - padding), min(kernel_index + padding + 1, len(spline_path.poses))):
            path_pose = spline_path.poses[i]
            x_loss = abs(path_pose.pose.position.x - pose.pose.position.x)
            y_loss = abs(path_pose.pose.position.y - pose.pose.position.y)
            loss = (x_loss + y_loss) / 2.0
            if min_loss is None or loss < min_loss:
                min_loss = loss
                ideal_pose_index = i
        return ideal_pose_index

    def get_path_speeds(self, spline_path: Path, sd_steps: list, cumulative_sd_steps: list, acceleration: float, max_linear_speed: float, max_angular_speed: float, max_centripetal_acceleration: float) -> tuple[list, list]:
        """
        Calculates speeds for each pose along a spline path. Linear and angular speeds are are determined given pose data while stepping through 
        the path to interpolate speeds knowing the specified acceleration. Pose data used come from the i-th pose along the spline, the i-th arc distance 
        between a pose and the previous on the spline from sd_steps, and the i-th arc distance between the initial position and the i-th pose from cumulative_sd_steps.
        Motion profiling criteria are taken into account as well. The following being the criteria of linear acceleration [m/sec^2], max linear speed [m/sec], 
        max angular speed [radians/sec], and max centripetal acceleration [m/sec^2]; motion criteria clamps speeds within absolution boundaries between zero and 
        the maximums specified and only changes speeds by the rate of acceleration specified. Finally, speeds are return in the form of a tuple of linear and angular 
        speeds of the same length and corresponding MSE order of the spline path specified. 

        :param spline_path [Path] The specified spline path to profile speeds for
        :param sd_steps [list] The specified list of float arc distances between a given pose and the previous on a spline path
        :param cumulative_sd_steps [list] The specified list of float cumulative arc distances between a given pose and the starting position of a spline path
        :param acceleration [float] The specified magnitude of acceleration in [m/sec^2] to profile by
        :param max_linear_speed [float] The specified max magnitude of tangential speed in [m/sec] to profile by
        :param max_angular_speed [float] The specified max magnitude of angular speed in [radians/sec] to profile by
        :param max_centripetal_acceleration [float] The specified max magnitude of centripetal acceleration in [m/sec^2] to profile by
        :returns a tuple of linear speeds and angular speeds respectively in the same corresponding order as the spline path specified 
        """
        tolerance = 1 # how much reach +/- the index to select other points for approximating a spline circle
        base_index = tolerance 
        acceleration = max(0.00001, abs(acceleration)) 
        # computed speeds
        linear_speeds = []
        angular_speeds = []
        deccelerate = False

        # iterate through spline waypoints to compute speeds
        for index in range(0, len(spline_path.poses)):
            # scroll tolerance range while iterating
            if index > base_index and index <= len(spline_path.poses) - tolerance - 1:
                base_index = index
            # compute instantaneous circle approximating continuous spline at given point
            p0, p1, p2 = spline_path.poses[base_index - tolerance], spline_path.poses[base_index], spline_path.poses[base_index + tolerance]
            x_ICC, y_ICC, R = handler.get_circle((p0.pose.position.x, p0.pose.position.y), (p1.pose.position.x, p1.pose.position.y), (p2.pose.position.x, p2.pose.position.y))
            
            # determine initial velocity (v_0) and arc distance (sd) coming from previous waypoint to the current
            v0 = 0 if index == 0 else linear_speeds[len(linear_speeds) - 1]
            sd = sd_steps[index]
            # determine the direction (sign of angular speed (w_sgn)) of which the robot will turn
            delta_theta = handler.get_heading(p2) - handler.get_heading(p0)
            w_sgn = delta_theta / handler.non_zero(abs(delta_theta), 0.00001)
            # compute current waypoint's linear and angular velocity by kinematics
            v1 = pow(abs(pow(v0, 2.0) + 2.0*sd*acceleration), 0.5)
            w1 = v1 / handler.non_zero(abs(R), 0.00001) * w_sgn
            # compute centripetal acceleration (a_c) and decceleration distance
            a_c = pow(v1, 2.0) / handler.non_zero(abs(R), 0.00001)
            deccel_distance = abs(-pow(v1, 2.0) / (2.0 * acceleration))
            # check to deccelerate or not: check if remaining spline distance is longer than the distance needed to deccelerate
            remaining_distance = abs(cumulative_sd_steps[len(cumulative_sd_steps) - 1] - cumulative_sd_steps[index])
            remaining_distance = remaining_distance if remaining_distance > 0.05 else 0
            if not deccelerate and remaining_distance <= deccel_distance:
                # if just flagged True, the robot must of just noticed its speed, or any faster, would require slowing now to not overshoot the
                # end of the spline. Once deccelerating, the robot should not wait to slow or ever go faster; don't consider handling those cases.
                deccelerate = True
                acceleration = -abs(acceleration)
            if abs(a_c) > abs(max_centripetal_acceleration):
                # the robot must of just met/passed the max centripetal acceleration threshold; do not accelerate here but hold constant speed at the max linear speed.
                w1 = v0 / handler.non_zero(abs(R), 0.00001) * w_sgn
                v1 = v0
            elif abs(w1) > abs(max_angular_speed):
                # the robot must of just met/passed the max angular speed threshold; do not accelerate here but hold constant speed at the max angular speed.
                w1 = max_angular_speed * w_sgn
                v1 = max_angular_speed * R
            elif abs(v1) > abs(max_linear_speed):
                # the robot must of just met/passed the max linear speed threshold; do not accelerate here but hold constant speed at the max linear speed. 
                w1 = max_linear_speed / handler.non_zero(abs(R), 0.00001) * w_sgn
                v1 = max_linear_speed
            elif deccelerate and remaining_distance > deccel_distance:
                # if the robot is slowing down to soon stop but find the stop may come too early, then hold speeds constants
                w1 = v0 / handler.non_zero(abs(R), 0.00001) * w_sgn
                v1 = v0
            # add speeds
            linear_speeds.append(v1)
            angular_speeds.append(w1)
    
        return (linear_speeds, angular_speeds)

    def point_turn_drive(self, rotate: float): 
        """
        Rotates the robot in-place by the specified angle. 
        :param rotate [float] The specified angle in radians
        """
        # headings for turning
        init_heading = handler.get_heading(self.curr_position)
        final_heading = init_heading + rotate

        # defines the error function for PID
        def feedback_process_variable() -> float:
            curr_heading = handler.get_heading(self.curr_position) 
            error = final_heading - curr_heading
            # map error between [-pi, +pi]
            error = (error + math.pi) % (2.0*math.pi) - math.pi
            return error
        
        # pid angular speed feedback controller 
        angular_feedback = PID.PID(
            kp=1.0, ki=0.0001, kd=0.5, 
            process_variable=feedback_process_variable, 
            set_point=lambda: 0.0, 
            clegg_integration=True
        ) 

        speed = angular_feedback.output()
        while abs(speed) >= 0.01: 
            speed = angular_feedback.output()
            self.set_speed(linear=0, angular=speed)

    def spline_drive(self, spline_path: Path, spline_sd_steps: list, spline_cumulative_sd_steps: list, acceleration: float, max_linear_speed: float, max_angular_speed: float, max_centripetal_acceleration: float):
        """
        Motion profiles and drives wheels speeds along a specified spline path given path poses, path arc distances, along with specified acceleration [m/sec^2], max linear speed [m/sec],
        max angular speed [radians/sec], and max centripetal acceleration [m/sec^2]. 
        :param spline_path [Path] The specified spline path of interpolated poses
        :param spline_sd_steps [list] The specified list of float arc distances between a given spline path pose and the previous. 
        :param spline_cumulative_sd_steps [list] The specified list of float cumulative arc distances between a given spline path pose and the starting position. 
        :param acceleration [float] The specified acceleration in [m/sec^2] to profile by
        :param max_linear_speed [float] The specified max linear speed in [m/sec] to profile by
        :param max_angular_speed [float] The specified max angular speed in [radians/sec] to profile by
        :param max_centripetal_acceleration [float] The specified max centripetal acceleration in [m/sec^2] to profile by
        """
        # current index point considered from a set of points about a base point on a spline path with a radius range of padding
        index, padding = 0, 10
        # feedback control offset from the current index to target when applying PID feedback systems
        feedback_target_offset = 3

        # create angular speed pid feedback handler based on the current index point along a spline
        def feedback_process_variable() -> float:
            orig_x = self.curr_position.pose.position.x
            orig_y = self.curr_position.pose.position.y
            orig_radians = handler.get_heading(self.curr_position)

            spline_x = spline_path.poses[min(index + feedback_target_offset, len(spline_path.poses) - 1)].pose.position.x
            spline_y = spline_path.poses[min(index + feedback_target_offset, len(spline_path.poses) - 1)].pose.position.y
            
            variable = handler.rotate(spline_x - orig_x, spline_y - orig_y, -orig_radians)[1]
            return variable

        # compute wheel speeds
        linear_speeds, angular_speeds = self.get_path_speeds(
            spline_path, 
            spline_sd_steps, 
            spline_cumulative_sd_steps, 
            acceleration, 
            max_linear_speed, 
            max_angular_speed, 
            max_centripetal_acceleration
        )
        # pid angular speed feedback controller 
        angular_speed_feedback = PID.PID(
            kp=self.ANG_KP, 
            ki=self.ANG_KI, 
            kd=self.ANG_KD, 
            process_variable=feedback_process_variable, 
            set_point=lambda: 0.0, 
            clegg_integration=True
        )

        # max time to travel between waypoints  
        frontier_timeout = 10
        # time stamp of when last waypoint was reached 
        prev_frontier_update_time = -1.0
        min_position_error, frontier_index = None, -1

        # drive robot with speed data and feedback control
        while index < len(spline_path.poses) - 1:
            # approximate current position (current position; not recorded approximates to waypoints)
            index = self.get_pose_index(spline_path, self.curr_position, index, padding) 
            # current position error as a vector magnitude at any time
            raw_x_error = (self.curr_position.pose.position.x - spline_path.poses[index].pose.position.x)
            raw_y_error = (self.curr_position.pose.position.y - spline_path.poses[index].pose.position.y)
            
            position_x_pct_error = raw_x_error / (2.0 * self.TURTLEBOT3_RADIUS)
            position_y_pct_error = raw_y_error / (2.0 * self.TURTLEBOT3_RADIUS)
            position_error = handler.euclid_distance((0,0), (position_x_pct_error, position_y_pct_error))
            
            # check if the robot is taking too long to progress along the path; 
            if index > padding and prev_frontier_update_time > 0 and self.get_clock().now().nanoseconds/1e9 - prev_frontier_update_time > frontier_timeout:
                self.set_speed(0, 0) # stop driving, end early, and return pct performance errors of 100% to indicate faulty path driving
            # record the closest point the robot drove to 
            if index > frontier_index:
                prev_frontier_update_time = self.get_clock().now().nanoseconds / 1e9 
                frontier_index = index
            # evaluate the closest the robot drove by min error for current point
            elif min_position_error is None or position_error < min_position_error:
                min_position_error = position_error
                recorded_pose = self.curr_position

            # look up memoized speed calculations given position index and system feedback control
            ang_speed = angular_speeds[index] + angular_speed_feedback.output() 
            lin_speed = linear_speeds[index]
            self.set_speed(lin_speed, ang_speed) 
        # come to a stop and return data
        self.set_speed(0, 0)

    def drive_spline_path(self):
        """
        Drives a path of splines given waypoints to constrain the spline onto with motion profiling constraints of acceleration [m/sec^2], 
        max linear speed [m/sec], max angular speed [radians/sec], and max centripetal acceleration [m/sec^2]. The recorded path is returned alongside
        percent errors for position and heading.
        """ 
        self.spline_drive(
            self.spline_path, 
            self.sd_steps, 
            self.cumulative_sd_steps, 
            self.ACCELERATION,
            self.MAX_LINEAR_SPEED,
            self.MAX_ANGULAR_SPEED, 
            self.MAX_CENTRIPETAL_ACCELERATION
        )
        
def main(args=None):
    rclpy.init()
    node = Navigator()
    try: 
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass 
    finally: 
        node.destroy_node()
        rclpy.shutdown()
 
if __name__ == "__main__":
    main(args=None)