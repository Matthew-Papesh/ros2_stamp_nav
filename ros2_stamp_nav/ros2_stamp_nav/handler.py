#!/usr/bin/env python3
from __future__ import annotations
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from nav_msgs.msg import Path, GridCells
import numpy as np
import math

def get_ros_package_root(package_name):
    try:
        return get_package_share_directory(package_name)
    except Exception:
        return None

def get_heading(pose: PoseStamped | Quaternion) -> float:
    """
    Returns a heading measured in radians given a PoseStamped type. 
    param: pose [PoseStamped] The specified pose 
    returns: The corresponding pose heading in radians 
    """
    orien = None
    if isinstance(pose, PoseStamped): 
        orien = pose.pose.orientation 
    elif isinstance(pose, Quaternion): 
        orien = pose 
    quat_list = [orien.x, orien.y, orien.z, orien.w]
    heading = euler_from_quaternion(quat_list)
    return heading[2]

def get_orientation(heading: float) -> Quaternion:
    """
    Returns a PoseStamped.orientation type given a heading measured in radians. 
    param: heading [float] The specified heading 
    returns: a orientation type of the specified heading
    """
    q = Quaternion()
    half = float(heading) * 0.5
    q.x = 0.0
    q.y = 0.0
    q.z = float(math.sin(half))
    q.w = float(math.cos(half))
    return q

def get_pose_stamped(x: float, y: float, heading: float, frame_id: str = "map") -> PoseStamped:
    """
    Returns a PoseStamped type given a position and orientation in the world frame of reference.
    param: x [float] The specified x
    param: y [float] The specified y
    param: heading [float] The specified angle or orientation measured in radians
    returns: a pose stamped type of the specified pose
    """
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation = get_orientation(heading)
    return pose

def get_path(origin: tuple[float, float], resolution: float, x_coords: list, y_coords: list, frame_id: str = "map") -> Path:
    """
    Returns a Path type in world frame of reference for a specified list of coordinate pairs in a map 
    with a origin position and resolution in the world frame. 
    param: origin [tuple[float, float]] The specified origin position of the map in world coordinates
    param: resolution [float] The specified transfer rate from map to world coordinates
    param: x_coords [list] The specified list of x coordinates belonging to the map
    param: y_coords [list] The specified list of y coordinates belonging to the map
    returns: a path type of the specified coordinates
    """
    count = min(len(x_coords), len(y_coords))
    path = Path()
    for i in range(0, count):
        theta = 0.0
        if i < count - 1:
            theta = math.atan2(y_coords[i + 1] - y_coords[i], x_coords[i + 1] - x_coords[i])
        elif count > 1:
            theta = math.atan2(y_coords[i] - y_coords[i - 1], x_coords[i] - x_coords[i - 1])
        else:
            theta = 0.0
        x = origin[0] + x_coords[i] * resolution
        y = origin[1] + y_coords[i] * resolution
        path.poses.append(get_pose_stamped(x, y, theta, frame_id))
    return path

def get_gridcells_by_list(origin: tuple[float, float], resolution: float, map_x_coords: list, map_y_coords: list) -> GridCells:
    """
    Returns a GridCells type in world frame of reference for a specified list of coordinate pairs 
    in a map with a origin position and resolution in the world frame. 
    param: origin [tuple[float, float]] The specified origin position of the map in world coordinates
    param: resolution [float] The specified transfer rate from map to world coordinates
    param: map_x_coords [list] The specified list of x coordinates belonging to the map to generate 
    param: map_y_coords [list] The specified list of y coordinates belonging to the map to generate 
    returns: a gridcells type of the specified map coordinates 
    """
    gridcells = GridCells()
    gridcells.cell_width = 0.05
    gridcells.cell_height = 0.05
    gridcells.header.frame_id = "odom"
    count = min(len(map_x_coords), len(map_y_coords))
    for i in range(0, count):
        cell = Point()
        cell.x = origin[0] + float(map_x_coords[i]) * resolution
        cell.y = origin[1] + float(map_y_coords[i]) * resolution
        gridcells.cells.append(cell)
    return gridcells

def get_gridcells_by_path(origin: tuple[float, float], resolution: float, map_path: Path) -> GridCells:
    """
    Returns a GridCells type in a world frame of reference for a specified Path type in a map with a
    origin position and resolution in the world frame. 
    param: origin [tuple[float, float]] The specified origin position of the map in world coordinates
    param: resolution [float] The specified transfer rate from map to world coordinates
    param: map_path [Path] The specified Path type to record as GridCells
    returns: a gridcells type of the specified map path
    """
    gridcells = GridCells()
    gridcells.cell_width = 0.05
    gridcells.cell_height = 0.05
    gridcells.header.frame_id = "map"
    for pose in map_path.poses:
        cell = Point()
        cell.x = origin[0] + pose.pose.position.x * resolution
        cell.y = origin[1] + pose.pose.position.y * resolution
        gridcells.cells.append(cell)
    return gridcells

def euclid_distance(p_0: tuple[float, float], p_1: tuple[float, float]) -> float:
    """
    Returns the euclidean distance between two specified positions.
    param: p_0 [tuple[float, float]] The specified initial 2D position
    param: p_1 [tuple[float, float]] The specified final 2D position
    returns: the calculated euclidean distance 
    """
    return pow(pow(p_1[0] - p_0[0], 2.0) + pow(p_1[1] - p_0[1], 2.0), 0.5)

def non_zero(x: float, tolerance: float):
    """
    Returns the value x provided its magnitude is greater than or equal to the specified tolerance; else returns the tolerance to approximate zero.
    :param x [float] The specified value
    :param tolerance [float] The specified tolerance
    :returns a non zero value of x
    """
    magnitude = max(abs(tolerance), abs(x))
    sgn = 1.0 if x >= 0 else -1.0
    return magnitude * sgn

def rotate(x: float | np.ndarray, y: float | np.ndarray, radians: float) -> tuple[float, float] | tuple[np.ndarray, np.ndarray]:
    """
    Computes a rotational transformation on a vector (x,y) about the origin with the radians specified. 
    :param x [float | np.ndarray] The specified x
    :param y [float | np.ndarray] The specified y
    :param radians [float] The specified radians to rotate
    :returns the rotated vector
    """
    x_prime = x*np.cos(radians) - y*np.sin(radians)
    y_prime = x*np.sin(radians) + y*np.cos(radians) 
    return (x_prime, y_prime)

def get_circle(p_0: tuple[float, float], p_1: tuple[float, float], p_2: tuple[float, float]) -> tuple[float, float, float]:
    """
    Calculates a constrained circle given three specified positions in 2D space. 
    param: p_0 [tuple[float, float]] The specified first point
    param: p_1 [tuple[float, float]] The specified second point
    param: p_2 [tuple[float, float]] The specified third point
    returns: a vector of the circle center position in 2D space followed by the radius of the circle (i.e.: tuple[x=float, y=float, radius=float])
    """
    # given points:
    (x_0, y_0) = (p_0[0], p_0[1])
    (x_1, y_1) = (p_1[0], p_1[1])
    (x_2, y_2) = (p_2[0], p_2[1])
    
    # check straight line approximation
    area = abs(x_0 * (y_1 - y_2) + x_1 * (y_2 - y_0) + x_2 * (y_0 - y_1))
    if area < 1e-6:
        # return infinite radius for straight lines
        return (0.0, 0.0, float('inf'))

    # compute coefficients
    A = -(pow(x_0, 2.0)*y_1 - pow(x_0, 2.0)*y_2 - pow(x_1, 2.0)*y_0 + pow(x_1, 2.0)*y_2 + pow(x_2, 2.0)*y_0 - pow(x_2, 2.0)*y_1 + pow(y_0, 2.0)*y_1 - pow(y_0, 2.0)*y_2 - y_0*pow(y_1, 2.0) + y_0*pow(y_2, 2.0) + pow(y_1, 2.0)*y_2 - y_1*pow(y_2, 2.0))/non_zero(2*(x_0*y_1 - x_1*y_0 - x_0*y_2 + x_2*y_0 + x_1*y_2 - x_2*y_1), 0.000001)
    B = -(- pow(x_0, 2.0)*x_1 + pow(x_0, 2.0)*x_2 + x_0*pow(x_1, 2.0) - x_0*pow(x_2, 2.0) + x_0*pow(y_1, 2.0) - x_0*pow(y_2, 2.0) - pow(x_1, 2.0)*x_2 + x_1*pow(x_2, 2.0) - x_1*pow(y_0, 2.0) + x_1*pow(y_2, 2.0) + x_2*pow(y_0, 2.0) - x_2*pow(y_1, 2.0))/non_zero(2*(x_0*y_1 - x_1*y_0 - x_0*y_2 + x_2*y_0 + x_1*y_2 - x_2*y_1), 0.000001)
    C = (- pow(x_0, 2.0)*x_1*y_2 + pow(x_0, 2.0)*x_2*y_1 + x_0*pow(x_1, 2.0)*y_2 - x_0*pow(x_2, 2.0)*y_1 + x_0*pow(y_1, 2.0)*y_2 - x_0*y_1*pow(y_2, 2.0) - pow(x_1, 2.0)*x_2*y_0 + x_1*pow(x_2, 2.0)*y_0 - x_1*pow(y_0, 2.0)*y_2 + x_1*y_0*pow(y_2, 2.0) + x_2*pow(y_0, 2.0)*y_1 - x_2*y_0*pow(y_1, 2.0))/non_zero(x_0*y_1 - x_1*y_0 - x_0*y_2 + x_2*y_0 + x_1*y_2 - x_2*y_1, 0.000001)
    
    # computed circle properties
    x_center = -A
    y_center = -B
    R = pow(pow(A, 2.0) + pow(B, 2.0) - C, 0.5) 
    return (x_center, y_center, R)



