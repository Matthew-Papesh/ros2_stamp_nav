#!/usr/bin/env python3
import matplotlib.cm as cm
import ros2_stamp_nav.handler as handler
import numpy as np
from enum import Enum
import math
import rclpy

from rclpy.node import Node
from ros2_stamp_nav.quintic import Quintic
from ros2_stamp_nav_interfaces.srv import Spline
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import GridCells, OccupancyGrid
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# plotting flags
RVIZ_PLOT_SPLINE_GRIDCELLS = True
MATPLOTLIB_PLOT_QUINTIC = False

class SplinePath(Node):
    """
    Represents a SplinePath ROS node that generates a spline path of interpolated waypoints.
    """
    def __init__(self):
        """
        Creates a `SplinePath` class instance.
        """
        super().__init__("spline_path")
        # matplotlib coordinate range: ((min_x, max_x), (min_y, max_y))
        self.plot_range = ((-4, 10), (-3, 10))
        # map profile        
        self.map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
       
        # optimizer modes 
        self.optimizer_mode = self.OptMode.IDLE

        # class map fields 
        self.map_data = None
        self.cspace_data = None
        self.map_header = None
        self.map_dims = (0,0)
        self.map_pos = (0,0)
        self.map_res = 0
        # optimizer visualization
        self.candidate_splines = np.empty((0, 4), dtype=np.float32)
        self.dropout_splines = np.empty((0, 4), dtype=np.float32)
        self.OPTIMIZER_EPOCHS = 100

        # path data:
        self.path_x, self.path_y = None, None
        self.sd_steps, self.cumulative_sd_steps = None, None
        self.global_obstacles = None
        self.memoizing_obstacles = False
        # fills obstacle's cell plus an additional cspace cell 
        self.obstacle_radius = 0

        # publishers and subscribers:
        self.spline_gridcells_pub_ = self.create_publisher(GridCells, "/spline_path/spline_gridcells", 10)
        self.optimizer_training_pub_ = self.create_publisher(PointCloud2, "/spline_path/optimizer_training", self.map_qos)
        self.obstacle_map_sub_ = self.create_subscription(OccupancyGrid, "/map", self.map_callback, self.map_qos)
        # services 
        self.srv = self.create_service(Spline, "/spline_path/spline_plan", self.handle_spline_plan_service)
        # create timer 
        self.timer = self.create_timer(0.1, self.loop)
    
    class OptMode(Enum):
        IDLE = 0 
        INITIAL_TRAINING = 1 # trains for the initial set of waypoints
        DROPOUT_TRAINING = 2 # removes unnecessary waypoints with training
        HEADING_TRAINING = 3 # trains the final waypoint heading
        
    def map_callback(self, msg: OccupancyGrid):
        """
        Callback function for the obstacle map listener
        """
        self.get_logger().info("Obstacle map found! Memoizing...")
        # record map metadata 
        self.map_dims = (msg.info.width, msg.info.height)
        self.map_pos = (msg.info.origin.position.x, msg.info.origin.position.y)
        self.map_res = msg.info.resolution
        self.map_data = msg.data
        self.map_header = msg.header
        self.obstacle_radius = 1.5*msg.info.resolution

        # parse/memoize obstacle data; make np array matrix of Nx2 for N obstacles  
        np_cspace = np.array(self.map_data, dtype=np.int8).reshape(self.map_dims[1], self.map_dims[0])
        y_idxs, x_idxs = np.where(np_cspace > 0) # extract all obstacles 

        # transform to global coords 
        x_global = x_idxs*self.map_res + self.map_pos[0] 
        y_global = y_idxs*self.map_res + self.map_pos[1]
        # create final array
        self.global_obstacles = np.column_stack((x_global, y_global)) 
        self.get_logger().info("Memoized obstacles for spline optimization!")

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

    def handle_spline_plan_service(self, request, response):
        """
        Represents the GetSplinePlan service for a simple spline path that expects a specified Path of waypoints 
        to constrain the spline path to. Returns the interpolated Path of points of the constrained spline. 
        """
        self.get_logger().info("QuinticSpline.py: GetSplinePlan service request heard")
        self.candidate_splines = np.empty((0, 4), dtype=np.float32) # reset cached splines from optimizer
        self.dropout_splines = np.empty((0, 4), dtype=np.float32)
        self.set_rviz_optimizer_splines(splines=self.candidate_splines)

        simple_path = request.waypoints_path
        path_x, path_y, sd_steps, cum_sd_steps = self.get_interpolated_spline(simple_path.poses)
        spline_path = handler.get_path((0, 0), 1.0, path_x, path_y)
        
        if RVIZ_PLOT_SPLINE_GRIDCELLS: # view spline
            self.plot_spline_gridcells(self.path_x, self.path_y)
        
        response.spline_path=spline_path 
        response.sd_steps = sd_steps
        response.cumulative_sd_steps = cum_sd_steps
        return response

    def plot_spline_gridcells(self, path_x: list, path_y: list) :
        gridcells = handler.get_gridcells_by_list((0, 0), 1.0, path_x, path_y)
        gridcells.header.stamp = self.get_clock().now().to_msg()
        self.spline_gridcells_pub_.publish(gridcells)

    def optimizer_callback(self, epoch: int, x_arr: np.ndarray, y_arr: np.ndarray): 
        """
        Handles memoizing and publishing candidate splines from the optimizer. 
        :param epoch [int] The current optimizer training epoch 
        :param x_arr [np.ndarray] The global candidate spline x points 
        :param y_arr [np.ndarray] The global candidate spline y points
        """
        coord_count = min(len(x_arr), len(y_arr))
        z_arr = None 
        if self.optimizer_mode != self.OptMode.DROPOUT_TRAINING:
            z_arr = np.zeros(shape=coord_count, dtype=np.float32)
        else:
            lin_space = np.abs(np.linspace(-1.0, 1.0, num=coord_count, dtype=np.float32))
            poly_space = np.power(lin_space, 2.0)
            z_arr = 0.5*(1.0-poly_space)

        # normalize epochs and color code 
        norm_epoch = float(epoch) / float(self.OPTIMIZER_EPOCHS)
        r, g, b, _ = cm.plasma(norm_epoch)
        # opacity is lowered for training final heading
        alpha = 0.3 if self.optimizer_mode != self.OptMode.HEADING_TRAINING else 1.0
        # reformat color into 32 bit format 
        r_int, g_int, b_int, a_int = int(r*255), int(g*255), int(b*255), int(alpha*255)
        packed_int = (a_int << 24) | (r_int << 16) | (g_int << 8) | b_int
        
        # convert to float32
        packed_float = np.uint32(packed_int).view(np.float32)
        x_arr = x_arr[:coord_count].astype(np.float32)
        y_arr = y_arr[:coord_count].astype(np.float32)

        # handle color column
        color_arr = np.full(coord_count, packed_float, dtype=np.float32)
        # candidate spline coordinates 
        candidate_coords = np.column_stack((x_arr, y_arr, z_arr, color_arr))

        # cache coordinates and publish to rviz
        if self.optimizer_mode != self.OptMode.DROPOUT_TRAINING:
            self.candidate_splines = np.vstack((self.candidate_splines, candidate_coords))
            self.set_rviz_optimizer_splines(splines=self.candidate_splines)
        # cache dropout coordinates 
        elif self.optimizer_mode == self.OptMode.DROPOUT_TRAINING and epoch==0: 
            self.dropout_splines = np.empty((0, 4), dtype=np.float32) # create new dropout spline record
        if self.optimizer_mode == self.OptMode.DROPOUT_TRAINING:
            self.dropout_splines = np.vstack((self.dropout_splines, candidate_coords))

    def compute_spline(self, p0: PoseStamped, p1: PoseStamped) -> tuple[int, list, list, list]: 
        """
        Creates a single-spline path between points p0 and p1. Spline curvature is optimized with
        simulated annealing. 
        :param p0 [PoseStamped] The spceified initial waypoint 
        :param p1 [PoseStamped] The specified final waypoint 
        :returns A 4D tuple repesenting the spline cost, and lists for x points, y points, and discretized arc distance.
        """
        # record path arc distance
        sd_steps = []
        # find relative frame of reference
        delta_x = p1.pose.position.x - p0.pose.position.x
        delta_y = p1.pose.position.y - p0.pose.position.y
        rel_x, rel_y = handler.rotate(delta_x, delta_y, -handler.get_heading(p0))
        rel_theta = handler.get_heading(p1) - handler.get_heading(p0)

        # convert coords
        p_origin, p_rel = PoseStamped(), PoseStamped()
        p_rel.pose.position.x = rel_x
        p_rel.pose.position.y = rel_y
        p_rel.pose.orientation = handler.get_orientation(rel_theta)
            
        # compute the relative quintic spline function
        q = Quintic(p_origin, p_rel)
        if self.global_obstacles is not None: 
            q.set_obstacles( # define known map server obstacles 
                obstacles=self.global_obstacles, 
                radius=self.obstacle_radius
            )
        # transform from relative spline back to global map for obstacle cost calculations 
        q.set_inverse_transform( 
            x_origin=p0.pose.position.x,
            y_origin=p0.pose.position.y,
            theta_origin=handler.get_heading(p0)
        )
        q.set_curvature(0, 0)
        # optimize for object-avoidance with simulated annealing 
        opt_k0, opt_k1, cost = q.optimize(
            epochs=self.OPTIMIZER_EPOCHS, 
            k0=0.0, k1=0.0, T0=50, 
            step_size=0.2,
            push_sensitivity=0,
            callback_event=self.optimizer_callback
        )
        # show optimzation results 
        self.get_logger().info(f"\nTrained Spline: (k0, k1, cost) = {(
            round(opt_k0, 3), round(opt_k1, 3), int(cost))}")
        # the specified number of points to partition and interpolate
        partition_density = 50.0 
        euclid_distance = handler.euclid_distance((0.0, 0.0), (rel_x, rel_y))
        partitions = int(euclid_distance * partition_density)
            
        # the amount to step by through t-domain when interpolating
        t = np.linspace(0.0, 1.0, partitions)
        dt = 1.0 / (partitions + 1.0)
        data_x, data_y = q.f(t)

        # apply inverse transform 
        data_x, data_y = handler.rotate(data_x, data_y, handler.get_heading(p0))
        data_x += p0.pose.position.x
        data_y += p0.pose.position.y 

        # calculate arc distance steps for spline 
        dtdx, dtdy = q.dydx(t)
        instantaneous_steps = np.sqrt(dtdx**2.0 + dtdy**2.0) * dt
        sd_steps = sd_steps + instantaneous_steps.tolist()

        # return spline sub-path
        return (cost, data_x.tolist(), data_y.tolist(), sd_steps) 

    def get_interpolated_spline(self, waypoints: list[PoseStamped], dropout_err_pct: float=1) -> tuple[list, list, list, list]:
        """
        Retrieves the trained optimal spline path that follows the specified waypoints. The resulting path 
        will be optimized to be smooth, short, and avoid environment obstacles. Some given waypoints may not be used
        if the path is found to be better without; this is done by selectively removing waypoints in a process known as dropout. 
        :param waypoints [list[PoseStamped]] The specified list of waypoints 
        :param dropout_err_pct [float] Max absolute percent cost error allowed for waypoint dropout 
        :returns The 4D tuple of lists for interpolated x points, y points, step arc distances, and cumulative arc distance
        """
        # create subpoint lists
        path_data = {}
        path_costs = {}
        next_pt_map = {}
        # drop unecessary waypoints 
        dropout_cnt = 0
        last_dropout_cnt = 0
        max_dropout = int(math.ceil(0.75*len(waypoints)))
        # the final heading at end of driving through waypoints 
        # use a max +/- bound for adjusting the final end heading
        end_heading_bound = 0.5*math.pi
        end_heading_epochs = 2
        # recorded overall spline path
        path_x = []
        path_y = []
        sd_steps = []

        def query_cost(edge: tuple[PoseStamped, PoseStamped], ) -> int:
            """
            Computes and memoizes cost and path data that serves as a connecting edge between two waypoints. 
            """
            edge_id = (ptid(edge[0]), ptid(edge[1]))
            if edge_id not in path_costs:
                cost, data_x, data_y, sd = self.compute_spline(edge[0], edge[1])
                path_data[edge_id] = (data_x, data_y, sd)
                path_costs[edge_id] = cost
            return path_costs[edge_id]
        
        def ptid(pt: PoseStamped) -> tuple[float, float, float]:
            """
            Standardizes hash key id for a `PoseStamped` instance. 
            """
            x = round(pt.pose.position.x, 3)
            y = round(pt.pose.position.y, 3)
            heading = round(handler.get_heading(pt), 3)
            return (x, y, heading)

        self.get_logger().info(f"\n\033[1mCreating Splines Between Waypoints...\033[1m\n")
        # circular map end-start waypoints 
        next_pt_map[ptid(waypoints[-1])] = waypoints[0] 
        
        # memoize spline before dropout; assign opt mode
        self.optimizer_mode = self.OptMode.INITIAL_TRAINING
        for spline_index in range(1, len(waypoints)):
            p0 = waypoints[spline_index - 1]
            p1 = waypoints[spline_index]
            # find optimized splines 
            query_cost((p0,p1))
            next_pt_map[ptid(p0)] = p1
        
        self.get_logger().info(f"\n\033[1mSimplifying with Waypoint Dropout...\033[1m\n")
        # handle waypoint dropout
        self.optimizer_mode = self.OptMode.DROPOUT_TRAINING
        curr_pt = waypoints[0]
        while dropout_cnt < max_dropout:
            # retrieve the next two waypoints 
            next_pt = next_pt_map[ptid(curr_pt)]
            next_next_pt = next_pt_map[ptid(next_pt)]
            
            # finish dropout if spline is one path
            if next_next_pt == curr_pt: 
                break
           
            # retrieve costs 
            root_cost_0 = query_cost((curr_pt, next_pt))
            root_cost_1 = query_cost((next_pt, next_next_pt))
            # retrieve new dropout cost
            self.optimizer_mode = self.OptMode.DROPOUT_TRAINING
            dropout_cost = query_cost((curr_pt, next_next_pt))
            self.optimizer_mode = self.OptMode.INITIAL_TRAINING

            # handle new potential dropout 
            sum = root_cost_0 + root_cost_1
            if ((dropout_cost - sum) / sum <= dropout_err_pct and ptid(next_pt) != ptid(waypoints[-1]) 
                and ptid(next_pt) != ptid(waypoints[0])):
                next_pt_map[ptid(next_pt)] = None # pruned
                next_pt_map[ptid(curr_pt)] = next_next_pt # re-map waypoints 
                dropout_cnt += 1
                # view new spline path
                self.candidate_splines = np.vstack((self.candidate_splines, self.dropout_splines))
                self.set_rviz_optimizer_splines(splines=self.candidate_splines)

            # at end of waypoints; reset
            if next_next_pt == waypoints[-1]:
                if dropout_cnt == last_dropout_cnt:
                    break # checked and found no more dropouts; exit 
                last_dropout_cnt = dropout_cnt
                curr_pt = waypoints[0]
            else: # go to next point 
                curr_pt = next_pt_map[ptid(curr_pt)]
        
        self.get_logger().info(f"\n\033[1mDropout dropped {dropout_cnt} Waypoints.\033[1m\n")
        self.get_logger().info(f"\n\033[1mReconstructing Final Path...\033[1m\n")
        # reconstruct final path 
        self.optimizer_mode = self.OptMode.HEADING_TRAINING
        curr_pt = waypoints[0]
        build_path = True
        while build_path:
            # retrieve sub path data
            next_pt = next_pt_map[ptid(curr_pt)]
            id = (ptid(curr_pt), ptid(next_pt))
            # path data 
            data_x, data_y, sd = None, None, None

            # dropout done; breakout 
            if id not in path_data:
                break
            # final sub-path; optimize its end-heading
            if next_pt == waypoints[-1]:
                x_end = waypoints[-1].pose.position.x
                y_end = waypoints[-1].pose.position.y
                heading_end = handler.get_heading(waypoints[-1])
                
                self.get_logger().info(f"\n\033[1mTuning End-Trajectory Heading...\033[1m\n")
                # binary search for best end heading
                for epoch in range(0, end_heading_epochs):
                    bound_step = end_heading_bound / float(epoch + 1)
                    # create end points 
                    base_pt = handler.get_pose_stamped(x_end, y_end, heading_end)
                    cw_pt = handler.get_pose_stamped(x_end, y_end, heading_end + bound_step)
                    ccw_pt = handler.get_pose_stamped(x_end, y_end, heading_end - bound_step)
                    # compute costs for heading adjustment
                    base_cost = query_cost((curr_pt, base_pt))
                    cw_cost = query_cost((curr_pt, cw_pt))
                    ccw_cost =  query_cost((curr_pt, ccw_pt))

                    # adjust the base heading based on best cost
                    min_cost = min(base_cost, cw_cost, ccw_cost)
                    if base_cost == min_cost:
                        id = (ptid(curr_pt), ptid(base_pt))
                    elif cw_cost == min_cost:
                        id = (ptid(curr_pt), ptid(cw_pt))
                        heading_end += bound_step
                    elif ccw_cost == min_cost:
                        id = (ptid(curr_pt), ptid(ccw_pt))
                        heading_end -= bound_step
                # flag build is almost over
                build_path = False

            # pull path data from dropout passes
            data_x, data_y, sd = path_data[id]
            
            # remove last row for next waypoint to overlap
            if len(data_x) != 0:
                data_x.pop()
            if len(data_y) != 0:
                data_y.pop()
            if len(sd) != 0:
                sd.pop()

            # append sub-path
            path_x += data_x
            path_y += data_y
            sd_steps += sd
            # get next waypoint 
            curr_pt = next_pt_map[ptid(curr_pt)]
        
        # record and return results 
        self.path_x = path_x
        self.path_y = path_y
        self.sd_steps = sd_steps
        self.cumulative_sd_steps = np.cumsum(sd_steps).tolist()
        self.get_logger().info(f"\n\033[1mFinal Path Compelete. Returning Results...\033[1m")
        self.optimizer_mode = self.OptMode.IDLE # update opt mode
        return (path_x, path_y, sd_steps, self.cumulative_sd_steps)

    def set_rviz_optimizer_splines(self, splines: np.ndarray):
        """
        Visualizes the training process of the optimizer. 
        :param splines [np.ndarray] The specified array of candidate splines tested during training
        """
        # create instance
        msg = PointCloud2()
        msg.header = self.map_header
        msg.height = 1
        msg.width = len(splines)
        msg.is_bigendian = False
        msg.point_step = 16 # 4 floats x 4 bytes = 16
        msg.row_step = msg.point_step*len(splines)
        msg.is_dense = True

        msg.fields = [ # fields to define data types
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1), 
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="rgba", offset=12, datatype=PointField.UINT32, count=1)
        ]
        # pass cached candidates and publish
        msg.data = self.candidate_splines.tobytes()
        self.optimizer_training_pub_.publish(msg)

    def loop(self):
        ''' Main node loop. '''
        if self.path_x is not None and self.path_y is not None and RVIZ_PLOT_SPLINE_GRIDCELLS:
            self.plot_spline_gridcells(self.path_x, self.path_y)

def main(args=None):
    rclpy.init()
    node = SplinePath()
    try: 
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass 
    finally: 
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main(args=None)