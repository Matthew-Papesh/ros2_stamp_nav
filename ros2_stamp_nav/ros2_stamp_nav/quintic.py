#!/usr/bin/env python3
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from matplotlib.axes import Axes
from collections.abc import Callable
from geometry_msgs.msg import PoseStamped
import ros2_stamp_nav.handler as handler

import math
import numpy as np
import csv

# Represents the relative quintic function to compute sub-points from 
class Quintic:
    """
    Represents the relative quintic function to compute given a specified starting and ending waypoint. 
    """    
    def __init__(self, waypoint_0: PoseStamped | tuple[float, float, float], waypoint_1: PoseStamped | tuple[float, float, float]):
        """
        Initializes the Quintic with the necessary coefficients based on the specified waypoints and headings.
        :param waypoint_0 [PoseStamped | tuple[float, float, float]] The specified initial waypoint 
        :param waypoint_1 [PoseStamped | tuple[float, float, float]] The specified final waypoint
        """
        # the specified waypoints
        self.waypoint_0 = waypoint_0
        self.waypoint_1 = waypoint_1
        self.obstacles = np.empty((0,2), dtype=np.float64)
        self.obs_radius = 1.0
        self.max_inv = 3.0
        # spline-space transformation before evaluating against obstacles
        self.spline_to_obs_tf = (0, 0, 0)
        # Function coefficients
        self.k_0, self.k_1 = 0, 0
        # Queried cost map 
        self.cost_query = {}

        self.coeffs_x = None
        self.coeffs_y = None
        self.set_inverse_transform(0, 0, 0)


    def set_curvature(self, k_0: float, k_1: float): 
        """
        Defines the quintic model curvature. `compute_quintic()` must be called after setting these parameters.
        :param k_0 [float] The specified first coefficient 
        :param k_1 [float] The specified second coefficient
        """
        self.k_0 = k_0
        self.k_1 = k_1

    def set_obstacles(self, obstacles: set[tuple[float, float]] | np.ndarray, radius: float):
        """
        Defines the set of 2D obstacles for the quintic model to avoid. Each obstacle is a tuple singularity. 
        Each obstacle has the same set radius from its center. 
        :param obstacles [] The specified set of obstacles 
        :param radius [float] The specified radius
        """
        if isinstance(obstacles, set):
            self.obstacles = np.array(list(obstacles), dtype=np.float64)
        elif isinstance(obstacles, np.ndarray):
            self.obstacles = obstacles
        self.obs_radius = radius

    def set_inverse_transform(self, x_origin: float, y_origin: float, theta_origin: float):
        """
        Defines a transformation between the quintic function frame of reference to the obstacle map's frame of reference. 
        :param x_origin [float] Origin x for the global obstacle map 
        :param y_origin [float] Origin y for the global obstacle map 
        :param theta_origin [float] Origin orientation in radians for the global obstacle map
        """
        self.spline_to_obs_tf = (x_origin, y_origin, theta_origin)

    # Determines the quintic function coefficients such that the quintic tangentially instersects the specified waypoints
    def compute_quintic(self):
        """
        Computes the coefficients of a quintic polynomial into 100 samples of variable `t`.
        """
        # handle initializing waypoint 
        def init_waypoint(waypoint: PoseStamped | tuple[float, float, float]):
            if isinstance(waypoint, PoseStamped):
                x = waypoint.pose.position.x
                y = waypoint.pose.position.y
                heading = handler.get_heading(waypoint)
                return x, y, heading
            elif isinstance(waypoint, tuple):
                return waypoint[0], waypoint[1], waypoint[2]

        # initialize waypoints and k-space constants 
        x_0, y_0, t_0 = init_waypoint(self.waypoint_0)
        x_1, y_1, t_1 = init_waypoint(self.waypoint_1)
        k_0, k_1 = self.k_0, self.k_1
    
        # define euclid distance between waypoints 
        L = math.sqrt((x_1 - x_0)**2.0 + (y_1 - y_0)**2.0)
        if L < 1e-5:
            L = 1.0

        # geometric first derivatives 
        dx_0, dy_0 = L*math.cos(t_0), L*math.sin(t_0)
        dx_1, dy_1 = L*math.cos(t_1), L*math.sin(t_1)

        # geomtric second derivatives 
        ddx_0 = -k_0 * (L**2.0) * math.sin(t_0)
        ddy_0 =  k_0 * (L**2.0) * math.cos(t_0)
        ddx_1 = -k_1 * (L**2.0) * math.sin(t_1)
        ddy_1 =  k_1 * (L**2.0) * math.cos(t_1)

        bx = np.array([x_0, dx_0, ddx_0, x_1, dx_1, ddx_1])
        by = np.array([y_0, dy_0, ddy_0, y_1, dy_1, ddy_1])
    
        M_inv = np.array([
            [ 1,  0,   0,    0,   0,   0],  # Coeff f (t^0)
            [ 0,  1,   0,    0,   0,   0],  # Coeff e (t^1)
            [ 0,  0, 0.5,    0,   0,   0],  # Coeff d (t^2)
            [-10, -6, -1.5,  10,  -4, 0.5],  # Coeff c (t^3)
            [ 15,  8,  1.5, -15,   7,  -1],  # Coeff b (t^4)
            [-6,  -3, -0.5,   6,  -3, 0.5]   # Coeff a (t^5)
        ])

        self.coeffs_x = M_inv @ bx
        self.coeffs_y = M_inv @ by
     
    # Represents the quintic function dependent on the domain of `t` bounded [0.0, 1.0]
    def f(self, t: float | np.ndarray) -> tuple[float | np.ndarray, float | np.ndarray]:
        """
        Represents the quintic function dependent on the domain of `t` bounded by [0.0, 1.0].
        """
        # evaluation: a*t^5 + b*t^4 + c*t^3 + d*t^2 + e*t + f
        x = ((((self.coeffs_x[5]*t + self.coeffs_x[4])*t + self.coeffs_x[3])*t + self.coeffs_x[2])*t + self.coeffs_x[1])*t + self.coeffs_x[0]
        y = ((((self.coeffs_y[5]*t + self.coeffs_y[4])*t + self.coeffs_y[3])*t + self.coeffs_y[2])*t + self.coeffs_y[1])*t + self.coeffs_y[0]
        return x, y
    # Represents the derivative of the quintic function
    def dydx(self, t: float | np.ndarray) -> tuple[float | np.ndarray, float | np.ndarray]:
        """
        Represents the derivative of the quintic function on the domain of `t` bounded by [0.0, 1.0].
        """
        # evaluation: 5a*t^4 + 4b*t^3 + 3c*t^2 + 2d*t + e
        dx = (((5*self.coeffs_x[5]*t + 4*self.coeffs_x[4])*t + 3*self.coeffs_x[3])*t + 2*self.coeffs_x[2])*t + self.coeffs_x[1]
        dy = (((5*self.coeffs_y[5]*t + 4*self.coeffs_y[4])*t + 3*self.coeffs_y[3])*t + 2*self.coeffs_y[2])*t + self.coeffs_y[1]
        return dx, dy
    # Represents the second derivative of the quintic function
    def ddydxx(self, t: float | np.ndarray):
        """
        Represents the second derivative of the quintic function on the domain of `t` bounded [0.0, 1.0]
        """
        # evaluation: 20a*t^3 + 12b*t^2 + 6c*t + 2d
        dx = ((20*self.coeffs_x[5]*t + 12*self.coeffs_x[4])*t + 6*self.coeffs_x[3])*t + 2*self.coeffs_x[2]
        dy = ((20*self.coeffs_y[5]*t + 12*self.coeffs_y[4])*t + 6*self.coeffs_y[3])*t + 2*self.coeffs_y[2]
        return dx, dy

    def cost(self, spline_partitions: int=100) -> tuple[int, np.ndarray, np.ndarray]:
        """
        Represents the length of the quintic curve between the specified x0 and x1 values.
        :param x0 [float] The initial x value to compute the length from
        :param x1 [float] The final x value to compute the length to
        :param steps [int] The number of steps to use in approximating the length (higher is more accurate but more computationally expensive)
        :returns The cost of the candidate spline and global coordinate of generated candidate spline
        """
        # the distance array allows for this activation function allows for evaluating against several vectorized obstacles at once
        def step(distance: np.ndarray, radius: float) -> np.ndarray:
            """
            Defines a continuous sigmoid step function that collapses for distances larger than the specified radius. 
            :param distance [np.ndarray] A specified distance away from a given obstacle to evaluate 
            :param radius [float] The radius the sigmoid output collapses for distances that exceed the threshold
            :returns The sigmoid step output
            """
            exponent = np.clip(100.0 * (distance - radius), -500, 500)
            return 1.0 / (1.0 + np.exp(exponent))
        # the distance array for this function allows for evaluating against several vectorized obstacles at once 
        def inv_distance(distance: np.ndarray, max_inv: float) -> np.ndarray:
            """
            Defines an inverse distance relationship for a given distance. This represents an inversely proportional relationship such that the maximum output 
            specified cannot be exceeded at a distance=0. This is not a direct inverse or reciprocal of the specified distance, but is phased by the max inverse provided.  
            :param distance [np.ndarray] The specified distance away from a given obstical to compute inverse for
            :param max_inv [float] The maximum, non-negative, inverse that can be returned; this phase-offsets the x-domain to avoid divide-by-zero exceptions 
            :returns The phase-offset inverse result 
            """
            return max_inv / (max_inv*distance + 1.0)

        def obs_cost(x: np.ndarray, y: np.ndarray) -> np.ndarray:
            """
            Defines the cost map component for avoiding obstacles (obs). This function is called iteratively 
            while stepping along the arc of the spline and summed as a total obstacle cost.   
            :param x [np.ndarray] The specified instantaneous x point along the spline arc
            :param y [np.ndarray] The specified instantaneous y point along the spline arc 
            :returns The instantaneous obstacle sub-cost
            """
            # casts all inputs to np array for N rows of (x,y) points
            x_arr = np.atleast_1d(x) # arrays of Nx1 shape
            y_arr = np.atleast_1d(y)

            # retrieve coords for M obstacles: Wx and Wy are Mx1 shape
            Wx = self.obstacles[:, 0][:, np.newaxis] 
            Wy = self.obstacles[:, 1][:, np.newaxis]

            # calculate distances for each N of the (x,y) points 
            dx = Wx - x_arr # shape of MxN
            dy = Wy - y_arr
            # create distance matrix shape MxN
            dist = np.sqrt(dx*dx + dy*dy) 

            # calculates sub costs against each obstacle across rows; each column is a point (x,y) shape MxN
            sub_costs = step(dist, self.obs_radius) * inv_distance(dist, self.max_inv)
            sub_costs = np.cumsum(sub_costs, axis=1)
            point_costs = np.sum(sub_costs, axis=0) # vector of obs costs across spline 
            return point_costs.T

        # interpolate parametric quintic 
        t_data = np.linspace(0, 1, spline_partitions)
        x_arr, y_arr = self.f(t_data)
        dtdx, dtdy = self.dydx(t_data)
        d2tdx2, d2dy2 = self.ddydxx(t_data) 

        # transform coord (x,y) to global obstacle map: Nx1 shape for both outputs
        tf_x, tf_y = handler.rotate(x_arr, y_arr, self.spline_to_obs_tf[2])
        tf_x += self.spline_to_obs_tf[0]
        tf_y += self.spline_to_obs_tf[1]
        # cast to np array
        tf_x = np.atleast_1d(tf_x)
        tf_y = np.atleast_1d(tf_y)

        C = 0.00001 # tuning coefficient 
        # find interpolated sub-costs 
        length_cost = C*(dtdx**2+dtdy**2)
        smooth_cost = C*(d2tdx2**2+d2dy2**2)
        obstacle_cost = obs_cost(tf_x, tf_y)
        # calculate costs 
        sub_costs = length_cost + smooth_cost + obstacle_cost
        # sum point costs for total cost
        return sub_costs.sum(axis=0), tf_x, tf_y
    
    def query_cost_map(self, axis_partitions: int, k0_range: tuple[float, float], k1_range: tuple[float, float], export_file: str=None) -> dict:
        """
        Calculates the cost map surface of this quintic spline. This surface maps the (k0,k1) surface (k-space) to its cost. 
        This is an intensive process. Data can be exported to a CSV file. 
        :param axis_partitions [int] The step size to discretize the cost map surface
        :param k0_range [tuple[float, float]] Specifies a tuple for k0 min max range. 
        :param k1_range [tuple[float, float]] Specified a tuple for k1 min max range. 
        :param export_file [str] Optionally specifies a CSV file to export the cost map surface to. 
        :returns A the cost map dictionary such that (k0,k1) tuple is the key to corresponding cost 
        """
        kd0 = (k0_range[1]-k0_range[0]) / float(axis_partitions)
        kd1 = (k1_range[1]-k1_range[0]) / float(axis_partitions)
        curr_k0, curr_k1 = self.k_0, self.k_1
        query = {}
        index = 0
        progress = 0

        file_writer = None
        file = None
        if export_file is not None:
            open(export_file, mode='w', newline='').close()
            file = open(export_file, mode='w', newline='')
            file_writer = csv.writer(file)

        for k0idx in range(0, axis_partitions):
            k0 = round(k0_range[0] + k0idx*kd0, 3)
            for k1idx in range(0, axis_partitions):
                k1 = round(k1_range[0] + k1idx*kd1, 3)
                self.set_curvature(k0, k1)
                self.compute_quintic()
                cost, _, _ = self.cost()
                query[(k0,k1)] = cost

                if export_file is not None:
                    file_writer.writerow([k0, k1, cost])
                
                next_progress = int(100.0*(index+1)/(axis_partitions**2))
                if next_progress != progress:
                    progress = next_progress
                    print(f"querying cost map; prog: {progress}%")
                index += 1

        if file is not None:
            file.close()

        self.set_curvature(curr_k0, curr_k1)
        self.compute_quintic()
        self.cost_query = query
        return query

    def import_query_cost_map(self, import_file: str):
        """
        Memoizes (caches) a CSV file for visualizing the cost map surface via `self.show_cost_map()`. 
        CSV records take the form of: <k0>, <k1>, <cost>. 
        Visualizing is an intensive process; data can be saved to a CSV and imported later. 
        :param import_file [str] The specified file location to import. 
        """
        with open(import_file, mode='r', newline='') as file:
            self.cost_query.clear()
            file_reader = csv.reader(file)

            for record in file_reader:
                if not record:
                    continue

                print(record)
                k0 = float(record[0])
                k1 = float(record[1])
                cost = float(record[2])
                self.cost_query[(k0,k1)] = cost

    def get_cost_map(self) -> dict:
        return self.cost_query

    def show_cost_map(self, show_cost_trend: bool=False): 
        """
        Visualizes the cost map surface from the (k0,k1) plane (k-space) while projecting the cost orthogonally. This is done with matplotlib. 
        :param show_cost_trend [bool] Toggles if the global cost trend plane is visualized. 
        """
        if self.cost_query is None or not self.cost_query:
            return 
        k_keys = np.array(list(self.cost_query.keys()))
        cost_vals = np.array(list(self.cost_query.values()))
        k0_vals = k_keys[:, 0]
        k1_vals = k_keys[:, 1]

        fig = plt.figure(figsize=(10,7))
        ax = fig.add_subplot(projection='3d')
        scatter=None

        if show_cost_trend:
            beta, k0_min_path, k1_min_path, min_cost = self.get_cost_trend_surface()
            print(f"Cost trend surface coefficients: {beta}, min path: ({k0_min_path}, {k1_min_path}), cost: {min_cost}")   
            # define a simple 2x2 grid representing k-space corners 
            k0_min, k0_max = k0_vals.min(), k0_vals.max()
            k1_min, k1_max = k1_vals.min(), k1_vals.max()
            
            # create a minimal meshgrid containing only 4 points total
            K0_grid, K1_grid = np.meshgrid([k0_min, k0_max], [k1_min, k1_max])
            # evaluate plane across 4 corners
            Z_trend = beta[0] + beta[1]*K0_grid + beta[2]*K1_grid

            # render as a lightweight surface instead of a massive trisurf mesh
            ax.plot_surface(K0_grid, K1_grid, Z_trend, color='red', alpha=0.4, rstride=1, cstride=1, zorder=1)
            ax.scatter(k0_min_path, k1_min_path, min_cost, color='black', s=100, label='Min Cost Path', zorder=2)
            
        # plot points
        scatter = ax.scatter(k0_vals, k1_vals, cost_vals, c=cost_vals, cmap='viridis', s=30, depthshade=True)
        ax.set_xlabel('k0 Axis')
        ax.set_ylabel('k1 Axis')
        ax.set_zlabel('Path Cost')
        ax.set_title('Quintic Path Cost Map')

        fig.colorbar(scatter, ax=ax, label='Cost Scale', pad=0.1)
        plt.show()

    def get_cost_trend_surface(self, in_ring_radius: float=3.0, out_ring_radius: float=7.0, origin_radius: float=2.0, n_ring:int=50, n_origin: int=5):
        """
        Samples global geometry of the cost map surface associated with this quintic spline. Retrieves a global trending slope and the lowest point (k0,k1,cost) discovered.
        This slope is the coefficients of a planar regression of the sampled points.  
        Sampling happens at the (k0,k1)=(0,0) origin, and at a inner and outer radius specified. 
        :param in_ring_radius [float] The outer radius to sample ring points 
        :param out_ring_radius [float] The inner radius to sample ring points
        :param origin_radius [float] The max radius to sample origin points within
        :param n_ring [int] Sample size between inner and outer rings
        :param n_origin [int] Origin sample size
        :returns A 4D tuple of a regression plane coefficients, the k0, k1, and cost found at the lowest cost sampled. 
        """
        rng = np.random.default_rng() # init rng
        # deterministic macro-horizon arrays avoids 0 and 2pi duplication
        thetas = np.linspace(0.0, 2.0*math.pi, int(n_ring/2), endpoint=False)
        
        # inner ring
        k0_inner = in_ring_radius*np.cos(thetas)
        k1_inner = in_ring_radius*np.sin(thetas)
        # outer ring
        k0_out = out_ring_radius*np.cos(thetas)
        k1_out = out_ring_radius*np.sin(thetas)
        # origin sampling
        k0_origin = rng.normal(0, origin_radius / 2.0, size=n_origin)
        k1_origin = rng.normal(0, origin_radius / 2.0, size=n_origin)
        
        # concatenate everything
        k0s = np.concatenate([k0_inner, k0_out, k0_origin])
        k1s = np.concatenate([k1_inner, k1_out, k1_origin])
        # sample k-space costs 
        costs = np.empty(k0s.shape[0], dtype=np.float64)
        min_path = (0,0,None)

        for i in range(0, len(costs)):
            # set sample point and find cost
            self.set_curvature(k0s[i], k1s[i])
            self.compute_quintic()
            costs[i], _, _ = self.cost()
            # track best path sampled
            if min_path[2] is None or costs[i] < min_path[2]:
                min_path = (k0s[i], k1s[i], costs[i])
               
        # filter out upper outliers and noisy ridges/walls/mountain-peaks
        q1, q3 = np.percentile(costs, [25, 75])
        iqr = q3 - q1
        upper_bound = q3 + 1.5*iqr
        outlier_mask = costs > upper_bound
        # apply filter mask
        k0s = k0s[~outlier_mask]
        k1s = k1s[~outlier_mask]
        costs = costs[~outlier_mask]

        # find best found path samples 
        cost_threshold = np.percentile(costs, 15)
        best_samples_mask = costs <= cost_threshold
        best_k0s = k0s[best_samples_mask]
        best_k1s = k1s[best_samples_mask]
        best_costs = costs[best_samples_mask]

        # best educated guess for where to find true minimum cost 
        centroid_k0 = np.mean(best_k0s)
        centroid_k1 = np.mean(best_k1s)
        centroid_cost = np.mean(best_costs)

        # formalize samples into matrix
        ones = np.ones_like(k0s)
        X_matrix = np.column_stack((ones, k0s, k1s))
        # solve for coffecients
        beta, _, _, _ = np.linalg.lstsq(X_matrix, costs, rcond=None)

        # return results: surface constraints, and best guess for global minimum cost (k0, k1, and cost)
        return beta, centroid_k0, centroid_k1, centroid_cost

    def run_optimization(self, callback_event: Callable[[int, np.ndarray, np.ndarray], None] | None, epochs: int, k0: float=0.0, k1: float=0.0, T0: float=100.0, step_size: float=3.0, push_sensitivity: float=1.5, max_push_scaler: float=5.0, print_progress: bool=False) -> tuple[float, float, float]:
        """
        Trains quintic function's curvature parameters (k0, k1) with simulated annealing.
        :param callback_event [Callable[[int], None] | None] An optional callback function that takes in the current training epoch and global candidate spline coordinates (can be used for visualization or logging purposes)
        :param epochs [int] The number of optimization epochs to run
        :param k0 [float] The initial k0 curvature parameter to start optimization from
        :param k1 [float] The initial k1 curvature parameter to start optimization from
        :param T0 [float] The initial temperature parameter for simulated annealing 
        :param step_size [float] The initial step size for generating new k0 and k1 parameters 
        :param print_progress [bool] Whether to print the optimization progress at each epoch
        :return [tuple] The final optimized (k0, k1) curvature parameters and the corresponding cost value as a tuple (k0, k1, cost)
        """
        # init quintic function
        self.set_curvature(k0, k1)
        self.compute_quintic()
        curr_cost, _, _ = self.cost()
        T = T0 # init temperature
        rng = np.random.default_rng() # init rng
        beta, guess_min_k0, guess_min_k1, _ = self.get_cost_trend_surface() # precompute cost trend 

        dtdk0, dtdk1 = beta[1], beta[2] # extract cost trend coefficients
        push_scaler = np.sqrt(guess_min_k0**2.0 + guess_min_k1**2.0) 
        push_scaler = max_push_scaler / (1.0 + np.exp(10.0 - push_sensitivity*push_scaler)) 

        # begin training loop
        for epoch in range(0, epochs):
            # find next k params 
            curr_step_size = step_size/(1+0.00005*epoch) 
            alpha = push_scaler / ((1+0.00005*epoch)**2.0 + push_scaler)# converges epoch:0->inf. => alpha:1->0
            dk0, dk1 = rng.normal(0, curr_step_size, size=2)
            next_k0 = k0 + (1.0-alpha)*dk0 - alpha*dtdk0
            next_k1 = k1 + (1.0-alpha)*dk1 - alpha*dtdk1

            # compute next cost
            self.set_curvature(next_k0, next_k1)
            self.compute_quintic()
            next_cost, x_arr, y_arr = self.cost() # cost and candidate spline in global coords
            dcost = next_cost - curr_cost

            # update k params if new cost is lesser 
            if next_cost < curr_cost: 
                k0 = next_k0
                k1 = next_k1
                curr_cost = next_cost
            else: # larger new cost; deviate from minimizing and accept 
                # this new k params based on model below
                deviate_pct = np.exp((curr_cost-next_cost)/T)
                should_deviate = rng.random() < deviate_pct
                if should_deviate:
                    k0 = next_k0
                    k1 = next_k1
                    curr_cost = next_cost

            dcost = 0 if curr_cost != next_cost else dcost
            # update temperature T
            T = T0/(1+epoch)
            # print progress state 
            if print_progress:
                print(f"spline opt progress: {int(100.0 * (epoch+1)/epochs)}% => [ step_size={curr_step_size:.3f}, T={T:.3f},    cost={round(curr_cost, 5)},   (k0,k1)=({k0:.3f}, {k1:.3f}),   push_scaler={round(push_scaler, 5)} ]")
            # handle any callback 
            if callback_event is not None: # epoch and current spline
                callback_event(epoch, x_arr, y_arr)

        # final k param update and return results
        self.set_curvature(k0, k1)
        self.compute_quintic()
        return (k0, k1, curr_cost)

    def optimize_and_show(self, epochs: int, k0: float=0.0, k1: float=0.0, T0: float=100.0, step_size: float=3.0, push_sensitivity: float=1.5, max_push_scaler: float=5.0, x_lim: tuple[float, float]=(-1,6), y_lim: tuple[float, float]=(-1,6), print_progress: bool=False) -> tuple[float, float, float]:
        """
        Optimizes the quintic function's curvature parameters (k0, k1) using simulated annealing and visualizes the optimization process using matplotlib.
        :param epochs [int] The number of optimization epochs to run
        :param k0 [float] The initial k0 curvature parameter to start optimization from
        :param k1 [float] The initial k1 curvature parameter to start optimization from
        :param T0 [float] The initial temperature parameter for simulated annealing 
        :param step_size [float] The initial step size for generating new k0 and k1 parameters 
        :param push_scaler [float] The push scaler parameter for controlling the influence of the cost trend
        :return [tuple] The final optimized (k0, k1) curvature parameters and the corresponding cost value as a tuple (k0, k1, cost)
        """
        def draw(fig: Figure, ax: Axes, draw_obstacles: bool, color: any, linewidth: float=1.5, alpha: float=0.15):
            """
            Draws the current quintic curve on the given matplotlib figure and axes, along with the obstacles if specified.
            :param fig [Figure] The matplotlib figure to draw on
            :param ax [Axes] The matplotlib axes to draw on
            :param draw_obstacles [bool] Whether to draw the obstacles on the plot
            :param color [any] The color to draw the quintic curve with
            :param linewidth [float] The width of the quintic curve line
            :param alpha [float] The alpha value for drawing
            """
            ax.set_xlim(x_lim[0], x_lim[1]) # set graph limits 
            ax.set_ylim(y_lim[0], y_lim[1])
            # setup for drawing 
            partitions = 100

            # draw obstacles 
            if self.obstacles.size > 0 and draw_obstacles:
                for obs in self.obstacles:
                    obs_x = obs[0]
                    obs_y = obs[1]
                    ax.scatter(obs_x, obs_y, color='black', s=50, zorder=5)
                    safety_circle = plt.Circle((obs_x, obs_y), self.obs_radius, color='maroon', alpha=alpha, zorder=4)
                    ax.add_patch(safety_circle)

            # draw quintic curve
            t_data = np.linspace(0, 1, partitions)
            x_data, y_data = self.f(t_data)
            # transform coord (x,y) to global obstacle map 
            tf_x, tf_y = handler.rotate(x_data, y_data, self.spline_to_obs_tf[2])
            tf_x += self.spline_to_obs_tf[0]
            tf_y += self.spline_to_obs_tf[1]

            ax.plot(tf_x, tf_y, color=color, alpha=0.08, linewidth=linewidth, zorder=2)
            ax.set_title(f"Spline Simulated Annealing: (k0,k1)={(round(self.k_0, 3), round(self.k_1, 3))}")
            fig.canvas.draw()
            fig.canvas.flush_events()

        # matplotlib handling 
        plt.ion()
        fig2d, ax2d = plt.subplots(num=2, figsize=(10,5))
        cmap = plt.get_cmap('plasma')

        # drawing handler for optimization loop
        def handle_draw(epoch: int, x_arr: np.ndarray, y_arr: np.ndarray):
            draw(fig=fig2d, ax=ax2d, draw_obstacles=epoch==0, color=cmap(epoch/epochs), linewidth=1.5, alpha=0.8)
        # run optimization loop with drawing callback
        final_k0, final_k1, final_cost = self.run_optimization(
            callback_event=handle_draw, 
            epochs=epochs, 
            k0=k0, 
            k1=k1, 
            T0=T0, 
            step_size=step_size, 
            push_sensitivity=push_sensitivity, 
            max_push_scaler=max_push_scaler,
            print_progress=print_progress
        )
        # final drawing loop to show final curve with higher opacity
        for i in range(0,10):
            draw(fig=fig2d, ax=ax2d, draw_obstacles=False, color='black', linewidth=3, alpha=1)
        # print final results and show plot
        print(f"Done:100% => [ cost={final_cost},   (k0,k1)=({final_k0:.3f}, {final_k1:.3f}) ]")    
        plt.ioff()
        plt.show()
        return (final_k0, final_k1, final_cost)

    def optimize(self, epochs: int, k0: float=0.0, k1: float=0.0, T0: float=100.0, step_size: float=3.0, push_sensitivity: float=1.5, max_push_scaler: float=5.0, callback_event: Callable[[int, np.ndarray, np.ndarray], None]=None) -> tuple[float, float, float]:
        """
        Optimizes the quintic function's curvature parameters (k0, k1) using simulated annealing without visualization.
        :param epochs [int] The number of optimization epochs to run
        :param k0 [float] The initial k0 curvature parameter to start optimization from
        :param k1 [float] The initial k1 curvature parameter to start optimization from
        :param T0 [float] The initial temperature parameter for simulated annealing 
        :param step_size [float] The initial step size for generating new k0 and k1 parameters 
        :return [tuple] The final optimized (k0, k1) curvature parameters and the corresponding cost value as a tuple (k0, k1, cost)
        """
        # run optimization loop without drawing callback
        final_k0, final_k1, final_cost = self.run_optimization(
            callback_event=callback_event, 
            epochs=epochs, 
            k0=k0, 
            k1=k1, 
            T0=T0, 
            step_size=step_size, 
            push_sensitivity=push_sensitivity, 
            max_push_scaler=max_push_scaler,
            print_progress=False
        )
        return (final_k0, final_k1, final_cost)
            
