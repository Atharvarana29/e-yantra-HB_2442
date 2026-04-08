#!/usr/bin/env python3

'''
* Team Id: HB_2442
* Author List: Kumar Sushant Raj, Atharva Rana , Aditya Kumar
* Filename: MultiHolonomicController.py
* Theme: eYRC Holo Battalion
* Functions: __init__, ir_cb(), get_crate_color_and_zone(), task_complete_cb(),
             get_optimal_approach_pose(), allocate_tasks(), pose_cb(), crate_pose_cb(),
             task_cb(), control_cb(), replan(), navigate_to_crate(), pick_crate(),
             transport_crate(), place_crate(), return_to_dock(), calculate_repulsion(),
             navigate_to_crate_direct(), compute_path_velocity(), navigate_to_goal(),
             body_to_wheel_velocities(), normalize_angle(), stop_robot(), move_arm(),
             set_gripper(), publish_wheel_velocities(), main()
* Global Variables: NONE
'''

# ---------------------- Import Required Libraries ----------------------------
import rclpy
from rclpy.node import Node
from hb_interfaces.msg import BotCmd, BotCmdArray, Poses2D
from std_msgs.msg import String, Bool
from std_srvs.srv import SetBool
import numpy as np
import math
import time
from scipy.optimize import linear_sum_assignment
from scipy.ndimage import distance_transform_edt  
from heapq import heappush, heappop
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

# ---------------------- Drop Zone Manager Class -----------------------------
class DropZoneManager:
    """
    * Function Name: __init__
    * Input: zones (dict), slot_rows (int), slot_cols (int), margin (int)
    * Output: None
    * Logic: Initializes the zone boundaries and generates coordinate slots for stacking.
    * Example Call: DropZoneManager(bounds)
    """

    """
    Function: __init__
    Input:
        zones (dict) :
            Dictionary defining rectangular boundaries for each drop zone.
            Format:
            {
                "ZoneA": (x_min, y_min, x_max, y_max),
                "ZoneB": (x_min, y_min, x_max, y_max)
            }

        slot_rows (int) :Number of rows used to divide a zone vertically into grid slots.

        slot_cols (int) :Number of columns used to divide a zone horizontally into grid slots.

        margin (int) :Offset from the zone boundary to ensure crates are not placed
                      too close to edges, improving placement safety and tolerance.

    Output: None

    Logic Explanation:

        1. Store zone boundaries and grid configuration parameters.
           These values determine how slot coordinates are computed.

        2. Initialize data structures for managing slots and occupancy.
           Precomputed slots reduce runtime computation and ensure
           consistent placement behavior.

        3. Initialize per-zone layer tracking to support vertical stacking
           without mixing height states between zones.

        4. Maintain a crate counter per zone to assist with capacity checks
           and placement decisions.

        5. Invoke _generate_slots() to compute evenly spaced slot coordinates
           within each zone based on rows, columns, and margin.

    Example call:
        bounds = {
            "ZoneA": (100, 100, 500, 500),
            "ZoneB": (600, 100, 1000, 500)
        }

        manager = DropZoneManager(bounds)
    """

    
    def __init__(self, zones, slot_rows=2, slot_cols=2, margin=120):
        self.zones = zones
        self.slot_rows = slot_rows
        self.slot_cols = slot_cols
        self.margin = margin
        self.zone_slots = {}
        self.zone_occupancy = {}

        # Track current height level
        self.zone_layers = {zone: 0 for zone in zones} 
        self.crates_in_zone = {zone: 0 for zone in zones} # Counter

        self._generate_slots()


    """
    ------------------------------------------------------------
    Function: _generate_slots

    Input:
        None

    Output:
        None

    Logic Explanation:

        1. Set fixed horizontal and vertical step sizes to control
           spacing between slots.

        2. For each zone, iterate within its boundary limits and
           generate (x, y) positions using incremental steps.

        3. Ensure generated slots remain inside valid placement
           bounds.

        4. Store computed slots and initialize their occupancy
           status as empty.

        5. Print debug information for verification.

    Example:
        self._generate_slots()
    """

    def _generate_slots(self):
        step_x = 70  # increment for slot width
        # similarly doing for y since we are using rows also
        step_y = 70

        for zone, bounds in self.zones.items():
            xmin, xmax, ymin, ymax = bounds
            slots = []

            # Start at xmin + step_x and go until xmax
            # We use a while loop to handle fixed increments
            curr_y = ymin + step_y
            while curr_y < ymax:
                curr_x = xmin  + 100
                while curr_x < xmax-20:
                    slots.append((curr_x, curr_y))
                    curr_x += step_x
                curr_y += step_y

            self.zone_slots[zone] = slots
            self.zone_occupancy[zone] = [False] * len(slots)
            print(f"DEBUG: Generated {len(slots)} fixed-step slots for {zone}")


    """
    ------------------------------------------------------------
    Function: get_next_pyramid_slot

    Input:
        zone (str) :
            Name of the drop zone.

    Output:
        ((x, y), layer) :
            Coordinates where the crate should be placed and
            the corresponding pyramid layer.
            Layer 0 represents the base layer.

    Logic Explanation:

        1. Validate that the zone exists. If not, return None.

        2. Determine how many crates are already placed in the zone.
           This count defines the next pyramid position.

        3. Compute horizontal spacing from base slots to maintain
           proper alignment between layers.

        4. Iterate through pyramid levels diagonally (layer + index
           combinations) to map the crate count to a valid
           (slot_index, layer) pair.

        5. Adjust X position by half-step per layer to create
           the pyramid shift while keeping Y constant.

        6. Update occupancy and return the computed position
           along with the layer index.

        7. If no valid position exists, report that the zone
           is full.

    Example:
        self.get_next_pyramid_slot("D1")
    """

    def get_next_pyramid_slot(self, zone):
        if zone not in self.zone_occupancy:
            return None, 0

        # 1. Get current count and base configuration
        count = sum(self.zone_occupancy[zone])
        base_slots = self.zone_slots[zone]

        # Need at least 2 slots to calculate a step size
        if len(base_slots) < 2:
            return (base_slots[0], 0) if count == 0 else (None, 0)

        # Calculate the horizontal distance between two base slots
        step_x = base_slots[1][0] - base_slots[0][0]

        # 2. Iterate to find which 'Pyramid Coordinate' (layer, index) corresponds to our 'count'
        current_counter = 0

        # 's' is the sum of (layer + index). This iterates the "diagonals" of the pyramid.
        # Max iterations limited by base_slots length to prevent infinite loops
        for s in range(len(base_slots)):
            # In every diagonal 's', layer goes from 0 up to s
            for layer in range(s + 1):
                slot_index = s - layer

                # Check bounds: ensure we don't go wider than our base slots allow
                if slot_index >= len(base_slots) - layer:
                    continue

                if current_counter == count:
                    # 3. Found our match! Calculate real world coordinates.
                    # Start at the Base Slot for this index
                    anchor_x, anchor_y = base_slots[slot_index]

                    # Shift X right by half a step for each layer up
                    final_x = anchor_x + (layer * (step_x / 2.0))
                    final_y = anchor_y 

                    # Mark occupancy 
                    if count < len(self.zone_occupancy[zone]):
                        self.zone_occupancy[zone][count] = True

                    return (final_x, final_y), layer

                current_counter += 1

        print(f"Zone {zone} is full or Logic Error!")
        return None, 0


    """
    * Function Name: release_slot
    * Input: zone (string), index (int)
    * Output: None
    * Logic: Marks a specific slot in a zone as False (vacant).
    * Example Call: self.release_slot('D1', 0)
    """
    def release_slot(self, zone, index):
        if zone in self.zone_occupancy and index < len(self.zone_occupancy[zone]):
            self.zone_occupancy[zone][index] = False


# ---------------------- Theta* Planner Class -------------------------------------

class ThetaStarPlanner:
    """
    Theta* path planner (any-angle A*) with Corridor Centering (Distance Transform)
    """
    def __init__(self, resolution=25.0):
        self.res = resolution
        self.grid_dim = 120 
        self.cost_map = np.zeros((self.grid_dim, self.grid_dim))

        # 8-connected grid
        self.neighbors = [
            (-1,  0), (1,  0), (0, -1), (0,  1),
            (-1, -1), (-1, 1), (1, -1), (1, 1)
        ]


    """
    ------------------------------------------------------------
    Function: update_cost_map

    Input:
        obstacles (list of tuples) :
            List of obstacle coordinates in grid space.

    Output:
        None

    Logic Explanation:

        1. Initialize a grid where all cells are considered free space.

        2. Mark obstacle cells with zero value to indicate blocked regions.

        3. Apply Euclidean Distance Transform (EDT) to compute
           the distance of every cell from the nearest obstacle.

        4. Define a maximum penalty distance to limit the influence
           radius of obstacles.

        5. Convert distances into a cost field where cells closer
           to obstacles receive higher penalties, and cells beyond
           the threshold receive zero penalty.

        6. Store the resulting bounded heat map in self.cost_map
           for use in path planning or navigation logic.
    """

    def update_cost_map(self, obstacles):
        """
        Creates a 'Heat Map' using a bounded distance. 
        Cells further than 8 units (200mm) from an obstacle get 0 penalty.
        Cells right next to an obstacle get a maximum penalty.
        """
        grid = np.ones((self.grid_dim, self.grid_dim))
        
        for ox, oy in obstacles:
            if 0 <= ox < self.grid_dim and 0 <= oy < self.grid_dim:
                grid[ox, oy] = 0
        
        edt = distance_transform_edt(grid)
        
        # --- Using FIXED PENALTY FORMULA ---
        # 8.0 means the penalty field extends 8 grid units (200mm) outward.
        # np.clip ensures the penalty gently goes to 0 in wide open spaces.
        max_penalty_distance = 8.0 
        self.cost_map = np.clip(max_penalty_distance - edt, 0.0, max_penalty_distance)

    
    """
    ------------------------------------------------------------
    Function: heuristic

    Input:
        a (tuple), b (tuple) :
            Two grid or world coordinate points.

    Output:
        float :
            Estimated cost between the two points.

    Logic Explanation:
            Uses Euclidean distance as an admissible heuristic
            for path planning. Provides a straight-line estimate
            without considering obstacles.
    """

    def heuristic(self, a, b):
        """Euclidean heuristic"""
        return math.dist(a, b)

    """
    ------------------------------------------------------------
    Function: to_grid

    Input:
        p (tuple) :
            World coordinate (x, y).

    Output:
        tuple :
            Corresponding grid coordinate.

    Logic Explanation:
            Converts continuous world coordinates into discrete
            grid indices using the planner resolution. Rounding
            ensures alignment with grid cells.
    """
    def to_grid(self, p):
        """World → Grid"""
        return (round(p[0] / self.res), round(p[1] / self.res))

    """
    ------------------------------------------------------------
    Function: to_world

    Input:
        p (tuple) :
            Grid coordinate.

    Output:
        tuple :
            Corresponding world coordinate.

    Logic Explanation:
            Converts discrete grid indices back into real-world
            coordinates by scaling with resolution.
    """
    def to_world(self, p):
        """Grid → World"""
        return (p[0] * self.res, p[1] * self.res)

    """
    ------------------------------------------------------------
    Function: line_of_sight

    Input:
        p0 (tuple), p1 (tuple) :
            Start and end grid coordinates.
        obstacles (set or list) :
            Obstacle positions in grid space.

    Output:
        bool :
            True if path is collision-free, otherwise False.

    Logic Explanation:

        1. Use an incremental grid traversal (Bresenham-style)
           to move from start to end cell.

        2. At each step, check a local 7x7 neighborhood around
           the current cell to approximate robot footprint.

        3. If any cell within this neighborhood intersects
           an obstacle, the path is invalid.

        4. If traversal completes without collision, the
           path segment is considered safe.
    """
    def line_of_sight(self, p0, p1, obstacles):
        """
        Checks a 7x7 grid area (approx 175mm) along the path to ensure
        the robot footprint never hits an obstacle.
        """
        x0, y0 = p0
        x1, y1 = p1

        dx, dy = abs(x1 - x0), abs(y1 - y0)
        sx, sy = (1 if x1 > x0 else -1), (1 if y1 > y0 else -1)
        err = dx - dy

        x, y = x0, y0

        while (x, y) != (x1, y1):# 225 mm 25*9
            for nx in range(x - 4, x + 5):
                for ny in range(y - 4, y + 5):
                    if (nx, ny) in obstacles:
                        return False

            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy

        return True

    """
    ------------------------------------------------------------
    Function: plan

    Input:
        start (tuple) :
            Start position in world coordinates.
        goal (tuple) :
            Goal position in world coordinates.
        obstacles (set or list) :
            Obstacle positions in grid space.

    Output:
        list :
            Planned path from start to goal in world coordinates.
            Returns an empty list if no path is found.

    Logic Explanation:

        1. Convert start and goal from world coordinates to grid space.

        2. Update the cost map to generate obstacle-based penalties
           before running the planner.

        3. Initialize open set (priority queue), cost dictionary (g),
           parent tracking (came_from), and closed set.

        4. Perform Cost-Aware Theta* search:
           - Pop the node with lowest f = g + heuristic.
           - Stop if goal is reached.
           - Explore neighboring cells while skipping obstacles
             and already processed nodes.

        5. For each neighbor:
           - Retrieve obstacle proximity penalty from cost_map.
           - Gradually fade out penalty when close to the goal
             to allow smoother final approach.
           - Compute step cost including penalty.

        6. Apply Theta* optimization:
           - If there is line of sight from parent to neighbor,
             use any-angle shortcut.
           - Otherwise, follow grid-based expansion.

        7. Update cost and parent if a better path is found,
           and push neighbor into the priority queue.

        8. If goal is reached, reconstruct and return the path.
           If search exceeds iteration limit or no path exists,
           return an empty list.
    """
    def plan(self, start, goal, obstacles):
        """Compute path from start to goal using Cost-Aware Theta*"""
        print(f"DEBUG: Start Grid: {self.to_grid(start)} Blocked? {self.to_grid(start) in obstacles}")
        print(f"DEBUG: Goal Grid: {self.to_grid(goal)} Blocked? {self.to_grid(goal) in obstacles}")
        start_grid = self.to_grid(start)
        goal_grid  = self.to_grid(goal)

        # --- UPDATE COST MAP BEFORE PLANNING ---
        self.update_cost_map(obstacles)

        open_set = []
        heappush(open_set, (0.0, start_grid))

        came_from = {start_grid: start_grid}
        g = {start_grid: 0.0}

        closed_set = set()

        iterations = 0
        MAX_ITERATIONS = 5000 

        while open_set:
            iterations += 1
            if iterations > MAX_ITERATIONS:
                return []

            _, current = heappop(open_set)

            if current == goal_grid:
                return self.reconstruct_path(came_from, goal_grid)

            closed_set.add(current)

            for dx, dy in self.neighbors:
                nbr = (current[0] + dx, current[1] + dy)

                if nbr in obstacles or nbr in closed_set:
                    continue
                
                 
                #  FETCH CENTERING PENALTY WITH GOAL 
                if 0 <= nbr[0] < self.grid_dim and 0 <= nbr[1] < self.grid_dim:
                    # 1. Calculate how far this neighbor is from the goal (in grid cells)
                    dist_to_goal_cells = self.heuristic(nbr, goal_grid)
                    
                    # 2. Fade out the penalty when within 16 cells (400mm) of the goal
                    if dist_to_goal_cells < 16.0:
                        # As dist goes from 16 to 0, fade_factor goes from 1.0 to 0.0
                        fade_factor = dist_to_goal_cells / 16.0
                    else:
                        fade_factor = 1.0
                        
                    # 3. Apply penalty: Lower base multiplier to 1.5, and multiply by fade_factor
                    center_penalty = self.cost_map[nbr[0], nbr[1]] * 1.5 * fade_factor 
                else:
                    center_penalty = 0.0

                step_cost = math.dist(current, nbr)
                parent = came_from[current]

                if self.line_of_sight(parent, nbr, obstacles):
                    # Add penalty to the any-angle step
                    new_g = g[parent] + math.dist(parent, nbr) + center_penalty
                    new_parent = parent
                else:
                    # Add penalty to the grid-following step
                    new_g = g[current] + step_cost + center_penalty
                    new_parent = current

                if nbr not in g or new_g < g[nbr]:
                    g[nbr] = new_g
                    came_from[nbr] = new_parent
                    f = new_g + self.heuristic(nbr, goal_grid)
                    heappush(open_set, (f, nbr))

        return []


    """
    ------------------------------------------------------------
    Function: reconstruct_path

    Input:
        came_from (dict) :
            Dictionary mapping each node to its parent node.
        current (tuple) :
            Goal node in grid coordinates.

    Output:
        list :
            Ordered path from start to goal in world coordinates.

    Logic Explanation:

        1. Start from the goal node and iteratively trace
           parent nodes using the came_from dictionary.

        2. Continue backtracking until the start node
           (where parent equals itself) is reached.

        3. Reverse the collected sequence to obtain
           start-to-goal order.

        4. Convert grid coordinates back into world
           coordinates before returning the final path.
    """
    def reconstruct_path(self, came_from, current):
        path = [current]
        while came_from[current] != current:
            current = came_from[current]
            path.append(current)

        path.reverse()
        return [self.to_world(p) for p in path]

# ---------------------- PID Controller Class ---------------------------------
class PID:

    """
    ------------------------------------------------------------
    Function: __init__

    Input:
        kp (float) :
            Proportional gain controlling response to current error.
        ki (float) :
            Integral gain correcting accumulated past error.
        kd (float) :
            Derivative gain reducing overshoot by reacting to error rate.
        max_out (float) :
            Maximum allowed controller output for saturation control.

    Output:
        None

    Logic Explanation:

        1. Store PID gain parameters for proportional,
           integral, and derivative components.

        2. Set maximum output limit to prevent excessive
           actuator commands and reduce instability.

        3. Initialize integral term to zero to begin
           accumulated error tracking.

        4. Initialize previous error to support
           derivative computation in future updates.

    Example:
        PID(0.5, 0.01, 0.1)
    """
    def __init__(self, kp, ki, kd, max_out=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_out = max_out
        self.integral = 0.0
        self.prev_error = 0.0


    """
    ------------------------------------------------------------
    Function: compute

    Input:
        error (float) :
            Current control error.
        dt (float) :
            Time step since last update.

    Output:
        float :
            Saturated PID control output.

    Logic Explanation:

        1. Compute derivative term using change in error
           over time to anticipate future trend.

        2. Compute proportional term directly from
           current error magnitude.

        3. Combine proportional, integral, and derivative
           contributions to form raw controller output.

        4. Clip the output within ±max_out to prevent
           actuator saturation.

        5. Apply anti-windup:
           Update integral term only when output is not
           saturated, preventing excessive accumulation.

        6. Store current error for next derivative
           calculation and return the clipped output.

    Example:
        pid.compute(10.5, 0.05)
    """
    def compute(self, error, dt):
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        proportional = self.kp * error
        derivative_term = self.kd * derivative
        output = proportional + derivative_term + (self.ki * self.integral)
        clipped_output = np.clip(output, -self.max_out, self.max_out)

        if abs(output) < self.max_out:
            self.integral += error * dt

        self.prev_error = error
        return clipped_output


    """
    * Function Name: reset
    * Input: None
    * Output: None
    * Logic: Resets integral and previous error to zero.
    * Example Call: pid.reset()
    """
    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0


# ---------------------- Main Node Class -------------------------------------
class MultiHolonomicController(Node):


    """
    ------------------------------------------------------------
    Function: __init__

    Input:
        None

    Output:
        None

    Logic Explanation:

        1. Initialize the ROS2 node and retrieve configurable
           robot parameters such as name and ID.

        2. Set up multi-robot coordination structures including
           zone locking, intermediate waiting points, and
           active task tracking for safe shared-zone access.

        3. Initialize global path planning using Theta* along
           with cost-aware navigation and deadlock monitoring.

        4. Configure drop zone boundaries and slot manager for
           structured crate placement with layered stacking.

        5. Initialize robot state machine variables for task
           execution, goal tracking, arm positioning, and docking.

        6. Define robot geometry, tolerances, PID controllers,
           and motion constraints for stable navigation.

        7. Configure allocator-related data structures to
           manage crate assignments across multiple robots.

        8. Set up collision avoidance parameters including
           repulsion gains and safety distances.

        9. Create ROS2 publishers, subscribers, callback groups,
           and timers to support multithreaded execution where
           control and background communication run independently.

        10. Initialize gripper and arm to safe default positions
            before entering the control loop.

    Example:
        Automatically invoked by the ROS2 entry point.
    """
    def __init__(self):
        super().__init__('multiholonomic_controller')

        # ---------------- Robot Parameters ----------------
        self.declare_parameter('robot_name', 'hb_crystal')
        self.declare_parameter('robot_id', 0)

        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        self.robot_id = self.get_parameter('robot_id').get_parameter_value().integer_value

        self.LEADER_ROBOT_ID = 0

        
        self.zone_locks = {
            'D1': {'status': 'FREE', 'owner': None},
            'D2': {'status': 'FREE', 'owner': None},
            'D3': {'status': 'FREE', 'owner': None},
            'INT_D1_A': {'status': 'FREE', 'owner': None},
            'INT_D1_B': {'status': 'FREE', 'owner': None},
            'INT_D2_A': {'status': 'FREE', 'owner': None},
            'INT_D3_A': {'status': 'FREE', 'owner': None},
            'INT_COMMON_23': {'status': 'FREE', 'owner': None},
        }

        # Tracks which slots are actively being transported per zone
        self.active_zone_tasks = {'D1': {}, 'D2': {}, 'D3': {}}
        self.assigned_slot_idx = 999 

        # Define 2 safe waiting spots for each zone.
        self.intermediate_pts = {
            'D1': {
                'INT_D1_A': (820.0, 1100.0, 0.0),  # Left side of D1
                'INT_D1_B': (1620.0, 1100.0, 0.0)   # Right side of D1
            },
            'D2': {
                'INT_D2_A': (450.0, 2000.0, 0.0),      # left 
                'INT_COMMON_23': (1218.0, 2000.0, 0.0) # shared middle point
            },
            'D3': {
                'INT_D3_A': (2000.0, 2000.0, 0.0),    # right
                'INT_COMMON_23': (1218.0, 2000.0, 0.0) # Shared middle point
            }
        }

        self.get_logger().info(f"--- Starting Controller for {self.robot_name} (ID: {self.robot_id}) ---")
        if self.robot_id == self.LEADER_ROBOT_ID:
            self.get_logger().info(f"*** I AM THE LEADER/ALLOCATOR ***")

        self.last_time = self.get_clock().now()
        self.max_vel = 600.0

        # ---------------- Global Planner Init ----------------
        self.paths = {}
        self.path = []
        self.path_index = 0
        self.planner = ThetaStarPlanner(resolution=25.0)

        # Deadlock detection
        self.last_progress_time = self.get_clock().now()
        self.last_dist_to_goal = float('inf')

        # ---------------- Slot Management Init ----------------
        self.drop_zone_bounds = {
            "D1": (1020, 1410, 1075, 1355),
            "D2": (675, 965, 1920, 2115),
            "D3": (1470, 1762, 1920, 2115),
        }
        self.drop_manager = DropZoneManager(self.drop_zone_bounds, slot_rows=2, slot_cols=2)

        # ---------------- Controller-Specific Logic ----------------
        self.current_pose = None
        self.state = 'IDLE'
        self.crate_picked = False
        self.stable_count = 0
        self.boundary_condition_met=0

        self.assigned_crate_id = None
        self.assigned_crate_color = None
        self.assigned_drop_goal = None
        self.crate_model_name = None
        self.crate_pose = None
        self.current_goal = None

        self.my_pose_received = False

        # Drop Zones
        self.drop_zones = {
            'RED': (np.mean([1020, 1410]), np.mean([1075, 1355])),
            'GREEN': (np.mean([675, 965]), np.mean([1920, 2115])),
            'BLUE': (np.mean([1470, 1762]), np.mean([1920, 2115])),
        }
        self.docking_zones = {
            'hb_glacio': (864.0, 204.0, 0.0),
            'hb_crystal': (1218.0, 205.0, 0.0),
            'hb_frostbite': (1568.0, 202.0, 0.0)
        }
        self.docking_zone = self.docking_zones[self.robot_name]

        # Arm Config
        self.arm_base_pickup = 175.0
        self.arm_elbow_pickup = 100.0
        self.arm_base_neutral = 100.0
        self.arm_elbow_neutral = 30.0
        # Placement geometry
        self.arm_reach = 90.0  # mm (distance from robot center to crate while placing)


        self.current_arm_base = 90.0
        self.current_arm_elbow = 40.0
        self.arm_offset_rad = math.radians(-90.0)

        # Robot Geom & Tolerances
        self.robot_radius = 80.0
        self.approach_distance = 225.0
        self.pickup_distance = 150.0
        self.position_tolerance = 30.0
        self.angle_tolerance = 7.0
        self.final_position_tolerance = 25.0
        self.final_angle_tolerance = math.radians(10.0)

        # PID Controllers
        self.pid_params = {
            'x': {'kp': 0.5, 'ki': 0.005, 'kd': 0.2, 'max_out': self.max_vel * 0.8},
            'y': {'kp': 0.5, 'ki': 0.005, 'kd': 0.2, 'max_out': self.max_vel * 0.8},
            'theta': {'kp': 0.7, 'ki': 0.05, 'kd': 0.5, 'max_out': self.max_vel * 1.0}
        }
        self.pid_x = PID(**self.pid_params['x'])
        self.pid_y = PID(**self.pid_params['y'])
        self.pid_theta = PID(**self.pid_params['theta'])

        # ---------------- Allocator-Specific Logic ----------------
        self.robot_names = ['hb_glacio', 'hb_crystal', 'hb_frostbite']
        self.robot_name_to_id = {'hb_glacio': 4, 'hb_crystal': 0, 'hb_frostbite': 2}
        self.robot_id_to_name = {v: k for k, v in self.robot_name_to_id.items()}

        self.all_robot_poses = {}
        self.all_crate_poses = {}
        self.robot_states = {name: 'IDLE' for name in self.robot_names}
        self.crate_data = {}
        self.ALLOWED_CRATE_IDS = {14, 26, 23, 12, 21, 30}

        self.get_logger().warn(
            f"🧹 Crate filter enabled. Allowed crate IDs: {self.ALLOWED_CRATE_IDS}"
        )

        self.allocator_drop_zones = {
            'D1_RED': (np.mean([1020, 1410]), np.mean([1075, 1355])),
            'D2_GREEN': (np.mean([675, 965]), np.mean([1920, 2115])),
            'D3_BLUE': (np.mean([1470, 1762]), np.mean([1920, 2115])),
        }




        self.layer_config = {
            0: {'reach': 120.0, 'base': 170.0, 'elbow': 100.0},
            1: {'reach': 120.0, 'base': 128.0, 'elbow': 60.0},
            # We can add Layer 2 later when we have the values:
            # 2: {'reach': 65.0, 'base': 100.0, 'elbow': 40.0},
        }
        self.placed_crate_ids = set()
        self.total_crates_to_deliver = 4

        # ---------------- Collision Avoidance ----------------
        self.avoid_robot_dist = 350.0
        self.avoid_crate_dist = 200.0
        self.robot_repulsion_gain = 5.5
        self.crate_repulsion_gain = 5.5

        self.priority_gain_multiplier = 4.0
        self.non_priority_gain_multiplier = 4.0
        self.max_repulsion_vel = self.max_vel * 1.0
        self.idle_start_time = None


        # ---------------- Callback Groups for Multithreading ----------------
        # Group 1: Background listening (never sleeps)
        self.listener_group = MutuallyExclusiveCallbackGroup()
        # Group 2: Main control loop (allowed to use time.sleep)
        self.control_group = MutuallyExclusiveCallbackGroup()

        # ---------------- ROS 2 Publishers & Subscribers ----------------
        self.ir_detected = False
        # Add callback_group=self.listener_group to ALL subscribers
        self.ir_sub = self.create_subscription(Bool, f'/robot_{self.robot_id}/ir_sensor_status', self.ir_cb, 10, callback_group=self.listener_group)

        self.bot_cmd_pub = self.create_publisher(BotCmdArray, f'/bot_cmd', 10)
        self.attach_client = self.create_client(SetBool, f'/robot_{self.robot_id}/attach')

        self.pose_sub = self.create_subscription(Poses2D, '/bot_pose', self.pose_cb, 10, callback_group=self.listener_group)
        self.crate_sub = self.create_subscription(Poses2D, '/crate_pose', self.crate_pose_cb, 10, callback_group=self.listener_group)

        self.task_sub = self.create_subscription(String, '/task_assignments', self.task_cb, 10, callback_group=self.listener_group)
        self.task_complete_pub = self.create_publisher(String, '/task_complete', 10)

        self.task_assign_pub = self.create_publisher(String, '/task_assignments', 10)
        self.task_complete_sub = self.create_subscription(String, '/task_complete', self.task_complete_cb, 10, callback_group=self.listener_group)

        # Communication for Locking
        self.zone_status_pub = self.create_publisher(String, '/zone_status', 10)
        self.zone_status_sub = self.create_subscription(String, '/zone_status', self.zone_status_cb, 10, callback_group=self.listener_group)

        # Map Colors to Zone IDs for easy lookup
        self.color_to_zone_map = {'RED': 'D1', 'GREEN': 'D2', 'BLUE': 'D3'}

        # ---------------- Timers ----------------
        # The control loop goes into the control_group
        self.control_timer = self.create_timer(0.05, self.control_cb, callback_group=self.control_group)
        # The allocator goes into the listener_group
        self.alloc_timer = self.create_timer(1.0, self.allocate_tasks, callback_group=self.listener_group)

        self.get_logger().info('🚀 Multi-Holonomic Controller (Multithreaded) started.')
        self.set_gripper(False)
        self.move_arm(self.current_arm_base, self.current_arm_elbow)


    """
    ------------------------------------------------------------
    Function: zone_status_cb

    Input:
        msg (String) :
            Message received from the '/zone_status' topic.
            Expected format: 'ZONE_ID,STATUS,OWNER_ID'
            Example: 'D1,OCCUPIED,2'

    Output:
        None

    Logic Explanation:

        1. Parse the incoming string message into zone ID,
           status, and owner robot ID.

        2. Validate that the zone exists in local tracking
           (self.zone_locks).

        3. Update the zone’s status (FREE/OCCUPIED) and assign
           or clear the owner accordingly.

        4. Handle parsing errors gracefully and log failures
           without interrupting controller execution.

    Example:
        Triggered automatically when a message is received
        on the '/zone_status' topic.
    """
    def zone_status_cb(self, msg):
        """
        Updates local state based on what other robots publish.
        Expected Msg format: 'ZONE_ID,STATUS,OWNER_ID' (e.g., 'D1,OCCUPIED,2')
        """
        try:
            parts = msg.data.split(',')
            zone_id = parts[0]
            status = parts[1]
            owner_id = int(parts[2])

            if zone_id in self.zone_locks:
                self.zone_locks[zone_id]['status'] = status
                self.zone_locks[zone_id]['owner'] = owner_id if status == 'OCCUPIED' else None
        except Exception as e:
            self.get_logger().error(f"Failed to parse zone msg: {e}")

    """
    ------------------------------------------------------------
    Function: publish_zone_update

    Input:
        zone_id (str) :
            Identifier of the zone to update.
        status (str) :
            New status of the zone ('FREE' or 'OCCUPIED').

    Output:
        None

    Logic Explanation:

        1. Determine the owner ID:
           - If status is 'OCCUPIED', assign this robot’s ID.
           - If status is 'FREE', use -1 to indicate no owner.

        2. Construct and publish a formatted message
           'ZONE_ID,STATUS,OWNER_ID' to notify all robots.

        3. Immediately update the local zone_locks dictionary
           to maintain consistency without waiting for the
           subscriber callback.

        4. Ensure that owner is cleared locally when the
           zone is released.

    Example:
        self.publish_zone_update('D1', 'OCCUPIED')
    """
    def publish_zone_update(self, zone_id, status):
        """Broadcasts a lock/unlock event to all robots"""
        owner = self.robot_id if status == 'OCCUPIED' else -1
        msg = String()
        msg.data = f"{zone_id},{status},{owner}"
        self.zone_status_pub.publish(msg)

        # Update our own local state immediately (don't wait for the callback)
        self.zone_locks[zone_id]['status'] = status
        self.zone_locks[zone_id]['owner'] = owner if status == 'OCCUPIED' else None

    """
    * Function Name: get_target_zone_id
    * Input: None
    * Output: string ('D1', 'D2', 'D3')
    * Logic: Maps current assigned crate color to its corresponding physical drop zone.
    * Example Call: self.get_target_zone_id()
    """
    def get_target_zone_id(self):
        """Returns 'D1', 'D2', or 'D3' based on current crate color"""
        return self.color_to_zone_map.get(self.assigned_crate_color)


    """
    ------------------------------------------------------------
    Function: log_navigation_debug

    Input:
        state_name (str) :
            Current navigation state label.
        next_wp (tuple, optional) :
            Next waypoint in world coordinates.
        goal (tuple, optional) :
            Final goal position in world coordinates.

    Output:
        None

    Logic Explanation:

        1. Ensure current robot pose is available before logging.

        2. Construct a formatted debug message including
           current position.

        3. If a next waypoint is provided, compute and append
           distance to waypoint.

        4. If a goal is provided, compute and append
           distance to goal.

        5. Use throttled logging to limit output frequency
           and maintain terminal readability during control loops.

    Example:
        self.log_navigation_debug("NAV", wp, goal)
    """
    def log_navigation_debug(self, state_name, next_wp=None, goal=None):
        if self.current_pose is None:
            return

        cx, cy, _ = self.current_pose

        msg = f"[NAV][{state_name}] Pose=({cx:.1f},{cy:.1f})"

        if next_wp is not None:
            dx = math.dist((cx, cy), next_wp)
            msg += f" | NextWP=({next_wp[0]:.1f},{next_wp[1]:.1f}) d_wp={dx:.1f}"

        if goal is not None:
            dg = math.dist((cx, cy), goal)
            msg += f" | Goal=({goal[0]:.1f},{goal[1]:.1f}) d_goal={dg:.1f}"

        self.get_logger().info(msg, throttle_duration_sec=1.0)

    """
    ------------------------------------------------------------
    Function: add_zone_as_obstacles

    Input:
        bounds (tuple) :
            Zone boundary in world coordinates
            (xmin, xmax, ymin, ymax).
        obstacles (set) :
            Set of blocked grid cells used by the planner.
        resolution (float) :
            Grid resolution (mm per cell).

    Output:
        None

    Logic Explanation:

        1. Inflate the zone boundaries by a fixed offset to
           account for robot radius and safety clearance.

        2. Keep one boundary (entry side) minimally inflated
           if required to allow controlled approach.

        3. Convert inflated world coordinates into grid indices
           using the planner resolution.

        4. Mark all grid cells inside the inflated region as
           blocked by adding them to the obstacles set.

        5. This ensures the planner treats the entire zone
           (with buffer) as non-traversable space.

    Example:
        self.add_zone_as_obstacles(bounds, obs, 50.0)
    """
    def add_zone_as_obstacles(self, bounds, obstacles, resolution):
        """Add zone as obstacles with an INFLATION buffer for robot radius."""
        offset = 40.0
        xmin, xmax, ymin, ymax = bounds
        # inflate the bounds in all directions of zone obstacles
        inf_xmin=xmin + offset
        inf_xmax=xmax - offset
        inf_ymin=ymin + 0.0  # clear boundary in the dropping face of the drop zone
        inf_ymax=ymax + 0.0

        # Convert inflated world coordinates to grid coordinates

        gx_min = int(inf_xmin / resolution)
        gx_max = int(inf_xmax / resolution)
        gy_min = int(inf_ymin / resolution)
        gy_max = int(inf_ymax / resolution)

        # Block all cells within the inflated area
        for gx in range(gx_min, gx_max + 1):
            for gy in range(gy_min, gy_max + 1):
                obstacles.add((gx, gy))

       

    """
    ------------------------------------------------------------
    Function: is_in_inflated_drop_zone

    Input:
        pose (tuple) :
            Robot position in world coordinates (x, y).

    Output:
        bool :
            True if the pose lies inside any inflated
            drop zone boundary, otherwise False.

    Logic Explanation:

        1. Iterate through all configured drop zone bounds.

        2. Apply the same inflation margin used for
            obstacle generation to maintain consistency.

        3. Check whether the given pose falls within
            the inflated boundary limits.

        4. Return True immediately if a match is found;
            otherwise return False after checking all zones.

    Example:
        self.is_in_inflated_drop_zone((x, y))
    """
    def is_in_inflated_drop_zone(self, pose):
        """Check if the given pose is inside any inflated drop zone boundary."""
        if pose is None:
            return False

        x, y = pose[0], pose[1]
        offset = 0

        for bounds in self.drop_zone_bounds.values():
            xmin, xmax, ymin, ymax = bounds
            inf_xmin = xmin - offset
            inf_xmax = xmax + offset
            inf_ymin = ymin - offset
            inf_ymax = ymax + offset

            if inf_xmin <= x <= inf_xmax and inf_ymin <= y <= inf_ymax:
                return True

        return False

    # ------get_all_obstacles and inflate for planning-----
    """
    ------------------------------------------------------------
    Function: get_all_obstacles

    Input:
        exclude_zone (str, optional) :
            Zone name to ignore while building obstacle set.
        skip_id (int, optional) :
            Robot ID to exclude (typically this robot).
        exclude_crate_id (int, optional) :
            Crate ID to ignore (e.g., the crate being picked).

    Output:
        set :
            Set of blocked grid cells for path planning.

    Logic Explanation:

        1. Initialize an empty obstacle set and retrieve
           planner resolution.

        2. Add other robots as obstacles with inflation
           to create a safety buffer around each robot.

        3. Add crates as obstacles with smaller inflation,
           excluding the crate currently being handled.

        4. Add drop zones as inflated obstacle regions,
           unless a specific zone is intentionally excluded.

        5. Ensure the robots current grid position is not
           accidentally marked as blocked.

        6. Return the aggregated obstacle set for use
           in collision-aware path planning.

    Example:
        self.get_all_obstacles(exclude_zone='D1')
    """
    def get_all_obstacles(self, exclude_zone=None, skip_id=None, exclude_crate_id=None):
        """Helper to gather all current obstacles (robots + zones + crates) with proper inflation."""
        obstacles = set()
        res = self.planner.res

        # Default to skipping ourselves if no ID provided
        if skip_id is None:
            skip_id = self.robot_id

        # 1. Add OTHER ROBOTS with Inflation (80mm bubble)
        robot_inflation = 100
        for rid, p in self.all_robot_poses.items():
            if rid == skip_id:
                continue
            rx, ry = p[0], p[1]
            # Block a square area around the other robot
            for dx in range(-robot_inflation, robot_inflation + 1, int(res)):
                for dy in range(-robot_inflation, robot_inflation + 1, int(res)):
                    obstacles.add((round((rx + dx) / res), round((ry + dy) / res)))

        # 2. Add CRATES with Inflation (30mm bubble)
        crate_inflation = 30
        for cid, cp in self.all_crate_poses.items():
            # Don't block the crate we are currently trying to pick up!
            if exclude_crate_id is not None and cid == exclude_crate_id:
                continue
            cx, cy = cp[0], cp[1]
            for dx in range(-crate_inflation, crate_inflation + 1, int(res)):
                for dy in range(-crate_inflation, crate_inflation + 1, int(res)):
                    obstacles.add((round((cx + dx) / res), round((cy + dy) / res)))

        # 3. Add DROP ZONES with Inflation
        for zone_name, bounds in self.drop_zone_bounds.items():
            # Check if this zone should be ignored
            if exclude_zone and exclude_zone in zone_name:
                continue
            self.add_zone_as_obstacles(bounds, obstacles, res)

        # 4. Ensuring current position isn't blocked by its own inflation
        curr_grid = (round(self.current_pose[0]/res), round(self.current_pose[1]/res))
        if curr_grid in obstacles:
            obstacles.remove(curr_grid)

        return obstacles




    # ---------------- IR Callback ----------------

    """
    * Function Name: ir_cb
    * Input: msg (mqtt message)
    * Output: None
    * Logic: Updates the local IR detection state from the MQTT bridge.
    * Example Call: self.ir_cb(msg)
    """
    def ir_cb(self, msg):
        """Update local IR state from mqtt bridge"""
        self.ir_detected = msg.data

    # ---------------- Allocator Logic & Callbacks ----------------


    """
    * Function Name: get_crate_color_and_zone
    * Input: crate_id (int)
    * Output: tuple (color, zone)
    * Logic: Determines crate property based on ID modulo 3.
    * Example Call: self.get_crate_color_and_zone(12)
    """
    def get_crate_color_and_zone(self, crate_id):
        mod = crate_id % 3
        if mod == 0:
            return 'RED', 'D1_RED'
        elif mod == 1:
            return 'GREEN', 'D2_GREEN'
        elif mod == 2:
            return 'BLUE', 'D3_BLUE'
        return 'UNKNOWN', 'UNKNOWN'


    """
    * Function Name: task_complete_cb

    Input:
        msg (String) :
            Message received from the '/task_complete' topic.
            Expected format: 'robot_name,crate_id'.

    Output:
        None

    Logic Explanation:

        1. Parse the message to extract robot name and
           completed crate ID.

        2. Convert robot name to numeric ID for
           internal tracking consistency.

        3. Remove any active slot assignments associated
           with the finished robot.

        4. Update robot state to 'IDLE' and mark the
           crate as 'PLACED' in tracking structures.

        5. Add crate ID to the placed set to prevent
           reallocation.

        6. If this robot is the leader, verify that no
           zone remains locked by the now-idle robot.
           Force unlock any such zone to prevent deadlock.

        7. Handle parsing errors safely and log issues
           without interrupting execution.

    Example:
        Triggered automatically by '/task_complete'.
    """
    def task_complete_cb(self, msg):
        try:
            robot_name, crate_id_str = msg.data.split(',')
            crate_id = int(crate_id_str)

            # Get the numeric ID of the robot that finished
            finished_robot_id = self.robot_name_to_id.get(robot_name, -1)
            
            for zone, tasks in self.active_zone_tasks.items():
                keys_to_delete = [s for s, r in list(tasks.items()) if r == robot_name]
                for k in keys_to_delete:
                    del tasks[k]
            if robot_name in self.robot_states:
                self.robot_states[robot_name] = 'IDLE'        
                self.get_logger().info(f"📬 ALLOCATOR: Robot {robot_name} is now IDLE.")

            if crate_id in self.crate_data:
                self.crate_data[crate_id]['state'] = 'PLACED'

            self.placed_crate_ids.add(crate_id)

            if self.robot_id == self.LEADER_ROBOT_ID and finished_robot_id != -1:
                for z_id, info in self.zone_locks.items():
                    if info['status'] == 'OCCUPIED' and info['owner'] == finished_robot_id:
                        self.get_logger().warn(f"🧹 SUPERVISOR: Force-Unlocking Zone {z_id} (Held by IDLE robot {finished_robot_id})")
                        self.publish_zone_update(z_id, 'FREE')

        except Exception as e:
            self.get_logger().error(f"Failed to parse task_complete msg: {e}")


    """
    Function: get_optimal_approach_pose

    Input:
        crate_pose (tuple/list) :
            Crate position and orientation (x, y, theta).
        robot_pose (tuple/list) :
            Current robot pose (x, y, ...).
        offset (float) :
            Distance from crate center to generate approach point.
        obstacles (set, optional) :
            Grid obstacles for collision validation.
        res (float) :
            Grid resolution for obstacle lookup.

    Output:
        tuple (x, y) :
            Best valid approach position in world coordinates.

    Logic Explanation:

        1. Generate four candidate approach points aligned
           perpendicular to each crate face.

        2. Discard candidates that fall outside arena limits
           to prevent invalid navigation targets.

        3. If obstacle data is provided, reject approach
           points that lie inside blocked grid cells.

        4. Compute distance from robot to each valid
           candidate and select the closest one.

        5. If all faces are blocked, fall back to the
           crate center as a safe default.

    Example:
        self.get_optimal_approach_pose(crate_pose, robot_pose)
    """
    

    def get_optimal_approach_pose(self, crate_pose, robot_pose, offset=170.0, obstacles=None, res=25.0):
        """
        Generates 4 points perpendicular to crate faces and filters out blocked ones.
        """
        cx, cy, cw = crate_pose
        rx, ry = robot_pose[:2]

        ARENA_MAX, ARENA_MIN = 2438.4, 0.0
        angles = [0.0, math.pi/2, math.pi, -math.pi/2]
        candidates = []

        for ang in angles:
            theta = cw + ang
            px = cx + offset * math.cos(theta)
            py = cy + offset * math.sin(theta)

            if (ARENA_MIN + 100) < px < (ARENA_MAX - 100) and \
               (ARENA_MIN + 100) < py < (ARENA_MAX - 100):
                
                # --- If this specific approach face is blocked, skip it! ---
                if obstacles is not None:
                    grid_pt = (round(px / res), round(py / res))
                    if grid_pt in obstacles:
                        continue 

                dist = math.dist((rx, ry), (px, py))
                candidates.append((px, py, dist))

        if not candidates:
            # Fallback if ALL 4 faces are blocked 
            return (float(cx), float(cy)) 

        candidates.sort(key=lambda x: x[2]) # Closest valid face
        best_x, best_y, _ = candidates[0]
        return (float(best_x), float(best_y))


    """
    ------------------------------------------------------------
    Function: allocate_tasks

    Input:
        None

    Output:
        None

    Logic Explanation:

        1. Execute only if this robot is the designated leader.

        2. Identify all IDLE robots with valid pose data and
           all crates marked as 'UNASSIGNED'.

        3. If either list is empty, exit early.

        4. Construct a cost matrix where each entry represents
           Euclidean distance between a robot and a crate.

        5. Apply the Hungarian Algorithm to compute an optimal
           minimum-distance assignment.

        6. For each assignment:
           - Determine the correct drop zone.
           - Request the next available pyramid slot.
           - If zone is full, skip safely.

        7. Generate obstacles excluding the assigned robot
           and crate, then compute a valid approach pose.

        8. Plan a path using Cost-Aware Theta* toward the
           approach point. If planning fails, release the slot.

        9. Append final docking point (crate center) to
           complete the pickup path.

        10. Publish assignment message including path,
            drop coordinates, layer index, zone, and slot index.

        11. Update robot and crate states to reflect
            active task allocation.

        12. Log summary if assignments were successfully published.

    Example:
        Triggered periodically by allocator timer.
    """
    def allocate_tasks(self):
        if self.robot_id != self.LEADER_ROBOT_ID:
            return

        idle_robot_names = [name for name, state in self.robot_states.items() if state == 'IDLE']
        idle_robot_ids = [self.robot_name_to_id[name] for name in idle_robot_names
                          if self.robot_name_to_id[name] in self.all_robot_poses]

        unassigned_crate_ids = [cid for cid, data in self.crate_data.items()
                                if data['state'] == 'UNASSIGNED']

        if not idle_robot_ids or not unassigned_crate_ids:
            return

        self.get_logger().info(f'ALLOCATOR: Found {len(idle_robot_ids)} idle robots and {len(unassigned_crate_ids)} unassigned crates.', throttle_duration_sec=5.0)

        num_robots = len(idle_robot_ids)
        num_crates = len(unassigned_crate_ids)
        cost_matrix = np.zeros((num_robots, num_crates))

        for i in range(num_robots):
            robot_id = idle_robot_ids[i]
            robot_pose = self.all_robot_poses[robot_id]
            for j in range(num_crates):
                crate_id = unassigned_crate_ids[j]
                crate_pose = self.crate_data[crate_id]['pose']
                cost = math.dist(robot_pose[:2], crate_pose[:2])
                cost_matrix[i, j] = cost

        # try:
        #     row_ind, col_ind = linear_sum_assignment(cost_matrix)
        # except ValueError as e:
        #     self.get_logger().warn(f"Hungarian assignment failed: {e}")
        #     return

        
        # num_assignments = 0
        # for r, c in zip(row_ind, col_ind):
        #     if r >= num_robots or c >= num_crates:
        #         continue

        #     robot_id = idle_robot_ids[r]
        #     crate_id = unassigned_crate_ids[c]
        #     robot_name = self.robot_id_to_name[robot_id]

        #     full_zone_name = self.crate_data[crate_id]["zone"]
        #     simple_zone = full_zone_name.split('_')[0]


        #     slot_coords, layer_idx = self.drop_manager.get_next_pyramid_slot(simple_zone)

        #     if slot_coords is None:
        #         self.get_logger().error(f"❌ Allocator: Zone {simple_zone} is FULL or Pyramid Complete!")
        #         continue

        #     slot_idx = sum(self.drop_manager.zone_occupancy[simple_zone]) - 1

        #     drop_x, drop_y = slot_coords


        #     self.get_logger().info(f"📍 Allocator: Assigned {robot_name} to Crate {crate_id} -> Drop: {simple_zone} at ({drop_x:.1f}, {drop_y:.1f})")


        #     start = self.all_robot_poses[robot_id][:2]
        #     goal = self.crate_data[crate_id]['pose'][:2]


        #     robot_pose = self.all_robot_poses[robot_id]
        #     crate_pose = self.crate_data[crate_id]['pose'] # [x, y, w]


        try:
            row_ind, col_ind = linear_sum_assignment(cost_matrix)
        except ValueError as e:
            self.get_logger().warn(f"Hungarian assignment failed: {e}")
            return

        # --- NEW LOGIC: Calculate Total Heuristic Distance & Sort ---
        pending_assignments = []
        
        for r, c in zip(row_ind, col_ind):
            if r >= num_robots or c >= num_crates:
                continue

            robot_id = idle_robot_ids[r]
            crate_id = unassigned_crate_ids[c]

            # 1. Get Coordinates
            robot_pose = self.all_robot_poses[robot_id][:2]
            crate_pose = self.crate_data[crate_id]['pose'][:2]
            color = self.crate_data[crate_id]['color']
            zone_center = self.drop_zones[color]

            # 2. Calculate Total Heuristic Distance
            # Distance(Robot -> Crate) + Distance(Crate -> Drop Zone Center)
            dist_robot_to_crate = math.dist(robot_pose, crate_pose)
            dist_crate_to_zone = math.dist(crate_pose, zone_center)
            total_dist = dist_robot_to_crate + dist_crate_to_zone

            # 3. Store pending assignment details
            pending_assignments.append({
                'robot_id': robot_id,
                'crate_id': crate_id,
                'total_dist': total_dist
            })

        # 4. Sort assignments by total heuristic distance (Ascending)
        # This ensures robots with shorter total trips pull the earliest slot indexes.
        pending_assignments.sort(key=lambda x: x['total_dist'])

        num_assignments = 0
        
        # --- EXECUTE ASSIGNMENTS IN PRIORITIZED ORDER ---
        for assign in pending_assignments:
            robot_id = assign['robot_id']
            crate_id = assign['crate_id']
            robot_name = self.robot_id_to_name[robot_id]

            full_zone_name = self.crate_data[crate_id]["zone"]
            simple_zone = full_zone_name.split('_')[0]

            # Because the list is sorted, the robot with the lowest distance 
            # naturally claims the first available slot here:
            slot_coords, layer_idx = self.drop_manager.get_next_pyramid_slot(simple_zone)

            if slot_coords is None:
                self.get_logger().error(f"❌ Allocator: Zone {simple_zone} is FULL or Pyramid Complete!")
                continue

            slot_idx = sum(self.drop_manager.zone_occupancy[simple_zone]) - 1
            drop_x, drop_y = slot_coords

            self.get_logger().info(f"📍 Allocator: Assigned {robot_name} to Crate {crate_id} -> Drop: {simple_zone} at ({drop_x:.1f}, {drop_y:.1f}) | Heuristic Dist: {assign['total_dist']:.1f}mm")

            # ... (Continue with the rest of your path planning logic below this line)
            robot_pose = self.all_robot_poses[robot_id]
            crate_pose = self.crate_data[crate_id]['pose'] # [x, y, w]
            # ...



            # 2. PLAN TO APPROACH POINT
            start = robot_pose[:2]
            obstacles = self.get_all_obstacles(exclude_zone=None, skip_id=robot_id, exclude_crate_id=crate_id)
            approach_pt = self.get_optimal_approach_pose(
                crate_pose,
                robot_pose,
                offset=170.0, 
                obstacles=obstacles,  
                res=self.planner.res  
            )


            path = self.planner.plan(start, approach_pt, obstacles)

            # to check if path planning was successful
            if not path:
                self.get_logger().error(f"❌ PLANNER FAIL: Could not find path for {robot_name} to Crate {crate_id}. Start/Goal might be blocked!")

                # --- Release the slot so we don't burn through them! ---
                self.drop_manager.release_slot(simple_zone, slot_idx)
                continue # This prevents the robot from being marked 'WORKING'

            # 3. APPEND THE FINAL TARGET (The Crate itself)
            # This creates a straight-line "docking" segment at the end of the path
            path.append((crate_pose[0], crate_pose[1]))

            

            # self.get_logger().info(f"start: {start} , and goal: {goal}")
            self.paths[robot_name] = path

            self.get_logger().info(f"📈 PATH FOR {robot_name}: {path}")
            self.get_logger().info(
                f"[Theta*] Path for {robot_name} → Crate {crate_id} "
                f"({len(path)} pts): {path}",
                throttle_duration_sec=2.0
            )

            path_str = "|".join([f"{p[0]}:{p[1]}" for p in path])

            # Append layer_idx to the message
            # FORMAT: robot_name, crate_id, path, drop_x:drop_y, layer_idx
            msg = String()
            msg.data = f"{robot_name},{crate_id},{path_str},{drop_x}:{drop_y},{layer_idx},{simple_zone},{slot_idx}"
            self.task_assign_pub.publish(msg)

            self.robot_states[robot_name] = 'WORKING'
            self.crate_data[crate_id]['state'] = 'ASSIGNED'
            num_assignments += 1

        if num_assignments > 0:
            self.get_logger().info(f"ALLOCATOR: Published {num_assignments} new assignments.")

    # ---------------- Controller Logic & Callbacks ----------------


    """
    ------------------------------------------------------------
    Function: pose_cb

    Input:
        msg (Poses2D) :
            Message containing pose updates for all robots.

    Output:
        None

    Logic Explanation:

        1. If no poses are received, exit early.

        2. Iterate through all robot poses in the message
           and update the global robot pose dictionary.

        3. If the pose corresponds to this robot's ID:
           - Update current_pose.
           - Mark that the robot has received its first pose.
           - Log confirmation once to avoid repeated messages.

        4. Ensures synchronized multi-robot pose tracking
           for planning and coordination.

    Example:
        Triggered automatically by '/bot_pose' topic.
    """
    def pose_cb(self, msg):
        if len(msg.poses) == 0:
            return
        for pose in msg.poses:
            self.all_robot_poses[pose.id] = [pose.x, pose.y, pose.w]
            if pose.id == self.robot_id:
                self.current_pose = [pose.x, pose.y, pose.w]
                if not self.my_pose_received:
                    self.get_logger().info(f'✅ My pose (ID: {self.robot_id}) has been acquired.')
                    self.my_pose_received = True
    """
    ------------------------------------------------------------
    Function: crate_pose_cb

    Input:
        msg (Poses2D) :
            Message containing detected crate poses.

    Output:
        None

    Logic Explanation:

        1. Iterate through detected crates and filter using
           ALLOWED_CRATE_IDS.

        2. Maintain a temporary pose dictionary to track
           currently visible crates.

        3. If a crate is already marked as placed, skip it.

        4. Check whether a crate lies inside any allocator
           drop zone (within threshold distance). If so,
           mark it as 'PLACED' and update tracking sets.

        5. For newly detected crates:
           - Determine color and target zone.
           - Initialize crate_data entry as 'UNASSIGNED'.

        6. For known crates still unassigned, update their pose.

        7. Replace all_crate_poses with latest visible data
           to maintain accurate obstacle tracking.

        8. If this robot has an assigned crate, update its
           local crate_pose reference when visible.

        Example:
            Triggered automatically by '/crate_pose' topic.
    """
    def crate_pose_cb(self, msg):
        current_visible_crates = set()
        temp_poses = {}

        for crate in msg.poses:
            if crate.id not in self.ALLOWED_CRATE_IDS:
                continue
            current_visible_crates.add(crate.id)
            temp_poses[crate.id] = [crate.x, crate.y, crate.w]

            if crate.id in self.placed_crate_ids:
                continue

            is_in_drop_zone = False
            for zone_center in self.allocator_drop_zones.values():
                dist_to_zone = math.dist(zone_center, (crate.x, crate.y))
                if dist_to_zone < 150.0:
                    is_in_drop_zone = True
                    break

            if is_in_drop_zone:
                if crate.id not in self.placed_crate_ids:
                    self.get_logger().info(f"📦 Crate {crate.id} detected in drop zone. Marking as PLACED.")
                    self.placed_crate_ids.add(crate.id)
                    if crate.id in self.crate_data:
                        self.crate_data[crate.id]['state'] = 'PLACED'
                continue

            if crate.id not in self.crate_data:
                color, zone = self.get_crate_color_and_zone(crate.id)
                if color == 'UNKNOWN':
                    continue
                self.crate_data[crate.id] = {
                    'pose': [crate.x, crate.y, crate.w],
                    'state': 'UNASSIGNED',
                    'color': color,
                    'zone': zone
                }
                self.get_logger().info(f"👀 New Crate {crate.id} ({color}) detected at pickup.", throttle_duration_sec=5.0)
            else:
                if self.crate_data[crate.id]['state'] == 'UNASSIGNED':
                    self.crate_data[crate.id]['pose'] = [crate.x, crate.y, crate.w]

        self.all_crate_poses = temp_poses

        if self.assigned_crate_id is not None:
            if self.assigned_crate_id in self.all_crate_poses:
                self.crate_pose = self.all_crate_poses[self.assigned_crate_id]
                self.get_logger().info(f"🎯 Acquired/Updated pose for assigned crate {self.assigned_crate_id}", throttle_duration_sec=2.0)
        """
    ------------------------------------------------------------
    Function: task_cb

    Input:
        msg (String) :
            Task assignment message from '/task_assignments'.
            Format:
            robot_name,crate_id,path,drop_x:drop_y,layer,zone,slot_idx

    Output:
        None

    Logic Explanation:

        1. Parse the incoming message and extract robot name,
           crate ID, planned path, drop slot, stacking layer,
           zone name, and slot index (if provided).

        2. Update active_zone_tasks to reflect that a
           specific slot is being transported by a robot.

        3. If the assignment is for this robot:
           - Store crate ID and stacking layer.
           - Parse and store the planned navigation path.
           - Store the assigned drop goal coordinates.
           - Validate crate data availability.

        4. Update crate-related metadata such as color,
           model name, and slot index.

        5. Reset PID controllers and navigation timers
           to prepare for a new task cycle.

        6. Transition state machine to
           'NAVIGATE_TO_CRATE' and restore maximum speed.

        7. Handle parsing errors safely and ignore
           assignments not intended for this robot.

        8. If already busy (state not IDLE), ignore
           new task messages to prevent conflicts.

    Example:
        Triggered automatically by '/task_assignments'.
    """ 
    def task_cb(self, msg):

        try:
            parts = msg.data.split(',')
            robot_name = parts[0]
            self.get_logger().info(f"📍 the message :{msg}")## got the message

            if len(parts) > 6:
                simple_zone = parts[5]
                slot_idx = int(parts[6])
                self.active_zone_tasks.setdefault(simple_zone, {})[slot_idx] = robot_name

            if robot_name == self.robot_name:
                self.assigned_crate_id = int(parts[1])
                path_str = parts[2] if len(parts) > 2 else ""
                drop_coord_str = parts[3] if len(parts) > 3 else ""
                self.get_logger().info(f"assigned crate ID: {self.assigned_crate_id}, path: {path_str}, drop slot: {drop_coord_str}")

                if path_str:
                    self.path = [(float(x),float(y)) for x,y in [p.split(':') for p in path_str.split('|')]]
                    self.get_logger().info(f"📍 My Planned Path Coordinates through task_cb: {self.path}")
                    self.get_logger().info(f"📍 My drop slot : {drop_coord_str}")
                else:
                    self.path = []
                self.path_index = 0

                self.get_logger().info(f"📍 My drop slot : {parts[3]}")


                if len(parts) > 4:
                    self.assigned_layer = int(parts[4])
                    self.get_logger().info(f"🏗️ Stacking Task: Layer {self.assigned_layer}")

                if drop_coord_str:
                    dx, dy = drop_coord_str.split(':')
                    self.assigned_drop_goal = (float(dx), float(dy), 0.0)
                    self.get_logger().info(f"📥 Received specific Drop Slot: {self.assigned_drop_goal}")
                    self.get_logger().info(
                        f"🧭 SLOT ASSIGNED | Robot={self.robot_name} "
                        f"Slot=({self.assigned_drop_goal[0]:.1f},{self.assigned_drop_goal[1]:.1f})"
                    )

                else:
                    self.assigned_drop_goal = None

                if self.assigned_crate_id not in self.crate_data:
                    self.get_logger().error(f"❌ ERROR: Assigned crate {self.assigned_crate_id} but I have no data for it!")
                    self.assigned_crate_id = None
                    return
                
                if len(parts) > 6:
                    self.assigned_slot_idx = slot_idx

                color, _ = self.get_crate_color_and_zone(self.assigned_crate_id)
                self.assigned_crate_color = color

                self.crate_model_name = f"crate_{self.assigned_crate_color.lower()}_{self.assigned_crate_id}"

                self.pid_x.reset()
                self.pid_y.reset()
                self.pid_theta.reset()
                self.idle_start_time = None
                self.last_progress_time = self.get_clock().now()

                self.max_vel = 600.0 # Reset to max speed for the new task
                self.state = 'NAVIGATE_TO_CRATE'

                self.get_logger().info(f"✅ NEW TASK: Go to Crate {self.assigned_crate_id} ({self.assigned_crate_color}). Slot Rank: {self.assigned_slot_idx}")

        except Exception as e:
            self.get_logger().error(f"Failed to parse task assignment because I am not the robot which is supposed to accept it: {e}")

        if self.state != 'IDLE':
            self.get_logger().warn(f'Ignoring task msg (I am busy in state: {self.state})', throttle_duration_sec=5.0)
            return
        """
    ------------------------------------------------------------
    Function: control_cb

    Input:
        None (Timer-based callback)

    Output:
        None

    Logic Explanation:

        1. Ensure current robot pose is available.
           If not, wait until localization is acquired.

        2. Compute time delta (dt) between control cycles
           for stable PID and motion updates.

        3. Log current finite state machine (FSM) state
           with throttling for readability.

        4. Execute behavior based on current state:

           - IDLE:
             Stop the robot and monitor idle duration.
             If idle exceeds threshold, transition to
             'RETURN' to move back to docking zone.

           - NAVIGATE_TO_CRATE:
             Execute path-following toward assigned crate.

           - PICK:
             Perform crate pickup sequence.

           - TRANSPORT:
             Navigate toward assigned drop slot.

           - PLACE:
             Execute stacking/placement routine.

           - RETURN:
             Navigate back to docking zone.

        5. Maintains centralized state-driven control
           for navigation, manipulation, and coordination.

    Example:
        Triggered periodically by control timer.
    """
    def control_cb(self):
        if self.current_pose is None:
            self.get_logger().warn('⏳ Waiting for my pose...', throttle_duration_sec=2.0)
            return

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0:
            dt = 0.03
        self.last_time = now

        self.get_logger().info(f'--- STATE: {self.state} ---', throttle_duration_sec=2.0)

        if self.state == 'IDLE':
            self.stop_robot()
            if self.idle_start_time is None:
                self.idle_start_time = self.get_clock().now()

            idle_duration_sec = (self.get_clock().now() - self.idle_start_time).nanoseconds / 1e9
            if idle_duration_sec > 2.0:
                self.get_logger().info("Idle for 1s, no new task. Returning to dock.")
                self.state = 'RETURN'
                self.idle_start_time = None
            return

        elif self.state == 'NAVIGATE_TO_CRATE':
            self.navigate_to_crate(dt)
        elif self.state == 'PICK':
            self.pick_crate()
        elif self.state == 'TRANSPORT':
            self.transport_crate(dt)
        elif self.state == 'PLACE':
            self.place_crate()
        elif self.state == 'RETURN':
            self.return_to_dock(dt)

    # ---------------- State Machine Functions ----------------


    """
    ------------------------------------------------------------
    Function: replan

    Input:
        None

    Output:
        None

    Logic Explanation:

        1. Triggered when deadlock, stagnation, or path
           obstruction is detected.

        2. Recompute obstacle set while excluding the
           currently assigned crate to avoid self-blocking.

        3. Determine appropriate goal based on current state:
           - NAVIGATE_TO_CRATE → crate position
           - TRANSPORT → drop goal
           - RETURN → docking goal
           If no valid goal exists, exit safely.

        4. Invoke planner to compute a new path from
           current pose to selected goal.

        5. Log replanning summary including pose,
           goal, and number of path points.

        6. Reset path index and progress timer to
           resume navigation with updated plan.

    Example:
        self.replan()
    """
    def replan(self):
        self.get_logger().warn("🚨 Reactive Replanning Initiated due to deadlock")
        obstacles = self.get_all_obstacles(exclude_crate_id = self.assigned_crate_id)

        

        if self.state == 'NAVIGATE_TO_CRATE':
             goal_pt = self.crate_pose[:2] if self.crate_pose else self.current_pose[:2]
        elif self.state == 'TRANSPORT' and self.current_goal:
             goal_pt = self.current_goal[:2]
        elif self.state == 'RETURN' and self.current_goal:
             goal_pt = self.current_goal[:2]
        else:
             return

        self.path = self.planner.plan(self.current_pose[:2], goal_pt, obstacles=obstacles)

        self.get_logger().warn(
            f"🔄 REPLAN | {self.robot_name}(ID {self.robot_id}) "
            f"Pose=({self.current_pose[0]:.1f},{self.current_pose[1]:.1f}) "
            f"Goal=({goal_pt[0]:.1f},{goal_pt[1]:.1f}) "
            f"Pts={len(self.path)} "
            f"Path={self.path}"
        )
        self.path_index = 0
        self.last_progress_time = self.get_clock().now()


    """
    ------------------------------------------------------------
    Function: navigate_to_crate

    Input:
        dt (float) :
            Control loop time delta.

    Output:
        None

    Logic Explanation:

        1. If crate pose is available, monitor distance and
           switch from waypoint-following to direct alignment
           when within a close threshold.

           when within a close threshold.

        2. Dynamically adjust maximum velocity:
           - Slow down near crate.
           - Reduce speed inside inflated drop zones.
           - Use full speed in open space.

        3. If a planned path exists, follow waypoints sequentially:
           - Log navigation debug information.
           - Move toward current waypoint.
           - Track progress and detect stagnation.
           - Advance to next waypoint upon reaching target.

        4. If no waypoints remain, perform final approach:
           - Wait if crate pose is unavailable.
           - Compute distance and angular alignment to crate.
           - Determine alignment using arm offset and tolerance.

        5. If IR sensor confirms crate detection while aligned,
           stop robot and transition to 'PICK' state.

        6. Otherwise, execute direct navigation toward crate
           for fine alignment and controlled approach.

        7. Maintains smooth transition from global path
           following to precision pickup alignment.

    Example:
        Triggered by control_cb during 'NAVIGATE_TO_CRATE' state.
    """
    def navigate_to_crate(self, dt):
        """Navigate close to the assigned crate - FIXED VERSION"""

        # 1. Only switch to direct mode when VERY close 
        FORCE_ALIGNMENT_DIST = 160.0
        if self.crate_pose:
            # If we are close, DELETE the path to force the code to skip the waypoint block below and fall through to navigate_to_crate_direct
            dist_to_crate = math.dist(self.current_pose[:2], self.crate_pose[:2])
            if dist_to_crate < FORCE_ALIGNMENT_DIST:
                if self.path:
                    self.get_logger().info(f"🎯 Entering Alignment Zone ({dist_to_crate:.1f}mm). Dropping path.")
                    self.path = []

        # 2. Final Approach Logic
            dist_to_crate = math.dist(self.current_pose[:2], self.crate_pose[:2])
            if dist_to_crate < 230.0:
                self.max_vel = 120.0
            elif self.is_in_inflated_drop_zone(self.current_pose[:2]):
                self.max_vel = 150.0
            else:
                self.max_vel = 600.0


        # 3. Follow waypoints first
        if self.path and self.path_index < len(self.path):
            goal = self.path[self.path_index]

            self.log_navigation_debug(
                state_name="NAVIGATE_TO_CRATE (WAYPOINT)",
                next_wp=goal,
                goal=(self.crate_pose[0], self.crate_pose[1]) if self.crate_pose else None
            )
            is_approach_pt = (self.path_index >= len(self.path) - 2)
            reached = self.navigate_to_goal((goal[0], goal[1], 0.0), dt, use_relaxed_tolerance=True, is_waypoint=True,is_last_waypoint=is_approach_pt)

            now = self.get_clock().now()
            curr_pos = self.current_pose[:2]
            goal_dist = math.dist(curr_pos, goal)

            if goal_dist < self.last_dist_to_goal - 1.0:
                 self.last_progress_time = now
                 self.last_dist_to_goal = goal_dist
            elif (now - self.last_progress_time).nanoseconds/1e9 > 5.0:
                pass

            if reached:
                self.path_index += 1
                self.last_progress_time = now
                self.last_dist_to_goal = float('inf')
                self.get_logger().info(f"✅ Waypoint {self.path_index}/{len(self.path)} reached")
            return

        # 4. Crate Pose Check 
        if self.crate_pose is None:
            self.get_logger().warn(f'⏳ Waiting for assigned crate {self.assigned_crate_id} to be visible...', throttle_duration_sec=2.0)
            self.stop_robot()
            return



        crate_x, crate_y = self.crate_pose[0], self.crate_pose[1]
        curr_x, curr_y, curr_theta_arena = self.current_pose
        dist_to_crate = math.sqrt((crate_x - curr_x)**2 + (crate_y - curr_y)**2)
        desired_angle_math = math.atan2(-(crate_y - curr_y), crate_x - curr_x)
        curr_theta_math = -curr_theta_arena
        target_marker_angle_math = self.normalize_angle(desired_angle_math - self.arm_offset_rad)
        raw_error_theta = target_marker_angle_math - curr_theta_math
        error_theta_math = math.atan2(math.sin(raw_error_theta), math.cos(raw_error_theta))
        alignment_tolerance_rad = math.radians(3.0)
        is_aligned = abs(error_theta_math) < alignment_tolerance_rad
        is_close_enough = dist_to_crate < self.pickup_distance



        #  Hardware IR Check and Alignment Check
        if self.ir_detected and is_aligned:
            self.stable_count += 1
            self.stop_robot()
            if self.stable_count >= 15:
                self.stop_robot()
                self.get_logger().info(f'🚨 IR TRIGGER: and tolerance_angle {math.degrees(error_theta_math)}° Crate detected! Transitioning to PICK.')
                self.state = 'PICK'
                self.stable_count = 0
                return



        self.log_navigation_debug(
            state_name="NAVIGATE_TO_CRATE (FINAL)",
            next_wp=None,
            goal=(crate_x, crate_y)
        )

        self.navigate_to_crate_direct(dt)


    """
    ------------------------------------------------------------
    Function: pick_crate

    Input:
        None

    Output:
        None

    Output:
        None

    Logic Explanation:

        1. Verify IR detection:
           - If IR contact is lost, transition back to
             'NAVIGATE_TO_CRATE' for re-alignment.

        2. Validate crate data:
           - If crate pose or assignment is missing,
             abort and return to 'IDLE'.

        3. Stop robot and stabilize before manipulation.

        4. Perform fine distance correction:
           - Compute distance error relative to pickup distance.
           - Strafe left/right to achieve proper alignment.
           - Limit correction duration for safety.

        5. Move arm to pickup configuration.

        6. Activate electromagnetic gripper:
           - If successful, lift crate to secure position.
           - Reset PID controllers.
           - Transition to 'TRANSPORT' state.

        7. If gripper activation fails:
           - Return arm to neutral.
           - Reset PID and transition to 'IDLE'.

        8. Ensures controlled pickup sequence combining
           IR alignment, distance correction, arm motion,
           and gripper actuation.

    Example:
        Triggered by state machine in 'PICK' state.
    """
    # def pick_crate(self):
    #     """Pick up the crate using arm and MQTT gripper."""

    #     if not self.ir_detected:
    #          self.get_logger().warn('⚠️ IR lost contact! Re-aligning...')
    #          self.state = 'NAVIGATE_TO_CRATE'
    #          return

    #     if self.crate_pose is None or self.assigned_crate_id is None:
    #         self.get_logger().error('❌ Lost crate info during PICK! Returning to IDLE.')
    #         self.state = 'IDLE'
    #         return

    #     self.stop_robot()
    #     time.sleep(0.2)

    #     # 1. Correct Distance
    #     dist_to_crate = math.dist(self.current_pose[:2], self.crate_pose[:2])
    #     dist_error = self.pickup_distance - dist_to_crate

    #     if abs(dist_error) > 50.0:
    #         move_speed = 100.0
    #         if dist_error < 0:
    #             vy_robot_cmd = -move_speed
    #             self.get_logger().info('Correcting distance: Too far, strafing RIGHT')
    #         else:
    #             vy_robot_cmd = move_speed
    #             self.get_logger().info('Correcting distance: Too close, strafing LEFT')

    #         # Calculate time needed to move
    #         move_time = abs(dist_error) / move_speed
    #         move_time = min(move_time, 0.1)
    #         self.get_logger().info(f'🚗 Correcting distance. Error={dist_error:.1f}mm. Moving for {move_time:.2f}s...')

    #         wheel_vels = self.body_to_wheel_velocities(0.0, vy_robot_cmd, 0.0)
    #         self.publish_wheel_velocities(wheel_vels)

    #         self.stop_robot()

    #     # 2. Move Arm
    #     self.get_logger().info('🦾 Step 1: Moving arm to pickup position...')
    #     self.move_arm(self.arm_base_pickup, self.arm_elbow_pickup)

    #     # 3. Attach
    #     self.get_logger().info(f'🔗 Step 2: Activating gripper via MQTT...')
    #     if self.set_gripper(True):
    #         self.publish_wheel_velocities([0.0,0.0,0.0])# to execute the solenoid command
    #         self.get_logger().info('✅ Gripper Active! Lifting...')
    #         time.sleep(0.2)
    #         self.get_logger().info('⬆️ Lifting Secured Crate...')
    #         self.move_arm(100.0, 46.0)
    #         time.sleep(0.2)

    #         self.crate_picked = True
    #         self.stop_robot()  #  resetting pid values after pickup
    #         self.pid_x.reset()
    #         self.pid_y.reset()
    #         self.pid_theta.reset()
    #         self.state = 'TRANSPORT'
    #         self.path = []
    #         self.get_logger().info('🚚 Transporting crate to drop zone...')
    #     else:
    #         self.get_logger().error('❌ Failed to activate gripper! Returning to IDLE.')
    #         self.move_arm(self.arm_base_neutral, self.arm_elbow_neutral)
    #         self.state = 'IDLE'

    #         self.stop_robot()# resetting pid values after state transition to idle
    #         self.pid_x.reset()
    #         self.pid_y.reset()
    #         self.pid_theta.reset()
    def pick_crate(self):
        """Pick up the crate using arm and MQTT gripper. Includes precision angle alignment."""

        if not self.ir_detected:
             self.get_logger().warn('⚠️ IR lost contact! Re-aligning...')
             self.state = 'NAVIGATE_TO_CRATE'
             return

        if self.crate_pose is None or self.assigned_crate_id is None:
            self.get_logger().error('❌ Lost crate info during PICK! Returning to IDLE.')
            self.state = 'IDLE'
            return

        self.stop_robot()
        time.sleep(0.2)

        # ---------------------------------------------------------
        # 1. Correct Distance
        # ---------------------------------------------------------
        # We use the LAST KNOWN crate_pose because the arm blocks the ArUco marker now.
        crate_x, crate_y = self.crate_pose[0], self.crate_pose[1]
        
        dist_to_crate = math.dist(self.current_pose[:2], (crate_x, crate_y))
        dist_error = self.pickup_distance - dist_to_crate

        if abs(dist_error) > 50.0:
            move_speed = 100.0
            if dist_error < 0:
                vy_robot_cmd = -move_speed
                self.get_logger().info('Correcting distance: Too far, strafing RIGHT')
            else:
                vy_robot_cmd = move_speed
                self.get_logger().info('Correcting distance: Too close, strafing LEFT')

            move_time = abs(dist_error) / move_speed
            move_time = min(move_time, 0.1)
            self.get_logger().info(f'🚗 Correcting distance. Error={dist_error:.1f}mm. Moving for {move_time:.2f}s...')

            wheel_vels = self.body_to_wheel_velocities(0.0, vy_robot_cmd, 0.0)
            self.publish_wheel_velocities(wheel_vels)
            
            # Allow the motors to actually execute the correction
            time.sleep(move_time)
            self.stop_robot()
            time.sleep(0.1)

        # ---------------------------------------------------------
        # 1.5. Precision Angle Verification (Using Live Robot Pose vs Static Crate Pose)
        # ---------------------------------------------------------
        self.get_logger().info('🔄 Step 1.5: Final precision angle verification...')
        fine_tune_start = self.get_clock().now()
        
        while True:
            # Safety timeout (4.0 seconds) to prevent infinite loop
            if (self.get_clock().now() - fine_tune_start).nanoseconds / 1e9 > 4.0:
                self.get_logger().warn('⚠️ Angle fine-tuning timed out. Proceeding with pickup.')
                break
                
            # Current robot pose updates live from the overhead camera via listener_group thread
            curr_x, curr_y, curr_theta_arena = self.current_pose
            
            # Calculate angle to the fixed (last known) crate position
            error_x = crate_x - curr_x
            error_y = crate_y - curr_y
            desired_angle_math = math.atan2(-error_y, error_x)
            
            curr_theta_math = -curr_theta_arena
            target_marker_angle_math = self.normalize_angle(desired_angle_math - self.arm_offset_rad)
            raw_error_theta = target_marker_angle_math - curr_theta_math
            error_theta_math = math.atan2(math.sin(raw_error_theta), math.cos(raw_error_theta))
            
            deg_error = math.degrees(error_theta_math)

            # Tight tolerance: 2.0 degree margin for error
            if abs(deg_error) <= 1.0:
                self.get_logger().info('✅ Angle perfectly aligned for pickup!')
                self.stop_robot()
                break
                
            # Rotate slowly for precision locking
            FINE_ROT_SPEED = 0.2 # Adjust this if it rotates too fast/slow
            if 0 <= deg_error < 150.0: 
                omega = -FINE_ROT_SPEED
            elif deg_error > -150.0 and deg_error < 0: 
                omega = FINE_ROT_SPEED
            else: 
                omega = FINE_ROT_SPEED

            wheel_vels = self.body_to_wheel_velocities(0.0, 0.0, omega)
            self.publish_wheel_velocities(wheel_vels)
            
            # Yield briefly to let physics and the listener thread catch up with new poses
            time.sleep(0.05) 
            
        self.stop_robot()
        time.sleep(0.1)

        # ---------------------------------------------------------
        # 2. Move Arm
        # ---------------------------------------------------------
        self.get_logger().info('🦾 Step 2: Moving arm to pickup position...')
        self.move_arm(self.arm_base_pickup, self.arm_elbow_pickup)

        # ---------------------------------------------------------
        # 3. Attach
        # ---------------------------------------------------------
        self.get_logger().info(f'🔗 Step 3: Activating gripper via MQTT...')
        if self.set_gripper(True):
            self.publish_wheel_velocities([0.0,0.0,0.0]) # to execute the solenoid command
            self.get_logger().info('✅ Gripper Active! Lifting...')
            time.sleep(0.2)
            self.get_logger().info('⬆️ Lifting Secured Crate...')
            self.move_arm(100.0, 46.0)
            time.sleep(0.2)

            self.crate_picked = True
            self.stop_robot()  # resetting pid values after pickup
            self.pid_x.reset()
            self.pid_y.reset()
            self.pid_theta.reset()
            self.state = 'TRANSPORT'
            self.path = []
            self.get_logger().info('🚚 Transporting crate to drop zone...')
        else:
            self.get_logger().error('❌ Failed to activate gripper! Returning to IDLE.')
            self.move_arm(self.arm_base_neutral, self.arm_elbow_neutral)
            self.state = 'IDLE'

            self.stop_robot() # resetting pid values after state transition to idle
            self.pid_x.reset()
            self.pid_y.reset()
            self.pid_theta.reset()

    """
    ------------------------------------------------------------
    Function: transport_crate

    Input:
        dt (float) :
            Control loop time delta.

    Output:
        None

    Logic Explanation:

        1. Determine target drop zone and compute this robot’s
           priority rank based on assigned slot index within
           active_zone_tasks.

        2. Rank-Based Routing:
           - Rank 0 → Primary robot proceeds directly to drop slot.
           - Rank ≥1 → Navigate to nearest available intermediate
             (INT) waiting point and lock it.

        3. Manage zone locking:
           - Lock/unlock INT points dynamically.
           - Ensure only one robot enters drop zone at a time.
           - Prevent lower-rank robots from interfering.

        4. Compute final transport goal:
           - If assigned drop slot exists, offset goal using
             layer-specific reach configuration.
           - Otherwise fallback to zone center.

        5. Perform throttled path planning:
           - Aggregate obstacles.
           - Generate Theta* path.
           - Retry periodically if blocked.

        6. If waiting at INT point:
           - Stop precisely and hold position until
             promotion to Rank 0.

        7. Adjust speed dynamically:
           - Slow near drop.
           - Reduce speed inside inflated zones.
           - Use full speed in open space.

        8. Follow planned waypoints sequentially.

        9. Execute final alignment:
           - Trigger arm preparation at 100mm threshold.
           - Perform position correction using PID.
           - Perform fine orientation alignment.

        10. Upon satisfying distance and angle tolerances:
            - Stop robot.
            - Transition to 'PLACE' state.

        11. Convert world-frame velocities to robot-frame
            commands and publish wheel velocities.

    Example:
        Triggered during 'TRANSPORT' state by control_cb.
    """
    def transport_crate(self, dt):
        """Transport utilizing dynamic Rank-Based Queueing with Multiple INT points."""
        # Lazy Init Timer
        if not hasattr(self, 'last_replan_time'):
            self.last_replan_time = rclpy.time.Time(seconds=0, clock_type=self.get_clock().clock_type)#rclpy.time.Time(seconds=0, nanoseconds=0)#self.get_clock().now()

        # 1. Identify Zone
        target_zone_id = self.get_target_zone_id()
        # 2. Zone Gatekeeper
        if not target_zone_id: return

        # ---------------- PRIORITY RANKING LOGIC ----------------
        my_slot_idx = getattr(self, 'assigned_slot_idx', 999)
        active_slots_in_my_zone = sorted(list(self.active_zone_tasks.get(target_zone_id, {}).keys()))

        if not active_slots_in_my_zone:
            my_rank = 0
        else:
            try: my_rank = active_slots_in_my_zone.index(my_slot_idx)
            except ValueError: my_rank = 0 

        # Track which specific INT point we locked (if any)
        locked_int_id = getattr(self, f'locked_int_{target_zone_id}', None)
        waiting_at_int = (locked_int_id is not None)

        # ---------------- ROUTING BASED ON RANK ----------------
        if my_rank == 0:
            # PRIMARY BOT -> Go directly to actual Drop Slot
            if waiting_at_int:
                self.get_logger().info(f"🔓 My turn! Moving from {locked_int_id} to {target_zone_id} Drop Slot.")
                self.publish_zone_update(locked_int_id, 'FREE')
                setattr(self, f'locked_int_{target_zone_id}', None)
                waiting_at_int = False
                locked_int_id = None
                self.path = [] # Force replan from INT point to final slot

            self.publish_zone_update(target_zone_id, 'OCCUPIED')
            self.current_goal = None
            exclude_zone_param = None  # i.e. do not allow entry into drop zone during inflation

        elif my_rank >= 1:
            # Using QUEUED BOTS logic-> Find and go to the NEAREST Intermediate Point
            if not locked_int_id:
                # We haven't picked an INT point yet. Calculate the nearest one.
                pts = self.intermediate_pts[target_zone_id]
                nearest_int_id = None
                min_dist = float('inf')
                curr_pos = self.current_pose[:2]
                
                for int_id, coords in pts.items():
                    d = math.dist(curr_pos, coords[:2])
                    if d < min_dist:
                        min_dist = d
                        nearest_int_id = int_id
            else:
                # Ones already picked and locked one, stick with it
                nearest_int_id = locked_int_id

            int_info = self.zone_locks.get(nearest_int_id, {'status': 'FREE', 'owner': None})

            # Check if the nearest one is free
            if int_info['status'] == 'FREE' or int_info['owner'] == self.robot_id:
                if int_info['status'] == 'FREE':
                    self.get_logger().info(f"🟡 {target_zone_id} is busy. Locking Nearest INT Point: {nearest_int_id}.")
                    self.publish_zone_update(nearest_int_id, 'OCCUPIED')
                    setattr(self, f'locked_int_{target_zone_id}', nearest_int_id)
                    locked_int_id = nearest_int_id
                
                self.current_goal = self.intermediate_pts[target_zone_id][nearest_int_id]
                waiting_at_int = True
                exclude_zone_param = None # Treat main zone as obstacle so we don't hit Rank 0 bot
            else:
                # The nearest INT point is taken. DO NOT look at the far one. Just hold position.
                self.stop_robot()
                self.get_logger().warn(f"🚦 Nearest INT pt ({nearest_int_id}) occupied. Rank {my_rank} holding at pickup.", throttle_duration_sec=2.0)
                return

        # ---------------- NAVIGATION EXECUTION ----------------
        # 3. Setup Goal
        if self.current_goal is None:
            if self.assigned_drop_goal is not None:
                slot_x, slot_y, _ = self.assigned_drop_goal
                config = self.layer_config.get(self.assigned_layer, self.layer_config[0])
                current_reach = config['reach']
                self.current_goal = (slot_x, slot_y - current_reach, 0.0)
            else:
                drop_zone_coords = self.drop_zones[self.assigned_crate_color]
                self.current_goal = (drop_zone_coords[0], drop_zone_coords[1], 0.0)

        curr_pos = self.current_pose[:2]
        dist_to_drop = math.dist(curr_pos, self.current_goal[:2])

        # Path planning with Throttle
        if not self.path and dist_to_drop > 100.0:
            now = self.get_clock().now()
           
            if (now - self.last_replan_time).nanoseconds / 1e9 < 2.5:
                self.stop_robot()
                return

            self.last_replan_time = now
            
            # We DO NOT exclude any zones. All walls remain fully solid.
            obstacles = self.get_all_obstacles(exclude_zone=None, skip_id=self.robot_id, exclude_crate_id=self.assigned_crate_id)

            new_path = self.planner.plan(self.current_pose[:2], self.current_goal[:2], obstacles=obstacles)


            if not new_path:
                self.get_logger().warn("❌ Path Blocked! Waiting...", throttle_duration_sec=1.0)
                self.stop_robot()
                return

            self.path = new_path
            self.path_index = 0
            self.get_logger().info(f"🗺️ [TRANSPORT] Path Generated ({len(self.path)} pts): {self.path}")

        # Halt precisely if arriving at the Intermediate Point
        if waiting_at_int:
            if dist_to_drop < 30.0:
                self.stop_robot()
                self.get_logger().info(f"📍 Resting at {locked_int_id}. Waiting for Rank 0 to finish...", throttle_duration_sec=2.0)
                return

        # Speed Adjustment
        if dist_to_drop < 110.0: self.max_vel = 200.0
        elif self.is_in_inflated_drop_zone(self.current_pose[:2]): self.max_vel = 300.0
        else: self.max_vel = 600.0

        # Follow Waypoints
        if self.path and self.path_index < len(self.path):
            goal = self.path[self.path_index]
            is_last = (self.path_index == len(self.path) - 1)
            if self.navigate_to_goal((goal[0], goal[1], 0.0), dt, use_relaxed_tolerance=True, is_waypoint=True, is_last_waypoint=is_last):
                self.path_index += 1
            return

        # Final Alignment (Force Alignment Logic)
        curr_x, curr_y, curr_theta_arena = self.current_pose
        goal_x, goal_y, goal_theta = self.current_goal
        dist_err = math.dist((curr_x, curr_y), (goal_x, goal_y))

        # --- Using 100mm ARM TRIGGER ---
        # Initialize the flag for arm movement
        if not hasattr(self, 'arm_prepared_for_drop'):
            self.arm_prepared_for_drop = False
            
        # Trigger exactly ONCE when crossing the 100mm boundary
        if dist_err <= 100.0 and not self.arm_prepared_for_drop:
            self.get_logger().info('🦾 100mm threshold reached! Moving arm.')
            self.move_arm(90.0, 70.0)
            self.arm_prepared_for_drop = True
        # -----------------------------

        curr_theta_math = -curr_theta_arena
        target_theta_math = goal_theta
        raw_angle_err = math.atan2(math.sin(target_theta_math - curr_theta_math), math.cos(target_theta_math - curr_theta_math))


        if dist_err > 5.0:
            vx_w = self.pid_x.compute(goal_x - curr_x, dt) * 0.5
            vy_w = self.pid_y.compute(goal_y - curr_y, dt) * 0.5
            omega = 0.0
        elif abs(raw_angle_err) > math.radians(1.5):
            vx_w = vy_w = 0.0

            if abs(math.degrees(raw_angle_err)) > 70.0:
                    FIXED_ROT_SPEED = 100.0

            else:
                    FIXED_ROT_SPEED = 100.0

            deg_error = math.degrees(raw_angle_err)
            if 0 <= deg_error < 150.0: omega = -FIXED_ROT_SPEED
            elif deg_error > -150.0 and deg_error < 0: omega = FIXED_ROT_SPEED
            else: omega = FIXED_ROT_SPEED
        else:
            self.stop_robot()
            self.state = 'PLACE'
            self.current_goal = None
            self.path = []
            return

        cos_t, sin_t = math.cos(curr_theta_arena), math.sin(curr_theta_arena)
        vx_r = vx_w * cos_t + vy_w * sin_t
        vy_r = -vx_w * sin_t + vy_w * cos_t

        self.publish_wheel_velocities(self.body_to_wheel_velocities(vx_r, vy_r, omega))

    """
    ------------------------------------------------------------
    Function: place_crate

    Input:
        None

    Output:
        None

    Logic Explanation:

        1. Determine stacking layer using assigned_layer.
           If layer is not calibrated, safely fall back
           to Layer 0 configuration.

        2. Retrieve calibrated arm base and elbow angles
           for the selected stacking height.

        3. Move arm to the precise drop configuration
           and allow stabilization time.

        4. Deactivate electromagnetic gripper to release crate.

        5. If release succeeds:
           - Lift arm back to neutral position.
           - Publish task completion message to leader.
           - Clear all crate-related state variables.
           - Unlock the occupied drop zone.
           - Reset arm preparation flag.
           - Transition to 'IDLE' state.

        6. If release fails:
           - Log error and allow retry mechanism.

        7. Ensures controlled placement, proper
           inter-robot synchronization, and safe
           state reset after stacking.

    Example:
        self.place_crate()
    """
    def place_crate(self):
        """Place the crate using calibrated layer parameters"""
        self.get_logger().info('📦 Lowering arm...')

        # 1. Determine which Layer config to use
        # Use 'getattr' to be safe in case assigned_layer isn't set
        layer = getattr(self, 'assigned_layer', 0)

        # Default to Layer 0 if the requested layer isn't in our config yet
        if layer in self.layer_config:
            config = self.layer_config[layer]
        else:
            self.get_logger().warn(f"⚠️ Layer {layer} not calibrated! Using Layer 0.")
            config = self.layer_config[0]

        target_base = config['base']
        target_elbow = config['elbow']

        self.get_logger().info(f"🔻 Placing at Layer {layer} | Base: {target_base}° | Elbow: {target_elbow}°")

        # 2. Move Arm to the calibrated drop position
        self.move_arm(target_base, target_elbow)
        time.sleep(1.0)

        # 3. Detach and Finish
        if self.set_gripper(False):
            self.get_logger().info('✅ Crate placed successfully (Gripper OFF)!')

            # Move arm UP to neutral to clear the stack
            self.move_arm(self.arm_base_neutral, self.arm_elbow_neutral)
            time.sleep(0.5)
            self.crate_picked = False

            self.get_logger().info(f"📤 Reporting task complete for crate {self.assigned_crate_id}")
            report_msg = String()
            report_msg.data = f"{self.robot_name},{self.assigned_crate_id}"
            self.task_complete_pub.publish(report_msg)

            self.assigned_crate_id = None
            self.assigned_crate_color = None
            self.assigned_drop_goal = None
            self.crate_model_name = None
            self.crate_pose = None

            # Unlock the Zone
            target_zone_id = self.get_target_zone_id()
            if target_zone_id:
                self.get_logger().info(f"🔓 UNLOCKING Zone {target_zone_id} (Switching to RETURN).")
                self.publish_zone_update(target_zone_id, 'FREE')

            self.arm_prepared_for_drop = False

            self.state = 'IDLE'
            self.get_logger().info('✅ Placement complete. Going IDLE.')
        else:
            self.get_logger().error('❌ Failed to detach crate! Trying again.')


    """
    ------------------------------------------------------------
    Function: return_to_dock

    Input:
        dt (float) :
            Control loop time delta.
            

    Output:
        None

    Logic Explanation:

        1. Initialize docking goal if not already set.
           Reset path and index for fresh planning.

        2. Plan return path once using full obstacle set
           (all robots, crates, and drop zones treated as blocked).

        3. Dynamically adjust speed:
           - Slow near docking zone.
           - Reduce speed inside inflated drop zones.
           - Use higher speed in open space.

        4. Phase A – Waypoint Navigation:
           Follow planned Theta* path until all waypoints
           are completed.

        5. Phase B – Precision Docking:
           a) Move linearly to dock coordinates while
              suppressing rotation.
           b) Once within tight distance tolerance,
              perform pure rotation to align with
              docking orientation.
           c) Use fixed rotation speed for stability.

        6. Upon satisfying position and angle tolerances:
           - Stop robot.
           - Clear path and goal.
           - Transition to 'IDLE'.

        7. Convert world-frame velocity commands into
           robot-frame velocities before publishing.

    Example:
        Triggered during 'RETURN' state by control_cb.
    """
    def return_to_dock(self, dt):
        """Return to dock with sequential XY movement followed by pure rotation."""

        # 1. Set goal
        if self.current_goal is None:
            self.current_goal = self.docking_zone
            self.path = []
            self.path_index = 0
            self.get_logger().info(f'🏠 STATE: RETURN → Goal: {self.current_goal}')

        # 2. Plan path ONCE (Obstacle avoidance with buffer)
        if not self.path:
            # Block all other robots and all drop zones  , as When returning, the robot should avoid every crate on the floor.
            obstacles = self.get_all_obstacles() # Blocks everything


            start = self.current_pose[:2]
            goal = self.current_goal[:2]

            self.path = self.planner.plan(start, goal, obstacles)
            self.path_index = 0
            self.get_logger().warn(f"🔁 RETURN PATH: {len(self.path)} points found.")
            self.get_logger().info(f"📍 My Planned Path Coordinates through return_to_dock: {self.path}")

                # --- DYNAMIC SPEED SWITCHING ---
        curr_pos = self.current_pose[:2]
        dist_to_dock = math.dist(curr_pos, self.current_goal[:2])

        if dist_to_dock < 120.0:
            self.max_vel = 400.0  # Slow down to enter the dock safely

        elif self.is_in_inflated_drop_zone(self.current_pose[:2]):
            self.max_vel = 400.0  # Slow down further in drop zone
        else:
            self.max_vel = 600.0  # High speed return
        # -------------------------------

        # 3. Phase A: Follow Waypoints (Move through safe path)
        if self.path and self.path_index < len(self.path):
            wp = self.path[self.path_index]
            reached = self.navigate_to_goal((wp[0], wp[1], 0.0), dt,
                                            use_relaxed_tolerance=True,
                                            is_waypoint=True)
            if reached:
                self.path_index += 1
            return

        # 4. Phase B: Final Precision Sequential Positioning (Like Transport Crate)
        curr_x, curr_y, curr_theta_arena = self.current_pose
        goal_x, goal_y, goal_theta = self.current_goal

        dist_err = math.dist((curr_x, curr_y), (goal_x, goal_y))

        # Dock orientation (convert to math convention)
        curr_theta_math = -curr_theta_arena
        target_theta_math = goal_theta
        raw_angle_err = math.atan2(math.sin(target_theta_math - curr_theta_math),
                                   math.cos(target_theta_math - curr_theta_math))

        self.get_logger().info(
            f"🔍 DOCK ALIGN | Dist: {dist_err:.1f}mm | Angle Err: {math.degrees(raw_angle_err):.1f}°",
            throttle_duration_sec=0.5
        )

        # STEP 1: Move to Dock Coordinate (Ignore rotation until very close)
        # We use a 25mm tolerance for the "Safe Linear Arrival"
        if dist_err > 15.0:
            vx_w = self.pid_x.compute(goal_x - curr_x, dt) * 0.6 # Moderate speed
            vy_w = self.pid_y.compute(goal_y - curr_y, dt) * 0.6
            omega = 0.0 # Force no rotation to keep linear path stable

        # STEP 2: Pure Rotation (Stop moving, just spin to dock heading)
        elif abs(raw_angle_err) > math.radians(20.0): # 25 degree tolerance for dock
            vx_w = 0.0
            vy_w = 0.0
            # FIXED_ROT_SPEED for quick rotation
            if abs(math.degrees(raw_angle_err)) > 70.0:
                FIXED_ROT_SPEED = 400.0

            else:
                    FIXED_ROT_SPEED = 400.0

            deg_error = math.degrees(raw_angle_err)
            self.get_logger().info(f"🔄 ROTATING | Speed: {FIXED_ROT_SPEED} | Angle Error: {deg_error:.1f}")

            # 1. Positive error less than 150 -> -omega
            if 0 <= deg_error < 150.0:
                omega = -FIXED_ROT_SPEED

            # 2. Negative error greater than -150 -> +omega
            elif deg_error > -150.0 and deg_error < 0:
                omega = FIXED_ROT_SPEED

            # 3. Everything else (Large angles > 150 or < -150) -> +omega
            else:
                omega = FIXED_ROT_SPEED
        # STEP 3: Complete
        else:
            self.get_logger().info('🏁 DOCK REACHED: Entering IDLE.')
            self.stop_robot()
            self.state = 'IDLE'
            self.current_goal = None
            self.path = []
            return

        # Coordinate Transform: World -> Robot Frame
        cos_t, sin_t = math.cos(curr_theta_arena), math.sin(curr_theta_arena)
        vx_r = vx_w * cos_t + vy_w * sin_t
        vy_r = -vx_w * sin_t + vy_w * cos_t

        wheel_vels = self.body_to_wheel_velocities(vx_r, vy_r, omega)
        self.publish_wheel_velocities(wheel_vels)

    # ---------------- Navigation & Collision Avoidance ----------------

    """
    ------------------------------------------------------------
    Function: calculate_repulsion

    Input:
        None

    Output:
        tuple (rep_x, rep_y, should_stop) :
            Repulsion velocity components in world frame
            and emergency stop flag.

    Logic Explanation:

        1. Initialize repulsion vectors and validate
           current robot pose.

        2. For each other robot:
           - Compute inter-robot distance.
           - If within avoidance radius, apply
             distance-based repulsion gain.
           - Increase gain progressively as distance
             decreases (soft → strong → emergency).
           - In emergency zone, optionally trigger
             replanning based on priority rules.
           - Accumulate normalized repulsion vector.

        3. For crates:
           - Ignore currently assigned crate during
             pickup/navigation.
           - Apply smaller avoidance radius and
             proportional repulsion force.

        4. Limit total repulsion magnitude to
           max_repulsion_vel to maintain stability.

        5. Log avoidance activity when repulsion
           exceeds minimal threshold.

        6. Return computed repulsion components
           for integration into motion command.

    Example:
        self.calculate_repulsion()
    """
    def calculate_repulsion(self):
        rep_x, rep_y = 0.0, 0.0
        should_stop = False
        if self.current_pose is None:
            return 0.0, 0.0, False

        curr_x, curr_y = self.current_pose[0], self.current_pose[1]

        for other_id, pose in self.all_robot_poses.items():
            if other_id == self.robot_id:
                continue

            dist = math.dist(self.current_pose[:2], pose[:2])

            if dist < self.avoid_robot_dist and dist > 0:
                
                if dist > 350:
                    # very far – negligible repulsion
                    effective_gain = self.robot_repulsion_gain * self.non_priority_gain_multiplier**1
                    self.get_logger().info(f"Robot {other_id} is far, dist={dist:.2f}, applying weak repulsion 1.", throttle_duration_sec=1.0)

                elif dist > 340:
                    # soft repulsion
                    effective_gain = self.robot_repulsion_gain * (self.non_priority_gain_multiplier**1.2)
                    self.get_logger().info(f"Robot {other_id} is moderately far, dist={dist:.2f}, applying normal repulsion=1.2", throttle_duration_sec=1.0)

                elif dist > 330:
                    # moderate repulsion
                    effective_gain = self.robot_repulsion_gain * (self.non_priority_gain_multiplier**1.93)
                    self.get_logger().info(f"Robot {other_id} is getting close, dist={dist:.2f}, applying increased repulsion multiplier 1.93", throttle_duration_sec=1.0)

                elif dist > 320:
                    # strong repulsion
                    effective_gain = self.robot_repulsion_gain * (self.non_priority_gain_multiplier**2.45)
                    self.get_logger().warn(f"Robot {other_id} is close, dist={dist:.2f}, applying strong repulsion multiplier 2.45", throttle_duration_sec=1.0)

                elif dist > 310:
                    # extreme repulsion
                    effective_gain = self.robot_repulsion_gain * (self.non_priority_gain_multiplier**2.72)
                    self.get_logger().warn(f"Robot {other_id} is very close, dist={dist:.2f}, applying extreme repulsion multiplier 2.72.", throttle_duration_sec=1.0)

                else:
                    # dist <= 310 mm → emergency zone
                    if other_id > self.robot_id :
                        if self.state == 'NAVIGATE_TO_CRATE':
                            effective_gain = self.robot_repulsion_gain * (self.non_priority_gain_multiplier**2.9)


                        else:
                            self.get_logger().warn(
                                f"EMERGENCY YIELD to bot {other_id}, dist={dist:.2f}",
                                throttle_duration_sec=1.0
                            )
                            self.replan()
                            # self.max_vel = 480.0# changed
                            effective_gain = self.robot_repulsion_gain * (self.non_priority_gain_multiplier**2.9)
                    else:
                        if self.state == 'NAVIGATE_TO_CRATE':
                            effective_gain = self.robot_repulsion_gain * (self.non_priority_gain_multiplier**2.9)

                            self.get_logger().warn(f"Robot {other_id} is in EMERGENCY ZONE, dist={dist:.2f}, applying max repulsion multiplier 2.9", throttle_duration_sec=1.0)
                        else:
                            self.replan()
                            # self.max_vel = 480.0# changed
                            # time.sleep(0.1)
                            effective_gain = self.robot_repulsion_gain * self.non_priority_gain_multiplier**2.9
                            self.get_logger().warn(f"Robot {other_id} is in EMERGENCY ZONE, dist={dist:.2f}, applying max repulsion multiplier 2.9", throttle_duration_sec=1.0)



                force = effective_gain * (1.0 - (dist / self.avoid_robot_dist))
                vec_x = curr_x - pose[0]
                vec_y = curr_y - pose[1]
                rep_x += force * vec_x / dist
                rep_y += force * vec_y / dist

        for id, pose in self.all_crate_poses.items():
            if id == self.assigned_crate_id and (self.state == 'NAVIGATE_TO_CRATE' or self.state == 'PICK'):
                continue

            dist = math.dist(self.current_pose[:2], pose[:2])
            if dist < self.avoid_crate_dist and dist > 0:
                force = self.crate_repulsion_gain * (1.0 - (dist / self.avoid_crate_dist))
                vec_x = curr_x - pose[0]
                vec_y = curr_y - pose[1]
                rep_x += force * vec_x / dist
                rep_y += force * vec_y / dist

        rep_mag = math.sqrt(rep_x**2 + rep_y**2)
        if rep_mag > self.max_repulsion_vel:
            rep_x = (rep_x / rep_mag) * self.max_repulsion_vel
            rep_y = (rep_y / rep_mag) * self.max_repulsion_vel

        if rep_mag > 0.1:
             self.get_logger().info(f'AVOIDANCE: Applying rep_x={rep_x:.2f}, rep_y={rep_y:.2f}', throttle_duration_sec=0.5)

        return rep_x, rep_y, False

    """
    ------------------------------------------------------------
    Function: navigate_to_crate_direct

    Input:
        dt (float) :
            Control loop time delta.

    Output:
        bool :
            Returns False (continues navigation loop).

    Logic Explanation:

        1. Validate crate pose availability. If missing,
           abort navigation safely.

        2. Perform boundary safety monitoring:
           - Detect repeated proximity to arena edges.
           - Trigger emergency stop if violation persists.

        3. Compute position and orientation errors:
           - Linear error toward crate center.
           - Angular error adjusted using arm offset.
           - Normalize angle for stable control.

        4. Compute attractive velocities using PID
           controllers (x, y, theta).

        5. Add repulsive potential field forces from
           nearby robots and crates.
           - Reduce attractive gain when strong
             repulsion is present.

        6. Convert world-frame velocities into
           robot-frame velocities.

        7. Apply logic gates for safe approach:

           a) Safety Threshold:
              If too close to crate, force small
              backward recovery motion.

           b) Far Distance:
              Use full PID-based motion.

           c) Close Alignment Zone:
              If angular error is large,
              suppress linear motion and
              prioritize pure rotation.

        8. Publish final wheel velocities and
           continue control cycle.

        9. Combines PID control, artificial
           potential fields, and safety overrides
           for precise crate alignment.

    Example:
        self.navigate_to_crate_direct(0.05)
    """
    def navigate_to_crate_direct(self, dt):
        """Navigate to crate with PID + Repulsion, plus a Safety Threshold override."""

        if self.crate_pose is None:
            self.get_logger().warn(f'navigate_to_crate_direct called but crate_pose is None!', throttle_duration_sec=2.0)
            return False

        crate_x, crate_y = self.crate_pose[0], self.crate_pose[1]
        curr_x, curr_y, curr_theta_arena = self.current_pose

        # Boundary safety check

        if curr_x < 100 or curr_x > 2338.4 or curr_y < 100 or curr_y > 2338.4:
            self.boundary_condition_met += 1
            if self.boundary_condition_met > 15:  # Require multiple consecutive detections to avoid false positives
                self.get_logger().error('🚨 Robot TOO CLOSE to boundary! EMERGENCY STOP!')
                self.stop_robot()
                self.state = 'COMPLETE'
                self.boundary_condition_met = 0
                return False

        error_x = crate_x - curr_x
        error_y = crate_y - curr_y
        desired_angle_math = math.atan2(-error_y, error_x)
        curr_theta_math = -curr_theta_arena
        target_marker_angle_math = self.normalize_angle(desired_angle_math - self.arm_offset_rad)
        raw_error_theta = target_marker_angle_math - curr_theta_math
        error_theta_math = math.atan2(math.sin(raw_error_theta), math.cos(raw_error_theta))
        position_error = math.sqrt(error_x**2 + error_y**2)


        # 1. using the above PID & Repulsion Logic

        vx_attractive = self.pid_x.compute(error_x, dt)
        vy_attractive = self.pid_y.compute(error_y, dt)

        vx_repulsive, vy_repulsive, should_stop = self.calculate_repulsion()

        rep_mag = math.sqrt(vx_repulsive**2 + vy_repulsive**2)
        if rep_mag > 10.0:
            vx_attractive *= 0.5
            vy_attractive *= 0.5

        vx_world = vx_attractive + vx_repulsive
        vy_world = vy_attractive + vy_repulsive

        omega = self.pid_theta.compute(error_theta_math, dt)

        # Transform to Robot Frame (Normal calculation)
        cos_theta = math.cos(curr_theta_arena)
        sin_theta = math.sin(curr_theta_arena)
        vx_robot = vx_world * cos_theta + vy_world * sin_theta
        vy_robot = -vx_world * sin_theta + vy_world * cos_theta

        # ---------------------------------------------------------
        # 2. Logic Gates
        # ---------------------------------------------------------
        ALIGNMENT_DISTANCE_MM = 155.0
        ANGLE_ALIGNMENT_TOLERANCE = math.radians(2.0)

        SAFE_THRESHOLD = self.pickup_distance - 5.0

        angle_too_large = abs(error_theta_math) > ANGLE_ALIGNMENT_TOLERANCE
 

        # Check Safety Threshold First
        if position_error < SAFE_THRESHOLD:
            self.get_logger().warn(f"⚠️ TOO CLOSE ({position_error:.1f}mm)! Backing up...", throttle_duration_sec=0.2)

            # Force reverse velocity relative to the crate vector
            RECOVERY_SPEED = -5.0


            rel_angle = math.atan2(error_y, error_x) - curr_theta_arena
            vx_robot = math.cos(rel_angle) * RECOVERY_SPEED
            vy_robot = math.sin(rel_angle) * RECOVERY_SPEED


        # Far away? Use full PID (calculated above)
        elif position_error >= ALIGNMENT_DISTANCE_MM:
            pass # Do nothing, use the vx_robot/vy_robot calculated in Step 1

        # Close range alignment logic
        elif position_error < ALIGNMENT_DISTANCE_MM:

            if angle_too_large:
                # Poor Alignment: Stop linear motion, prioritize PURE ROTATION


                self.get_logger().info(f'🔄 ALIGNING: Prioritizing rotation (Δθ={math.degrees(error_theta_math):.1f}°)', throttle_duration_sec=0.1)
                vx_robot = 0.0
                vy_robot = 0.0
            
                if abs(math.degrees(error_theta_math)) > 30.0:
                    FIXED_ROT_SPEED = 70.0

                else:
                    FIXED_ROT_SPEED = 0.2

                deg_error = math.degrees(error_theta_math)
                self.get_logger().info(f"🔄 ROTATING | Speed: {FIXED_ROT_SPEED} | Angle Error: {deg_error:.1f}")

                # 1. Positive error less than 150 -> -omega
                if 0 <= deg_error < 150.0:
                    omega = -FIXED_ROT_SPEED

                # 2. Negative error greater than -150 -> +omega
                elif deg_error > -150.0 and deg_error < 0:
                    omega = FIXED_ROT_SPEED

                # 3. Everything else (Large angles > 150 or < -150) -> +omega
                else:
                    omega = FIXED_ROT_SPEED

            else:
                pass


        # 4. Publish
        wheel_vels = self.body_to_wheel_velocities(vx_robot, vy_robot, omega)

        self.get_logger().info(
            f'Nav (Crate) Vels: Robot(vx={vx_robot:.1f}, vy={vy_robot:.1f}, ω={omega:.1f}) | '
            f'Dist={position_error:.1f}',
            throttle_duration_sec=0.5
        )

        self.publish_wheel_velocities(wheel_vels)
        return False


    """
    ------------------------------------------------------------
    Function: compute_path_velocity

    Input:
        curr (tuple) :
            Current position (x, y).
        target (tuple) :
            Target waypoint (x, y).
        max_speed (float) :
            Desired constant traversal speed.

    Output:
        tuple (vx, vy) :
            Velocity components toward the target.

    Logic Explanation:

        1. Compute displacement vector from current
           position to target.

        2. Calculate Euclidean distance to determine
           direction and stopping condition.

        3. If already very close to the waypoint,
           return zero velocity to avoid oscillation.

        4. Normalize the displacement vector to
           obtain a unit direction vector.

        5. Scale the unit vector by max_speed to
           maintain constant-speed motion along
           the path segment.

        6. Ensures smooth, direction-consistent
           waypoint following without acceleration spikes.

    Example:
        self.compute_path_velocity(curr, target, 400.0)
    """
    def compute_path_velocity(self, curr, target, max_speed):
        """FIXED: Smoother velocity profile for waypoint following"""
        dx = target[0] - curr[0]
        dy = target[1] - curr[1]
        dist = math.hypot(dx, dy)

        if dist < 1.0:
            return 0.0, 0.0

        # Normalize direction
        ux = dx / dist
        uy = dy / dist

        speed = max_speed
        return ux * speed, uy * speed

    """
    ------------------------------------------------------------
    Function: navigate_to_goal

    Input:
        goal (tuple) :
            Target pose (x, y, theta) in arena frame.
        dt (float) :
            Control loop time delta.
        use_relaxed_tolerance (bool) :
            Enables relaxed tolerances for intermediate goals.
        is_waypoint (bool) :
            Indicates whether the goal is a path waypoint.

    Output:
        bool :
            True if goal/waypoint reached, otherwise False.

    Logic Explanation:

        1. Compute position and orientation errors
           between current pose and target.

        2. Apply tolerance logic:
           - Waypoints use fixed small tolerance.
           - Final goals use configurable position
             and angle tolerances.
           - If within thresholds, stop robot,
             reset PIDs, and report success.

        3. Determine heading behavior:
           - Waypoints → move in direction of
             displacement (ignore final orientation).
           - Final goal → align with goal theta.

        4. Compute angular velocity using PID.

        5. Linear velocity:
           - Waypoints → constant-speed normalized motion.
           - Final goal → PID-based attractive motion.

        6. Compute repulsive velocities using
           artificial potential fields.

        7. If emergency stop is triggered by
           repulsion logic, halt immediately.

        8. Blend attractive and repulsive forces:
           - Reduce attractive gain when strong
             repulsion is active.
           - Combine both components.

        9. Transform world-frame velocities
           into robot-frame velocities.

        10. Publish wheel velocities and
            continue navigation loop.

        11. Provides unified low-level control
            for both path-following and final
            precision goal alignment.

    Example:
        self.navigate_to_goal(goal, dt)
    """
    def navigate_to_goal(self, goal, dt, use_relaxed_tolerance=False, is_waypoint=False,is_last_waypoint=False):
        """FIXED: Navigate to a specific goal with stricter waypoint following"""


        goal_x, goal_y, goal_theta_arena = goal
        curr_x, curr_y, curr_theta_arena = self.current_pose

        # 1. Error Calculations
        error_x = goal_x - curr_x
        error_y = goal_y - curr_y
        position_error = math.sqrt(error_x**2 + error_y**2)

        # 2. waypoint tolerance
        if is_last_waypoint:
            current_pos_tol = 5.0  # Tight tolerance for the optimal approach point
        elif is_waypoint:
            current_pos_tol = 20.0 # Relaxed tolerance for intermediate points
        else:
            current_pos_tol = self.position_tolerance if use_relaxed_tolerance else self.final_position_tolerance
        if position_error < current_pos_tol:
            if is_waypoint:
                return True

            goal_theta_math = -goal_theta_arena
            curr_theta_math = -curr_theta_arena
            raw_error_theta = math.atan2(math.sin(goal_theta_math - curr_theta_math), math.cos(goal_theta_math - curr_theta_math))

            if abs(raw_error_theta) < (math.radians(self.angle_tolerance) if use_relaxed_tolerance else self.final_angle_tolerance):
                self.stop_robot()
                self.pid_x.reset()
                self.pid_y.reset()
                self.pid_theta.reset()
                self.get_logger().info('🎉 Goal reached!')
                return True


        if is_waypoint:
            target_angle_math = math.atan2(-error_y, error_x)
        else:
            target_angle_math = -goal_theta_arena

        curr_theta_math = -curr_theta_arena
        raw_angle_err = math.atan2(math.sin(target_angle_math - curr_theta_math), math.cos(target_angle_math - curr_theta_math))

        omega = self.pid_theta.compute(raw_angle_err, dt)


        if is_waypoint:
            vx_world, vy_world = self.compute_path_velocity((curr_x, curr_y), (goal_x, goal_y), self.max_vel)

            omega = 0.0
        else:
            # 1. Compute Attractive Force from PID
            vx_world = self.pid_x.compute(error_x, dt)
            vy_world = self.pid_y.compute(error_y, dt)

            
        vx_rep, vy_rep, should_stop = self.calculate_repulsion()

        ## changed
        if should_stop:
            self.stop_robot()
            self.get_logger().info('🛑 EMERGENCY STOP triggered by repulsion')
            time.sleep(0.1)  # Brief pause to ensure the stop command takes effect
            return False

        # 3. --- IMPROVED BLENDING ---
        # If repulsion is active (magnitude > 10), we dampen the attractive PID
        # so the robot slows down to "listen" to the safety push.
        rep_mag = math.sqrt(vx_rep**2 + vy_rep**2)
        if rep_mag > 5.0:
            vx_world *= 0.8
            vy_world *= 0.8

        # 4. Combine Forces
        vx_world += vx_rep
        vy_world += vy_rep

        # 5. Coordinate Transformation
        cos_theta = math.cos(curr_theta_arena)
        sin_theta = math.sin(curr_theta_arena)
        vx_robot = vx_world * cos_theta + vy_world * sin_theta
        vy_robot = -vx_world * sin_theta + vy_world * cos_theta

        # 6. Publish
        wheel_vels = self.body_to_wheel_velocities(vx_robot, vy_robot, omega)
        self.publish_wheel_velocities(wheel_vels)

        return False

    # ---------------- Utility Functions ----------------

    """
    ------------------------------------------------------------
    Function: body_to_wheel_velocities

    Input:
        vx (float) :
            Linear velocity along robot X-axis.
        vy (float) :
            Linear velocity along robot Y-axis.
        omega (float) :
            Angular velocity about robot center.

    Output:
        list :
            Wheel velocity commands [v1, v2, v3].

    Logic Explanation:

        1. Adjust Y-axis sign to match the robot’s
           internal coordinate convention.

        2. Apply inverse kinematics for a 3-wheel
           holonomic (120° spaced) drive system.

        3. Compute individual wheel velocities
           using linear and rotational components.

        4. Add rotational contribution equally
           (scaled by rotation_gain) to all wheels.

        5. Return wheel velocity list for publishing
           to motor controller.

        6. Enables omnidirectional motion with
           independent control of translation
           and rotation.

    Example:
        self.body_to_wheel_velocities(100.0, 0.0, 0.0)
    """
    def body_to_wheel_velocities(self, vx, vy, omega):
        # Invert Y-axis for robot's coordinate system
        vy_corrected = -vy
        rotation_gain = 1.0 #0.185
        v1 = (-0.5 * vx) - (0.866 * vy_corrected) + (rotation_gain * omega)
        v2 = (-0.5 * vx) + (0.866 * vy_corrected) + (rotation_gain * omega)
        v3 = vx + (rotation_gain * omega)
        return [v1, v2, v3]

    """
    * Function Name: normalize_angle
    * Input: angle (float)
    * Output: float
    * Logic: Normalizes an angle to the range of [-pi, pi].
    * Example Call: self.normalize_angle(6.28)
    """
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    """
    * Function Name: stop_robot
    * Input: None
    * Output: None
    * Logic: Publishes zero velocity to all motors.
    * Example Call: self.stop_robot()
    """
    def stop_robot(self):
        self.get_logger().info('🛑 STOPPING ROBOT', throttle_duration_sec=1.0)
        self.publish_wheel_velocities([0.0, 0.0, 0.0])


    """
    ------------------------------------------------------------
    Function: move_arm

    Input:
        target_base (float) :
            Desired base servo angle (degrees).
        target_elbow (float) :
            Desired elbow servo angle (degrees).

    Output:
        None

    Logic Explanation:

        1. Compute angular differences between current
           and target joint positions.

        2. Determine required number of interpolation
           steps based on maximum joint movement and
           predefined step size.

        3. If movement is negligible, directly set
           final angles and exit.

        4. Calculate incremental angle updates for
           both joints to ensure synchronized motion.

        5. Iteratively update joint angles in small
           increments with short delays to reduce
           mechanical jerk and sudden torque spikes.

        6. Publish zero wheel velocity during motion
           to keep the robot base stationary while
           manipulating the arm.

        7. At completion, explicitly set final angles
           to eliminate rounding errors.

        8. Ensures smooth, stable arm movement and
           reduces stress on servos and structure.

    Example:
        self.move_arm(180.0, 90.0)
    """
    def move_arm(self, target_base, target_elbow):
        """
        Moves the arm smoothly to the target position to reduce jerk.
        Interpolates from current angles to target angles in small increments.
        """

        step_size = 4.0   # Degrees to move per step
        step_delay = 0.02 # Seconds to wait between steps

        self.get_logger().info(
            f'🦾 SMOOTH ARM: Base {self.current_arm_base:.1f}->{target_base}, '
            f'Elbow {self.current_arm_elbow:.1f}->{target_elbow}'
        )

        # 1. Calculate the total change needed
        delta_base = target_base - self.current_arm_base
        delta_elbow = target_elbow - self.current_arm_elbow

        # 2. Determine how many steps we need based on the largest movement
        max_delta = max(abs(delta_base), abs(delta_elbow))

        # If the move is tiny, just set it and exit
        if max_delta < 0.5:
            self.current_arm_base = float(target_base)
            self.current_arm_elbow = float(target_elbow)
            self.publish_wheel_velocities([0.0, 0.0, 0.0])
            return

        # Calculate number of steps required
        steps = int(max_delta / step_size)
        if steps == 0: steps = 1  # Avoid division by zero

        # Calculate the precise increment per step
        inc_base = delta_base / steps
        inc_elbow = delta_elbow / steps

        # 3. Execute the smooth motion loop
        for _ in range(steps):
            self.current_arm_base += inc_base
            self.current_arm_elbow += inc_elbow

            # Send the incremental command
            # Note: We send 0.0 wheel velocity to keep the base steady while lifting
            self.publish_wheel_velocities([0.0, 0.0, 0.0])

            # Short pause to let the servo physically move to this intermediate point
            time.sleep(step_delay)

        # 4. Ensure we land exactly on the target at the end
        self.current_arm_base = float(target_base)
        self.current_arm_elbow = float(target_elbow)
        self.publish_wheel_velocities([0.0, 0.0, 0.0])

            

    """
    * Function Name: set_gripper
    * Input: state (bool)
    * Output: bool
    * Logic: Calls a ROS service (SetBool) on the MQTT bridge to activate or deactivate the solenoid magnet.
    * Example Call: self.set_gripper(True)
    """
    def set_gripper(self, state: bool):
        """Call the /attach service on mqtt_bridge_node"""

        req = SetBool.Request()
        req.data = state

        self.get_logger().info(f'📞 Calling gripper service: {state}...')
        future = self.attach_client.call_async(req)

        if True:
            self.get_logger().info(f"✅ Gripper set to {'ON' if state else 'OFF'}")
            return True

    """
    * Function Name: publish_wheel_velocities
    * Input: wheel_vel (list)
    * Output: None
    * Logic: Packs individual wheel speeds and arm angles into a BotCmdArray and publishes to the motor topic.
    * Example Call: self.publish_wheel_velocities([100.0, 100.0, 100.0])
    """
    def publish_wheel_velocities(self, wheel_vel):
        """Publish wheel velocities and CURRENT arm state"""
        cmd = BotCmd()
        cmd.id = self.robot_id
        cmd.m1 = float(wheel_vel[0])
        cmd.m2 = float(wheel_vel[1])
        cmd.m3 = float(wheel_vel[2])
        cmd.base = self.current_arm_base
        cmd.elbow = self.current_arm_elbow

        msg = BotCmdArray()
        msg.cmds = [cmd]

        self.get_logger().info(
            f'Publishing Vels: m1={cmd.m1:.1f}, m2={cmd.m2:.1f}, m3={cmd.m3:.1f}',
            throttle_duration_sec=1.0
        )

        self.bot_cmd_pub.publish(msg)

# ---------------------- Main Function -------------------------------------

"""
------------------------------------------------------------
Function: main

Input:
    args :
        Optional ROS2 command-line arguments.

Output:
    None

Logic Explanation:

    1. Initialize the ROS2 communication layer using rclpy.

    2. Create an instance of the MultiHolonomicController node.

    3. Configure a MultiThreadedExecutor with two threads:
       - Thread 1 handles the main control loop (allows safe delays).
       - Thread 2 handles background callbacks such as
         pose updates, task assignments, and zone status messages.

    4. Add the controller node to the executor.

    5. Start spinning the executor to process callbacks
       and timers until shutdown.

    6. On interruption (e.g., Ctrl+C), gracefully destroy
       the node and shut down ROS2.

Example:
    main()
"""


def main(args=None):
    rclpy.init(args=args)
    controller = MultiHolonomicController()
    
    # Using 2 threads. 
    # Thread 1: Main Control loop ( to handle time.sleep safely)
    # Thread 2: Background listeners (Poses, task updates)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(controller)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()