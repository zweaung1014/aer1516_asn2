"""
Assignment #2 Template file
"""

import random
import math
import numpy as np

"""
Problem Statement
--------------------
Implement the Rapidly-Exploring Random Trees (RRT) planning algorithm
for the problem setup provided by the RRT_dubins_problem class.

INSTRUCTIONS
--------------------
1. The only file to be submitted is this file: rrt_planner.py. Your implementation
   can be tested by running dubins_path_problem.py (check the main function).
2. Read all class and function documentation in dubins_path_problem.py carefully.
   There are plenty of helper functions in the class to ease implementation.
3. Your solution must meet all the conditions specified below.
4. Below are some DOs and DONTs for this problem.

Conditions
-------------------
There are several conditions that must be satisfied for an acceptable solution.
These may or may not be verified by the auto-grading script.

1. The solution loop must not run for more than a certain number of random iterations
   (specified by the class member max_iter). This is mainly a safety
   measure to avoid time-out-related issues and will be set generously.
2. The planning function must return a list of nodes that represent a collision-free path
   from the start node to the goal node. The path states (path_x, path_y, path_yaw)
   specified by each node must define a Dubins-style path and traverse from node i-1 -> node i.
   (READ the documentation for the Node class to understand the terminology).
3. The returned path should have the start node at index 0 and the goal node at index -1.
   The parent node for node i from the list should be node i-1 from the list (i.e.,
   the path should be a valid, continuous list of connected nodes).
4. The node locations must not lie outside the map boundaries specified by
   RRT_dubins_problem.map_area.

DOs and DONTs
-------------------
1. DO NOT rename the file rrt_planner.py for submission.
2. DO NOT change the rrt_planner function signature.
3. DO NOT import anything other than what is already imported in this file.
4. YOU MAY write additional helper functions in this file to reduce code repetition,
   but these functions can only be used inside the rrt_planner function
   (since only the rrt_planner function will be imported for grading).
"""


def rrt_planner(rrt_dubins, display_map=False):
    """
    Execute RRT planning using Dubins-style paths. Make sure to populate the node_list.

    Inputs
    -------------
    rrt_dubins  - (RRT_dubins_problem) Class containing the planning
                  problem specification.
    display_map - (boolean) Flag for animation on or off (OPTIONAL).

    Outputs
    --------------
    (list of Node) This must be a valid list of connected nodes that form
                   a continuous path from the start node to the goal node.

    NOTE: In order for the rrt_dubins.draw_graph function to work properly, it is
    important to populate rrt_dubins.node_list with all valid RRT nodes.
    """
    # ----- Helper Functions -----
    def find_nearest_node(node_list, x, y):
        """Find the node in node_list closest to (x, y) using Euclidean distance."""
        min_dist = float('inf')
        nearest = None
        for node in node_list:
            dist = math.hypot(node.x - x, node.y - y)
            if dist < min_dist:
                min_dist = dist
                nearest = node
        return nearest

    def is_within_bounds(node):
        """Check if all path waypoints are within map boundaries."""
        if node is None:
            return False
        for x, y in zip(node.path_x, node.path_y):
            if x < rrt_dubins.x_lim[0] or x > rrt_dubins.x_lim[1]:
                return False
            if y < rrt_dubins.y_lim[0] or y > rrt_dubins.y_lim[1]:
                return False
        return True

    def build_path(goal_node):
        """Backtrack from goal_node to start via parent pointers, return path list."""
        path = []
        node = goal_node
        while node is not None:
            path.append(node)
            node = node.parent
        path.reverse()  # Reverse to get [start, ..., goal]
        return path

    # ----- Parameters -----
    goal_bias = 0.15       # Probability of sampling the goal pose
    goal_threshold = 2.0   # Distance threshold to attempt direct goal connection

    # Variable to store the final goal node when path is found
    final_goal_node = None

    # LOOP for max iterations
    for i in range(rrt_dubins.max_iter):

        # Generate a random vehicle state (x, y, yaw) with goal bias
        if random.random() < goal_bias:
            # Sample the goal pose directly
            rand_x = rrt_dubins.goal.x
            rand_y = rrt_dubins.goal.y
            rand_yaw = rrt_dubins.goal.yaw
        else:
            # Sample a random pose within map bounds
            rand_x = random.uniform(rrt_dubins.x_lim[0], rrt_dubins.x_lim[1])
            rand_y = random.uniform(rrt_dubins.y_lim[0], rrt_dubins.y_lim[1])
            rand_yaw = random.uniform(-math.pi, math.pi)

        # Find an existing node nearest to the random vehicle state
        nearest_node = find_nearest_node(rrt_dubins.node_list, rand_x, rand_y)

        # Create a temporary node for the random state and propagate
        rand_node = rrt_dubins.Node(rand_x, rand_y, rand_yaw)
        new_node = rrt_dubins.propagate(nearest_node, rand_node)

        # Check if the path between nearest node and random state has obstacle collision
        # Add the node to node_list if it is valid (collision-free and within bounds)
        if new_node is not None and rrt_dubins.check_collision(new_node) and is_within_bounds(new_node):
            rrt_dubins.node_list.append(new_node)  # Storing all valid nodes

            # Draw current view of the map
            # PRESS ESCAPE TO EXIT
            if display_map:
                rrt_dubins.draw_graph()

            # Check if new_node is close enough to the goal
            dist_to_goal = rrt_dubins.calc_dist_to_goal(new_node.x, new_node.y)
            if dist_to_goal < goal_threshold:
                # Attempt to connect directly to the goal
                goal_node = rrt_dubins.propagate(new_node, rrt_dubins.goal)
                if goal_node is not None and rrt_dubins.check_collision(goal_node) and is_within_bounds(goal_node):
                    rrt_dubins.node_list.append(goal_node)
                    final_goal_node = goal_node
                    print("Iters:", i, ", number of nodes:", len(rrt_dubins.node_list))
                    break

    else:
        # This else block executes if the for loop finishes without breaking
        print("Reached max iterations without finding a path")

    # Return path, which is a list of nodes leading to the goal
    if final_goal_node is not None:
        return build_path(final_goal_node)
    return None
