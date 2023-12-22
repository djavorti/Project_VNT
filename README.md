# Project: 3D Motion Planning
## G1 - Team Members: Djalmar Vargas, Adrian Correa,Angel Guzman, Hugo Medina.
## Explain the Starter Code
### 1. Explain the functionality of what's provided in motion_planning.py and planning_utils.py

In motion_planning.py, an additional Planning state is introduced, and a key event handler, plan_path, manages the transition to this state. The plan_path method performs the following actions:

1. Sets the flight state to Planning.
2. Specifies a predetermined target altitude and safety distance.
3. Generates a grid representation of the configuration space with the defined altitude and safety distance.
4. Reads obstacle data from a file.
5. Defines start and goal positions.
6. Utilizes the A* algorithm to plan a path through the configuration space, using the provided heuristic as a cost function.

The accompanying planning_utils.py contains utility functions crucial for the path planning process. These utilities include:

1. create_grid: Generates a 2.5D grid representation based on obstacle data (formatted similarly to colliders.csv) and a specified drone altitude and safety distance.

2. Action: An enumeration class defining a 3-tuple for each of the four movement directions on the grid (North, South, East, West). Each enumeration element includes a grid-based position delta (N, E) and a cost.

3. valid_actions: Determines all valid actions for a given grid cell, considering the current grid and node.

4. a_star: Implements an A* path planning algorithm for a grid-based representation, given start and goal states, a specified heuristic, and the grid itself.

5. heuristic: Provides a cost heuristic for the A* algorithm, aiding in approximating the relative cost between two given points.

## Implementing Your Path Planning Algorithm
### 1. Set your global home position
The code reads the first line of a CSV file ('colliders.csv'), extracts longitude (lon0) and latitude (lat0) values, assuming a specific format, and converts them to floating-point numbers. These values are then used to set the global home position.
```
with open('colliders.csv') as f:
            first_line = f.readline().strip()
        latlon = first_line.split(',')
        lon0 = float(latlon[0].strip().split(' ')[1])
        lat0 = float(latlon[1].strip().split(' ')[1])
        self.set_home_position(lat0, lon0, 0)
```
### 2. Set your current local position
The code sets the local position (local_pos) by converting the current global position (self.global_position) to local coordinates using a function like global_to_local, with the global home position specified as self.global_home.
```
local_pos = global_to_local(self.global_position,global_home=self.global_home)
```
### 3. Set grid start position from local position
This code snippet sets the grid start position based on the local position (local_pos). It extracts the north and east coordinates from local_pos, adjusts them by subtracting corresponding offset values (north_offset and east_offset), rounds the results to integers, and assigns them to the variable grid_start.
```
north, east, att = local_pos
grid_start = (int(np.rint(north - north_offset)),
                      int(np.rint(east - east_offset)))
        print("Grid Start: ", grid_start)
```
### 4. Set grid goal position from geodetic coords
The code starts with an empty goal_list and a boolean variable goal_obs set to True. The code then iterates through a loop, attempting to randomly select a goal location while avoiding obstacles. In each iteration, a random change vector is generated, and the global goal position is calculated by adding this vector to the global home position. The code checks whether the resulting local goal position falls within the map boundaries and adjusts it if necessary. It continues this process until a non-obstructed goal is found or a maximum number of attempts (100 in this case) is reached. The final grid goal position is printed along with the grid start position. The goal positions are adjusted to ensure they are within the valid grid boundaries and not obstructed by obstacles.
```
goal_list = []

        # If goal location is in an obstacle
        goal_obs = True

        # randomly select a goal
        dist_idx = 100.0
        goal_try = 0
        while goal_obs and goal_try < 100:
            goal_try += 1
            change = np.random.rand(3)
            change -= 0.5
            print("change", change)
            goal = (self.global_home[0] + change[0] / dist_idx,
                    self.global_home[1] + change[1] / (dist_idx),
                    self.global_home[2] + change[2] * 10.0)
            print("Goal Global: ", goal)
            local_goal = global_to_local(goal, global_home=self.global_home)
            print("Goal Local: ", local_goal)
            ng, eg, ag = local_goal
            grid_goal = (int(np.rint(ng - north_offset)),
                         int(np.rint(eg - east_offset)))

            if grid_goal[0] > grid_shape[0] - 2:
                grid_goal = (grid_shape[0] - 2, grid_goal[1])
            elif grid_goal[0] < 1:
                grid_goal = (1, grid_goal[1])
            if grid_goal[1] > grid_shape[1] - 2:
                grid_goal = (grid_goal[0], grid_shape[1] - 2)
            elif grid_goal[1] < 1:
                grid_goal = (grid_goal[0], 1)

            goal_obs = grid[grid_goal[0], grid_goal[1]]
            if goal_obs:
                goal_list.append(grid_goal)

        print('Grid Start and Goal: ', grid_start, grid_goal)
```
### 5. Modify A* to include diagonal motion
Changes respect to the original version:
- Added Diagonal Movements to Action Enum:
Introduces four new actions (NORTH_WEST, NORTH_EAST, SOUTH_WEST, SOUTH_EAST) representing diagonal movements.
Assigns appropriate delta and cost values for diagonal actions.
-Updated valid_actions Function:
Introduces a cost adjustment to prevent zigzags by moving the previous action to the front of the list.
Considers the new diagonal actions when determining valid actions.
Calculates new positions for each action, checking for boundaries and obstacles.
Returns a list of valid actions with their corresponding new positions.
-Enhanced a_star Function:
Introduces additional variables (depth, depth_act, report_int) to provide information about the planning process for debugging purposes.
Dynamically adjusts the move distance (move) based on the heuristic cost (current_h_cost).
```
WEST = (0, -1, 1)
    EAST = (0, 1, 1)
    NORTH = (-1, 0, 1)
    SOUTH = (1, 0, 1)
    NORTH_WEST = (-1, -1, np.sqrt(2))
    NORTH_EAST = (-1, 1, np.sqrt(2))
    SOUTH_WEST = (1, -1, np.sqrt(2))
    SOUTH_EAST = (1, 1, np.sqrt(2))
```
```
    # To prevent zigzags add a cost to changing action
    # Move previous action first
    if (current_action is not None and
            current_action in all_actions):
        all_actions.remove(current_action)
        all_actions = [current_action] + all_actions

    for new_action in all_actions:
        new_x = current_node[0] + new_action.delta[0] * move
        new_y = current_node[1] + new_action.delta[1] * move

        if (new_x < 0 or new_x > n or
            new_y < 0 or new_y > m or
                grid[new_x, new_y]):
            pass
        else:
            valid_actions_nodes.append((new_action, (new_x, new_y)))
```
```
if current_node in visited:
            continue

        visited.add(current_node)
        depth += 1

        if current_node == start:
            current_cost = 0.0
            current_action = None
        else:
            current_cost = branch[current_node][0]
            current_action = branch[current_node][2]
        if depth % report_int == 0:
            print("#Nodes:%s, #Actions:%s, Cost:%.2f, Currenct Node:%s,"
                  " Time:%.2f" % (depth, depth_act, current_cost,
                                  current_node, time.time() - t0))
            report_int *= 2

        current_h_cost = current_q_cost - current_cost

        if current_h_cost < np.sqrt(2) * float(max_move):
            move = 1
        else:
            move = max_move
```

### 6. Cull waypoints
The collinearity_check function determines if three points are collinear. The prune_path function iteratively removes waypoints exhibiting collinearity, up to a specified limit (max_p). The resulting pruned path is returned, simplifying the trajectory while reducing the number of waypoints.
```
def collinearity_check(p1, p2, p3, epsilon=1e-6):
    m = np.concatenate((p1, p2, p3), 0)
    det = np.linalg.det(m)
    return abs(det) < epsilon


def prune_path(path, max_p=50):
    pruned_path = [p for p in path]
    # prune the path!
    i = 0
    ri = 0
    while i < len(pruned_path) - 2:
        p1 = point(pruned_path[i])
        p2 = point(pruned_path[i + 1])
        p3 = point(pruned_path[i + 2])
        if collinearity_check(p1, p2, p3) and ri < max_p:
            pruned_path.remove(pruned_path[i + 1])
            ri += 1
        else:
            i += 1
            ri = 0
    return pruned_path
```
## Executing the flight
Youtube link: https://youtu.be/w9FtqwVHqOM
