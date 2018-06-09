import operator as op
import time
from enum import Enum
from queue import PriorityQueue

import matplotlib.pyplot as plt
import numpy as np


class Action2D(Enum):
    """
    Actions when doing 2.5D A* search.

    Actions are only stored as how the will alter the location.
    Costs are inferred from them.
    """

    # I've used a step size of 2 when performing 2.5D A* search
    # This will surely speed up the searching, with drawback that
    # the algorithm may not find a valid way to the goal because of
    # overshooting
    #
    # to handle with this, A* search will stop when the algorithm found
    # a location near to the goal (see `a_star_2_5d` below)
    WEST = (0, -4)
    EAST = (0, 4)
    NORTH = (-4, 0)
    SOUTH = (4, 0)

    # NORTH_EAST = (1, 1)
    # SOUTH_EAST = (1, -1)
    # SOUTH_WEST = (-1, -1)
    # NORTH_WEST = (-1, 1)

    @property
    def delta(self):
        return self.value[0], self.value[1]

    @property
    def cost(self):
        return np.linalg.norm(np.array(self.value))


def create_grid_2_5d(data, safe_distance):
    """
    Create a 2.5D grid from given obstacle data.

    :param data: obstacle data
    :param safe_distance: safe distance added to the surrounding of obstacle
    :return: grid-based 2.5D configuration space
    """
    # minimum and maximum north coordinates
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil((north_max - north_min + 1)))
    east_size = int(np.ceil((east_max - east_min + 1)))

    # Initialize an empty grid
    grid = np.zeros((north_size, east_size))

    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        obstacle = [
            int(np.clip(north - d_north - safe_distance - north_min, 0, north_size - 1)),
            int(np.clip(north + d_north + safe_distance - north_min, 0, north_size - 1)),
            int(np.clip(east - d_east - safe_distance - east_min, 0, east_size - 1)),
            int(np.clip(east + d_east + safe_distance - east_min, 0, east_size - 1)),
        ]
        obs = grid[obstacle[0]:obstacle[1] + 1, obstacle[2]:obstacle[3] + 1]
        np.maximum(obs, np.ceil(alt + d_alt + safe_distance), obs)

    return grid, int(north_min), int(east_min)


def heuristic(position, goal):
    """
    Heuristic function used for A* planning. Simply return the euclidean distance of the two points given.
    """
    return np.linalg.norm(np.array(position) - np.array(goal))


def heuristic_manhattan_dist_2d(position, goal):
    """
    Heuristic function used for calculating manhattan distance between given 2D points.
    """
    return abs(position[0] - goal[0]) + abs(position[1] - goal[1])


def valid_actions(grid, current_node):
    """
    Returns a list of valid actions given a grid and current node.
    """
    actions = list(Action2D)
    north_max, east_max = grid.shape[0] - 1, grid.shape[1] - 1
    n, e, a = current_node

    # check if the node is off the grid or
    # it's an obstacle
    valid = []
    for action in actions:
        dn, de = action.delta
        nn, ne = n + dn, e + de
        # theoretically, a drone can climb up as high as the obstacles then fly over them.
        # in reality, climbing up always requires more thrust power, so it is not always a better
        # choice to climb up when facing with obstacles
        #
        # here I made a simplification: when the drone need to go up 10 meters more than it's current
        # altitude, then going up will be ignored.
        if not (nn < 0 or nn > north_max or
                ne < 0 or ne > east_max):
            # altitude cost. going up will always cost more
            altitude_cost = max(grid[nn, ne] - a, 0) * 50
            valid.append((altitude_cost, action))

    return valid


def reconstruct_path(goal, branch, waypoint_fn):
    """
    Reconstruct a path from the goal state and branch information
    """
    current_node = goal
    path = [current_node]
    while current_node is not None:
        previous_node = branch[waypoint_fn(current_node)]
        path.append(previous_node)
        current_node = previous_node
    path.pop()
    path.reverse()
    return path


def waypoint_fn_2_5d(node):
    """
    Return the waypoint used in 2.5D A* search planning
    """
    return tuple(node[:2])


def waypoint_fn_3d(node):
    """
    Return the waypoint used in 3D A* search planning
    """
    return tuple(node[:3])


def points_collinear_2d_xy(p1, p2, p3):
    """
    Test if given 3 points are collinear if projected onto XY-plane.

    Given points can be in any dimensions greater or equal to 2, but only the first 2 dimensions will be used
    for collinear test.
    """
    x1, y1 = p1[:2]
    x2, y2 = p2[:2]
    x3, y3 = p3[:2]
    return x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2) == 0


def points_collinear_3d(p1, p2, p3):
    """
    By computing the cross product of \vec{p_1 p_2} and \vec{p_2 p_3}, if the result is zero (vector),
    then the 3 points are collinear.
    """
    return np.allclose(np.cross(np.array(p1) - np.array(p2), np.array(p2) - np.array(p3)), (0, 0, 0))


def prune_path(path, collinear_fn):
    """
    Remove unnecessary intermediate waypoints in the path.

    :param path: path to be pruned
    :param collinear_fn: collinearity testing function used for determine if points are collinear
    """
    if len(path) <= 1:
        return path[:]
    new_path = [path[0]]
    line_start = path[0]
    line_end = path[1]
    for i in range(2, len(path)):
        next_end = path[i]
        if collinear_fn(line_start, line_end, next_end):
            line_end = next_end
        else:
            new_path.append(line_end)
            line_start = line_end
            line_end = next_end
    new_path.append(line_end)
    return new_path


def local_path_to_global_path(path, start_local, north_span, east_span, altitude_span):
    center_n, center_e, center_a = start_local
    north_min = max(center_n - north_span, 0)
    east_min = max(center_e - east_span, 0)
    alt_min = max(center_a - altitude_span, 0)
    return [(n + north_min, e + east_min, a + alt_min) for n, e, a in path]


def a_star_2_5d(grid, h, start, goal, flight_altitude, waypoint_fn=lambda n: tuple(n[:2])):
    """
    Perform 2.5D A* search

    :param grid: The 2.5D grid map
    :param h: heuristic function
    :param start: start node in the grid. shall be a 3-element tuple (north, east, altitude) specified in local grid
    coordinates. altitudes shall be specified as positive up.
    :param goal: goal node in the grid.
    :param flight_altitude: target flight altitude
    :param waypoint_fn: a function extracting 2D representation of nodes.
    :return: A path from start to goal in grid coordinate.
    """
    t0 = time.time()
    start_2d = waypoint_fn(start)
    goal_2d = waypoint_fn(goal)

    final_plan = None
    visited = set()
    queue = PriorityQueue()

    queue.put((0, start))
    visited.add(start_2d)
    branch = {start_2d: None}
    found = False
    while not queue.empty() and not found:
        current_cost, current_node = queue.get()
        for alt_cost, action in valid_actions(grid, current_node):
            if found:
                break

            cost = action.cost + alt_cost
            next_node = tuple(map(op.add, waypoint_fn(current_node), action.delta))
            # we want to keep the drone flying in relatively low altitude because that's power saving,
            # on the other hand, the drone shall fly above certain altitude to avoid risk of hitting
            # pedestrians, cars or other objects in low altitudes.
            #
            # limit the drone so that it will at least flying at the lowest flight altitude we specified.
            lowest_alt = int(np.ceil(max(np.ceil(grid[next_node]) + 1, flight_altitude)))
            new_node = (next_node + (lowest_alt,))

            new_node_2d = waypoint_fn(new_node)
            if new_node_2d not in visited:
                new_cost = current_cost + cost + h(new_node, goal)
                branch[new_node_2d] = current_node
                visited.add(new_node_2d)
                queue.put((new_cost, new_node))

                # beware: since the step size of actions are set to 2, the algorithm
                # may overshoot the goal and finally report no paths is found
                #
                # so here instead of exact equal, I use a range for determine if
                # the goal is reached
                if goal_2d[0] - 2 <= new_node_2d[0] <= goal_2d[0] + 2 and \
                        goal_2d[1] - 2 <= new_node_2d[1] <= goal_2d[1] + 2:
                    branch[goal_2d] = current_node
                    goal_loc = (goal[0], goal[1], new_node[2])
                    final_plan = new_cost, reconstruct_path(goal_loc, branch, waypoint_fn)
                    found = True

    if found:
        print("Found a plan. Total cost: {}, time cost: {}".format(final_plan[0], time.time() - t0))
        return final_plan[1]
    else:
        print("Path not found")
        return None


def path_2_5d_to_3d_path(path):
    """
    Convert plan in 2.5D to 3D

    In details, loop over each waypoint in the path. If altitude of current waypoint is different with the previous
    one, insert a transition waypoint that alter the altitude only between them
    """
    path_3d = [path[0]]
    previous = path[0]
    for i in range(1, len(path)):
        current = path[i]
        if previous[2] > current[2]:
            path_3d.append((current[0], current[1], previous[2]))
        elif previous[2] < current[2]:
            path_3d.append((previous[0], previous[1], current[2]))
        path_3d.append(current)
        previous = current
    return path_3d


def visualize_grid_and_pickup_goal(grid, start, callback):
    """
    Visualize 2.5D grid and wait for the goal being picked up
    """
    im = plt.imshow(grid, cmap='gray_r', picker=True)
    plt.axis((0, grid.shape[1], 0, grid.shape[0]))
    plt.xlabel("EAST")
    plt.ylabel("NORTH")
    plt.scatter(start[1], start[0], marker='x', c='red')
    fig = plt.gcf()
    fig.colorbar(im)
    fig.canvas.mpl_connect('pick_event', callback)
    plt.gca().set_title("Pickup the goal on the map\n(close the figure to continue)", fontsize=16)
    plt.show()


def simplify_path(grid, path):
    """
    Check against path[0] --- path[-1], path[0] --- path[-2], ... path[0] --- path[1],
    see whether we have a direct path among them. Returns the longest path once we have found one.
    """
    if len(path) <= 2:
        return path
    print("Simplifying path:", path)
    start_idx = 0
    end_idx = len(path) - 1
    result_path = [path[0]]
    while start_idx < end_idx:
        start = path[start_idx]
        end = path[end_idx]
        min_height = min(start[2], end[2])
        cells = bresenham(start, end)

        has_obs = False
        for n, e in cells:
            if grid[n, e] >= min_height:
                has_obs = True
                break

        if has_obs:
            end_idx -= 1
            if end_idx == start_idx:
                print("Warning. No clear path starts from {}".format(path[start_idx]))
        else:
            result_path.append(end)
            start_idx = end_idx
            end_idx = len(path) - 1
    if result_path[-1] != path[-1]:
        result_path.append(path[-1])

    print("Result path:", result_path)
    return result_path


def bresenham(start, end):
    n1, e1 = start[:2]
    n2, e2 = end[:2]

    if abs(e2 - e1) < 1e-5:
        return [(n, e1) for n in range(min(n1, n2), max(n1, n2) + 1)]

    slope = (n2 - n1) / (e2 - e1)

    if e1 < e2:
        n, e = n1, e1
        ne, ee = n2, e2
    else:
        n, e = n2, e2
        ne, ee = n1, e1

    cells = []

    f = n
    if slope >= 0:
        while e <= ee and n <= ne:
            cells.append((n, e))
            f_new = f + slope
            if f_new > n + 1:
                n += 1
            else:
                e += 1
                f = f_new
    else:
        while e <= ee and n >= ne:
            cells.append((n, e))
            f_new = f + slope
            if f_new < n - 1:
                n -= 1
            else:
                e += 1
                f = f_new

    return cells
