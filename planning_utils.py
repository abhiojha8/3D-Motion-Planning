from enum import Enum
from queue import PriorityQueue
import numpy as np
import operator as op
import time
from enum import Enum
from queue import PriorityQueue
import matplotlib.pyplot as plt

class Action(Enum):
    """
    Actions when doing 2.5D A* search.

    Actions are only stored as how the will alter the location.
    Costs are inferred from them.
    """
    WEST = (0, -2)
    EAST = (0, 2)
    NORTH = (-2, 0)
    SOUTH = (2, 0)

    @property
    def delta(self):
        return self.value[0], self.value[1]

    @property
    def cost(self):
        return np.linalg.norm(np.array(self.value))


def create_grid(data, safe_distance):
    """
    Create a 2.5D grid from given obstacle data.
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
    Returns the eucledian distance between 2 points.
    """
    return np.linalg.norm(np.array(position) - np.array(goal))

def valid_actions(grid, current_node):
    """
    Returns a list of valid actions given a grid and current node.
    """
    actions = list(Action)
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

def collinear_points(p1, p2, p3):
    """
    Check collinearity by using vector cross product
    """
    return np.allclose(np.cross(np.array(p1) - np.array(p2), np.array(p2) - np.array(p3)), (0, 0, 0))


def path_prune(path, collinear_fn):
    """
    prune the path, i.e. remove unnecessary waypoints that are collinear to each other
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


def a_star(grid, h, start, goal, flight_altitude, waypoint_fn=lambda n: tuple(n[:2])):
    """
    2.5D A* search
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


def path_25d_3d(path):
    """
    Convert plan in 2.5D to 3D grid map
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


def pickup_goal(grid, start, callback):
    """
    Pick up goal from the 2.5D grid map
    """
    im = plt.imshow(grid, cmap='gray_r', picker=True)
    plt.axis((0, grid.shape[1], 0, grid.shape[0]))
    plt.xlabel("EAST")
    plt.ylabel("NORTH")
    plt.scatter(start[1], start[0], marker='x', c='red')
    fig = plt.gcf()
    fig.colorbar(im)
    fig.canvas.mpl_connect('pick_event', callback)
    plt.gca().set_title("Choose the goal location\n(close the figure to continue)", fontsize=18)
    plt.show()


def simplify_path(grid, path):
    """
    Test many nodes and find the longest possible direct path between them.
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
                print("No direct path! {}".format(path[start_idx]))
        else:
            result_path.append(end)
            start_idx = end_idx
            end_idx = len(path) - 1
    if result_path[-1] != path[-1]:
        result_path.append(path[-1])

    print("Final path:", result_path)
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
