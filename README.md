## 3D Motion Planning

This project is a continuation of the Crazyflie backyard flyer project where I executed a simple square shaped flight path. In this project I have integrated various techniques such as A* search, Bresenham algorithm, Voronoi graphs, to plan a path through an urban environment. 

Click [here](https://github.com/abhiojha8/Flying-Car-Motion-Planning-A-Star) to check out the code and results.

Let us begin with introducing the planning problem and then introduce various techniques that are helpful in creating a flight plan.

### Planning problem

Planning is the core capability of any autonomous vehicle. 

We might not notice but in reality planning is central to our everyday life. In fact throughout the day, we come up with detailed series of actions to get from place A to place B. Similarly, planning is the core capability of any autonomous vehicle. 

Suppose a drone is located at position A and it needs to go to position B. Before flying, it should make a plan, which requires taking decision about which path to take around the buildings, and many other factors such as no fly zones, fuel efficiency, shortest flight time, etc. This planning tasks already seems a bit complex. To further add to this complexity, a drone might now know everything about the state of the world ahead of time. A good planner should consider possible contingencies and take various uncertainties into account. 

Solving a planning problem mostly comes down to setting up your *search space* and then conducting a search through that space. 

### 2D Planning

---

### Search Space

As we discussed, before a drone takes a flight, it needs to have a plan. A plan is defined as a series of actions that the drone must take in order to safely and efficiently  move from some initial location to some goal location. Consider the image below:

![](https://github.com/abhiojha8/3D-Motion-Planning/blob/master/random_images/search_space0.PNG)

One way to think about a path from start (park) to goal (convenience store) is to think about a continuous curve through the free space between obstacles (as shown through the orange lines). However, the vehicle would need to make infinitely many decisions to follow any continuous curve to the goal. Due to this, the number of possible different action plans becomes infinitely large.

When it comes to solving a planning problem, we often face constraints in terms of time and computational resources. The real goal is not to find the perfect solution, but to find a reasonable solution in reasonable amount of time.  Thus, choosing the right search space is key to solving the problem efficiently. 

Due to computational complexities and time constraints in the continuous universe, we will recast the problem by breaking the continuous universe into a finite set of discrete states. Therefore, rather than describing the drone's path as a continuous curve, we will describe it as a series of states the vehicle must pass to reach from start to goal. Some ways to achieve this discretization are depicted in the images below

* Regular grid

  ![](https://github.com/abhiojha8/3D-Motion-Planning/blob/master/random_images/regular_grid.PNG)

* An exotic way

  ![](https://github.com/abhiojha8/3D-Motion-Planning/blob/master/random_images/exotic_way.PNG)

  

To represent your planning problem and defining the search space, we need the following:

* All possible states
* Start State
* Goal State
* Actions
* Cost to each possible action

The image below shows a simple way to discretize the space using grids, and mark feasible and infeasible regions of search space:

![](https://github.com/abhiojha8/3D-Motion-Planning/blob/master/random_images/ud_grid.PNG)



### A* Search

We will implement A* search to get an optimal path between start and goal positions. Let `g` be the cost function, which is defined as the sum of the actions we have taken so far in the plan, and `h` be the heuristic function, which is an underestimate of the remaining cost to get from the last state of a partial plan to the goal state.

* `g` = cost function
* `h` = heuristic
* `f = g + h` (estimate of total cost)

The difference between `g` and `h` is that gg models the cost of performing actions, irrespective of the environment, while `h` models the cost based on the environment, i.e., the distance to the goal. 

For further theory on A* algorithm check this [article](http://theory.stanford.edu/~amitp/GameProgramming/AStarComparison.html).

Lets understand the working of A* algorithm with the help of some code.

#### A* search example

```python
from queue import PriorityQueue
import numpy as np
from enum import Enum

class Action(Enum):
    """
    An action is represented by a 3 element tuple.

    The first 2 values are the delta of the action relative
    to the current grid position. The third and final value
    is the cost of performing the action.
    """
    LEFT = (0, -1, 1)
    RIGHT = (0, 1, 1)
    UP = (-1, 0, 1)
    DOWN = (1, 0, 1)

    def __str__(self):
        if self == self.LEFT:
            return '<'
        elif self == self.RIGHT:
            return '>'
        elif self == self.UP:
            return '^'
        elif self == self.DOWN:
            return 'v'

    @property
    def cost(self):
        return self.value[2]

    @property
    def delta(self):
        return (self.value[0], self.value[1])

def valid_actions(grid, current_node):
    """
    Returns a list of valid actions given a grid and current node.
    """
    valid = [Action.UP, Action.LEFT, Action.RIGHT, Action.DOWN]
    n, m = grid.shape[0] - 1, grid.shape[1] - 1
    x, y = current_node

    # check if the node is off the grid or
    # it's an obstacle

    if x - 1 < 0 or grid[x-1, y] == 1:
        valid.remove(Action.UP)
    if x + 1 > n or grid[x+1, y] == 1:
        valid.remove(Action.DOWN)
    if y - 1 < 0 or grid[x, y-1] == 1:
        valid.remove(Action.LEFT)
    if y + 1 > m or grid[x, y+1] == 1:
        valid.remove(Action.RIGHT)

    return valid

def visualize_path(grid, path, start):
    sgrid = np.zeros(np.shape(grid), dtype=np.str)
    sgrid[:] = ' '
    sgrid[grid[:] == 1] = 'O'

    pos = start

    for a in path:
        da = a.value
        sgrid[pos[0], pos[1]] = str(a)
        pos = (pos[0] + da[0], pos[1] + da[1])
    sgrid[pos[0], pos[1]] = 'G'
    sgrid[start[0], start[1]] = 'S'
    return sgrid

def heuristic(position, goal_position):
    """
    Here we are using the Eucledian norm heuristic
    """
    h = np.sqrt((position[0] - goal_position[0])**2 + (position[1] - goal_position[1])**2)
    return h

def a_star(grid, h, start, goal):
	"""
	Here we have implemented A* search with the help of a priority queue.
	"""
    path = []
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)

    branch = {}
    found = False

    while not queue.empty():
        item = queue.get()
        current_cost = item[0]
        current_node = item[1]

        if current_node == goal:
            print('Found a path.')
            found = True
            break
        else:
            for action in valid_actions(grid, current_node):
                # get the tuple representation
                da = action.delta
                cost = action.cost
                next_node = (current_node[0] + da[0], current_node[1] + da[1])
                new_cost = current_cost + cost + h(next_node, goal)

                if next_node not in visited:
                    visited.add(next_node)
                    queue.put((new_cost, next_node))

                    branch[next_node] = (new_cost, current_node, action)

    path = []
    path_cost = 0
    if found:

        # retrace steps
        path = []
        n = goal
        path_cost = branch[n][0]
        while branch[n][1] != start:
            path.append(branch[n][2])
            n = branch[n][1]
        path.append(branch[n][2])

    return path[::-1], path_cost
```

Time to test the code! Here's how the results look:

![](D:\Documents\myProjects\3d_motion_planning\random_images\a_star_test.PNG)

### Moving to graphs

We know that the world is a continuous space and thus there are infinitely many places for a drone to be. Till now we had discretized the world into little blocks called grids. The problem with this discretization is that for a 3D space, which is as big as a city, we get a lot of grid blocks to represent. This requires a lot of storage and computational power. To reduce this computational complexity, we will look at discretization as a graph. A graph can be thought of discretization of space with discrete nodes that correspond  to intersections and discrete arcs in between. 

In the context of our drone configuration space nodes are like grid cell locations or states, and edges show where an action connects to states. So instead of a plan that traverses across a grid cell by cell, we can make a plan that traverses a graph by moving from node to node via edges.

![](https://github.com/abhiojha8/3D-Motion-Planning/blob/master/random_images/graph_traversal.PNG)

Similar to searching in grids, A* search can be implemented for graphs to yield a minimal cost path. Some differences between grid and graph based approach are shown below:



| Non-metric graphs              | Grids                          |
| ------------------------------ | ------------------------------ |
| Complete: hard to know         | Complete: Yes                  |
| Optimal: hard to know          | Optimal: Yes                   |
| Computationally Expensive: No! | Computationally Expensive: Yes |

#### Medial Axis Skeletonization

In this section, I will explain the medial axis transformation technique to generate graph used for obtaining path for our drone. The medial axis method is an image processing technique for identifying a "skeleton" of a binary image, or in this case, our grid map of obstacles and free space.  In this exercise, I have used the [medial_axis()](http://scikit-image.org/docs/0.10.x/auto_examples/plot_medial_transform.html) transform method within the the Scikit-Image library. The code to implement medial axis is as follows:

```python
import numpy as np
import matplotlib.pyplot as plt
from grid import create_grid
from skimage.morphology import medial_axis
from skimage.util import invert
from planning import a_star
%matplotlib inline

plt.rcParams['figure.figsize'] = 12, 12

# getting obstacle data
filename = 'colliders.csv'
data = np.loadtxt(filename, delimiter=',', dtype='Float64', skiprows=2)

# start and goal positions
start_ne = (25,  100)
goal_ne = (650, 500)

# Static drone altitude (meters)
drone_altitude = 5
safety_distance = 2

grid = create_grid(data, drone_altitude, safety_distance)
skeleton = medial_axis(invert(grid))

# plot the edges on top of the grid along with start and goal locations
plt.imshow(grid, origin='lower')
plt.imshow(skeleton, cmap='Greys', origin='lower', alpha=0.7)

plt.plot(start_ne[1], start_ne[0], 'rx')
plt.plot(goal_ne[1], goal_ne[0], 'rx')

plt.xlabel('EAST')
plt.ylabel('NORTH')
plt.show()

```

The generated plot  looks like this:

![](https://github.com/abhiojha8/3D-Motion-Planning/blob/master/random_images/medial_axis_res_0.png)

```python
def find_start_goal(skel, start, goal):
    skel_cells = np.transpose(skel.nonzero())
    start_min_dist = np.linalg.norm(np.array(start) - np.array(skel_cells), axis=1).argmin()
    near_start = skel_cells[start_min_dist]
    goal_min_dist = np.linalg.norm(np.array(goal) - np.array(skel_cells), axis=1).argmin()
    near_goal = skel_cells[goal_min_dist]
    return near_start, near_goal

skel_start, skel_goal = find_start_goal(skeleton, start_ne, goal_ne)

print(start_ne, goal_ne)
print(skel_start, skel_goal)

# the above statement prints:
# (25, 100) (650, 500)
# [25 90] [649 500]

def heuristic_func(position, goal_position):
    return np.sqrt((position[0] - goal_position[0])**2 + (position[1] - goal_position[1])**2)

# Run A* on the skeleton
path, cost = a_star(invert(skeleton).astype(np.int), heuristic_func, tuple(skel_start), tuple(skel_goal))
print("Path length = {0}, path cost = {1}".format(len(path), cost))

# the above statement prints:
# Found a path.
# Path length = 675, path cost = 241266.79856747386

plt.imshow(grid, cmap='Greys', origin='lower')
plt.imshow(skeleton, cmap='Greys', origin='lower', alpha=0.7)
# For the purposes of the visual the east coordinate lay along
# the x-axis and the north coordinates long the y-axis.
plt.plot(start_ne[1], start_ne[0], 'x')
plt.plot(goal_ne[1], goal_ne[0], 'x')

pp = np.array(path)
plt.plot(pp[:, 1], pp[:, 0], 'g')
pp2 = np.array(path2)
plt.plot(pp2[:, 1], pp2[:, 0], 'r')

plt.xlabel('EAST')
plt.ylabel('NORTH')
plt.show()
```

The result is shown below:

![](https://github.com/abhiojha8/3D-Motion-Planning/blob/master/random_images/medial_axis_res_1.png)

The path in green is generated by running A* on the skeleton, while the path in red is generated by running A* on the grid.

### Finding Way in the City

Here we will use Voronoi graphs and medial axis transform to find paths which maximize safety distance from obstacles. . In addition, graph representation will allow further optimizations and more succinct queries.

```python
import sys
!{sys.executable} -m pip install -I networkx==2.1
import pkg_resources
pkg_resources.require("networkx==2.1")

import networkx as nx
# In order to make graph creation and manipulation simple, we're going to leverage a powerful package called NetworkX. With Networkx, it's easy to take the nodes and edges you found using the Voronoi and medial axis methods and arrange them into a single graph object.

import numpy as np
import matplotlib.pyplot as plt
from grid import create_grid_and_edges
import numpy.linalg as LA
%matplotlib inline

plt.rcParams['figure.figsize'] = 12, 12

filename = 'colliders.csv'
data = np.loadtxt(filename, delimiter=',', dtype='Float64', skiprows=2)
print(data)

# The printed data looks like this:
# [[-310.2389   -439.2315    85.5         5.          5.         85.5 ]
# [-300.2389   -439.2315     85.5         5.          5.         85.5 ]
# [-290.2389   -439.2315     85.5         5.          5.         85.5 ]

# [ 257.8061    425.1645      1.75852     1.292725    1.292725    1.944791]
# [ 293.9967    368.3391      3.557666    1.129456    1.129456    3.667319]
# [ 281.5162    354.4156      4.999351    1.053772    1.053772    4.950246]]

# Starting and goal positions in (north, east)
start_ne = (25,  100)
goal_ne = (750., 370.)

# Static drone altitude (metres)
drone_altitude = 5
safety_distance = 3

# This is now the routine using Voronoi
grid, edges = create_grid_and_edges(data, drone_altitude, safety_distance)
print(len(edges))
#prints: 1895

# Plotting the edges on top of the grid along with start and goal locations
plt.imshow(grid, origin='lower', cmap='Greys')

for e in edges:
    p1 = e[0]
    p2 = e[1]
    plt.plot([p1[1], p2[1]], [p1[0], p2[0]], 'b-')

plt.plot(start_ne[1], start_ne[0], 'rx')
plt.plot(goal_ne[1], goal_ne[0], 'rx')

plt.xlabel('EAST')
plt.ylabel('NORTH')
plt.show()
```

The resulting plot looks like this:

![](https://github.com/abhiojha8/3D-Motion-Planning/blob/master/random_images/voronoi%20edges.PNG)

```python
# We now have a graph, well at least visually. The next step is to use the networkx to create the graph. NetworkX is a popular library handling anything and everything related to graph data structures and algorithms.

# Creating the graph with the weight of the edges set to the Euclidean distance between the points
G = nx.Graph()
for e in edges:
    p1 = e[0]
    p2 = e[1]
    dist = LA.norm(np.array(p2) - np.array(p1))
    G.add_edge(p1, p2, weight=dist)

# We need a method to search the graph, we'll adapt A* in order to do this. The notable differences being the actions are now the outgoing edges and the cost of an action is that weight of that edge.

from queue import PriorityQueue

def heuristic(n1, n2):
    return LA.norm(np.array(n2) - np.array(n1))

def a_star(graph, h, start, goal):
    """Modified A* to work with NetworkX graphs."""

    path = []
    path_cost = 0
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)

    branch = {}
    found = False

    while not queue.empty():
        item = queue.get()
        current_node = item[1]
        if current_node == start:
            current_cost = 0.0
        else:
            current_cost = branch[current_node][0]

        if current_node == goal:
            print('Found a path.')
            found = True
            break
        else:
            for next_node in graph[current_node]:
                cost = graph.edges[current_node, next_node]['weight']
                branch_cost = current_cost + cost
                queue_cost = branch_cost + h(next_node, goal)

                if next_node not in visited:
                    visited.add(next_node)
                    branch[next_node] = (branch_cost, current_node)
                    queue.put((queue_cost, next_node))

    if found:
        # retrace steps
        n = goal
        path_cost = branch[n][0]
        path.append(goal)
        while branch[n][1] != start:
            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])
    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************')
    return path[::-1], path_cost
```

We now need to find the closest point in the graph to our current location, same thing for the goal location. Next, we need to compute the path from the two points in the graph using the A* algorithm.

```python
def closest_point(graph, current_point):
    """
    Compute the closest point in the `graph`
    to the `current_point`.
    """
    closest_point = None
    dist = 100000
    for p in graph.nodes:
        d = LA.norm(np.array(p) - np.array(current_point))
        if d < dist:
            closest_point = p
            dist = d
    return closest_point

start_ne_g = closest_point(G, start_ne)
goal_ne_g = closest_point(G, goal_ne)
print(start_ne_g)
print(goal_ne_g)

#prints:
# (20.761099999999999, 103.26850000000002)
# (748.71871888430212, 364.41446809309031)

# Using A* to compute the path
path, cost = a_star(G, heuristic, start_ne_g, goal_ne_g)
print(len(path))

#prints:
# Found a path.
# 97

plt.imshow(grid, origin='lower', cmap='Greys') 

for e in edges:
    p1 = e[0]
    p2 = e[1]
    plt.plot([p1[1], p2[1]], [p1[0], p2[0]], 'b-')
    
plt.plot([start_ne[1], start_ne_g[1]], [start_ne[0], start_ne_g[0]], 'r-')
for i in range(len(path)-1):
    p1 = path[i]
    p2 = path[i+1]
    plt.plot([p1[1], p2[1]], [p1[0], p2[0]], 'r-')
plt.plot([goal_ne[1], goal_ne_g[1]], [goal_ne[0], goal_ne_g[0]], 'r-')
    
plt.plot(start_ne[1], start_ne[0], 'gx')
plt.plot(goal_ne[1], goal_ne[0], 'gx')

plt.xlabel('EAST', fontsize=20)
plt.ylabel('NORTH', fontsize=20)
plt.show()
```

The resulting path is shown below (in red):

![](https://github.com/abhiojha8/3D-Motion-Planning/blob/master/random_images/medial_voronoi_path.PNG)



### 3D Planning

---

Till now we have looked at different ways to solve 2D planning problem for flying cars. But flying cars exist in 3D. In this section, we will see how to solve a planning problem for a 3D vehicle in a 3D world. We know that in order to solve the planning problem we need the state-space, an action space, cost function, start and goal state. To get the state-space we need to represent the geometry of the world in 3D and the configuration space.

#### 3D Grids

The most basic kind of 3D representation is a grid with once cell per discrete X, Y, Z coordinate. 

![](https://github.com/abhiojha8/3D-Motion-Planning/blob/master/random_images/3d_grid.PNG)

3D cells are known as "Voxels", which is short for Volume Element (just like "pixel" is short for picture element). Thus, a 3D grid map is called a Voxel map. 

![](D:\Documents\myProjects\3d_motion_planning\random_images\voxel_map.PNG)

Let's check out code which returns a 3D grid where cells containing voxel are set to true.

```python
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

%matplotlib inline 

plt.rcParams['figure.figsize'] = 16, 16

filename = 'colliders.csv'
data = np.loadtxt(filename, delimiter=',', dtype='Float64', skiprows=2)
print(data)
#prints:
#[[-305.  -435.    85.5    5.     5.    85.5]
# [-295.  -435.    85.5    5.     5.    85.5]
# [-285.  -435.    85.5    5.     5.    85.5]
# ..., 
# [ 435.   465.     8.     5.     5.     8. ]
# [ 445.   465.     8.     5.     5.     8. ]
# [ 455.   465.     8.     5.     5.     8. ]]

def create_voxmap(data, voxel_size=5):
    """
    Returns a grid representation of a 3D configuration space
    based on given obstacle data.
    
    The `voxel_size` argument sets the resolution of the voxel map. 
    """
    # minimum and maximum north coordinates
    north_min = np.floor(np.amin(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.amax(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.amin(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.amax(data[:, 1] + data[:, 4]))

    # maximum altitude
    alt_max = np.ceil(np.amax(data[:, 2] + data[:, 5]))
    
    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil(north_max - north_min)) // voxel_size
    east_size = int(np.ceil(east_max - east_min)) // voxel_size
    alt_size = int(alt_max) // voxel_size

    # Create an empty grid
    voxmap = np.zeros((north_size, east_size, alt_size), dtype=np.bool)

    for i in range(data.shape[0]):
        # Filling in the voxels that are part of an obstacle with `True`
        # i.e. grid[0:5, 20:26, 2:7] = True
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        obstacle = [
            int(north - d_north - north_min) // voxel_size,
            int(north + d_north - north_min) // voxel_size,
            int(east - d_east - east_min) // voxel_size,
            int(east + d_east - east_min) // voxel_size,
        ]

        height = int(alt + d_alt) // voxel_size
        voxmap[obstacle[0]:obstacle[1], obstacle[2]:obstacle[3], 0:height] = True

    return voxmap

voxmap = create_voxmap(data, 10)
print(voxmap.shape)
#prints:
# (81, 91, 21)

# plot the 3D grid
fig = plt.figure()
ax = fig.gca(projection='3d')
ax.voxels(voxmap, edgecolor='k')
ax.set_xlim(voxmap.shape[0], 0)
ax.set_ylim(0, voxmap.shape[1])
# add a bit to z-axis height for visualization
ax.set_zlim(0, voxmap.shape[2]+20)

plt.xlabel('North')
plt.ylabel('East')

plt.show()
```

The resulting plot looks like this:

![](https://github.com/abhiojha8/3D-Motion-Planning/blob/master/random_images/3d_voxel_map.PNG)

Since 3D maps are very much computationally expensive, we will be using 2.5D maps for the planning problem. In 3D maps, each cell was labelled by whether or not it was feasible (infeasible when the cell has an obstacle). In 2.5D maps, each cell is labelled with the height of the obstacle OR the minimum height at which that cell becomes feasible. 

#### Random Sampling of an environment using 2.5D representation

Sampling our environment at random is a relatively efficient way to build up a set of feasible states through our free space. To generate a set of feasible states, we will first scatter points at random throughout our environment over some range in x, y, and z. Then for each of those points, we'll test whether it lies inside the ground plane polygon of any obstacles, and if so, whether or not it is above or below the height of the obstacle.

We'll then discard points that are in collision with obstacles, or in other words, those that lie within the ground plane polygon of an obstacle and below the height of that obstacle. And what we're left with are a collection of states that lie in the free space!

```python
import time
import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import Polygon, Point
%matplotlib inline

plt.rcParams['figure.figsize'] = 12, 12

filename = 'colliders.csv'
data = np.loadtxt(filename, delimiter=',', dtype='Float64', skiprows=2)
print(data)
#prints:
#[[-310.2389   -439.2315     85.5         5.          5.         85.5     ]
# [-300.2389   -439.2315     85.5         5.          5.         85.5     ]
# [-290.2389   -439.2315     85.5         5.          5.         85.5     ]
# ..., 
# [ 257.8061    425.1645      1.75852     1.292725    1.292725    1.944791]
# [ 293.9967    368.3391      3.557666    1.129456    1.129456    3.667319]
# [ 281.5162    354.4156      4.999351    1.053772    1.053772    4.950246]]

# create polygon
def extract_polygons(data):

    polygons = []
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        
        # Extract the 4 corners of each obstacle
        # NOTE: The order of the points needs to be counterclockwise
        # in order to work with the simple angle test
        # Also, `shapely` draws sequentially from point to point.
        # If the area of the polygon in shapely is 0, you've likely got a weird order.
        obstacle = [north - d_north, north + d_north, east - d_east, east + d_east]
        corners = [(obstacle[0], obstacle[2]), (obstacle[0], obstacle[3]), (obstacle[1], 	     				obstacle[3]), (obstacle[1], obstacle[2])]
        
        # Compute the height of the polygon
        height = alt + d_alt

        p = Polygon(corners)
        polygons.append((p, height))

    return polygons

polygons = extract_polygons(data)
print(len(polygons))
#prints: 2926

```



#### Random Sampling using KD Tree

[KD Tree](https://en.wikipedia.org/wiki/K-d_tree) is a space partitioning data structure which allows for fast search queries. Using KD Tree for random sampling brings the total search time down to $[Math Processing Error]O(m∗log(n))$ from $[Math Processing Error]O(m*n)O(m∗n)$, where $[Math Processing Error]m$ is the number of elements to compare to and $[Math Processing Error]n$ is the number of elements in the KD Tree. 

```python
import sys
!{sys.executable} -m pip install -I networkx==2.1
import pkg_resources
pkg_resources.require("networkx==2.1")
import networkx as nx

import numpy as np
import matplotlib.pyplot as plt
from sampling import Sampler
from shapely.geometry import Polygon, Point, LineString
from queue import PriorityQueue

%matplotlib inline

plt.rcParams['figure.figsize'] = 12, 12

filename = 'colliders.csv'
data = np.loadtxt(filename, delimiter=',', dtype='Float64', skiprows=2)
print(data)

# prints:
"""
[[-310.2389   -439.2315     85.5         5.          5.         85.5     ]
 [-300.2389   -439.2315     85.5         5.          5.         85.5     ]
 [-290.2389   -439.2315     85.5         5.          5.         85.5     ]
 ..., 
 [ 257.8061    425.1645      1.75852     1.292725    1.292725    1.944791]
 [ 293.9967    368.3391      3.557666    1.129456    1.129456    3.667319]
 [ 281.5162    354.4156      4.999351    1.053772    1.053772    4.950246]]
"""

# Sample points

from sampling import Sampler
sampler = Sampler(data)
polygons = sampler._polygons
# Example: sampling 100 points and removing ones conflicting with obstacles.
nodes = sampler.sample(300)
print(len(nodes))

# prints:
# 218

# Connecting nodes

import numpy.linalg as LA
from sklearn.neighbors import KDTree

def can_connect(n1, n2):
    l = LineString([n1, n2])
    for p in polygons:
        if p.crosses(l) and p.height >= min(n1[2], n2[2]):
            return False
    return True

def create_graph(nodes, k):
    g = nx.Graph()
    tree = KDTree(nodes)
    for n1 in nodes:
        # for each node connect try to connect to k nearest nodes
        idxs = tree.query([n1], k, return_distance=False)[0]
        
        for idx in idxs:
            n2 = nodes[idx]
            if n2 == n1:
                continue
                
            if can_connect(n1, n2):
                g.add_edge(n1, n2, weight=1)
    return g

import time
t0 = time.time()
g = create_graph(nodes, 10)
print('graph took {0} seconds to build'.format(time.time()-t0))

# prints:
# graph took 36.59697699546814 seconds to build

print("Number of edges", len(g.edges))
# prints:
# Number of edges 636

# Visualizing graph:

from grid import create_grid
grid = create_grid(data, sampler._zmax, 1)

fig = plt.figure()

plt.imshow(grid, cmap='Greys', origin='lower')

nmin = np.min(data[:, 0])
emin = np.min(data[:, 1])

# draw edges
for (n1, n2) in g.edges:
    plt.plot([n1[1] - emin, n2[1] - emin], [n1[0] - nmin, n2[0] - nmin], 'black' , alpha=0.5)
# draw all nodes
for n1 in nodes:
    plt.scatter(n1[1] - emin, n1[0] - nmin, c='blue')    
# draw connected nodes
for n1 in g.nodes:
    plt.scatter(n1[1] - emin, n1[0] - nmin, c='red')
 
plt.xlabel('NORTH')
plt.ylabel('EAST')
plt.show()
```

The resulting plot looks like this:

![](https://github.com/abhiojha8/3D-Motion-Planning/blob/master/random_images/kd_tree_plot_1.PNG)

```python
# Next we will define a heuristic and run A* search

def heuristic(n1, n2):
    # TODO: finish
    return LA.norm(np.array(n2) - np.array(n1))

def a_star(graph, heuristic, start, goal):
    """Modified A* to work with NetworkX graphs."""
    path = []
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)

    branch = {}
    found = False
    
    while not queue.empty():
        item = queue.get()
        current_cost = item[0]
        current_node = item[1]

        if current_node == goal:        
            print('Found a path.')
            found = True
            break
        else:
            for next_node in graph[current_node]:
                cost = graph.edges[current_node, next_node]['weight']
                new_cost = current_cost + cost + heuristic(next_node, goal)
                
                if next_node not in visited:                
                    visited.add(next_node)               
                    queue.put((new_cost, next_node))      
                    branch[next_node] = (new_cost, current_node)
             
    path = []
    path_cost = 0
    if found:
        # retrace steps
        path = []
        n = goal
        path_cost = branch[n][0]
        while branch[n][1] != start:
            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])
            
    return path[::-1], path_cost

# setting start and goal positions
start = list(g.nodes)[0]
k = np.random.randint(len(g.nodes))
print(k, len(g.nodes))
goal = list(g.nodes)[k]

# prints
# 29 207

path, cost = a_star(g, heuristic, start, goal)
print(len(path), path)

# prints:
"""
Found a path.
14 [(413.49489677115827, -124.51843092865352, 4.865069145657861), (491.47042718257518, -131.72798482935417, 15.287310053474492), (516.74333007780547, -45.00321102819305, 11.957013174250498), (474.85039745313503, 40.082743701482514, 12.174017469321235), (445.23486464827891, 100.44174549026502, 14.268897874269422), (376.72535483378482, 78.92280411055026, 17.150023273294483), (293.37125799703477, 119.90182406374481, 9.1884670522723759), (262.92024287422674, 98.995451705151368, 6.3476205555021554), (181.71701963410572, 23.306050718325992, 18.039238237030567), (147.84605250920458, 59.945079480475613, 8.8119883582117158), (151.4653148249908, 133.26129675613743, 13.126536812931906), (93.089512264263135, 155.35123826871313, 18.060365497377092), (11.763542252446825, 240.57729026090692, 15.007799958835333), (-29.557771204484595, 332.09296869622983, 4.7361657677699025)]
"""

path_pairs = zip(path[:-1], path[1:])
for (n1, n2) in path_pairs:
    print(n1, n2)
    
# prints:
"""
(413.49489677115827, -124.51843092865352, 4.865069145657861) (491.47042718257518, -131.72798482935417, 15.287310053474492)
(491.47042718257518, -131.72798482935417, 15.287310053474492) (516.74333007780547, -45.00321102819305, 11.957013174250498)
(516.74333007780547, -45.00321102819305, 11.957013174250498) (474.85039745313503, 40.082743701482514, 12.174017469321235)
(474.85039745313503, 40.082743701482514, 12.174017469321235) (445.23486464827891, 100.44174549026502, 14.268897874269422)
(445.23486464827891, 100.44174549026502, 14.268897874269422) (376.72535483378482, 78.92280411055026, 17.150023273294483)
(376.72535483378482, 78.92280411055026, 17.150023273294483) (293.37125799703477, 119.90182406374481, 9.1884670522723759)
(293.37125799703477, 119.90182406374481, 9.1884670522723759) (262.92024287422674, 98.995451705151368, 6.3476205555021554)
(262.92024287422674, 98.995451705151368, 6.3476205555021554) (181.71701963410572, 23.306050718325992, 18.039238237030567)
(181.71701963410572, 23.306050718325992, 18.039238237030567) (147.84605250920458, 59.945079480475613, 8.8119883582117158)
(147.84605250920458, 59.945079480475613, 8.8119883582117158) (151.4653148249908, 133.26129675613743, 13.126536812931906)
(151.4653148249908, 133.26129675613743, 13.126536812931906) (93.089512264263135, 155.35123826871313, 18.060365497377092)
(93.089512264263135, 155.35123826871313, 18.060365497377092) (11.763542252446825, 240.57729026090692, 15.007799958835333)
(11.763542252446825, 240.57729026090692, 15.007799958835333) (-29.557771204484595, 332.09296869622983, 4.7361657677699025)
"""

# Visualize the path
fig = plt.figure()

plt.imshow(grid, cmap='Greys', origin='lower')

nmin = np.min(data[:, 0])
emin = np.min(data[:, 1])

# draw nodes
for n1 in g.nodes:
    plt.scatter(n1[1] - emin, n1[0] - nmin, c='red')    
# draw edges
for (n1, n2) in g.edges:
    plt.plot([n1[1] - emin, n2[1] - emin], [n1[0] - nmin, n2[0] - nmin], 'yellow')    
# code to visualize the path
path_pairs = zip(path[:-1], path[1:])
for (n1, n2) in path_pairs:
    plt.plot([n1[1] - emin, n2[1] - emin], [n1[0] - nmin, n2[0] - nmin], 'green')

plt.xlabel('NORTH')
plt.ylabel('EAST')

plt.show()
```

The result is as follows:

![](https://github.com/abhiojha8/3D-Motion-Planning/blob/master/random_images/kd_tree_plot_2.PNG)

The green line indicates the drone path found using A* search.

So far, we know that there are 2 big sources of computational complexity:

* Cost of building a model.
* Cost of planning through the model.

The probabilistic mapping technique used above, to build a graph around obstacles works well if you assume that you know environment perfectly ahead of time and the positions of object don't change. However, this is not the case in real world. Assuming that the environment is relatively fixed, we are likely to run probabilistic road map algorithm once and A* search multiple times. This is known as multiple query planning. This means that for a fixed graph (that we built using probabilistic road map algorithm), the computational cost will be dominated by the search. Thus, we recognize that planning the entire trajectory from start to goal, considering all state variables and features of environment is not the right approach. A better strategy is to plan an approximate route in 2D using coarse grid or graph, and as the vehicle travels, keep improving that plan after some distance or using the global plan as heuristic. 

![](https://github.com/abhiojha8/3D-Motion-Planning/blob/master/random_images/receding_horizon.PNG)



### Receding Horizon Planning

Receding horizon planning is a two-tiered approach to solving the planning problem. First, we find a coarse global plan all the way from the start to the goal. Then, as we execute that plan, we continuously re-plan in a local volume around the vehicle at a higher resolution. This approach allows for fine tuning our plan on the fly, reacting to obstacles that weren't on the map or other uncertainties, like sensor errors or wind. This planning approach will be looked at in future developments of this project.

The following paper, [Path Planning for Non-Circular Micro Aerial Vehicles in Constrained Environments](https://www.cs.cmu.edu/~maxim/files/pathplanforMAV_icra13.pdf), addresses the problem of path planning for a quadrotor using more advanced techniques.

## 3D Motion Planning Project

For implementation, I have used 2.5 D grid maps, A* search, Euclidean distance heuristic. The code can be seen in files [planning_utils.py](https://github.com/abhiojha8/3D-Motion-Planning/blob/master/planning_utils.py) and [motion_planning.py](https://github.com/abhiojha8/3D-Motion-Planning/blob/master/motion_planning.py).

The resultant path can be seen in the image below:

