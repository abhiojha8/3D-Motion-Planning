## 3D Motion Planning

This project is a continuation of the Crazyflie backyard flyer project where I executed a simple square shaped flight path. In this project I have integrated various techniques such as A* search, to name a few, to plan a path through an urban environment. 

Click [here](https://github.com/abhiojha8/Flying-Car-Motion-Planning-A-Star) to check out the code and results.

Let us begin with introducing the planning problem and then introduce various techniques that are helpful in creating a flight plan.

### Planning problem

Planning is the core capability of any autonomous vehicle. 

We might not notice but in reality planning is central to our everyday life. In fact throughout the day, we come up with detailed series of actions to get from place A to place B. Similarly, planning is the core capability of any autonomous vehicle. 

Suppose a drone is located at position A and it needs to go to position B. Before flying, it should make a plan, which requires taking decision about which path to take around the buildings, and many other factors such as no fly zones, fuel efficiency, shortest flight time, etc. This planning tasks already seems a bit complex. To further add to this complexity, a drone might now know everything about the state of the world ahead of time. A good planner should consider possible contingencies and take various uncertainties into account. 

Solving a planning problem mostly comes down to setting up your *search space* and then conducting a search through that space. 

### Search Space

As we discussed, before a drone takes a flight, it needs to have a plan. A plan is defined as a series of actions that the drone must take in order to safely and efficiently  move from some initial location to some goal location. Consider the image below:

![](D:\Documents\myProjects\3d_motion_planning\random_images\search_space0.PNG)

One way to think about a path from start (park) to goal (convenience store) is to think about a continuous curve through the free space between obstacles (as shown through the orange lines). However, the vehicle would need to make infinitely many decisions to follow any continuous curve to the goal. Due to this, the number of possible different action plans becomes infinitely large.

When it comes to solving a planning problem, we often face constraints in terms of time and computational resources. The real goal is not to find the perfect solution, but to find a reasonable solution in reasonable amount of time.  Thus, choosing the right search space is key to solving the problem efficiently. 

Due to computational complexities and time constraints in the continuous universe, we will recast the problem by breaking the continuous universe into a finite set of discrete states. Therefore, rather than describing the drone's path as a continuous curve, we will describe it as a series of states the vehicle must pass to reach from start to goal. Some ways to achieve this discretization are depicted in the images below

* Regular grid

  ![](D:\Documents\myProjects\3d_motion_planning\random_images\regular_grid.PNG)

* An exotic way

  ![](D:\Documents\myProjects\3d_motion_planning\random_images\exotic_way.PNG)

  

To represent your planning problem and defining the search space, we need the following:

* All possible states
* Start State
* Goal State
* Actions
* Cost to each possible action

The image below shows a simple way to discretize the space using grids, and mark feasible and infeasible regions of search space:

![](D:\Documents\myProjects\3d_motion_planning\random_images\ud_grid.PNG)



### A* Search

Let `g` be the cost function, which is defined as the sum of the actions we have taken so far in the plan, and `h` be the heuristic function, which is an underestimate of the remaining cost to get from the last state of a partial plan to the goal state.

* g = cost function
* h = heuristic
* f = g + h (estimate of total cost)

