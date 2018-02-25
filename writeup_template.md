## Project: 3D Motion Planning
![image1]
[//]: # (Image References)
[image1]: ./random_images/enroute.png
[image2]: ./random_images/high_up.png
[image3]: ./random_images/in_the_trees.png
[image4]: ./random_images/map.png

---


# Required Steps for a Passing Submission:
1. Load the 2.5D map in the colliders.csv file describing the environment.
2. Discretize the environment into a grid or graph representation.
3. Define the start and goal locations.
4. Perform a search using A* or other search algorithm.
5. Use a collinearity test or ray tracing method (like Bresenham) to remove unnecessary waypoints.
6. Return waypoints in local ECEF coordinates (format for `self.all_waypoints` is [N, E, altitude, heading], where the drone’s start location corresponds to [0, 0, 0, 0].
7. Write it up.
8. Congratulations!  Your Done!

## [Rubric](https://review.udacity.com/#!/rubrics/1534/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it! Below I describe how I addressed each rubric point and where in my code each point is handled.

### Modify Existing Code

#### 1. Modify BY Flyer solution code
I modified my Backyard Flyer solution to include a new `PLANNING` state and reconfigured my `state_callback_function()` to allow for a transition to planning after arming and before taking off. To accomplish this, I modified the existing code at line ### etc.

And here's a lovely image of my results (ok this image has nothing to do with it, but it's a nice example of how to include images in your writeup!)
![image2]


#### 2. Add a function for path planning
I added a new function to my main script to be called for path planning.  This function passes information about the vehicle's home position, the goal location etc. to some planning methods I implemented in a separate script. The following table describes how I approached this.

Here's | A | Snappy | Table
--- | --- | --- | ---
1 | `highlight` | **bold** | 7.41
2 | a | b | c
3 | *italic* | text | 403
4 | 2 | 3 | abcd

### Implementing the path planning algorithm

#### 1. Load map and discretize
I did this and here's a look at how the environment looks from above in 2D!
![image4]

#### 2. Perform Search (**max altitude of 3 meters**)
I used A* to search through my graph and find a path from start to goal.  Here I'll describe line by line what my implementation is doing to accomplish this task...

Meanwhile, here's a picture of me flying through the trees!
![image3]

#### 2. Cull waypoints
I accomplished this task using Bresenham's algorithm to check for collisions...

### Execute the flight
#### 1. Does it work?
It works!

### Double check that you've met specifications for each of the [rubric](https://review.udacity.com/#!/rubrics/1534/view) points.
  
# Extra Challenges: Real World Planning

For an extra challenge, consider implementing some of the techniques described in the "Real World Planning" lesson. You could try implementing a vehicle model to take dynamic constraints into account, or implement a replanning method to invoke if you get off course or encounter unexpected obstacles.


