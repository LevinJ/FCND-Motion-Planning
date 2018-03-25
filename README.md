# FCND - 3D Motion Planning
![Quad Image](./misc/enroute.png)



This project is a continuation of the Backyard Flyer project where you executed a simple square shaped flight path. In this project you will integrate the techniques that you have learned throughout the last several lessons to plan a path through an urban environment.



## Project rubrics

### Explain the Starter Code

`motion_planning.py` is basically a modified version of `backyard_flyer.py` that leverages some extra functions in `planning_utils.py`. 

Both complete the flight by transition the program from one state to next in a state machine fashion, namely, MANUAL, ARMING ,TAKEOFF, WAYPOINT, LANDING, DISARMING states. What's different is, in this project, we are now inserting a new state PLANNING between ARMING and TAKEOFF. In this new PLANNING, we will plan out the wayoints that the flight should follow.

### Implementing Path Planning Algorithm

Below are the keys steps involved when implementing the PLANNING sages.

1. Retrieve configuration space

With information from 2.5D map file `colliders.csv`, we are able to calcuate a grid, which indicates where the quodra can safely fly at any particulr altitude. 

2. Set home position

We set the map center (extracted from `colliders.csv`) as our home position.

3. Convert current position from geodetic coordinates to local coordinates

4. Set start position

We set current position as the start position of our search algorithm. Note that the grid we used int the search algorithm are in grid coordinate, so we need to convert local coordinate to grid coordinate, this can be simply done as below,

```
grid_start = (int(local_position[0]-north_offset), int(local_position[1]-east_offset))
```

5. Set goal position

We Tried three different goal positions in our test of search algorithm,

```
//Goal position 1, grid_goal = (900 , 522)
// grid_goal = lontat2grid([-1.22396533e+02,  3.77977389e+01, -1.00000000e-02], north_offset, east_offset, self.global_home)

 //Goal position 2, grid_goal = (276 , 116)
//grid_goal = lontat2grid([-1.22401189e+02,  3.77921385e+01, -1.00000000e-02], north_offset, east_offset, self.global_home)
        
//Goal position 3, grid_goal = (531, 13)
//grid_goal = lontat2grid([-1.22402341e+02,  3.77944427e+01, -1.00000000e-02], north_offset, east_offset, self.global_home)

```
Global coordinate to grid coordinte function `lontat2grid` is implemented in the `mp_utils` file.
6. Imlement search algorithm

A* is used to perform the optimal path from start position to goal positions.

Three different methods are explored to construct the grid/graph that A* uses.

1. raw grid search 

Perfrom A* on the raw configuration space. The advantage is that the algorithm is guranteed to be complete as long as the grid resolution is small engough. The disadvantage is that path planned would often are very close to buiding, and are deemed a bit "unsafe".

This is implemented in the `raw_grid_method ` function in the "mp_utils" file.

2. Media axis based search

Perfrom A* on the media axis grid. This approach is also complete, and the path is also as far away from surrounding obstacle as possible. 
This is implemented in the `media_axis_method ` function in the "mp_utils" file.

3. Voronoi based search
Perfrom A* on the Voronoi graph. This approach is not compete. This search is not guranteed to be complete. This is because the center of the obstacle may not correspond well the contour of the obstacle, and thus some places may not be reachable by by voronoi graph even if they are in reality are. The good side is that the path is also as far away from surrounding obstacle as possible too.

7. Prune waypoints 

Two methods are explord to prune waypoints generated by A* algorithm.

Generaly speaking, Bresenham has a better result than collinearity, though the path it pruned is a bit closer to the building.

This is implemented in the `prune_path ` function in the "mp_utils" file.

```
//check_linear is a flag which allows us to choose either collinearity test or Bresenham
//the grid in only used for Bresenham method
def prune_path(path, grid = None, check_linear = True):
```

### Executing the flight

With the path being planned, the quodra is able to fly smoothly from start to goal! 

## Reflections:

This is a fun project, using grid/graph construction and A* to plan out waypoints, and drive the quad to fly from one place to another.

Given more time, I think I would try out more advanced techniques like Receding Horizon, which looks like a very promising stategy to handle constant replanning required by changing environment.
 

