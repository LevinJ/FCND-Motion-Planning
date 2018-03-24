import numpy as np
from udacidrone.frame_utils import global_to_local, local_to_global
from planning_utils import a_star, heuristic, create_grid
from skimage.morphology import medial_axis
from skimage.util import invert


def lontat2grid(global_position, north_offset,east_offset, global_home):
    location_position = global_to_local(global_position, global_home)
    grid_position = (int(round(location_position[0]-north_offset)), int(round(location_position[1]-east_offset)))
    return grid_position

def point(p):
    return np.array([p[0], p[1], 1.])

# def collinearity_check(p1, p2, p3, epsilon=1e-6):   
#     
#     m = np.vstack((p1, p2, p3))
#     det = np.linalg.det(m)
#     return abs(det) < epsilon
def bres_no_obstacle_check(p1, p2,grid): 
    """
    Note this solution requires `x1` < `x2` and `y1` < `y2`.
    """
    x1, y1 = p1
    x2, y2 = p2
    if x1 > x2:
        x1,x2 = x2,x1
        y1,y2 = y2,y1
    if x2 == x1:
        m =None
    else:
        m = (y2 - y1) / (x2 - x1)
    
          
    if m is None:
        if y1>y2:
            ys = np.arange(y2, y1)
        else:
            ys = np.arange(y1, y2)
        for y in ys:
            if grid[x1,y] == 1:
                return False
        return True
      
    
    line_val = y1
    i = x1
    j = y1
    
    while i < x2:

        if grid[i,j] == 1:
            return False
        if line_val + m > j + 1:
            j += 1
        else:
            line_val += m
            i += 1
        
    return True

def collinearity_no_obstacle(p1, p2, p3, epsilon=1e-6): 
    #here we assume is p1,p2,p3 are in a straight line, then there is no obstacle
    det = p1[0]*(p2[1] - p3[1]) + p2[0]*(p3[1] - p1[1]) + p3[0]*(p1[1] - p2[1])
    return abs(det) < epsilon

def check_no_obstacle(p1, p2, p3,grid, check_linear = True):
    if check_linear:
        return collinearity_no_obstacle(p1, p2, p3)
    p1 = p1[:2].astype(np.int)
    p3 = p3[:2].astype(np.int)
    return bres_no_obstacle_check(p1, p3,grid)
    

# We're using collinearity here, but you could use Bresenham as well!
def prune_path(path, grid = None, check_linear = True):
    pruned_path = [p for p in path]
    # TODO: prune the path!
    
    i = 0
    while i < len(pruned_path) - 3:
        p1 = point(pruned_path[i])
        p2 = point(pruned_path[i+1])
        p3 = point(pruned_path[i+2])
        
        # If the 3 points are in a line remove
        # the 2nd point.
        # The 3rd point now becomes and 2nd point
        # and the check is redone with a new third point
        # on the next iteration.
        if check_no_obstacle(p1, p2, p3,grid, check_linear = check_linear):
            # Something subtle here but we can mutate
            # `pruned_path` freely because the length
            # of the list is check on every iteration.
            pruned_path.remove(pruned_path[i+1])
        else:
            i += 1
    return pruned_path

def raw_grid_method(grid, grid_start, grid_goal, check_linear = False):
    path, _ = a_star(grid, heuristic, grid_start, grid_goal)
    print("path point num = {}, path={}".format(len(path), path))
    path = prune_path(path, grid = grid, check_linear = check_linear)
    print("pruned path point num = {}, path={}".format(len(path), path))
    return path

def find_start_goal(skel, start, goal):
    skel_cells = np.transpose(skel.nonzero())
    start_min_dist = np.linalg.norm(np.array(start) - np.array(skel_cells), axis=1).argmin()
    near_start = skel_cells[start_min_dist]
    goal_min_dist = np.linalg.norm(np.array(goal) - np.array(skel_cells), axis=1).argmin()
    near_goal = skel_cells[goal_min_dist]
    
    return near_start, near_goal

def media_axis_method(grid, grid_start, grid_goal, check_linear = True):
    #check_linear specify how we do path pruning, if True, we choose colinearity
    #if False, we use bresenham, usually bresenham yield a much better result
    skeleton = medial_axis(invert(grid))
    skel_start, skel_goal = find_start_goal(skeleton, grid_start, grid_goal)
    path, _ = a_star(invert(skeleton).astype(np.int), heuristic, tuple(skel_start), tuple(skel_goal))
    
    #insert the start and end point if necessary
    if tuple(skel_start) != grid_start:
        path.insert(0, grid_start)
    if tuple(skel_goal) != grid_goal:
        path.append(grid_goal)
    
    #prune the path
    print("path point num = {}, path={}".format(len(path), path))
    path = prune_path(path, grid = grid, check_linear = check_linear)
    print("pruned path point num = {}, path={}".format(len(path), path))
    path = np.array(path).astype(int).tolist()
    return path


    
    