import numpy as np
from udacidrone.frame_utils import global_to_local, local_to_global
from planning_utils import a_star, heuristic, create_grid


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

def collinearity_check(p1, p2, p3, epsilon=1e-6): 
    det = p1[0]*(p2[1] - p3[1]) + p2[0]*(p3[1] - p1[1]) + p3[0]*(p1[1] - p2[1])
    return abs(det) < epsilon

# We're using collinearity here, but you could use Bresenham as well!
def prune_path(path):
    pruned_path = [p for p in path]
    # TODO: prune the path!
    
    i = 0
    while i < len(pruned_path) - 2:
        p1 = point(pruned_path[i])
        p2 = point(pruned_path[i+1])
        p3 = point(pruned_path[i+2])
        
        # If the 3 points are in a line remove
        # the 2nd point.
        # The 3rd point now becomes and 2nd point
        # and the check is redone with a new third point
        # on the next iteration.
        if collinearity_check(p1, p2, p3):
            # Something subtle here but we can mutate
            # `pruned_path` freely because the length
            # of the list is check on every iteration.
            pruned_path.remove(pruned_path[i+1])
        else:
            i += 1
    return pruned_path

def raw_grid_method(grid, grid_start, grid_goal):
        path, _ = a_star(grid, heuristic, grid_start, grid_goal)
        print("path point num = {}, path={}".format(len(path), path))
        path = prune_path(path)
        print("pruned path point num = {}, path={}".format(len(path), path))
        return path