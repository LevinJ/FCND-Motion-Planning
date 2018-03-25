import numpy as np
from udacidrone.frame_utils import global_to_local, local_to_global
from planning_utils import a_star, heuristic, create_grid
from skimage.morphology import medial_axis
from skimage.util import invert
from scipy.spatial import Voronoi, voronoi_plot_2d
from bresenham import bresenham
import networkx as nx
import numpy.linalg as LA
import a_star_graph


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
    

# check_linear is a flag which allows us to choose either collinearity test or Bresenham
#the grid in only used for Bresenham method
def prune_path(path, grid = None, check_linear = True):
    pruned_path = [p for p in path]
    
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



def find_start_goal(skel, start, goal):
    skel_cells = np.transpose(skel.nonzero())
    start_min_dist = np.linalg.norm(np.array(start) - np.array(skel_cells), axis=1).argmin()
    near_start = skel_cells[start_min_dist]
    goal_min_dist = np.linalg.norm(np.array(goal) - np.array(skel_cells), axis=1).argmin()
    near_goal = skel_cells[goal_min_dist]
    
    return near_start, near_goal


def create_grid_and_edges(data, drone_altitude, safety_distance):
    """
    Returns a grid representation of a 2D configuration space
    along with Voronoi graph edges given obstacle data and the
    drone's altitude.
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
    
    # Define a list to hold Voronoi points
    points = []
    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        if alt + d_alt + safety_distance > drone_altitude:
            obstacle = [
                int(np.clip(north - d_north - safety_distance - north_min, 0, north_size-1)),
                int(np.clip(north + d_north + safety_distance - north_min, 0, north_size-1)),
                int(np.clip(east - d_east - safety_distance - east_min, 0, east_size-1)),
                int(np.clip(east + d_east + safety_distance - east_min, 0, east_size-1)),
            ]
            grid[obstacle[0]:obstacle[1]+1, obstacle[2]:obstacle[3]+1] = 1
            # add center of obstacles to points list
            points.append([north - north_min, east - east_min])
            
    # TODO: create a voronoi graph based on
    # location of obstacle centres
    graph = Voronoi(points)

    # TODO: check each edge from graph.ridge_vertices for collision
    edges = []
    for v in graph.ridge_vertices:
        p1 = graph.vertices[v[0]]
        p2 = graph.vertices[v[1]]
        cells = list(bresenham(int(p1[0]), int(p1[1]), int(p2[0]), int(p2[1])))
        hit = False

        for c in cells:
            # First check if we're off the map
            if np.amin(c) < 0 or c[0] >= grid.shape[0] or c[1] >= grid.shape[1]:
                hit = True
                break
            # Next check if we're in collision
            if grid[c[0], c[1]] == 1:
                hit = True
                break

        # If the edge does not hit on obstacle
        # add it to the list
        if not hit:
            # array to tuple for future graph creation step)
            p1 = (p1[0], p1[1])
            p2 = (p2[0], p2[1])
            edges.append((p1, p2))

    return grid, edges
def raw_grid_method(grid, grid_start, grid_goal, check_linear = False):
    path, _ = a_star(grid, heuristic, grid_start, grid_goal)
    print("path point num = {}, path={}".format(len(path), path))
    path = prune_path(path, grid = grid, check_linear = check_linear)
    print("pruned path point num = {}, path={}".format(len(path), path))
    return path

def media_axis_method(grid, grid_start, grid_goal, check_linear = True):
    #check_linear specify how we do path pruning, if True, we choose colinearity
    #if False, we use bresenham, usually bresenham yield a much better result
    skeleton = medial_axis(invert(grid))
    skel_start, skel_goal = find_start_goal(skeleton, grid_start, grid_goal)
    path, _ = a_star(invert(skeleton).astype(np.int), heuristic, tuple(skel_start), tuple(skel_goal))
    if len(path) == 0:
#         print("warning, no path is found, please select another point")
        return path
    
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
    return path, skeleton, grid_start, grid_goal 

def voronoi_method(data, drone_altitude, safety_distance, start_ne, goal_ne, check_linear = True):
    grid, edges = create_grid_and_edges(data, drone_altitude, safety_distance)
    G = nx.Graph()
    for e in edges:
        p1 = e[0]
        p2 = e[1]
        dist = LA.norm(np.array(p2) - np.array(p1))
        G.add_edge(p1, p2, weight=dist)
    
    start_ne_g = a_star_graph.closest_point(G, start_ne)
    goal_ne_g = a_star_graph.closest_point(G, goal_ne)
    print(start_ne_g)
    print(goal_ne_g)
    
    path, _ = a_star_graph.a_star(G, heuristic, start_ne_g, goal_ne_g)
    if len(path) == 0:
#         print("warning, no path is found, please select another point")
        return path,edges, start_ne, start_ne_g, goal_ne, goal_ne_g
    
    #insert the start and end point if necessary
    if tuple(start_ne_g) != start_ne:
        path.insert(0, start_ne)
    if tuple(goal_ne_g) != goal_ne:
        path.append(goal_ne)
    
    #prune the path
    print("path point num = {}, path={}".format(len(path), path))
    path = prune_path(path, grid = grid, check_linear = check_linear)
    print("pruned path point num = {}, path={}".format(len(path), path))
    path = np.array(path).astype(int).tolist()
    return path,edges, start_ne, start_ne_g, goal_ne, goal_ne_g 


    
    