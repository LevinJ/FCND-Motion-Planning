import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np

from planning_utils import a_star, heuristic, create_grid
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local
import re
import matplotlib.pyplot as plt
from mp_utils import prune_path

from udacidrone.frame_utils import global_to_local, local_to_global
from mp_utils import lontat2grid, raw_grid_method,media_axis_method, voronoi_method


# plt.rcParams["figure.figsize"] = [12, 12]


class VisualizeMap(object):
    def __init__(self):
        return
    
    def grid2lonlat(self, p, north_offset,east_offset,TARGET_ALTITUDE=5):
        home_lon, home_lat = -122.397450, 37.792480, 
        global_home = (home_lon, home_lat)
        local_position = [p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE]
        lonlat = local_to_global(local_position, global_home)
        print("global lon, lat={}".format(lonlat))
        
        return 
#     def lontat2grid(self, global_position, north_offset,east_offset, global_home):
#         location_position = global_to_local(global_position, global_home)
#         grid_position = (int(round(location_position[0]-north_offset)), int(round(location_position[1]-east_offset)))
#         return grid_position
    def show_map(self,grid,grid_start,grid_goal, path = None):
        plt.imshow(grid, origin='lower') 
        plt.scatter(grid_start[1], grid_start[0], c='red')
        plt.scatter(grid_goal[1], grid_goal[0], c='green')
        
        if path is not None:
            pp = np.array(path)
            plt.plot(pp[:, 1], pp[:, 0], 'g')
            plt.scatter(pp[:, 1], pp[:, 0],s=0.1,c='white')
         
        plt.xlabel('EAST')
        plt.ylabel('NORTH')
        plt.show()
        return
    
   
   
    def run(self):
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
        
        # Define a grid for a particular altitude and safety margin around obstacles
        TARGET_ALTITUDE = 0.01
        SAFETY_DISTANCE = 5
        grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
#         print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
        home_lon, home_lat = -122.397450, 37.792480, 
        self.global_home = (home_lon, home_lat)
        grid_start = (0-north_offset, 0-east_offset)
        
         
        grid_goal = (900 , 522)
        grid_goal = lontat2grid([-1.22396533e+02,  3.77977389e+01, -1.00000000e-02], north_offset, east_offset, self.global_home)
       
#         grid_goal = (276 , 116)
#         grid_goal = lontat2grid([-1.22401189e+02,  3.77921385e+01, -1.00000000e-02], north_offset, east_offset, self.global_home)
        
#         grid_goal = (531, 13)
#         grid_goal = lontat2grid([-1.22402341e+02,  3.77944427e+01, -1.00000000e-02], north_offset, east_offset, self.global_home)
        
        
        
        print("grid_start={}, grid_goal={}".format(grid_start, grid_goal))
        
        self.grid2lonlat(grid_goal, north_offset, east_offset, TARGET_ALTITUDE)
        
        
        
        b_show_map = False
        

        
        
        if b_show_map:
            self.show_map(grid, grid_start, grid_goal)
            return
    
        

#         path = raw_grid_method(grid, grid_start, grid_goal, check_linear = False)
        
        
#         path, skeleton, start_ne, goal_ne = media_axis_method(grid, grid_start, grid_goal, check_linear = False)
#         self.show_media_axis(grid, skeleton, start_ne, goal_ne, path)
       
        
        
        path, edges, start_ne, start_ne_g, goal_ne, goal_ne_g = voronoi_method(data, TARGET_ALTITUDE, SAFETY_DISTANCE, grid_start, grid_goal, check_linear = False)
        self.show_voronoi(grid, edges, start_ne, start_ne_g, goal_ne, goal_ne_g, path)
        
        
        if len(path) ==0:
            print("failed to find the path!!!")
            return
        self.show_map(grid, grid_start, grid_goal, path)
        
        
        return
    def show_media_axis(self,grid,skeleton,start_ne,goal_ne,path):
        plt.imshow(grid, cmap='Greys', origin='lower')
        plt.imshow(skeleton, cmap='Greys', origin='lower', alpha=0.7)
        # For the purposes of the visual the east coordinate lay along
        # the x-axis and the north coordinates long the y-axis.
        plt.plot(start_ne[1], start_ne[0], 'x')
        plt.plot(goal_ne[1], goal_ne[0], 'x')
        
        pp = np.array(path)
        plt.plot(pp[:, 1], pp[:, 0], 'g')
       
        plt.xlabel('EAST')
        plt.ylabel('NORTH')
        plt.show()
        return
    def show_voronoi(self, grid, edges,start_ne, start_ne_g, goal_ne,goal_ne_g, path):
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
            
        plt.plot(start_ne_g[1], start_ne_g[0], 'gx')
        plt.plot(goal_ne_g[1], goal_ne_g[0], 'gx')
        
        plt.xlabel('EAST', fontsize=20)
        plt.ylabel('NORTH', fontsize=20)
        plt.show()
        return
  
    
if __name__ == "__main__":   
    obj= VisualizeMap()
    obj.run()
    
    
    