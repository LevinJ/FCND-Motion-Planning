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
from mp_utils import lontat2grid, raw_grid_method,media_axis_method


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
        global_home = (home_lon, home_lat)
        grid_start = (0-north_offset, 0-east_offset)
#         grid_goal = (900 , 522)
#         grid_goal = lontat2grid([-1.22396533e+02,  3.77977389e+01, -1.00000000e-02], north_offset, east_offset, global_home)
       
        grid_goal = (276 , 116)
        grid_goal = lontat2grid([-1.22401189e+02,  3.77921385e+01, -1.00000000e-02], north_offset, east_offset, global_home)
        print("grid_start={}, grid_goal={}".format(grid_start, grid_goal))
        self.grid2lonlat(grid_goal, north_offset, east_offset, TARGET_ALTITUDE)
        
        
        
        b_show_map = False
        

        
        
        if b_show_map:
            self.show_map(grid, grid_start, grid_goal)
            return
    
        

#         path = raw_grid_method(grid, grid_start, grid_goal, check_linear = False)
        path = media_axis_method(grid, grid_start, grid_goal, check_linear = False)
        
        
        if len(path) ==0:
            print("failed to find the path!!!")
            return
        self.show_map(grid, grid_start, grid_goal, path)
        
        
        return
  
    
if __name__ == "__main__":   
    obj= VisualizeMap()
    obj.run()
    
    
    