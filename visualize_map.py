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


# plt.rcParams["figure.figsize"] = [12, 12]


class VisualizeMap(object):
    def __init__(self):
        return
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
#         with open("colliders.csv", "r") as f:
#             first_line = f.readline()
#             searchObj = re.search( r'lat0 (.*), lon0 (.*)', first_line)
#         lat0, lon0 = float(searchObj.group(1)),float(searchObj.group(2))
#         # TODO: set home position to (lat0, lon0, 0)
#         self.set_home_position(lon0, lat0, 0)
# 
#         # TODO: retrieve current global position
#         global_position = self.global_position
#  
#         # TODO: convert to current local position using global_to_local()
#         local_position = global_to_local(global_position, self.global_home)
# #         assert(local_position == self.local_position)
#         
#         print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
#                                                                          self.local_position))
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
        
        # Define a grid for a particular altitude and safety margin around obstacles
        TARGET_ALTITUDE = 0.01
        SAFETY_DISTANCE = 5
        grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
        
        grid_start = (0-north_offset, 0-east_offset)
        grid_goal = (900 , 522)
#         self.show_map(grid, grid_start, grid_goal)
        
        print('Local Start and Goal: ', grid_start, grid_goal)
        path, _ = a_star(grid, heuristic, grid_start, grid_goal)
        print("path point num = {}, path={}".format(len(path), path))
        
        path = prune_path(path)
        print("pruned path point num = {}, path={}".format(len(path), path))
        self.show_map(grid, grid_start, grid_goal, path)
        
        
        return
  
    
if __name__ == "__main__":   
    obj= VisualizeMap()
    obj.run()
    
    
    