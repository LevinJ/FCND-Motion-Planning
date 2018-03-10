import numpy as np
import matplotlib.pyplot as plt
from grid import create_grid
from planning import a_star
import math



plt.rcParams['figure.figsize'] = 12, 12

# This is the same obstacle data from the previous lesson.
filename = './data/colliders.csv'
data = np.loadtxt(filename, delimiter=',', dtype='Float64', skiprows=2)
print(data)


# Static drone altitude (meters)
drone_altitude = 5

# Minimum distance stay away from obstacle (meters)
safe_distance = 3


# TODO: Use `create_grid` to create a grid configuration space of
# the obstacle data.
grid = create_grid(data, drone_altitude, safe_distance)

# equivalent to
# plt.imshow(np.flip(grid, 0))
plt.imshow(grid, origin='lower') 

plt.xlabel('EAST')
plt.ylabel('NORTH')
plt.show()

start_ne = (25,  100)
goal_ne = (750., 370.)

def heuristic_func(position, goal_position):
    h = math.fabs(position[0] - goal_position[0]) + math.fabs(position[1] - goal_position[1])
    return h

start, goal = start_ne, goal_ne
grid_flip = np.flip(grid, 0)
path, cost = (grid_flip, heuristic_func, start, goal)
print(len(path), cost)

plt.imshow(grid, cmap='Greys', origin='lower')

# For the purposes of the visual the east coordinate lay along
# the x-axis and the north coordinates long the y-axis.
plt.plot(start_ne[1], start_ne[0], 'x')
plt.plot(goal_ne[1], goal_ne[0], 'x')

pp = np.array(path)
plt.plot(pp[:, 1], pp[:, 0], 'g')

plt.xlabel('EAST')
plt.ylabel('NORTH')
plt.show()





