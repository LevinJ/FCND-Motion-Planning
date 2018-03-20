import numpy as np
import matplotlib.pyplot as plt
from exercise.grid import create_grid
from exercise.planning import a_star

plt.rcParams['figure.figsize'] = 12, 12


# This is the same obstacle data from the previous lesson.
filename = '../data/colliders.csv'
data = np.loadtxt(filename, delimiter=',', dtype='Float64', skiprows=2)
print(data)

# Static drone altitude (meters)
drone_altitude = 5

# Minimum distance stay away from obstacle (meters)
safe_distance = 3


grid = create_grid(data, drone_altitude, safe_distance)


# plt.imshow(grid, origin='lower') 
# 
# plt.xlabel('EAST')
# plt.ylabel('NORTH')
# plt.show()

start_ne = (25,  100)
goal_ne = (750., 370.)


def heuristic_func(position, goal_position):
    return np.abs(position[0] - goal_position[0]) + np.abs(position[1] - goal_position[1])



path, cost = a_star(grid, heuristic_func, start_ne, goal_ne)
print(len(path), cost)



# plt.imshow(grid, cmap='Greys', origin='lower')
# 
# # For the purposes of the visual the east coordinate lay along
# # the x-axis and the north coordinates long the y-axis.
# plt.plot(start_ne[1], start_ne[0], 'x')
# plt.plot(goal_ne[1], goal_ne[0], 'x')
# 
# pp = np.array(path)
# plt.plot(pp[:, 1], pp[:, 0], 'g')
# 
# plt.xlabel('EAST')
# plt.ylabel('NORTH')
# plt.show()



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

pruned_path = prune_path(path)
print(len(pruned_path))



plt.imshow(grid, cmap='Greys', origin='lower')

plt.plot(start_ne[1], start_ne[0], 'x')
plt.plot(goal_ne[1], goal_ne[0], 'x')

pp = np.array(pruned_path)
plt.plot(pp[:, 1], pp[:, 0], 'g')
plt.scatter(pp[:, 1], pp[:, 0])

plt.xlabel('EAST')
plt.ylabel('NORTH')

plt.show()

