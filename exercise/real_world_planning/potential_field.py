import numpy as np 
import matplotlib.pyplot as plt
import math


plt.rcParams['figure.figsize'] = 12, 12




def attraction(position, goal, alpha):
    # TODO: implement attraction force
    fx = alpha * (position[0] - goal[0])
    fy = alpha * (position[1] - goal[1])
    return np.array([fx,fy])


def repulsion(position, obstacle, beta, q_max):
    # TODO: implement replusion force
    distance = math.fabs(position[0] - obstacle[0])
    if distance > q_max:
        return np.array([0,0])
    elif distance == 0:
        distance = 0.00001
    fx = beta * (1.0/q_max - 1.0/distance)  * 1.0/(distance ** 2)
    
    distance = math.fabs(position[1] - obstacle[1])
    if distance > q_max:
        return np.array([0,0])
    elif distance == 0:
        distance = 0.00001
    fy = beta * (1.0/q_max - 1.0/distance)  * 1.0/(distance ** 2)
    
    return np.array([fx,fy])



def potential_field(grid, goal, alpha, beta, q_max):
    x = []
    y = []
    fx = []
    fy = []
    
    obs_i, obs_j = np.where(grid == 1)

    for i in range(grid.shape[0]):
        for j in range(grid.shape[1]):
            if grid[i, j] == 0:
                
                # add attraction force
                force = attraction([i, j], goal, alpha)

                for (oi, oj) in zip(obs_i, obs_j):
                    if np.linalg.norm(np.array([i, j]) - np.array([oi, oj])) < q_max:
                        # add replusion force
                        force += repulsion([i, j], [oi, oj], beta, q_max)
                    
                x.append(i)
                y.append(j)
                fx.append(force[0])
                fy.append(force[1])

    return x, y, fx, fy

# generate environment
grid = np.zeros((30, 30))
grid[10:15,10:15] = 1.0
grid[17:25,10:17] = 1.0

goal  = [5, 5]

# constants
alpha = 1.0
beta = 2.0
q_max = 10


x, y, fx, fy = potential_field(grid, goal, alpha, beta, q_max)




plt.imshow(grid, cmap = 'Greys', origin='lower')
plt.plot(goal[1], goal[0], 'ro')
plt.quiver(y, x, fy, fx)
plt.xlabel('X')
plt.ylabel('Y')
plt.show()









