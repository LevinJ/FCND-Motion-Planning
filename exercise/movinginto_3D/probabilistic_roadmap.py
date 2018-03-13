# Again, ugly but we need the latest version of networkx!
import sys
import pkg_resources
import networkx as nx

import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import Polygon, Point, LineString
from queue import PriorityQueue


# This is the same obstacle data from the previous lesson.
filename = '../data/colliders.csv'
data = np.loadtxt(filename, delimiter=',', dtype='Float64', skiprows=2)
print(data)

from sampling import Sampler
sampler = Sampler(data)
polygons = sampler._polygons

# Example: sampling 100 points and removing
# ones conflicting with obstacles.
nodes = sampler.sample(300)
print(len(nodes))