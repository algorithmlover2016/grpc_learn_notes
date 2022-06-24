import os
import sys
import pandas as pd
import numpy as np
from descartes import PolygonPatch
from shapely.geometry import Polygon, mapping, LineString, Point
import matplotlib.pyplot as plt
sys.path.insert(0, os.path.dirname(os.getcwd()))
import alphashape
import pdb

# pdb.set_trace()
# https://pypi.org/project/alphashape/

points_2ds = [
            [(0., 0.), (0., 1.), (1., 1.), (1., 0.), (0.5, 0.25), (0.5, 0.75), (0.25, 0.5), (0.75, 0.5)],
            [(0., 0.), (0., 1.), (1., 1.)],
            [(0., 0.), (0., 1.)],
            [(0., 0.)]]
for points_2d in points_2ds:
    alpha_shape = alphashape.alphashape(points_2d, 3)
    print(mapping(alpha_shape))
    if isinstance(alpha_shape, Polygon):
        fig, ax = plt.subplots()
        ax.scatter(*zip(*points_2d))
        ax.add_patch(PolygonPatch(alpha_shape, alpha=0.2))
        plt.show()
