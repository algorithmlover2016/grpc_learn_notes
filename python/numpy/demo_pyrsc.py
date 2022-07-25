#-*- coding:utf-8 -*-

import numpy as np
import pyransac3d as pyrsc
import cv2
from pathlib import Path
import pdb

from torch import int8
pdb.set_trace()

for filename in ["./points_222-depth_k.npz", "./points_333-depth_k.npz"]:
    data = np.load(filename)
    all = data['all_points']
    index = data['index']
    k = data['k']
    points = all[index == False]
    all_points_len = points.shape[0]
    print(all.shape, index.shape, points.shape)
    plane = pyrsc.Plane()
    # best_inliers = np.arange(all_points_len)
    for i in range(2):
        thresh = np.sqrt(sum(np.sqrt(sum((points - points.mean())**2)/(points.shape[0] - 1)) ** 2)) / 16
        best_eq, best_inliers = plane.fit(points, max(thresh, 1), maxIteration = 10000)
        print(best_eq, best_inliers.shape, "%.4f" % thresh, "{:.4f}".format(best_inliers.shape[0]/all_points_len * 100))
        points = points[best_inliers]
    print(f"final {filename}: {best_eq}, {best_inliers.shape}, {best_inliers.shape[0]/all_points_len * 100:.4f}")
    height, width = index.shape
    points[:, 0] = np.rint(points[:, 0] / points[:, 2] * k + width / 2)
    points[:, 1] = np.rint(points[:, 1] / points[:, 2] * k + height / 2)
    points = points[:, :-1].astype(int)
    points[:, 0][points[:, 0] >= width] = width - 1
    points[:, 1][points[:, 1] >= height] = height - 1
    print(points.shape)
    im = cv2.imread(f"{Path(filename).stem}.png")
    print(im.shape)
    for x, y in points.tolist():
        im[y][x] = (255, 255, 255)
    cv2.imwrite(f"white_{Path(filename).stem}.png", im)
    # np.sqrt(sum((points - points.mean())**2)/points.shape[0] - 1)
    # np.sqrt(sum(np.sqrt(sum((points - points.mean())**2)/(points.shape[0] - 1)) ** 2))