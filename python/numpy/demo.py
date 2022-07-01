from math import floor
import random
import numpy as np
import logging
import math
random_arr = np.random.rand(10, 7)
print(random_arr)
round_arr = random_arr >= 0.5
print(round_arr)
print(random_arr[round_arr])

around_arr = np.around(random_arr)
print(around_arr)

point = [3.4, 5.7]
def get_near_points(point):
    if len(point) < 2:
        logging.warning(f"input point with error shape, the point is {point}")
        return 1
    pf1, pf2 = math.floor(point[0]), math.floor(point[1])
    pc1, pc2 = math.ceil(point[0]), math.ceil(point[1])
    return ((pf1, pf2), (pc1, pf2), (pc1, pc2), (pf1, pc2))
box_point = get_near_points(point)
print(box_point)
box_point_arr = np.asarray(box_point)
print(box_point_arr)
# it will print rows from box_point_arr[i][0] to box_point_arr[i][1]
print(random_arr[box_point_arr])

# it will print first row  that box_point_arr[i][0] box_point_arr[i][1]
print(np.take(random_arr, indices = box_point_arr))

# take the target point value
print(random_arr[np.ix_([box_point_arr[0][0]], [box_point_arr[0][1]])])

# take all the nearby value
box_point_transpose = np.asarray(box_point).T.tolist()
print("Transpose: ", box_point_transpose)
point_nearby_val = random_arr[tuple(box_point_transpose)]
print(point_nearby_val)
val = point_nearby_val > 0.5
print("points: ",  point_nearby_val[val])


from tempfile import TemporaryFile
outfile = TemporaryFile()
x1 = np.random.rand(10, 10)

x2 = np.random.rand(10, 1)
ans = [x1, x2]
np.savez("./ans.npz", x1=x1, x2 = x2)
load_ans = np.load("./ans.npz")
print([load_ans.keys()])
print(load_ans['x1'].shape)
print(load_ans['x2'].shape)

data = [np.arange(100).reshape(20, 5), np.arange(120).reshape(30, 4)]
np.savez("./data.npz", *data)

container = np.load("./data.npz")
print(type(container), len(container))
load_data = []
for key in container:
    print("key with npz:", key)
    print(container[key].shape)
    load_data.append(container[key])

import pickle

data = [np.arange(100).reshape(20, 5), np.arange(120).reshape(30, 4)]
with open("./data.pkl", 'wb') as outfile:
    pickle.dump(data, outfile, pickle.HIGHEST_PROTOCOL)

with open("./data.pkl", 'rb') as infile:
    pkl_load_data = pickle.load(infile)
print(type(pkl_load_data))
for ele in pkl_load_data:
    print(ele.shape)

import matplotlib as mpl
import matplotlib.pyplot as plt
import skimage.io
PLTOPTS = {"color": "#33FFFF", "s": 15, "edgecolors": "none", "zorder": 5}
cmap = plt.get_cmap("jet")
norm = mpl.colors.Normalize(vmin=0.9, vmax=1.0)
sm = plt.cm.ScalarMappable(cmap=cmap, norm=norm)
sm.set_array([])


def c(x):
    return sm.to_rgba(x)

def plot():
    import pdb
    # pdb.set_trace()
    imname = "./xiaoxingfu-2.png"
    im = skimage.io.imread(imname)
    if im.ndim == 2:
        im = np.repeat(im[:, :, None], 3, 2)
    im = im[:, :, :3]

    load_ans = np.load("./lcnn_data.npz")
    nlines, nscores = load_ans['nlines'], load_ans['nscores']
    print(nlines.shape, nscores.shape)
    for i, t in enumerate([0.99, 0.991, 0.992, 0.993, 0.994, 0.995, 0.996, 0.997, 0.998, 0.999]):
        plt.gca().set_axis_off()
        plt.subplots_adjust(top=1, bottom=0, right=1, left=0, hspace=0, wspace=0)
        plt.margins(0, 0)
        plt.imshow(im)
        for (a, b), s in zip(nlines, nscores):
            if s < t:
                continue
            plt.plot([a[1], b[1]], [a[0], b[0]], c=c(s), linewidth=2, zorder=s)
            plt.scatter(a[1], a[0], **PLTOPTS)
            plt.scatter(b[1], b[0], **PLTOPTS)
        plt.gca().xaxis.set_major_locator(plt.NullLocator())
        plt.gca().yaxis.set_major_locator(plt.NullLocator())
        plt.savefig(imname.replace(".png", f"-{t:.04f}.svg"), bbox_inches="tight")
        plt.show()

plot()


# np.select
aa = np.random.rand(13, 7)
score = np.random.rand(13, 1)
bb = np.select([score > 0.5], [aa])
# same to bb = np.select([score > 0.5], [aa], 0)
print(bb)
bb = np.select([score > 0.5], [aa], None)
# same to bb = np.select([score > 0.5, score <= 0.5], [aa, None])
print(bb)

bb = aa[(score > 0.5).flatten()]
print(bb)