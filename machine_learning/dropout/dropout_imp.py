# -*- coding:utf-8 -*-

import numpy as np

def dropout(x, level):
    if level < 0 or level >= 1:
        raise ValueError("Dropout Level must be in interval[0, 1)")
    retain_prob = 1. - level

    random_tensor = np.random.binomial(n = 1, p = retain_prob, size = x.shape)
    print(random_tensor)

    x *= random_tensor
    print(x)

    x /= retain_prob
    print(x)
    return x

x = np.array([1, 2, 3, 4, 5, 6, 7, 8, 9], dtype = np.float32)
dropout(x, 0.4)
