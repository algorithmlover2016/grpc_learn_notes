# -*- coding:utf-8 -*-

import numpy as np
import math
import matplotlib.pyplot as plt
import torch

def gelu(x):
    """Implementation of the gelu activation function.
        For information: OpenAI GPT's gelu is slightly different (and gives slightly different results):
        0.5 * x * (1 + torch.tanh(math.sqrt(2 / math.pi) * (x + 0.044715 * torch.pow(x, 3))))
        Also see https://arxiv.org/abs/1606.08415
    """
    return x * 0.5 * (1.0 + torch.erf(x / math.sqrt(2.0)))

x = np.arange(-2, 3, 0.001)
y = []
for t in x:
    y .append(gelu(torch.tensor(t)))

plt.plot(x, x, label = "y=x")
plt.plot(x, y, label = "GELU")
plt.xlabel("x")
plt.ylabel("y")
plt.ylim(-1, 3)
plt.legend()
plt.grid()
plt.title("x * 0.5 * (1.0 + torch.erf(x / math.sqrt(2.0)))")
plt.show()
