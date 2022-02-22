# -*- coding:utf-8 -*-
import numpy as np
import math
import matplotlib.pyplot as plt
import torch

def gelu(x, isMultiply = True):
    """Implementation of the gelu activation function.
        For information: OpenAI GPT's gelu is slightly different (and gives slightly different results):
        0.5 * x * (1 + torch.tanh(math.sqrt(2 / math.pi) * (x + 0.044715 * torch.pow(x, 3))))
        Also see https://arxiv.org/abs/1606.08415
    """
    if isMultiply:
        return x * 0.5 * (1.0 + torch.erf(x / math.sqrt(2.0)))
    else:
        return 0.5 * (1.0 + torch.erf(x / math.sqrt(2.0)))

def relu(x, lamda = 0.1):
    return x if x > 0 else lamda * x

# reference to https://zhuanlan.zhihu.com/p/75588369
def sigmoid(x):
    return 1.0 / (1.0 + math.exp(-x))

def sin(x):
    return np.sin(x)

def cos(x):
    return np.cos(x)

def lanczos(x, a = 2):
    """
            sinc(x) * sinc(x/a); if -a < x < a
    L(x) =
            0 otherwise
    
    Equivalently,

            1 if x = 0,
    L(x) =  a * sin(pi * x) * sin(pi * x / a) / (pi^2 * x^2) if -a <= x < a and x != 0
            0 otherwise

    """
    return np.sinc(x) * np.sinc(x / a)


def main():
    x = np.arange(-2, 3, 0.001)
    yGelu = []
    yRelu = []
    ySigmoid = []
    ySin = []
    yCos = []
    yLanczos = []
    for t in x:
        yGelu .append(gelu(torch.tensor(t), False))
        yRelu .append(relu(torch.tensor(t), 0.1))
        ySigmoid.append(sigmoid(torch.tensor(t)))
        ySin.append(sin(torch.tensor(t)))
        yCos.append(cos(torch.tensor(t)))
        yLanczos.append(lanczos(torch.tensor(t)))
    plt.plot(x, x, label = "y=x")
    plt.plot(x, yGelu, label = "GELU")
    plt.plot(x, yRelu, label = "RELU")
    plt.plot(x, ySigmoid, label = "Sigmoid")
    plt.plot(x, ySin, label = "Sin")
    plt.plot(x, yCos, label = "Cos")
    plt.plot(x, yLanczos, label = "Lanczos")
    plt.xlabel("x")
    plt.ylabel("y")
    plt.ylim(-1, 3)
    plt.legend()
    plt.grid()
    plt.title("curves")
    plt.show()

if __name__ == "__main__":
    main()
