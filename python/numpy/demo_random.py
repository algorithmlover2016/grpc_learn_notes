#-*- coding:utf-8 -*-
import torch

# https://zhuanlan.zhihu.com/p/162144706
index = torch.tensor([range(100)])
y = (index // 40).float()
x = (index % 40).float()
print(y, x)
xy = torch.cat([y[..., None], x[..., None]], dim = -1)
print(xy)