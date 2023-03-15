#-*- coding:utf-8 -*-
import torch
torch.cuda.empty_cache()
print('memory_allocated', torch.cuda.memory_allocated() / 1e9, 'memory_cached', torch.cuda.memory_cached() / 1e9)

# https://zhuanlan.zhihu.com/p/162144706
index = torch.tensor([range(100)]) # torch.gather(tensor, dim, index)
y = (index // 40).float()
x = (index % 40).float()
print(y, x)
xy = torch.cat([y[..., None], x[..., None]], dim = -1)
print(xy)
