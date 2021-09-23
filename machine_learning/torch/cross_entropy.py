import math
import torch
from torch.nn import functional as F

import pdb
pdb.set_trace()

input = torch.randn(3, 5, requires_grad= True)
target = torch.randint(5, (3, ), dtype=torch.int64)
loss = F.cross_entropy(input, target)
"""
the computition is as follows:
for example:
    input is:
        tensor([[-0.6486, -2.4473,  0.3581,  1.1828, -0.0526],
            [-0.0951,  0.3171, -0.3763, -0.3765, -0.4585],
            [ 0.4125, -0.3339, -2.7247, -1.2282,  0.2043]], requires_grad=True)
    target is:
        tensor([4, 4, 0])
then the loss will be computed as follows:
    a1 = sum([math.exp(e) for e in input[0]])
    a2 = sum([math.exp(e) for e in input[1]])
    a3 = sum([math.exp(e) for e in input[2]])
    loss = -sum([math.log(math.exp(input[0][4]) / a1), math.log(math.exp(input[1][4]) / a2), math.log(math.exp(input[2][0]) / a3)]) / 3
    reference to https://blog.csdn.net/wuliBob/article/details/104119616 and https://www.cnblogs.com/marsggbo/p/10401215.html
"""

print(loss)
loss.backward()

assert(len(input) == len(target))
loss = 0
for (eles, idx) in zip(input, target):
    asum = sum([math.exp(e) for e in eles])
    loss += math.log(math.exp(eles[idx]) / asum)
loss = -loss / len(input)
print(loss)

