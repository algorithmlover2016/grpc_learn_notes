# -*- coding:utf-8 -*-
# refer to https://tqdm.github.io/

from tqdm import tqdm

for i in tqdm(range(int(9e6))):
    pass

for i in tqdm(range(int(9e6)), ncols = 97):
    pass

for i in tqdm(range(int(9e6)), ascii = True, desc = "hello"):
    pass

# shell
# seq 99999999 | tqdm --unit_scale --initial 9999 | wc -l
# seq 99999999 | tqdm --unit_scale --total 9999 | wc -l