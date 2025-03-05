import numpy as np
import matplotlib.pyplot as plt
import os
from pathlib import Path
import pandas as pd
import sys
import torch.nn as nn

import torch
import copy


aaa_val = np.arange(7,12)


aaa = pd.DataFrame(aaa_val,columns=(['test']))
print(aaa)


tmp = copy.deepcopy(aaa['test'].to_numpy())
print(tmp)
tmp[1:-1] = tmp[0:-2]
print(tmp)
tmp[0] = 0
print(tmp)
aaa['test_2'] = tmp
print(aaa)

# diff = aaa['test'].diff(-2).to_numpy()
# diff = diff[~np.isnan(diff)]
# print(diff)
# # for _ in range(10):
# #     print(np.random.rand())

# idx = np.flatnonzero(diff != 0)
# print(idx)
# for _ in range(20):
#     print(np.random.choice(idx))