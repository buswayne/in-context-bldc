import numpy as np
import matplotlib.pyplot as plt
import os
from pathlib import Path
import pandas as pd
import sys
import torch.nn as nn

import torch
import copy


aaa = torch.ones(2,10,4)
aaa[0,:,0] = torch.range(1,10)
print(aaa.diff(dim=1))
# aaa_val = np.arange(12)

# print(aaa_val)
# print(aaa_val[3:6])
# print(aaa_val[5])


# diff = aaa['test'].diff(-2).to_numpy()
# diff = diff[~np.isnan(diff)]
# print(diff)
# # for _ in range(10):
# #     print(np.random.rand())

# idx = np.flatnonzero(diff != 0)
# print(idx)
# for _ in range(20):
#     print(np.random.choice(idx))