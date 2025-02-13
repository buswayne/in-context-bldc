import numpy as np
import matplotlib.pyplot as plt
import os
from pathlib import Path
import pandas as pd
import sys
import torch.nn as nn

import torch


aaa_val = [0,0,0,0,0,0,3,3,3,3,3,3,3,6,6,6,6,8,8,8,3,3,3]


aaa = pd.DataFrame(aaa_val,columns=(['test']))
print(aaa)

diff = aaa['test'].diff(-2).to_numpy()
diff = diff[~np.isnan(diff)]
print(diff)
# for _ in range(10):
#     print(np.random.rand())

idx = np.flatnonzero(diff != 0)
print(idx)
for _ in range(20):
    print(np.random.choice(idx))