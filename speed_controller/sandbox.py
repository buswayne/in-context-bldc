import numpy as np
import glob
import os
from torch.utils.data import Dataset, DataLoader
import matplotlib.pyplot as plt
import copy
import torch
# import onnx
# import onnxruntime
# import onnxscript
import pandas as pd
from transformer_zerostep import GPTConfig
# print(torch.__version__)
# print(onnx.__version__)
# print(onnxruntime.__version__)
# print(onnxscript.__version__)
from collections import OrderedDict
import seaborn as sns



# current_path = os.getcwd().split("in-context-bldc")[0]
# data_path = os.path.join(current_path,"in-context-bldc", "data")

# # folder = "CL_experiments_double_sensor_low_speed_ekf_and_meta/final/inertia13_ki-0.0029-kp-3.0000"
# folder = "simulated/50_percent_control"
# folder_path = os.path.join(data_path, folder)

# file_list = glob.glob(os.path.join(folder_path, '*.csv'))

# test_idx = 582
# df = pd.read_csv(file_list[test_idx])
# metadata = df.keys()[-1].split(',')
# print(metadata)
# T_ass = float(metadata[0].split(":")[1])
# S_pct = float(metadata[1].split(":")[1])
# Kp = float(metadata[2].split(":")[1])
# Ki = float(metadata[3].split(":")[1])
# # print(T_ass, S_pct, Kp, Ki)


# model=torch.jit.load("test_controller2.pt", map_location='cpu')
# input = torch.zeros((1,10,8)).to('cpu')
# print(input.device)
# # print(model)
# # print(model(input)[0,-1,0]*10+5)


# def bias_no_tril(block_size):
#     i = torch.arange(block_size).unsqueeze(0)
#     j = torch.arange(block_size).unsqueeze(1)
#     bias = (i <= j).float().view(1, 1, block_size, block_size)
#     return bias

# print(bias_no_tril(4))
# print(torch.tril(torch.ones(4,4)).view(1,1,4,4))


# model_args = dict(n_layer=8, n_head=4, n_embd=16, n_x=1, n_y=1, n_u=8, block_size=10,
#                       bias=False, dropout=0)  

# gptconf = GPTConfig(**model_args)
# keys_raw = gptconf.__dict__.keys()
# print(keys_raw)
# keys = [key for key in keys_raw if key[0] != '_']

# print(keys)

# gpt_dict_raw = gptconf.__dict__
# print(gpt_dict_raw)
# gpt_dict_ord = OrderedDict(gpt_dict_raw)
# print(gpt_dict_ord)
# # gpt_dict = gpt_dict_raw['block_size']
# print(gpt_dict)



# x = np.random.lognormal(0,1, 1000)
x = np.random.beta(a=0.8, b=4, size=10000) *40
plt.figure(figsize=(5,4))
# plt.hist(x, density=True)
sns.histplot(data=x, kde=True, stat="density")
plt.xlim((0,50))
plt.title("Overshoot distribution")
plt.xlabel("$OS_{\%}$")
plt.grid()
plt.tight_layout()



x = np.random.uniform(0,4, 10000)
plt.figure(figsize=(5,4))
# plt.hist(x, density=True)
sns.histplot(data=x, kde=True, stat="density")
plt.xlim((0,4.2))
plt.title("Settling time distribution")
plt.xlabel("$T_s$")
plt.tight_layout()
plt.grid()
plt.show()