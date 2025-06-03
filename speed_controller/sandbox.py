import numpy as np
import glob
import os
from torch.utils.data import Dataset, DataLoader
import matplotlib.pyplot as plt
import copy
import torch
import onnx
import onnxruntime
import onnxscript
import pandas as pd
print(torch.__version__)
print(onnx.__version__)
print(onnxruntime.__version__)
print(onnxscript.__version__)

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


model=torch.jit.load("test_controller2.pt", map_location='cpu')
input = torch.zeros((1,10,8)).to('cpu')
print(input.device)
# print(model)
print(model(input)[0,-1,0]*10+5)


