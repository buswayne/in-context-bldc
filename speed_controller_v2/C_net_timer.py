import os
from pathlib import Path
import time
import torch
import numpy as np
import math
import gc
from transformer_zerostep import GPTConfig, GPT, warmup_cosine_lr

import matplotlib.pyplot as plt
import ctypes

# # Overall settings
# out_dir = "out"

# model_name = "new_dataset_long_noise_h10_40k.pt"
# # model_name = "model_high_speed.pt"

# current_path = os.getcwd().split("in-context-bldc")[0]
# data_path = os.path.join(current_path,"in-context-bldc", "data")

# # folder = "simulated/50_percent_control_with_noise/validation"
# # folder = "simulated/50_percent_control/validation"
# # folder = "CL_experiments_double_sensor_control/test/inertia07"
# # folder_path = os.path.join(data_path, folder)


# # Compute settings
# cuda_device = "cuda:0"
# no_cuda = False
# threads = 10
# compile = False

# # Configure compute
# torch.set_num_threads(threads) 
# use_cuda = not no_cuda and torch.cuda.is_available()
# # device_name  =  cuda_device if use_cuda else "cpu"
# device_name  =  "cpu"
# device = torch.device(device_name)
# device_type = 'cuda' if 'cuda' in device_name else 'cpu' # for later use in torch.autocast
# torch.set_float32_matmul_precision("high")
# print(torch.cuda.is_available())
# # Create out dir
# out_dir = Path(out_dir)
# exp_data = torch.load(out_dir/model_name, map_location=device, weights_only=False)
# seq_len = exp_data["cfg"].seq_len
# nx = exp_data["cfg"].nx
# exp_data["iter_num"]


# model_args = exp_data["model_args"]
# gptconf = GPTConfig(**model_args)
# model = GPT(gptconf).to(device)
# print(model.get_num_params())

# state_dict = exp_data["model"]
# unwanted_prefix = '_orig_mod.'
# for k,v in list(state_dict.items()):
#     if k.startswith(unwanted_prefix):
#         state_dict[k[len(unwanted_prefix):]] = state_dict.pop(k)
#     if k.startswith('module.'):
#         state_dict[k[7:]] = v
#         state_dict.pop(k)

# model.load_state_dict(state_dict)






current_path = os.getcwd().split("speed_controller_v2")[0]


dll_dir = os.path.join(current_path,"speed_controller_v2", "C_libs")
  
# os.add_dll_directory(dll_dir)
# dll_dir_gomp = r"C:\Strawberry\c\bin"  
# os.add_dll_directory(dll_dir_gomp)

# --- Step 2: load the DLL ---
lib_path = os.path.join(dll_dir, "net_predict_L_40k_test_mo_openmp.dll")
lib = ctypes.CDLL(lib_path)

# --- Step 3: define function signatures ---
# Example: if your C function is
# double myFunction(double x, double y);
lib.net_predict_L_40k.restype = None
lib.net_predict_L_40k.argtypes = [
    np.ctypeslib.ndpointer(dtype=np.float64, ndim=1, flags="C_CONTIGUOUS"),  # input
    np.ctypeslib.ndpointer(dtype=np.float32, ndim=1, flags="C_CONTIGUOUS")   # output
]


print("ready to roll")

test_length = 1000
el_time_list = np.zeros(test_length)
y_out = np.zeros(10, dtype=np.float32)        # output array to be filled


x_in_big = np.random.rand(1,10,6)




#.astype(np.float64).flatten(order='F')
for i in range(10):
    # rand_in = torch.rand(1,10,6, device=device)
    # out = model(rand_in)
    x_in = np.random.rand(60).astype(np.float64)  # your input array
    lib.net_predict_L_40k(x_in, y_out)


for i in range(test_length):
    now = time.perf_counter_ns()
    # rand_in = torch.rand(1,10,6, device=device)
    # out = model(rand_in)
    
    # x_in = np.random.rand(60).astype(np.float64)  # your input array
    x_in_big[0,0:9,:] = x_in_big[0,1:10,:]
    x_in_big[0,9,:] = np.random.rand(6)
    x_in = x_in_big.astype(np.float64).flatten(order='F')
    lib.net_predict_L_40k(x_in, y_out)
    el_time_list[i] = time.perf_counter_ns()-now


# fig = plt.figure()
# plt.plot(el_time_list)

fig = plt.figure()
plt.plot(el_time_list*1e-9)

print(el_time_list.mean()*1e-9)

plt.show()