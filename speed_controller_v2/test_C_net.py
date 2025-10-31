import os
import ctypes
import numpy as np  

from pathlib import Path
import time
import torch
import math
import gc
from transformer_zerostep import GPTConfig, GPT, warmup_cosine_lr

current_path = os.getcwd().split("speed_controller_v2")[0]


dll_dir = os.path.join(current_path,"speed_controller_v2", "C_libs")
  
os.add_dll_directory(dll_dir)
# dll_dir_gomp = r"C:\Strawberry\c\bin"  
# os.add_dll_directory(dll_dir_gomp)

# --- Step 2: load the DLL ---
lib_path = os.path.join(dll_dir, "net_predict_L_40k.dll")
lib = ctypes.CDLL(lib_path)

# --- Step 3: define function signatures ---
# Example: if your C function is
# double myFunction(double x, double y);
lib.net_predict_L_40k.restype = None
lib.net_predict_L_40k.argtypes = [
    np.ctypeslib.ndpointer(dtype=np.float64, ndim=1, flags="C_CONTIGUOUS"),  # input
    np.ctypeslib.ndpointer(dtype=np.float32, ndim=1, flags="C_CONTIGUOUS")   # output
]

x_in = np.random.rand(60).astype(np.float64)  # your input array
y_out = np.zeros(10, dtype=np.float32)        # output array to be filled


lib.net_predict_L_40k(x_in, y_out)

print("Output:", y_out)


# Overall settings
out_dir = "out"

model_name = "new_dataset_long_noise_h10_40k.pt"
# model_name = "model_high_speed.pt"

current_path = os.getcwd().split("in-context-bldc")[0]
data_path = os.path.join(current_path,"in-context-bldc", "data")

# folder = "simulated/50_percent_control_with_noise/validation"
# folder = "simulated/50_percent_control/validation"
# folder = "CL_experiments_double_sensor_control/test/inertia07"
# folder_path = os.path.join(data_path, folder)


# Compute settings
cuda_device = "cuda:0"
no_cuda = False
threads = 10
compile = False

# Configure compute
torch.set_num_threads(threads) 
use_cuda = not no_cuda and torch.cuda.is_available()
# device_name  =  cuda_device if use_cuda else "cpu"
device_name  =  "cpu"
device = torch.device(device_name)
device_type = 'cuda' if 'cuda' in device_name else 'cpu' # for later use in torch.autocast
torch.set_float32_matmul_precision("high")
print(torch.cuda.is_available())
# Create out dir
out_dir = Path(out_dir)
exp_data = torch.load(out_dir/model_name, map_location=device, weights_only=False)
seq_len = exp_data["cfg"].seq_len
nx = exp_data["cfg"].nx
exp_data["iter_num"]


model_args = exp_data["model_args"]
gptconf = GPTConfig(**model_args)
model = GPT(gptconf).to(device)
print(model.get_num_params())

state_dict = exp_data["model"]
unwanted_prefix = '_orig_mod.'
for k,v in list(state_dict.items()):
    if k.startswith(unwanted_prefix):
        state_dict[k[len(unwanted_prefix):]] = state_dict.pop(k)
    if k.startswith('module.'):
        state_dict[k[7:]] = v
        state_dict.pop(k)

model.load_state_dict(state_dict)


print("ready to roll")





rand_in = torch.rand(1,10,6, device=device)


out_python = model(rand_in).detach().numpy().flatten()


rand_in_C = rand_in.detach().numpy().astype(np.float64).flatten(order='F')
y_out = np.zeros(10, dtype=np.float32)        # output array to be filled

lib.net_predict_L_40k(rand_in_C, y_out)

print(out_python)
print(y_out)

print(np.sqrt(np.mean((out_python-y_out)**2)))

