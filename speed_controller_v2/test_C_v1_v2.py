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
lib1_path = os.path.join(dll_dir, "net_predict_S_30k.dll")
lib1 = ctypes.CDLL(lib1_path)

transformer_function1 = lib1.net_predict_S_30k
# --- Step 3: define function signatures ---
# Example: if your C function is
# double myFunction(double x, double y);
transformer_function1.restype = None
transformer_function1.argtypes = [
    np.ctypeslib.ndpointer(dtype=np.float64, ndim=1, flags="C_CONTIGUOUS"),  # input
    np.ctypeslib.ndpointer(dtype=np.float32, ndim=1, flags="C_CONTIGUOUS")   # output
]



# --- Step 2: load the DLL ---
lib2_path = os.path.join(dll_dir, "test_l2_e32_h10.dll")
lib2 = ctypes.CDLL(lib2_path)

transformer_function2 = lib2.gpt_forward
# --- Step 3: define function signatures ---
# Example: if your C function is
# double myFunction(double x, double y);
transformer_function2.restype = None
transformer_function2.argtypes = [
    np.ctypeslib.ndpointer(dtype=np.float32, ndim=1, flags="C_CONTIGUOUS"),  # input
    np.ctypeslib.ndpointer(dtype=np.float32, ndim=1, flags="C_CONTIGUOUS")   # output
]


input_matrix = np.random.rand(1,10,6)
y_out1 = np.zeros(10, dtype=np.float32)
y_out2 = np.zeros(10, dtype=np.float32)

### matlab_lib:

net_in1 = input_matrix.astype(np.float64).flatten(order='F')
transformer_function1(net_in1, y_out1)


### new lib:
net_in2 = input_matrix[0].astype(np.float32).flatten(order='C')
# net_in2 = input_matrix.astype(np.float32).flatten(order='F')
transformer_function2(net_in2, y_out2)



print(np.sqrt(np.mean((y_out1-y_out2)**2)))





