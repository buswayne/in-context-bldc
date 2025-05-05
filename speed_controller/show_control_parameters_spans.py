import numpy as np
import pandas as pd
import glob
import os
from torch.utils.data import Dataset, DataLoader
import matplotlib.pyplot as plt
import copy


current_path = os.getcwd().split("in-context-bldc")[0]
data_path = os.path.join(current_path,"in-context-bldc", "data")

# folder = "CL_experiments_double_sensor_low_speed_ekf_and_meta/final/inertia13_ki-0.0029-kp-3.0000"
folder = "simulated/50_percent_control"
folder_path = os.path.join(data_path, folder)

file_list = glob.glob(os.path.join(folder_path, '*.csv'))

metadata_matrix = np.zeros((len(file_list),4))
i = 0
for file in file_list:
    df = pd.read_csv(file)
    metadata = df.keys()[-1].split(',')
    # print(metadata)
    T_ass = float(metadata[0].split(":")[1])
    S_pct = float(metadata[1].split(":")[1])
    Kp = float(metadata[2].split(":")[1])
    Ki = float(metadata[3].split(":")[1])
    # print(T_ass, S_pct, Kp, Ki)
    metadata_matrix[i,:] = [T_ass, S_pct, Kp, Ki]
    i+=1

fig = plt.figure()
plt.scatter(metadata_matrix[:,0],metadata_matrix[:,1])
plt.xlabel("T_ass")
plt.ylabel("S_pct")


fig = plt.figure()
plt.scatter(metadata_matrix[:,2],metadata_matrix[:,3])
plt.xlabel("Kp")
plt.ylabel("Ki")
ax = plt.gca()
ax.set_xscale('log')
ax.set_yscale('log')

plt.show()






