import numpy as np
import pandas as pd
import glob
import os
import torch
from torch.utils.data import Dataset, DataLoader
import matplotlib.pyplot as plt


folder_path = '../data/simulated/50_percent_longer_steps'

dataframes = []
# Use glob to find all CSV files in the specified folder
for file in glob.glob(os.path.join(folder_path, '*.csv')):
    df = pd.read_csv(file)
    try:
        df.columns = ['t', 'theta', 'omega', 'r', 'id', 'iq', 'iq_ref', 'vd', 'vq']
    except:
        pass
    dataframes.append(df)

