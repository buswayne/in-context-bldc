import numpy as np
import pandas as pd
import glob
import os
import torch
from torch.utils.data import Dataset, DataLoader
import matplotlib.pyplot as plt
import copy
# import logging
# from bldc_simulator_OL_V import BLDCMotor
# from signals import steps_sequence

# Assuming your DataFrame is named df and contains 6 columns
class Dataset(Dataset):
    def __init__(self, dfs, seq_len):
        self.dfs = dfs
        self.seq_len = seq_len

    def __len__(self):
        return 512

    def __getitem__(self, idx):
        # Randomly select a DataFrame
        df_idx = np.random.choice(len(self.dfs))
        df = self.dfs[df_idx]
        # print(len(df))

        # difference between starting time and ending time of the batch
        diff_array = df['r'].diff(-self.seq_len).to_numpy()
        diff_array = diff_array[~np.isnan(diff_array)]
        # print(len(diff_array))
        prob_ratio = 0.5 # ratio between constant samples and step samples
        if np.random.rand() >= prob_ratio:
            good_idx = np.flatnonzero(diff_array == 0)
            if len(good_idx) == 0:
                good_idx = np.flatnonzero(diff_array != 0)
        else:
            good_idx = np.flatnonzero(diff_array != 0)
            if len(good_idx) == 0:
                good_idx = np.flatnonzero(diff_array == 0)

        # Randomly select a starting index
        # max_val = len(df) - self.seq_len
        # start_idx = np.random.randint(0, max_val)
        start_idx = np.random.choice(good_idx)
        tmp = copy.deepcopy(df['omega'].to_numpy())
        tmp[1:-1] = tmp[0:-2]
        tmp[0] = 0

        df['last_omega'] = tmp

        # print(df['omega'].to_numpy()[100:110])
        # print(df['last_omega'].to_numpy()[100:110])
        # print("...")

        # Get the sequence for batch_u and batch_y
        batch_y = torch.tensor(df['omega'].iloc[start_idx:start_idx + self.seq_len].values, dtype=torch.float32)
        batch_u = torch.tensor(df[['ia', 'ib', 'va', 'vb', 'last_omega']].iloc[start_idx:start_idx + self.seq_len].values,
                               dtype=torch.float32)

        # Add a batch dimension
        batch_y = batch_y.view(-1,1)  # Shape (1, seq_len, 1)

        return batch_u, batch_y

    def get_full_experiment(self, idx):
        df = self.dfs[idx]
        tmp = copy.deepcopy(df['omega'].to_numpy())
        tmp[1:-1] = tmp[0:-2]
        tmp[0] = 0
        df['last_omega'] = tmp
        batch_y = torch.tensor(df['omega'].to_numpy(), dtype=torch.float32)
        batch_u = torch.tensor(df[['ia', 'ib', 'va', 'vb', 'last_omega']].to_numpy(), dtype=torch.float32)
        # Add a batch dimension
        batch_y = batch_y.view(-1,1)  # Shape (1, seq_len, 1)

        return batch_u, batch_y
    
    def get_experiment_observer(self, idx):
        """
        returns pll observer estimated speed, non-normalized
        """
        df = self.dfs[idx]
        obs_y = df['omega_obs'].to_numpy()
        
        return obs_y
    
    def get_experiment_ekf(self, idx):
        """
        returns pll observer estimated speed, non-normalized
        """
        df = self.dfs[idx]
        obs_y = df['omega_ekf'].to_numpy()
        
        return obs_y




# Normalization function
def normalize_fixed_ranges(df):
    df['ia'] = (df['ia'] + 5) / 10  # Normalize iq from -5 to 5 -> [0, 1]
    df['ib'] = (df['ib'] + 5) / 10  # Normalize id from -5 to 5 -> [0, 1]
    df['va'] = (df['va'] + 24) / 48  # Normalize vq from -24 to 24 -> [0, 1]
    df['vb'] = (df['vb'] + 24) / 48  # Normalize vd from -24 to 24 -> [0, 1]
    df['omega'] = df['omega'] / 2500  # Normalize omega from 0 to 2500 -> [0, 1]
    return df


def reverse_normalization(batch_u, batch_y, batch_y_pred):
    # Define the normalization constants
    min_currents = -5
    max_currents = 5
    min_voltages = -24
    max_voltages = 24
    min_speed = 0
    max_speed = 2500

    # Reverse normalization for currents (iq, id)
    # Assuming batch_u contains currents in the first two columns
    batch_u[:, :, 0] = batch_u[:, :, 0] * (max_currents - min_currents) + min_currents  # ia
    batch_u[:, :, 1] = batch_u[:, :, 1] * (max_currents - min_currents) + min_currents  # ib

    # Reverse normalization for voltages (vq, vd)
    batch_u[:, :, 2] = batch_u[:, :, 2] * (max_voltages - min_voltages) + min_voltages  # va
    batch_u[:, :, 3] = batch_u[:, :, 3] * (max_voltages - min_voltages) + min_voltages  # vb

    # Reverse normalization for speed (omega)
    batch_u[:, :, 4] = batch_u[:, :, 4] * (max_speed - min_speed) + min_speed  # last_omega
    batch_y = batch_y * (max_speed - min_speed) + min_speed
    batch_y_pred = batch_y_pred * (max_speed - min_speed) + min_speed

    return batch_u, batch_y, batch_y_pred


def load_dataframes_from_folder(folder_path):
    # Create a list to hold all DataFrames
    dataframes = []
    # Use glob to find all CSV files in the specified folder
    for file in glob.glob(os.path.join(folder_path, '*.csv')):
        df = pd.read_csv(file)
        try:
            df.columns = ['t', 'theta', 'omega', 'r', 'ia', 'ib', 'iq_ref', 'va', 'vb']
        except:
            pass
        df = normalize_fixed_ranges(df)
        # Find the first index where r changes from 0 to 1
        first_non_zero_index = df.index[df['r'].diff().gt(0)].min()
        # df = df.loc[first_non_zero_index:]  # Keep rows up to that index
        dataframes.append(df)

    return dataframes

# Example usage
if __name__ == "__main__":
    folder_path = '../../../in-context-bldc-data/simulated/90_percent_with_alfa_beta_alt'
    # folder_path = '../data/CL_experiments/train/inertia13_ki-0.0061-kp-11.8427'
    dfs = load_dataframes_from_folder(folder_path)
    # Log the number of DataFrames loaded
    print(f"Loaded {len(dfs)} DataFrames from {folder_path}.")

    seq_len = 50

    # Create an instance of the dataset
    dataset = Dataset(dfs=dfs, seq_len=seq_len)
    # dataset = DatasetOnTheFly(dt=0.01, seq_len=50, perturbation_percentage=0.5)
    dataloader = DataLoader(dataset, batch_size=32, shuffle=True)

    # Example of accessing an item
    batch_u, batch_y = next(iter(dataloader))
    print(f'batch_u: {batch_u.shape}, batch_y: {batch_y.shape}')

    # Convert batch tensors to numpy for plotting
    batch_u_np = batch_u.squeeze(0).numpy()  # Shape (seq_len, n_u)
    batch_y_np = batch_y.squeeze(0).numpy()  # Shape (seq_len, 1)

    # Plotting
    fig = plt.figure(figsize=(12, 6))

    # Plot batch_y (omega)
    # plt.subplot(2, 1, 1)
    ax1 = fig.add_subplot(2,1,1)
    # plt.scatter(np.ones_like(batch_y_np[:,:,0].T) * seq_len,batch_y_np[:,:,0].T, label='Batch y (omega)', color='blue')
    # plt.title('Batch y (omega)')
    # plt.xlabel('Time step')
    # plt.ylabel('Value')
    ax1.scatter(np.ones_like(batch_y_np[:,:,0].T) * seq_len,batch_y_np[:,:,0].T, label='Batch y (omega)', color='blue')
    ax1.set_title('Batch y (omega)')
    ax1.set_xlabel('Time step')
    ax1.set_ylabel('Value')
    
    # plt.legend()

    # Plot each component of batch_u
    # plt.subplot(2, 1, 2)
    ax2 = fig.add_subplot(2,1,2, sharex = ax1)
    # plt.plot(batch_u_np[:, :, 0].T, label='Batch u (ia)', color='orange')
    # plt.plot(batch_u_np[:, :, 1].T, label='Batch u (ib)', color='green')
    # plt.plot(batch_u_np[:, :, 2].T, label='Batch u (va)', color='red')
    # plt.plot(batch_u_np[:, :, 3].T, label='Batch u (vb)', color='purple')
    # plt.plot(batch_u_np[:, :, 4].T, label='Batch u (last_omega)', color='grey')
    # plt.title('Batch u (ia, ib, va, vb, last_omega)')
    # plt.xlabel('Time step')
    # plt.ylabel('Value')
    ax2.plot(batch_u_np[:, :, 0].T, label='Batch u (ia)', color='orange')
    ax2.plot(batch_u_np[:, :, 1].T, label='Batch u (ib)', color='green')
    ax2.plot(batch_u_np[:, :, 2].T, label='Batch u (va)', color='red')
    ax2.plot(batch_u_np[:, :, 3].T, label='Batch u (vb)', color='purple')
    ax2.plot(batch_u_np[:, :, 4].T, label='Batch u (last_omega)', color='grey')
    ax2.set_title('Batch u (ia, ib, va, vb, last_omega)')
    ax2.set_xlabel('Time step')
    ax2.set_ylabel('Value')
    # plt.legend()

    plt.tight_layout()

    batch_u, batch_y, _ = reverse_normalization(batch_u, batch_y, batch_y)

    for i in range(2):
        fig = plt.figure()
        ax0 = fig.add_subplot(4,1,1)
        ax0.scatter(np.ones_like(batch_y_np[i,:,:].T) * seq_len,batch_y[i,:,:])
        ax0.set_ylim(-50,3050)
        ax1 = fig.add_subplot(4,1,2, sharex = ax0)
        ax1.plot(batch_u[i,:,0])
        ax1.plot(batch_u[i,:,1])
        ax2 = fig.add_subplot(4,1,3, sharex = ax0)
        ax2.plot(batch_u[i,:,2])
        ax2.plot(batch_u[i,:,3])
        ax3 = fig.add_subplot(4,1,4, sharex = ax0, sharey = ax0)
        ax3.plot(batch_u[i,:,4])
        ax3.set_ylim(-50,3050)

        fig = plt.figure()
        plt.scatter(np.ones_like(batch_y_np[i,:,:].T) * seq_len,batch_y[i,:,:])
        plt.plot(batch_u[i,:,4])
        plt.legend(['orig speed', 'speed+noise'])
        plt.ylim(-50,3050)

    # plt.figure(figsize=(12, 6))
    # # Plot batch_y (omega)
    # plt.subplot(2, 1, 1)
    # plt.plot(batch_y_np[:,:,0].T, label='example y (omega)', color='blue')
    # plt.title('Batch y (omega)')
    # plt.xlabel('Time step')
    # plt.ylabel('Value')
    # # plt.legend()

    # # Plot each component of batch_u
    # plt.subplot(2, 1, 2)
    # plt.plot(batch_u_np[:, :, 0].T, label='example u (iq)', color='orange')
    # plt.plot(batch_u_np[:, :, 1].T, label='example u (id)', color='green')
    # plt.plot(batch_u_np[:, :, 2].T, label='example u (vq)', color='red')
    # plt.plot(batch_u_np[:, :, 3].T, label='example u (vd)', color='purple')
    # plt.title('Batch u (iq, id, vq, vd)')
    # plt.xlabel('Time step')
    # plt.ylabel('Value')
    # # plt.legend()

    # plt.tight_layout()
    plt.show()
