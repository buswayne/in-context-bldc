import numpy as np
import pandas as pd
import glob
import os
import torch
from torch.utils.data import Dataset, DataLoader
import matplotlib.pyplot as plt
import copy

class Dataset(Dataset):
    def __init__(self, dfs, seq_len):
        self.dfs = dfs
        self.seq_len = seq_len

    def __len__(self):
        return 512

    def __getitem__(self, idx):
        prob_2000 = 0.55 # 1 - ratio between samples that go >2000 rpm and not
        prob_0 = 0.1 # probability of getting a sample with a zero in it
        prob_step = 0.5 # ratio between constant samples and step samples

        random_extraction = np.random.rand()

        if random_extraction >= prob_2000: #look for a window with a 2000 speed sample in it
            
            # look for experiments in which the speed reaches 2000rpm
            flag = True
            while flag: # should eventually add a counter to avoid infinite loops
                df_idx = np.random.choice(len(self.dfs))
                df = self.dfs[df_idx]
                if np.max(df['omega'].to_numpy()*2500) >= 2000: #speed is assumed normalized, hence the *2500
                    flag = False

            # evaluate whether the first and last element of the window of length H of the reference speed are different (e.g. if a step is present inside the window)
            diff_array = df['r'].diff(-self.seq_len).to_numpy()
            diff_array = diff_array[~np.isnan(diff_array)]

            # extract the window and check that the speed is at list 2000
            flag2 = True
            while flag2:
                #see dataset.py for comments on this if else
                if np.random.rand() >= prob_step:
                    good_idx = np.flatnonzero(diff_array == 0)
                    if len(good_idx) == 0:
                        good_idx = np.flatnonzero(diff_array != 0)
                else:
                    good_idx = np.flatnonzero(diff_array != 0)
                    if len(good_idx) == 0:
                        good_idx = np.flatnonzero(diff_array == 0)
                start_idx = np.random.choice(good_idx)
                
                # check if the selected window has a >2000 speed sample in it
                if np.max(df['omega'].to_numpy()[start_idx:start_idx + self.seq_len]*2500) >= 2000:
                    flag2 = False


        elif random_extraction <= prob_0: #look for experiments that have a 0 in them
            flag0 = True
            while flag0:
                df_idx = np.random.choice(len(self.dfs))
                df = self.dfs[df_idx]
                if min(df['omega'].to_numpy()*2500) <= 1: #speed is assumed normalized, hence the *2500
                    flag0 = False

            zero_indexes = np.where(df['omega'].to_numpy()*2500 <= 1)[0]
            good_zero_idx = []
            for id in zero_indexes:
                good_starting_indexes = np.arange(max(0, id-self.seq_len+1), min(len(df), id + self.seq_len)+1 )
                good_zero_idx = list(set(good_zero_idx).union(good_starting_indexes))
            good_zero_idx = np.array(good_zero_idx)

            # evaluate whether the first and last element of the window of length H of the reference speed are different (e.g. if a step is present inside the window)
            diff_array = df['r'].diff(-self.seq_len).to_numpy()
            diff_array = diff_array[~np.isnan(diff_array)]
            
            if np.random.rand() >= prob_step:
                good_idx = np.flatnonzero(diff_array == 0)
                if len(good_idx) == 0:
                    good_idx = np.flatnonzero(diff_array != 0)
            else:
                good_idx = np.flatnonzero(diff_array != 0)
                if len(good_idx) == 0:
                    good_idx = np.flatnonzero(diff_array == 0)

            good_idx = list(set(good_idx).intersection(good_zero_idx))
            start_idx = np.random.choice(good_idx)
            
            # check if the selected window has a zero speed in it
            if np.max(df['omega'].to_numpy()[start_idx:start_idx + self.seq_len]*2500) <= 1 :
                flag2 = False

        else: # look for windows with speed <2000rpm
            flag2 = True
            while flag2:
                df_idx = np.random.choice(len(self.dfs))
                df = self.dfs[df_idx]
                # difference between starting time and ending time of the batch
                diff_array = df['r'].diff(-self.seq_len).to_numpy()
                diff_array = diff_array[~np.isnan(diff_array)]

                if np.random.rand() >= prob_step:
                    good_idx = np.flatnonzero(diff_array == 0)
                    if len(good_idx) == 0:
                        good_idx = np.flatnonzero(diff_array != 0)
                else:
                    good_idx = np.flatnonzero(diff_array != 0)
                    if len(good_idx) == 0:
                        good_idx = np.flatnonzero(diff_array == 0)
                start_idx = np.random.choice(good_idx)
                
                # check if the selected window has speed <2000rpm
                if np.max(df['omega'].to_numpy()[start_idx:start_idx + self.seq_len]*2500) < 2000:
                    flag2 = False


        # generate a column in the dataset for the "past" values of omega. These are not actually used in training, but are overwritten with the past estimations
        tmp = copy.deepcopy(df['omega'].to_numpy())
        tmp[1:-1] = tmp[0:-2]
        tmp[0] = 0

        df['last_omega'] = tmp

        # Get the sequence for batch_u and batch_y
        batch_y = torch.tensor(df['omega'].iloc[start_idx:start_idx + self.seq_len].values, dtype=torch.float32)
        batch_u = torch.tensor(df[['ia', 'ib', 'va', 'vb', 'last_omega']].iloc[start_idx:start_idx + self.seq_len].values,
                               dtype=torch.float32)

        # Add a batch dimension
        batch_y = batch_y.view(-1,1)  # Shape (1, seq_len, 1)

        return batch_u, batch_y

    def get_full_experiment(self, idx):
        '''
        Outputs the entirety of the experiment at index idx as a torch tensor (normalized if the data files were passed to the Dataset object correctly)
        '''
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
        returns ekf estimated speed, non-normalized
        """
        df = self.dfs[idx]
        obs_y = df['omega_ekf'].to_numpy()
        
        return obs_y



# Normalization function
def normalize_fixed_ranges(df):
    '''
    Transforms the relevant column of the dataframe so that their valuse is in the range [0,1], or at least in its order of magnitude
    '''
    df['ia'] = (df['ia'] + 5) / 10  # Normalize iq from -5 to 5 -> [0, 1]
    df['ib'] = (df['ib'] + 5) / 10  # Normalize id from -5 to 5 -> [0, 1]
    df['va'] = (df['va'] + 24) / 48  # Normalize vq from -24 to 24 -> [0, 1]
    df['vb'] = (df['vb'] + 24) / 48  # Normalize vd from -24 to 24 -> [0, 1]
    df['omega'] = df['omega'] / 2500  # Normalize omega from 0 to 2500 -> [0, 1]
    return df


def reverse_normalization(batch_u, batch_y, batch_y_pred):
    '''
    Transforms the batch values into their orignal values, inverting the transfotrmation of "normalized_fixed_ranges()"
    '''
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
    '''
    Transforms the batch values into their orignal values, inverting the transfotrmation of "normalized_fixed_ranges()"
    '''
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
    
    current_path = os.getcwd().split("in-context-bldc")[0]
    data_path = os.path.join(current_path,"in-context-bldc", "data")

    # folder = "CL_experiments_double_sensor_low_speed_ekf_and_meta/final/inertia13_ki-0.0029-kp-3.0000"
    folder = "CL_experiments_double_sensor_high_speed_ekf_and_meta/final/inertia13_ki-0.0061-kp-11.8427"
    folder_path = os.path.join(data_path, folder)

    dfs = load_dataframes_from_folder(folder_path)
    # Log the number of DataFrames loaded
    print(f"Loaded {len(dfs)} DataFrames from {folder_path}.")

    seq_len = 50

    # Create an instance of the dataset
    dataset = Dataset(dfs=dfs, seq_len=seq_len)
    dataloader = DataLoader(dataset, batch_size=32, shuffle=True)

    # Example of accessing an item
    batch_u, batch_y = next(iter(dataloader))
    print(f'batch_u: {batch_u.shape}, batch_y: {batch_y.shape}')

    # Convert batch tensors to numpy for plotting
    batch_u_np = batch_u.squeeze(0).numpy()  # Shape (seq_len, n_u)
    batch_y_np = batch_y.squeeze(0).numpy()  # Shape (seq_len, 1)

    fig = plt.figure(figsize=(12, 6))

    # Plot batch_y (omega)
    ax1 = fig.add_subplot(2,1,1)
    ax1.plot(batch_y_np[:,:,0].T, label='Batch y (omega)', color='blue')
    ax1.set_title('Batch y (omega)')
    ax1.set_xlabel('Time step')
    ax1.set_ylabel('Value')
    
    # Plot each component of batch_u
    ax2 = fig.add_subplot(2,1,2, sharex = ax1)
    ax2.plot(batch_u_np[:, :, 0].T, label='Batch u (ia)', color='orange')
    ax2.plot(batch_u_np[:, :, 1].T, label='Batch u (ib)', color='green')
    ax2.plot(batch_u_np[:, :, 2].T, label='Batch u (va)', color='red')
    ax2.plot(batch_u_np[:, :, 3].T, label='Batch u (vb)', color='purple')
    ax2.plot(batch_u_np[:, :, 4].T, label='Batch u (last_omega)', color='grey')
    ax2.set_title('Batch u (ia, ib, va, vb, last_omega)')
    ax2.set_xlabel('Time step')
    ax2.set_ylabel('Value')

    plt.tight_layout()


    # plot some window examples
    batch_u, batch_y, _ = reverse_normalization(batch_u, batch_y, batch_y)

    for i in range(2):
        fig = plt.figure()
        ax0 = fig.add_subplot(4,1,1)
        ax0.plot(batch_y[i,:,:],label = "$\omega$")
        ax0.legend()
        # ax0.set_ylim(-50,3050)
        ax1 = fig.add_subplot(4,1,2, sharex = ax0)
        ax1.plot(batch_u[i,:,0],label = "$I_a$")
        ax1.plot(batch_u[i,:,1],label = "$I_b$")
        ax1.legend()
        ax2 = fig.add_subplot(4,1,3, sharex = ax0)
        ax2.plot(batch_u[i,:,2],label = "$V_a$")
        ax2.plot(batch_u[i,:,3],label = "$V_b$")
        ax2.legend()
        ax3 = fig.add_subplot(4,1,4, sharex = ax0, sharey = ax0)
        ax3.plot(batch_u[i,:,4],label = "$\omega_{k-1}$")
        ax3.legend()
        # ax3.set_ylim(-50,3050)



    plt.show()
