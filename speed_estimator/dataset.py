import numpy as np
import pandas as pd
import glob
import os
import torch
from torch.utils.data import Dataset, DataLoader
import matplotlib.pyplot as plt
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

        # Randomly select a starting index
        max_val = len(df) - self.seq_len
        start_idx = np.random.randint(0, max_val)

        # Get the sequence for batch_u and batch_y
        batch_y = torch.tensor(df['omega'].iloc[start_idx:start_idx + self.seq_len].values, dtype=torch.float32)
        batch_u = torch.tensor(df[['iq', 'id', 'vq', 'vd']].iloc[start_idx:start_idx + self.seq_len].values,
                               dtype=torch.float32)

        # Add a batch dimension
        batch_y = batch_y.view(-1,1)  # Shape (1, seq_len, 1)

        return batch_u, batch_y


# class DatasetOnTheFly(Dataset):
#     def __init__(self, dt, seq_len, perturbation_percentage):
#         self.dt = dt
#         self.seq_len = seq_len
#         self.T_max = 10
#         self.t_span = np.arange(0, self.T_max, self.dt)
#         self.initial_state = [0.0, 0.0, 0.0, 0.0, 0.0]
#         self.d_min, self.d_max = 1, 5
#
#         # Perturbation range: ±100%
#         self.perturb_percentage = perturbation_percentage
#
#         # Motor nominal params
#         self.R, self.L, self.Kt, self.Ke, self.J, self.B = 0.994, 0.995e-3, 91e-3, 1 / 10.9956, 44e-4, 0.528e-3
#         self.V_nominal, self.I_nominal = 48, 10
#         self.P = 4  # Number of pole pairs
#         self.Kt, self.Ke = self.Kt / self.P, self.Ke / self.P
#     def __len__(self):
#         return 512
#
#     def __getitem__(self, idx):
#         # Randomly select a starting index
#         max_val = len(self.t_span) - self.seq_len
#         start_idx = np.random.randint(0, max_val)
#
#         # Randomly perturb motor parameters within ±100% of their nominal values
#         R_perturbed = self.R * (1 + self.perturb_percentage * np.random.uniform(-1, 1))
#         L_perturbed = self.L * (1 + self.perturb_percentage * np.random.uniform(-1, 1))
#         Kt_perturbed = self.Kt * (1 + self.perturb_percentage * np.random.uniform(-1, 1))
#         Ke_perturbed = self.Ke * (1 + self.perturb_percentage * np.random.uniform(-1, 1))
#         J_perturbed = self.J * (1 + self.perturb_percentage * np.random.uniform(-1, 1))
#         B_perturbed = self.B * (1 + self.perturb_percentage * np.random.uniform(-1, 1))
#
#         # Initialize the motor with perturbed parameters
#         motor = BLDCMotor(R_perturbed,
#                           L_perturbed,
#                           Kt_perturbed,
#                           Ke_perturbed,
#                           J_perturbed,
#                           B_perturbed,
#                           self.V_nominal,
#                           self.I_nominal,
#                           self.P)
#
#         V_q_steps = steps_sequence(self.T_max, self.dt, -self.V_nominal, self.V_nominal, self.d_min, self.d_max).flatten()
#         V_d_steps = np.zeros_like(V_q_steps)
#         r = np.zeros_like(V_q_steps)
#
#         # Run the simulation
#         t_span, omega_sol_rpm, theta_sol, i_d_sol, i_q_sol = motor.simulate(self.dt, self.initial_state, self.t_span, V_q_steps)
#
#         data = np.array([t_span, i_q_sol, i_d_sol, V_q_steps, V_d_steps, omega_sol_rpm, r])
#         df = pd.DataFrame(data.T, columns=['timestamp', 'iq', 'id', 'vq', 'vd', 'omega', 'r'])
#         df = normalize_fixed_ranges(df) # normalize
#
#         batch_y = torch.tensor(df['omega'].iloc[start_idx:start_idx + self.seq_len].values, dtype=torch.float32)
#         batch_u = torch.tensor(df[['iq', 'id', 'vq', 'vd']].iloc[start_idx:start_idx + self.seq_len].values,
#                                dtype=torch.float32)
#         # Add a batch dimension
#         batch_y = batch_y.view(-1,1)  # Shape (1, seq_len, 1)
#
#         return batch_u, batch_y

# Normalization function
def normalize_fixed_ranges(df):
    df['iq'] = (df['iq'] + 5) / 10  # Normalize iq from -5 to 5 -> [0, 1]
    df['id'] = (df['id'] + 5) / 10  # Normalize id from -5 to 5 -> [0, 1]
    df['vq'] = (df['vq'] + 24) / 48  # Normalize vq from -24 to 24 -> [0, 1]
    df['vd'] = (df['vd'] + 24) / 48  # Normalize vd from -24 to 24 -> [0, 1]
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
    batch_u[:, :, 0] = batch_u[:, :, 0] * (max_currents - min_currents) + min_currents  # iq
    batch_u[:, :, 1] = batch_u[:, :, 1] * (max_currents - min_currents) + min_currents  # id

    # Reverse normalization for voltages (vq, vd)
    batch_u[:, :, 2] = batch_u[:, :, 2] * (max_voltages - min_voltages) + min_voltages  # vq
    batch_u[:, :, 3] = batch_u[:, :, 3] * (max_voltages - min_voltages) + min_voltages  # vd

    # Reverse normalization for speed (omega)
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
            df.columns = ['t', 'theta', 'omega', 'r', 'id', 'iq', 'iq_ref', 'vd', 'vq']
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
    folder_path = '../data/simulated/50_percent_longer_steps'
    dfs = load_dataframes_from_folder(folder_path)
    # Log the number of DataFrames loaded
    print(f"Loaded {len(dfs)} DataFrames from {folder_path}.")

    # Create an instance of the dataset
    dataset = Dataset(dfs=dfs, seq_len=50)
    # dataset = DatasetOnTheFly(dt=0.01, seq_len=50, perturbation_percentage=0.5)
    dataloader = DataLoader(dataset, batch_size=32, shuffle=True)

    # Example of accessing an item
    batch_u, batch_y = next(iter(dataloader))
    print(f'batch_u: {batch_u.shape}, batch_y: {batch_y.shape}')

    # Convert batch tensors to numpy for plotting
    batch_u_np = batch_u.squeeze(0).numpy()  # Shape (seq_len, n_u)
    batch_y_np = batch_y.squeeze(0).numpy()  # Shape (seq_len, 1)

    # Plotting
    plt.figure(figsize=(12, 6))

    # Plot batch_y (omega)
    plt.subplot(2, 1, 1)
    plt.plot(batch_y_np[:,:,0].T, label='Batch y (omega)', color='blue')
    plt.title('Batch y (omega)')
    plt.xlabel('Time step')
    plt.ylabel('Value')
    # plt.legend()

    # Plot each component of batch_u
    plt.subplot(2, 1, 2)
    plt.plot(batch_u_np[:, :, 0].T, label='Batch u (iq)', color='orange')
    plt.plot(batch_u_np[:, :, 1].T, label='Batch u (id)', color='green')
    plt.plot(batch_u_np[:, :, 2].T, label='Batch u (vq)', color='red')
    plt.plot(batch_u_np[:, :, 3].T, label='Batch u (vd)', color='purple')
    plt.title('Batch u (iq, id, vq, vd)')
    plt.xlabel('Time step')
    plt.ylabel('Value')
    # plt.legend()

    plt.tight_layout()

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
