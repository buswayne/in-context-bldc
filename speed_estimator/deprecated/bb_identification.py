import torch
import torch.nn as nn
import torch.optim as optim
from torchdiffeq import odeint
from dataset import Dataset, load_dataframes_from_folder
from torch.utils.data import DataLoader


# Neural network representing the dynamics (state + control input)
class DynamicsNN(nn.Module):
    def __init__(self, state_size, control_size, hidden_size, output_size):
        super(DynamicsNN, self).__init__()
        self.fc1 = nn.Linear(state_size + control_size, hidden_size)
        self.relu = nn.ReLU()
        self.fc2 = nn.Linear(hidden_size, output_size)

    def forward(self, t, state_and_control):
        # x will contain both state and control input concatenated
        x = self.fc1(state_and_control)
        x = self.relu(x)
        x = self.fc2(x)
        return x

# Neural ODE class
class NeuralODE(nn.Module):
    def __init__(self, dynamics_func, control_inputs):
        super(NeuralODE, self).__init__()
        self.dynamics_func = dynamics_func  # f_theta in the ODE
        self.control_inputs = control_inputs

    def forward(self, x0, t_span):
        solution = []

        for t_idx in range(len(t_span)):
            current_control = self.control_inputs[:,t_idx,:]  # Control input at time t_idx

            # Define a function to pass to odeint that concatenates the state and control input
            def ode_func(t, state):
                # Concatenate the state with the current control input
                state_and_control = torch.cat([state, current_control], dim=-1)
                # Compute dx/dt using the neural network dynamics
                return self.dynamics_func(t, state_and_control)

            # Solve the ODE for a single time step
            x0 = odeint(ode_func, x0, t_span[t_idx:t_idx + 2])[-1]  # Take the last point of the step
            solution.append(x0)

        return torch.stack(solution)  # Return solution across all time steps


# Hyperparameters
state_size = 3  # You have 3 states
control_size = 2  # Assume 2 control input
hidden_size = 8
output_size = state_size  # We want to output dx/dt which has the same size as the state

# Real data
folder_path = '../data/CL_experiments/train/inertia13_ki-0.0061-kp-11.8427'
dfs = load_dataframes_from_folder(folder_path)
# Log the number of DataFrames loaded
print(f"Loaded {len(dfs)} DataFrames from {folder_path}.")

# Create an instance of the dataset
dataset = Dataset(dfs=dfs, seq_len=50)
dataloader = DataLoader(dataset, batch_size=1, shuffle=True)

# Example of accessing an item
batch_u, batch_y = next(iter(dataloader))
ts = 1e-2
b, tt, nu = batch_u.shape

# Example initial condition and time span
x0 = torch.cat((batch_u[:,0,:2], batch_y[:,0,:]), dim=1)  # Initial state (1 sample, 3D state)
t_span = torch.arange(0, tt*ts, ts)  # Time span
print(t_span.shape)
control_inputs = batch_u[:,:,2:]  # Control input for each time step

# Model
dynamics_nn = DynamicsNN(state_size, control_size, hidden_size, output_size)
neural_ode = NeuralODE(dynamics_nn, control_inputs)

# Forward pass (solving the ODE)
solution = neural_ode(x0, t_span)  # Solution has shape [100, 1, 3]

# Training loop setup (assuming you have input/output data from your system)
criterion = nn.MSELoss()
optimizer = optim.Adam(neural_ode.parameters(), lr=0.001)

def measured_output(state):
    return state[:, :, -1].unsqueeze(-1)  # Example: the output is the last state

# Training loop
for epoch in range(100):  # Number of epochs
    optimizer.zero_grad()

    # Solve ODE from initial condition x0
    batch_x_pred = neural_ode(x0, t_span)

    # Extract the measured output (e.g., first state)
    batch_y_pred = measured_output(batch_x_pred.view(b, -1, state_size))

    # Compute loss
    loss = criterion(batch_y, batch_y_pred)
    loss.backward()
    optimizer.step()

    if epoch % 10 == 0:
        print(f'Epoch [{epoch}/100], Loss: {loss.item():.4f}')
