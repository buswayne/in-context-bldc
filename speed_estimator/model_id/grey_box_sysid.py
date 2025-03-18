import sys

import matplotlib.pyplot as plt
import torch
import torch.nn as nn
from torch.utils.data import DataLoader
from dataset import load_dataframes_from_folder, Dataset
from bldc import dynamics
from torch_utils import select_gpu_with_most_free_memory
import matplotlib.pyplot as plt
import pickle
import wandb


class GreyBoxModel(nn.Module):
    def __init__(self):
        super(GreyBoxModel, self).__init__()

        #
        # Parameters
        # Rs, Ls, Kt, J, b
        # theta_0, P = 7;
        self.dt = 1e-2

        # Initialize the parameters for identification as nn.Parameters
        self.params = nn.ParameterList([
            nn.Parameter(torch.tensor(1, dtype=torch.float32, device='cuda')),
            nn.Parameter(torch.tensor(1, dtype=torch.float32, device='cuda')),
            nn.Parameter(torch.tensor(1, dtype=torch.float32, device='cuda')),
            nn.Parameter(torch.tensor(1, dtype=torch.float32, device='cuda')),
            nn.Parameter(torch.tensor(1, dtype=torch.float32, device='cuda')),
        ])


    def optimize_parameters(self, x_init, u_data, y_data, lr=1e-1, num_iter=1_000):
        """
        Optimize the parameters based on experimental input-output data.

        Args:
        - x_data (torch.Tensor): Initial state vectors (shape: (num_samples, state_dim))
        - u_data (torch.Tensor): Input vectors (shape: (num_samples, control_dim))
        - y_data (torch.Tensor): Output (target) vectors (shape: (num_samples, state_dim))
        - lr (float): Learning rate for the optimizer
        - num_iter (int): Number of optimization iterations

        Returns:
        - optimized_params (list): List of optimized parameters
        """
        optimizer = torch.optim.Adam(self.parameters(), lr=lr)

        for epoch in range(num_iter):
            optimizer.zero_grad()

            y_pred = torch.zeros_like(y_data)
            x_pred = x_init

            # Predict the states using the dynamics function
            for k in range(u_data.shape[0]):
                x_pred_dot = dynamics(x_pred, u_data[k, :], *self.params)
                x_pred = x_pred + self.dt * x_pred_dot
                # print(x_pred[0])
                # print(x_pred_dot.shape)
                # print(y_pred[k, :].shape)
                # print(x_pred[:2].shape)

                y_pred[k, :] = x_pred[:2,0]

            # Compute the loss between predicted states and actual states
            print(y_pred.isnan().sum())
            mse_loss = nn.MSELoss()
            loss = mse_loss(y_pred, y_data)

            # Backpropagation
            loss.backward()

            # Update parameters
            optimizer.step()

            if epoch % 1 == 0:
                print(f'Epoch {epoch}, Loss: {loss.item()}')
                if epoch % 5 == 0:
                    plt.figure()
                    plt.subplot(211)
                    plt.plot(y_data[:,0].clone().detach().cpu().numpy())
                    plt.plot(y_pred[:,0].clone().detach().cpu().numpy())
                    plt.subplot(212)
                    plt.plot(y_data[:,1].clone().detach().cpu().numpy())
                    plt.plot(y_pred[:,1].clone().detach().cpu().numpy())
                    plt.show()

        # Return the optimized parameters
        optimized_params = [param.detach() for param in self.params]
        return optimized_params


def main():

    torch.autograd.set_detect_anomaly(True)
    # Example usage
    best_gpu = select_gpu_with_most_free_memory()
    device = f'cuda:{best_gpu}' if best_gpu is not None else 'cpu'

    folder_path = '../data/real_aux_sensor/'

    dfs = load_dataframes_from_folder(folder_path)
    # Log the number of DataFrames loaded
    print(f"Loaded {len(dfs)} DataFrames from {folder_path}.")

    # Create an instance of the dataset
    dataset = Dataset(dfs=dfs, seq_len=500)
    dataloader = DataLoader(dataset, batch_size=1, shuffle=True)

    # Example of accessing an item
    batch_u, batch_y = next(iter(dataloader))
    batch_u, batch_y = batch_u.to(device), batch_y.to(device)

    plt.figure()
    plt.subplot(311)
    plt.plot(batch_u[0, :, 0].clone().detach().cpu().numpy())
    plt.plot(batch_u[0, :, 1].clone().detach().cpu().numpy())
    plt.subplot(312)
    plt.plot(batch_u[0, :, 2].clone().detach().cpu().numpy())
    plt.plot(batch_u[0, :, 3].clone().detach().cpu().numpy())
    plt.subplot(313)
    plt.plot(batch_y[0, :, 0].clone().detach().cpu().numpy())
    plt.show()
    # print(batch_u.shape)
    # print(batch_y.shape)

    for idx in range(1):

        print('Test number:', idx)

        # open loop experiment
        U = batch_u[0, :, 2:4]
        Y = batch_u[0, :, :2]
        # Y = torch.cat((Y, batch_u[0, :, 4].view(-1,1)), dim=1)

        X_init = torch.zeros((4, 1), device=device)

        # Initialize and train the grey-box model
        greybox_model = GreyBoxModel().to(device)

        optimized_params = greybox_model.optimize_parameters(X_init, U, Y)

        print(optimized_params)
        #
        # X_hat = torch.zeros(2, T+1, device=device)
        # Y_hat = torch.zeros(1, T, device=device)
        # X_hat[:,0] = torch.tensor([25.0, 49.743])
        #
        # for k in range(T):
        #     X_dot = dynamics(X_hat[:,k], U[:,k], *optimized_params)
        #     X_hat[:,k+1] = X_hat[:,k] + 1 * X_dot
        #     Y_hat[:,k] = X_hat[0,k]
        #
        #
        #
        # plt.figure(figsize=(10,6))
        #
        # plt.subplot(211)
        # plt.plot(X[0,:].T.cpu().detach().numpy(), label='$x_1$')
        # plt.plot(X_hat[0, :].T.cpu().detach().numpy(), label='$\hat{x}_1$')
        # # plt.plot(X[1, :].T.cpu().detach().numpy(), label='$x_2$')
        # plt.legend()
        #
        # plt.subplot(212)
        # plt.plot(U[0,:].T.cpu().detach().numpy(), label='$u_1$')
        # plt.plot(U[1, :].T.cpu().detach().numpy(), label='$u_2$')
        # plt.legend()
        #
        # plt.show()


if __name__ == '__main__':
    main()