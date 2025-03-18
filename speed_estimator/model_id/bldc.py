import torch
from networkx.algorithms.smallworld import omega


def dynamics(x, u, Rs, Ls, Kt, J, b):
    """ System dynamics function (discrete time, PyTorch version for GPU) """

    theta0 = 0
    P = 7

    i_d = x[0]
    i_q = x[1]
    omega = x[2]
    theta_e = x[3]

    V_alpha = u[0]
    V_beta = u[1]

    # Calculate intermediate variables using the provided parameters
    i_d_dot = -Rs/Ls*i_d + omega*i_q + torch.cos(theta_e) /Ls * V_alpha + torch.sin(theta_e) /Ls * V_beta
    i_q_dot = -Rs/Ls*i_q + -omega*i_d - Kt/Ls*omega - torch.sin(theta_e)/Ls * V_alpha + torch.cos(theta_e)/Ls * V_beta
    omega_dot = 3/2*P*Kt/J*i_q - b/J*omega
    theta_e_dot = P*omega

    # # Return state derivatives as a tensor
    xdot = torch.empty(x.shape, dtype=x.dtype, device=x.device)
    xdot[0] = i_d_dot
    xdot[1] = i_q_dot
    xdot[2] = omega_dot
    xdot[3] = theta_e_dot

    # xdot = torch.cat((X2_dot.view(1,-1), P2_dot.view(1,-1)), dim=1).to(x.device)
    return xdot