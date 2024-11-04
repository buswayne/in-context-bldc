import numpy as np
import matplotlib.pyplot as plt
import torch

# def trapezoidal_emf(theta):
#     theta = theta + 4/6 * np.pi
#     theta = np.mod(theta, 2 * np.pi)
#     if 0 <= theta < 1 * np.pi / 6:
#         return 1 / (np.pi/6) * theta  

#     elif np.pi / 6 <= theta < 5 * np.pi / 6:
#         return 1

#     elif 5 * np.pi / 6 <= theta < 7 * np.pi / 6:
#         return -1 / (np.pi/6) * (theta - np.pi)

#     elif 7 * np.pi / 6 <= theta < 11 * np.pi / 6:
#         return -1

#     else:
#         return 1 / (np.pi/6) * (theta -2 * np.pi)


# theta = np.linspace(0, 2*np.pi, 100)

# phase_a = np.array([trapezoidal_emf(t) for t in theta])
# phase_b = np.array([trapezoidal_emf(t) for t in (theta - 2/3*np.pi)])
# phase_c = np.array([trapezoidal_emf(t) for t in (theta + 2/3*np.pi)])

# plt.figure()

# plt.plot(theta, phase_a)
# plt.plot(theta, phase_b)
# plt.plot(theta, phase_c)

# # plt.figure()
# # plt.plot(theta, phase_a+phase_b+phase_c)
# plt.show()

tensor = torch.ones(4, 4)
# print(f"First row: {tensor[0]}")
# print(f"First column: {tensor[:, 0]}")
# print(f"Last column: {tensor[:, -1]}")
tensor[:,1] = 0
# print(tensor)
# print(tensor.device)
# if torch.cuda.is_available():
#     tensor = tensor.to("cuda")

# print(tensor.device)

t1 = torch.cat([tensor, tensor, tensor], dim=0)
print(t1)