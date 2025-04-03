import numpy as np

# Utility Functions
def clarke_transform(i_a, i_b, i_c):
    i_alpha = 2 / 3 * (i_a - i_b / 2 - i_c / 2)
    i_beta = 2 / 3 * (i_b * np.sqrt(3) / 2 - i_c * np.sqrt(3) / 2)
    return i_alpha, i_beta

def inverse_clarke_transform(V_alpha, V_beta):
    V_a = V_alpha
    V_b = (-V_alpha + np.sqrt(3) * V_beta) / 2
    V_c = (-V_alpha - np.sqrt(3) * V_beta) / 2
    return V_a, V_b, V_c

def park_transform(i_alpha, i_beta, theta):
    i_d = i_alpha * np.cos(theta) + i_beta * np.sin(theta)
    i_q = -i_alpha * np.sin(theta) + i_beta * np.cos(theta)
    return i_d, i_q

def inverse_park_transform(V_d, V_q, theta):
    V_alpha = V_d * np.cos(theta) - V_q * np.sin(theta)
    V_beta = V_d * np.sin(theta) + V_q * np.cos(theta)
    return V_alpha, V_beta

def rad_s_to_rpm(omega):
    return omega * (60 / (2 * np.pi))

def rpm_to_rad_s(omega_rpm):
    return omega_rpm * (2 * np.pi / 60)