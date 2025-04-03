import numpy as np
import matplotlib.pyplot as plt
from bldc_simulator_ale import *
import os
import pandas as pd

current_path = os.getcwd().split("in-context-bldc")[0]
data_path_tmp = os.path.join(current_path, 'in-context-bldc\\data\\CL_experiments\\train\\inertia13_ki-0.0061-kp-11.8427')
# C:\Users\39340\Documents\GitHub\in-context-bldc\data\CL_experiments\train\inertia13_ki-0.0061-kp-11.8427\2024-10-16--10-57-42_exp  26.csv
exp = "2024-10-16--10-57-42_exp  26.csv"
exp_path =  os.path.join(data_path_tmp, exp)

params = MotorParameters(
    R=0.994,
    L=0.995e-3,
    Kt=91e-3,
    Ke=1 / 10.9956,
    J=44e-7,
    B=44e-7/0.528e-3,
    V_nominal=24,
    I_nominal=10,
    Pole_pairs=14/2
)

#### take file, cut to 10s, resample
df_real_data = pd.read_csv(exp_path)

n_samples = 1000
t_real = df_real_data["t"].to_numpy()[:n_samples]
iq_real = df_real_data["iq"].to_numpy()[:n_samples]
id_real = df_real_data["id"].to_numpy()[:n_samples]
vd_real = df_real_data["vd"].to_numpy()[:n_samples]
vq_real = df_real_data["vq"].to_numpy()[:n_samples]
omega_real = df_real_data["omega"].to_numpy()[:n_samples]




dt = 0.0001
t_span = np.arange(0, t_real[-1], dt)
initial_state = [0.0, 0.0, 0.0, 0.0, 0.0]
vd_ref = np.zeros_like(t_span)
vq_ref = np.zeros_like(t_span)
# for t_idx_resample in np.arange(len(t_span)):
#     vd_ref[t_idx_resample] = vd_real[np.max(np.where(t_real<=t_span[t_idx_resample]))]
#     vq_ref[t_idx_resample] = vq_real[np.max(np.where(t_real<=t_span[t_idx_resample]))]

vd_ref = np.interp(t_span, t_real, vd_real)
vq_ref = np.interp(t_span, t_real, vq_real)

# plt.figure()
# plt.plot(t_span, vd_ref)
# plt.plot(t_real, vd_real)
# plt.figure()
# plt.plot(t_span, vq_ref)
# plt.plot(t_real, vq_real)
# plt.show()
# a=0


# speed_reference_rpm = np.zeros(len(t_span))
# step_times = [1, 5, 15]
# step_values_rpm = [500, 1000, 1500]

# for i in range(len(step_times)):
#     speed_reference_rpm[t_span >= step_times[i]] = step_values_rpm[i]

# Initialize control system
bldc_motor = BLDCMotor(params)
control_system = BLDCControlSystem(bldc_motor, Kp_speed=3, Ki_speed=22, Kp_current=0.01, Ki_current=25)

omega_sol_rpm, theta_sol, i_d_sol, i_q_sol, V_a_sol, V_b_sol, V_c_sol, V_a_sat, V_b_sat, V_c_sat= control_system.simulate_open_loop(t_span, initial_state,vd_ref,vq_ref, dt)

plt.figure(figsize=(12, 7))

plt.subplot(4, 1, 1)
plt.plot(t_span, omega_sol_rpm, label='Speed (RPM)', color='blue')
plt.plot(t_real, omega_real, label='Real speed (RPM)', color='red', linestyle='--')
plt.title('Motor Speed')
plt.xlabel('Time (s)')
plt.ylabel('Speed (RPM)')
plt.legend()

plt.subplot(4, 1, 2)
plt.plot(t_span, i_d_sol, label='d-axis Current (i_d)', color='green')
plt.plot(t_real, id_real, label='Real d-axis Current', color='red', linestyle='--')
plt.title('d-axis Current (i_d)')
plt.xlabel('Time (s)')
plt.ylabel('Current (A)')
plt.legend()

plt.subplot(4, 1, 3)
plt.plot(t_span, i_q_sol, label='q-axis Current (i_q)', color='orange')
plt.plot(t_real, iq_real, label='Real q-axis Current(RPM)', color='red', linestyle='--')
plt.title('q-axis Current (i_q)')
plt.xlabel('Time (s)')
plt.ylabel('Current (A)')
plt.legend()

plt.subplot(4, 1, 4)
plt.plot(t_real, vd_real, label='q-axis Voltage')
plt.plot(t_real, vq_real, label='q-axis Voltage')
plt.title('Input voltages')
plt.xlabel('Time (s)')
plt.ylabel('Voltage (V)')
plt.legend()

plt.tight_layout()

# plt.figure(figsize=(12, 7))

# plt.subplot(3, 1, 1)
# plt.plot(t_span, V_a_sat, color='blue')
# plt.plot(t_span, V_a_sol, color='red', linestyle='--')
# plt.title('Phase A')
# plt.xlabel('Time (s)')

# plt.subplot(3, 1, 2)
# plt.plot(t_span, V_b_sat, color='blue')
# plt.plot(t_span, V_b_sol, color='red', linestyle='--')
# plt.title('Phase B')
# plt.xlabel('Time (s)')

# plt.subplot(3, 1, 3)
# plt.plot(t_span, V_c_sat, color='blue')
# plt.plot(t_span, V_c_sol, color='red', linestyle='--')
# plt.title('Phase C')
# plt.xlabel('Time (s)')

# plt.tight_layout()


plt.show()
