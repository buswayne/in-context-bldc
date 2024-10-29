import numpy as np
import matplotlib.pyplot as plt
from bldc_simulator_ale import *

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

dt = 0.0001
t_span = np.arange(0, 5, dt)
initial_state = [0.0, 0.0, 0.0, 0.0, 0.0]
speed_reference_rpm = np.zeros(len(t_span))
step_times = [1, 5, 15]
step_values_rpm = [500, 1000, 1500]

for i in range(len(step_times)):
    speed_reference_rpm[t_span >= step_times[i]] = step_values_rpm[i]

# Initialize control system
bldc_motor = BLDCMotor(params)
control_system = BLDCControlSystem(bldc_motor, Kp_speed=3, Ki_speed=22, Kp_current=0.01, Ki_current=25)

omega_sol_rpm, theta_sol, i_d_sol, i_q_sol, V_a_sol, V_b_sol, V_c_sol, V_a_sat, V_b_sat, V_c_sat= control_system.simulate_open_loop(t_span, initial_state, dt)

plt.figure(figsize=(12, 7))

plt.subplot(3, 1, 1)
plt.plot(t_span, omega_sol_rpm, label='Speed (RPM)', color='blue')
# plt.plot(t_span, speed_reference_rpm, label='Speed Reference (RPM)', color='red', linestyle='--')
plt.title('Motor Speed')
plt.xlabel('Time (s)')
plt.ylabel('Speed (RPM)')
plt.legend()

plt.subplot(3, 1, 2)
plt.plot(t_span, i_d_sol, label='d-axis Current (i_d)', color='green')
# plt.plot(t_span, reference_id, label='d-axis Current Reference (RPM)', color='red', linestyle='--')
plt.title('d-axis Current (i_d)')
plt.xlabel('Time (s)')
plt.ylabel('Current (A)')
plt.legend()

plt.subplot(3, 1, 3)
plt.plot(t_span, i_q_sol, label='q-axis Current (i_q)', color='orange')
# plt.plot(t_span, reference_iq, label='q-axis Current Reference (RPM)', color='red', linestyle='--')
plt.title('q-axis Current (i_q)')
plt.xlabel('Time (s)')
plt.ylabel('Current (A)')
plt.legend()

plt.tight_layout()

plt.figure(figsize=(12, 7))

plt.subplot(3, 1, 1)
plt.plot(t_span, V_a_sat, color='blue')
plt.plot(t_span, V_a_sol, color='red', linestyle='--')
plt.title('Phase A')
plt.xlabel('Time (s)')

plt.subplot(3, 1, 2)
plt.plot(t_span, V_b_sat, color='blue')
plt.plot(t_span, V_b_sol, color='red', linestyle='--')
plt.title('Phase B')
plt.xlabel('Time (s)')

plt.subplot(3, 1, 3)
plt.plot(t_span, V_c_sat, color='blue')
plt.plot(t_span, V_c_sol, color='red', linestyle='--')
plt.title('Phase C')
plt.xlabel('Time (s)')

plt.tight_layout()


plt.show()
