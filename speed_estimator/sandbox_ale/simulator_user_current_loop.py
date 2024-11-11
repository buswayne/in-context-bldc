import numpy as np
import matplotlib.pyplot as plt
from bldc_simulator_ale import *

params = MotorParameters(
    R=0.994,
    L=0.995e-3,
    Kt=91e-3,
    Ke=1 / 10.9956,
    J=44e-7,
    B=0.0083, #44e-7/0.528e-3
    V_nominal=24,
    I_nominal=10,
    Pole_pairs=14/2
)
bldc_motor = BLDCMotor(params)

dt = 0.0001
t_span = np.arange(0, 3, dt)
initial_state = [0.0, 0.0, 0.0, 0.0, 0.0]
i_q_ref_array =  np.zeros(len(t_span))
step_times = [1]
step_values = [10]
for i in range(len(step_times)):
    i_q_ref_array[t_span >= step_times[i]] = step_values[i]

kp_list = [0.01]
ki_list = [100]
k_sat_list = [1]

for KP in kp_list:
    for KI in ki_list:
        for KSAT in k_sat_list:


            # Initialize control system
            control_system = BLDCControlSystem(bldc_motor, Kp_speed=3, Ki_speed=22, Kp_current=KP, Ki_current=KI, K_sat_current=KSAT)

            omega_sol_rpm, theta_sol, i_d_sol, i_q_sol, i_d_ref, i_q_ref, V_a_sol, V_b_sol, V_c_sol, V_a_sat, V_b_sat, V_c_sat= control_system.simulate_current_loop(t_span, initial_state, i_q_ref_array, dt)

            plt.figure(figsize=(12, 7))
            plt.suptitle(f"kp = {KP}, ki = {KI}, ksat = {KSAT}")

            plt.subplot(3, 1, 1)
            plt.plot(t_span, omega_sol_rpm, label='Speed (RPM)', color='blue')
            # plt.plot(t_span, speed_reference_rpm, label='Speed Reference (RPM)', color='red', linestyle='--')
            plt.title('Motor Speed')
            plt.xlabel('Time (s)')
            plt.ylabel('Speed (RPM)')
            plt.legend()

            plt.subplot(3, 1, 2)
            plt.plot(t_span, i_d_sol, label='d-axis Current (i_d)', color='green')
            plt.plot(t_span, i_d_ref, label='d-axis Current Reference (RPM)', color='red', linestyle='--')
            plt.title('d-axis Current (i_d)')
            plt.xlabel('Time (s)')
            plt.ylabel('Current (A)')
            plt.legend()

            plt.subplot(3, 1, 3)
            plt.plot(t_span, i_q_sol, label='q-axis Current (i_q)', color='orange')
            plt.plot(t_span, i_q_ref, label='q-axis Current Reference (RPM)', color='red', linestyle='--')
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
