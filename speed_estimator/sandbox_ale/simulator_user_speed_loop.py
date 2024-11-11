import numpy as np
import matplotlib.pyplot as plt
from bldc_simulator_ale import *

params = MotorParameters(
    R=0.994,  ## Ohm
    L=0.995e-3, ## H = Ohm*s = V*s/A
    Kt=91e-3,  ## N*m/A
    Ke=1 / 10.9956, ##V*s/rad
    J=44e-7, ## kg*m^2
    B=0.0083, #44e-7/0.528e-3 kg*m^2/s
    V_nominal=48, ## V
    I_nominal=10, ## A
    Pole_pairs=14/2
)
bldc_motor = BLDCMotor(params)

dt = 0.0001
t_span = np.arange(0, 10, dt)
initial_state = [0.0, 0.0, 0.0, 0.0, 0.0]
speed_reference_rpm = np.zeros(len(t_span))
# step_times = [1, 5, 15]
# step_values_rpm = [100, 250, 500]
step_times = [5]
step_values_rpm = [1000]

for i in range(len(step_times)):
    speed_reference_rpm[t_span >= step_times[i]] = step_values_rpm[i]

kp_list = [0.5]
ki_list = [10]
k_sat_list = [2]
index = 0
for KP in kp_list:
    for KI in ki_list:
        for KSAT in k_sat_list:
            progress = 100* (index/(len(kp_list)*len(ki_list)*len(k_sat_list)))
            print(f"Simulating for: kp = {KP}, ki = {KI}, ksat = {KSAT}, progress: {progress}%")
            index += 1


            # Initialize control system
            control_system = BLDCControlSystem(bldc_motor, Kp_speed=KP, Ki_speed=KI, Kp_current=0.01, Ki_current=100, K_sat_speed=KSAT)

            omega_sol_rpm, theta_sol, i_d_sol, i_q_sol, i_d_ref, i_q_ref, V_a_sol, V_b_sol, V_c_sol, V_a_sat, V_b_sat, V_c_sat = control_system.simulate_speed_loop(t_span, initial_state, speed_reference_rpm, dt)

            plt.figure(figsize=(12, 7))
            plt.suptitle(f"kp = {KP}, ki = {KI}, ksat = {KSAT}")

            plt.subplot(3, 1, 1)
            plt.plot(t_span, omega_sol_rpm, label='Speed (RPM)', color='blue')
            plt.plot(t_span, speed_reference_rpm, label='Speed Reference (RPM)', color='red', linestyle='--')
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
