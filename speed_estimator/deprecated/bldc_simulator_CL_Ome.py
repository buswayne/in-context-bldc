import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
from bldc_utils import *
from signals import *
import wandb
from sandbox_ale.bldc_simulator_ale import *
import datetime
import os
# from pathlib import Path

current_path = os.getcwd().split("in-context-bldc")[0]
data_path_tmp = os.path.join(current_path, 'in-context-bldc\\data\\simulated\\CL_speed')


date_time_str = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
print(date_time_str)

wandb.init(
    # set the wandb project where this run will be logged
    project="in-context bldc estimator",
    name="data_generator"
)

def main():
    # Main simulation loop
    params = MotorParameters(
        R=0.994,
        L=0.995e-3,
        Kt=91e-3,
        Ke=1 / 10.9956,
        J=44e-7,
        B=0.0083,
        V_nominal=48,
        I_nominal=10,
        Pole_pairs=14/2
    )

    dt = 0.001
    T_max = 10
    t_span = np.arange(0, T_max, dt)
    initial_state = [0.0, 0.0, 0.0, 0.0, 0.0]
    
    # step duration limits
    d_min, d_max = 1, 5
    #speed range
    speed_min = 0
    speed_max = 500

    # Number of experiments
    num_experiments = 2000

    # Perturbation range: ±100%
    perturb_percentage = 0.70

    for experiment in range(num_experiments):
        progress = experiment/num_experiments
        print("progress: ", progress*100, "%")
        # Randomly perturb motor parameters within ±100% of their nominal values
        R_perturbed = params.R * (1 + perturb_percentage * np.random.uniform(-1, 1))
        L_perturbed = params.L * (1 + perturb_percentage * np.random.uniform(-1, 1))
        Kt_perturbed = params.Kt * (1 + perturb_percentage * np.random.uniform(-1, 1))
        Ke_perturbed = params.Ke * (1 + perturb_percentage * np.random.uniform(-1, 1))
        J_perturbed = params.J * (1 + perturb_percentage * np.random.uniform(-1, 1))
        B_perturbed = params.B * (1 + perturb_percentage * np.random.uniform(-1, 1))

        # Initialize the motor with perturbed parameters
        params_perturbed = MotorParameters(
        R=R_perturbed,
        L=L_perturbed,
        Kt=Kt_perturbed,
        Ke=Ke_perturbed,
        J=J_perturbed,
        B=B_perturbed,
        V_nominal=params.V_nominal,
        I_nominal=params.I_nominal,
        Pole_pairs=params.P
    )

        bldc_motor = BLDCMotor(params_perturbed)
        control_system = BLDCControlSystem(bldc_motor, Kp_speed=0.5, Ki_speed=10, Kp_current=0.01, Ki_current=100, K_sat_speed=2)

        # Generate random step sequence for Vq
        speed_reference_rpm = steps_sequence(T_max, dt, speed_min, speed_max, d_min, d_max).flatten()
        r = np.zeros_like(speed_reference_rpm)

        # Run the simulation
        # t_span, omega_sol_rpm, theta_sol, i_d_sol, i_q_sol = motor.simulate(dt, initial_state, t_span, V_q_steps)
        omega_sol_rpm, theta_sol, i_d_sol, i_q_sol, i_d_ref, i_q_ref, V_a_sol, V_b_sol, V_c_sol, V_a_sat, V_b_sat, V_c_sat, v_d, v_q = control_system.simulate_speed_loop(t_span, initial_state, speed_reference_rpm, dt)

        data = np.array([t_span, i_q_sol, i_d_sol, i_q_ref, i_d_ref, v_q, v_d, omega_sol_rpm, speed_reference_rpm, r])
        df = pd.DataFrame(data.T, columns=['timestamp','iq','id','iq_ref','id_ref','vq','vd','omega','omega_ref', 'r'])

        # print(data_path_tmp)
        exp_name = 'experiment_' + date_time_str + '_' + str(experiment) + '.csv'
        obj_path = os.path.join(data_path_tmp, exp_name)
        # print(obj_path)
        # input()
        df.to_csv(obj_path, index=False)

        # Here you can save or analyze the results of the experiment
        print(f"Experiment {experiment + 1} complete.")
        #
        # # Plotting
        # plt.figure(figsize=(12, 10))
        #
        # plt.subplot(3, 1, 1)
        # plt.plot(t_span, omega_sol_rpm, label='Speed (RPM)', color='blue')
        # plt.title('Motor Speed')
        # plt.xlabel('Time (s)')
        # plt.ylabel('Speed (RPM)')
        # plt.grid()
        # plt.legend()
        #
        # plt.subplot(3, 1, 2)
        # plt.plot(t_span, i_d_sol, label='Direct Current (A)', color='green')
        # plt.plot(t_span, i_q_sol, label='Quadrature Current (A)', color='orange')
        # plt.title('Direct and Quadrature Current')
        # plt.xlabel('Time (s)')
        # plt.ylabel('Current (A)')
        # plt.grid()
        # plt.legend()
        #
        # plt.subplot(3, 1, 3)
        # plt.plot(t_span, theta_sol, label='Theta (rad)', color='purple')
        # plt.title('Rotor Position')
        # plt.xlabel('Time (s)')
        # plt.ylabel('Theta (rad)')
        # plt.grid()
        # plt.legend()
        #
        # plt.tight_layout()
        # plt.show()

if __name__ == "__main__":
    main()
