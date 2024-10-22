import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
from bldc_utils import *
from signals import *

class BLDCMotor:
    def __init__(self, R, L, Kt, Ke, J, B, V_nominal, I_nominal, P):
        self.R = R
        self.L = L
        self.Kt = Kt
        self.Ke = Ke
        self.J = J
        self.B = B
        self.V_nominal = V_nominal
        self.I_nominal = I_nominal
        self.P = P

    def trapezoidal_emf(self, theta):
        theta = theta + 4 / 6 * np.pi
        theta = np.mod(theta, 2 * np.pi)
        if 0 <= theta < 1 * np.pi / 6:
            return 1 / (np.pi / 6) * theta
        elif np.pi / 6 <= theta < 5 * np.pi / 6:
            return 1
        elif 5 * np.pi / 6 <= theta < 7 * np.pi / 6:
            return -1 / (np.pi / 6) * (theta - np.pi)
        elif 7 * np.pi / 6 <= theta < 11 * np.pi / 6:
            return -1
        else:
            return 1 / (np.pi / 6) * (theta - 2 * np.pi)

    def dynamics(self, t, state, V_a, V_b, V_c):
        i_a, i_b, i_c, omega, theta = state

        # Back EMF for each phase
        e_a = self.Ke * omega * self.trapezoidal_emf(theta) * self.P
        e_b = self.Ke * omega * self.trapezoidal_emf(theta - 2 * np.pi / 3) * self.P
        e_c = self.Ke * omega * self.trapezoidal_emf(theta + 2 * np.pi / 3) * self.P

        # Electrical dynamics (di/dt for each phase)
        di_a_dt = (V_a - self.R * i_a - e_a) / self.L
        di_b_dt = (V_b - self.R * i_b - e_b) / self.L
        di_c_dt = (V_c - self.R * i_c - e_c) / self.L

        # Mechanical torque
        T_m = self.Kt * (i_a * self.trapezoidal_emf(theta) +
                         i_b * self.trapezoidal_emf(theta - 2 * np.pi / 3) +
                         i_c * self.trapezoidal_emf(theta + 2 * np.pi / 3)) * self.P

        # Mechanical dynamics
        domega_dt = (T_m - self.B * omega) / self.J
        dtheta_dt = omega

        return [di_a_dt, di_b_dt, di_c_dt, domega_dt, dtheta_dt]

    def simulate(self, initial_state, t_span, V_q_steps):
        omega_sol_rpm = [0]
        theta_sol = [0]
        i_d_sol = [0]
        i_q_sol = [0]

        current_step = 0

        for t_idx in range(len(t_span) - 1):
            V_q = V_q_steps[t_idx]
            V_d = 0.0
            V_alpha, V_beta = inverse_park_transform(V_d, V_q, initial_state[4])
            V_a, V_b, V_c = inverse_clarke_transform(V_alpha, V_beta)

            V_a = np.clip(V_a, -self.V_nominal, self.V_nominal)
            V_b = np.clip(V_b, -self.V_nominal, self.V_nominal)
            V_c = np.clip(V_c, -self.V_nominal, self.V_nominal)

            sol = solve_ivp(self.dynamics, [0, dt], initial_state, args=(V_a, V_b, V_c), t_eval=[dt])
            initial_state = sol.y[:, -1]

            i_alpha, i_beta = clarke_transform(initial_state[0], initial_state[1], initial_state[2])
            i_d, i_q = park_transform(i_alpha, i_beta, initial_state[4])

            omega_sol_rpm.append(rad_s_to_rpm(initial_state[3]))
            theta_sol.append(initial_state[4])
            i_d_sol.append(i_d)
            i_q_sol.append(i_q)

        return t_span, omega_sol_rpm, theta_sol, i_d_sol, i_q_sol

# Main simulation loop
R, L, Kt, Ke, J, B = 0.994, 0.995e-3, 91e-3, 1 / 10.9956, 44e-4, 0.528e-3
V_nominal, I_nominal = 48, 10
P = 4  # Number of pole pairs
Kt, Ke = Kt / P, Ke / P

dt = 0.01
T_max = 20
t_span = np.arange(0, T_max, dt)
initial_state = [0.0, 0.0, 0.0, 0.0, 0.0]
d_min, d_max = 1, 5

# Number of experiments
num_experiments = 1000

# Perturbation range: ±100%
perturb_percentage = 1.0

for experiment in range(555, num_experiments):
    # Randomly perturb motor parameters within ±100% of their nominal values
    R_perturbed = R * (1 + perturb_percentage * np.random.uniform(-1, 1))
    L_perturbed = L * (1 + perturb_percentage * np.random.uniform(-1, 1))
    Kt_perturbed = Kt * (1 + perturb_percentage * np.random.uniform(-1, 1))
    Ke_perturbed = Ke * (1 + perturb_percentage * np.random.uniform(-1, 1))
    J_perturbed = J * (1 + perturb_percentage * np.random.uniform(-1, 1))
    B_perturbed = B * (1 + perturb_percentage * np.random.uniform(-1, 1))

    # Initialize the motor with perturbed parameters
    motor = BLDCMotor(R_perturbed, L_perturbed, Kt_perturbed, Ke_perturbed, J_perturbed, B_perturbed, V_nominal, I_nominal, P)

    # Generate random step sequence for Vq
    num_steps = np.random.randint(3, 10)  # Random number of steps

    V_q_steps = steps_sequence(T_max, dt, -V_nominal, V_nominal, d_min, d_max).flatten()

    V_d_steps = np.zeros_like(V_q_steps)
    r = np.zeros_like(V_q_steps)

    # Run the simulation
    t_span, omega_sol_rpm, theta_sol, i_d_sol, i_q_sol = motor.simulate(initial_state, t_span, V_q_steps)

    data = np.array([t_span, i_q_sol, i_d_sol, V_q_steps, V_d_steps, omega_sol_rpm, r])
    df = pd.DataFrame(data.T, columns=['timestamp','iq','id','vq','vd','omega','r'])

    df.to_csv('../data/simulated/OL/experiment_' + str(experiment) + '.csv', index=False)
    # Here you can save or analyze the results of the experiment
    print(f"Experiment {experiment + 1} complete.")

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
