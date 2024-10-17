import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

# Define BLDC motor parameters
R = 0.5  # Resistance (Ohms)
L = 0.001  # Inductance (Henries)
Ke = 0.01  # Back EMF constant (V/rad/s)
Kt = 0.01  # Torque constant (Nm/A)
J = 0.01  # Rotor inertia (kg.m^2)
B = 0.001  # Damping coefficient (Nms)


# PI Controller for FOC
class PIController:
    def __init__(self, kp, ki):
        self.kp = kp
        self.ki = ki
        self.integral = 0

    def control(self, error, dt):
        self.integral += error * dt
        return self.kp * error + self.ki * self.integral


# Clarke Transformation (ABC -> Alpha-Beta frame)
def clarke_transform(i_a, i_b, i_c):
    i_alpha = i_a
    i_beta = (i_a + 2 * i_b) / np.sqrt(3)
    return i_alpha, i_beta


# Park Transformation (Alpha-Beta -> DQ frame)
def park_transform(i_alpha, i_beta, theta):
    i_d = i_alpha * np.cos(theta) + i_beta * np.sin(theta)
    i_q = -i_alpha * np.sin(theta) + i_beta * np.cos(theta)
    return i_d, i_q


# Inverse Park Transformation (DQ -> Alpha-Beta frame)
def inverse_park_transform(v_d, v_q, theta):
    v_alpha = v_d * np.cos(theta) - v_q * np.sin(theta)
    v_beta = v_d * np.sin(theta) + v_q * np.cos(theta)
    return v_alpha, v_beta


# Inverse Clarke Transformation (Alpha-Beta -> ABC frame)
def inverse_clarke_transform(v_alpha, v_beta):
    v_a = v_alpha
    v_b = (-v_alpha + np.sqrt(3) * v_beta) / 2
    v_c = (-v_alpha - np.sqrt(3) * v_beta) / 2
    return v_a, v_b, v_c


# Define the BLDC motor dynamics function
# Define the BLDC motor dynamics function
def bldc_dynamics(t, state, V_a, V_b, V_c, i_d_ref, i_q_ref):
    i_a, i_b, i_c, omega, theta = state

    # Back EMF for each phase
    e_a = Ke * omega * trapezoidal_emf(theta)
    e_b = Ke * omega * trapezoidal_emf(theta - 2 * np.pi / 3)
    e_c = Ke * omega * trapezoidal_emf(theta + 2 * np.pi / 3)

    # Electrical dynamics (di/dt for each phase)
    di_a_dt = (V_a - R * i_a - e_a) / L
    di_b_dt = (V_b - R * i_b - e_b) / L
    di_c_dt = (V_c - R * i_c - e_c) / L

    # Mechanical torque (assumes perfect control of currents)
    T_m = Kt * (i_a * trapezoidal_emf(theta) + i_b * trapezoidal_emf(theta - 2 * np.pi / 3) + i_c * trapezoidal_emf(
        theta + 2 * np.pi / 3))

    # Mechanical dynamics
    domega_dt = (T_m - B * omega) / J
    dtheta_dt = omega

    return [di_a_dt, di_b_dt, di_c_dt, domega_dt, dtheta_dt]


# Back EMF function (trapezoidal)
def trapezoidal_emf(theta):
    theta = np.mod(theta, 2 * np.pi)
    if 0 <= theta < 2 * np.pi / 3:
        return 1
    elif 2 * np.pi / 3 <= theta < 4 * np.pi / 3:
        return 0
    else:
        return -1

# Simulation parameters
dt = 0.01  # Sampling time (seconds)
t_span = np.arange(0, 20, dt)  # Time span for simulation (20 seconds)
initial_state = [0.0, 0.0, 0.0, 0.0, 0.0]  # Initial condition: [i_a, i_b, i_c, omega, theta]

# Define the desired reference currents (perfectly controlled)
i_d_ref = 0.0  # Flux current reference (d-axis)
i_q_ref = 5.0  # Torque current reference (q-axis)

# Generate speed reference signal (step changes)
speed_reference = np.zeros(len(t_span))
step_times = [5, 10, 15]  # Speed step changes at 5s, 10s, and 15s
step_values = [5, 10, 15]  # Corresponding speed values

for i in range(len(step_times)):
    speed_reference[t_span >= step_times[i]] = step_values[i]

# Initialize lists for storing results
omega_sol = []
theta_sol = []

for t_idx in range(len(t_span) - 1):
    # Current state of the motor
    state = initial_state

    # Clarke Transformation: Get i_alpha and i_beta
    i_alpha, i_beta = clarke_transform(state[0], state[1], state[2])

    # Park Transformation: Get i_d and i_q
    i_d, i_q = park_transform(i_alpha, i_beta, state[4])

    # Set the phase voltages based on perfect control of currents
    V_d = 0.0  # Set d-axis voltage to maintain i_d_ref
    V_q = i_q_ref  # Set q-axis voltage based on torque reference
    V_alpha, V_beta = inverse_park_transform(V_d, V_q, state[4])

    # Inverse Clarke Transformation to get phase voltages V_a, V_b, V_c
    V_a, V_b, V_c = inverse_clarke_transform(V_alpha, V_beta)

    # Simulate motor dynamics for one time step using solve_ivp
    sol = solve_ivp(bldc_dynamics, [t_span[t_idx], t_span[t_idx + 1]], initial_state,
                    args=(V_a, V_b, V_c, i_d_ref, i_q_ref), method='RK45', rtol=1e-6, atol=1e-6)

    # Update state with the last result
    initial_state = sol.y[:, -1]

    # Store results
    omega_sol.append(initial_state[3])  # Angular velocity
    theta_sol.append(initial_state[4])  # Rotor position

# Plot the results
plt.figure(figsize=(10, 8))

# Plot rotor angular velocity
plt.subplot(2, 1, 1)
plt.plot(t_span[:-1], omega_sol, label='Rotor Angular Velocity')
plt.plot(t_span, speed_reference, '--', label='Speed Reference', color='red')
plt.title('Rotor Angular Velocity vs Time (with FOC)')
plt.xlabel('Time [s]')
plt.ylabel('Angular Velocity [rad/s]')
plt.legend()
plt.grid(True)

# Plot rotor position
plt.subplot(2, 1, 2)
plt.plot(t_span[:-1], theta_sol, label='Rotor Position')
plt.title('Rotor Position vs Time')
plt.xlabel('Time [s]')
plt.ylabel('Position [rad]')
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.show()
