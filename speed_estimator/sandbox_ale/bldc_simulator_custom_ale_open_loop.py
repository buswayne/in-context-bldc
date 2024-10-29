import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

# Motor parameters based on technical data
R = 0.994  # Terminal resistance (Ohms)
L = 0.995e-3  # Terminal inductance (Henries)
Kt = 91e-3  # Torque constant (Nm/A)
Ke = 1 / 10.9956  # Back EMF constant (V/rad/s)
J = 44e-4  # Rotor inertia (kg.m^2)
B = 0.528e-3  # Mechanical damping (Nms)
V_nominal = 24  # Nominal voltage (Volts)
I_nominal = 10

n_poles_pairs = 14/2

# # PI controller parameters
# Kp_speed = 0.5  # Proportional gain for speed control
# Ki_speed = 0.005  # Integral gain for speed control
# Kp_current = 0.5  # Proportional gain for current control (d/q axis)
# Ki_current = 0.05  # Integral gain for current control (d/q axis)

# # Kp_speed = 10.0  # Proportional gain for speed control
# # Ki_speed = 0.01  # Integral gain for speed control
# # Kp_current = 2  # Proportional gain for current control (d/q axis)
# # Ki_current = 0.001  # Integral gain for current control (d/q axis)

# # Initialize PI controller states
# integral_error_speed = 0.0
# integral_error_id = 0.0
# integral_error_iq = 0.0


# Clarke Transform: Converts 3-phase (i_a, i_b, i_c) to (i_alpha, i_beta)
def clarke_transform(i_a, i_b, i_c):
    i_alpha = 2 / 3 * (i_a - i_b / 2 - i_c / 2)
    i_beta = 2 / 3 * (i_b * np.sqrt(3) / 2 - i_c * np.sqrt(3) / 2)
    return i_alpha, i_beta


# Inverse Clarke Transform: Converts (V_alpha, V_beta) back to 3-phase (V_a, V_b, V_c)
def inverse_clarke_transform(V_alpha, V_beta):
    V_a = V_alpha
    V_b = (-V_alpha + np.sqrt(3) * V_beta) / 2
    V_c = (-V_alpha - np.sqrt(3) * V_beta) / 2
    return V_a, V_b, V_c


# Park Transform: Converts (i_alpha, i_beta) to (i_d, i_q) in the rotating frame
def park_transform(i_alpha, i_beta, theta):
    i_d = i_alpha * np.cos(theta) + i_beta * np.sin(theta)
    i_q = -i_alpha * np.sin(theta) + i_beta * np.cos(theta)
    return i_d, i_q


# Inverse Park Transform: Converts (V_d, V_q) to (V_alpha, V_beta)
def inverse_park_transform(V_d, V_q, theta):
    V_alpha = V_d * np.cos(theta) - V_q * np.sin(theta)
    V_beta = V_d * np.sin(theta) + V_q * np.cos(theta)
    return V_alpha, V_beta


# Update the Back EMF calculation to account for direction
# def trapezoidal_emf(theta):
#     theta = np.mod(theta, 2 * np.pi)
#     if 0 <= theta < 2 * np.pi / 3:
#         return 1  # Phase A
#     elif 2 * np.pi / 3 <= theta < 4 * np.pi / 3:
#         return 0  # Phase B
#     else:
#         return -1  # Phase C
    
# def trapezoidal_emf(theta):
#     # theta = theta - 1/3 * np.pi
#     theta = np.mod(theta, 2 * np.pi)
#     if 0 <= theta < 2 * np.pi / 3:
#         return 1  # Phase A
#     elif np.pi <= theta < 5 * np.pi / 3:
#         return -1  # Phase B
#     else:
#         return 0  # Phase C
    
# def trapezoidal_emf(theta):
#     # theta = theta - 1/3 * np.pi
#     theta = np.mod(theta, 2 * np.pi)
#     if 0 <= theta < 1 * np.pi / 3 or 5 / 3 * np.pi <= theta < 2 * np.pi:
#         return 1  # Phase A
#     elif 2 * np.pi / 3 <= theta < 4 * np.pi / 3:
#         return -1  # Phase B
#     else:
#         return 0  # Phase C


def trapezoidal_emf(theta):
    theta = theta + 4/6 * np.pi
    theta = np.mod(theta, 2 * np.pi)
    if 0 <= theta < 1 * np.pi / 6:
        return 1 / (np.pi/6) * theta  
    
    elif np.pi / 6 <= theta < 5 * np.pi / 6:
        return 1
    
    elif 5 * np.pi / 6 <= theta < 7 * np.pi / 6:
        return -1 / (np.pi/6) * (theta - np.pi)
    
    elif 7 * np.pi / 6 <= theta < 11 * np.pi / 6:
        return -1
    
    else:
        return 1 / (np.pi/6) * (theta -2 * np.pi)
    

# # Modify the speed control to reverse the torque-producing current
# def speed_pi_controller(omega_ref_rpm, omega_rpm, dt):
#     global integral_error_speed

#     # Compute speed error
#     error = omega_rpm - omega_ref_rpm  # Invert the error calculation

#     # Update integral term
#     integral_error_speed += error * dt

#     # PI control law for i_q_ref (torque control)
#     i_q_ref = Kp_speed * error + Ki_speed * integral_error_speed

#     return i_q_ref


# Current PI controller for i_d and i_q control
# def current_pi_controller(i_ref, i_actual, integral_error, Kp, Ki, dt):
#     # Compute current error
#     error = i_ref - i_actual

#     # Update integral term
#     integral_error += error * dt

#     # PI control law for voltage
#     voltage_ref = Kp * error + Ki * integral_error

#     return voltage_ref, integral_error


# Convert angular velocity (rad/s) to RPM
def rad_s_to_rpm(omega):
    return omega * (60 / (2 * np.pi))


# Convert RPM to angular velocity (rad/s)
def rpm_to_rad_s(omega_rpm):
    return omega_rpm * (2 * np.pi / 60)


# Define BLDC motor dynamics function
def bldc_dynamics(t, state, V_a, V_b, V_c):
    i_a, i_b, i_c, omega, theta = state

    # Back EMF for each phase
    e_a = Ke * omega * trapezoidal_emf(theta) * n_poles_pairs
    e_b = Ke * omega * trapezoidal_emf(theta - 2 * np.pi / 3) * n_poles_pairs
    e_c = Ke * omega * trapezoidal_emf(theta + 2 * np.pi / 3) * n_poles_pairs

    # Electrical dynamics (di/dt for each phase)
    di_a_dt = (V_a - R * i_a - e_a) / L
    di_b_dt = (V_b - R * i_b - e_b) / L
    di_c_dt = (V_c - R * i_c - e_c) / L

    # Mechanical torque
    T_m = Kt * (i_a * trapezoidal_emf(theta) + i_b * trapezoidal_emf(theta - 2 * np.pi / 3) + i_c * trapezoidal_emf(
        theta + 2 * np.pi / 3)) * n_poles_pairs

    # Mechanical dynamics
    domega_dt = (T_m - B * omega) / J
    dtheta_dt = omega

    return [di_a_dt, di_b_dt, di_c_dt, domega_dt, dtheta_dt]


# Simulation parameters
dt = 0.001  # Sampling time (seconds)
T_max = 5
t_span = np.arange(0, T_max, dt)  # Time span for simulation (20 seconds)
initial_state = [0.0, 0.0, 0.0, 0.0, 0.0]  # Initial condition: [i_a, i_b, i_c, omega, theta]


# Initialize lists for storing results
omega_sol_rpm = [0]
theta_sol = [0]
i_d_sol = [0]
i_q_sol = [0]
V_a_sat_array = [0]
V_b_sat_array = [0]
V_c_sat_array = [0]
V_a_array = [0]
V_b_array = [0]
V_c_array = [0]

# reference_id = [0]
# reference_iq = [0]

# Initialize the integral errors for the PI controllers
# integral_error_speed = 0.0
# integral_error_id = 0.0
# integral_error_iq = 0.0

# Simulation loop
for t_idx in range(len(t_span) - 1):
    # Current state of the motor
    state = initial_state

    # Clarke Transformation: Get i_alpha and i_beta
    i_alpha, i_beta = clarke_transform(state[0], state[1], state[2])

    # Park Transformation: Get i_d and i_q
    i_d, i_q = park_transform(i_alpha, i_beta, state[4])

    # Store d and q currents
    i_d_sol.append(i_d)
    i_q_sol.append(i_q)

    # Get speed reference in RPM and apply speed PI controller
    # omega_ref_rpm = speed_reference_rpm[t_idx]
    # omega_rpm = rad_s_to_rpm(state[3])  # Convert omega from rad/s to RPM

    # i_q_ref = speed_pi_controller(omega_ref_rpm, omega_rpm, dt)

    # # Current control (for both i_d and i_q)
    # V_d, integral_error_id = current_pi_controller(i_d_ref, i_d, integral_error_id, Kp_current, Ki_current, dt)
    # V_q, integral_error_iq = current_pi_controller(i_q_ref, i_q, integral_error_iq, Kp_current, Ki_current, dt)

    V_d = 0
    V_q = V_nominal

    # Inverse Park Transform to get V_alpha and V_beta
    V_alpha, V_beta = inverse_park_transform(V_d, V_q, state[4])

    # Inverse Clarke Transform to get phase voltages V_a, V_b, V_c
    V_a, V_b, V_c = inverse_clarke_transform(V_alpha, V_beta)

    V_a_array.append(V_a)
    V_b_array.append(V_b)
    V_c_array.append(V_c)

    # Apply voltage limits (nominal voltage)
    V_a = np.clip(V_a, - V_nominal, V_nominal)
    V_b = np.clip(V_b, - V_nominal, V_nominal)
    V_c = np.clip(V_c, - V_nominal, V_nominal)

    V_a_sat_array.append(V_a)
    V_b_sat_array.append(V_b)
    V_c_sat_array.append(V_c)
    # V_a = V_nominal
    # V_b = V_nominal
    # V_c = V_nominal

    # Simulate motor dynamics using ODE solver
    sol = solve_ivp(bldc_dynamics, [0, dt], initial_state, args=(V_a, V_b, V_c), t_eval=[dt])
    # Clip to maximum currents
    # sol.y[:3, -1] = np.clip(sol.y[:3, -1], -I_nominal, I_nominal)
    initial_state = sol.y[:, -1]  # Update state with the result

    # Store the speed and position
    omega_sol_rpm.append(rad_s_to_rpm(initial_state[3]))  # Convert omega from rad/s to RPM
    theta_sol.append(initial_state[4])  # Append theta in radians

# Convert results to arrays for plotting
omega_sol_rpm = np.array(omega_sol_rpm)
theta_sol = np.array(theta_sol)
i_d_sol = np.array(i_d_sol)
i_q_sol = np.array(i_q_sol)

# Plotting
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

# plt.figure(figsize=(12, 7))

# plt.subplot(3, 1, 1)
# plt.plot(t_span, V_a_sat_array, color='blue')
# plt.plot(t_span, V_a_array, color='red', linestyle='--')
# plt.title('Phase A')
# plt.xlabel('Time (s)')

# plt.subplot(3, 1, 2)
# plt.plot(t_span, V_b_sat_array, color='blue')
# plt.plot(t_span, V_b_array, color='red', linestyle='--')
# plt.title('Phase B')
# plt.xlabel('Time (s)')

# plt.subplot(3, 1, 3)
# plt.plot(t_span, V_c_sat_array, color='blue')
# plt.plot(t_span, V_c_array, color='red', linestyle='--')
# plt.title('Phase C')
# plt.xlabel('Time (s)')

# plt.tight_layout()


plt.show()
