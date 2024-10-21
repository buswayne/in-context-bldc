import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp


class MotorParameters:
    def __init__(self, R, L, Kt, Ke, J, B, V_nominal, I_nominal):
        self.R = R  # Terminal resistance
        self.L = L  # Terminal inductance
        self.Kt = Kt  # Torque constant
        self.Ke = Ke  # Back EMF constant
        self.J = J  # Rotor inertia
        self.B = B  # Mechanical damping
        self.V_nominal = V_nominal  # Nominal voltage
        self.I_nominal = I_nominal  # Nominal current


class PIController:
    def __init__(self, Kp, Ki):
        self.Kp = Kp
        self.Ki = Ki
        self.integral_error = 0.0

    def compute(self, error, dt):
        self.integral_error += error * dt
        return self.Kp * error + self.Ki * self.integral_error


class ClarkeParkTransforms:
    @staticmethod
    def clarke_transform(i_a, i_b, i_c):
        i_alpha = (2/3) * (i_a - 0.5 * i_b - 0.5 * i_c)
        i_beta = (2/3) * ((np.sqrt(3)/2) * (i_b - i_c))
        return i_alpha, i_beta

    @staticmethod
    def inverse_clarke_transform(V_alpha, V_beta):
        V_a = V_alpha
        V_b = -0.5 * V_alpha + (np.sqrt(3)/2) * V_beta
        V_c = -0.5 * V_alpha - (np.sqrt(3)/2) * V_beta
        return V_a, V_b, V_c

    @staticmethod
    def park_transform(i_alpha, i_beta, theta):
        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)
        i_d = i_alpha * cos_theta + i_beta * sin_theta
        i_q = -i_alpha * sin_theta + i_beta * cos_theta
        return i_d, i_q

    @staticmethod
    def inverse_park_transform(V_d, V_q, theta):
        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)
        V_alpha = V_d * cos_theta - V_q * sin_theta
        V_beta = V_d * sin_theta + V_q * cos_theta
        return V_alpha, V_beta


class BLDCMotor:
    def __init__(self, params: MotorParameters):
        self.params = params

    def trapezoidal_emf(self, theta):
        # Normalize theta within 0 to 2*pi
        theta = np.mod(theta, 2 * np.pi)

        # Calculate trapezoidal EMF based on the phase angle
        if 0 <= theta < 2 * np.pi / 3:
            return 1
        elif 2 * np.pi / 3 <= theta < np.pi:
            return (2 * np.pi / 3 - theta) / (np.pi / 3)  # Linearly decreasing
        elif np.pi <= theta < 4 * np.pi / 3:
            return -1
        elif 4 * np.pi / 3 <= theta < 5 * np.pi / 3:
            return (theta - 4 * np.pi / 3) / (np.pi / 3)  # Linearly increasing
        else:
            return 1

    def dynamics(self, t, state, V_a, V_b, V_c):
        i_a, i_b, i_c, omega, theta = state

        # Back EMF for each phase
        e_a = self.params.Ke * omega * self.trapezoidal_emf(theta)
        e_b = self.params.Ke * omega * self.trapezoidal_emf(theta - 2 * np.pi / 3)
        e_c = self.params.Ke * omega * self.trapezoidal_emf(theta + 2 * np.pi / 3)

        # Electrical dynamics (di/dt for each phase)
        di_a_dt = (V_a - self.params.R * i_a - e_a) / self.params.L
        di_b_dt = (V_b - self.params.R * i_b - e_b) / self.params.L
        di_c_dt = (V_c - self.params.R * i_c - e_c) / self.params.L

        # Mechanical torque
        T_m = self.params.Kt * (
            i_a * self.trapezoidal_emf(theta) +
            i_b * self.trapezoidal_emf(theta - 2 * np.pi / 3) +
            i_c * self.trapezoidal_emf(theta + 2 * np.pi / 3)
        )

        # Mechanical dynamics
        domega_dt = (T_m - self.params.B * omega) / self.params.J
        dtheta_dt = omega

        return [di_a_dt, di_b_dt, di_c_dt, domega_dt, dtheta_dt]


class BLDCControlSystem:
    def __init__(self, motor: BLDCMotor, Kp_speed, Ki_speed, Kp_current, Ki_current):
        self.motor = motor
        self.speed_controller = PIController(Kp_speed, Ki_speed)
        self.current_controller_d = PIController(Kp_current, Ki_current)
        self.current_controller_q = PIController(Kp_current, Ki_current)
        self.transforms = ClarkeParkTransforms()

    def simulate(self, t_span, initial_state, speed_reference_rpm, dt):
        omega_sol_rpm = [0]
        theta_sol = [0]
        i_d_sol = [0]
        i_q_sol = [0]

        # Simulation loop
        for t_idx in range(len(t_span) - 1):
            state = initial_state

            # Clarke Transformation
            i_alpha, i_beta = self.transforms.clarke_transform(state[0], state[1], state[2])

            # Park Transformation
            i_d, i_q = self.transforms.park_transform(i_alpha, i_beta, state[4])

            i_d_sol.append(i_d)
            i_q_sol.append(i_q)

            # Speed PI controller
            omega_ref_rpm = speed_reference_rpm[t_idx]
            omega_rpm = self.rad_s_to_rpm(state[3])
            # Compute the speed error using the correct convention: ref - value
            i_q_ref = self.speed_controller.compute(omega_ref_rpm - omega_rpm, dt)
            print(i_q_ref)
            # Current control for i_d and i_q using ref - value convention
            i_d_ref = 0
            V_d = self.current_controller_d.compute(i_d_ref - i_d, dt)  # d-axis current control
            V_q = self.current_controller_q.compute(i_q_ref - i_q, dt)  # q-axis current control

            V_d = 10
            V_q = 0

            # Inverse Park and Clarke Transform
            V_alpha, V_beta = self.transforms.inverse_park_transform(V_d, V_q, state[4])
            V_a, V_b, V_c = self.transforms.inverse_clarke_transform(V_alpha, V_beta)

            # Voltage limits
            V_a = np.clip(V_a, -self.motor.params.V_nominal, self.motor.params.V_nominal)
            V_b = np.clip(V_b, -self.motor.params.V_nominal, self.motor.params.V_nominal)
            V_c = np.clip(V_c, -self.motor.params.V_nominal, self.motor.params.V_nominal)

            # Simulate motor dynamics
            sol = solve_ivp(self.motor.dynamics, [0, dt], initial_state, args=(V_a, V_b, V_c), t_eval=[dt])
            # Clip to maximum currents
            sol.y[:3, -1] = np.clip(sol.y[:3, -1], -self.motor.params.I_nominal, self.motor.params.I_nominal)
            initial_state = sol.y[:, -1]

            omega_sol_rpm.append(self.rad_s_to_rpm(initial_state[3]))
            theta_sol.append(initial_state[4])

        return np.array(omega_sol_rpm), np.array(theta_sol), np.array(i_d_sol), np.array(i_q_sol)

    @staticmethod
    def rad_s_to_rpm(omega):
        return omega * (60 / (2 * np.pi))


# Motor parameters (example values)
params = MotorParameters(
    R=0.994,
    L=0.995e-3,
    Kt=91e-3,
    Ke=1 / 10.9956,
    J=44e-4,
    B=0.528e-3,
    V_nominal=24,
    I_nominal=10
)

# Simulation parameters
dt = 0.01
t_span = np.arange(0, 20, dt)
initial_state = [0.0, 0.0, 0.0, 0.0, 0.0]
speed_reference_rpm = np.zeros(len(t_span))
step_times = [1, 5, 15]
step_values_rpm = [500, 1000, 1500]

for i in range(len(step_times)):
    speed_reference_rpm[t_span >= step_times[i]] = step_values_rpm[i]

# Initialize control system
bldc_motor = BLDCMotor(params)
control_system = BLDCControlSystem(bldc_motor, Kp_speed=0.5, Ki_speed=0.005, Kp_current=5, Ki_current=0.5)

# Run simulation
omega_sol_rpm, theta_sol, i_d_sol, i_q_sol = control_system.simulate(t_span, initial_state, speed_reference_rpm, dt)

# Plot results
plt.figure(figsize=(12, 8))

plt.subplot(3, 1, 1)
plt.plot(t_span, omega_sol_rpm, label='Speed (RPM)', color='blue')
plt.plot(t_span, speed_reference_rpm, label='Speed Reference (RPM)', color='red', linestyle='--')
plt.title('Motor Speed')
plt.xlabel('Time (s)')
plt.ylabel('Speed (RPM)')
plt.legend()

plt.subplot(3, 1, 2)
plt.plot(t_span, i_d_sol, label='d-axis Current', color='orange')
plt.plot(t_span, i_q_sol, label='q-axis Current', color='green')
plt.title('dq Currents')
plt.xlabel('Time (s)')
plt.ylabel('Current (A)')
plt.legend()

plt.subplot(3, 1, 3)
plt.plot(t_span, theta_sol, label='Rotor Position (rad)', color='purple')
plt.title('Rotor Position')
plt.xlabel('Time (s)')
plt.ylabel('Position (rad)')
plt.legend()

plt.tight_layout()
plt.show()
