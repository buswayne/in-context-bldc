import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp


class MotorParameters:
    def __init__(self, R, L, Kt, Ke, J, B, V_nominal, I_nominal, Pole_pairs):
        self.R = R  # Terminal resistance
        self.L = L  # Terminal inductance
        self.Kt = Kt  # Torque constant
        self.Ke = Ke  # Back EMF constant
        self.J = J  # Rotor inertia
        self.B = B  # Mechanical damping
        self.V_nominal = V_nominal  # Nominal voltage
        self.I_nominal = I_nominal  # Nominal current
        self.P = Pole_pairs # Number of poles PAIRS


class PIController:
    def __init__(self, Kp, Ki, K_sat = 1.0):
        self.Kp = Kp
        self.Ki = Ki
        self.integral_error = 0.0
        self.K_sat = K_sat

    def compute(self, error, dt, delta_saturation = 0.0):
        self.integral_error += (error + self.K_sat*delta_saturation) * dt #anti windup
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
        # theta = theta + 4/6 * np.pi
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

    def dynamics(self, t, state, V_a, V_b, V_c):
        i_a, i_b, i_c, omega, theta = state

        # Back EMF for each phase  ## V
        e_a = self.params.Ke * omega * self.trapezoidal_emf(theta) * self.params.P
        e_b = self.params.Ke * omega * self.trapezoidal_emf(theta - 2 * np.pi / 3) * self.params.P
        e_c = self.params.Ke * omega * self.trapezoidal_emf(theta + 2 * np.pi / 3) * self.params.P

        # Electrical dynamics (di/dt for each phase) ## A/s
        di_a_dt = (V_a - self.params.R * i_a - e_a) / self.params.L
        di_b_dt = (V_b - self.params.R * i_b - e_b) / self.params.L
        di_c_dt = (V_c - self.params.R * i_c - e_c) / self.params.L

        # Mechanical torque ## Nm
        T_m = self.params.Kt * (
            i_a * self.trapezoidal_emf(theta) +
            i_b * self.trapezoidal_emf(theta - 2 * np.pi / 3) +
            i_c * self.trapezoidal_emf(theta + 2 * np.pi / 3)
        ) * self.params.P

        # Mechanical dynamics ## 1/s^2, 1/s
        domega_dt = (T_m - self.params.B * omega) / self.params.J
        dtheta_dt = omega

        return [di_a_dt, di_b_dt, di_c_dt, domega_dt, dtheta_dt]


class BLDCControlSystem:
    def __init__(self, motor: BLDCMotor, Kp_speed, Ki_speed, Kp_current, Ki_current, K_sat_speed = 1.0, K_sat_current = 1.0):
        self.motor = motor
        self.speed_controller = PIController(Kp_speed, Ki_speed, K_sat_speed)
        self.current_controller_d = PIController(Kp_current, Ki_current, K_sat_current)
        self.current_controller_q = PIController(Kp_current, Ki_current, K_sat_current)
        self.transforms = ClarkeParkTransforms()

    def simulate_speed_loop(self, t_span, initial_state, speed_reference_rpm, dt):
        # data to save
        omega_sol_rpm = [0]
        theta_sol = [0]
        i_d_sol = [0]
        i_q_sol = [0]
        i_d_ref_array = [0]
        i_q_ref_array = [0]
        V_d_sol = [0]
        V_q_sol = [0]

        V_a_sat_array = [0]
        V_b_sat_array = [0]
        V_c_sat_array = [0]
        V_a_array = [0]
        V_b_array = [0]
        V_c_array = [0]
        
        # init anti-windup 
        V_d_delta_sat = 0
        V_q_delta_sat = 0
        i_q_delta_sat = 0

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
            i_q_ref = self.speed_controller.compute(omega_ref_rpm - omega_rpm, dt, i_q_delta_sat)
            # print(omega_ref_rpm - omega_rpm)
            i_q_sat = np.clip(i_q_ref, -self.motor.params.I_nominal, self.motor.params.I_nominal)
            i_q_delta_sat = i_q_sat - i_q_ref

            i_q_ref = i_q_sat

            
            i_d_ref_array.append(0)
            i_q_ref_array.append(i_q_ref)
            # Current control for i_d and i_q using ref - value convention
            i_d_ref = 0
            V_d = self.current_controller_d.compute(i_d_ref - i_d, dt, V_d_delta_sat)  # d-axis current control
            V_q = self.current_controller_q.compute(i_q_ref - i_q, dt, V_q_delta_sat)  # q-axis current control

            # Inverse Park and Clarke Transform
            V_alpha, V_beta = self.transforms.inverse_park_transform(V_d, V_q, state[4])
            V_a, V_b, V_c = self.transforms.inverse_clarke_transform(V_alpha, V_beta)
            V_a_array.append(V_a)
            V_b_array.append(V_b)
            V_c_array.append(V_c)

            # Voltage limits
            V_a = np.clip(V_a, -self.motor.params.V_nominal, self.motor.params.V_nominal)
            V_b = np.clip(V_b, -self.motor.params.V_nominal, self.motor.params.V_nominal)
            V_c = np.clip(V_c, -self.motor.params.V_nominal, self.motor.params.V_nominal)

            V_a_sat_array.append(V_a)
            V_b_sat_array.append(V_b)
            V_c_sat_array.append(V_c)

            V_alpha_tmp, V_beta_tmp = self.transforms.clarke_transform(V_a, V_b, V_c)

            V_q_tmp, V_d_tmp = self.transforms.park_transform(V_alpha_tmp, V_beta_tmp, state[4])

            V_d_delta_sat = V_d_tmp - V_d
            V_q_delta_sat = V_q_tmp - V_q

            V_d_sol.append(V_d_tmp)
            V_q_sol.append(V_q_tmp)

            # Simulate motor dynamics
            sol = solve_ivp(self.motor.dynamics, [0, dt], initial_state, args=(V_a, V_b, V_c), t_eval=[dt])
            # Clip to maximum currents
            # sol.y[:3, -1] = np.clip(sol.y[:3, -1], -self.motor.params.I_nominal, self.motor.params.I_nominal)
            initial_state = sol.y[:, -1]

            omega_sol_rpm.append(self.rad_s_to_rpm(initial_state[3]))
            theta_sol.append(initial_state[4])

        return np.array(omega_sol_rpm), np.array(theta_sol), np.array(i_d_sol), np.array(i_q_sol), np.array(i_d_ref_array), np.array(i_q_ref_array), np.array(V_a_array), np.array(V_b_array), np.array(V_c_array), np.array(V_a_sat_array), np.array(V_b_sat_array), np.array(V_c_sat_array), np.array(V_d_sol), np.array(V_q_sol)
    
    def simulate_open_loop(self, t_span, initial_state, v_d_ref, v_q_ref, dt):
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

        # Simulation loop
        for t_idx in range(len(t_span) - 1):
            state = initial_state

            # Clarke Transformation
            i_alpha, i_beta = self.transforms.clarke_transform(state[0], state[1], state[2])

            # Park Transformation
            i_d, i_q = self.transforms.park_transform(i_alpha, i_beta, state[4])

            i_d_sol.append(i_d)
            i_q_sol.append(i_q)

            V_d = v_d_ref[t_idx]
            V_q = v_q_ref[t_idx]


            # Inverse Park and Clarke Transform
            V_alpha, V_beta = self.transforms.inverse_park_transform(V_d, V_q, state[4])
            V_a, V_b, V_c = self.transforms.inverse_clarke_transform(V_alpha, V_beta)

            V_a_array.append(V_a)
            V_b_array.append(V_b)
            V_c_array.append(V_c)

            # Voltage limits
            V_a = np.clip(V_a, -self.motor.params.V_nominal, self.motor.params.V_nominal)
            V_b = np.clip(V_b, -self.motor.params.V_nominal, self.motor.params.V_nominal)
            V_c = np.clip(V_c, -self.motor.params.V_nominal, self.motor.params.V_nominal)

            V_a_sat_array.append(V_a)
            V_b_sat_array.append(V_b)
            V_c_sat_array.append(V_c)

            # Simulate motor dynamics
            sol = solve_ivp(self.motor.dynamics, [0, dt], initial_state, args=(V_a, V_b, V_c), t_eval=[dt])
            # Clip to maximum currents
            # sol.y[:3, -1] = np.clip(sol.y[:3, -1], -self.motor.params.I_nominal, self.motor.params.I_nominal)
            initial_state = sol.y[:, -1]

            omega_sol_rpm.append(self.rad_s_to_rpm(initial_state[3]))
            theta_sol.append(initial_state[4])

        return np.array(omega_sol_rpm), np.array(theta_sol), np.array(i_d_sol), np.array(i_q_sol), np.array(V_a_array), np.array(V_b_array), np.array(V_c_array), np.array(V_a_sat_array), np.array(V_b_sat_array), np.array(V_c_sat_array) 
    

    def simulate_current_loop(self, t_span, initial_state, current_reference, dt):
        # data to save
        omega_sol_rpm = [0]
        theta_sol = [0]
        i_d_sol = [0]
        i_q_sol = [0]
        i_d_ref_array = [0]
        i_q_ref_array = [0]

        V_a_sat_array = [0]
        V_b_sat_array = [0]
        V_c_sat_array = [0]
        V_a_array = [0]
        V_b_array = [0]
        V_c_array = [0]

        
        # init anti-windup 
        V_d_delta_sat = 0
        V_q_delta_sat = 0

        # Simulation loop
        for t_idx in range(len(t_span) - 1):
            state = initial_state

            # Clarke Transformation
            i_alpha, i_beta = self.transforms.clarke_transform(state[0], state[1], state[2])

            # Park Transformation
            i_d, i_q = self.transforms.park_transform(i_alpha, i_beta, state[4])

            i_d_sol.append(i_d)
            i_q_sol.append(i_q)

          
            i_d_ref_array.append(0)
            i_q_ref_array.append(current_reference[t_idx])
            # Current control for i_d and i_q using ref - value convention
            i_d_ref = 0
            V_d = self.current_controller_d.compute(i_d_ref - i_d, dt, V_d_delta_sat)  # d-axis current control
            V_q = self.current_controller_q.compute(current_reference[t_idx] - i_q, dt, V_q_delta_sat)  # q-axis current control


            # Inverse Park and Clarke Transform
            V_alpha, V_beta = self.transforms.inverse_park_transform(V_d, V_q, state[4])
            V_a, V_b, V_c = self.transforms.inverse_clarke_transform(V_alpha, V_beta)

            V_a_array.append(V_a)
            V_b_array.append(V_b)
            V_c_array.append(V_c)

            # Voltage limits
            V_a = np.clip(V_a, -self.motor.params.V_nominal, self.motor.params.V_nominal)
            V_b = np.clip(V_b, -self.motor.params.V_nominal, self.motor.params.V_nominal)
            V_c = np.clip(V_c, -self.motor.params.V_nominal, self.motor.params.V_nominal)

            V_a_sat_array.append(V_a)
            V_b_sat_array.append(V_b)
            V_c_sat_array.append(V_c)

            V_alpha_tmp, V_beta_tmp = self.transforms.clarke_transform(V_a, V_b, V_c)

            V_q_tmp, V_d_tmp = self.transforms.park_transform(V_alpha_tmp, V_beta_tmp, state[4])

            V_d_delta_sat = V_d_tmp - V_d
            V_q_delta_sat = V_q_tmp - V_q

            # Simulate motor dynamics
            sol = solve_ivp(self.motor.dynamics, [0, dt], initial_state, args=(V_a, V_b, V_c), t_eval=[dt])
            # Clip to maximum currents
            # sol.y[:3, -1] = np.clip(sol.y[:3, -1], -self.motor.params.I_nominal, self.motor.params.I_nominal)
            initial_state = sol.y[:, -1]

            omega_sol_rpm.append(self.rad_s_to_rpm(initial_state[3]))
            theta_sol.append(initial_state[4])

        return np.array(omega_sol_rpm), np.array(theta_sol), np.array(i_d_sol), np.array(i_q_sol), np.array(i_d_ref_array), np.array(i_q_ref_array), np.array(V_a_array), np.array(V_b_array), np.array(V_c_array), np.array(V_a_sat_array), np.array(V_b_sat_array), np.array(V_c_sat_array) 

    @staticmethod
    def rad_s_to_rpm(omega):
        return omega * (60 / (2 * np.pi))



if __name__ == "__main__":
    # Motor parameters (example values)
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

    # Simulation parameters
    dt = 0.0001
    t_span = np.arange(0, 20, dt)
    initial_state = [0.0, 0.0, 0.0, 0.0, 0.0]
    speed_reference_rpm = np.zeros(len(t_span))
    step_times = [1, 5, 15]
    step_values_rpm = [100, 250, 500]

    for i in range(len(step_times)):
        speed_reference_rpm[t_span >= step_times[i]] = step_values_rpm[i]

    # Initialize control system
    bldc_motor = BLDCMotor(params)
    control_system = BLDCControlSystem(bldc_motor, Kp_speed=0.5, Ki_speed=10, Kp_current=0.01, Ki_current=100, K_sat_speed=2)

    # Run simulation
    omega_sol_rpm, theta_sol, i_d_sol, i_q_sol, i_d_ref, i_q_ref, V_a_sol, V_b_sol, V_c_sol, V_a_sat, V_b_sat, V_c_sat, v_d, v_q = control_system.simulate_speed_loop(t_span, initial_state, speed_reference_rpm, dt)

    # Plot results
    plt.figure(figsize=(12, 7))

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

    plt.figure(figsize=(12, 7))

    plt.subplot(2, 1, 1)
    plt.plot(t_span, i_d_sol, label='i_d', color='blue')
    plt.plot(t_span, i_d_ref, label='i_d_ref', color='red', linestyle='--')
    plt.title('I_d')
    plt.xlabel('Time (s)')
    plt.ylabel('Current (A)')
    plt.legend()

    plt.subplot(2, 1, 2)
    plt.plot(t_span, i_q_sol, label='i_q', color='blue')
    plt.plot(t_span, i_q_ref, label='i_q_ref', color='red', linestyle='--')
    plt.title('I_q')
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

    plt.figure()
    plt.subplot(2, 1, 1)
    plt.plot(t_span, v_d, color='blue')
    plt.title('Direct')
    plt.xlabel('Time (s)')

    plt.subplot(2, 1, 2)
    plt.plot(t_span, v_q, color='blue')
    plt.title('Quadrature')
    plt.xlabel('Time (s)')

    plt.tight_layout()



    plt.show()
