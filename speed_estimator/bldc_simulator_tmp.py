import numpy as np
import matplotlib.pyplot as plt


class BLDCMotor:
    def __init__(self, R, L, Kt, Ke, J, B):
        self.R = R  # Terminal resistance (Ohm)
        self.L = L  # Terminal inductance (H)
        self.Kt = Kt  # Torque constant (N·m/A)
        self.Ke = Ke  # Back EMF constant (V·s/rad)
        self.J = J  # Rotor inertia (kg·m²)
        self.B = B  # Viscous friction coefficient (N·m·s/rad)

        # Initial conditions
        self.i_a = 0.0  # Phase A current (A)
        self.i_b = 0.0  # Phase B current (A)
        self.i_c = 0.0  # Phase C current (A)
        self.omega = 0.0  # Angular speed (rad/s)
        self.theta = 0.0  # Rotor angle (rad)

    def update(self, V_a, V_b, V_c, dt):
        # Update the phase currents using Euler integration
        d_i_a = (V_a - self.R * self.i_a - self.Ke * self.omega) / self.L
        d_i_b = (V_b - self.R * self.i_b - self.Ke * self.omega) / self.L
        d_i_c = (V_c - self.R * self.i_c - self.Ke * self.omega) / self.L

        self.i_a += d_i_a * dt
        self.i_b += d_i_b * dt
        self.i_c += d_i_c * dt

        # Calculate total torque
        T = self.Kt * (self.i_a + self.i_b + self.i_c) / 3  # Average torque from all phases
        d_omega = (T - self.B * self.omega) / self.J
        self.omega += d_omega * dt

        # Update angle
        self.theta += self.omega * dt

    def generate_voltages(self, t):
        # Generate phase voltages for a rotating field
        frequency = 50  # Frequency of rotation in Hz
        phase_shift = 2 * np.pi / 3  # 120 degrees phase shift

        V_a = np.sin(2 * np.pi * frequency * t)
        V_b = np.sin(2 * np.pi * frequency * t + phase_shift)
        V_c = np.sin(2 * np.pi * frequency * t + 2 * phase_shift)

        return V_a, V_b, V_c


def simulate_motor(motor, duration=1.0, dt=0.01):
    time_steps = int(duration / dt)
    omega_values = []
    theta_values = []

    for step in range(time_steps):
        t = step * dt
        V_a, V_b, V_c = motor.generate_voltages(t)
        motor.update(V_a, V_b, V_c, dt)

        omega_values.append(motor.omega)
        theta_values.append(motor.theta)

    return omega_values, theta_values


# Parameters for the BLDC motor
R = 0.994
L = 0.995e-3
Kt = 91e-3
Ke = 1 / 10.9956
J = 44e-4
B = 0.528e-3
V_nominal = 24
I_nominal = 10

# Create a motor instance
motor = BLDCMotor(R, L, Kt, Ke, J, B)

# Simulate the motor
omega_values, theta_values = simulate_motor(motor, duration=1.0, dt=0.01)

# Plot the results
time = np.arange(0, 1.0, 0.01)
plt.figure(figsize=(12, 6))
plt.subplot(2, 1, 1)
plt.plot(time, omega_values)
plt.title('Angular Speed of BLDC Motor')
plt.xlabel('Time (s)')
plt.ylabel('Angular Speed (rad/s)')

plt.subplot(2, 1, 2)
plt.plot(time, theta_values)
plt.title('Rotor Angle of BLDC Motor')
plt.xlabel('Time (s)')
plt.ylabel('Angle (rad)')

plt.tight_layout()
plt.show()
