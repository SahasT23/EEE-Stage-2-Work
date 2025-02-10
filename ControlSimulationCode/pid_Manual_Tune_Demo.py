import numpy as np
import matplotlib.pyplot as plt

def simulate_pid(kp, ki, kd, a, b, setpoint, y0=0.0, t_final=5.0, dt=0.01):
    """
    Simulates the plant:
       dy/dt = -a * y + b * u
    with a PID controller:
       u = P + I + D
       e(t) = setpoint - y(t).

    Returns:
      t       - time array
      y_array - system output array
      u_array - controller output array
    """

    n_steps = int(t_final / dt)
    t = np.linspace(0, t_final, n_steps)

    # Initial conditions
    y = y0
    integral = 0.0
    error_prev = setpoint - y

    y_array = []
    u_array = []

    for _ in range(n_steps):
        # Calculate error
        error = setpoint - y

        # PID terms:
        P = kp * error
        integral += error * dt
        I = ki * integral
        derivative = (error - error_prev) / dt
        D = kd * derivative

        # Controller output
        u = P + I + D

        # Update plant using Euler's method
        dy_dt = -a * y + b * u
        y = y + dy_dt * dt

        # Store data
        y_array.append(y)
        u_array.append(u)

        # For next iteration
        error_prev = error

    return t, np.array(y_array), np.array(u_array)

def main():
    """
    Demonstrates a manual PID tuning approach by running multiple simulations
    with different (Kp, Ki, Kd) sets to show how the response changes step by step.
    Now each run is plotted in its own figure.
    """

    # Plant parameters for 1st-order system
    a = 1.0  # decay rate
    b = 2.0  # input gain

    # Desired setpoint
    setpoint = 5.0

    # We'll do a few different runs to illustrate a typical manual tuning sequence.
    gains_run1 = (0.5, 0.0, 0.0)
    gains_run2 = (2.0, 0.0, 0.0)
    gains_run3 = (2.0, 1.0, 0.0)
    gains_run4 = (2.0, 1.0, 0.5)

    runs = [
        {"label": "Run1: Kp=0.5, Ki=0,   Kd=0",   "gains": gains_run1},
        {"label": "Run2: Kp=2.0, Ki=0,   Kd=0",   "gains": gains_run2},
        {"label": "Run3: Kp=2.0, Ki=1.0, Kd=0",   "gains": gains_run3},
        {"label": "Run4: Kp=2.0, Ki=1.0, Kd=0.5", "gains": gains_run4},
    ]

    t_final = 5.0
    dt = 0.01

    # Run each simulation and plot separately
    for run in runs:
        kp, ki, kd = run["gains"]
        label = run["label"]

        # Simulate
        t, y_vals, u_vals = simulate_pid(kp, ki, kd,
                                         a=a, b=b, setpoint=setpoint,
                                         y0=0.0, t_final=t_final, dt=dt)

        # Create a new figure for each run
        fig, axs = plt.subplots(2, 1, figsize=(8, 6))

        # Top subplot: system output
        axs[0].plot(t, y_vals, label='Output (y)')
        axs[0].axhline(setpoint, color='r', linestyle='--', label='Setpoint')
        axs[0].set_ylabel("Output (y)")
        axs[0].set_title(label)
        axs[0].legend()
        axs[0].grid(True)

        # Bottom subplot: controller output
        axs[1].plot(t, u_vals, label='Control (u)')
        axs[1].set_xlabel("Time (s)")
        axs[1].set_ylabel("Control Input (u)")
        axs[1].legend()
        axs[1].grid(True)

        # Adjust layout and display
        plt.tight_layout()
        plt.show()

if __name__ == '__main__':
    main()
