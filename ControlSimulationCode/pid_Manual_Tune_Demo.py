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
        #  Proportional
        P = kp * error

        #  Integral (simple numerical accumulation)
        integral += error * dt
        I = ki * integral

        #  Derivative (discrete approximation)
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

        # For next loop
        error_prev = error

    return t, np.array(y_array), np.array(u_array)

def main():
    """
    Demonstrates a manual PID tuning approach by running multiple simulations
    with different (Kp, Ki, Kd) sets to show how the response changes step by step.
    """

    # Plant parameters for 1st-order system
    a = 1.0  # decay rate
    b = 2.0  # input gain

    # Desired setpoint
    setpoint = 5.0

    # We'll do a few different runs to illustrate a typical manual tuning sequence.

    # 1) Very small Kp, no I, no D (system likely responds slowly)
    gains_run1 = (0.5, 0.0, 0.0)
    
    # 2) Bigger Kp, no I, no D (system responds faster, might start overshooting)
    gains_run2 = (2.0, 0.0, 0.0)

    # 3) Add some Integral action to remove steady-state error
    gains_run3 = (2.0, 1.0, 0.0)

    # 4) Add a small Derivative to help damp oscillations
    gains_run4 = (2.0, 1.0, 0.5)

    # We'll store all these runs in a list to loop through
    runs = [
        {"label": "Run1: Kp=0.5, Ki=0,   Kd=0",   "gains": gains_run1},
        {"label": "Run2: Kp=2.0, Ki=0,   Kd=0",   "gains": gains_run2},
        {"label": "Run3: Kp=2.0, Ki=1.0, Kd=0",   "gains": gains_run3},
        {"label": "Run4: Kp=2.0, Ki=1.0, Kd=0.5", "gains": gains_run4},
    ]

    # We'll simulate each run for the same time window
    t_final = 5.0
    dt = 0.01

    plt.figure(figsize=(10, 6))
    for i, run in enumerate(runs, start=1):
        # Unpack the gains
        kp, ki, kd = run["gains"]
        label = run["label"]

        # Simulate
        t, y_vals, u_vals = simulate_pid(kp, ki, kd,
                                         a=a, b=b, setpoint=setpoint,
                                         y0=0.0, t_final=t_final, dt=dt)

        # Plot the output (y) for each run on the same axes
        plt.subplot(2, 1, 1)
        plt.plot(t, y_vals, label=label)

        # Plot the control input (u) on the second axes
        plt.subplot(2, 1, 2)
        plt.plot(t, u_vals, label=label)

    # Format the output plot
    plt.subplot(2, 1, 1)
    plt.axhline(setpoint, color='r', linestyle='--', label='Setpoint')
    plt.ylabel("Output (y)")
    plt.title("Manual PID Tuning Example (First-Order System)")
    plt.legend()
    plt.grid(True)

    # Format the control plot
    plt.subplot(2, 1, 2)
    plt.xlabel("Time (s)")
    plt.ylabel("Control Input (u)")
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    main()
