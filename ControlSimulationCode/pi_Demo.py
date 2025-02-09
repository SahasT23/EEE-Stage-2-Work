import numpy as np
import matplotlib.pyplot as plt

def pi_controller(setpoint, y_current, dt, Kp, Ki, integral_prev):
    """
    PI controller.

    Terms:
      error = setpoint - y_current
      P = Kp * error
      I_new = I_old + error * dt  (numerical integration)
      Controller output = P + Ki * I_new

    Returns:
      (u, integral_new)

    Where:
      u = PI control output
      integral_new = updated integral term
    """
    error = setpoint - y_current

    # Proportional term
    P = Kp * error

    # Integral term (accumulate error)
    integral_new = integral_prev + error * dt
    I = Ki * integral_new

    # Final control signal
    u = P + I

    return u, integral_new

def plant_update(y, u, a, b, dt):
    """
    One time-step of the 1st-order plant:
       dy/dt = -a*y + b*u

    - y: current output
    - u: control input
    - a, b: plant parameters
    - dt: time step

    Returns: y_new
    """
    dy_dt = -a * y + b * u
    y_new = y + dy_dt * dt
    return y_new

def main():
    # --- Simulation parameters ---
    dt = 0.01
    t_final = 5.0
    n_steps = int(t_final / dt)
    time = np.linspace(0, t_final, n_steps)

    # --- Plant parameters ---
    a = 1.0
    b = 2.0

    # --- PI gains ---
    Kp = 2.0
    Ki = 1.0

    # --- Setpoint ---
    setpoint = 5.0

    # --- Initial conditions ---
    y = 0.0  # plant output
    integral_prev = 0.0

    # --- Data storage ---
    y_vals = []
    u_vals = []

    # --- Main loop ---
    for i in range(n_steps):
        # 1) PI control
        u, integral_prev = pi_controller(setpoint, y, dt, Kp, Ki, integral_prev)

        # 2) Update the plant
        y = plant_update(y, u, a, b, dt)

        # 3) Store for plotting
        y_vals.append(y)
        u_vals.append(u)

    # --- Plot results ---
    plt.figure(figsize=(10,5))

    # Top plot: output vs setpoint
    plt.subplot(2,1,1)
    plt.plot(time, y_vals, 'b-', label='Plant Output (y)')
    plt.axhline(setpoint, color='r', linestyle='--', label='Setpoint')
    plt.title('PI Control')
    plt.xlabel('Time (s)')
    plt.ylabel('Output')
    plt.legend()
    plt.grid(True)

    # Bottom plot: control input
    plt.subplot(2,1,2)
    plt.plot(time, u_vals, 'g-', label='Control Input (u)')
    plt.xlabel('Time (s)')
    plt.ylabel('Controller Output')
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    main()
