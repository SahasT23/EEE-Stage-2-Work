import numpy as np
import matplotlib.pyplot as plt

def pid_controller(setpoint, y_current, dt, 
                   K_p, K_i, K_d, 
                   integral_prev, error_prev):
    """
    This function computes the PID controller output given:
    - setpoint: The desired value of the output
    - y_current: The actual current value of the output (plant measurement)
    - dt: time step
    - Kp, Ki, Kd: PID gains
    - integral_prev: The accumulated integral value from the previous step
    - error_prev: The error from the previous step

    Returns:
      (u, integral_new, error_new)

    WHERE:
      - u is the new control signal (PID output)
      - integral_new is the updated integral term
      - error_new is the current error (which becomes error_prev for the next iteration)
    
    PID Explanation:
    ----------------------
    1) The 'error' is e(t) = setpoint - y_current.
    2) Proportional term P = Kp * e(t).
       - Magnifies the current error. A bigger Kp responds more aggressively.
    3) Integral term I = Integral of e(t) over time.
       - We approximate it numerically: I_new = I_old + e(t)*dt
         Then the controller contribution is Ki * I_new.
       - The integral term corrects long-term offset. If the output is below setpoint consistently,
         the integral grows and eventually adds enough control effort to drive error to zero.
    4) Derivative term D ~ derivative of e(t) with respect to time.
       - We approximate: D = (e(t) - e(t - dt)) / dt
         Then the controller contribution is Kd * D.
       - This term "predicts" error changes and helps dampen overshoot or oscillations.
    5) Total PID output: u = P + I + D
    """
    # 1) Calculate current error
    error_new = setpoint - y_current
    
    # 2) Integrate the error over time (for the I term)
    integral_new = integral_prev + error_new * dt
    
    # 3) Compute derivative of error (for the D term)
    derivative = (error_new - error_prev) / dt
    
    # 4) Compute PID output
    P = K_p * error_new
    I = K_i * integral_new
    D = K_d * derivative
    u = P + I + D
    
    return u, integral_new, error_new

def plant_update(y, u, a, b, dt):
    """
    This function simulates one time-step of a 1st-order plant:
       dy/dt = -a*y + b*u

    We use a simple Euler approximation (y_new = y_old + dy/dt * dt).

    - y is the current plant output
    - u is the control input
    - a, b are plant parameters
    - dt is the time step

    Returns:
      y_new (the updated plant output)
    """
    dy_dt = -a * y + b * u
    y_new = y + dy_dt * dt
    return y_new

def main():
    # --- Simulation parameters ---
    dt = 0.01         # time step (seconds)
    t_final = 5.0     # final time (seconds)
    n_steps = int(t_final / dt)
    time = np.linspace(0, t_final, n_steps)
    
    # --- Plant parameters for a 1st-order system: dy/dt = -a*y + b*u ---
    a = 1.0   # 'natural' decay rate of the system
    b = 2.0   # how strongly the input affects the output
    
    # --- PID Gains (adjust to see different behaviors) ---
    Kp = 2.0  # proportional gain
    Ki = 1.0  # integral gain
    Kd = 0.5  # derivative gain
    
    # --- Setpoint (desired output) ---
    setpoint = 5.0
    
    # -- Initial conditions for plant and PID states --
    y = 0.0               # plant output
    integral_prev = 0.0   # integral term initially 0
    error_prev = setpoint - y   # initial error
    
    # -- Data storage for plotting --
    y_vals = []
    u_vals = []
    
    # --- Main Simulation Loop ---
    for i in range(n_steps):
        # 1) PID control: compute the control signal 'u'
        u, integral_prev, error_prev = pid_controller(
            setpoint, y, dt, 
            Kp, Ki, Kd,
            integral_prev, error_prev
        )
        
        # 2) Update the plant using that control 'u'
        y = plant_update(y, u, a, b, dt)
        
        # 3) Store data for plotting
        y_vals.append(y)
        u_vals.append(u)
    
    # --- Plot results ---
    plt.figure(figsize=(10,5))
    
    # Top plot: the output 'y' vs. time (and the setpoint)
    plt.subplot(2,1,1)
    plt.plot(time, y_vals, 'b-', label='Plant Output (y)')
    plt.axhline(setpoint, color='r', linestyle='--', label='Setpoint')
    plt.xlabel('Time (s)')
    plt.ylabel('Output')
    plt.title('PID Control of a 1st-Order Plant')
    plt.legend()
    plt.grid(True)
    
    # Bottom plot: the controller output 'u' vs. time
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
