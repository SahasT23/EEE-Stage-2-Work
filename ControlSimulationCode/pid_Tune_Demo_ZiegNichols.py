import numpy as np
import matplotlib.pyplot as plt

# -------------------------------------------------------------------------
# (1) Plant + Controller Simulation
# -------------------------------------------------------------------------
def simulate_plant(Kp, Ki, Kd, a, b, setpoint, 
                   y0=0.0, t_final=5.0, dt=0.01):
    """
    Simulates the 1st-order plant:
       dy/dt = -a*y + b*u
    under a PID controller:
       u = P + I + D
       where e(t) = setpoint - y(t).

    For the "auto-tune" phase, we often set Ki=0, Kd=0 (P-only).
    
    Parameters:
      - Kp, Ki, Kd: Controller gains (float).
      - a, b: Plant parameters (float).
      - setpoint: Desired output (float).
      - y0: Initial output of the plant (float).
      - t_final: Simulation end time (float).
      - dt: Timestep (float).

    Returns:
      t: time array
      y_list: plant output array
      u_list: control input array
      e_list: error array
    """

    # Number of steps
    n_steps = int(t_final / dt)
    t = np.linspace(0, t_final, n_steps)

    # Initialize states
    y = y0
    integral = 0.0
    error_prev = setpoint - y

    y_list = []
    u_list = []
    e_list = []

    for i in range(n_steps):
        # Time loop

        # 1) Compute current error
        error = setpoint - y

        # 2) PID terms
        P = Kp * error

        # Integrator (numerical)
        integral += error * dt
        I = Ki * integral

        # Derivative (approx) 
        derivative = (error - error_prev) / dt
        D = Kd * derivative

        # 3) Control signal
        u = P + I + D

        # 4) Update plant: dy/dt = -a*y + b*u
        dy_dt = -a * y + b * u
        y = y + dy_dt * dt

        # 5) Save data
        y_list.append(y)
        u_list.append(u)
        e_list.append(error)

        # 6) For next iteration
        error_prev = error

    return t, np.array(y_list), np.array(u_list), np.array(e_list)


# -------------------------------------------------------------------------
# (2) Automatic Tuning: Find K_u and T_u
# -------------------------------------------------------------------------
def find_ultimate_gain_and_period(a, b, setpoint, 
                                  y0=0.0, dt=0.01, 
                                  kp_min=0.0, kp_max=20.0, kp_steps=50):
    """
    Sweeps Kp from kp_min to kp_max, performing P-only control on the plant,
    and checks for sustained oscillations in the final segment of each run.

    This is a naive approach:
      - For each Kp, run the sim for a fixed time (e.g. 5 seconds).
      - Check the output's last N samples for oscillations (peak-to-peak amplitude).
      - If amplitude is large but not diverging, we guess it's near the ultimate gain.
      - Then measure approximate oscillation period T_u by zero-crossing or peak detection.

    Returns:
      Ku: The "ultimate" gain that gave the largest stable oscillation amplitude.
      Tu: The approximate period of that oscillation.

    NOTE: This method is purely illustrative. Real auto-tuners often use
    relay feedback or more robust detection. 
    """

    kp_values = np.linspace(kp_min, kp_max, kp_steps)
    
    best_kp = None
    best_amp = 0.0
    best_period = None

    for Kp in kp_values:
        # Simulate with P-only control
        t, y_vals, _, e_vals = simulate_plant(Kp=Kp, Ki=0, Kd=0, 
                                              a=a, b=b, setpoint=setpoint, 
                                              y0=y0, t_final=5.0, dt=dt)

        # Let's analyze the last 1 second of simulation for amplitude
        # We'll just do a rough measure: peak-to-peak amplitude of (y - setpoint)
        # If it's nearly 0, it's stable (no oscillation). If it's huge, might be diverging.
        # We'll store the largest stable amplitude.

        last_idx = int(1.0 / dt)  # last 1 second
        segment = (y_vals - setpoint)[-last_idx:]  # y - setpoint, last part
        ptp = segment.max() - segment.min()  # peak-to-peak

        # Also let's check if the system is obviously diverging
        # e.g. if the absolute value is > some threshold 
        # We'll define a "huge" threshold as 100 for demonstration
        if np.any(np.abs(segment) > 100):
            # It's blowing up or saturating, skip
            continue

        # We'll treat a larger amplitude as "closer to ultimate"
        if ptp > best_amp:
            best_amp = ptp
            best_kp = Kp

            # Approximate period from zero-crossings or naive method
            # We'll do a simple approach: find time between consecutive sign changes
            # in the last segment of y - setpoint
            sign_changes = []
            seg_time = t[-last_idx:]
            s_prev = np.sign(segment[0])
            for i in range(1, len(segment)):
                s_now = np.sign(segment[i])
                if s_now != s_prev and s_now != 0:
                    # sign changed
                    sign_changes.append(seg_time[i])
                s_prev = s_now

            # If we have at least 2 sign changes, we can measure half-period roughly
            # i.e. consecutive sign changes ~ half wave in a sinusoid
            if len(sign_changes) >= 2:
                # measure average delta
                deltas = []
                for i in range(1, len(sign_changes)):
                    deltas.append(sign_changes[i] - sign_changes[i-1])
                half_period_est = np.mean(deltas) if len(deltas) > 0 else 0
                # full period ~ 2 * half_period_est
                best_period = 2 * half_period_est
            else:
                best_period = None

    # Return the best result
    return best_kp, best_period


# -------------------------------------------------------------------------
# (3) Ziegler–Nichols Formulas
# -------------------------------------------------------------------------
def ziegler_nichols_classic_pid(Ku, Tu):
    """
    Ziegler–Nichols Classic PID Tuning rule:
      Kp = 0.6 Ku
      Ki = 2*Kp / Tu  (i.e. Ti = 0.5 Tu, so Ki = Kp / Ti)
      Kd = 0.125 Kp Tu
    """
    if Ku is None or Tu is None or Tu <= 0:
        return (0, 0, 0)  # fallback

    Kp = 0.6 * Ku
    Ki = (2.0 * Kp) / Tu
    Kd = 0.125 * Kp * Tu
    return (Kp, Ki, Kd)


# -------------------------------------------------------------------------
# MAIN DEMO
# -------------------------------------------------------------------------
def main():
    # Plant parameters for: dy/dt = -a*y + b*u
    a = 1.0
    b = 2.0

    # Setpoint for the plant
    setpoint = 5.0

    # 1) Find ultimate gain (Ku) and period (Tu) by sweeping Kp
    Ku, Tu = find_ultimate_gain_and_period(a=a, b=b, setpoint=setpoint)
    print(f"Estimated Ku (Ultimate Gain) = {Ku:.3f}")
    print(f"Estimated Tu (Ultimate Period) = {Tu:.3f}")

    if Ku is None or Tu is None:
        print("Could not find sustained oscillations with given search range.")
        print("Try increasing kp_max or adjusting approach.")
        return

    # 2) Use Ziegler–Nichols formula for "Classic PID"
    Kp, Ki, Kd = ziegler_nichols_classic_pid(Ku, Tu)
    print(f"Ziegler–Nichols PID Gains:\n  Kp={Kp:.3f}, Ki={Ki:.3f}, Kd={Kd:.3f}")

    # 3) Simulate with these PID gains
    t, y_vals, u_vals, e_vals = simulate_plant(Kp=Kp, Ki=Ki, Kd=Kd, 
                                               a=a, b=b, setpoint=setpoint, 
                                               y0=0.0, t_final=5.0, dt=0.01)
    # 4) Plot the results
    plt.figure(figsize=(10,5))

    # Output vs. setpoint
    plt.subplot(2,1,1)
    plt.plot(t, y_vals, label='Plant Output (y)')
    plt.axhline(setpoint, color='r', linestyle='--', label='Setpoint')
    plt.title('PID Control with Auto-Tuned Gains (Ziegler-Nichols)')
    plt.xlabel('Time (s)')
    plt.ylabel('Output (y)')
    plt.legend()
    plt.grid(True)

    # Control input
    plt.subplot(2,1,2)
    plt.plot(t, u_vals, 'g-', label='Controller Output (u)')
    plt.xlabel('Time (s)')
    plt.ylabel('Control Signal')
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.show()


# -------------------------------------------------------------------------
# Run the demo
# -------------------------------------------------------------------------
if __name__ == '__main__':
    main()
