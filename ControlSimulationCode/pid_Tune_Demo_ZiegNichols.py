#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt

# ---------------------------------------------------------
# 1) Open-Loop Step Test
# ---------------------------------------------------------
def open_loop_step_test(a, b, U, y0=0.0, t_final=10.0, dt=0.01):
    """
    Applies a constant input u(t)=U to the 1st-order plant:
      dy/dt = -a*y + b*U
    with initial output y(0)=y0.
    Returns time t, output y(t).
    """
    n_steps = int(t_final / dt)
    t = np.linspace(0, t_final, n_steps)

    y = y0
    y_vals = []
    for _ in range(n_steps):
        dy_dt = -a*y + b*U
        y = y + dy_dt * dt
        y_vals.append(y)
    return t, np.array(y_vals)

# ---------------------------------------------------------
# 2) Find Inflection Point and Estimate L, T
# ---------------------------------------------------------
def find_L_T_from_step(t, y, U):
    """
    Given time, output arrays from an open-loop step (and the input step U),
    approximate the inflection point and compute:
      L = time-axis intercept of the tangent
      T = difference between that intercept and tangent crossing of y_ss
    Also compute process gain K_proc = (y_ss - y0) / U.

    Returns (L, T, K_proc).  If no valid inflection found, returns None, None, None.
    """
    y0 = y[0]
    y_ss = y[-1]
    if abs(U) < 1e-12:
        return None, None, None
    K_proc = (y_ss - y0) / U

    dt = t[1] - t[0]
    dydt = np.gradient(y, dt)  # numerical derivative
    idx_inflection = np.argmax(dydt)
    slope_inf = dydt[idx_inflection]

    # If slope <= 0, can't do S-curve
    if slope_inf <= 1e-12:
        return None, None, None

    t_inf = t[idx_inflection]
    y_inf = y[idx_inflection]

    # Tangent intercept with y=0
    t_intersect = t_inf - (y_inf / slope_inf)
    L = t_intersect

    # If L <= 0, fix it (avoid zero or negative 'delay')
    # We'll clamp it to a small epsilon
    if L <= 1e-12:
        L = 1e-5

    # T where tangent hits final value y_ss
    # y_ss = y_inf + slope_inf*(t_end - t_inf)
    # => t_end = t_inf + (y_ss - y_inf)/slope_inf
    t_end = t_inf + (y_ss - y_inf) / slope_inf
    T = t_end - L
    if T <= 1e-12:
        T = 1e-5  # also clamp

    return L, T, K_proc

# ---------------------------------------------------------
# 3) Ziegler–Nichols from L, T, K_proc
# ---------------------------------------------------------
def ziegler_nichols_openloop_pid(L, T, K_proc):
    """
    Classic PID from open-loop Z-N approach:
      Kp = 1.2 * (T / L) / K_proc
      Ti = 2*L -> Ki = Kp / Ti
      Td = 0.5*L -> Kd = Kp * Td
    """
    Kp = 1.2 * (T / L) / K_proc
    Ti = 2.0 * L
    Td = 0.5 * L

    Ki = Kp / Ti
    Kd = Kp * Td
    return Kp, Ki, Kd

# ---------------------------------------------------------
# 4) Simulate Closed-Loop with the Found PID Gains
# ---------------------------------------------------------
def simulate_pid(Kp, Ki, Kd, a, b, setpoint, 
                 y0=0.0, t_final=5.0, dt=0.01):
    """
    Closed-loop simulation:
       dy/dt = -a*y + b*u, 
    where u = PID(e) = Kp e + Ki ∫e dt + Kd de/dt,
    e(t)=setpoint - y(t).
    """
    n_steps = int(t_final / dt)
    t = np.linspace(0, t_final, n_steps)

    y = y0
    integral = 0.0
    error_prev = setpoint - y

    y_list = []
    u_list = []

    for _ in range(n_steps):
        error = setpoint - y
        P = Kp * error
        integral += error * dt
        I = Ki * integral
        derivative = (error - error_prev) / dt
        D = Kd * derivative
        u = P + I + D

        # Update plant
        dy_dt = -a*y + b*u
        y = y + dy_dt * dt

        y_list.append(y)
        u_list.append(u)
        error_prev = error

    return t, np.array(y_list), np.array(u_list)

# ---------------------------------------------------------
# MAIN DEMO
# ---------------------------------------------------------
def main():
    """
    1) Perform open-loop step test => get 'S' shaped response.
    2) Estimate L, T, and K_proc using the inflection point approach.
    3) Compute Z-N Classic PID gains, avoiding infinite or NaN by clamping small L.
    4) Simulate closed-loop with those PID gains.
    """
    # Plant: dy/dt = -a*y + b*u
    a = 1.0
    b = 2.0

    # Step input
    U_step = 1.0

    # (A) Open-loop step
    t_ol, y_ol = open_loop_step_test(a, b, U=U_step, y0=0.0, t_final=10.0, dt=0.01)

    # (B) Estimate L, T, K_proc
    L, T, K_proc = find_L_T_from_step(t_ol, y_ol, U=U_step)
    if (L is None) or (T is None) or (K_proc is None):
        print("Failed to get valid (L,T,K_proc). The system might be purely first-order with no delay.")
        return

    print(f"Open-Loop Step:\n  L={L:.6f}, T={T:.6f}, K_proc={K_proc:.6f}")

    # (C) Compute Z-N Gains
    Kp, Ki, Kd = ziegler_nichols_openloop_pid(L, T, K_proc)
    print(f"Z-N Gains:\n  Kp={Kp}, Ki={Ki}, Kd={Kd}")

    # (D) Closed-loop simulation
    setpoint = 5.0
    t_cl, y_cl, u_cl = simulate_pid(Kp, Ki, Kd, a, b, setpoint,
                                   y0=0.0, t_final=5.0, dt=0.01)

    # (E) Plot
    plt.figure(figsize=(10,6))

    # top: open-loop
    plt.subplot(2,1,1)
    plt.plot(t_ol, y_ol, 'b-', label='Open-Loop Step')
    plt.title("Open-Loop Step Test")
    plt.xlabel("Time (s)")
    plt.ylabel("Output (y)")
    plt.grid(True)
    plt.legend()

    # bottom: closed-loop
    plt.subplot(2,1,2)
    plt.plot(t_cl, y_cl, 'r-', label='Closed-Loop (Z-N PID)')
    plt.axhline(setpoint, color='k', linestyle='--', label='Setpoint')
    plt.title(f"Closed-Loop: Kp={Kp:.2f}, Ki={Ki:.2f}, Kd={Kd:.2f}")
    plt.xlabel("Time (s)")
    plt.ylabel("Output (y)")
    plt.grid(True)
    plt.legend()

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
