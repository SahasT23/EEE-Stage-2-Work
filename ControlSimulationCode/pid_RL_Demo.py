#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import random

def simulate_pid(kp, ki, kd, a, b, setpoint,
                 t_final=5.0, dt=0.01,
                 blow_up_threshold=1e3,
                 u_sat=100.0):
    """
    Simulates the 1st-order plant:
       dy/dt = -a*y + b*u
    under a PID controller:
       u = kp*error + ki*integral(error) + kd*d(error)/dt

    SAFEGUARDS:
      - If |y| or |u| exceed 'blow_up_threshold', we abort and return a large cost.
      - We saturate 'u' within +/- u_sat, emulating actuator limits.

    Returns:
      cost = integral of (error^2 + 0.01 * u^2) dt over the run
    """
    n_steps = int(t_final / dt)
    y = 0.0
    integral = 0.0
    error_prev = setpoint - y
    cost = 0.0

    for _ in range(n_steps):
        error = setpoint - y

        # PID terms
        P = kp * error
        integral += error * dt
        I = ki * integral
        derivative = (error - error_prev) / dt
        D = kd * derivative

        u = P + I + D

        # (A) Saturate control to emulate actuator limits
        if u > u_sat:
            u = u_sat
        elif u < -u_sat:
            u = -u_sat

        # Update the plant
        dy_dt = -a * y + b * u
        y = y + dy_dt * dt

        # (B) Check for blow-up
        if abs(y) > blow_up_threshold:
            # Assign a large cost for this episode, end early
            return 1e9

        # Accumulate cost: error^2 + small penalty on control^2
        cost += (error**2 + 0.01 * u**2) * dt

        error_prev = error

    return cost

def reinforcement_learning_pid(a, b, setpoint,
                               episodes=50,
                               gain_min=0.0, gain_max=2.0):
    """
    A toy 'RL' approach that searches for PID gains in [gain_min, gain_max].
    We do random sampling of (kp, ki, kd) each episode.

    We keep the best (lowest) cost found so far.
    """
    best_kp, best_ki, best_kd = 0.0, 0.0, 0.0
    best_cost = float('inf')

    for ep in range(episodes):
        # Randomly pick gains
        kp = random.uniform(gain_min, gain_max)
        ki = random.uniform(gain_min, gain_max)
        kd = random.uniform(gain_min, gain_max)

        # Simulate and get cost
        cost = simulate_pid(kp, ki, kd,
                            a=a, b=b, setpoint=setpoint,
                            t_final=5.0, dt=0.01)

        # If cost is better, update
        if cost < best_cost:
            best_cost = cost
            best_kp, best_ki, best_kd = kp, ki, kd
            print(f"Episode {ep}: better gains -> Kp={kp:.3f}, Ki={ki:.3f}, Kd={kd:.3f}, cost={cost:.4f}")

    return best_kp, best_ki, best_kd, best_cost

def main():
    # Plant parameters
    a = 1.0
    b = 2.0
    setpoint = 5.0

    # 1) "Train" / "Tune" the PID with random search RL
    kp_best, ki_best, kd_best, cost_best = reinforcement_learning_pid(
        a=a, b=b, setpoint=setpoint,
        episodes=50,    # number of random attempts
        gain_min=0.0,
        gain_max=2.0    # narrower range, prevents huge gains
    )

    print("\n==== BEST GAINS FOUND ====")
    print(f"Kp={kp_best:.3f}, Ki={ki_best:.3f}, Kd={kd_best:.3f}, cost={cost_best:.4f}\n")

    # 2) Run final simulation with these best gains
    dt = 0.01
    t_final = 5.0
    n_steps = int(t_final / dt)
    time = np.linspace(0, t_final, n_steps)

    y = 0.0
    integral = 0.0
    error_prev = setpoint - y

    y_vals = []
    u_vals = []

    for _ in range(n_steps):
        error = setpoint - y

        P = kp_best * error
        integral += error * dt
        I = ki_best * integral
        derivative = (error - error_prev) / dt
        D = kd_best * derivative

        u = P + I + D

        # Saturate control
        if u > 100.0:
            u = 100.0
        elif u < -100.0:
            u = -100.0

        # Update plant
        dy_dt = -a * y + b * u
        y = y + dy_dt * dt

        y_vals.append(y)
        u_vals.append(u)
        error_prev = error

    # 3) Plot final results
    plt.figure(figsize=(10,4))

    plt.subplot(2,1,1)
    plt.plot(time, y_vals, label='Plant Output (y)')
    plt.axhline(setpoint, color='r', linestyle='--', label='Setpoint')
    plt.title("RL-Tuned PID (with Safeguards)")
    plt.ylabel("Output")
    plt.legend()
    plt.grid(True)

    plt.subplot(2,1,2)
    plt.plot(time, u_vals, 'g', label='Control Input (u)')
    plt.xlabel("Time (s)")
    plt.ylabel("Controller Output")
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
