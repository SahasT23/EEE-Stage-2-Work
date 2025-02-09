#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint

def dc_motor_dynamics(x, t, params):
    """
    ODEs for the DC motor:
      x[0] = i_a   (armature current)
      x[1] = omega (rotor speed, rad/s)
    params = [Va, Ra, La, K, J, B, TL]
    """
    i_a, omega = x
    Va, Ra, La, K, J, B, TL = params
    
    di_a_dt = (Va - Ra*i_a - K*omega) / La
    domega_dt = (K*i_a - B*omega - TL) / J
    return [di_a_dt, domega_dt]

def get_steady_state_speed(Va, Ra, La, K, J, B, TL, t_final=2.0):
    """
    Simulates the motor from t=0 to t=t_final for a given Va,
    and returns the *final* speed as the approximate steady-state.
    """
    # ODE parameters
    params = [Va, Ra, La, K, J, B, TL]
    # Initial conditions: [current=0, speed=0]
    x0 = [0.0, 0.0]
    # Time vector (2 seconds, 2000 points)
    t = np.linspace(0, t_final, 2000)
    
    # Solve ODE
    sol = odeint(dc_motor_dynamics, x0, t, args=(params,))
    i_a = sol[:, 0]
    omega = sol[:, 1]
    
    # We can take the speed at the last time step as "steady-state"
    # or average the last 50 points, to reduce any small fluctuations.
    # Let's just take the last value:
    final_speed = omega[-1]
    return final_speed

def main():
    # -- Motor Parameters --
    Ra = 2.0     # Resistance (Ohms)
    La = 0.5     # Inductance (H)
    K  = 0.1     # Torque/Back-emf constant
    J  = 0.01    # Rotor inertia (kg·m^2)
    B  = 0.001   # Viscous friction (N·m·s/rad)
    TL = 0.0     # Load torque (N·m)
    
    # We'll sweep Va from 0 to 50 V in steps
    Va_values = np.linspace(0, 50, 21)  # e.g. 21 points from 0, 2.5, 5, ... 50
    
    steady_speeds = []
    
    for Va in Va_values:
        final_speed = get_steady_state_speed(Va, Ra, La, K, J, B, TL, t_final=2.0)
        steady_speeds.append(final_speed)
    
    # Plot speed vs. Va
    plt.figure(figsize=(6,4))
    plt.plot(Va_values, steady_speeds, 'o-', label='Steady-State Speed')
    plt.grid(True)
    plt.xlabel("Armature Voltage Va (V)")
    plt.ylabel("Steady-State Speed (rad/s)")
    plt.title("Steady-State Speed vs. Armature Voltage")
    plt.legend()
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
