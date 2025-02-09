import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.lines as mlines
from scipy.integrate import odeint

def dc_motor_dynamics(x, t, params):
    """
    ODEs for a separately excited DC motor (assuming constant field current).

    State vector x:
      x[0] = i_a   (armature current, A)
      x[1] = omega (rotor angular speed, rad/s)

    params = [Va, Ra, La, K, J, B, TL]
      Va = armature voltage [V]
      Ra = armature resistance [Ohms]
      La = armature inductance [H]
      K  = torque/back-emf constant [N·m/A] & [V·s/rad]
      J  = rotor inertia [kg·m^2]
      B  = viscous friction [N·m·s/rad]
      TL = load torque [N·m]
    """
    i_a, omega = x
    Va, Ra, La, K, J, B, TL = params

    # Electrical (armature) equation
    di_a_dt = (Va - Ra*i_a - K*omega) / La

    # Mechanical (rotor) equation
    domega_dt = (K*i_a - B*omega - TL) / J

    return [di_a_dt, domega_dt]

def simulate_dc_motor():
    """
    Runs a numeric simulation of the DC motor equations (zero load)
    and then plots the armature current and speed.
    """
    # --- Example motor parameters --- Can change these values 
    Va = 24.0    # Armature voltage
    Ra = 2.0     # Armature resistance
    La = 0.5     # Armature inductance
    K  = 0.1     # Torque/back-emf constant
    J  = 0.01    # Rotor inertia
    B  = 0.001   # Viscous friction
    TL = 0.0     # Zero load torque

    params = [Va, Ra, La, K, J, B, TL]

    # Initial conditions: current=0, speed=0
    x0 = [0.0, 0.0]

    # Time vector
    t = np.linspace(0, 2, 2000)

    # Solve ODE
    sol = odeint(dc_motor_dynamics, x0, t, args=(params,))
    i_a = sol[:, 0]
    omega = sol[:, 1]

    # --- Plot results ---
    plt.figure(figsize=(8,5))
    # Current
    plt.subplot(2,1,1)
    plt.plot(t, i_a, 'b')
    plt.grid(True)
    plt.ylabel("Current (A)")
    plt.title("DC Motor Response")

    # Speed
    plt.subplot(2,1,2)
    plt.plot(t, omega, 'r')
    plt.grid(True)
    plt.xlabel("Time (s)")
    plt.ylabel("Speed (rad/s)")

    plt.tight_layout()
    plt.show()

def draw_simulink_schematic():
    """
    Draws a Python approximation of the Simulink DC motor schematic:
      - DC Machine block with A+, A-, F+, F-, T_L, speed output
      - Armature DC source block
      - Field DC source block
      - '0' (Constant) block for load torque
      - Gain and Scope blocks for speed measurement
      - powergui (discrete) block

    Everything is placed to look somewhat like the original Simulink layout.
    """
    fig, ax = plt.subplots(figsize=(10,6))
    ax.set_xlim(0, 12)
    ax.set_ylim(0, 7)
    ax.set_aspect('equal', adjustable='box')
    ax.axis('off')

    # --- DC MACHINE BLOCK ---
    motor_x, motor_y = 5, 3
    motor_w, motor_h = 2, 2
    motor_rect = patches.Rectangle((motor_x, motor_y), motor_w, motor_h,
                                   fill=False, ec='black', lw=1)
    ax.add_patch(motor_rect)
    ax.text(motor_x + motor_w/2, motor_y + motor_h/2,
            "DC\nMachine", ha='center', va='center', fontsize=10)

    # Label terminals: A+, A-, F+, F-, T_L, speed
    # A+ near left top
    ax.text(motor_x - 0.3, motor_y + 0.65*motor_h, "A+", ha='right', va='center', fontsize=8)
    # A- near left bottom
    ax.text(motor_x - 0.3, motor_y + 0.35*motor_h, "A-", ha='right', va='center', fontsize=8)
    # F+ near top of motor
    ax.text(motor_x + 0.3*motor_w, motor_y + motor_h + 0.2, "F+", ha='center', va='bottom', fontsize=8)
    # F- near top
    ax.text(motor_x + 0.7*motor_w, motor_y + motor_h + 0.2, "F-", ha='center', va='bottom', fontsize=8)
    # T_L at bottom
    ax.text(motor_x + motor_w/2, motor_y - 0.3, "T_L", ha='center', va='top', fontsize=8)
    # Speed on right
    ax.text(motor_x + motor_w + 0.2, motor_y + motor_h/2, "Speed\n(rad/s)",
            ha='left', va='center', fontsize=8)

    # --- ARMATURE DC SOURCE (left) ---
    arm_src_x, arm_src_y = 1, 3.3
    arm_src_w, arm_src_h = 1, 0.8
    arm_rect = patches.Rectangle((arm_src_x, arm_src_y), arm_src_w, arm_src_h,
                                 fill=False, ec='black', lw=1)
    ax.add_patch(arm_rect)
    ax.text(arm_src_x + arm_src_w/2, arm_src_y + arm_src_h/2,
            "DC Source\n(Arm.)", ha='center', va='center', fontsize=8)

    # Wires from arm source to motor A+ / A-
    lineAplus = mlines.Line2D([arm_src_x + arm_src_w, motor_x],
                              [arm_src_y + 0.6*arm_src_h, motor_y + 0.65*motor_h],
                              color='black')
    ax.add_line(lineAplus)
    lineAminus = mlines.Line2D([arm_src_x + arm_src_w, motor_x],
                               [arm_src_y + 0.3*arm_src_h, motor_y + 0.35*motor_h],
                               color='black')
    ax.add_line(lineAminus)

    # --- FIELD DC SOURCE (above motor) ---
    field_src_x, field_src_y = 5, 6
    field_src_w, field_src_h = 1, 0.8
    field_rect = patches.Rectangle((field_src_x, field_src_y), field_src_w, field_src_h,
                                   fill=False, ec='black', lw=1)
    ax.add_patch(field_rect)
    ax.text(field_src_x + field_src_w/2, field_src_y + field_src_h/2,
            "DC Source\n(Field)", ha='center', va='center', fontsize=8)

    # Lines from motor top to field block
    lineFplus = mlines.Line2D([motor_x + 0.3*motor_w, field_src_x + 0.3*field_src_w],
                              [motor_y + motor_h, field_src_y],
                              color='black')
    ax.add_line(lineFplus)
    lineFminus = mlines.Line2D([motor_x + 0.7*motor_w, field_src_x + 0.7*field_src_w],
                               [motor_y + motor_h, field_src_y],
                               color='black')
    ax.add_line(lineFminus)

    # --- CONSTANT "0" BLOCK FOR T_L ---
    zero_x, zero_y = 5.5, 1.3
    zero_w, zero_h = 0.6, 0.6
    zero_rect = patches.Rectangle((zero_x, zero_y), zero_w, zero_h,
                                  fill=False, ec='black', lw=1)
    ax.add_patch(zero_rect)
    ax.text(zero_x + zero_w/2, zero_y + zero_h/2, "0", ha='center', va='center', fontsize=10)

    # Wire from "0" block to T_L of motor
    lineTL = mlines.Line2D([zero_x + zero_w/2, motor_x + motor_w/2],
                           [zero_y + zero_h, motor_y],
                           color='black')
    ax.add_line(lineTL)

    # --- SPEED OUTPUT TO GAIN TO SCOPE ---
    # Gain block
    gain_x, gain_y = motor_x + motor_w + 1, motor_y + 0.4*motor_h
    gain_w, gain_h = 0.8, 0.8
    gain_rect = patches.Rectangle((gain_x, gain_y), gain_w, gain_h,
                                  fill=False, ec='black', lw=1)
    ax.add_patch(gain_rect)
    ax.text(gain_x + gain_w/2, gain_y + gain_h/2, "Gain", ha='center', va='center', fontsize=8)

    # Line from motor to Gain
    motor_to_gain = mlines.Line2D([motor_x + motor_w, gain_x],
                                  [motor_y + motor_h/2, gain_y + gain_h/2],
                                  color='black')
    ax.add_line(motor_to_gain)

    # Scope block
    scope_x, scope_y = gain_x + gain_w + 1, gain_y
    scope_w, scope_h = 1.0, 0.8
    scope_rect = patches.Rectangle((scope_x, scope_y), scope_w, scope_h,
                                   fill=False, ec='black', lw=1)
    ax.add_patch(scope_rect)
    ax.text(scope_x + scope_w/2, scope_y + scope_h/2, "Scope",
            ha='center', va='center', fontsize=8)

    # Line from Gain to Scope
    gain_to_scope = mlines.Line2D([gain_x + gain_w, scope_x],
                                  [gain_y + gain_h/2, scope_y + scope_h/2],
                                  color='black')
    ax.add_line(gain_to_scope)

    # --- POWERGUI (DISCRETE) BLOCK ---
    pg_x, pg_y = 9, 5.5
    pg_w, pg_h = 1.5, 0.8
    pg_rect = patches.Rectangle((pg_x, pg_y), pg_w, pg_h,
                                fill=False, ec='black', lw=1)
    ax.add_patch(pg_rect)
    ax.text(pg_x + pg_w/2, pg_y + pg_h/2,
            "powergui\n(discrete)", ha='center', va='center', fontsize=8)

    plt.title("DC Motor Model (Drawn in Python)", fontsize=12)
    plt.show()

def main():
    # 1) Draw the approximate schematic
    draw_simulink_schematic()

    # 2) Simulate the DC motor and plot results
    simulate_dc_motor()

if __name__ == "__main__":
    main()
