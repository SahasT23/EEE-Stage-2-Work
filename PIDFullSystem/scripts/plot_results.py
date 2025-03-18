import pandas as pd
import matplotlib.pyplot as plt

data = pd.read_csv('sim_data.csv')

plt.figure(figsize=(12,8))

# Position and Steering Plot
plt.subplot(3,1,1)
plt.plot(data['Time'], data['Position'], label='Buggy Position')
plt.axhline(TRACK_CENTER, color='r', linestyle='--', label='Track Center')
plt.ylabel('Position (mm)')
plt.title('Line Following Simulation')
plt.legend()

# Sensor Readings
plt.subplot(3,1,2)
plt.plot(data['Time'], data['L_Ind'], label='Left Sensor')
plt.plot(data['Time'], data['R_Ind'], label='Right Sensor')
plt.ylabel('Inductance (μH)')
plt.legend()

# Control Signals
plt.subplot(3,1,3)
plt.plot(data['Time'], data['PWM'], label='Motor PWM')
plt.plot(data['Time'], data['Steering'], label='Steering Angle')
plt.xlabel('Time (s)')
plt.ylabel('Control Signals')
plt.legend()

plt.tight_layout()
plt.savefig('simulation_results.png')
plt.show()