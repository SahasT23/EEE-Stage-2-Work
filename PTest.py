import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button

class MotorSimulation:
    def __init__(self):
        # Default parameters
        self.Kp = 10.0           # Proportional gain
        self.reference = 100.0   # Desired speed/position (target)
        self.simulation_time = 5  # Total simulation time in seconds
        self.time_step = 0.01     # Time step for simulation
        self.motor_inertia = 0.01 # Motor inertia (resistance to changes in motion)
        self.friction = 0.5       # Friction coefficient
        
        # Initial conditions
        self.initial_position = 0.0
        self.initial_velocity = 0.0
        
        # Create the simulation figure
        self.create_simulation()
    
    def simulate_system(self, Kp, reference):
        """Simulate a simple DC motor with P control"""
        # Time array
        time = np.arange(0, self.simulation_time, self.time_step)
        
        # Arrays to store position, velocity, and control signal
        position = np.zeros_like(time)
        velocity = np.zeros_like(time)
        control_signal = np.zeros_like(time)
        error = np.zeros_like(time)
        
        # Set initial conditions
        position[0] = self.initial_position
        velocity[0] = self.initial_velocity
        
        # Simulation loop
        for i in range(1, len(time)):
            # Calculate error
            error[i-1] = reference - position[i-1]
            
            # Calculate control signal (P controller)
            control_signal[i-1] = Kp * error[i-1]
            
            # Limit control signal (motor saturation)
            control_signal[i-1] = np.clip(control_signal[i-1], -100, 100)
            
            # Calculate acceleration (simplified motor model)
            acceleration = (control_signal[i-1] - self.friction * velocity[i-1]) / self.motor_inertia
            
            # Update velocity using Euler integration
            velocity[i] = velocity[i-1] + acceleration * self.time_step
            
            # Update position using Euler integration
            position[i] = position[i-1] + velocity[i] * self.time_step
        
        # Calculate final error
        error[-1] = reference - position[-1]
        control_signal[-1] = Kp * error[-1]
        
        return time, position, velocity, control_signal, error
    
    def create_simulation(self):
        """Create interactive simulation plot"""
        # Initial simulation
        time, position, velocity, control_signal, error = self.simulate_system(self.Kp, self.reference)
        
        # Create figure and subplots
        self.fig, self.axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
        self.fig.subplots_adjust(bottom=0.3)  # Make room for sliders
        
        # Plot position
        self.pos_line, = self.axs[0].plot(time, position, 'b-', label='Position')
        self.ref_line, = self.axs[0].plot(time, np.ones_like(time) * self.reference, 'r--', label='Reference')
        self.axs[0].set_ylabel('Position')
        self.axs[0].legend(loc='upper right')
        self.axs[0].grid(True)
        
        # Plot control signal
        self.ctrl_line, = self.axs[1].plot(time, control_signal, 'g-', label='Control Signal')
        self.axs[1].set_ylabel('Control Signal')
        self.axs[1].legend(loc='upper right')
        self.axs[1].grid(True)
        
        # Plot error
        self.error_line, = self.axs[2].plot(time, error, 'm-', label='Error')
        self.axs[2].set_xlabel('Time (s)')
        self.axs[2].set_ylabel('Error')
        self.axs[2].legend(loc='upper right')
        self.axs[2].grid(True)
        
        # Add sliders for parameters
        ax_Kp = plt.axes([0.25, 0.20, 0.65, 0.03])
        ax_ref = plt.axes([0.25, 0.15, 0.65, 0.03])
        ax_inertia = plt.axes([0.25, 0.10, 0.65, 0.03])
        ax_friction = plt.axes([0.25, 0.05, 0.65, 0.03])
        
        # Create sliders
        self.slider_Kp = Slider(ax_Kp, 'Kp', 0.1, 50.0, valinit=self.Kp)
        self.slider_ref = Slider(ax_ref, 'Reference', 0.0, 200.0, valinit=self.reference)
        self.slider_inertia = Slider(ax_inertia, 'Motor Inertia', 0.001, 0.1, valinit=self.motor_inertia)
        self.slider_friction = Slider(ax_friction, 'Friction', 0.0, 2.0, valinit=self.friction)
        
        # Add reset button
        ax_reset = plt.axes([0.8, 0.25, 0.1, 0.04])
        self.button_reset = Button(ax_reset, 'Reset')
        
        # Connect callbacks
        self.slider_Kp.on_changed(self.update)
        self.slider_ref.on_changed(self.update)
        self.slider_inertia.on_changed(self.update)
        self.slider_friction.on_changed(self.update)
        self.button_reset.on_clicked(self.reset)
        
        # Add title
        self.fig.suptitle('P Controller Simulation', fontsize=16)
        
        # Show plot
        plt.tight_layout(rect=[0, 0.3, 1, 0.95])
    
    def update(self, val=None):
        """Update the plot when sliders are moved"""
        # Get current parameter values from sliders
        self.Kp = self.slider_Kp.val
        self.reference = self.slider_ref.val
        self.motor_inertia = self.slider_inertia.val
        self.friction = self.slider_friction.val
        
        # Run simulation with new parameters
        time, position, velocity, control_signal, error = self.simulate_system(self.Kp, self.reference)
        
        # Update plots
        self.pos_line.set_ydata(position)
        self.ref_line.set_ydata(np.ones_like(time) * self.reference)
        self.ctrl_line.set_ydata(control_signal)
        self.error_line.set_ydata(error)
        
        # Adjust y-axis limits as needed
        self.axs[0].relim()
        self.axs[0].autoscale_view()
        self.axs[1].relim()
        self.axs[1].autoscale_view()
        self.axs[2].relim()
        self.axs[2].autoscale_view()
        
        # Redraw the figure
        self.fig.canvas.draw_idle()
    
    def reset(self, event):
        """Reset sliders to initial values"""
        self.slider_Kp.reset()
        self.slider_ref.reset()
        self.slider_inertia.reset()
        self.slider_friction.reset()
    
    def show(self):
        """Show the interactive plot"""
        plt.show()

# Create a command line interface to run the simulation
if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="DC Motor P Controller Simulation")
    parser.add_argument('--Kp', type=float, default=10.0, help='Proportional gain')
    parser.add_argument('--reference', type=float, default=100.0, help='Reference position/speed')
    parser.add_argument('--inertia', type=float, default=0.01, help='Motor inertia')
    parser.add_argument('--friction', type=float, default=0.5, help='Friction coefficient')
    
    args = parser.parse_args()
    
    # Create simulation
    sim = MotorSimulation()
    
    # Set parameters from command line arguments
    sim.Kp = args.Kp
    sim.reference = args.reference
    sim.motor_inertia = args.inertia
    sim.friction = args.friction
    
    # Update sliders to match command line arguments
    sim.slider_Kp.set_val(sim.Kp)
    sim.slider_ref.set_val(sim.reference)
    sim.slider_inertia.set_val(sim.motor_inertia)
    sim.slider_friction.set_val(sim.friction)
    
    # Show the simulation
    print("P Controller Simulation")
    print("----------------------")
    print(f"Initial Parameters:")
    print(f"Kp: {sim.Kp}")
    print(f"Reference: {sim.reference}")
    print(f"Motor Inertia: {sim.motor_inertia}")
    print(f"Friction: {sim.friction}")
    print("\nUse the sliders to adjust parameters and observe the system response.")
    
    sim.show()