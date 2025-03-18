import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button

class BangBangSimulation:
    def __init__(self):
        # Default parameters
        self.reference = 100.0   # Desired speed/position (target)
        self.hysteresis = 5.0    # Hysteresis band (prevents rapid switching)
        self.max_control = 80.0  # Maximum control signal value
        self.simulation_time = 10 # Total simulation time in seconds
        self.time_step = 0.01     # Time step for simulation
        self.motor_inertia = 0.01 # Motor inertia (resistance to changes in motion)
        self.friction = 0.5       # Friction coefficient
        self.noise_level = 0.0    # Level of measurement noise (0 for no noise)
        
        # Initial conditions
        self.initial_position = 0.0
        self.initial_velocity = 0.0
        
        # Create the simulation figure
        self.create_simulation()
    
    def bang_bang_control(self, measurement, reference, hysteresis):
        """
        Simple bang-bang controller with hysteresis
        
        Args:
            measurement: Current measured value
            reference: Desired setpoint
            hysteresis: Width of hysteresis band (prevents rapid switching)
            
        Returns:
            Control signal (positive or negative max value)
        """
        half_band = hysteresis / 2
        
        if measurement < reference - half_band:
            return self.max_control  # Full positive control
        elif measurement > reference + half_band:
            return -self.max_control  # Full negative control
        else:
            # Inside the hysteresis band - maintain previous state
            # This is a simplified approach for this simulation
            return 0  # For hysteresis band, we'll just use 0 as a middle state
    
    def simulate_system(self, reference, hysteresis, max_control, noise_level):
        """Simulate a simple DC motor with bang-bang control"""
        # Store the max control value
        self.max_control = max_control
        
        # Time array
        time = np.arange(0, self.simulation_time, self.time_step)
        
        # Arrays to store position, velocity, control signal, etc.
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
            
            # Add noise to the measurement if specified
            if noise_level > 0:
                measured_position = position[i-1] + np.random.normal(0, noise_level)
            else:
                measured_position = position[i-1]
            
            # Calculate control signal (bang-bang controller)
            control_signal[i-1] = self.bang_bang_control(measured_position, reference, hysteresis)
            
            # Calculate acceleration (simplified motor model)
            acceleration = (control_signal[i-1] - self.friction * velocity[i-1]) / self.motor_inertia
            
            # Update velocity using Euler integration
            velocity[i] = velocity[i-1] + acceleration * self.time_step
            
            # Update position using Euler integration
            position[i] = position[i-1] + velocity[i-1] * self.time_step
        
        # Final values
        error[-1] = reference - position[-1]
        
        # Add noise to the final measurement if specified
        if noise_level > 0:
            measured_position = position[-1] + np.random.normal(0, noise_level)
        else:
            measured_position = position[-1]
            
        control_signal[-1] = self.bang_bang_control(measured_position, reference, hysteresis)
        
        return time, position, velocity, control_signal, error
    
    def create_simulation(self):
        """Create interactive simulation plot"""
        # Initial simulation
        time, position, velocity, control_signal, error = self.simulate_system(
            self.reference, self.hysteresis, self.max_control, self.noise_level)
        
        # Create figure and subplots
        self.fig, self.axs = plt.subplots(4, 1, figsize=(10, 10), sharex=True)
        self.fig.subplots_adjust(bottom=0.35)  # Make room for sliders
        
        # Plot position
        self.pos_line, = self.axs[0].plot(time, position, 'b-', label='Position')
        self.ref_line, = self.axs[0].plot(time, np.ones_like(time) * self.reference, 'r--', label='Reference')
        # Plot hysteresis band
        self.upper_band, = self.axs[0].plot(time, np.ones_like(time) * (self.reference + self.hysteresis/2), 
                                       'r:', label='Hysteresis Band')
        self.lower_band, = self.axs[0].plot(time, np.ones_like(time) * (self.reference - self.hysteresis/2), 
                                       'r:', label='_nolegend_')
        self.axs[0].set_ylabel('Position')
        self.axs[0].set_title('System Response with Bang-Bang Control')
        self.axs[0].legend(loc='upper right')
        self.axs[0].grid(True)
        
        # Plot control signal
        self.ctrl_line, = self.axs[1].plot(time, control_signal, 'g-', label='Control Signal')
        self.axs[1].set_ylabel('Control Signal')
        self.axs[1].set_title('Bang-Bang Control Signal')
        self.axs[1].legend(loc='upper right')
        self.axs[1].grid(True)
        
        # Plot error
        self.error_line, = self.axs[2].plot(time, error, 'm-', label='Error')
        self.axs[2].set_ylabel('Error')
        self.axs[2].set_title('System Error')
        self.axs[2].legend(loc='upper right')
        self.axs[2].grid(True)
        
        # Plot velocity
        self.vel_line, = self.axs[3].plot(time, velocity, 'c-', label='Velocity')
        self.axs[3].set_xlabel('Time (s)')
        self.axs[3].set_ylabel('Velocity')
        self.axs[3].set_title('System Velocity')
        self.axs[3].legend(loc='upper right')
        self.axs[3].grid(True)
        
        # Add sliders for parameters
        ax_ref = plt.axes([0.25, 0.25, 0.65, 0.03])
        ax_hyst = plt.axes([0.25, 0.21, 0.65, 0.03])
        ax_max_ctrl = plt.axes([0.25, 0.17, 0.65, 0.03])
        ax_inertia = plt.axes([0.25, 0.13, 0.65, 0.03])
        ax_friction = plt.axes([0.25, 0.09, 0.65, 0.03])
        ax_noise = plt.axes([0.25, 0.05, 0.65, 0.03])
        
        # Create sliders
        self.slider_ref = Slider(ax_ref, 'Reference', 0.0, 200.0, valinit=self.reference)
        self.slider_hyst = Slider(ax_hyst, 'Hysteresis', 0.1, 20.0, valinit=self.hysteresis)
        self.slider_max_ctrl = Slider(ax_max_ctrl, 'Max Control', 10.0, 200.0, valinit=self.max_control)
        self.slider_inertia = Slider(ax_inertia, 'Motor Inertia', 0.001, 0.1, valinit=self.motor_inertia)
        self.slider_friction = Slider(ax_friction, 'Friction', 0.0, 2.0, valinit=self.friction)
        self.slider_noise = Slider(ax_noise, 'Noise Level', 0.0, 10.0, valinit=self.noise_level)
        
        # Add reset button
        ax_reset = plt.axes([0.8, 0.30, 0.1, 0.04])
        self.button_reset = Button(ax_reset, 'Reset')
        
        # Connect callbacks
        self.slider_ref.on_changed(self.update)
        self.slider_hyst.on_changed(self.update)
        self.slider_max_ctrl.on_changed(self.update)
        self.slider_inertia.on_changed(self.update)
        self.slider_friction.on_changed(self.update)
        self.slider_noise.on_changed(self.update)
        self.button_reset.on_clicked(self.reset)
        
        # Add title
        self.fig.suptitle('Bang-Bang Controller Simulation', fontsize=16)
        
        # Show plot
        plt.tight_layout(rect=[0, 0.35, 1, 0.95])
    
    def update(self, val=None):
        """Update the plot when sliders are moved"""
        # Get current parameter values from sliders
        self.reference = self.slider_ref.val
        self.hysteresis = self.slider_hyst.val
        self.max_control = self.slider_max_ctrl.val
        self.motor_inertia = self.slider_inertia.val
        self.friction = self.slider_friction.val
        self.noise_level = self.slider_noise.val
        
        # Run simulation with new parameters
        time, position, velocity, control_signal, error = self.simulate_system(
            self.reference, self.hysteresis, self.max_control, self.noise_level)
        
        # Update plots
        self.pos_line.set_ydata(position)
        self.ref_line.set_ydata(np.ones_like(time) * self.reference)
        self.upper_band.set_ydata(np.ones_like(time) * (self.reference + self.hysteresis/2))
        self.lower_band.set_ydata(np.ones_like(time) * (self.reference - self.hysteresis/2))
        self.ctrl_line.set_ydata(control_signal)
        self.error_line.set_ydata(error)
        self.vel_line.set_ydata(velocity)
        
        # Adjust y-axis limits as needed
        for ax in self.axs:
            ax.relim()
            ax.autoscale_view()
        
        # Redraw the figure
        self.fig.canvas.draw_idle()
    
    def reset(self, event):
        """Reset sliders to initial values"""
        self.slider_ref.reset()
        self.slider_hyst.reset()
        self.slider_max_ctrl.reset()
        self.slider_inertia.reset()
        self.slider_friction.reset()
        self.slider_noise.reset()
    
    def show(self):
        """Show the interactive plot"""
        plt.show()

# Create a command line interface to run the simulation
if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="DC Motor Bang-Bang Controller Simulation")
    parser.add_argument('--reference', type=float, default=100.0, help='Reference position/speed')
    parser.add_argument('--hysteresis', type=float, default=5.0, help='Hysteresis band width')
    parser.add_argument('--max_control', type=float, default=80.0, help='Maximum control signal')
    parser.add_argument('--inertia', type=float, default=0.01, help='Motor inertia')
    parser.add_argument('--friction', type=float, default=0.5, help='Friction coefficient')
    parser.add_argument('--noise', type=float, default=0.0, help='Measurement noise level')
    
    args = parser.parse_args()
    
    # Create simulation
    sim = BangBangSimulation()
    
    # Set parameters from command line arguments
    sim.reference = args.reference
    sim.hysteresis = args.hysteresis
    sim.max_control = args.max_control
    sim.motor_inertia = args.inertia
    sim.friction = args.friction
    sim.noise_level = args.noise
    
    # Update sliders to match command line arguments
    sim.slider_ref.set_val(sim.reference)
    sim.slider_hyst.set_val(sim.hysteresis)
    sim.slider_max_ctrl.set_val(sim.max_control)
    sim.slider_inertia.set_val(sim.motor_inertia)
    sim.slider_friction.set_val(sim.friction)
    sim.slider_noise.set_val(sim.noise_level)
    
    # Show the simulation
    print("Bang-Bang Controller Simulation")
    print("-------------------------------")
    print(f"Initial Parameters:")
    print(f"Reference: {sim.reference}")
    print(f"Hysteresis Band: {sim.hysteresis}")
    print(f"Max Control: {sim.max_control}")
    print(f"Motor Inertia: {sim.motor_inertia}")
    print(f"Friction: {sim.friction}")
    print(f"Noise Level: {sim.noise_level}")
    print("\nUse the sliders to adjust parameters and observe the system response.")
    
    sim.show()