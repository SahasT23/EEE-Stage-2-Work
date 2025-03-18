import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button

class MotorSimulation:
    def __init__(self):
        # Default parameters
        self.Kp = 10.0           # Proportional gain
        self.Ki = 2.0            # Integral gain
        self.Kd = 1.0            # Derivative gain
        self.reference = 100.0   # Desired speed/position (target)
        self.simulation_time = 5  # Total simulation time in seconds
        self.time_step = 0.01     # Time step for simulation
        self.motor_inertia = 0.01 # Motor inertia (resistance to changes in motion)
        self.friction = 0.5       # Friction coefficient
        
        # Initial conditions
        self.initial_position = 0.0
        self.initial_velocity = 0.0
        self.initial_integral = 0.0
        
        # Create the simulation figure
        self.create_simulation()
    
    def simulate_system(self, Kp, Ki, Kd, reference):
        """Simulate a simple DC motor with PID control"""
        # Time array
        time = np.arange(0, self.simulation_time, self.time_step)
        
        # Arrays to store position, velocity, and control signal
        position = np.zeros_like(time)
        velocity = np.zeros_like(time)
        control_signal = np.zeros_like(time)
        error = np.zeros_like(time)
        integral = np.zeros_like(time)
        deriv_error = np.zeros_like(time)
        p_term = np.zeros_like(time)
        i_term = np.zeros_like(time)
        d_term = np.zeros_like(time)
        
        # Set initial conditions
        position[0] = self.initial_position
        velocity[0] = self.initial_velocity
        integral[0] = self.initial_integral
        error[0] = reference - position[0]
        
        # Simulation loop
        for i in range(1, len(time)):
            # Calculate error
            error[i] = reference - position[i-1]
            
            # Update integral term
            integral[i] = integral[i-1] + error[i] * self.time_step
            
            # Anti-windup for integral term (limit the integral term)
            if Ki > 0:
                integral[i] = np.clip(integral[i], -100/Ki, 100/Ki)
            
            # Derivative term (using velocity for stability)
            deriv_error[i] = -velocity[i-1]  # Using negative velocity as derivative of error
            
            # Calculate PID terms
            p_term[i] = Kp * error[i]
            i_term[i] = Ki * integral[i]
            d_term[i] = Kd * deriv_error[i]
            
            # Calculate control signal (PID controller)
            control_signal[i] = p_term[i] + i_term[i] + d_term[i]
            
            # Limit control signal (motor saturation)
            control_signal[i] = np.clip(control_signal[i], -100, 100)
            
            # Calculate acceleration (simplified motor model)
            acceleration = (control_signal[i] - self.friction * velocity[i-1]) / self.motor_inertia
            
            # Update velocity using Euler integration
            velocity[i] = velocity[i-1] + acceleration * self.time_step
            
            # Update position using Euler integration
            position[i] = position[i-1] + velocity[i-1] * self.time_step
        
        return time, position, velocity, control_signal, error, p_term, i_term, d_term
    
    def create_simulation(self):
        """Create interactive simulation plot"""
        # Initial simulation
        time, position, velocity, control_signal, error, p_term, i_term, d_term = self.simulate_system(
            self.Kp, self.Ki, self.Kd, self.reference)
        
        # Create figure and subplots
        self.fig, self.axs = plt.subplots(4, 1, figsize=(10, 10), sharex=True)
        self.fig.subplots_adjust(bottom=0.35)  # Make room for sliders
        
        # Plot position
        self.pos_line, = self.axs[0].plot(time, position, 'b-', label='Position')
        self.ref_line, = self.axs[0].plot(time, np.ones_like(time) * self.reference, 'r--', label='Reference')
        self.axs[0].set_ylabel('Position')
        self.axs[0].set_title('System Response')
        self.axs[0].legend(loc='upper right')
        self.axs[0].grid(True)
        
        # Plot control signal and PID terms
        self.ctrl_line, = self.axs[1].plot(time, control_signal, 'g-', label='Control Signal')
        self.p_term_line, = self.axs[1].plot(time, p_term, 'y-', label='P Term', alpha=0.5)
        self.i_term_line, = self.axs[1].plot(time, i_term, 'm-', label='I Term', alpha=0.5)
        self.d_term_line, = self.axs[1].plot(time, d_term, 'c-', label='D Term', alpha=0.5)
        self.axs[1].set_ylabel('Control Signal')
        self.axs[1].set_title('Controller Components')
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
        ax_Kp = plt.axes([0.25, 0.25, 0.65, 0.03])
        ax_Ki = plt.axes([0.25, 0.21, 0.65, 0.03])
        ax_Kd = plt.axes([0.25, 0.17, 0.65, 0.03])
        ax_ref = plt.axes([0.25, 0.13, 0.65, 0.03])
        ax_inertia = plt.axes([0.25, 0.09, 0.65, 0.03])
        ax_friction = plt.axes([0.25, 0.05, 0.65, 0.03])
        
        # Create sliders
        self.slider_Kp = Slider(ax_Kp, 'Kp', 0.1, 50.0, valinit=self.Kp)
        self.slider_Ki = Slider(ax_Ki, 'Ki', 0.0, 10.0, valinit=self.Ki)
        self.slider_Kd = Slider(ax_Kd, 'Kd', 0.0, 5.0, valinit=self.Kd)
        self.slider_ref = Slider(ax_ref, 'Reference', 0.0, 200.0, valinit=self.reference)
        self.slider_inertia = Slider(ax_inertia, 'Motor Inertia', 0.001, 0.1, valinit=self.motor_inertia)
        self.slider_friction = Slider(ax_friction, 'Friction', 0.0, 2.0, valinit=self.friction)
        
        # Add reset button
        ax_reset = plt.axes([0.8, 0.30, 0.1, 0.04])
        self.button_reset = Button(ax_reset, 'Reset')
        
        # Connect callbacks
        self.slider_Kp.on_changed(self.update)
        self.slider_Ki.on_changed(self.update)
        self.slider_Kd.on_changed(self.update)
        self.slider_ref.on_changed(self.update)
        self.slider_inertia.on_changed(self.update)
        self.slider_friction.on_changed(self.update)
        self.button_reset.on_clicked(self.reset)
        
        # Add title
        self.fig.suptitle('PID Controller Simulation', fontsize=16)
        
        # Show plot
        plt.tight_layout(rect=[0, 0.35, 1, 0.95])
    
    def update(self, val=None):
        """Update the plot when sliders are moved"""
        # Get current parameter values from sliders
        self.Kp = self.slider_Kp.val
        self.Ki = self.slider_Ki.val
        self.Kd = self.slider_Kd.val
        self.reference = self.slider_ref.val
        self.motor_inertia = self.slider_inertia.val
        self.friction = self.slider_friction.val
        
        # Run simulation with new parameters
        time, position, velocity, control_signal, error, p_term, i_term, d_term = self.simulate_system(
            self.Kp, self.Ki, self.Kd, self.reference)
        
        # Update plots
        self.pos_line.set_ydata(position)
        self.ref_line.set_ydata(np.ones_like(time) * self.reference)
        self.ctrl_line.set_ydata(control_signal)
        self.p_term_line.set_ydata(p_term)
        self.i_term_line.set_ydata(i_term)
        self.d_term_line.set_ydata(d_term)
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
        self.slider_Kp.reset()
        self.slider_Ki.reset()
        self.slider_Kd.reset()
        self.slider_ref.reset()
        self.slider_inertia.reset()
        self.slider_friction.reset()
    
    def show(self):
        """Show the interactive plot"""
        plt.show()

# Create a command line interface to run the simulation
if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="DC Motor PID Controller Simulation")
    parser.add_argument('--Kp', type=float, default=10.0, help='Proportional gain')
    parser.add_argument('--Ki', type=float, default=2.0, help='Integral gain')
    parser.add_argument('--Kd', type=float, default=1.0, help='Derivative gain')
    parser.add_argument('--reference', type=float, default=100.0, help='Reference position/speed')
    parser.add_argument('--inertia', type=float, default=0.01, help='Motor inertia')
    parser.add_argument('--friction', type=float, default=0.5, help='Friction coefficient')
    
    args = parser.parse_args()
    
    # Create simulation
    sim = MotorSimulation()
    
    # Set parameters from command line arguments
    sim.Kp = args.Kp
    sim.Ki = args.Ki
    sim.Kd = args.Kd
    sim.reference = args.reference
    sim.motor_inertia = args.inertia
    sim.friction = args.friction
    
    # Update sliders to match command line arguments
    sim.slider_Kp.set_val(sim.Kp)
    sim.slider_Ki.set_val(sim.Ki)
    sim.slider_Kd.set_val(sim.Kd)
    sim.slider_ref.set_val(sim.reference)
    sim.slider_inertia.set_val(sim.motor_inertia)
    sim.slider_friction.set_val(sim.friction)
    
    # Show the simulation
    print("PID Controller Simulation")
    print("----------------------")
    print(f"Initial Parameters:")
    print(f"Kp: {sim.Kp}")
    print(f"Ki: {sim.Ki}")
    print(f"Kd: {sim.Kd}")
    print(f"Reference: {sim.reference}")
    print(f"Motor Inertia: {sim.motor_inertia}")
    print(f"Friction: {sim.friction}")
    print("\nUse the sliders to adjust parameters and observe the system response.")
    
    sim.show()