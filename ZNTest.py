import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button, RadioButtons
from scipy import signal

class ZNFirstMethodSimulation:
    def __init__(self):
        # Process parameters (second-order system with delay)
        self.K = 1.0            # Process gain
        self.tau1 = 1.0         # First time constant
        self.tau2 = 0.5         # Second time constant
        self.delay = 0.2        # Process delay
        
        # Simulation parameters
        self.simulation_time = 20.0    # Total simulation time in seconds
        self.time_step = 0.01          # Time step for simulation
        self.reference = 100.0         # Setpoint
        
        # PID parameters (to be tuned)
        self.Kp = 1.0
        self.Ki = 0.0
        self.Kd = 0.0
        
        # Ziegler-Nichols First Method parameters
        self.S = 0.0            # Slope of response
        self.L = 0.0            # Dead time
        self.T = 0.0            # Time constant
        
        # Create the figure for simulation
        self.create_simulation()
        
    def create_process_model(self):
        """Create a transfer function model of the process"""
        # Second-order process with delay
        num = [self.K]
        den = [self.tau1 * self.tau2, self.tau1 + self.tau2, 1]
        delay_steps = int(self.delay / self.time_step)
        
        return num, den, delay_steps
    
    def step_response(self):
        """Calculate the open-loop step response of the process"""
        num, den, delay_steps = self.create_process_model()
        
        # Time vector
        t = np.arange(0, self.simulation_time, self.time_step)
        
        # Create a step input at t=1
        u = np.zeros_like(t)
        step_index = int(1.0 / self.time_step)
        u[step_index:] = 1.0
        
        # Simulate the system response using lsim
        sys = signal.TransferFunction(num, den)
        tout, y, _ = signal.lsim(sys, u, t)  # Fixed line - unpack 3 values
        
        # Apply the delay
        y_delayed = np.zeros_like(y)
        if delay_steps > 0:
            y_delayed[delay_steps:] = y[:-delay_steps]
        else:
            y_delayed = y
            
        return t, u, y_delayed
    
    def closed_loop_simulation(self, Kp, Ki, Kd):
        """Simulate the closed-loop response with PID controller"""
        num, den, delay_steps = self.create_process_model()
        
        # Time vector
        t = np.arange(0, self.simulation_time, self.time_step)
        
        # Create arrays for signals
        r = np.ones_like(t) * self.reference  # Reference (setpoint)
        y = np.zeros_like(t)                  # Process output
        u = np.zeros_like(t)                  # Control signal
        e = np.zeros_like(t)                  # Error
        p_term = np.zeros_like(t)             # Proportional term
        i_term = np.zeros_like(t)             # Integral term
        d_term = np.zeros_like(t)             # Derivative term
        
        # Create the process model
        sys = signal.TransferFunction(num, den)
        
        # Initial conditions
        integral = 0
        prev_error = 0
        
        # Apply a simple anti-windup limit for the integral term
        i_term_limit = 100.0
        u_limit = 150.0  # Control signal limit
        
        # Simulation loop
        for i in range(1, len(t)):
            # Calculate error
            e[i] = r[i] - y[i-1]
            
            # Calculate PID terms
            p_term[i] = Kp * e[i]
            
            # Update integral with anti-windup
            integral += Ki * e[i] * self.time_step
            integral = np.clip(integral, -i_term_limit, i_term_limit)
            i_term[i] = integral
            
            # Calculate derivative term (on measurement, not error, for stability)
            if i > 1:
                d_term[i] = -Kd * (y[i-1] - y[i-2]) / self.time_step
            
            # Sum the terms to get control signal
            u[i] = p_term[i] + i_term[i] + d_term[i]
            
            # Apply control signal limits
            u[i] = np.clip(u[i], -u_limit, u_limit)
            
            # Apply the process model to get the next output
            # For simplicity, we'll use a discrete approximation of the continuous model
            t_segment = t[i-1:i+1]
            u_segment = np.array([u[i-1], u[i]])
            
            # Fixed line - get only output from lsim
            _, y_response, _ = signal.lsim(sys, u_segment, t_segment, y[i-1:i])
            
            y_next = y_response[-1]
            
            # Apply delay
            if i + delay_steps < len(t):
                y[i] = y_next
            
            # Store previous error for derivative calculation
            prev_error = e[i]
        
        return t, r, y, u, e, p_term, i_term, d_term
    
    def zn_first_method(self, t, y):
        """
        Apply Ziegler-Nichols First Method (Process Reaction Curve)
        to determine PID parameters from step response.
        """
        # Find the maximum slope point
        dy = np.diff(y) / self.time_step
        max_slope_idx = np.argmax(dy) + 1
        
        # Calculate the slope at the inflection point
        self.S = dy[max_slope_idx - 1]
        
        # Draw a tangent line at the inflection point
        tangent_line = y[max_slope_idx] + self.S * (t - t[max_slope_idx])
        
        # Find where the tangent line intersects y=0
        L_idx = np.argmin(np.abs(tangent_line[:max_slope_idx]))
        self.L = t[L_idx]
        
        # Find where the tangent line intersects y=K (steady state)
        if y[-1] > 0:  # Make sure we've reached steady state
            ss_value = y[-1]
            T_idx = np.argmin(np.abs(tangent_line - ss_value))
            self.T = t[T_idx] - self.L
        else:
            # If steady state isn't reached, make an estimate
            self.T = (y[-1] / self.S) - self.L
        
        # Calculate PID parameters based on Ziegler-Nichols rules
        if self.L > 0 and self.T > 0:
            # P controller
            Kp_P = 1.0 / (self.S * self.L)
            
            # PI controller
            Kp_PI = 0.9 / (self.S * self.L)
            Ti_PI = self.L / 0.3
            Ki_PI = Kp_PI / Ti_PI
            
            # PID controller
            Kp_PID = 1.2 / (self.S * self.L)
            Ti_PID = 2.0 * self.L
            Ki_PID = Kp_PID / Ti_PID
            Td_PID = 0.5 * self.L
            Kd_PID = Kp_PID * Td_PID
            
            return {
                'P': {'Kp': Kp_P, 'Ki': 0, 'Kd': 0},
                'PI': {'Kp': Kp_PI, 'Ki': Ki_PI, 'Kd': 0},
                'PID': {'Kp': Kp_PID, 'Ki': Ki_PID, 'Kd': Kd_PID}
            }
        
        return None
    
    def create_simulation(self):
        """Create the interactive simulation interface"""
        # Create initial response curves
        t_step, u_step, y_step = self.step_response()
        t_cl, r_cl, y_cl, u_cl, e_cl, p_term, i_term, d_term = self.closed_loop_simulation(
            self.Kp, self.Ki, self.Kd)
        
        # Create figure and subplots
        self.fig = plt.figure(figsize=(12, 10))
        gs = self.fig.add_gridspec(12, 2)
        
        # Step response plot (for first method)
        self.ax_step = self.fig.add_subplot(gs[0:3, 0])
        self.step_line, = self.ax_step.plot(t_step, y_step, 'b-', label='Process Response')
        self.step_input, = self.ax_step.plot(t_step, u_step * y_step[-1], 'r--', label='Step Input')
        self.tangent_line, = self.ax_step.plot([], [], 'g--', label='Tangent Line')
        self.ax_step.set_title('Step Response - First Method')
        self.ax_step.set_ylabel('Amplitude')
        self.ax_step.set_xlabel('Time')
        self.ax_step.grid(True)
        self.ax_step.legend()
        
        # Closed-loop response plot
        self.ax_cl = self.fig.add_subplot(gs[0:3, 1])
        self.cl_output, = self.ax_cl.plot(t_cl, y_cl, 'b-', label='Output')
        self.cl_reference, = self.ax_cl.plot(t_cl, r_cl, 'r--', label='Reference')
        self.ax_cl.set_title('Closed-Loop Response')
        self.ax_cl.set_ylabel('Amplitude')
        self.ax_cl.set_xlabel('Time')
        self.ax_cl.grid(True)
        self.ax_cl.legend()
        
        # Control signal plot
        self.ax_control = self.fig.add_subplot(gs[3:6, 1])
        self.control_line, = self.ax_control.plot(t_cl, u_cl, 'g-', label='Control Signal')
        self.p_term_line, = self.ax_control.plot(t_cl, p_term, 'r--', label='P Term', alpha=0.5)
        self.i_term_line, = self.ax_control.plot(t_cl, i_term, 'b--', label='I Term', alpha=0.5)
        self.d_term_line, = self.ax_control.plot(t_cl, d_term, 'y--', label='D Term', alpha=0.5)
        self.ax_control.set_title('Control Signal')
        self.ax_control.set_ylabel('Amplitude')
        self.ax_control.set_xlabel('Time')
        self.ax_control.grid(True)
        self.ax_control.legend()
        
        # Error plot
        self.ax_error = self.fig.add_subplot(gs[3:6, 0])
        self.error_line, = self.ax_error.plot(t_cl, e_cl, 'm-', label='Error')
        self.ax_error.set_title('Error Signal')
        self.ax_error.set_ylabel('Amplitude')
        self.ax_error.set_xlabel('Time')
        self.ax_error.grid(True)
        self.ax_error.legend()
        
        # Sliders for process parameters
        self.ax_K = plt.axes([0.1, 0.35, 0.35, 0.02])
        self.ax_tau1 = plt.axes([0.1, 0.32, 0.35, 0.02])
        self.ax_tau2 = plt.axes([0.1, 0.29, 0.35, 0.02])
        self.ax_delay = plt.axes([0.1, 0.26, 0.35, 0.02])
        
        self.slider_K = Slider(self.ax_K, 'Process Gain (K)', 0.1, 5.0, valinit=self.K)
        self.slider_tau1 = Slider(self.ax_tau1, 'Time Constant 1', 0.1, 5.0, valinit=self.tau1)
        self.slider_tau2 = Slider(self.ax_tau2, 'Time Constant 2', 0.1, 5.0, valinit=self.tau2)
        self.slider_delay = Slider(self.ax_delay, 'Delay', 0.0, 2.0, valinit=self.delay)
        
        # Sliders for PID parameters
        self.ax_Kp = plt.axes([0.55, 0.35, 0.35, 0.02])
        self.ax_Ki = plt.axes([0.55, 0.32, 0.35, 0.02])
        self.ax_Kd = plt.axes([0.55, 0.29, 0.35, 0.02])
        self.ax_ref = plt.axes([0.55, 0.26, 0.35, 0.02])
        
        self.slider_Kp = Slider(self.ax_Kp, 'Kp', 0.0, 10.0, valinit=self.Kp)
        self.slider_Ki = Slider(self.ax_Ki, 'Ki', 0.0, 5.0, valinit=self.Ki)
        self.slider_Kd = Slider(self.ax_Kd, 'Kd', 0.0, 5.0, valinit=self.Kd)
        self.slider_ref = Slider(self.ax_ref, 'Reference', 0.0, 200.0, valinit=self.reference)
        
        # Controller type selection
        self.ax_controller = plt.axes([0.1, 0.18, 0.15, 0.05])
        self.radio_controller = RadioButtons(self.ax_controller, ('P', 'PI', 'PID'))
        
        # ZN parameters display
        self.ax_zn_params = plt.axes([0.3, 0.15, 0.6, 0.1])
        self.ax_zn_params.axis('off')
        
        # Buttons
        self.ax_calculate = plt.axes([0.1, 0.12, 0.2, 0.04])
        self.button_calculate = Button(self.ax_calculate, 'Calculate ZN Parameters')
        
        self.ax_apply = plt.axes([0.35, 0.12, 0.2, 0.04])
        self.button_apply = Button(self.ax_apply, 'Apply ZN Parameters')
        
        self.ax_reset = plt.axes([0.6, 0.12, 0.15, 0.04])
        self.button_reset = Button(self.ax_reset, 'Reset')
        
        # Connect callbacks
        self.slider_K.on_changed(self.update_process)
        self.slider_tau1.on_changed(self.update_process)
        self.slider_tau2.on_changed(self.update_process)
        self.slider_delay.on_changed(self.update_process)
        
        self.slider_Kp.on_changed(self.update_controller)
        self.slider_Ki.on_changed(self.update_controller)
        self.slider_Kd.on_changed(self.update_controller)
        self.slider_ref.on_changed(self.update_controller)
        
        self.radio_controller.on_clicked(self.update_controller_type)
        
        self.button_calculate.on_clicked(self.calculate_zn_parameters)
        self.button_apply.on_clicked(self.apply_zn_parameters)
        self.button_reset.on_clicked(self.reset)
        
        # Add title
        self.fig.suptitle('Ziegler-Nichols First Method (Process Reaction Curve) Simulation', fontsize=16)
        
        # Initial text for ZN parameters
        self.zn_text = self.ax_zn_params.text(0.05, 0.5, 'Calculate ZN parameters to see values', 
                                             transform=self.ax_zn_params.transAxes)
        
        # Adjust layout
        plt.tight_layout(rect=[0, 0.05, 1, 0.95])
        
        # Initialize ZN parameters
        self.zn_params = None
    
    def update_process(self, val=None):
        """Update the process model parameters"""
        self.K = self.slider_K.val
        self.tau1 = self.slider_tau1.val
        self.tau2 = self.slider_tau2.val
        self.delay = self.slider_delay.val
        
        # Update step response
        t_step, u_step, y_step = self.step_response()
        self.step_line.set_ydata(y_step)
        self.step_input.set_ydata(u_step * (y_step[-1] if y_step[-1] > 0 else 1.0))
        
        # Reset tangent line
        self.tangent_line.set_data([], [])
        
        # Update closed-loop response
        t_cl, r_cl, y_cl, u_cl, e_cl, p_term, i_term, d_term = self.closed_loop_simulation(
            self.Kp, self.Ki, self.Kd)
        
        self.cl_output.set_ydata(y_cl)
        self.control_line.set_ydata(u_cl)
        self.error_line.set_ydata(e_cl)
        self.p_term_line.set_ydata(p_term)
        self.i_term_line.set_ydata(i_term)
        self.d_term_line.set_ydata(d_term)
        
        # Adjust y-axis limits
        self.ax_step.relim()
        self.ax_step.autoscale_view()
        self.ax_cl.relim()
        self.ax_cl.autoscale_view()
        self.ax_control.relim()
        self.ax_control.autoscale_view()
        self.ax_error.relim()
        self.ax_error.autoscale_view()
        
        # Clear ZN parameters
        self.zn_params = None
        self.zn_text.set_text('Calculate ZN parameters to see values')
        
        self.fig.canvas.draw_idle()
    
    def update_controller(self, val=None):
        """Update PID controller parameters"""
        self.Kp = self.slider_Kp.val
        self.Ki = self.slider_Ki.val
        self.Kd = self.slider_Kd.val
        self.reference = self.slider_ref.val
        
        # Update closed-loop response
        t_cl, r_cl, y_cl, u_cl, e_cl, p_term, i_term, d_term = self.closed_loop_simulation(
            self.Kp, self.Ki, self.Kd)
        
        self.cl_output.set_ydata(y_cl)
        self.cl_reference.set_ydata(r_cl)
        self.control_line.set_ydata(u_cl)
        self.error_line.set_ydata(e_cl)
        self.p_term_line.set_ydata(p_term)
        self.i_term_line.set_ydata(i_term)
        self.d_term_line.set_ydata(d_term)
        
        # Adjust y-axis limits
        self.ax_cl.relim()
        self.ax_cl.autoscale_view()
        self.ax_control.relim()
        self.ax_control.autoscale_view()
        self.ax_error.relim()
        self.ax_error.autoscale_view()
        
        self.fig.canvas.draw_idle()
    
    def update_controller_type(self, label):
        """Update the selected controller type (P, PI, PID)"""
        if self.zn_params is not None:
            # Update display of parameter values
            ctrl_params = self.zn_params[label]
            
            # Format the text to display
            text = f"Ziegler-Nichols First Method Parameters ({label} Controller):\n"
            text += f"Kp = {ctrl_params['Kp']:.4f}\n"
            text += f"Ki = {ctrl_params['Ki']:.4f}\n"
            text += f"Kd = {ctrl_params['Kd']:.4f}\n"
            text += f"\nProcess Parameters: L = {self.L:.4f}, T = {self.T:.4f}, Slope = {self.S:.4f}"
            
            self.zn_text.set_text(text)
            
            self.fig.canvas.draw_idle()
    
    def calculate_zn_parameters(self, event):
        """Calculate Ziegler-Nichols parameters using First Method"""
        # Get the step response
        t_step, _, y_step = self.step_response()
        
        # Calculate ZN parameters using first method
        self.zn_params = self.zn_first_method(t_step, y_step)
        
        if self.zn_params is not None:
            # Get the current controller type
            controller_type = self.radio_controller.value_selected
            
            # Update display of parameter values
            ctrl_params = self.zn_params[controller_type]
            
            # Format the text to display
            text = f"Ziegler-Nichols First Method Parameters ({controller_type} Controller):\n"
            text += f"Kp = {ctrl_params['Kp']:.4f}\n"
            text += f"Ki = {ctrl_params['Ki']:.4f}\n"
            text += f"Kd = {ctrl_params['Kd']:.4f}\n"
            text += f"\nProcess Parameters: L = {self.L:.4f}, T = {self.T:.4f}, Slope = {self.S:.4f}"
            
            self.zn_text.set_text(text)
            
            # Draw the tangent line on the step response
            dy = np.diff(y_step) / self.time_step
            max_slope_idx = np.argmax(dy) + 1
            
            x_range = np.array([0, self.simulation_time])
            y_range = y_step[max_slope_idx] + self.S * (x_range - t_step[max_slope_idx])
            
            self.tangent_line.set_data(x_range, y_range)
        else:
            self.zn_text.set_text("Could not calculate ZN parameters.\nTry adjusting process parameters.")
        
        self.fig.canvas.draw_idle()
    
    def apply_zn_parameters(self, event):
        """Apply calculated Ziegler-Nichols parameters to the controller"""
        if self.zn_params is not None:
            # Get the current controller type
            controller_type = self.radio_controller.value_selected
            
            # Get parameters for this controller type
            ctrl_params = self.zn_params[controller_type]
            
            # Update sliders
            self.slider_Kp.set_val(ctrl_params['Kp'])
            self.slider_Ki.set_val(ctrl_params['Ki'])
            self.slider_Kd.set_val(ctrl_params['Kd'])
            
            # Update controller parameters
            self.Kp = ctrl_params['Kp']
            self.Ki = ctrl_params['Ki']
            self.Kd = ctrl_params['Kd']
            
            # Update closed-loop response
            self.update_controller()
    
    def reset(self, event):
        """Reset all parameters to default values"""
        # Reset process parameters
        self.slider_K.reset()
        self.slider_tau1.reset()
        self.slider_tau2.reset()
        self.slider_delay.reset()
        
        # Reset controller parameters
        self.slider_Kp.reset()
        self.slider_Ki.reset()
        self.slider_Kd.reset()
        self.slider_ref.reset()
        
        # Reset Ziegler-Nichols parameters
        self.zn_params = None
        self.zn_text.set_text('Calculate ZN parameters to see values')
        
        # Reset tangent line
        self.tangent_line.set_data([], [])
        
        # Update plots
        self.update_process()
        self.update_controller()
    
    def show(self):
        """Show the interactive plot"""
        plt.show()

# Main execution
if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="Ziegler-Nichols First Method Simulation")
    parser.add_argument('--K', type=float, default=1.0, help='Process gain')
    parser.add_argument('--tau1', type=float, default=1.0, help='First time constant')
    parser.add_argument('--tau2', type=float, default=0.5, help='Second time constant')
    parser.add_argument('--delay', type=float, default=0.2, help='Process delay')
    
    args = parser.parse_args()
    
    # Create and show simulation
    sim = ZNFirstMethodSimulation()
    
    # Set parameters from command line arguments
    sim.K = args.K
    sim.tau1 = args.tau1
    sim.tau2 = args.tau2
    sim.delay = args.delay
    
    # Update sliders to match command line arguments
    sim.slider_K.set_val(sim.K)
    sim.slider_tau1.set_val(sim.tau1)
    sim.slider_tau2.set_val(sim.tau2)
    sim.slider_delay.set_val(sim.delay)
    
    # Show simulation instructions
    print("Ziegler-Nichols First Method Simulation")
    print("---------------------------------------")
    print("This simulation demonstrates the Process Reaction Curve method for PID tuning.")
    print()
    print("Instructions:")
    print("1. Adjust process parameters to modify the system dynamics")
    print("2. Click 'Calculate ZN Parameters' to analyze the step response")
    print("3. Choose controller type (P, PI, or PID)")
    print("4. Click 'Apply ZN Parameters' to apply the calculated values")
    print()
    print("The simulation shows:")
    print("- Step response for process characterization with tangent line")
    print("- Closed-loop system response with the current controller")
    print("- Control signal and individual PID components")
    print("- Error signal over time")
    
    # Show the simulation
    sim.show()