import time
import matplotlib.pyplot as plt

class HBridgeSimulator:
    def __init__(self, supply_voltage=7.2):
        self.supply_voltage = supply_voltage
        self.direction = 'Stop'
        self.pwm_duty = 0
        self.motor_voltage = 0.0
        self.start_time = time.time()
        self.time_log = []
        self.voltage_log = []
        
        # Record initial state
        self._record_voltage()

    def _record_voltage(self):
        elapsed_time = time.time() - self.start_time
        self.time_log.append(elapsed_time)
        self.voltage_log.append(self.motor_voltage)

    def set_direction(self, direction):
        direction = direction.capitalize()
        if direction in ['Forward', 'Reverse', 'Stop']:
            self.direction = direction
            self._calculate_voltage()
            self._record_voltage()
        else:
            raise ValueError("Invalid direction. Use 'Forward', 'Reverse', or 'Stop'")

    def set_pwm(self, duty_cycle):
        if 0 <= duty_cycle <= 100:
            self.pwm_duty = duty_cycle
            self._calculate_voltage()
            self._record_voltage()
        else:
            raise ValueError("PWM duty cycle must be between 0 and 100")

    def _calculate_voltage(self):
        if self.direction == 'Stop':
            self.motor_voltage = 0.0
        else:
            sign = 1 if self.direction == 'Forward' else -1
            self.motor_voltage = sign * self.supply_voltage * (self.pwm_duty / 100)

    def plot_voltage(self):
        plt.figure(figsize=(10, 5))
        plt.step(self.time_log, self.voltage_log, where='post', label='Motor Voltage')
        plt.title('H-Bridge Motor Voltage Simulation')
        plt.xlabel('Time (seconds)')
        plt.ylabel('Voltage (V)')
        plt.grid(True)
        plt.axhline(0, color='black', linewidth=0.5)
        
        # Annotate direction changes
        for i, t in enumerate(self.time_log):
            if i > 0:
                dir_change = "↑ Forward" if self.voltage_log[i] > 0 else "↓ Reverse" if self.voltage_log[i] < 0 else "■ Stop"
                plt.annotate(dir_change, (t, self.voltage_log[i]),
                             textcoords="offset points", xytext=(0,10),
                             ha='center', fontsize=8, color='red')
        
        plt.legend()
        plt.show()

    def __str__(self):
        status = {
            'direction': self.direction,
            'pwm_duty': f"{self.pwm_duty}%",
            'motor_voltage': f"{self.motor_voltage:.2f}V",
            'state': 'Running' if self.pwm_duty > 0 and self.direction != 'Stop' else 'Stopped'
        }
        return (
            f"H-Bridge Status:\n"
            f"Direction: {status['direction']}\n"
            f"PWM Duty Cycle: {status['pwm_duty']}\n"
            f"Motor Voltage: {status['motor_voltage']}\n"
            f"State: {status['state']}"
        )

# Example usage with plotting
if __name__ == "__main__":
    h_bridge = HBridgeSimulator()
    
    # Simulate different states
    h_bridge.set_direction('Forward')
    h_bridge.set_pwm(75)
    time.sleep(2)  # Simulate time between operations
    
    h_bridge.set_pwm(50)
    time.sleep(1)
    
    h_bridge.set_direction('Reverse')
    h_bridge.set_pwm(30)
    time.sleep(1.5)
    
    h_bridge.set_direction('Stop')
    
    # Generate plot
    h_bridge.plot_voltage()