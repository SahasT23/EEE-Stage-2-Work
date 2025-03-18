import matplotlib.pyplot as plt
import matplotlib.patches as patches

import time
import matplotlib.pyplot as plt

class HBridgeSimulator:
    def __init__(self):
        # Component parameters with defaults from schematic
        self.supply_voltage = 7.2      # V (VDD)
        self.mosfet_rds_on = 0.0017    # Ω (IRL40B215 typical)
        self.diode_vf = 0.45           # V (1N5819 typical)
        self.load_current = 2.0        # A (arbitrary default)
        
        # Control parameters
        self.direction = 'Stop'
        self.pwm_duty = 0
        self.motor_voltage = 0.0
        
        # Logging
        self.start_time = time.time()
        self.time_log = []
        self.voltage_log = []

    def set_component(self, component, **params):
        """Update component parameters"""
        valid_components = {
            'mosfet': ['rds_on'],
            'diode': ['vf'],
            'power': ['supply_voltage'],
            'load': ['current']
        }
        
        if component == 'mosfet':
            self.mosfet_rds_on = params.get('rds_on', self.mosfet_rds_on)
        elif component == 'diode':
            self.diode_vf = params.get('vf', self.diode_vf)
        elif component == 'power':
            self.supply_voltage = params.get('supply_voltage', self.supply_voltage)
        elif component == 'load':
            self.load_current = params.get('current', self.load_current)
        else:
            raise ValueError(f"Invalid component. Use: {', '.join(valid_components.keys())}")

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
            return

        # Calculate voltage drops
        mosfet_drop = 2 * self.mosfet_rds_on * self.load_current
        diode_drop = 2 * self.diode_vf
        
        # Calculate effective voltage components
        on_voltage = (self.supply_voltage - mosfet_drop) * (self.pwm_duty/100)
        off_voltage = -diode_drop * (1 - self.pwm_duty/100)
        
        # Apply direction
        sign = 1 if self.direction == 'Forward' else -1
        self.motor_voltage = sign * (on_voltage + off_voltage)

    def _record_voltage(self):
        self.time_log.append(time.time() - self.start_time)
        self.voltage_log.append(self.motor_voltage)

    def plot_voltage(self):
        plt.figure(figsize=(12, 6))
        plt.step(self.time_log, self.voltage_log, where='post', label='Motor Voltage')
        plt.title('H-Bridge Simulation with Component Parameterization')
        plt.xlabel('Time (seconds)')
        plt.ylabel('Voltage (V)')
        plt.grid(True)
        plt.axhline(0, color='black', lw=0.5)
        
        # Annotate parameter changes
        params = f"Vsup: {self.supply_voltage}V\nRds_on: {self.mosfet_rds_on*1000:.2f}mΩ\nVf: {self.diode_vf}V\nIload: {self.load_current}A"
        plt.annotate(params, (0.05, 0.7), xycoords='axes fraction', 
                    bbox=dict(boxstyle='round', fc='white', ec='gray'))
        
        plt.legend()
        plt.show()

    def __str__(self):
        params = (
            f"Supply Voltage: {self.supply_voltage}V\n"
            f"MOSFET Rds(on): {self.mosfet_rds_on*1000:.2f}mΩ\n"
            f"Diode Vf: {self.diode_vf}V\n"
            f"Load Current: {self.load_current}A\n"
            f"Direction: {self.direction}\n"
            f"PWM Duty: {self.pwm_duty}%\n"
            f"Motor Voltage: {self.motor_voltage:.2f}V"
        )
        return params

# Example usage
if __name__ == "__main__":
    h_bridge = HBridgeSimulator()
    
    # Initial parameters
    h_bridge.set_component('mosfet', rds_on=0.002)  # 2mΩ MOSFETs
    h_bridge.set_component('diode', vf=0.5)         # Higher Vf diodes
    h_bridge.set_component('load', current=3.0)     # 3A load
    
    # Test sequence
    h_bridge.set_direction('Forward')
    h_bridge.set_pwm(80)
    time.sleep(2)
    
    h_bridge.set_component('power', supply_voltage=9.0)  # Change supply voltage
    time.sleep(1)
    
    h_bridge.set_direction('Reverse')
    h_bridge.set_pwm(60)
    time.sleep(1.5)
    
    h_bridge.plot_voltage()
    print(h_bridge)

class HBridgeSimulator(HBridgeSimulator):  # Inherits from previous version
    def draw_circuit(self):
        fig, ax = plt.subplots(figsize=(12, 8))
        ax.set_aspect('equal')
        ax.axis('off')
        
        # Draw MOSFETs
        self._draw_mosfet(ax, (3, 7), "TR1\nIRL40B215", "N-CH")
        self._draw_mosfet(ax, (3, 3), "TR2\nIRL40B215", "N-CH")
        self._draw_mosfet(ax, (7, 7), "TR3\nIPP80P03P4L-04", "P-CH")
        self._draw_mosfet(ax, (7, 3), "TR4\nIPP80P03P4L-04", "P-CH")
        
        # Draw Diodes
        self._draw_diode(ax, (2.5, 6.5), "D1\n1N5819")
        self._draw_diode(ax, (2.5, 3.5), "D2\n1N5819")
        self._draw_diode(ax, (7.5, 6.5), "D3\n1N5819")
        self._draw_diode(ax, (7.5, 3.5), "D4\n1N5819")
        
        # Draw Power Rails
        ax.add_patch(patches.Rectangle((2, 8), 6, 0.2, color='red'))  # VDD
        ax.add_patch(patches.Rectangle((2, 2), 6, 0.2, color='black'))  # GND
        
        # Draw Motor
        ax.add_patch(patches.Circle((5, 5), 0.5, color='skyblue'))
        ax.text(5, 5, "MOTOR\nJ1", ha='center', va='center')
        
        # Draw Drivers
        self._draw_ic(ax, (1, 4), "U1A\nTC4426CPA")
        self._draw_ic(ax, (9, 4), "U1B\nTC4426CPA")
        
        # Draw Connections
        self._draw_wire(ax, [(3, 8), (3, 7.5)])  # TR1 to VDD
        self._draw_wire(ax, [(7, 8), (7, 7.5)])  # TR3 to VDD
        self._draw_wire(ax, [(3, 2.5), (3, 2)])  # TR2 to GND
        self._draw_wire(ax, [(7, 2.5), (7, 2)])  # TR4 to GND
        self._draw_wire(ax, [(3, 5), (5, 5)])    # Motor connections
        self._draw_wire(ax, [(7, 5), (5, 5)])
        
        # Add component parameters
        param_text = (
            f"Supply Voltage: {self.supply_voltage} V\n"
            f"MOSFET Rds(on): {self.mosfet_rds_on*1000:.2f} mΩ\n"
            f"Diode Vf: {self.diode_vf} V\n"
            f"Load Current: {self.load_current} A"
        )
        ax.text(10, 7, param_text, bbox=dict(facecolor='white', alpha=0.8))
        
        plt.title("H-Bridge Circuit Schematic with Live Parameters")
        plt.show()

    def _draw_mosfet(self, ax, position, label, mos_type):
        x, y = position
        color = 'darkgreen' if 'N-CH' in mos_type else 'darkred'
        
        # Draw MOSFET symbol
        ax.add_patch(patches.Rectangle((x-0.3, y-0.3), 0.6, 0.6, fill=False, ec=color))
        ax.plot([x], [y], marker='$'+mos_type[0]+'$', markersize=15, color=color)
        ax.text(x, y-0.7, label, ha='center', fontsize=8)
        
        # Draw terminals
        ax.text(x+0.5, y, "D", ha='left', va='center', fontsize=8)
        ax.text(x-0.5, y, "S", ha='right', va='center', fontsize=8)
        ax.text(x, y+0.5, "G", ha='center', va='bottom', fontsize=8)

    def _draw_diode(self, ax, position, label):
        x, y = position
        ax.add_patch(patches.Polygon(
            [(x, y), (x+0.5, y), (x+0.25, y+0.5), (x+0.25, y-0.5)],
            fill=True, color='orange'
        ))
        ax.text(x+0.7, y, label, ha='left', va='center', fontsize=8)

    def _draw_ic(self, ax, position, label):
        x, y = position
        ax.add_patch(patches.Rectangle((x-0.5, y-1), 1, 2, fill=True, ec='black', fc='lightgray'))
        ax.text(x, y, label, ha='center', va='center', fontsize=8)

    def _draw_wire(self, ax, points):
        x = [p[0] for p in points]
        y = [p[1] for p in points]
        ax.plot(x, y, color='gray', linewidth=2)

# Example usage
if __name__ == "__main__":
    h_bridge = HBridgeSimulator()
    
    # Modify components
    h_bridge.set_component('mosfet', rds_on=0.0025)
    h_bridge.set_component('diode', vf=0.48)
    h_bridge.set_component('power', supply_voltage=9)
    
    # Draw the circuit
    h_bridge.draw_circuit()
    
    # Run simulation
    h_bridge.set_direction('Forward')
    h_bridge.set_pwm(75)
    h_bridge.plot_voltage()