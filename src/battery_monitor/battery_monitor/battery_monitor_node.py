import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
import board
from adafruit_ina219 import INA219

class BatteryMonitor(Node):
    def __init__(self):
        super().__init__('battery_monitor_node')

        # Initialize INA219
        i2c_bus = board.I2C()
        self.ina219 = INA219(i2c_bus, addr=0x40)

        # Create a publisher for battery state
        self.battery_publisher = self.create_publisher(BatteryState, 'battery_state', 10)

        # Set the timer to publish data at regular intervals
        self.timer = self.create_timer(1.0, self.publish_battery_data)

    def publish_battery_data(self):
        # Create a new BatteryState message
        msg = BatteryState()

        # Get voltage, current, and power from INA219
        msg.voltage = self.ina219.bus_voltage  # Battery voltage in Volts
        msg.current = self.ina219.current / 1000.0  # Current in Amps
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING

        # Publish the message
        self.battery_publisher.publish(msg)
        # self.get_logger().info(f"Battery Voltage: {msg.voltage:.2f} V, Current: {msg.current:.2f} A")

def main(args=None):
    rclpy.init(args=args)
    battery_monitor = BatteryMonitor()
    rclpy.spin(battery_monitor)

    battery_monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
