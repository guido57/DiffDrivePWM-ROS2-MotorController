import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import board
import busio
from adafruit_ina219 import INA219, BusVoltageRange

class BatteryMonitor(Node):
    def __init__(self):
        super().__init__('battery_monitor')

        # Initialize I2C bus 1 and specify the address (e.g., 0x41)
        i2c = busio.I2C(board.SCL, board.SDA)
        i2c_address = 0x40  # Set your specific INA219 I2C address here
        self.ina219 = INA219(i2c, addr=i2c_address)
        
        # Publishers for voltage and current
        self.voltage_publisher = self.create_publisher(Float32, 'battery_voltage', 10)
        self.current_publisher = self.create_publisher(Float32, 'battery_current', 10)

        # Timer to publish data at a fixed interval (e.g., 1 second)
        self.timer = self.create_timer(1.0, self.publish_battery_data)
    
    def publish_battery_data(self):
        # Read voltage and current
        try:
            voltage = self.ina219.bus_voltage + self.ina219.shunt_voltage / 1000
            current = self.ina219.current / 1000  # in Amperes
            # Log and publish the data
            self.get_logger().info(f'Voltage: {voltage:.2f} V, Current: {current:.2f} A')
            
            # Publish voltage
            voltage_msg = Float32()
            voltage_msg.data = voltage
            self.voltage_publisher.publish(voltage_msg)
            
            # Publish current
            current_msg = Float32()
            current_msg.data = current
            self.current_publisher.publish(current_msg)
        
        except Exception as e:
            self.get_logger().error(f'Error reading from INA219: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    battery_monitor = BatteryMonitor()
    
    try:
        rclpy.spin(battery_monitor)
    except KeyboardInterrupt:
        pass
    
    battery_monitor.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
