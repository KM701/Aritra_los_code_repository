import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import serial

class Receiver(Node):
    def __init__(self):
        super().__init__('receiver')

        try:
            self.arduino_serial = serial.Serial('/dev/ttyACM0', 115200, timeout=0.5)
            self.get_logger().info("Serial connection established successfully.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to serial port: {e}")
            return
        
        # Subscribe to thrust topics
        self.create_subscription(Float64, '/left_thrust', self.left_thrust_callback, 10)
        self.create_subscription(Float64, '/right_thrust', self.right_thrust_callback, 10)

        # Initialize thrust values
        self.left_thrust = None
        self.right_thrust = None

    def left_thrust_callback(self, msg):
        """Update left thrust value and check if both values are available."""
        self.left_thrust = msg.data
        self.send_to_arduino()

    def right_thrust_callback(self, msg):
        """Update right thrust value and check if both values are available."""
        self.right_thrust = msg.data
        self.send_to_arduino()

    def send_to_arduino(self):
        """Send thrust values to Arduino only when both are received at least once."""
        if self.left_thrust is not None and self.right_thrust is not None:
            try:
                data_to_send = f"L{self.left_thrust:.4f} R{self.right_thrust:.4f}\n"
                self.arduino_serial.write(data_to_send.encode('utf-8'))
                self.get_logger().info(f"Sent to Arduino: {data_to_send.strip()}")
            except Exception as e:
                self.get_logger().error(f"Error sending data: {e}")

    def destroy_node(self):
        """Clean up serial connection before shutting down."""
        if self.arduino_serial and self.arduino_serial.is_open:
            self.arduino_serial.close()
            self.get_logger().info("Serial connection closed.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = Receiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Receiver node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
