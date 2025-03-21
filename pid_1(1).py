#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
import math
import numpy as np

# Constants for thruster matrix
h = 10.0
l = 2.0
A = np.array([[1.0, 1.0], [-l/2, l/2]])
IA = np.linalg.inv(A)

class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0.0
        self.integral = 0.0
    
    def control(self, error):
        self.integral += error
        derivative = error - self.prev_error
        self.prev_error = error
        return (self.kp * error + self.ki * self.integral + self.kd * derivative) * 10

class ASVController(Node):
    def __init__(self):
        super().__init__('asv_controller')
        
        # Thruster publishers
        self.thruster_publisher_left = self.create_publisher(Float64, '/left_thrust', 10)
        self.thruster_publisher_right = self.create_publisher(Float64, '/right_thrust', 10)
        
        # Position and IMU subscribers
        self.create_subscription(Odometry, '/kf/odom', self.odom_callback, 10)
        
        self.current_x = None
        self.current_y = None
        self.current_yaw = 0.0
        
        self.waypoint = (10.0, 0.0)  # Target waypoint
        self.distance_tolerance = 2.0
        self.angle_tolerance = 0.05  # Radians
        self.heading_pid = PID(10.0, 0.0, 0.5)
        self.distance_pid = PID(3.0, 0.0, 0.1)

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        _, _, yaw = self.quaternion_to_euler(orientation.x, orientation.y, orientation.z, orientation.w)
        self.current_yaw = self.normalize_angle(yaw)
        self.get_logger().info(f"Current Position: ({self.current_x}, {self.current_y}), Yaw: {math.degrees(self.current_yaw)}")
        self.navigate_to_waypoint()

    def quaternion_to_euler(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)
        
        t2 = +2.0 * (w * y - z * x)
        t2 = max(-1.0, min(1.0, t2))
        pitch = math.asin(t2)
        
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
    
        return roll, pitch, yaw  # Radians

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def navigate_to_waypoint(self):
        if self.current_x is None or self.current_y is None:
            return
        
        target_x, target_y = self.waypoint
        dx = target_x - self.current_x
        dy = target_y - self.current_y
        target_angle = math.atan2(dy, dx)
        angle_error = self.normalize_angle(target_angle - self.current_yaw)
        # if abs(angle_error) <= self.angle_tolerance:
        #     self.get_logger().info("Turning Complete")
            
        
        # else:
        #     self.get_logger().info("Turning In Progress")

        
        
        distance_control = self.distance_pid.control(math.sqrt(dx**2 + dy**2)) #if abs(angle_error) <= self.angle_tolerance else 0.0
        heading_control = self.heading_pid.control(angle_error)
        
        B = np.array([[distance_control], [heading_control]])
        result = np.dot(IA, B)
        thrust_left = (result[0][0]/1000)*4 #just for testing sake.
        thrust_right = (result[1][0]/1000)*4
        self.get_logger().info(f"Thrust -> Left: {thrust_left}, Right: {thrust_right}")

        
        self.publish_thrust(thrust_left, thrust_right)
        

    def force_to_pwm(self, thrust: float) -> float:
        if thrust < -0.07:
            # pwm = 4.4913 * thrust**3 + 34.6510 * thrust**2 + 168.7654 * thrust + 1463.9
            pwm=1.4094*thrust**3 - 16.5621*thrust**2+117.0371*thrust+1534.1570
            return float(max(pwm, 1300))  
        elif thrust > 0.07:
            # pwm = 2.1809 * thrust**3 - 22.0578 * thrust**2 + 134.6641 * thrust + 1536.3
            pwm=3.3116*thrust**3 + 28.9093*thrust**2+152.1417*thrust+1467.7035
            return float(min(pwm, 1700))  
        else:
            return 1500.0  

    def publish_thrust(self, left_thrust, right_thrust):
        left_msg = Float64()
        right_msg = Float64()
        left_msg.data = self.force_to_pwm(left_thrust)
        right_msg.data = self.force_to_pwm(right_thrust)
        self.thruster_publisher_left.publish(left_msg)
        self.thruster_publisher_right.publish(right_msg)
        self.get_logger().info(f"\nThrust -> Left: {left_msg.data}|| Right: {right_msg.data}")


def main(args=None):
    rclpy.init(args=args)
    asv_controller = ASVController()
    rclpy.spin(asv_controller)
    asv_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
