#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float64
import math
import numpy as np
import matplotlib.pyplot as plt
import threading

h = 10.0
l = 0.415 #Distance between thrusters(actual boat)
# l=2 it is for vrx
A = np.array([[1.0, 1.0], [-l/2, l/2]])
IA = np.linalg.inv(A)
MAX_THRUST=3000.0

EARTH_RADIUS_KM = 6371

class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0.0
        self.integral = 0.0

    def compute(self, error):
        self.integral += error
        derivative = error - self.prev_error
        self.prev_error = error
        return (self.Kp * error + self.Ki * self.integral + self.Kd * derivative) * 50

class ASVController(Node):
    def __init__(self):
        super().__init__('asv_controller')

        self.thruster_publisher_left = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
        self.thruster_publisher_right = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)

        self.gps_subscription = self.create_subscription(NavSatFix, '/wamv/sensors/gps/gps/fix', self.gps_callback, 10)
        self.imu_subscription = self.create_subscription(Imu, '/wamv/sensors/imu/imu/data', self.imu_callback, 10)

        self.timer = self.create_timer(2.0, self.log_position)

        self.reference_lat = -33.72281907897446
        self.reference_lon = 150.67395245498903
        self.reference_alt = 1.7199274003505707

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_yaw = 0.0
        self.waypoints = [(50.0, 60.0), (100.0, 50.0), (70.0, 80.0), (100.0, 150.0)]
        #self.waypoints = [(100.0, 100.0), (50.0, 50.0), (70.0, 80.0), (75.0, 85.0), (80.0, 90.0)]
        self.current_waypoint_index = 0
        
        self.distance_tolerance = 2.0
        self.heading_pid = PIDController(25.0, 0.0, 10.0)
        self.distance_pid = PIDController(3.0, 0.0, 0.1)

        #add lines for graphing
        self.x_coords = []
        self.y_coords = []
        self.plot_thread = threading.Thread(target=self.plot_coordinates)
        self.plot_thread.daemon = True
        self.plot_thread.start()

    def gps_callback(self, msg):
        latitude = msg.latitude
        longitude = msg.longitude
        altitude = msg.altitude
        self.current_x, self.current_y, self.current_z = self.convert_to_xyz(latitude, longitude, altitude)
        self.x_coords.append(self.current_x)
        self.y_coords.append(self.current_y)
        self.navigate_to_waypoint()

    def imu_callback(self, msg):
        orientation = msg.orientation
        self.current_yaw = self.normalize_angle(self.quaternion_to_yaw(orientation.x, orientation.y, orientation.z, orientation.w))

    def navigate_to_waypoint(self):
        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info("All waypoints reached!")
            thrust_left=0.0
            thrust_right=0.0
            self.publish_thrust(thrust_left, thrust_right)
            return
        
        waypoint_x, waypoint_y = self.waypoints[self.current_waypoint_index]
        prev_x, prev_y = self.waypoints[self.current_waypoint_index-1]

        dx = waypoint_x - prev_x
        dy = waypoint_y - prev_y
        
        if dx == 0:
            a = float('inf')
            b = 0
            c = -prev_x
        else:
            a = dy / dx
            b = -1.0
            c = prev_y - prev_x * a

        d = abs(a * self.current_x + b * self.current_y + c) / math.sqrt(a**2 + b**2)
        waypoints_angle = math.atan2(dy,dx)   #self.normalize_angle(self.normalize_direction(dy, dx))

        delta_x = waypoint_x - self.current_x
        delta_y = waypoint_y - self.current_y
        distance = math.sqrt(delta_x**2 + delta_y**2)

       

        if self.current_waypoint_index>0:
           
            foot_x=(self.current_x - a*(a*self.current_x + b*self.current_y + c)/(a**2 + b**2))
            foot_y=(self.current_y - b*(a*self.current_x + b*self.current_y + c)/(a**2 + b**2))
            if distance > h:
                intermediate_x = foot_x + h * math.cos(waypoints_angle)
                intermediate_y = foot_y + h * math.sin(waypoints_angle)
                
            else:
                intermediate_x = waypoint_x
                intermediate_y = waypoint_y
               
           
        else:
            intermediate_x = waypoint_x
            intermediate_y = waypoint_y
        
        delta_x_heading = intermediate_x - self.current_x
        delta_y_heading = intermediate_y - self.current_y
        target_angle = math.atan2(delta_y_heading,delta_x_heading)  #self.normalize_angle(self.normalize_direction(delta_x_heading,delta_y_heading))
        angle_diff = self.normalize_angle(target_angle - self.current_yaw)

        dis = math.sqrt((intermediate_x-self.current_x)**2+(intermediate_y-self.current_y)**2)
        self.get_logger().info(
            f"Waypoint {self.current_waypoint_index}: Target x={intermediate_x}, y={intermediate_y}, "
            f"Current x={self.current_x:.2f}, y={self.current_y:.2f}, Distance={distance:.2f}, yaw={math.degrees(self.current_yaw):.2f}, Target angle={math.degrees(target_angle):.2f}, Angle Diff={math.degrees(angle_diff):.2f}°"
        )

        heading_control = self.heading_pid.compute(angle_diff)
        distance_control = self.distance_pid.compute(dis)

        self.get_logger().info(f"Heading Control: {heading_control:.2f}, Distance Control: {distance_control:.2f}")

        m=-1/a
        z= self.current_y - waypoint_y - m*(self.current_x-waypoint_x)
        if distance < self.distance_tolerance or -0.1<(z)<0.1:
            self.get_logger().info(f"Reached waypoint {self.current_waypoint_index}!")
            self.current_waypoint_index += 1
            return

        B = np.array([[distance_control], [heading_control]])
        result = np.dot(IA, B)
        thrust_left = result[0][0]
        thrust_right = result[1][0]
        thrust_left=max(min(thrust_left,MAX_THRUST),-MAX_THRUST)
        thrust_right=max(min(thrust_right,MAX_THRUST),-MAX_THRUST)

        self.publish_thrust(thrust_left, thrust_right)

        self.get_logger().info(f"Thrust left :{thrust_left}| Thrust right: {thrust_right}")

    def publish_thrust(self, left, right):
    
        msg_left = Float64()
        msg_right = Float64()
        msg_left.data = left
        msg_right.data = right
        self.thruster_publisher_left.publish(msg_left)
        self.thruster_publisher_right.publish(msg_right)

    def log_position(self):
        self.get_logger().info(f"Position: x={self.current_x:.2f}, y={self.current_y:.2f}, z={self.current_z:.2f}")

    def convert_to_xyz(self, latitude, longitude, altitude):
        lat_rad = math.radians(latitude)
        lon_rad = math.radians(longitude)
        ref_lat_rad = math.radians(self.reference_lat)
        ref_lon_rad = math.radians(self.reference_lon)
        delta_lat = lat_rad - ref_lat_rad
        delta_lon = lon_rad - ref_lon_rad
        x = delta_lon * EARTH_RADIUS_KM * 1000 * math.cos(ref_lat_rad)
        y = delta_lat * EARTH_RADIUS_KM * 1000
        z = altitude - self.reference_alt
        return x, y, z

    def quaternion_to_yaw(self, x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

   
    
    def plot_coordinates(self):
        plt.ion()
        fig, ax = plt.subplots()
        while rclpy.ok():
            ax.clear()
            ax.plot(self.x_coords, self.y_coords, marker='o', linestyle='-')
            ax.plot(*zip(*self.waypoints), marker='x', linestyle='', color='red')
            ax.set_title('ASV Path')
            ax.set_xlabel('X Coordinate')
            ax.set_ylabel('Y Coordinate')
            plt.draw()
            plt.pause(0.1)

def main(args=None):
    rclpy.init(args=args)
    asv_controller = ASVController()
    rclpy.spin(asv_controller)
    asv_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
