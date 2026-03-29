"""
Copyright MIT
MIT License

UAV Neo Drone Course

File Name: physics_real.py
File Description: Contains the Physics module of the drone_core library
"""

from physics import Physics

# General
from collections import deque
import numpy as np
from nptyping import NDArray

# ROS2
import rclpy as ros2
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSReliabilityPolicy,
    QoSProfile,
)
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import TwistStamped


class PhysicsReal(Physics):
    # The ROS topic from which we read imu data
    __IMU_TOPIC = "/imu" 
    __NAV_TOPIC = "/nav"
    __TWIST_TOPIC = "/twist"

    # Limit on buffer size to prevent memory overflow
    __BUFFER_CAP = 60

    def __init__(self):
        self.node = ros2.create_node("imu_sub")

        qos_profile = QoSProfile(depth=1)
        qos_profile.history = QoSHistoryPolicy.KEEP_LAST
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_profile.durability = QoSDurabilityPolicy.VOLATILE

        # subscribe to the imu topic, which will call
        # __imu_callback every time the IMU publishes data
        self.__imu_sub = self.node.create_subscription(
            Imu, self.__IMU_TOPIC, self.__imu_callback, qos_profile
        )
        
        # subscribe to the nav topic, which will call
        # __nav_callback every time the IMU publishes data
        self.__nav_sub = self.node.create_subscription(
            NavSatFix, self.__NAV_TOPIC, self.__nav_callback, qos_profile
        )

        # subscribe to the twist topic, which will call
        # __twist_callback every time the IMU publishes data
        self.__twist_sub = self.node.create_subscription(
            TwistStamped, self.__TWIST_TOPIC, self.__twist_callback, qos_profile
        )

        self.__acceleration = np.array([0, 0, 0])
        self.__acceleration_buffer = deque()
        self.__linear_velocity = np.array([0, 0, 0])
        self.__linear_velocity_buffer = deque()
        self.__angular_velocity = np.array([0, 0, 0])
        self.__angular_velocity_buffer = deque()
        self.__altitude = 0.0
        self.__altitude_buffer = deque()
        self.__attitude = np.array([0, 0, 0])
        self.__attitude_buffer = deque()
        self.__gps = np.array([0, 0, 0])
        self.__gps_buffer = deque()

    def __imu_callback(self, data):
        new_acceleration = np.array(
            [data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z]
        )

        self.__acceleration_buffer.append(new_acceleration)
        if len(self.__acceleration_buffer) > self.__BUFFER_CAP:
            self.__acceleration_buffer.popleft()

        new_angular_velocity = np.array(
            [data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z]
        )

        self.__angular_velocity_buffer.append(new_angular_velocity)
        if len(self.__angular_velocity_buffer) > self.__BUFFER_CAP:
            self.__angular_velocity_buffer.popleft()

        new_attitude = np.array(
            quaternion_to_euler_angle_vectorized2(data.orientation.w, data.orientation.x, data.orientation.y, data.orientation.z)
        )

        self.__attitude_buffer.append(new_attitude)
        if len(self.__attitude_buffer) > self.__BUFFER_CAP:
            self.__attitude_buffer.popleft()

    def __nav_callback(self, data):
        new_altitude = float(
            data.altitude
        )

        self.__altitude_buffer.append(new_altitude)
        if len(self.__altitude_buffer) > self.__BUFFER_CAP:
            self.__altitude_buffer.popleft()

        new_gps = np.array(
            [data.latitude, data.longitude, data.altitude]
        )
            
        self.__gps_buffer.append(new_gps)
        if len(self.__gps_buffer) > self.__BUFFER_CAP:
            self.__gps_buffer.popleft()
 
    def __twist_callback(self, data):
        new_linear_velocity = np.array(
            [data.linear.x, data.linear.y, data.linear.z]
        )

        self.__linear_velocity_buffer.append(new_linear_velocity)
        if len(self.__linear_velocity_buffer) > self.__BUFFER_CAP:
            self.__linear_velocity_buffer.popleft()

    def __update(self):
        if len(self.__acceleration_buffer) > 0:
            self.__acceleration = np.mean(self.__acceleration_buffer, axis=0)
            self.__acceleration_buffer.clear()

        if len(self.__linear_velocity_buffer) > 0:
            self.__linear_velocity = np.mean(self.__linear_velocity_buffer, axis=0)
            self.__linear_velocity_buffer.clear() 

        if len(self.__angular_velocity_buffer) > 0:
            self.__angular_velocity = np.mean(self.__angular_velocity_buffer, axis=0)
            self.__angular_velocity_buffer.clear() 
            
        if len(self.__altitude_buffer) > 0:
            self.__altitude = np.mean(self.__altitude, axis=0)
            self.__altitude_buffer.clear()

        if len(self.__attitude_buffer) > 0:
            self.__attitude = np.mean(self.__attitude_buffer, axis=0)
            self.__attitude_buffer.clear() 
            
        if len(self.__gps_buffer) > 0:
            self.__gps = np.mean(self.__gps_buffer, axis=0)
            self.__gps_buffer.clear()

    def get_linear_acceleration(self) -> NDArray[3, np.float32]:
        return np.array(self.__acceleration)

    def get_linear_velocity(self) -> NDArray[3, np.float32]:
        return np.array(self.__linear_velocity)
    
    def get_angular_velocity(self) -> NDArray[3, np.float32]:
        return np.array(self.__angular_velocity) 

    def get_altitude(self) -> NDArray[3, np.float32]:
        return np.array(self.__altitude) 
    
    def get_attitude(self) -> NDArray[3, np.float32]:
        return np.array(self.__attitude)

    def get_gps(self) -> NDArray[3, np.float32]:
        return np.array(self.__gps) 
 
def quaternion_to_euler_angle_vectorized2(w, x, y, z): 
    ysqr = y * y

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    X = np.degrees(np.arctan2(t0, t1))

    t2 = +2.0 * (w * y - z * x)

    t2 = np.clip(t2, a_min=-1.0, a_max=1.0)
    Y = np.degrees(np.arcsin(t2))

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    Z = np.degrees(np.arctan2(t3, t4))

    return [X, Y, Z]