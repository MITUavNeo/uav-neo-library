"""
Copyright MIT
MIT License

UAV Neo Drone Course

File Name: flight_real.py
File Description: Contains the Flight module of the drone_core library
"""

from flight import Flight

import rclpy as ros2
import numbers  
from geometry_msgs.msg import TwistStamped
from mavros_msgs.srv import CommandBool, SetMode 

import sys

sys.path.insert(0, "../../library")
import drone_utils as rc_utils


class FlightReal(Flight):
    # The ROS topic to which we publish flight messages
    __VEL_TOPIC = "/mavros/setpoint_velocity/cmd_vel_unstamped"

    __ARM_SERVICE = "/mavros/cmd/arming"
    __MODE_SERVICE = "/mavros/set_mode" 

    # PWM constants
    __PWM_MIN = 1000
    __PWM_MID = 1500
    __PWM_MAX = 2000

    def __init__(self):
        # ROS node
        self.__node = ros2.create_node("flight_pub")
        # publish to the flight topic, which will publish a message
        # every time __update is called
        self.__publisher = self.__node.create_publisher(
            TwistStamped, self.__VEL_TOPIC, qos_profile=1
        )
        self.__arm_client = self.__node.create_client(
            CommandBool, self.__ARM_SERVICE
        )
        self.__mode_client = self.__node.create_client(
            SetMode, self.__MODE_SERVICE
        )  

        self.__message = TwistStamped()
        self.__max_speed = 0.25 
        self.__armed = False

        # Keep running update, ensuring setpoints are sent to keep offboard mode
        self.__node.create_timer(0.05, self.__update)   

    def send_pcmd(self, pitch: float, roll: float, yaw: float, throttle: float) -> None: 
        assert (
            -1.0 <= pitch <= 1.0
        ), f"speed [{pitch}] must be between -1.0 and 1.0 inclusive."
        assert (
            -1.0 <= roll <= 1.0
        ), f"angle [{roll}] must be between -1.0 and 1.0 inclusive."
        assert (
            -1.0 <= yaw <= 1.0
        ), f"speed [{yaw}] must be between -1.0 and 1.0 inclusive."
        assert (
            -1.0 <= throttle <= 1.0
        ), f"angle [{throttle}] must be between -1.0 and 1.0 inclusive."

        self.__message.twist.linear.x  =  pitch    * self.__max_speed
        self.__message.twist.linear.y  = roll     * self.__max_speed
        self.__message.twist.linear.z  =  -throttle * self.__max_speed
        self.__message.twist.angular.z =  yaw      * self.__max_speed

    def set_max_speed(self, max_speed: float = 0.25) -> None:
        assert (
            0.0 <= max_speed <= 1.0
        ), f"max_speed [{max_speed}] must be between 0.0 and 1.0 inclusive."

        self.__max_speed_scale_factor = max_speed
        
    def takeoff(self) -> bool:
        if self.__armed:
            return False
        
        # Send a few setpoints before arming in offboard mode
        for i in range(20):
            self.__message.twist.linear.x = 0.0
            self.__message.twist.linear.y = 0.0
            self.__message.twist.linear.z = -0.1  
            self.__message.twist.angular.z = 0.0

            self.__publisher.publish(self.__message) 
            ros2.spin_once(self.__node, timeout_sec=0.05)

        mode_req = SetMode.Request()
        mode_req.custom_mode = "OFFBOARD"
        self.__mode_client.call(mode_req)

        arm_req = CommandBool.Request()
        arm_req.value = True
        self.__arm_client.call(arm_req)

        # Continue sending the setpoints during offboard mode 
        for i in range(50):
            self.__message.twist.linear.z = 0.3  # ascend
            self.__publisher.publish(self.__message)
            ros2.spin_once(self.__node, timeout_sec=0.05)

        # Stop the drone and have it hover in place
        self.__message.twist.linear.z = 0.0
        self.__publisher.publish(self.__message)

        self.__armed = True
        return True
        
    def land(self) -> bool:
        if not self.__armed:
            return False
        
        mode_req = SetMode.Request()
        mode_req.custom_mode = "AUTO.LAND"

        # Continue sending setpoints to maintain offboard
        for i in range(20):
            self.__message.twist.linear.x = 0.0
            self.__message.twist.linear.y = 0.0
            self.__message.twist.linear.z = 0.0
            self.__message.twist.angular.z = 0.0

            self.__publisher.publish(self.__message) 
            ros2.spin_once(self.__node, timeout_sec=0.05)

        self.__armed = False
        return True

    def __update(self):
        """
        Publishes the current drive message.
        """
        self.__publisher.publish(self.__message)
