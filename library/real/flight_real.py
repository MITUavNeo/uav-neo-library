"""
Copyright MIT
MIT License

UAV Neo Drone Course

File Name: flight_real.py
File Description: Contains the Flight module of the drone_core library.
Publishes velocity setpoints to /mux/cmd_vel for the mux node to arbitrate.
Does NOT arm, disarm, or change flight modes — that is the safety pilot's job.
"""

from flight import Flight

import rclpy as ros2
from geometry_msgs.msg import TwistStamped


class FlightReal(Flight):
    # Publish to the mux input — the mux node decides whether to forward
    __CMD_VEL_TOPIC = "/mux/cmd_vel"

    def __init__(self):
        self.__node = ros2.create_node("flight_pub")
        self.__clock = self.__node.get_clock()

        self.__publisher = self.__node.create_publisher(
            TwistStamped, self.__CMD_VEL_TOPIC, qos_profile=1
        )

        self.__message = TwistStamped()

        # Continuous publishing at 20 Hz to keep setpoints flowing
        self.__node.create_timer(0.05, self.__update)

    @property
    def node(self):
        return self.__node

    def send_pcmd(self, pitch: float, roll: float, yaw: float, throttle: float) -> None:
        assert (
            -1.0 <= pitch <= 1.0
        ), f"pitch [{pitch}] must be between -1.0 and 1.0 inclusive."
        assert (
            -1.0 <= roll <= 1.0
        ), f"roll [{roll}] must be between -1.0 and 1.0 inclusive."
        assert (
            -1.0 <= yaw <= 1.0
        ), f"yaw [{yaw}] must be between -1.0 and 1.0 inclusive."
        assert (
            -1.0 <= throttle <= 1.0
        ), f"throttle [{throttle}] must be between -1.0 and 1.0 inclusive."

        # Publish raw normalized values — the mux node applies max_speed scaling
        self.__message.twist.linear.x = float(pitch)
        self.__message.twist.linear.y = float(roll)
        self.__message.twist.linear.z = float(throttle)
        self.__message.twist.angular.z = float(yaw)

    def takeoff(self) -> None:
        """Send ascending setpoints. The safety pilot must arm and switch to
        OFFBOARD before the drone will actually lift off."""
        self.__message.twist.linear.x = 0.0
        self.__message.twist.linear.y = 0.0
        self.__message.twist.linear.z = 0.5
        self.__message.twist.angular.z = 0.0

    def land(self) -> None:
        """Send descending setpoints. The safety pilot handles the actual
        landing mode switch."""
        self.__message.twist.linear.x = 0.0
        self.__message.twist.linear.y = 0.0
        self.__message.twist.linear.z = -0.3
        self.__message.twist.angular.z = 0.0

    def set_max_speed(self, max_speed: float = 0.25) -> None:
        """No-op on real drone. Speed limit is enforced by the mux node config."""
        pass

    def __stamp_and_publish(self):
        self.__message.header.stamp = self.__clock.now().to_msg()
        self.__publisher.publish(self.__message)

    def __update(self):
        try:
            self.__stamp_and_publish()
        except Exception as e:
            self.__node.get_logger().error(f'Flight publish failed: {e}')
