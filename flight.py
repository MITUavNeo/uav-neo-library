"""
Copyright MIT
MIT License

UAV Neo Drone Course

File Name: flight.py
File Description: Defines the interface of the Flight module of the drone_core library.
"""

import abc


class Flight(abc.ABC):
    """
    Controls the drone's flight via piloting commands (pitch, roll, yaw, throttle).
    """

    @abc.abstractmethod
    def send_pcmd(
        self, pitch: float, roll: float, yaw: float, throttle: float
    ) -> None:
        """
        Sends a piloting command to the drone.

        Args:
            pitch: Forward/backward tilt from -1.0 (backward) to 1.0 (forward).
            roll: Left/right tilt from -1.0 (left) to 1.0 (right).
            yaw: Rotation from -1.0 (counter-clockwise) to 1.0 (clockwise).
            throttle: Vertical speed from -1.0 (descend) to 1.0 (ascend).

        Note:
            All arguments are unitless ratios clamped to [-1.0, 1.0].

        Example::

            # Fly forward at half speed
            rc.flight.send_pcmd(0.5, 0, 0, 0)

            # Ascend while yawing right
            rc.flight.send_pcmd(0, 0, 0.3, 0.5)
        """
        pass

    def stop(self) -> None:
        """
        Zeros all flight inputs, bringing the drone to a hover.

        Note:
            Equivalent to rc.flight.send_pcmd(0, 0, 0, 0).

        Example::

            if counter > 5:
                rc.flight.stop()
        """
        self.send_pcmd(0, 0, 0, 0)

    @abc.abstractmethod
    def set_max_speed(self, max_speed: float = 0.25) -> None:
        """
        Sets the maximum speed multiplier for flight commands.

        Args:
            max_speed: A scale factor from 0.0 to 1.0 applied to all flight inputs.

        Example::

            rc.flight.set_max_speed(0.5)
        """
        pass

    @abc.abstractmethod
    def takeoff(self) -> bool:
        """
        Arms the motors and initiates an automatic launch sequence.

        Returns:
            True if the drone is now armed and launching, False if already armed.

        Example::

            if rc.flight.takeoff():
                print("Launching!")
        """
        pass

    @abc.abstractmethod
    def land(self) -> bool:
        """
        Initiates an automatic landing sequence.

        Returns:
            True if landing was initiated, False if not armed or already landing.

        Example::

            if rc.flight.land():
                print("Landing initiated")
        """
        pass
