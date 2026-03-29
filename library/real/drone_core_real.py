"""
Copyright MIT
MIT License

UAV Neo Drone Course

File Name: drone_core_real.py
File Description: Contains the Drone class, the top level of the drone_core library
"""

# General
from datetime import datetime
import threading
from typing import Callable, Optional

# ROS2
import rclpy as ros2

# racecar_core modules
import camera_real
import controller_real
import display_real
import flight_real
import physics_real
import telemetry_real

from drone_core import Drone


class DroneReal(Drone):
    # Default number of seconds to wait between calls to update_slow
    __DEFAULT_UPDATE_SLOW_TIME = 1

    # Number of frames per second
    __FRAME_RATE = 60

    def __init__(self, isHeadless: bool = False):
        # initialize ROS 2
        ros2.init()
        self.__executor = ros2.get_global_executor()
        self.__rate_node = ros2.create_node("rate_node")

        # Modules
        self.camera = camera_real.CameraReal()
        self.controller = controller_real.ControllerReal(self)
        self.display = display_real.DisplayReal(isHeadless)
        self.flight = flight_real.FlightReal() 
        self.physics = physics_real.PhysicsReal()
        self.telemetry = telemetry_real.TelemetryReal()

        # Add all nodes to the executor
        rate_added = self.__executor.add_node(self.__rate_node)
        camera_added = self.__executor.add_node(self.camera.node)
        flight_added = self.__executor.add_node(self.flight.node)
        controller_added = self.__executor.add_node(self.controller.node)
        physics_added = self.__executor.add_node(self.physics.node) 
        assert rate_added and flight_added and camera_added and controller_added and physics_added, (
            "Issues initializing Drone nodes. Node status: \n"
            f"Rate operational: {rate_added} | "
            f"Camera operational: {camera_added} | "
            f"Flight operational: {flight_added} | "
            f"Controller operational: {controller_added} | "
            f"Physics operational: {physics_added} | " 
        )

        # User provided start and update functions
        self.__user_start = None
        self.__user_update = None
        self.__user_update_slow = None

        # True if the main thread should be running
        self.__running = False

        # Variables relating to the run thread
        self.__run_thread = None
        self.__cur_update = self.__default_update
        self.__cur_update_slow = None
        self.__cur_frame_time = datetime.now()
        self.__last_frame_time = datetime.now()
        self.__cur_update_counter = 0
        self.__max_update_counter = 1
        self.set_update_slow_time(self.__DEFAULT_UPDATE_SLOW_TIME)

        # Start run_thread in default drive mode
        self.__handle_back()
        self.__run_thread = threading.Thread(target=self.__run)
        self.__run_thread.daemon = True
        self.__run_thread.start()

        # Async thread handler
        self.__async_thread = None

        # Print welcome message
        print(">> Drone initialization successful")
        print(
            ">> Controls:\n"
            "    START button = run your program\n"
            "    BACK button = enter default flight mode\n"
            "    BACK + START buttons simultaneously = exit the program\n"
            "    CTRL + Z on keyboard = force quit the program"
        )

    def go(self) -> None:
        self.__running = True
        while self.__running:
            try:
                self.__executor.spin_once()
            except KeyboardInterrupt:
                break
        ros2.shutdown()

    def go_async(self) -> None:
        self.__running = True
        self.__async_thread = threading.Thread(target=self.__spin_async, daemon=True)
        self.__async_thread.start()

    def __spin_async(self) -> None:
        self.__user_start = self.__default_start
        self.__user_update = self.__default_update
        self.__handle_start()
        while self.__running:
            self.__executor.spin_once()
        
    def set_start_update(
        self,
        start: Callable[[], None],
        update: Callable[[], None],
        update_slow: Optional[Callable[[], None]] = None,
    ) -> None:
        self.__user_start = start
        self.__user_update = update
        self.__user_update_slow = update_slow

    def get_delta_time(self) -> float:
        return (self.__cur_frame_time - self.__last_frame_time).total_seconds()

    def set_update_slow_time(self, time: float = 1.0) -> None:
        self.__max_update_counter = max(1, round(time * self.__FRAME_RATE))

    def __handle_start(self):
        """
        Handles when the START button is pressed by entering user program mode.
        """
        if self.__user_start is None or self.__user_update is None:
            print(
                ">> No user start and update functions found.  "
                "Did you call set_start_update with valid start and "
                "update functions?"
            )
        else:
            print(">> Entering user program mode")
            self.__user_start()
            self.__cur_update = self.__user_update
            self.__cur_update_slow = self.__user_update_slow

    def __handle_back(self):
        """
        Handles when the BACK button is pressed by entering default flight mode.
        """
        print(">> Entering default flight mode")
        self.__default_start()
        self.__cur_update = self.__default_update
        self.__cur_update_slow = None

    def __handle_exit(self):
        """
        Handles when BACK and START are pressed together by exiting the program.
        """
        print(">> Goodbye!")
        self.__running = False

    def __run(self):
        """
        Calls the current update and update_modules once per frame.
        """
        rate = self.__rate_node.create_rate(self.__FRAME_RATE)
        while True:
            self.__last_frame_time = self.__cur_frame_time
            self.__cur_frame_time = datetime.now()
            self.__cur_update()
            self.__update_modules()

            # Use a counter to decide when we need to call update_slow
            if self.__cur_update_slow is not None:
                self.__cur_update_counter -= 1
                if self.__cur_update_counter <= 0:
                    self.__cur_update_slow()
                    self.__cur_update_counter = self.__max_update_counter

            rate.sleep()

    def __update_modules(self):
        """
        Calls the update function on each module.
        """
        self.flight._FlightReal__update()
        self.controller._ControllerReal__update()
        self.camera._CameraReal__update()
        self.physics._PhysicsReal__update()
        self.telemetry._TelemetryReal__update()

    def __default_start(self):
        """
        The start function for default flight mode.
        """
        self.flight.stop()

    def __default_update(self):
        """
        The update function for default flight mode.

        Controls:
            Left/right triggers: Throttle
            Left joystick left/right: Yaw
            Right joystick up/down: Pitch
            Right joystick left/right: Roll
        """
        MAX_SPEED = 0.25   

        pitch    = self.controller.get_joystick(self.controller.Joystick.RIGHT)[1] * MAX_SPEED
        roll     = self.controller.get_joystick(self.controller.Joystick.RIGHT)[0] * MAX_SPEED
        yaw      = self.controller.get_joystick(self.controller.Joystick.LEFT)[0]  * MAX_SPEED
        throttle = (self.controller.get_trigger(self.controller.Trigger.RIGHT) - self.controller.get_trigger(self.controller.Trigger.LEFT))  * MAX_SPEED

        self.flight.send_pcmd(pitch, roll, yaw, throttle)