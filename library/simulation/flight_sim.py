"""
Copyright MIT
GNU General Public License v3.0

MIT BWSI Autonomous Drone Racing Course - UAV Neo

File Name: flight_sim.py
File Description: Flight simulation module — sends flight commands to the UAVNeo Simulator via UDP.
"""

import struct

from flight import Flight


class FlightSim(Flight):
    def __init__(self, drone) -> None:
        self.__drone = drone

    def send_pcmd(
        self, pitch: float, roll: float, yaw: float, throttle: float
    ) -> None:
        assert -1.0 <= pitch <= 1.0, f"pitch [{pitch}] must be between -1.0 and 1.0 inclusive."
        assert -1.0 <= roll <= 1.0, f"roll [{roll}] must be between -1.0 and 1.0 inclusive."
        assert -1.0 <= yaw <= 1.0, f"yaw [{yaw}] must be between -1.0 and 1.0 inclusive."
        assert -1.0 <= throttle <= 1.0, f"throttle [{throttle}] must be between -1.0 and 1.0 inclusive."

        self.__drone._DroneSim__send_data(
            struct.pack(
                "Bffff",
                self.__drone.Header.flight_send_pcmd.value,
                pitch, roll, yaw, throttle,
            )
        )

    def set_max_speed(self, max_speed: float = 0.25) -> None:
        assert (
            0.0 <= max_speed <= 1.0
        ), f"max_speed [{max_speed}] must be between 0.0 and 1.0 inclusive."

        self.__drone._DroneSim__send_data(
            struct.pack(
                "Bf",
                self.__drone.Header.flight_set_max_speed.value,
                max_speed,
            )
        )

    def takeoff(self) -> bool:
        self.__drone._DroneSim__send_header(
            self.__drone.Header.flight_takeoff
        )
        data = self.__drone._DroneSim__receive_data(1)
        return bool(data[0])

    def land(self) -> bool:
        self.__drone._DroneSim__send_header(
            self.__drone.Header.flight_land
        )
        data = self.__drone._DroneSim__receive_data(1)
        return bool(data[0])
