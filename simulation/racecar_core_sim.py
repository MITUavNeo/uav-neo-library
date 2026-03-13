"""
Backward-compatibility shim: racecar_core_sim -> drone_core_sim.
"""

from drone_core_sim import DroneSim as RacecarSim

__all__ = ["RacecarSim"]
