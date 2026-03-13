"""
Backward-compatibility shim: racecar_core -> drone_core.

Existing scripts that `import racecar_core` and call `create_racecar()`
will continue to work through this wrapper.
"""

from drone_core import Drone as Racecar, create_drone as create_racecar

__all__ = ["Racecar", "create_racecar"]
