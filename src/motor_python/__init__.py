"""Motor control module for CubeMars AK60-6 exosuit actuators.

Primary interface: CAN (CubeMarsAK606v3CAN / Motor).
Legacy UART interface: CubeMarsAK606v3.
"""

__version__ = "0.0.6"

from motor_python.cube_mars_motor import CubeMarsAK606v3
from motor_python.cube_mars_motor_can import CubeMarsAK606v3CAN

# Convenience alias — CAN is the primary interface
Motor = CubeMarsAK606v3CAN

__all__ = ["CubeMarsAK606v3", "CubeMarsAK606v3CAN", "Motor"]
