"""Motor control module for CubeMars AK60-6 motors via UART and CAN."""

from motor_python.cube_mars_motor import CubeMarsAK606v3
from motor_python.cube_mars_motor_can import CubeMarsAK606CAN

__version__ = "0.0.4"

__all__ = ["CubeMarsAK606CAN", "CubeMarsAK606v3"]
