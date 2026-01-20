"""AK60-6 Motor Control Class - CubeMars CAN Protocol via MCP2515.

This module implements CAN communication with the AK60-6 motor using the
Waveshare RS485 CAN expansion board with MCP2515 CAN controller over SPI.
"""

import struct
import time
from dataclasses import dataclass
from enum import IntEnum
from typing import Any

from loguru import logger

from motor_python.definitions import (
    CAN_DEFAULTS,
    CAN_LIMITS,
    CAN_SCALE_FACTORS,
    MOTOR_LIMITS,
    CANErrorCode,
)
from motor_python.mcp2515 import MCP2515, MCP2515BaudRate


class CANMotorCommand(IntEnum):
    """CubeMars CAN command codes."""

    CMD_DUTY_CYCLE = 0x00
    CMD_CURRENT_LOOP = 0x01
    CMD_BRAKE_CURRENT = 0x02
    CMD_VELOCITY_LOOP = 0x03
    CMD_POSITION_LOOP = 0x04
    CMD_POSITION_VELOCITY_LOOP = 0x06
    CMD_MIT_VELOCITY_LOOP = 0x08
    CMD_MIT_POSITION_LOOP = 0x08
    CMD_MIT_TORQUE_LOOP = 0x08


@dataclass
class MotorCANStatus:
    """Motor status from CAN feedback message."""

    position: float  # Position in degrees
    speed: float  # Speed in RPM (electrical)
    current: float  # Current in amps
    temperature: int  # Temperature in degrees C
    error_code: int  # Error code


class CubeMarsAK606CAN:
    """AK60-6 Motor Controller for CubeMars CAN Protocol.

    This class implements the CAN interface for the AK60-6 motor using the
    Waveshare RS485 CAN expansion board with MCP2515 CAN controller via SPI.

    The motor uses extended CAN frames with 8 bytes of data in timed upload mode.
    Motor status is automatically uploaded at a configurable frequency (1-500 Hz).
    """

    def __init__(
        self,
        motor_id: int = CAN_DEFAULTS.motor_id,
        spi_bus: int = 0,
        spi_device: int = 0,
    ) -> None:
        """Initialize CAN motor connection via MCP2515.

        :param motor_id: Motor CAN ID (default: 0x68).
        :param spi_bus: SPI bus number (default: 0).
        :param spi_device: SPI device/chip select (default: 0).
        :return: None
        """
        self.motor_id = motor_id
        self.spi_bus = spi_bus
        self.spi_device = spi_device
        self.mcp2515: MCP2515 | None = None
        self.connected = False
        self.communicating = False
        self.last_status: MotorCANStatus | None = None
        self._consecutive_no_response = 0
        self._max_no_response = CAN_DEFAULTS.max_no_response_attempts
        self._connect()

    def _connect(self) -> None:
        """Establish CAN bus connection via MCP2515.

        :return: None
        """
        try:
            # Initialize MCP2515 CAN controller
            self.mcp2515 = MCP2515(
                spi_bus=self.spi_bus,
                spi_device=self.spi_device,
                baudrate=MCP2515BaudRate.CAN_500Kbps,
            )

            if self.mcp2515.connected:
                # Initialize the CAN controller
                if self.mcp2515.init():
                    self.connected = True
                    logger.info(
                        f"CAN motor controller initialized "
                        f"(Motor ID: 0x{self.motor_id:02X})"
                    )
                else:
                    logger.warning("MCP2515 initialization failed")
                    self.connected = False
            else:
                self.connected = False

        except ImportError as e:
            logger.error(f"Missing dependency: {e}")
            logger.info(
                "Install spidev with: pip install spidev\n"
                "Make sure SPI is enabled on your Jetson."
            )
            self.connected = False
        except Exception as e:
            logger.warning(f"Failed to connect to CAN bus: {e}")
            logger.info(
                "Troubleshooting:\n"
                "  1. Enable SPI: sudo /opt/nvidia/jetson-io/jetson-io.py\n"
                "  2. Check SPI device: ls /dev/spidev*\n"
                "  3. Install spidev: pip install spidev\n"
            )
            self.connected = False

    def _parse_motor_feedback(self, data: bytes) -> MotorCANStatus | None:
        """Parse 8-byte motor feedback message from CAN.

        Data format (8 bytes):
        - Bytes 0-1: Position (int16, high byte first)
        - Bytes 2-3: Speed (int16, high byte first)
        - Bytes 4-5: Current (int16, high byte first)
        - Byte 6: Temperature (int8)
        - Byte 7: Error code (uint8)

        :param data: 8 bytes of CAN data.
        :return: MotorCANStatus object or None if parsing fails.
        """
        if len(data) != 8:
            logger.warning(f"Invalid CAN data length: {len(data)} (expected 8)")
            return None

        try:
            # Parse position (int16, big-endian)
            pos_int = struct.unpack(">h", data[0:2])[0]
            position = pos_int * CAN_SCALE_FACTORS.position

            # Parse speed (int16, big-endian)
            spd_int = struct.unpack(">h", data[2:4])[0]
            speed = spd_int * CAN_SCALE_FACTORS.speed

            # Parse current (int16, big-endian)
            cur_int = struct.unpack(">h", data[4:6])[0]
            current = cur_int * CAN_SCALE_FACTORS.current

            # Parse temperature (int8)
            temperature = struct.unpack(">b", data[6:7])[0]

            # Parse error code (uint8)
            error_code = data[7]

            status = MotorCANStatus(
                position=position,
                speed=speed,
                current=current,
                temperature=temperature,
                error_code=error_code,
            )

            # Log parsed status
            logger.info("=" * 50)
            logger.info("MOTOR CAN FEEDBACK:")
            logger.info(f"  Position: {position:.2f}°")
            logger.info(f"  Speed: {speed:.0f} RPM (electrical)")
            logger.info(f"  Current: {current:.2f} A")
            logger.info(f"  Temperature: {temperature}°C")
            if error_code != 0:
                try:
                    error_desc = CANErrorCode(error_code).get_description()
                except ValueError:
                    error_desc = "Unknown error"
                logger.warning(f"  Error: {error_desc} (Code: {error_code})")
            else:
                logger.info("  Error: None")
            logger.info("=" * 50)

            return status

        except Exception as e:
            logger.error(f"Error parsing CAN feedback: {e}")
            return None

    def _send_can_message(self, cmd: int, data: bytes) -> bool:
        """Send CAN message to motor.

        Message format: [00 00 CMD ID DATA...]

        :param cmd: Command byte.
        :param data: Command data bytes.
        :return: True if message sent successfully, False otherwise.
        """
        if not self.connected or self.mcp2515 is None:
            logger.debug("CAN bus not connected - skipping message send")
            return False

        try:
            # Build message: [00 00 CMD ID DATA...]
            # The CAN ID for sending is based on command type
            # Using standard 11-bit ID format
            can_id = (cmd << 8) | self.motor_id

            # Prepare data bytes (pad to 8 bytes)
            message_data = list(data)
            while len(message_data) < 8:
                message_data.append(0x00)

            # Send via MCP2515
            result = self.mcp2515.send(can_id, message_data[:8], dlc=8)

            if result:
                logger.debug(
                    f"TX: CMD=0x{cmd:02X} ID=0x{self.motor_id:02X} "
                    f"Data={' '.join(f'{b:02X}' for b in message_data[:8])}"
                )

            return result

        except Exception as e:
            logger.error(f"Error sending CAN message: {e}")
            return False

    def receive_feedback(
        self, timeout: float = CAN_DEFAULTS.response_timeout
    ) -> MotorCANStatus | None:
        """Receive motor feedback from CAN bus.

        The motor automatically sends feedback messages at a configured rate (1-500 Hz).
        This method waits for and parses the next feedback message.

        :param timeout: Timeout in seconds to wait for feedback.
        :return: MotorCANStatus object or None if no feedback received.
        """
        if not self.connected or self.mcp2515 is None:
            logger.debug("CAN bus not connected - cannot receive feedback")
            return None

        try:
            msg = self.mcp2515.receive(timeout=timeout)

            if msg is None:
                self._consecutive_no_response += 1
                if self._consecutive_no_response >= self._max_no_response:
                    logger.warning(
                        f"No CAN feedback after {self._consecutive_no_response} attempts - "
                        "motor may be powered off or disconnected"
                    )
                    self.communicating = False
                return None

            # Parse feedback
            status = self._parse_motor_feedback(msg.data)
            if status:
                self.last_status = status
                self._consecutive_no_response = 0
                self.communicating = True

            return status

        except Exception as e:
            logger.error(f"Error receiving CAN feedback: {e}")
            return None

    def check_communication(self) -> bool:
        """Check if motor is responding to CAN messages.

        :return: True if motor is communicating, False otherwise.
        """
        logger.info("Checking CAN communication with motor...")

        # Try to receive feedback messages
        for attempt in range(3):
            status = self.receive_feedback(timeout=0.5)
            if status is not None:
                logger.success("Motor is responding via CAN")
                return True
            time.sleep(0.1)

        logger.warning("Motor not responding via CAN")
        return False

    def set_duty_cycle(self, duty: float) -> None:
        """Set motor duty cycle.

        :param duty: Duty cycle from -0.95 to 0.95 (-95% to 95%).
        :return: None
        """
        # Clamp duty cycle to valid range
        duty = max(MOTOR_LIMITS.min_duty_cycle, min(MOTOR_LIMITS.max_duty_cycle, duty))

        # Convert to motor protocol format: duty * 100000 -> int32
        duty_int = int(duty * 100000)

        # Pack as 4 bytes (int32, big-endian)
        data = struct.pack(">i", duty_int)

        logger.info(f"Setting duty cycle: {duty:.3f} (0x{duty_int:08X})")
        self._send_can_message(CANMotorCommand.CMD_DUTY_CYCLE, data)

    def set_current(self, current_amps: float) -> None:
        """Set motor current (IQ current).

        :param current_amps: Current in amps, from -60.0 to 60.0 A.
        :return: None
        """
        # Clamp current to valid range
        current_amps = max(
            MOTOR_LIMITS.min_current_amps,
            min(MOTOR_LIMITS.max_current_amps, current_amps),
        )

        # Convert to motor protocol format: current / 0.01 -> int16
        current_int = int(current_amps / CAN_SCALE_FACTORS.current)

        # Pack as 4 bytes (int32, big-endian)
        data = struct.pack(">i", current_int)

        logger.info(f"Setting current: {current_amps:.2f} A")
        self._send_can_message(CANMotorCommand.CMD_CURRENT_LOOP, data)

    def set_brake_current(self, current_amps: float) -> None:
        """Set motor brake current.

        :param current_amps: Brake current in amps, from -60.0 to 60.0 A.
        :return: None
        """
        # Clamp current to valid range
        current_amps = max(
            MOTOR_LIMITS.min_current_amps,
            min(MOTOR_LIMITS.max_current_amps, current_amps),
        )

        # Convert to motor protocol format
        current_int = int(current_amps / CAN_SCALE_FACTORS.current)

        # Pack as 4 bytes (int32, big-endian)
        data = struct.pack(">i", current_int)

        logger.info(f"Setting brake current: {current_amps:.2f} A")
        self._send_can_message(CANMotorCommand.CMD_BRAKE_CURRENT, data)

    def set_velocity(self, velocity_rpm: float) -> None:
        """Set motor velocity (electrical RPM).

        :param velocity_rpm: Velocity in electrical RPM, from -100000 to 100000.
        :return: None
        """
        # Clamp velocity to valid range
        velocity_rpm = max(
            MOTOR_LIMITS.min_velocity_electrical_rpm,
            min(MOTOR_LIMITS.max_velocity_electrical_rpm, velocity_rpm),
        )

        # Convert to motor protocol format: velocity / 10.0 -> int16
        velocity_int = int(velocity_rpm / CAN_SCALE_FACTORS.speed)

        # Pack as 4 bytes (int32, big-endian)
        data = struct.pack(">i", velocity_int)

        logger.info(f"Setting velocity: {velocity_rpm:.0f} RPM (electrical)")
        self._send_can_message(CANMotorCommand.CMD_VELOCITY_LOOP, data)

    def set_position(self, position_degrees: float) -> None:
        """Set motor position in degrees.

        :param position_degrees: Position in degrees.
        :return: None
        """
        # Convert to motor protocol format: position / 0.1 -> int16
        # Clamp to valid CAN int16 range (-3200 to 3200 degrees)
        max_degrees = CAN_LIMITS.max_position_int * CAN_SCALE_FACTORS.position
        min_degrees = CAN_LIMITS.min_position_int * CAN_SCALE_FACTORS.position
        position_degrees = max(min_degrees, min(max_degrees, position_degrees))

        position_int = int(position_degrees / CAN_SCALE_FACTORS.position)

        # Pack as 4 bytes (int32, big-endian)
        data = struct.pack(">i", position_int)

        logger.info(f"Setting position: {position_degrees:.2f}°")
        self._send_can_message(CANMotorCommand.CMD_POSITION_LOOP, data)

    def set_position_velocity(
        self, position_degrees: float, velocity_rpm: float, acceleration_rpm_s: float
    ) -> None:
        """Set motor position with velocity and acceleration limits.

        :param position_degrees: Target position in degrees.
        :param velocity_rpm: Maximum velocity in electrical RPM.
        :param acceleration_rpm_s: Maximum acceleration in electrical RPM/s.
        :return: None
        """
        # Convert position
        max_degrees = CAN_LIMITS.max_position_int * CAN_SCALE_FACTORS.position
        min_degrees = CAN_LIMITS.min_position_int * CAN_SCALE_FACTORS.position
        position_degrees = max(min_degrees, min(max_degrees, position_degrees))
        position_int = int(position_degrees / CAN_SCALE_FACTORS.position)

        # Convert velocity
        velocity_rpm = max(
            MOTOR_LIMITS.min_velocity_electrical_rpm,
            min(MOTOR_LIMITS.max_velocity_electrical_rpm, velocity_rpm),
        )
        velocity_int = int(velocity_rpm / CAN_SCALE_FACTORS.speed)

        # Convert acceleration
        acceleration_int = int(acceleration_rpm_s / CAN_SCALE_FACTORS.speed)

        # Pack as 8 bytes: position (4 bytes) + velocity (2 bytes) + accel (2 bytes)
        data = struct.pack(">ihh", position_int, velocity_int, acceleration_int)

        logger.info(
            f"Setting position: {position_degrees:.2f}° at {velocity_rpm:.0f} RPM "
            f"with {acceleration_rpm_s:.0f} RPM/s acceleration"
        )
        self._send_can_message(CANMotorCommand.CMD_POSITION_VELOCITY_LOOP, data)

    def set_mit_position(self, position_rad: float, kp: float, kd: float) -> None:
        """Set motor position using MIT mode with position gains.

        :param position_rad: Target position in radians.
        :param kp: Position proportional gain.
        :param kd: Position derivative gain.
        :return: None
        """
        # Pack gains and position
        kp_int = int(kp * 100)
        kd_int = int(kd * 100)
        pos_int = int(position_rad * 100)

        data = struct.pack(">Bhhh", 0x06, kp_int, kd_int, pos_int)

        logger.info(f"Setting MIT position: {position_rad:.2f} rad (Kp={kp}, Kd={kd})")
        self._send_can_message(CANMotorCommand.CMD_MIT_POSITION_LOOP, data)

    def set_mit_velocity(self, velocity_rad_s: float, kd: float) -> None:
        """Set motor velocity using MIT mode with velocity gain.

        :param velocity_rad_s: Target velocity in rad/s.
        :param kd: Velocity derivative gain.
        :return: None
        """
        kd_int = int(kd * 100)
        vel_int = int(velocity_rad_s * 100)

        data = struct.pack(">Bhhh", 0x06, kd_int, 0x7F, vel_int)

        logger.info(f"Setting MIT velocity: {velocity_rad_s:.2f} rad/s (Kd={kd})")
        self._send_can_message(CANMotorCommand.CMD_MIT_VELOCITY_LOOP, data)

    def set_mit_torque(self, current_amps: float) -> None:
        """Set motor torque (IQ current) using MIT mode.

        :param current_amps: Target current in amps.
        :return: None
        """
        # Clamp current
        current_amps = max(
            MOTOR_LIMITS.min_current_amps,
            min(MOTOR_LIMITS.max_current_amps, current_amps),
        )

        # Convert to int16
        current_int = int(current_amps / CAN_SCALE_FACTORS.current)

        data = struct.pack(">Bhhh", 0x00, 0x00, 0x7F, current_int)

        logger.info(f"Setting MIT torque: {current_amps:.2f} A")
        self._send_can_message(CANMotorCommand.CMD_MIT_TORQUE_LOOP, data)

    def get_status(self) -> MotorCANStatus | None:
        """Get all motor parameters.

        For CAN, this returns the last received feedback or waits for a new one.

        :return: MotorCANStatus object or None if no status available.
        """
        # Try to receive fresh feedback
        status = self.receive_feedback(timeout=CAN_DEFAULTS.response_timeout)
        if status:
            return status
        # Return last known status if available
        return self.last_status

    def get_position(self) -> float | None:
        """Get current motor position.

        :return: Position in degrees or None if not available.
        """
        status = self.get_status()
        if status:
            return status.position
        return None

    def stop(self) -> None:
        """Stop the motor by setting all control values to zero.

        :return: None
        """
        self.set_duty_cycle(0.0)
        self.set_current(0.0)
        self.set_velocity(0)
        logger.info("Motor stopped")

    def _estimate_movement_time(
        self, target_degrees: float, motor_speed_rpm: float
    ) -> float:
        """Estimate time needed to reach target position at given speed.

        Converts RPM to degrees/second and calculates travel time.
        This is a rough estimate - actual time may vary.

        :param target_degrees: Target position in degrees
        :param motor_speed_rpm: Motor speed in electrical RPM (absolute value used for calculation)
        :return: Estimated time in seconds
        """
        if motor_speed_rpm == 0:
            return 0.0

        # Convert RPM to degrees per second
        # RPM = revolutions per minute
        # degrees/sec = (RPM / 60) * 360
        degrees_per_second = abs(motor_speed_rpm) / 60.0 * 360.0

        # Calculate time needed
        estimated_time = abs(target_degrees) / degrees_per_second
        return estimated_time

    def move_to_position_with_speed(
        self,
        target_degrees: float,
        motor_speed_rpm: float,
        step_delay: float = 0.01,
    ) -> None:
        """Reach the target position through speed-controlled increments.

        :param target_degrees: Target position in degrees.
        :param motor_speed_rpm: Motor speed in electrical RPM (absolute value, direction determined by target).
        :param step_delay: Delay between steps in seconds.
        :return: None
        """
        # Get current position (assume we're tracking it)
        # For now, we'll send velocity command to move, then switch to position

        # Use velocity control to move at specified speed
        direction = 1 if target_degrees > 0 else -1
        self.set_velocity(motor_speed_rpm * direction)

        # Calculate approximate time needed
        estimated_time = self._estimate_movement_time(target_degrees, motor_speed_rpm)
        time.sleep(min(estimated_time, 5.0))  # Cap at 5 seconds

        # Switch to position hold
        self.set_position(target_degrees)
        logger.info(f"Reached position: {target_degrees}° at {motor_speed_rpm} RPM")

    def close(self) -> None:
        """Close CAN bus connection.

        :return: None
        """
        if self.mcp2515 is not None:
            try:
                self.mcp2515.close()
                logger.info("CAN motor controller closed")
            except Exception as e:
                logger.warning(f"Error closing CAN connection: {e}")
            finally:
                self.mcp2515 = None
                self.connected = False
                self.communicating = False

    def __enter__(self) -> "CubeMarsAK606CAN":
        """Context manager entry.

        :return: Self.
        """
        return self

    def __exit__(self, exc_type: Any, exc_val: Any, exc_tb: Any) -> None:
        """Context manager exit.

        :param exc_type: Exception type.
        :param exc_val: Exception value.
        :param exc_tb: Exception traceback.
        :return: None
        """
        self.close()

    def __del__(self) -> None:
        """Destructor - ensure connection is closed.

        :return: None
        """
        self.close()
