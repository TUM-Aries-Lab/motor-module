"""AK60-6 Motor Control Class - CubeMars UART Protocol."""

import struct
import time
from enum import IntEnum
from pathlib import Path

import numpy as np
import serial
from loguru import logger

from motor_python.definitions import (
    CRC16_TAB,
    CRC_CONSTANTS,
    FRAME_BYTES,
    MOTOR_DEFAULTS,
    MOTOR_LIMITS,
    SCALE_FACTORS,
)
from motor_python.motor_status_parser import MotorStatusParser


class MotorCommand(IntEnum):
    """CubeMars UART command codes."""

    CMD_GET_PARAMS = 0x13  # Get specific parameters based on bit field
    CMD_GET_STATUS = 0x45  # Get all motor parameters
    CMD_SET_DUTY = 0x46
    CMD_SET_CURRENT = 0x47
    CMD_SET_SPEED = 0x49
    CMD_SET_POSITION = 0x4A
    CMD_GET_POSITION = 0x4C  # Get current position (updates every 10ms)


class CubeMarsAK606v3:
    """AK60-6 Motor Controller for CubeMars V3 UART Protocol."""

    def __init__(
        self,
        port: Path | str = MOTOR_DEFAULTS.port,
        baudrate: int = MOTOR_DEFAULTS.baudrate,
    ) -> None:
        """Initialize motor connection.

        :param port: Serial port path (default: MOTOR_DEFAULTS.port).
        :param baudrate: Communication baudrate (default: MOTOR_DEFAULTS.baudrate).
        :return: None
        """
        self.port = str(port)  # Convert Path to str for serial library
        self.baudrate = baudrate
        self.serial: serial.Serial | None = None
        self.status_parser = MotorStatusParser()
        self.connected = False
        self.communicating = False
        self._consecutive_no_response = 0
        self._consecutive_invalid_response = 0
        self._max_no_response = MOTOR_DEFAULTS.max_no_response_attempts
        self._connect()

    def _connect(self) -> None:
        """Establish serial connection to motor.

        :return: None
        """
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1,
                rtscts=False,
                dsrdtr=False,
            )
            time.sleep(0.1)  # Allow connection to stabilize
            self.connected = True
            logger.info(f"Connected to motor on {self.port} at {self.baudrate} baud")
        except serial.SerialException as e:
            logger.warning(f"Failed to connect to motor on {self.port}: {e}")
            self.connected = False
        except Exception as e:
            logger.warning(f"Unexpected error connecting to motor: {e}")
            self.connected = False

    def _crc16(self, data: bytes) -> int:
        """Calculate CRC16-CCITT checksum.

        :param data: Bytes to calculate checksum over.
        :return: 16-bit CRC checksum.
        """
        checksum = CRC_CONSTANTS.initial_value
        for byte in data:
            # Extract high byte of checksum and XOR with current byte
            high_byte = (checksum >> CRC_CONSTANTS.shift_bits) & CRC_CONSTANTS.byte_mask
            table_index = high_byte ^ byte
            # Lookup table value and combine with shifted checksum
            table_value = CRC16_TAB[table_index]
            shifted_checksum = (
                checksum << CRC_CONSTANTS.shift_bits
            ) & CRC_CONSTANTS.word_mask
            checksum = table_value ^ shifted_checksum
        return checksum

    def _build_frame(self, cmd: int, payload: bytes) -> bytes:
        """Build CubeMars UART frame with proper structure.

        Frame structure: AA | DataLength | CMD | Payload | CRC_H | CRC_L | BB

        :param cmd: Command byte (0x46, 0x47, 0x49, 0x4A).
        :param payload: Command payload data.
        :return: Complete frame ready to send.
        """
        data_frame = bytes([cmd]) + payload
        crc = self._crc16(data_frame)
        frame = bytes(
            [
                FRAME_BYTES.start,
                len(data_frame),
                *data_frame,
                crc >> 8,
                crc & 0xFF,
                FRAME_BYTES.end,
            ]
        )
        return frame

    def _get_command_from_frame(self, frame: bytes) -> int:
        """Extract command byte from frame.

        :param frame: Frame bytes.
        :return: Command byte, or 0 if frame is too short.
        """
        # Frame structure: AA | DataLength | CMD | Payload | CRC_H | CRC_L | BB
        # Command byte is at index 2
        return frame[2] if len(frame) > 2 else 0

    def _send_frame(self, frame: bytes) -> bytes:
        """Send frame to motor over UART and read response.

        :param frame: Complete frame to send.
        :return: Response bytes from motor (if any).
        """
        if not self.connected or self.serial is None or not self.serial.is_open:
            logger.debug("Motor not connected - skipping message send")
            return b""

        self.serial.write(frame)
        logger.debug(f"TX: {' '.join(f'{b:02X}' for b in frame)}")

        # Wait for response - some commands need more time
        time.sleep(0.1)

        # Read any available response
        response = b""
        bytes_waiting = self.serial.in_waiting
        logger.debug(f"Bytes waiting in buffer: {bytes_waiting}")

        if bytes_waiting > 0:
            response = self.serial.read(bytes_waiting)
            if response:
                logger.debug(f"RX: {' '.join(f'{b:02X}' for b in response)}")
                # Check if response is valid
                is_valid = self._parse_motor_response(response)
                if is_valid:
                    # Reset failure counters on successful communication
                    self._consecutive_no_response = 0
                    self._consecutive_invalid_response = 0
                    self.communicating = True
                else:
                    # Track consecutive invalid responses
                    self._consecutive_invalid_response += 1
                    if self._consecutive_invalid_response >= self._max_no_response:
                        logger.warning(
                            f"Motor sending invalid responses after {self._consecutive_invalid_response} attempts - "
                            "hardware may be powered off or cables disconnected"
                        )
                        self.communicating = False
                    # Clear response since it's invalid
                    response = b""
        else:
            logger.debug("No response from motor")
            # Track consecutive failures for status queries
            cmd_byte = self._get_command_from_frame(frame)
            if cmd_byte in (MotorCommand.CMD_GET_STATUS, MotorCommand.CMD_GET_POSITION):
                self._consecutive_no_response += 1
                if (
                    self._consecutive_no_response >= self._max_no_response
                    and self.communicating
                ):
                    logger.warning(
                        f"Motor not responding after {self._consecutive_no_response} attempts - "
                        "hardware may be disconnected or powered off"
                    )
                    self.communicating = False

        return response

    def _parse_full_status(self, payload: bytes) -> None:
        """Parse full status response (command 0x45).

        :param payload: Response payload bytes.
        :return: None
        """
        status = self.status_parser.parse_full_status(payload)
        if status:
            self.status_parser.log_motor_status(status)

    def _parse_motor_response(self, response: bytes) -> bool:
        """Parse and display motor response data.

        :param response: Raw response bytes from motor.
        :return: True if response was valid, False otherwise
        """
        if len(response) < 10:  # Minimum valid response length
            return False

        # Check for valid frame structure (AA ... BB)
        if response[0] != FRAME_BYTES.start or response[-1] != FRAME_BYTES.end:
            logger.warning("Invalid response frame structure")
            return False

        # Extract basic frame info
        data_length = response[1]
        cmd = response[2] if len(response) > 2 else 0

        logger.info("=" * 50)
        logger.info("MOTOR RESPONSE:")
        logger.info(f"  Command: 0x{cmd:02X}")
        logger.info(f"  Data Length: {data_length}")

        # Parse payload according to command type
        if len(response) >= 6:
            # Payload starts at byte 3, ends before CRC (last 3 bytes)
            payload = response[3:-3]

            try:
                if cmd == 0x45:  # Full status response
                    self._parse_full_status(payload)
                elif cmd in {0x4C, 0x57}:  # Position response (0x57 is echo of 0x4C)
                    if len(payload) >= 4:
                        position = struct.unpack(">f", payload[0:4])[0]
                        logger.info(f"  Position: {position:.2f}°")

            except Exception as e:
                logger.warning(f"Error parsing motor response: {e}")
                logger.info(f"  Raw payload: {payload.hex().upper()}")

        logger.info("=" * 50)
        return True

    def set_position(self, position_degrees: float) -> None:
        """Set motor position in degrees.

        Limited to ±360° for exosuit joint safety.

        :param position_degrees: Target position in degrees (±360° = ±1 rotation)
        :return: None
        """
        if (
            position_degrees > MOTOR_LIMITS.max_position_degrees
            or position_degrees < MOTOR_LIMITS.min_position_degrees
        ):
            logger.warning(
                f"Position {position_degrees:.2f}° exceeds limits "
                f"[{MOTOR_LIMITS.min_position_degrees:.2f}°, {MOTOR_LIMITS.max_position_degrees:.2f}°]. "
                f"Clamping to safe range."
            )
        position_degrees = np.clip(
            position_degrees,
            MOTOR_LIMITS.min_position_degrees,
            MOTOR_LIMITS.max_position_degrees,
        )
        value = int(position_degrees * SCALE_FACTORS.position)
        payload = struct.pack(">i", value)
        frame = self._build_frame(MotorCommand.CMD_SET_POSITION, payload)
        self._send_frame(frame)

    def set_velocity(self, velocity_erpm: int) -> None:
        """Set motor velocity in electrical RPM.

        :param velocity_erpm: Target velocity in ERPM
        :return: None
        """
        velocity_erpm_int = int(velocity_erpm)
        if (
            velocity_erpm_int > MOTOR_LIMITS.max_velocity_electrical_rpm
            or velocity_erpm_int < MOTOR_LIMITS.min_velocity_electrical_rpm
        ):
            logger.warning(
                f"Velocity {velocity_erpm_int} ERPM exceeds limits "
                f"[{MOTOR_LIMITS.min_velocity_electrical_rpm}, {MOTOR_LIMITS.max_velocity_electrical_rpm}]. "
                f"Clamping to safe range."
            )
        velocity_erpm = np.clip(
            velocity_erpm_int,
            MOTOR_LIMITS.min_velocity_electrical_rpm,
            MOTOR_LIMITS.max_velocity_electrical_rpm,
        )
        payload = struct.pack(">i", velocity_erpm)
        frame = self._build_frame(MotorCommand.CMD_SET_SPEED, payload)
        self._send_frame(frame)

    def _estimate_movement_time(
        self, target_degrees: float, motor_speed_erpm: int
    ) -> float:
        """Estimate time needed to reach target position at given speed.

        Converts ERPM to degrees/second and calculates travel time.
        This is a rough estimate - actual time may vary.

        :param target_degrees: Target position in degrees
        :param motor_speed_erpm: Motor speed in electrical RPM (absolute value used for calculation)
        :return: Estimated time in seconds
        """
        if motor_speed_erpm == 0:
            return 0.0

        # Convert ERPM to degrees per second
        # ERPM = electrical revolutions per minute
        # degrees/sec = (ERPM / 60) * 360
        degrees_per_second = abs(motor_speed_erpm) / 60.0 * 360.0

        # Calculate time needed
        estimated_time = abs(target_degrees) / degrees_per_second
        return estimated_time

    def move_to_position_with_speed(
        self,
        target_degrees: float,
        motor_speed_erpm: int,
        step_delay: float = MOTOR_DEFAULTS.step_delay,
    ) -> None:
        """Reach the target position through speed-controlled increments.

        :param target_degrees: Target position in degrees.
        :param motor_speed_erpm: Motor speed in electrical RPM (absolute value, direction determined by target).
        :param step_delay: Delay between steps in seconds.
        :return: None
        """
        # Get current position (assume we're tracking it)
        # For now, we'll send velocity command to move, then switch to position

        # Use velocity control to move at specified speed
        direction = 1 if target_degrees > 0 else -1
        self.set_velocity(motor_speed_erpm * direction)

        # Calculate approximate time needed
        estimated_time = self._estimate_movement_time(target_degrees, motor_speed_erpm)
        time.sleep(min(estimated_time, 5.0))  # Cap at 5 seconds

        # Switch to position hold
        self.set_position(target_degrees)
        logger.info(f"Reached position: {target_degrees}° at {motor_speed_erpm} ERPM")

    def set_duty_cycle(self, duty_cycle_percent: float) -> None:
        """Set motor PWM duty cycle percentage.

        :param duty_cycle_percent: Duty cycle value (-1.0 to 1.0, where 1.0 = 100%)
        :return: None
        """
        # Limit to 95% to prevent saturation
        if (
            duty_cycle_percent > MOTOR_LIMITS.max_duty_cycle
            or duty_cycle_percent < MOTOR_LIMITS.min_duty_cycle
        ):
            logger.warning(
                f"Duty cycle {duty_cycle_percent:.2f} exceeds limits "
                f"[{MOTOR_LIMITS.min_duty_cycle:.2f}, {MOTOR_LIMITS.max_duty_cycle:.2f}]. "
                f"Clamping to safe range."
            )
        duty_cycle_percent = np.clip(
            duty_cycle_percent,
            MOTOR_LIMITS.min_duty_cycle,
            MOTOR_LIMITS.max_duty_cycle,
        )
        value = int(duty_cycle_percent * 100000.0)
        payload = struct.pack(">i", value)
        frame = self._build_frame(MotorCommand.CMD_SET_DUTY, payload)
        self._send_frame(frame)

    def set_current(self, current_amps: float) -> None:
        """Set motor current in amperes.

        :param current_amps: Current in Amps
        :return: None
        """
        if (
            current_amps > MOTOR_LIMITS.max_current_amps
            or current_amps < MOTOR_LIMITS.min_current_amps
        ):
            logger.warning(
                f"Current {current_amps:.2f}A exceeds limits "
                f"[{MOTOR_LIMITS.min_current_amps:.2f}A, {MOTOR_LIMITS.max_current_amps:.2f}A]. "
                f"Clamping to safe range."
            )
        current_amps = np.clip(
            current_amps,
            MOTOR_LIMITS.min_current_amps,
            MOTOR_LIMITS.max_current_amps,
        )
        value = int(current_amps * 1000.0)
        payload = struct.pack(">i", value)
        frame = self._build_frame(MotorCommand.CMD_SET_CURRENT, payload)
        self._send_frame(frame)

    def get_status(self) -> bytes:
        """Get all motor parameters.

        :return: Raw status bytes from motor.
        """
        # Command 0x45 requires no payload - it returns everything
        frame = self._build_frame(MotorCommand.CMD_GET_STATUS, b"")
        status = self._send_frame(frame)
        return status

    def get_position(self) -> bytes:
        """Get current motor position via command 0x4C.

        Command 0x4C returns current position every 10ms.
        Lightweight query for position feedback only.

        :return: Raw response bytes from motor containing position
        """
        # Command 0x4C with no payload
        frame = self._build_frame(MotorCommand.CMD_GET_POSITION, b"")
        response = self._send_frame(frame)
        return response

    def check_communication(self) -> bool:
        """Verify motor is responding to commands.

        :return: True if motor responds, False otherwise
        """
        if not self.connected:
            return False

        # Try to get status MOTOR_DEFAULTS.max_communication_attempts times
        for _attempt in range(MOTOR_DEFAULTS.max_communication_attempts):
            status = self.get_status()
            if status and len(status) > 0:
                self.communicating = True
                self._consecutive_no_response = 0
                logger.info("Motor communication verified")
                return True
            time.sleep(MOTOR_DEFAULTS.communication_retry_delay)

        logger.warning("Motor not responding to status queries")
        self.communicating = False
        return False

    def stop(self) -> None:
        """Stop the motor by setting all control values to zero.

        :return: None
        """
        self.set_duty_cycle(0.0)
        self.set_current(0.0)
        self.set_velocity(0)
        logger.info("Motor stopped")

    def close(self) -> None:
        """Close serial connection to motor.

        :return: None
        """
        if self.serial and self.serial.is_open:
            self.stop()
            self.serial.close()
            logger.info("Motor connection closed")

    def __enter__(self) -> "CubeMarsAK606v3":
        """Context manager entry.

        :return: Self instance for use in with statement.
        """
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        """Context manager exit.

        :param exc_type: Exception type if an exception occurred.
        :param exc_val: Exception value if an exception occurred.
        :param exc_tb: Exception traceback if an exception occurred.
        :return: None
        """
        self.close()
