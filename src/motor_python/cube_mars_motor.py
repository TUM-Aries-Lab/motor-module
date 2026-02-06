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
)
from motor_python.motor_status_parser import MotorStatusParser


class MotorCommand(IntEnum):
    """CubeMars UART command codes."""

    CMD_GET_STATUS = 0x45  # Get all motor parameters
    CMD_SET_SPEED = 0x49  # Set velocity (primary exosuit control)


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
            if cmd_byte == MotorCommand.CMD_GET_STATUS:
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

            except Exception as e:
                logger.warning(f"Error parsing motor response: {e}")
                logger.info(f"  Raw payload: {payload.hex().upper()}")

        logger.info("=" * 50)
        return True

    def set_velocity(self, velocity_erpm: int, allow_low_speed: bool = False) -> None:
        """Set motor velocity in electrical RPM.

        Low speeds (<5000 ERPM) with high firmware acceleration cause current
        oscillations and audible noise. Use medium-to-high speeds for exosuit.

        :param velocity_erpm: Target velocity in ERPM (safe range: 5000-100000)
        :param allow_low_speed: Bypass the 5000 ERPM safety floor (default: False)
        :return: None
        :raises ValueError: If velocity below safe threshold and allow_low_speed=False
        """
        velocity_erpm_int = int(velocity_erpm)

        # Block dangerously low speeds unless explicitly allowed or stopping
        if not allow_low_speed and velocity_erpm_int != 0:
            if 0 < abs(velocity_erpm_int) < MOTOR_LIMITS.min_safe_velocity_erpm:
                raise ValueError(
                    f"Velocity {velocity_erpm_int} ERPM below safe threshold "
                    f"({MOTOR_LIMITS.min_safe_velocity_erpm} ERPM min). "
                    f"Use allow_low_speed=True to bypass."
                )

        # Clamp to absolute limits
        velocity_erpm = np.clip(
            velocity_erpm_int,
            MOTOR_LIMITS.min_velocity_electrical_rpm,
            MOTOR_LIMITS.max_velocity_electrical_rpm,
        )
        if velocity_erpm != velocity_erpm_int:
            logger.warning(
                f"Velocity {velocity_erpm_int} ERPM clamped to {velocity_erpm} ERPM"
            )
        payload = struct.pack(">i", velocity_erpm)
        frame = self._build_frame(MotorCommand.CMD_SET_SPEED, payload)
        self._send_frame(frame)

    def get_status(self) -> bytes:
        """Get all motor parameters.

        :return: Raw status bytes from motor.
        """
        # Command 0x45 requires no payload - it returns everything
        frame = self._build_frame(MotorCommand.CMD_GET_STATUS, b"")
        status = self._send_frame(frame)
        return status

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

    def control_exosuit_tendon(
        self,
        action: str,
        velocity_erpm: int = 10000,
    ) -> None:
        """Control exosuit tendon using safe velocity commands.

        :param action: Action - 'pull', 'release', 'stop'
        :param velocity_erpm: Velocity in ERPM (default: 10000, min: 5000)
        :return: None
        :raises ValueError: If action is invalid
        """
        if action == "pull":
            logger.info(f"Pulling tendon at {velocity_erpm} ERPM")
            self.set_velocity(velocity_erpm=abs(velocity_erpm))
        elif action == "release":
            logger.info(f"Releasing tendon at {velocity_erpm} ERPM")
            self.set_velocity(velocity_erpm=-abs(velocity_erpm))
        elif action == "stop":
            logger.info("Stopping tendon motion")
            self.set_velocity(velocity_erpm=0)
        else:
            raise ValueError(
                f"Invalid action '{action}'. Use: 'pull', 'release', or 'stop'"
            )

    def stop(self) -> None:
        """Stop the motor.

        Sends velocity=0 twice with a delay to let the motor fully decelerate
        before the firmware's velocity control loop settles.

        :return: None
        """
        self.set_velocity(0)
        time.sleep(0.3)
        self.set_velocity(0)
        time.sleep(0.3)
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
