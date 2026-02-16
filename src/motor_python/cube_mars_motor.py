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
    TendonAction,
)
from motor_python.motor_status_parser import MotorStatusParser


class MotorCommand(IntEnum):
    """CubeMars UART command codes."""

    CMD_GET_STATUS = 0x45  # Get all motor parameters
    CMD_SET_CURRENT = 0x47  # Set current in amps (0A = release motor)
    CMD_SET_SPEED = 0x49  # Set velocity (primary exosuit control)
    CMD_SET_POSITION = 0x4A  # Set target position in degrees
    CMD_GET_POSITION = 0x4C  # Get current position (updates every 10ms)
    CMD_POSITION_ECHO = 0x57  # Position command echo response


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

        :param cmd: Command byte from MotorCommand enum.
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
        """Parse full status response (CMD_GET_STATUS).

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
        if len(response) < FRAME_BYTES.min_response_length:
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
                if cmd == MotorCommand.CMD_GET_STATUS:
                    self._parse_full_status(payload)

                elif cmd in {
                    MotorCommand.CMD_GET_POSITION,
                    MotorCommand.CMD_POSITION_ECHO,
                }:
                    if len(payload) >= 4:
                        position = struct.unpack(">f", payload[0:4])[0]
                        logger.info(f"  Position: {position:.2f} deg")

            except Exception as e:
                logger.warning(f"Error parsing motor response: {e}")
                logger.info(f"  Raw payload: {payload.hex().upper()}")

        logger.info("=" * 50)
        return True

    def set_position(self, position_degrees: float) -> None:
        """Set motor position in degrees.

        Limited to +/-360 degrees (one full rotation) for exosuit joint safety.

        :param position_degrees: Target position in degrees (-360.0 to 360.0)
        :return: None
        :raises ValueError: If position exceeds safe limits
        """
        if (
            position_degrees > MOTOR_LIMITS.max_position_degrees
            or position_degrees < MOTOR_LIMITS.min_position_degrees
        ):
            logger.warning(
                f"Position {position_degrees:.2f} deg exceeds limits "
                f"[{MOTOR_LIMITS.min_position_degrees:.2f} deg, "
                f"{MOTOR_LIMITS.max_position_degrees:.2f} deg]. "
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

    def get_position(self) -> bytes:
        """Get current motor position via CMD_GET_POSITION.

        Returns current position every 10ms.
        Lightweight query for position feedback only.

        :return: Raw response bytes from motor containing position
        """
        frame = self._build_frame(MotorCommand.CMD_GET_POSITION, b"")
        response = self._send_frame(frame)
        return response

    def _estimate_movement_time(
        self, target_degrees: float, motor_speed_erpm: int
    ) -> float:
        """Estimate time needed to reach target position at given speed.

        Converts ERPM to degrees/second and calculates travel time.
        This is a rough estimate -- actual time may vary.

        :param target_degrees: Target position in degrees
        :param motor_speed_erpm: Motor speed in electrical RPM (absolute value used for calculation)
        :return: Estimated time in seconds
        """
        if motor_speed_erpm == 0:
            return 0.0

        # ERPM -> degrees/sec = (ERPM / 60) * 360
        degrees_per_second = abs(motor_speed_erpm) / 60.0 * 360.0
        estimated_time = abs(target_degrees) / degrees_per_second
        return estimated_time

    def move_to_position_with_speed(
        self,
        target_degrees: float,
        motor_speed_erpm: int,
        step_delay: float = MOTOR_DEFAULTS.step_delay,
    ) -> None:
        """Reach a target position using velocity control then hold with position.

        Uses velocity to move the motor toward the target, then switches to
        position hold once the estimated travel time elapses.

        :param target_degrees: Target position in degrees (-360 to 360).
        :param motor_speed_erpm: Motor speed in ERPM (absolute, direction auto).
        :param step_delay: Delay between steps in seconds.
        :return: None
        """
        # Use velocity control to move at specified speed
        direction = 1 if target_degrees > 0 else -1
        self.set_velocity(
            velocity_erpm=motor_speed_erpm * direction, allow_low_speed=True
        )

        # Calculate approximate time needed
        estimated_time = self._estimate_movement_time(target_degrees, motor_speed_erpm)
        time.sleep(min(estimated_time, MOTOR_LIMITS.max_movement_time))

        # Switch to position hold
        self.set_position(target_degrees)
        logger.info(
            f"Reached position: {target_degrees} deg at {motor_speed_erpm} ERPM"
        )

    def _soft_start(self, direction: int) -> None:
        """Pre-spin motor with gentle current to pass the noisy low-speed zone.

        The firmware's velocity PID has a fixed 60k ERPM/s² acceleration that
        causes current oscillations and high-pitch noise at low speeds (0-5000
        ERPM).  By first sending a moderate current command, the motor
        accelerates gently under direct torque control (no velocity PID) until
        it is past the dangerous zone, then the caller switches to velocity
        mode.

        :param direction: 1 for forward, -1 for reverse
        :return: None
        """
        current_ma = MOTOR_LIMITS.soft_start_current_ma * direction
        payload = struct.pack(">i", current_ma)
        frame = self._build_frame(MotorCommand.CMD_SET_CURRENT, payload)
        self._send_frame(frame)
        time.sleep(MOTOR_LIMITS.soft_start_duration)

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

        # Velocity 0 means stop -- use current=0 to release windings cleanly
        # instead of velocity PID which decelerates through the noisy zone
        if velocity_erpm_int == 0:
            self.stop()
            return

        # Block dangerously low speeds unless explicitly allowed
        if not allow_low_speed:
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

        # Soft-start: pre-spin with current to avoid noisy low-speed zone
        direction = 1 if velocity_erpm > 0 else -1
        self._soft_start(direction)

        payload = struct.pack(">i", velocity_erpm)
        frame = self._build_frame(MotorCommand.CMD_SET_SPEED, payload)
        self._send_frame(frame)

    def get_status(self) -> bytes:
        """Get all motor parameters.

        :return: Raw status bytes from motor.
        """
        # CMD_GET_STATUS requires no payload - it returns everything
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
        action: TendonAction,
        velocity_erpm: int = MOTOR_LIMITS.default_tendon_velocity_erpm,
    ) -> None:
        """Control exosuit tendon using safe velocity commands.

        :param action: TendonAction enum (PULL, RELEASE, or STOP)
        :param velocity_erpm: Velocity in ERPM (default: 10000, min: 5000)
        :return: None
        :raises ValueError: If action is invalid
        """
        if action == TendonAction.PULL:
            logger.info(f"Pulling tendon at {velocity_erpm} ERPM")
            self.set_velocity(velocity_erpm=abs(velocity_erpm))
        elif action == TendonAction.RELEASE:
            logger.info(f"Releasing tendon at {velocity_erpm} ERPM")
            self.set_velocity(velocity_erpm=-abs(velocity_erpm))
        elif action == TendonAction.STOP:
            logger.info("Stopping tendon motion")
            self.stop()
        else:
            raise ValueError(
                f"Invalid action {action}. Use TendonAction.PULL, TendonAction.RELEASE, or TendonAction.STOP"
            )

    def stop(self) -> None:
        """Stop the motor by setting current to zero (release windings).

        Sends current=0 via CMD_SET_CURRENT which puts the motor controller
        into current mode with a 0A target. This releases the motor windings
        completely -- no velocity PID deceleration through the noisy low-speed
        zone, no PWM switching. The rotor coasts to a mechanical stop.

        :return: None
        """
        # current=0A -> payload = int(0 * 1000) = 0, packed big-endian int32
        payload = struct.pack(">i", 0)
        frame = self._build_frame(MotorCommand.CMD_SET_CURRENT, payload)
        self._send_frame(frame)
        time.sleep(0.1)
        # Send a second time in case UART dropped the first frame
        self._send_frame(frame)
        time.sleep(0.2)
        logger.info("Motor stopped (current=0, windings released)")

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
