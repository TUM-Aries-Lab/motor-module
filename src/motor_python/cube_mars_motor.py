"""AK60-6 Motor Control Class - CubeMars UART Protocol."""

import struct
import time
from enum import IntEnum

import serial
from loguru import logger

from motor_python.definitions import (
    CRC16_TAB,
    CRC_BYTE_MASK,
    CRC_INITIAL_VALUE,
    CRC_SHIFT_BITS,
    CRC_WORD_MASK,
    DEFAULT_MOTOR_BAUDRATE,
    DEFAULT_MOTOR_PORT,
    DEFAULT_STEP_DELAY,
    FRAME_END_BYTE,
    FRAME_START_BYTE,
    MAX_CURRENT_AMPS,
    MAX_DUTY_CYCLE,
    MAX_POSITION_DEGREES,
    MAX_VELOCITY_ERPM,
    MIN_CURRENT_AMPS,
    MIN_DUTY_CYCLE,
    MIN_POSITION_DEGREES,
    MIN_VELOCITY_ERPM,
    POSITION_SCALE_FACTOR,
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
        self, port: str = DEFAULT_MOTOR_PORT, baudrate: int = DEFAULT_MOTOR_BAUDRATE
    ) -> None:
        """Initialize motor connection.

        :param port: Serial port path (default: DEFAULT_MOTOR_PORT).
        :param baudrate: Communication baudrate (default: DEFAULT_MOTOR_BAUDRATE).
        :return: None
        """
        self.port = port
        self.baudrate = baudrate
        self.serial: serial.Serial | None = None
        self.status_parser = MotorStatusParser()
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
            logger.info(f"Connected to motor on {self.port} at {self.baudrate} baud")
        except serial.SerialException as e:
            logger.error(f"Failed to connect to motor on {self.port}: {e}")
            raise
        except Exception as e:
            logger.error(f"Unexpected error connecting to motor: {e}")
            raise

    def _crc16(self, data: bytes) -> int:
        """Calculate CRC16-CCITT checksum.

        :param data: Bytes to calculate checksum over.
        :return: 16-bit CRC checksum.
        """
        checksum = CRC_INITIAL_VALUE
        for byte in data:
            # Extract high byte of checksum and XOR with current byte
            high_byte = (checksum >> CRC_SHIFT_BITS) & CRC_BYTE_MASK
            table_index = high_byte ^ byte
            # Lookup table value and combine with shifted checksum
            table_value = CRC16_TAB[table_index]
            shifted_checksum = (checksum << CRC_SHIFT_BITS) & CRC_WORD_MASK
            checksum = table_value ^ shifted_checksum
        return checksum

    def _build_message(self, cmd: int, payload: bytes) -> bytes:
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
                FRAME_START_BYTE,
                len(data_frame),
                *data_frame,
                crc >> 8,
                crc & 0xFF,
                FRAME_END_BYTE,
            ]
        )
        return frame

    def _send_message(self, frame: bytes) -> bytes:
        """Send message to motor over UART and read response.

        :param frame: Complete frame to send.
        :return: Response bytes from motor (if any).
        """
        if self.serial is None or not self.serial.is_open:
            raise RuntimeError("Motor serial connection not open")

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
                self._parse_motor_response(response)
        else:
            logger.debug("No response from motor (expected for control commands)")

        return response

    def _parse_full_status(self, payload: bytes) -> None:
        """Parse full status response (command 0x45).

        :param payload: Response payload bytes.
        :return: None
        """
        status = self.status_parser.parse_full_status(payload)
        if status:
            self.status_parser.log_motor_status(status)

    def _parse_motor_response(self, response: bytes) -> None:
        """Parse and display motor response data.

        :param response: Raw response bytes from motor.
        :return: None
        """
        if len(response) < 10:  # Minimum valid response length
            return

        # Check for valid frame structure (AA ... BB)
        if response[0] != FRAME_START_BYTE or response[-1] != FRAME_END_BYTE:
            logger.warning("Invalid response frame structure")
            return

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

    def set_position(self, position_degrees: float) -> None:
        """Set motor position in degrees.

        Limited to ±360° for exosuit joint safety.

        :param position_degrees: Target position in degrees (±360° = ±1 rotation)
        :return: None
        """
        if (
            position_degrees > MAX_POSITION_DEGREES
            or position_degrees < MIN_POSITION_DEGREES
        ):
            logger.warning(
                f"Position {position_degrees:.2f}° exceeds limits "
                f"[{MIN_POSITION_DEGREES:.2f}°, {MAX_POSITION_DEGREES:.2f}°]. "
                f"Clamping to safe range."
            )
        position_degrees = max(
            min(position_degrees, MAX_POSITION_DEGREES), MIN_POSITION_DEGREES
        )
        value = int(position_degrees * POSITION_SCALE_FACTOR)
        payload = struct.pack(">i", value)
        frame = self._build_message(MotorCommand.CMD_SET_POSITION, payload)
        self._send_message(frame)

    def set_velocity(self, velocity_erpm: int) -> None:
        """Set motor velocity in electrical RPM.

        :param velocity_erpm: Target velocity in ERPM
        :return: None
        """
        velocity_erpm_int = int(velocity_erpm)
        if (
            velocity_erpm_int > MAX_VELOCITY_ERPM
            or velocity_erpm_int < MIN_VELOCITY_ERPM
        ):
            logger.warning(
                f"Velocity {velocity_erpm_int} ERPM exceeds limits "
                f"[{MIN_VELOCITY_ERPM}, {MAX_VELOCITY_ERPM}]. "
                f"Clamping to safe range."
            )
        velocity_erpm = max(
            min(velocity_erpm_int, MAX_VELOCITY_ERPM), MIN_VELOCITY_ERPM
        )
        payload = struct.pack(">i", velocity_erpm)
        frame = self._build_message(MotorCommand.CMD_SET_SPEED, payload)
        self._send_message(frame)

    def _estimate_movement_time(self, target_degrees: float, speed_erpm: int) -> float:
        """Estimate time needed to reach target position at given speed.

        Converts ERPM to degrees/second and calculates travel time.
        This is a rough estimate - actual time may vary.

        :param target_degrees: Target position in degrees
        :param speed_erpm: Motor speed in electrical RPM
        :return: Estimated time in seconds
        """
        if speed_erpm == 0:
            return 0.0

        # Convert ERPM to degrees per second
        # ERPM = electrical revolutions per minute
        # degrees/sec = (ERPM / 60) * 360
        degrees_per_second = abs(speed_erpm) / 60.0 * 360.0

        # Calculate time needed
        estimated_time = abs(target_degrees) / degrees_per_second
        return estimated_time

    def move_to_position_with_speed(
        self,
        target_degrees: float,
        speed_erpm: int,
        step_delay: float = DEFAULT_STEP_DELAY,
    ) -> None:
        """Reach the target position through speed-controlled increments.

        :param target_degrees: Target position in degrees.
        :param speed_erpm: Motor speed in electrical RPM.
        :param step_delay: Delay between steps in seconds.
        :return: None
        """
        # Get current position (assume we're tracking it)
        # For now, we'll send velocity command to move, then switch to position

        # Use velocity control to move at specified speed
        direction = 1 if target_degrees > 0 else -1
        self.set_velocity(speed_erpm * direction)

        # Calculate approximate time needed
        estimated_time = self._estimate_movement_time(target_degrees, speed_erpm)
        time.sleep(min(estimated_time, 5.0))  # Cap at 5 seconds

        # Switch to position hold
        self.set_position(target_degrees)
        logger.info(f"Reached position: {target_degrees}° at {speed_erpm} ERPM")

    def set_duty_cycle(self, duty_cycle_percent: float) -> None:
        """Set motor PWM duty cycle percentage.

        :param duty_cycle_percent: Duty cycle value (-1.0 to 1.0, where 1.0 = 100%)
        :return: None
        """
        # Limit to 95% to prevent saturation
        if duty_cycle_percent > MAX_DUTY_CYCLE or duty_cycle_percent < MIN_DUTY_CYCLE:
            logger.warning(
                f"Duty cycle {duty_cycle_percent:.2f} exceeds limits "
                f"[{MIN_DUTY_CYCLE:.2f}, {MAX_DUTY_CYCLE:.2f}]. "
                f"Clamping to safe range."
            )
        duty_cycle_percent = max(
            min(duty_cycle_percent, MAX_DUTY_CYCLE), MIN_DUTY_CYCLE
        )
        value = int(duty_cycle_percent * 100000.0)
        payload = struct.pack(">i", value)
        frame = self._build_message(MotorCommand.CMD_SET_DUTY, payload)
        self._send_message(frame)

    def set_current(self, current_amps: float) -> None:
        """Set motor current in amperes.

        :param current_amps: Current in Amps
        :return: None
        """
        if current_amps > MAX_CURRENT_AMPS or current_amps < MIN_CURRENT_AMPS:
            logger.warning(
                f"Current {current_amps:.2f}A exceeds limits "
                f"[{MIN_CURRENT_AMPS:.2f}A, {MAX_CURRENT_AMPS:.2f}A]. "
                f"Clamping to safe range."
            )
        current_amps = max(min(current_amps, MAX_CURRENT_AMPS), MIN_CURRENT_AMPS)
        value = int(current_amps * 1000.0)
        payload = struct.pack(">i", value)
        frame = self._build_message(MotorCommand.CMD_SET_CURRENT, payload)
        self._send_message(frame)

    def get_status(self) -> bytes:
        """Get all motor parameters.

        :return: Raw response bytes from motor.
        """
        # Command 0x45 requires no payload - it returns everything
        frame = self._build_message(MotorCommand.CMD_GET_STATUS, b"")
        response = self._send_message(frame)
        return response

    def get_position(self) -> bytes:
        """Get current motor position via command 0x4C.

        Command 0x4C returns current position every 10ms.
        Lightweight query for position feedback only.

        :return: Raw response bytes from motor containing position
        """
        # Command 0x4C with no payload
        frame = self._build_message(MotorCommand.CMD_GET_POSITION, b"")
        response = self._send_message(frame)
        return response

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
