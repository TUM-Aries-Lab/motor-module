"""AK60-6 Motor Control Class - CubeMars UART Protocol."""

import struct
import time
from enum import IntEnum
from pathlib import Path

import serial
from loguru import logger

from motor_python.base_motor import BaseMotor, MotorState
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

    CMD_GET_STATUS = 0x45  # Get all motor parameters
    CMD_SET_CURRENT = 0x47  # Set current in amps (0A = release motor)
    CMD_SET_VELOCITY = 0x49  # Set velocity (primary exosuit control, can be negative)
    CMD_SET_POSITION = 0x4A  # Set target position in degrees
    CMD_GET_POSITION = 0x4C  # Get current position (updates every 10ms)
    CMD_SET_ORIGIN = 0x40  # Set current position as origin (zero point)
    CMD_POSITION_ECHO = 0x57  # Position command echo response


class CubeMarsAK606v3(BaseMotor):
    """AK60-6 Motor Controller for CubeMars V3 UART Protocol.

    Inherits shared safety checks, tendon helpers, and context-manager
    protocol from :class:`BaseMotor`.
    """

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
        super().__init__()
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

    def _extract_frame(self, data: bytes) -> bytes:
        """Find the first valid frame in a raw serial buffer.

        The motor pushes periodic status frames continuously; reading
        in_waiting bytes can return multiple concatenated frames.  This method
        scans for 0xAA, uses DataLength to compute the end index, and confirms
        0xBB at that position.

        Frame layout: AA | DataLength | CMD | Payload | CRC_H | CRC_L | BB
        Total bytes : 1  +     1      +        DataLength     +    2  +  1
                    = DataLength + 5

        :param data: Raw bytes from the serial buffer.
        :return: First self-consistent frame, or b"" if none found.
        """
        for i in range(len(data) - 1):
            if data[i] != FRAME_BYTES.start:
                continue
            data_length = data[i + 1]
            end_idx = i + data_length + 4  # AA(1)+len(1)+data_length+CRC(2)+BB(1) - 1
            if end_idx >= len(data):
                continue
            if data[end_idx] != FRAME_BYTES.end:
                continue
            frame = data[i : end_idx + 1]
            if len(frame) >= FRAME_BYTES.min_response_length:
                logger.debug(f"Frame extracted at offset {i}: {len(frame)} bytes")
                return frame
        return b""

    def _send_frame(self, frame: bytes) -> bytes:
        """Send frame to motor over UART and read response.

        :param frame: Complete frame to send.
        :return: Response bytes from motor (if any).
        """
        if not self.connected or self.serial is None or not self.serial.is_open:
            logger.debug("Motor not connected - skipping message send")
            return b""

        self.serial.reset_input_buffer()
        self.serial.write(frame)
        logger.debug(f"TX: {' '.join(f'{b:02X}' for b in frame)}")

        # Wait for response - some commands need more time
        time.sleep(0.1)

        # Read any available response
        response = b""
        bytes_waiting = self.serial.in_waiting
        logger.debug(f"Bytes waiting in buffer: {bytes_waiting}")

        if bytes_waiting > 0:
            raw = self.serial.read(bytes_waiting)
            if raw:
                logger.debug(
                    f"RX raw ({len(raw)} bytes): {' '.join(f'{b:02X}' for b in raw)}"
                )
                response = self._extract_frame(raw)
                if response:
                    logger.debug(f"RX frame: {' '.join(f'{b:02X}' for b in response)}")
                    is_valid = self._parse_motor_response(response)
                    if is_valid:
                        self._consecutive_no_response = 0
                        self._consecutive_invalid_response = 0
                        self.communicating = True
                    else:
                        self._consecutive_invalid_response += 1
                        if self._consecutive_invalid_response >= self._max_no_response:
                            logger.warning(
                                f"Motor sending invalid responses after {self._consecutive_invalid_response} attempts - "
                                "hardware may be powered off or cables disconnected"
                            )
                            self.communicating = False
                        response = b""
                else:
                    logger.debug(f"No valid frame found in {len(raw)}-byte buffer")
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

    def _parse_full_status(self, payload: bytes) -> MotorState | None:
        """Parse full status response (CMD_GET_STATUS).

        :param payload: Response payload bytes.
        :return: None
        """
        status = self.status_parser.parse_full_status(payload)
        if status:
            self.status_parser.log_motor_status(status)
            return MotorState(
                position_degrees=status.status_position.position_degrees,
                speed_erpm=status.duty_speed_voltage.speed_erpm,
                current_amps=status.currents.iq_current_amps,
                temperature_celsius=int(status.temperatures.mos_temp_celsius),
                error_code=status.status_position.status_code,
            )
        return None

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
                    # Position is a 4-byte float in big-endian format
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

        No artificial limits - motor can rotate continuously for spool-based cable systems.

        :param position_degrees: Target position in degrees (unlimited range)
        :return: None
        """
        value = int(position_degrees * SCALE_FACTORS.position)
        payload = struct.pack(">i", value)
        frame = self._build_frame(MotorCommand.CMD_SET_POSITION, payload)
        self._send_frame(frame)

    def set_origin(self, permanent: bool = False) -> None:
        """Set the current rotor position as the origin (zero point).

        After this call the motor treats its current position as 0 degrees.
        Use permanent=False (default) so the origin resets on power loss.

        :param permanent: If True, saves the new origin to flash (survives power cycle).
        :return: None
        """
        payload = bytes([1 if permanent else 0])
        frame = self._build_frame(MotorCommand.CMD_SET_ORIGIN, payload)
        self._send_frame(frame)

    def _get_current_position_for_estimate(self) -> float:
        """Return current UART position as float for movement estimation."""
        current_position = 0.0
        status = self.get_status()
        if status:
            current_position = status.position_degrees
        return current_position

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

    def _send_velocity_command(self, velocity_erpm: int) -> None:
        """Send velocity command over UART."""
        payload = struct.pack(">i", velocity_erpm)
        frame = self._build_frame(MotorCommand.CMD_SET_VELOCITY, payload)
        self._send_frame(frame)

    def get_status(self) -> MotorState | None:
        """Get all motor parameters and return a unified state.

        :return: MotorState object, or None if response failed.
        """
        # CMD_GET_STATUS requires no payload - it returns everything
        frame = self._build_frame(MotorCommand.CMD_GET_STATUS, b"")
        response = self._send_frame(frame)

        if not response or len(response) < FRAME_BYTES.min_response_length:
            return None

        # Extract payload from response (discarding headers + command + CRC)
        payload = response[3:-3]
        return self._parse_full_status(payload)

    def check_communication(self) -> bool:
        """Verify motor is responding to commands.

        :return: True if motor responds, False otherwise
        """
        if not self.connected:
            return False

        # Try to get status MOTOR_DEFAULTS.max_communication_attempts times
        for _attempt in range(MOTOR_DEFAULTS.max_communication_attempts):
            status = self.get_status()
            if status is not None:
                self.communicating = True
                self.consecutive_communication_errors = 0
                return True
            time.sleep(MOTOR_DEFAULTS.communication_retry_delay)

        logger.warning("Motor not responding to status queries")
        self.communicating = False
        return False

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

    def _stop_motor_transport(self) -> None:
        """Close serial connection to motor."""
        if self.serial and self.serial.is_open:
            self.stop()
            self.serial.close()
            logger.info("Motor connection closed")
