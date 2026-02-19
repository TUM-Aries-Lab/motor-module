"""AK60-6 Motor Control Class - CubeMars Native CAN Protocol."""

import struct
import time
from dataclasses import dataclass

import can
import numpy as np
from loguru import logger

from motor_python.definitions import (
    CAN_DEFAULTS,
    MOTOR_DEFAULTS,
    MOTOR_LIMITS,
    TendonAction,
)


@dataclass
class CANMotorFeedback:
    """Motor feedback data from CAN upload message (8 bytes).

    Real-time motor state transmitted periodically at configured rate (1-500 Hz).
    """

    position_degrees: float  # Motor position in degrees (-3200° to 3200°)
    speed_erpm: int  # Electrical speed in RPM (-320000 to 320000)
    current_amps: float  # Motor current in amps (-60 to 60 A)
    temperature_celsius: int  # Driver board temperature (-20 to 127°C)
    error_code: int  # Error code (0=no fault, 1-7=various faults)


class CANControlMode:
    """CAN extended ID control modes per official CubeMars documentation.

    The control mode is encoded in the upper bytes of the 29-bit extended CAN ID.
    Format: 00 00 0M XX where M is mode and XX is motor ID.
    """

    DUTY_CYCLE = 0x00  # Duty cycle control mode (square wave voltage)
    CURRENT_LOOP = 0x01  # Current control (IQ current, torque mode)
    CURRENT_BRAKE = 0x02  # Brake current mode (hold position with current)
    VELOCITY_LOOP = 0x03  # Velocity control (ERPM with max acceleration)
    POSITION_LOOP = 0x04  # Position control (degrees with max speed/accel)
    SET_ORIGIN = 0x05  # Set origin position mode (temporary or permanent)
    POSITION_VELOCITY = 0x06  # Position + velocity + acceleration control
    MIT_MODE = 0x08  # MIT control mode (Force mode with KP, KD)


class CubeMarsAK606v3CAN:
    """AK60-6 Motor Controller using native CAN bus protocol.

    Uses SocketCAN interface on Jetson Orin Nano with SN65HVD230 CAN transceiver.
    Implements the CubeMars native CAN protocol with extended IDs and 8-byte payloads.

    CAN Protocol Details:
    - Extended 29-bit CAN IDs encode control mode: 00 00 0M XX (M=mode, XX=motor ID)
    - Command payloads: 8 bytes direct data (not UART protocol)
    - Feedback: Motor auto-transmits 8-byte status at configured rate (1-500 Hz)
    """

    def __init__(
        self,
        motor_can_id: int = CAN_DEFAULTS.motor_can_id,
        interface: str = CAN_DEFAULTS.interface,
        bitrate: int = CAN_DEFAULTS.bitrate,
    ) -> None:
        """Initialize CAN motor connection.

        :param motor_can_id: Motor CAN ID (default: 0x03 from motor config).
        :param interface: CAN interface name (default: 'can0').
        :param bitrate: CAN bitrate in bits/sec (default: 1000000 = 1 Mbps).
        :return: None
        """
        self.motor_can_id = motor_can_id
        self.interface = interface
        self.bitrate = bitrate
        self.bus: can.BusABC | None = None
        self.connected = False
        self.communicating = False
        self._last_feedback: CANMotorFeedback | None = None
        self._consecutive_no_response = 0
        self._max_no_response = MOTOR_DEFAULTS.max_no_response_attempts
        self._connect()

    def _connect(self) -> None:
        """Establish CAN bus connection to motor.

        :return: None
        """
        try:
            # Initialize SocketCAN interface
            self.bus = can.interface.Bus(
                channel=self.interface,
                interface="socketcan",
                bitrate=self.bitrate,
            )
            time.sleep(CAN_DEFAULTS.connection_stabilization_delay)
            self.connected = True
            logger.info(
                f"Connected to motor CAN ID 0x{self.motor_can_id:02X} on {self.interface} "
                f"at {self.bitrate} bps"
            )
        except can.CanError as e:
            logger.warning(f"Failed to connect to CAN interface {self.interface}: {e}")
            self.connected = False
        except Exception as e:
            logger.warning(f"Unexpected error connecting to CAN bus: {e}")
            self.connected = False

    def enable_motor(self) -> None:
        """Enable the motor for Servo mode control.

        Sends power-on command 0xFFFFFFFFFFFFFFFC to motor ID.
        Must be called before sending control commands.
        """
        if not self.connected or self.bus is None:
            logger.warning("Cannot enable motor - CAN bus not connected")
            return

        try:
            msg = can.Message(
                arbitration_id=self.motor_can_id,
                data=bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC]),
                is_extended_id=True,
            )
            self.bus.send(msg, timeout=0.1)
            logger.info("Motor enabled (Servo mode)")
        except can.CanError as e:
            logger.error(f"Failed to enable motor: {e}")

    def disable_motor(self) -> None:
        """Disable the motor (power off).

        Sends power-off command 0xFFFFFFFFFFFFFFFD to motor ID.
        """
        if not self.connected or self.bus is None:
            logger.warning("Cannot disable motor - CAN bus not connected")
            return

        try:
            msg = can.Message(
                arbitration_id=self.motor_can_id,
                data=bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD]),
                is_extended_id=True,
            )
            self.bus.send(msg, timeout=0.1)
            logger.info("Motor disabled")
        except can.CanError as e:
            logger.error(f"Failed to disable motor: {e}")

    def _build_extended_id(self, mode: int) -> int:
        """Build extended 29-bit CAN ID with mode and motor ID.

        Format: 00 00 0M XX where M is control mode and XX is motor ID.

        :param mode: Control mode (from CANControlMode class).
        :return: Extended CAN arbitration ID.
        """
        extended_id = (mode << 8) | self.motor_can_id
        return extended_id

    def _send_can_command(self, mode: int, data: bytes) -> None:
        """Send CAN command to motor.

        :param mode: Control mode (from CANControlMode).
        :param data: 8-byte payload (padded with zeros if shorter).
        :return: None
        """
        if not self.connected or self.bus is None:
            logger.debug("CAN bus not connected - skipping command")
            return

        # Ensure data is exactly 8 bytes (pad with zeros if needed)
        if len(data) < 8:
            data = data + bytes(8 - len(data))
        elif len(data) > 8:
            data = data[:8]

        try:
            arbitration_id = self._build_extended_id(mode)
            msg = can.Message(
                arbitration_id=arbitration_id,
                data=data,
                is_extended_id=True,  # Use extended 29-bit CAN ID
            )

            self.bus.send(msg)
            logger.debug(
                f"TX CAN ID 0x{msg.arbitration_id:08X}: "
                f"{' '.join(f'{b:02X}' for b in msg.data)}"
            )
        except can.CanError as e:
            logger.error(f"CAN error sending command: {e}")

    def _receive_feedback(self, timeout: float = 0.1) -> CANMotorFeedback | None:
        """Receive and parse motor feedback message.

        Motor automatically sends 8-byte feedback at configured rate (1-500 Hz).
        Feedback format:
        - Data[0-1]: Position int16 (range -32000 to 32000 = -3200° to 3200°)
        - Data[2-3]: Speed int16 (range -32000 to 32000 = -320000 to 320000 ERPM)
        - Data[4-5]: Current int16 (range -6000 to 6000 = -60 to 60 A)
        - Data[6]: Temperature int8 (range -20 to 127°C)
        - Data[7]: Error code uint8

        :param timeout: Timeout in seconds to wait for feedback.
        :return: CANMotorFeedback object or None if no message received.
        """
        if not self.connected or self.bus is None:
            return None

        try:
            msg = self.bus.recv(timeout=timeout)
            if msg is None:
                return None

            # Check if message is from our motor
            # Motor may use extended ID format or simple ID - check both
            # Extended format example: 0x00002903 contains motor ID in lower byte
            expected_ids = [
                self.motor_can_id,  # Simple ID (e.g., 0x03)
                (0x2900 | self.motor_can_id),  # Extended feedback ID (e.g., 0x2903)
            ]

            if msg.arbitration_id not in expected_ids:
                logger.debug(
                    f"Ignoring CAN ID 0x{msg.arbitration_id:08X} (expecting {[hex(id) for id in expected_ids]})"
                )
                return None

            if len(msg.data) < 8:
                logger.warning(f"Received short CAN message: {len(msg.data)} bytes")
                return None

            # Parse feedback data
            # Position: int16, scaled by 0.1 degrees
            pos_int = struct.unpack(">h", msg.data[0:2])[0]
            position_degrees = pos_int * 0.1

            # Speed: int16, scaled by 10 ERPM
            speed_int = struct.unpack(">h", msg.data[2:4])[0]
            speed_erpm = speed_int * 10

            # Current: int16, scaled by 0.01 A
            current_int = struct.unpack(">h", msg.data[4:6])[0]
            current_amps = current_int * 0.01

            # Temperature: int8 (direct value in °C)
            temperature_celsius = struct.unpack("b", bytes([msg.data[6]]))[0]

            # Error code: uint8
            error_code = msg.data[7]

            feedback = CANMotorFeedback(
                position_degrees=position_degrees,
                speed_erpm=speed_erpm,
                current_amps=current_amps,
                temperature_celsius=temperature_celsius,
                error_code=error_code,
            )

            self._last_feedback = feedback
            self._consecutive_no_response = 0
            self.communicating = True

            logger.debug(
                f"RX Feedback: Pos={position_degrees:.1f}°, "
                f"Speed={speed_erpm} ERPM, "
                f"Current={current_amps:.2f}A, "
                f"Temp={temperature_celsius}°C, "
                f"Error={error_code}"
            )

            return feedback

        except (can.CanError, Exception) as e:
            logger.error(f"CAN error receiving feedback: {e}")
            return None

    def set_position(self, position_degrees: float) -> None:
        """Set motor target position in degrees using CAN Position Loop mode.

        Sends position command via CAN extended ID 0x00000468 (mode 04, motor ID in example 68).
        Position is sent as int32 value (degrees * 10000).
        Example: 600° = 6000000 = 0x005B8D80

        :param position_degrees: Target position in degrees (-3200° to 3200°).
        :return: None
        """
        # Clamp to CAN protocol feedback limits
        position_degrees = np.clip(position_degrees, -3200.0, 3200.0)

        # Convert to int32: degrees * 10000
        # Example from doc: 600 degrees = 0x005B8D80 = 6000000
        position_int = int(position_degrees * 10000.0)

        # Pack as big-endian int32
        data = struct.pack(">i", position_int)

        self._send_can_command(CANControlMode.POSITION_LOOP, data)
        logger.info(f"Set position: {position_degrees:.2f}°")

    def get_position(self) -> float | None:
        """Get current motor position from periodic feedback.

        :return: Current position in degrees, or None if no recent feedback.
        """
        # Listen for feedback message
        feedback = self._receive_feedback(timeout=0.2)
        if feedback:
            return feedback.position_degrees
        elif self._last_feedback:
            return self._last_feedback.position_degrees
        return None

    def set_velocity(self, velocity_erpm: int, allow_low_speed: bool = False) -> None:
        """Set motor velocity in electrical RPM using CAN Velocity Loop mode.

        Sends velocity command via CAN extended ID 0x00000368 (mode 03).
        Velocity is sent as int32 value (ERPM directly).
        Example: 5000 ERPM = 0x00001388, -5000 ERPM = 0xFFFFEC78

        Low speeds (<5000 ERPM) with high firmware acceleration cause current
        oscillations and audible noise. Use medium-to-high speeds for exosuit.

        :param velocity_erpm: Target velocity in ERPM (safe range: 5000-100000)
        :param allow_low_speed: Bypass the 5000 ERPM safety floor (default: False)
        :return: None
        :raises ValueError: If velocity below safe threshold and allow_low_speed=False
        """
        velocity_erpm_int = int(velocity_erpm)

        # Velocity 0 means stop -- use current=0 to release windings cleanly
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

        # Clamp to protocol limits (based on feedback range -320000 to 320000)
        velocity_erpm = np.clip(velocity_erpm_int, -320000, 320000)
        if velocity_erpm != velocity_erpm_int:
            logger.warning(
                f"Velocity {velocity_erpm_int} ERPM clamped to {velocity_erpm} ERPM"
            )

        # Pack as int32 directly (no scaling needed based on examples)
        # Example from doc: 5000 ERPM = 0x00001388
        data = struct.pack(">i", velocity_erpm)

        self._send_can_command(CANControlMode.VELOCITY_LOOP, data)
        logger.info(f"Set velocity: {velocity_erpm} ERPM")

    def set_current(self, current_amps: float) -> None:
        """Set motor current in amps using CAN Current Loop mode.

        Sends current command via CAN extended ID 0x00000168 (mode 01).
        Current is sent as int32 value (amps * 1000).
        Example: -4.4 A (IQ) = -4400 = 0xFFFFEED8 (from doc example)

        :param current_amps: Target current in amps (-60 to 60 A).
        :return: None
        """
        # Clamp to protocol limits
        current_amps = np.clip(current_amps, -60.0, 60.0)

        # Convert to int32: amps * 1000
        # Example from doc: -4.4 A = 0xFFFFEED8 = -4400
        current_int = int(current_amps * 1000.0)

        # Pack as big-endian int32
        data = struct.pack(">i", current_int)

        self._send_can_command(CANControlMode.CURRENT_LOOP, data)
        logger.info(f"Set current: {current_amps:.2f} A")

    def get_status(self) -> CANMotorFeedback | None:
        """Get motor status from periodic feedback.

        In CAN mode, the motor automatically sends feedback at configured rate.
        This method listens for the next feedback message.

        :return: CANMotorFeedback object with current motor state, or None.
        """
        feedback = self._receive_feedback(timeout=0.5)
        if feedback:
            logger.info("=" * 50)
            logger.info("MOTOR STATUS:")
            logger.info(f"  Position: {feedback.position_degrees:.2f}°")
            logger.info(f"  Speed: {feedback.speed_erpm} ERPM")
            logger.info(f"  Current: {feedback.current_amps:.2f} A")
            logger.info(f"  Temperature: {feedback.temperature_celsius}°C")
            logger.info(f"  Error Code: {feedback.error_code}")
            logger.info("=" * 50)
        return feedback

    def check_communication(self) -> bool:
        """Verify motor is responding by listening for feedback messages.

        :return: True if motor responds, False otherwise
        """
        if not self.connected:
            return False

        # Try to receive feedback multiple times
        for _attempt in range(MOTOR_DEFAULTS.max_communication_attempts):
            feedback = self._receive_feedback(timeout=0.5)
            if feedback:
                self.communicating = True
                self._consecutive_no_response = 0
                logger.info("Motor communication verified")
                return True
            time.sleep(MOTOR_DEFAULTS.communication_retry_delay)

        logger.warning("Motor not responding - no feedback messages received")
        self.communicating = False
        return False

    def move_to_position_with_speed(
        self,
        target_degrees: float,
        motor_speed_erpm: int,
        step_delay: float = MOTOR_DEFAULTS.step_delay,
    ) -> None:
        """Reach a target position using velocity control then hold with position.

        :param target_degrees: Target position in degrees.
        :param motor_speed_erpm: Motor speed in ERPM (absolute, direction auto).
        :param step_delay: Delay between steps in seconds (ignored in CAN mode).
        :return: None
        """
        # Get current position
        current_pos = self.get_position()
        if current_pos is None:
            current_pos = 0.0

        # Calculate direction and time
        distance = target_degrees - current_pos
        direction = 1 if distance > 0 else -1

        # Use velocity control to move
        self.set_velocity(
            velocity_erpm=motor_speed_erpm * direction, allow_low_speed=True
        )

        # Estimate movement time
        if motor_speed_erpm > 0:
            degrees_per_second = abs(motor_speed_erpm) / 60.0 * 360.0
            estimated_time = abs(distance) / degrees_per_second
            time.sleep(min(estimated_time, MOTOR_LIMITS.max_movement_time))

        # Switch to position hold
        self.set_position(target_degrees)
        logger.info(
            f"Reached position: {target_degrees:.1f}° at {motor_speed_erpm} ERPM"
        )

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

        :return: None
        """
        self.set_current(0.0)
        time.sleep(0.1)
        # Send a second time for reliability
        self.set_current(0.0)
        time.sleep(0.2)
        logger.info("Motor stopped (current=0, windings released)")

    def close(self) -> None:
        """Close CAN bus connection to motor.

        :return: None
        """
        if self.bus:
            self.stop()
            self.bus.shutdown()
            logger.info("CAN bus connection closed")

    def __enter__(self) -> "CubeMarsAK606v3CAN":
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
