"""AK60-6 Motor Control Class - CubeMars UART Protocol."""

import struct
import time

import serial
from loguru import logger

# Motor Fault Codes (from CubeMars Manual)
FAULT_CODES = {
    0: "NONE",
    1: "OVER_VOLTAGE",
    2: "UNDER_VOLTAGE",
    3: "DRV (Drive fault)",
    4: "ABS_OVER_CURRENT (Motor over-current)",
    5: "OVER_TEMP_FET (MOS over-temperature)",
    6: "OVER_TEMP_MOTOR (Motor over-temperature)",
    7: "GATE_DRIVER_OVER_VOLTAGE",
    8: "GATE_DRIVER_UNDER_VOLTAGE",
    9: "MCU_UNDER_VOLTAGE",
    10: "BOOTING_FROM_WATCHDOG_RESET",
    11: "ENCODER_SPI",
    12: "ENCODER_SINCOS_BELOW_MIN_AMPLITUDE",
    13: "ENCODER_SINCOS_ABOVE_MAX_AMPLITUDE",
    14: "FLASH_CORRUPTION",
    15: "HIGH_OFFSET_CURRENT_SENSOR_1",
    16: "HIGH_OFFSET_CURRENT_SENSOR_2",
    17: "HIGH_OFFSET_CURRENT_SENSOR_3",
    18: "UNBALANCED_CURRENTS",
}

# CRC16 Lookup Table from CubeMars Manual
CRC16_TAB = [
    0x0000,
    0x1021,
    0x2042,
    0x3063,
    0x4084,
    0x50A5,
    0x60C6,
    0x70E7,
    0x8108,
    0x9129,
    0xA14A,
    0xB16B,
    0xC18C,
    0xD1AD,
    0xE1CE,
    0xF1EF,
    0x1231,
    0x0210,
    0x3273,
    0x2252,
    0x52B5,
    0x4294,
    0x72F7,
    0x62D6,
    0x9339,
    0x8318,
    0xB37B,
    0xA35A,
    0xD3BD,
    0xC39C,
    0xF3FF,
    0xE3DE,
    0x2462,
    0x3443,
    0x0420,
    0x1401,
    0x64E6,
    0x74C7,
    0x44A4,
    0x5485,
    0xA56A,
    0xB54B,
    0x8528,
    0x9509,
    0xE5EE,
    0xF5CF,
    0xC5AC,
    0xD58D,
    0x3653,
    0x2672,
    0x1611,
    0x0630,
    0x76D7,
    0x66F6,
    0x5695,
    0x46B4,
    0xB75B,
    0xA77A,
    0x9719,
    0x8738,
    0xF7DF,
    0xE7FE,
    0xD79D,
    0xC7BC,
    0x48C4,
    0x58E5,
    0x6886,
    0x78A7,
    0x0840,
    0x1861,
    0x2802,
    0x3823,
    0xC9CC,
    0xD9ED,
    0xE98E,
    0xF9AF,
    0x8948,
    0x9969,
    0xA90A,
    0xB92B,
    0x5AF5,
    0x4AD4,
    0x7AB7,
    0x6A96,
    0x1A71,
    0x0A50,
    0x3A33,
    0x2A12,
    0xDBFD,
    0xCBDC,
    0xFBBF,
    0xEB9E,
    0x9B79,
    0x8B58,
    0xBB3B,
    0xAB1A,
    0x6CA6,
    0x7C87,
    0x4CE4,
    0x5CC5,
    0x2C22,
    0x3C03,
    0x0C60,
    0x1C41,
    0xEDAE,
    0xFD8F,
    0xCDEC,
    0xDDCD,
    0xAD2A,
    0xBD0B,
    0x8D68,
    0x9D49,
    0x7E97,
    0x6EB6,
    0x5ED5,
    0x4EF4,
    0x3E13,
    0x2E32,
    0x1E51,
    0x0E70,
    0xFF9F,
    0xEFBE,
    0xDFDD,
    0xCFFC,
    0xBF1B,
    0xAF3A,
    0x9F59,
    0x8F78,
    0x9188,
    0x81A9,
    0xB1CA,
    0xA1EB,
    0xD10C,
    0xC12D,
    0xF14E,
    0xE16F,
    0x1080,
    0x00A1,
    0x30C2,
    0x20E3,
    0x5004,
    0x4025,
    0x7046,
    0x6067,
    0x83B9,
    0x9398,
    0xA3FB,
    0xB3DA,
    0xC33D,
    0xD31C,
    0xE37F,
    0xF35E,
    0x02B1,
    0x1290,
    0x22F3,
    0x32D2,
    0x4235,
    0x5214,
    0x6277,
    0x7256,
    0xB5EA,
    0xA5CB,
    0x95A8,
    0x8589,
    0xF56E,
    0xE54F,
    0xD52C,
    0xC50D,
    0x34E2,
    0x24C3,
    0x14A0,
    0x0481,
    0x7466,
    0x6447,
    0x5424,
    0x4405,
    0xA7DB,
    0xB7FA,
    0x8799,
    0x97B8,
    0xE75F,
    0xF77E,
    0xC71D,
    0xD73C,
    0x26D3,
    0x36F2,
    0x0691,
    0x16B0,
    0x6657,
    0x7676,
    0x4615,
    0x5634,
    0xD94C,
    0xC96D,
    0xF90E,
    0xE92F,
    0x99C8,
    0x89E9,
    0xB98A,
    0xA9AB,
    0x5844,
    0x4865,
    0x7806,
    0x6827,
    0x18C0,
    0x08E1,
    0x3882,
    0x28A3,
    0xCB7D,
    0xDB5C,
    0xEB3F,
    0xFB1E,
    0x8BF9,
    0x9BD8,
    0xABBB,
    0xBB9A,
    0x4A75,
    0x5A54,
    0x6A37,
    0x7A16,
    0x0AF1,
    0x1AD0,
    0x2AB3,
    0x3A92,
    0xFD2E,
    0xED0F,
    0xDD6C,
    0xCD4D,
    0xBDAA,
    0xAD8B,
    0x9DE8,
    0x8DC9,
    0x7C26,
    0x6C07,
    0x5C64,
    0x4C45,
    0x3CA2,
    0x2C83,
    0x1CE0,
    0x0CC1,
    0xEF1F,
    0xFF3E,
    0xCF5D,
    0xDF7C,
    0xAF9B,
    0xBFBA,
    0x8FD9,
    0x9FF8,
    0x6E17,
    0x7E36,
    0x4E55,
    0x5E74,
    0x2E93,
    0x3EB2,
    0x0ED1,
    0x1EF0,
]


class CubeMarsMotor:
    """AK60-6 Motor Controller for CubeMars V3 UART Protocol."""

    # Command constants from CubeMars Manual
    CMD_SET_DUTY = 0x46
    CMD_SET_CURRENT = 0x47
    CMD_SET_SPEED = 0x49
    CMD_SET_POSITION = 0x4A
    CMD_GET_STATUS = 0x45  # Get all motor parameters
    CMD_GET_POSITION = 0x4C  # Get current position (updates every 10ms)
    CMD_GET_PARAMS = 0x13  # Get specific parameters based on bit field

    def __init__(self, port: str = "/dev/ttyTHS1", baudrate: int = 921600):
        """Initialize motor connection.

        Args:
            port: Serial port path (default: /dev/ttyTHS1)
            baudrate: Communication baudrate (default: 921600)

        """
        self.port = port
        self.baudrate = baudrate
        self.serial: serial.Serial | None = None
        self._connect()

    def _connect(self):
        """Establish serial connection to motor."""
        self.serial = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            timeout=1,
            rtscts=False,
            dsrdtr=False,
        )
        time.sleep(0.1)  # Allow connection to stabilize
        logger.info(f"Connected to motor on {self.port} at {self.baudrate} baud")

    def _crc16(self, data: bytes) -> int:
        """Calculate CRC16-CCITT checksum.

        Args:
            data: Bytes to calculate checksum over

        Returns:
            16-bit CRC checksum

        """
        cksum = 0
        for byte in data:
            cksum = CRC16_TAB[((cksum >> 8) ^ byte) & 0xFF] ^ (cksum << 8) & 0xFFFF
        return cksum

    def _build_message(self, cmd: int, payload: bytes) -> bytes:
        """Build CubeMars UART frame with proper structure.

        Frame structure: AA | DataLength | CMD | Payload | CRC_H | CRC_L | BB

        Args:
            cmd: Command byte (0x46, 0x47, 0x49, 0x4A)
            payload: Command payload data

        Returns:
            Complete frame ready to send

        """
        data_frame = bytes([cmd]) + payload
        crc = self._crc16(data_frame)
        frame = bytes([0xAA, len(data_frame), *data_frame, crc >> 8, crc & 0xFF, 0xBB])
        return frame

    def _send_message(self, frame: bytes) -> bytes:
        """Send message to motor over UART and read response.

        Args:
            frame: Complete frame to send

        Returns:
            Response bytes from motor (if any)

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

    def _parse_full_status(self, payload: bytes):
        """Parse full status response (command 0x45).

        Args:
            payload: Response payload bytes

        """
        idx = 0

        # Temperatures
        if len(payload) >= idx + 4:
            mos_temp = struct.unpack(">h", payload[idx : idx + 2])[0] / 10.0
            motor_temp = struct.unpack(">h", payload[idx + 2 : idx + 4])[0] / 10.0
            logger.info(
                f"  MOS Temp: {mos_temp:.1f}°C | Motor Temp: {motor_temp:.1f}°C"
            )
            idx += 4

        # Currents
        if len(payload) >= idx + 16:
            output_current = struct.unpack(">i", payload[idx : idx + 4])[0] / 100.0
            input_current = struct.unpack(">i", payload[idx + 4 : idx + 8])[0] / 100.0
            id_current = struct.unpack(">i", payload[idx + 8 : idx + 12])[0] / 100.0
            iq_current = struct.unpack(">i", payload[idx + 12 : idx + 16])[0] / 100.0
            logger.info(
                f"  Output: {output_current:.2f}A | Input: {input_current:.2f}A"
            )
            logger.info(f"  Id: {id_current:.2f}A | Iq: {iq_current:.2f}A")
            idx += 16

        # Duty, Speed, Voltage
        if len(payload) >= idx + 8:
            duty = struct.unpack(">h", payload[idx : idx + 2])[0] / 1000.0
            speed = struct.unpack(">i", payload[idx + 2 : idx + 6])[0]
            voltage = struct.unpack(">h", payload[idx + 6 : idx + 8])[0] / 10.0
            logger.info(
                f"  Duty: {duty:.1f}% | Speed: {speed} ERPM | Voltage: {voltage:.1f}V"
            )
            idx += 8

        # Skip reserved (24 bytes)
        idx += 24

        # Status, Position, Motor ID
        if len(payload) >= idx + 6:
            status = payload[idx]
            status_name = FAULT_CODES.get(status, f"UNKNOWN({status})")
            position = struct.unpack(">f", payload[idx + 1 : idx + 5])[0]
            motor_id = payload[idx + 5]
            logger.info(f"  Status: {status} - {status_name}")
            logger.info(f"  Position: {position:.2f}° | Motor ID: {motor_id}")
            idx += 6

        # Skip temperature reserved (6 bytes)
        idx += 6

        # Voltages and Control Mode
        if len(payload) >= idx + 12:
            vd = struct.unpack(">i", payload[idx : idx + 4])[0] / 1000.0
            vq = struct.unpack(">i", payload[idx + 4 : idx + 8])[0] / 1000.0
            ctrl_mode = struct.unpack(">i", payload[idx + 8 : idx + 12])[0]
            logger.info(f"  Vd: {vd:.3f}V | Vq: {vq:.3f}V | Control Mode: {ctrl_mode}")
            idx += 12

        # Encoder angles
        if len(payload) >= idx + 8:
            enc_angle = struct.unpack(">f", payload[idx : idx + 4])[0]
            outer_enc = struct.unpack(">f", payload[idx + 4 : idx + 8])[0]
            logger.info(
                f"  Encoder: {enc_angle:.2f}° | Outer Encoder: {outer_enc:.2f}°"
            )

    def _parse_motor_response(self, response: bytes):
        """Parse and display motor response data.

        Args:
            response: Raw response bytes from motor

        """
        if len(response) < 10:  # Minimum valid response length
            return

        # Check for valid frame structure (AA ... BB)
        if response[0] != 0xAA or response[-1] != 0xBB:
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

        :param position_degrees: Target position in degrees (-36000 to 36000)
        :return: None
        """
        position_degrees = max(min(position_degrees, 36000.0), -36000.0)
        value = int(position_degrees * 1_000_000.0)
        payload = struct.pack(">i", value)
        frame = self._build_message(self.CMD_SET_POSITION, payload)
        self._send_message(frame)

    def set_velocity(self, velocity_erpm: int) -> None:
        """Set motor velocity in electrical RPM.

        :param velocity_erpm: Target velocity in ERPM (-100000 to 100000)
        :return: None
        """
        velocity_erpm = max(min(int(velocity_erpm), 100000), -100000)
        payload = struct.pack(">i", velocity_erpm)
        frame = self._build_message(self.CMD_SET_SPEED, payload)
        self._send_message(frame)

    def set_speed(self, speed_erpm: int) -> None:
        """Set motor speed in electrical RPM (alias for set_velocity).

        :param speed_erpm: Target speed in ERPM (-100000 to 100000)
        :return: None
        """
        self.set_velocity(speed_erpm)

    def move_to_position_with_speed(
        self, target_degrees: float, speed_erpm: int, step_delay: float = 0.05
    ):
        """Reach the target position through speed-controlled increments."""
        # Get current position (assume we're tracking it)
        # For now, we'll send velocity command to move, then switch to position

        # Use velocity control to move at specified speed
        direction = 1 if target_degrees > 0 else -1
        self.set_velocity(speed_erpm * direction)

        # Calculate approximate time needed (rough estimate)
        # This is a simple approach - more sophisticated would read motor feedback
        estimated_time = (
            abs(target_degrees) / (abs(speed_erpm) / 60.0) if speed_erpm != 0 else 0
        )
        time.sleep(min(estimated_time, 5.0))  # Cap at 5 seconds

        # Switch to position hold
        self.set_position(target_degrees)
        logger.info(f"Reached position: {target_degrees}° at {speed_erpm} ERPM")

    def set_duty_cycle(self, duty_cycle_percent: float) -> None:
        """Set motor PWM duty cycle percentage.

        :param duty_cycle_percent: Duty cycle value (-1.0 to 1.0, where 1.0 = 100%)
        :return: None
        """
        duty_cycle_percent = max(min(duty_cycle_percent, 0.95), -0.95)
        value = int(duty_cycle_percent * 100000.0)
        payload = struct.pack(">i", value)
        frame = self._build_message(self.CMD_SET_DUTY, payload)
        self._send_message(frame)

    def set_current(self, current_amps: float) -> None:
        """Set motor current in amperes.

        :param current_amps: Current in Amps (-60.0 to 60.0)
        :return: None
        """
        current_amps = max(min(current_amps, 60.0), -60.0)
        value = int(current_amps * 1000.0)
        payload = struct.pack(">i", value)
        frame = self._build_message(self.CMD_SET_CURRENT, payload)
        self._send_message(frame)

    def get_status(self):
        """Get all motor parameters."""
        # Command 0x45 requires no payload - it returns everything
        frame = self._build_message(self.CMD_GET_STATUS, b"")
        response = self._send_message(frame)
        return response

    def get_position(self) -> bytes:
        """Get current motor position via command 0x4C.

        Command 0x4C returns current position every 10ms.
        Lightweight query for position feedback only.

        :return: Raw response bytes from motor containing position
        """
        # Command 0x4C with no payload
        frame = self._build_message(self.CMD_GET_POSITION, b"")
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

    def __enter__(self) -> "CubeMarsMotor":
        """Context manager entry.

        :return: Self instance for use in with statement
        """
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        """Context manager exit.

        :param exc_type: Exception type if an exception occurred
        :param exc_val: Exception value if an exception occurred
        :param exc_tb: Exception traceback if an exception occurred
        :return: None
        """
        self.close()
"""Sample doc string."""

from loguru import logger


def motor_v3() -> None:
    """Run dummy code for the motor."""
    logger.info("Hello, world!")
