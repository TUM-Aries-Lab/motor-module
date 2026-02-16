"""Common definitions for this module."""

from dataclasses import asdict, dataclass
from enum import IntEnum
from pathlib import Path

import numpy as np

np.set_printoptions(precision=3, floatmode="fixed", suppress=True)


# --- Directories ---
ROOT_DIR: Path = Path("src").parent
DATA_DIR: Path = ROOT_DIR / "data"
RECORDINGS_DIR: Path = DATA_DIR / "recordings"
LOG_DIR: Path = DATA_DIR / "logs"

# Default encoding
ENCODING: str = "utf-8"

DATE_FORMAT = "%Y-%m-%d_%H-%M-%S"


@dataclass
class LogLevel:
    """Log level."""

    trace: str = "TRACE"
    debug: str = "DEBUG"
    info: str = "INFO"
    success: str = "SUCCESS"
    warning: str = "WARNING"
    error: str = "ERROR"
    critical: str = "CRITICAL"

    def __iter__(self):
        """Iterate over log levels."""
        return iter(asdict(self).values())


DEFAULT_LOG_LEVEL = LogLevel.info
DEFAULT_LOG_FILENAME = "log_file"


@dataclass(frozen=True)
class MotorDefaults:
    """Motor communication and timing default settings."""

    port: Path = Path("/dev/ttyTHS1")
    baudrate: int = 921600
    step_delay: float = 0.05
    serial_timeout: float = 1.0  # Serial port timeout in seconds
    connection_stabilization_delay: float = 0.1  # Delay after opening serial port
    response_wait_delay: float = 0.1  # Delay before reading response
    communication_retry_delay: float = 0.2  # Delay between communication check retries
    max_no_response_attempts: int = (
        3  # Max failed attempts before marking as not communicating
    )
    max_communication_attempts: int = 2  # Number of retries for communication check


@dataclass(frozen=True)
class FrameBytes:
    """Motor protocol frame bytes."""

    start: int = 0xAA
    end: int = 0xBB
    min_response_length: int = 10  # Minimum valid response frame length
    min_frame_with_payload: int = 6  # Minimum frame length that includes payload
    expected_status_response_length: int = 90  # Full status response (cmd 0x45)
    min_status_response_length: int = 85  # Minimum acceptable status response
    separator_length: int = 50  # Length of separator line in logs
    start_index: int = 0  # Index of start byte in frame
    length_index: int = 1  # Index of length byte in frame
    cmd_index: int = 2  # Index of command byte in frame
    payload_start_index: int = 3  # Index where payload starts
    crc_and_end_length: int = 3  # CRC (2 bytes) + end byte (1 byte)
    position_payload_size: int = 4  # Size of position data in payload


@dataclass(frozen=True)
class CRCConstants:
    """CRC16 calculation constants."""

    initial_value: int = 0
    byte_mask: int = 0xFF
    word_mask: int = 0xFFFF
    shift_bits: int = 8


@dataclass(frozen=True)
class ScaleFactors:
    """Motor status payload scaling factors.

    The motor protocol transmits values as scaled integers to preserve precision.
    These factors convert between the motor's integer representation and SI units:
    - Temperature: Motor sends deci-degrees Celsius (0.1°C units) as int16
    - Current: Motor sends centi-amps (0.01A units) as int32
    - Voltage: Motor sends deci-volts (0.1V units) as int16
    - Duty: Motor sends per-mille (0.1% units) as int16 for status, different scaling for commands
    - Position: Motor sends micro-degrees (0.000001° units) as int32
    """

    temperature: float = (
        10.0  # Temperature is sent as int16 * 10 (deci-degrees Celsius)
    )
    current: float = 100.0  # Current is sent as int32 * 100 (centi-amps)
    duty: float = 1000.0  # Duty cycle is sent as int16 * 1000 (per-mille)
    voltage: float = 10.0  # Voltage is sent as int16 * 10 (deci-volts)
    vd_vq: float = 1000.0  # Vd/Vq voltages are sent as int32 * 1000
    position: float = (
        1_000_000.0  # Position is sent as int32 * 1,000,000 (micro-degrees)
    )


@dataclass(frozen=True)
class PayloadSizes:
    """Motor status payload structure sizes."""

    temperature: int = 4  # 2 int16 values
    current: int = 16  # 4 int32 values
    duty_speed_voltage: int = 8  # 1 int16 + 1 int32 + 1 int16
    status_position_id: int = 6  # 1 byte + 1 float + 1 byte
    reserved: int = 24  # Reserved bytes
    temp_reserved: int = 6  # Temperature reserved bytes
    voltage_control: int = 12  # 2 int32 + 1 int32
    encoder: int = 8  # 2 float values


@dataclass(frozen=True)
class MotorLimits:
    """Motor velocity and position control limits.

    Low speeds (< 5000 ERPM) with high firmware acceleration settings cause
    current oscillations and noise. Position is limited to +/-360 degrees
    (one full rotation each direction) for exosuit joint safety.
    """

    max_velocity_electrical_rpm: int = 100000
    min_velocity_electrical_rpm: int = -100000
    min_safe_velocity_erpm: int = 5000  # Below this, firmware accel causes oscillations
    max_position_degrees: float = 360.0  # Maximum position: 1 full rotation
    min_position_degrees: float = -360.0  # Minimum position: -1 full rotation
    max_movement_time: float = 5.0  # Maximum movement time cap in seconds
    soft_start_current_ma: int = 3000  # Gentle current (mA) to pre-spin past noisy zone
    soft_start_duration: float = 0.15  # Seconds to hold pre-spin current
    default_tendon_velocity_erpm: int = 10000  # Default velocity for tendon control
    default_velocity_demo_erpm: int = 8000  # Safe moderate velocity for demo/testing


@dataclass(frozen=True)
class ConversionFactors:
    """Unit conversion factors."""

    seconds_per_minute: float = 60.0  # Seconds in a minute
    degrees_per_revolution: float = 360.0  # Degrees in one full revolution
    crc_high_byte_shift: int = 8  # Bit shift for CRC high byte
    byte_mask: int = 0xFF  # Mask for extracting single byte


# Instantiate frozen dataclasses for easy access
MOTOR_DEFAULTS = MotorDefaults()
FRAME_BYTES = FrameBytes()
CRC_CONSTANTS = CRCConstants()
SCALE_FACTORS = ScaleFactors()
PAYLOAD_SIZES = PayloadSizes()
MOTOR_LIMITS = MotorLimits()
CONVERSION_FACTORS = ConversionFactors()


# Tendon action control
class TendonAction(IntEnum):
    """Exosuit tendon control actions."""

    PULL = 0  # Pull tendon (lift/assist)
    RELEASE = 1  # Release tendon (lower/relax)
    STOP = 2  # Stop motion


# Motor Fault Codes (from CubeMars Manual)
class FaultCode(IntEnum):
    """Motor fault codes."""

    NONE = 0
    OVER_VOLTAGE = 1
    UNDER_VOLTAGE = 2
    DRV = 3
    ABS_OVER_CURRENT = 4
    OVER_TEMP_FET = 5
    OVER_TEMP_MOTOR = 6
    GATE_DRIVER_OVER_VOLTAGE = 7
    GATE_DRIVER_UNDER_VOLTAGE = 8
    MCU_UNDER_VOLTAGE = 9
    BOOTING_FROM_WATCHDOG_RESET = 10
    ENCODER_SPI = 11
    ENCODER_SINCOS_BELOW_MIN_AMPLITUDE = 12
    ENCODER_SINCOS_ABOVE_MAX_AMPLITUDE = 13
    FLASH_CORRUPTION = 14
    HIGH_OFFSET_CURRENT_SENSOR_1 = 15
    HIGH_OFFSET_CURRENT_SENSOR_2 = 16
    HIGH_OFFSET_CURRENT_SENSOR_3 = 17
    UNBALANCED_CURRENTS = 18

    def get_description(self) -> str:
        """Get human-readable description of fault code.

        :return: Formatted string with code name and description.
        """
        descriptions = {
            FaultCode.NONE: "NONE",
            FaultCode.OVER_VOLTAGE: "OVER_VOLTAGE",
            FaultCode.UNDER_VOLTAGE: "UNDER_VOLTAGE",
            FaultCode.DRV: "DRV (Drive fault)",
            FaultCode.ABS_OVER_CURRENT: "ABS_OVER_CURRENT (Motor over-current)",
            FaultCode.OVER_TEMP_FET: "OVER_TEMP_FET (MOS over-temperature)",
            FaultCode.OVER_TEMP_MOTOR: "OVER_TEMP_MOTOR (Motor over-temperature)",
            FaultCode.GATE_DRIVER_OVER_VOLTAGE: "GATE_DRIVER_OVER_VOLTAGE",
            FaultCode.GATE_DRIVER_UNDER_VOLTAGE: "GATE_DRIVER_UNDER_VOLTAGE",
            FaultCode.MCU_UNDER_VOLTAGE: "MCU_UNDER_VOLTAGE",
            FaultCode.BOOTING_FROM_WATCHDOG_RESET: "BOOTING_FROM_WATCHDOG_RESET",
            FaultCode.ENCODER_SPI: "ENCODER_SPI",
            FaultCode.ENCODER_SINCOS_BELOW_MIN_AMPLITUDE: "ENCODER_SINCOS_BELOW_MIN_AMPLITUDE",
            FaultCode.ENCODER_SINCOS_ABOVE_MAX_AMPLITUDE: "ENCODER_SINCOS_ABOVE_MAX_AMPLITUDE",
            FaultCode.FLASH_CORRUPTION: "FLASH_CORRUPTION",
            FaultCode.HIGH_OFFSET_CURRENT_SENSOR_1: "HIGH_OFFSET_CURRENT_SENSOR_1",
            FaultCode.HIGH_OFFSET_CURRENT_SENSOR_2: "HIGH_OFFSET_CURRENT_SENSOR_2",
            FaultCode.HIGH_OFFSET_CURRENT_SENSOR_3: "HIGH_OFFSET_CURRENT_SENSOR_3",
            FaultCode.UNBALANCED_CURRENTS: "UNBALANCED_CURRENTS",
        }
        return descriptions.get(self, self.name)


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
