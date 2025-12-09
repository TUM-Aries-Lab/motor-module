"""Common definitions for this module."""

from dataclasses import asdict, dataclass
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

DUMMY_VARIABLE = "dummy_variable"


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
    """Motor communication default settings."""

    port: str = "/dev/ttyTHS1"
    baudrate: int = 921600
    step_delay: float = 0.05


@dataclass(frozen=True)
class FrameBytes:
    """Motor protocol frame bytes."""

    start: int = 0xAA
    end: int = 0xBB


@dataclass(frozen=True)
class CRCConstants:
    """CRC16 calculation constants."""

    initial_value: int = 0
    byte_mask: int = 0xFF
    word_mask: int = 0xFFFF
    shift_bits: int = 8


@dataclass(frozen=True)
class ScaleFactors:
    """Motor status payload scaling factors."""

    temperature: float = 10.0  # Temperature is sent as int16 * 10
    current: float = 100.0  # Current is sent as int32 * 100
    duty: float = 1000.0  # Duty cycle is sent as int16 * 1000
    voltage: float = 10.0  # Voltage is sent as int16 * 10
    vd_vq: float = 1000.0  # Vd/Vq voltages are sent as int32 * 1000
    position: float = 1_000_000.0  # Position is sent as int32 * 1,000,000


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
    """Motor control limits."""

    max_duty_cycle: float = 0.95  # Maximum duty cycle (95% to prevent saturation)
    min_duty_cycle: float = -0.95  # Minimum duty cycle
    max_current_amps: float = 60.0  # Maximum current in amps
    min_current_amps: float = -60.0  # Minimum current in amps
    max_velocity_erpm: int = 100000  # Maximum velocity in ERPM
    min_velocity_erpm: int = -100000  # Minimum velocity in ERPM
    max_position_degrees: float = 360.0  # Maximum position: 1 full rotation
    min_position_degrees: float = -360.0  # Minimum position: -1 full rotation


# Instantiate frozen dataclasses for easy access
MOTOR_DEFAULTS = MotorDefaults()
FRAME_BYTES = FrameBytes()
CRC_CONSTANTS = CRCConstants()
SCALE_FACTORS = ScaleFactors()
PAYLOAD_SIZES = PayloadSizes()
MOTOR_LIMITS = MotorLimits()

# Backward compatibility - keep old constants as aliases
DEFAULT_MOTOR_PORT = MOTOR_DEFAULTS.port
DEFAULT_MOTOR_BAUDRATE = MOTOR_DEFAULTS.baudrate
DEFAULT_STEP_DELAY = MOTOR_DEFAULTS.step_delay
FRAME_START_BYTE = FRAME_BYTES.start
FRAME_END_BYTE = FRAME_BYTES.end
CRC_INITIAL_VALUE = CRC_CONSTANTS.initial_value
CRC_BYTE_MASK = CRC_CONSTANTS.byte_mask
CRC_WORD_MASK = CRC_CONSTANTS.word_mask
CRC_SHIFT_BITS = CRC_CONSTANTS.shift_bits
TEMP_SCALE_FACTOR = SCALE_FACTORS.temperature
CURRENT_SCALE_FACTOR = SCALE_FACTORS.current
DUTY_SCALE_FACTOR = SCALE_FACTORS.duty
VOLTAGE_SCALE_FACTOR = SCALE_FACTORS.voltage
VD_VQ_SCALE_FACTOR = SCALE_FACTORS.vd_vq
POSITION_SCALE_FACTOR = SCALE_FACTORS.position
TEMPERATURE_SIZE = PAYLOAD_SIZES.temperature
CURRENT_SIZE = PAYLOAD_SIZES.current
DUTY_SPEED_VOLTAGE_SIZE = PAYLOAD_SIZES.duty_speed_voltage
STATUS_POSITION_ID_SIZE = PAYLOAD_SIZES.status_position_id
RESERVED_SIZE = PAYLOAD_SIZES.reserved
TEMP_RESERVED_SIZE = PAYLOAD_SIZES.temp_reserved
VOLTAGE_CONTROL_SIZE = PAYLOAD_SIZES.voltage_control
ENCODER_SIZE = PAYLOAD_SIZES.encoder
MAX_DUTY_CYCLE = MOTOR_LIMITS.max_duty_cycle
MIN_DUTY_CYCLE = MOTOR_LIMITS.min_duty_cycle
MAX_CURRENT_AMPS = MOTOR_LIMITS.max_current_amps
MIN_CURRENT_AMPS = MOTOR_LIMITS.min_current_amps
MAX_VELOCITY_ERPM = MOTOR_LIMITS.max_velocity_erpm
MIN_VELOCITY_ERPM = MOTOR_LIMITS.min_velocity_erpm
MAX_POSITION_DEGREES = MOTOR_LIMITS.max_position_degrees
MIN_POSITION_DEGREES = MOTOR_LIMITS.min_position_degrees

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
