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

# Motor communication defaults
DEFAULT_MOTOR_PORT = "/dev/ttyTHS1"
DEFAULT_MOTOR_BAUDRATE = 921600
DEFAULT_STEP_DELAY = 0.05
MAX_COMMUNICATION_ATTEMPTS = 2  # Maximum attempts for communication checks
COMMUNICATION_RETRY_DELAY = 0.2  # Delay between retry attempts (seconds)

# Motor protocol frame bytes
FRAME_START_BYTE = 0xAA
FRAME_END_BYTE = 0xBB

# CRC16 calculation constants
CRC_INITIAL_VALUE = 0
CRC_BYTE_MASK = 0xFF
CRC_WORD_MASK = 0xFFFF
CRC_SHIFT_BITS = 8

# Motor status payload scaling factors
TEMP_SCALE_FACTOR = 10.0  # Temperature is sent as int16 * 10
CURRENT_SCALE_FACTOR = 100.0  # Current is sent as int32 * 100
DUTY_SCALE_FACTOR = 1000.0  # Duty cycle is sent as int16 * 1000
VOLTAGE_SCALE_FACTOR = 10.0  # Voltage is sent as int16 * 10
VD_VQ_SCALE_FACTOR = 1000.0  # Vd/Vq voltages are sent as int32 * 1000
POSITION_SCALE_FACTOR = 1_000_000.0  # Position is sent as int32 * 1,000,000

# Motor status payload structure sizes
TEMPERATURE_SIZE = 4  # 2 int16 values
CURRENT_SIZE = 16  # 4 int32 values
DUTY_SPEED_VOLTAGE_SIZE = 8  # 1 int16 + 1 int32 + 1 int16
STATUS_POSITION_ID_SIZE = 6  # 1 byte + 1 float + 1 byte
RESERVED_SIZE = 24  # Reserved bytes
TEMP_RESERVED_SIZE = 6  # Temperature reserved bytes
VOLTAGE_CONTROL_SIZE = 12  # 2 int32 + 1 int32
ENCODER_SIZE = 8  # 2 float values

# Motor control limits
MAX_DUTY_CYCLE = 0.95  # Maximum duty cycle (95% to prevent saturation)
MIN_DUTY_CYCLE = -0.95  # Minimum duty cycle
MAX_CURRENT_AMPS = 60.0  # Maximum current in amps
MIN_CURRENT_AMPS = -60.0  # Minimum current in amps
MAX_VELOCITY_ERPM = 100000  # Maximum velocity in ERPM
MIN_VELOCITY_ERPM = -100000  # Minimum velocity in ERPM
MAX_POSITION_DEGREES = (
    360.0  # Maximum position: 1 full rotation (safe for exosuit joints)
)
MIN_POSITION_DEGREES = -360.0  # Minimum position: -1 full rotation


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
