"""Motor status payload parser for CubeMars protocol."""

import struct
from dataclasses import dataclass

from loguru import logger

from motor_python.definitions import (
    CURRENT_SCALE_FACTOR,
    CURRENT_SIZE,
    DUTY_SCALE_FACTOR,
    DUTY_SPEED_VOLTAGE_SIZE,
    ENCODER_SIZE,
    FAULT_CODES,
    RESERVED_SIZE,
    STATUS_POSITION_ID_SIZE,
    TEMP_RESERVED_SIZE,
    TEMP_SCALE_FACTOR,
    TEMPERATURE_SIZE,
    VD_VQ_SCALE_FACTOR,
    VOLTAGE_CONTROL_SIZE,
    VOLTAGE_SCALE_FACTOR,
)


@dataclass
class MotorTemperatures:
    """Motor temperature readings."""

    mos_temp_celsius: float
    motor_temp_celsius: float


@dataclass
class MotorCurrents:
    """Motor current readings."""

    output_current_amps: float
    input_current_amps: float
    id_current_amps: float
    iq_current_amps: float


@dataclass
class MotorDutySpeedVoltage:
    """Motor duty cycle, speed, and voltage readings."""

    duty_cycle_percent: float
    speed_erpm: int
    voltage_volts: float


@dataclass
class MotorStatusPosition:
    """Motor status, position, and ID."""

    status_code: int
    status_name: str
    position_degrees: float
    motor_id: int


@dataclass
class MotorVoltagesControl:
    """Motor Vd/Vq voltages and control mode."""

    vd_volts: float
    vq_volts: float
    control_mode: int


@dataclass
class MotorEncoders:
    """Motor encoder readings."""

    encoder_angle_degrees: float
    outer_encoder_degrees: float


@dataclass
class MotorStatus:
    """Complete motor status data."""

    temperatures: MotorTemperatures
    currents: MotorCurrents
    duty_speed_voltage: MotorDutySpeedVoltage
    status_position: MotorStatusPosition
    voltages_control: MotorVoltagesControl
    encoders: MotorEncoders


class MotorStatusParser:
    """Parser for motor status payload data."""

    def __init__(self) -> None:
        """Initialize the parser.

        :return: None
        """
        self.payload_offset = 0

    def _read_int16(self, payload: bytes) -> int:
        """Read a signed 16-bit integer in big-endian format.

        :param payload: Payload bytes to read from.
        :return: Signed 16-bit integer value.
        """
        value = struct.unpack(
            ">h", payload[self.payload_offset : self.payload_offset + 2]
        )[0]
        self.payload_offset += 2
        return value

    def _read_int32(self, payload: bytes) -> int:
        """Read a signed 32-bit integer in big-endian format.

        :param payload: Payload bytes to read from.
        :return: Signed 32-bit integer value.
        """
        value = struct.unpack(
            ">i", payload[self.payload_offset : self.payload_offset + 4]
        )[0]
        self.payload_offset += 4
        return value

    def _read_float(self, payload: bytes) -> float:
        """Read a 32-bit float in big-endian format.

        :param payload: Payload bytes to read from.
        :return: Float value.
        """
        value = struct.unpack(
            ">f", payload[self.payload_offset : self.payload_offset + 4]
        )[0]
        self.payload_offset += 4
        return value

    def _read_byte(self, payload: bytes) -> int:
        """Read a single byte.

        :param payload: Payload bytes to read from.
        :return: Byte value.
        """
        value = payload[self.payload_offset]
        self.payload_offset += 1
        return value

    def _skip_bytes(self, count: int) -> None:
        """Skip a number of bytes.

        :param count: Number of bytes to skip.
        :return: None
        """
        self.payload_offset += count

    def parse_temperatures(self, payload: bytes) -> MotorTemperatures | None:
        """Parse temperature data from payload.

        :param payload: Payload bytes.
        :return: MotorTemperatures object or None if insufficient data.
        """
        if len(payload) < self.payload_offset + TEMPERATURE_SIZE:
            return None

        mos_temp = self._read_int16(payload) / TEMP_SCALE_FACTOR
        motor_temp = self._read_int16(payload) / TEMP_SCALE_FACTOR

        return MotorTemperatures(
            mos_temp_celsius=mos_temp,
            motor_temp_celsius=motor_temp,
        )

    def parse_currents(self, payload: bytes) -> MotorCurrents | None:
        """Parse current data from payload.

        :param payload: Payload bytes.
        :return: MotorCurrents object or None if insufficient data.
        """
        if len(payload) < self.payload_offset + CURRENT_SIZE:
            return None

        output_current = self._read_int32(payload) / CURRENT_SCALE_FACTOR
        input_current = self._read_int32(payload) / CURRENT_SCALE_FACTOR
        id_current = self._read_int32(payload) / CURRENT_SCALE_FACTOR
        iq_current = self._read_int32(payload) / CURRENT_SCALE_FACTOR

        return MotorCurrents(
            output_current_amps=output_current,
            input_current_amps=input_current,
            id_current_amps=id_current,
            iq_current_amps=iq_current,
        )

    def parse_duty_speed_voltage(self, payload: bytes) -> MotorDutySpeedVoltage | None:
        """Parse duty cycle, speed, and voltage data from payload.

        :param payload: Payload bytes.
        :return: MotorDutySpeedVoltage object or None if insufficient data.
        """
        if len(payload) < self.payload_offset + DUTY_SPEED_VOLTAGE_SIZE:
            return None

        duty = self._read_int16(payload) / DUTY_SCALE_FACTOR
        speed = self._read_int32(payload)
        voltage = self._read_int16(payload) / VOLTAGE_SCALE_FACTOR

        return MotorDutySpeedVoltage(
            duty_cycle_percent=duty,
            speed_erpm=speed,
            voltage_volts=voltage,
        )

    def parse_status_position(self, payload: bytes) -> MotorStatusPosition | None:
        """Parse status, position, and motor ID from payload.

        :param payload: Payload bytes.
        :return: MotorStatusPosition object or None if insufficient data.
        """
        if len(payload) < self.payload_offset + STATUS_POSITION_ID_SIZE:
            return None

        status = self._read_byte(payload)
        status_name = FAULT_CODES.get(status, f"UNKNOWN({status})")
        position = self._read_float(payload)
        motor_id = self._read_byte(payload)

        return MotorStatusPosition(
            status_code=status,
            status_name=status_name,
            position_degrees=position,
            motor_id=motor_id,
        )

    def parse_voltages_control(self, payload: bytes) -> MotorVoltagesControl | None:
        """Parse Vd/Vq voltages and control mode from payload.

        :param payload: Payload bytes.
        :return: MotorVoltagesControl object or None if insufficient data.
        """
        if len(payload) < self.payload_offset + VOLTAGE_CONTROL_SIZE:
            return None

        vd = self._read_int32(payload) / VD_VQ_SCALE_FACTOR
        vq = self._read_int32(payload) / VD_VQ_SCALE_FACTOR
        ctrl_mode = self._read_int32(payload)

        return MotorVoltagesControl(
            vd_volts=vd,
            vq_volts=vq,
            control_mode=ctrl_mode,
        )

    def parse_encoders(self, payload: bytes) -> MotorEncoders | None:
        """Parse encoder angles from payload.

        :param payload: Payload bytes.
        :return: MotorEncoders object or None if insufficient data.
        """
        if len(payload) < self.payload_offset + ENCODER_SIZE:
            return None

        enc_angle = self._read_float(payload)
        outer_enc = self._read_float(payload)

        return MotorEncoders(
            encoder_angle_degrees=enc_angle,
            outer_encoder_degrees=outer_enc,
        )

    def parse_full_status(self, payload: bytes) -> MotorStatus | None:
        """Parse complete motor status from payload.

        :param payload: Full status payload bytes.
        :return: MotorStatus object or None if parsing fails.
        """
        self.payload_offset = 0

        try:
            # Parse all components sequentially
            temperatures = self.parse_temperatures(payload)
            currents = self.parse_currents(payload)
            duty_speed_voltage = self.parse_duty_speed_voltage(payload)

            # Skip reserved bytes
            self._skip_bytes(RESERVED_SIZE)

            status_position = self.parse_status_position(payload)

            # Skip temperature reserved bytes
            self._skip_bytes(TEMP_RESERVED_SIZE)

            voltages_control = self.parse_voltages_control(payload)
            encoders = self.parse_encoders(payload)

            # Validate all components were parsed successfully
            if not all(
                [
                    temperatures,
                    currents,
                    duty_speed_voltage,
                    status_position,
                    voltages_control,
                    encoders,
                ]
            ):
                logger.warning("Failed to parse one or more motor status components")
                return None

            # Type checker assurance - all values are non-None after validation
            assert temperatures is not None
            assert currents is not None
            assert duty_speed_voltage is not None
            assert status_position is not None
            assert voltages_control is not None
            assert encoders is not None

            return MotorStatus(
                temperatures=temperatures,
                currents=currents,
                duty_speed_voltage=duty_speed_voltage,
                status_position=status_position,
                voltages_control=voltages_control,
                encoders=encoders,
            )

        except Exception as e:
            logger.error(f"Error parsing motor status: {e}")
            return None

    def log_motor_status(self, status: MotorStatus) -> None:
        """Log motor status in a readable format.

        :param status: MotorStatus object to log
        :return: None
        """
        logger.info(
            f"  MOS Temp: {status.temperatures.mos_temp_celsius:.1f}°C | "
            f"Motor Temp: {status.temperatures.motor_temp_celsius:.1f}°C"
        )
        logger.info(
            f"  Output: {status.currents.output_current_amps:.2f}A | "
            f"Input: {status.currents.input_current_amps:.2f}A"
        )
        logger.info(
            f"  Id: {status.currents.id_current_amps:.2f}A | "
            f"Iq: {status.currents.iq_current_amps:.2f}A"
        )
        logger.info(
            f"  Duty: {status.duty_speed_voltage.duty_cycle_percent:.1f}% | "
            f"Speed: {status.duty_speed_voltage.speed_erpm} ERPM | "
            f"Voltage: {status.duty_speed_voltage.voltage_volts:.1f}V"
        )
        logger.info(
            f"  Status: {status.status_position.status_code} - "
            f"{status.status_position.status_name}"
        )
        logger.info(
            f"  Position: {status.status_position.position_degrees:.2f}° | "
            f"Motor ID: {status.status_position.motor_id}"
        )
        logger.info(
            f"  Vd: {status.voltages_control.vd_volts:.3f}V | "
            f"Vq: {status.voltages_control.vq_volts:.3f}V | "
            f"Control Mode: {status.voltages_control.control_mode}"
        )
        logger.info(
            f"  Encoder: {status.encoders.encoder_angle_degrees:.2f}° | "
            f"Outer Encoder: {status.encoders.outer_encoder_degrees:.2f}°"
        )
