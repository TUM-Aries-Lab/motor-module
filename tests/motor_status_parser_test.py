"""Unit tests for motor status parser."""

import struct

import pytest

from motor_python.motor_status_parser import (
    MotorStatusParser,
)


class TestMotorStatusParser:
    """Test suite for MotorStatusParser class."""

    def setup_method(self):
        """Set up test fixtures."""
        self.parser = MotorStatusParser()

    def test_parse_temperatures(self):
        """Test parsing temperature data."""
        # MOS temp: 45.5°C, Motor temp: 60.2°C (scaled by 10)
        payload = struct.pack(">hh", 455, 602)

        temps = self.parser.parse_temperatures(payload)

        assert temps is not None
        assert temps.mos_temp_celsius == pytest.approx(45.5, rel=1e-2)
        assert temps.motor_temp_celsius == pytest.approx(60.2, rel=1e-2)

    def test_parse_temperatures_insufficient_data(self):
        """Test parsing temperatures with insufficient data."""
        payload = b"\x00\x01"  # Only 2 bytes, need 4

        temps = self.parser.parse_temperatures(payload)

        assert temps is None

    def test_parse_currents(self):
        """Test parsing current data."""
        # Output: 12.34A, Input: 11.22A, Id: 5.67A, Iq: 8.90A (scaled by 100)
        payload = struct.pack(">iiii", 1234, 1122, 567, 890)

        currents = self.parser.parse_currents(payload)

        assert currents is not None
        assert currents.output_current_amps == pytest.approx(12.34, rel=1e-2)
        assert currents.input_current_amps == pytest.approx(11.22, rel=1e-2)
        assert currents.id_current_amps == pytest.approx(5.67, rel=1e-2)
        assert currents.iq_current_amps == pytest.approx(8.90, rel=1e-2)

    def test_parse_duty_speed_voltage(self):
        """Test parsing duty cycle, speed, and voltage."""
        # Duty: 0.5 (50%), Speed: 10000 ERPM, Voltage: 48.0V
        payload = struct.pack(">hih", 500, 10000, 480)

        dsv = self.parser.parse_duty_speed_voltage(payload)

        assert dsv is not None
        assert dsv.duty_cycle_percent == pytest.approx(0.5, rel=1e-3)
        assert dsv.speed_electrical_rpm == 10000
        assert dsv.voltage_volts == pytest.approx(48.0, rel=1e-2)

    def test_parse_status_position(self):
        """Test parsing status, position, and motor ID."""
        # Status: 0 (NONE), Position: 180.5°, Motor ID: 1
        payload = struct.pack(">Bfb", 0, 180.5, 1)

        status_pos = self.parser.parse_status_position(payload)

        assert status_pos is not None
        assert status_pos.status_code == 0
        assert status_pos.status_name == "NONE"
        assert status_pos.position_degrees == pytest.approx(180.5, rel=1e-2)
        assert status_pos.motor_id == 1

    def test_parse_status_position_unknown_fault(self):
        """Test parsing status with unknown fault code."""
        # Status: 99 (unknown), Position: 0.0°, Motor ID: 1
        payload = struct.pack(">Bfb", 99, 0.0, 1)

        status_pos = self.parser.parse_status_position(payload)

        assert status_pos is not None
        assert status_pos.status_code == 99
        assert status_pos.status_name == "UNKNOWN(99)"

    def test_parse_voltages_control(self):
        """Test parsing Vd/Vq voltages and control mode."""
        # Vd: 24.567V, Vq: 12.345V, Control mode: 2
        payload = struct.pack(">iii", 24567, 12345, 2)

        volt_ctrl = self.parser.parse_voltages_control(payload)

        assert volt_ctrl is not None
        assert volt_ctrl.vd_volts == pytest.approx(24.567, rel=1e-3)
        assert volt_ctrl.vq_volts == pytest.approx(12.345, rel=1e-3)
        assert volt_ctrl.control_mode == 2

    def test_parse_encoders(self):
        """Test parsing encoder angles."""
        # Encoder: 90.5°, Outer encoder: 270.25°
        payload = struct.pack(">ff", 90.5, 270.25)

        encoders = self.parser.parse_encoders(payload)

        assert encoders is not None
        assert encoders.encoder_angle_degrees == pytest.approx(90.5, rel=1e-2)
        assert encoders.outer_encoder_degrees == pytest.approx(270.25, rel=1e-2)

    def test_parse_full_status(self):
        """Test parsing complete motor status."""
        # Build a complete status payload
        payload = b""

        # Temperatures (4 bytes)
        payload += struct.pack(">hh", 450, 600)  # 45.0°C, 60.0°C

        # Currents (16 bytes)
        payload += struct.pack(">iiii", 1000, 900, 500, 600)  # 10A, 9A, 5A, 6A

        # Duty, Speed, Voltage (8 bytes)
        payload += struct.pack(">hih", 500, 10000, 480)  # 0.5, 10000, 48V

        # Reserved (24 bytes)
        payload += b"\x00" * 24

        # Status, Position, Motor ID (6 bytes)
        payload += struct.pack(">Bfb", 0, 90.0, 1)

        # Temperature reserved (6 bytes)
        payload += b"\x00" * 6

        # Voltages and Control Mode (12 bytes)
        payload += struct.pack(">iii", 24000, 12000, 1)

        # Encoder angles (8 bytes)
        payload += struct.pack(">ff", 90.0, 270.0)

        status = self.parser.parse_full_status(payload)

        assert status is not None
        assert status.temperatures.mos_temp_celsius == pytest.approx(45.0, rel=1e-2)
        assert status.temperatures.motor_temp_celsius == pytest.approx(60.0, rel=1e-2)
        assert status.currents.output_current_amps == pytest.approx(10.0, rel=1e-2)
        assert status.duty_speed_voltage.speed_electrical_rpm == 10000
        assert status.status_position.position_degrees == pytest.approx(90.0, rel=1e-2)
        assert status.voltages_control.control_mode == 1
        assert status.encoders.encoder_angle_degrees == pytest.approx(90.0, rel=1e-2)

    def test_parse_full_status_incomplete_data(self):
        """Test parsing with incomplete payload data."""
        # Only 10 bytes, not enough for full status
        payload = b"\x00" * 10

        status = self.parser.parse_full_status(payload)

        assert status is None

    def test_offset_reset_on_full_parse(self):
        """Test that offset is properly reset when parsing full status."""
        # Parse once
        payload = struct.pack(">hh", 450, 600) + b"\x00" * 80
        self.parser.parse_full_status(payload)

        # After parse_full_status, offset is at the end
        # Reset it manually to parse from the beginning again
        self.parser.payload_offset = 0
        temps = self.parser.parse_temperatures(payload)
        assert temps is not None
        assert temps.mos_temp_celsius == pytest.approx(45.0, rel=1e-2)
