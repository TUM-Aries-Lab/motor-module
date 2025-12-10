"""Unit tests for FaultCode enum."""

import pytest

from motor_python.definitions import FaultCode


class TestFaultCodeEnum:
    """Test cases for FaultCode enum."""

    def test_all_enum_values_defined(self) -> None:
        """Test that all expected fault codes are defined in the enum."""
        expected_codes = {
            0: "NONE",
            1: "OVER_VOLTAGE",
            2: "UNDER_VOLTAGE",
            3: "DRV",
            4: "ABS_OVER_CURRENT",
            5: "OVER_TEMP_FET",
            6: "OVER_TEMP_MOTOR",
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

        for value, name in expected_codes.items():
            fault_code = FaultCode(value)
            assert fault_code.name == name
            assert fault_code.value == value

    def test_enum_from_valid_integers(self) -> None:
        """Test that enum can be instantiated from valid integer values."""
        # Test a few representative values
        assert FaultCode(0) == FaultCode.NONE
        assert FaultCode(3) == FaultCode.DRV
        assert FaultCode(4) == FaultCode.ABS_OVER_CURRENT
        assert FaultCode(18) == FaultCode.UNBALANCED_CURRENTS

    def test_enum_from_invalid_integer_raises_error(self) -> None:
        """Test that invalid integer values raise ValueError."""
        with pytest.raises(ValueError):
            FaultCode(19)

        with pytest.raises(ValueError):
            FaultCode(-1)

        with pytest.raises(ValueError):
            FaultCode(100)

    def test_get_description_returns_formatted_strings(self) -> None:
        """Test that get_description() returns properly formatted descriptions."""
        # Test codes with descriptions
        assert FaultCode.DRV.get_description() == "DRV (Drive fault)"
        assert (
            FaultCode.ABS_OVER_CURRENT.get_description()
            == "ABS_OVER_CURRENT (Motor over-current)"
        )
        assert (
            FaultCode.OVER_TEMP_FET.get_description()
            == "OVER_TEMP_FET (MOS over-temperature)"
        )
        assert (
            FaultCode.OVER_TEMP_MOTOR.get_description()
            == "OVER_TEMP_MOTOR (Motor over-temperature)"
        )

        # Test codes without additional descriptions
        assert FaultCode.NONE.get_description() == "NONE"
        assert FaultCode.OVER_VOLTAGE.get_description() == "OVER_VOLTAGE"
        assert FaultCode.ENCODER_SPI.get_description() == "ENCODER_SPI"

    def test_get_description_covers_all_codes(self) -> None:
        """Test that get_description() works for all enum values."""
        for code in FaultCode:
            description = code.get_description()
            assert isinstance(description, str)
            assert len(description) > 0
            # Description should at least contain the enum name
            assert code.name in description

    def test_enum_comparison(self) -> None:
        """Test that enum values can be compared correctly."""
        assert FaultCode.NONE == FaultCode(0)
        assert FaultCode.DRV != FaultCode.ABS_OVER_CURRENT
        assert FaultCode.NONE < FaultCode.OVER_VOLTAGE
        assert FaultCode.UNBALANCED_CURRENTS > FaultCode.NONE

    def test_enum_in_collections(self) -> None:
        """Test that enum values work correctly in collections."""
        codes_set = {FaultCode.NONE, FaultCode.DRV, FaultCode.ABS_OVER_CURRENT}
        assert FaultCode.NONE in codes_set
        assert FaultCode.OVER_VOLTAGE not in codes_set

        codes_list = [FaultCode.NONE, FaultCode.DRV]
        assert codes_list[0] == FaultCode.NONE
        assert len(codes_list) == 2

    def test_enum_iteration(self) -> None:
        """Test that all enum members can be iterated."""
        all_codes = list(FaultCode)
        assert len(all_codes) == 19
        assert FaultCode.NONE in all_codes
        assert FaultCode.UNBALANCED_CURRENTS in all_codes

    def test_enum_attribute_access(self) -> None:
        """Test that enum members can be accessed by name."""
        assert FaultCode.NONE == FaultCode["NONE"]
        assert FaultCode.DRV == FaultCode["DRV"]
        assert FaultCode.UNBALANCED_CURRENTS == FaultCode["UNBALANCED_CURRENTS"]
