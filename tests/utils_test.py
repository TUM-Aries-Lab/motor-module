"""Test the utils module."""

from pathlib import Path
from tempfile import TemporaryDirectory

import pytest
from loguru import logger

from motor_python.definitions import CURRENT_MOTOR_SPEC, LogLevel
from motor_python.utils import (
    create_timestamped_filepath,
    erpm_to_degrees_per_second,
    float_to_uint,
    setup_logger,
    uint_to_float,
    write_summary_csv,
)


def test_logger_init() -> None:
    """Test logger initialization."""
    with TemporaryDirectory() as log_dir:
        log_dir_path = Path(log_dir)
        log_filepath = setup_logger(filename="log_file", log_dir=log_dir_path)
        assert Path(log_filepath).exists()
        # Release the sink before the directory is torn down. setup_logger
        # adds the file with enqueue=True, so loguru holds the handle open in
        # a writer thread. On Windows a directory containing an open file
        # cannot be deleted, so TemporaryDirectory raises WinError 32 on exit;
        # POSIX unlink() tolerates an open file, which is why this only bites
        # locally and never in the Linux CI.
        logger.remove()
    assert not Path(log_filepath).exists()


def test_log_level() -> None:
    """Test the log level."""
    # Act
    log_levels = list(LogLevel())

    # Assert
    assert type(log_levels) is list


def test_create_timestamped_filepath() -> None:
    """Test timestamped file creation."""
    with TemporaryDirectory() as tmp:
        output_dir = Path(tmp)

        filepath = create_timestamped_filepath(
            suffix="txt",
            output_dir=output_dir,
            prefix="test",
        )

        assert filepath.exists()
        assert filepath.suffix == ".txt"
        assert filepath.parent == output_dir
        assert filepath.name.startswith("test_")


def test_erpm_to_degrees_per_second() -> None:
    """Test ERPM to degrees/sec conversion."""
    motor_spec = CURRENT_MOTOR_SPEC

    expected = 1000 * 6.0 / (motor_spec.pole_pairs * motor_spec.gear_ratio)

    assert erpm_to_degrees_per_second(1000, motor_spec) == expected


def test_erpm_to_degrees_per_second_negative() -> None:
    """Negative ERPM should produce the same magnitude."""
    positive = erpm_to_degrees_per_second(1500)
    negative = erpm_to_degrees_per_second(-1500)

    assert positive == negative


def test_erpm_to_degrees_per_second_zero() -> None:
    """Zero ERPM should convert to zero."""
    assert erpm_to_degrees_per_second(0) == 0.0


def test_float_to_uint_clamps_to_range() -> None:
    """Values outside the range saturate at the integer bounds."""
    assert float_to_uint(-999.0, -1.0, 1.0, 12) == 0
    assert float_to_uint(999.0, -1.0, 1.0, 12) == (1 << 12) - 1


def test_uint_to_float_round_trip_midpoint() -> None:
    """The midpoint survives a float -> uint -> float round trip."""
    raw = float_to_uint(0.0, -12.56, 12.56, 16)
    value = uint_to_float(raw, -12.56, 12.56, 16)
    assert value == pytest.approx(0.0, abs=0.001)


def test_write_summary_csv() -> None:
    """Test writing summary CSV."""
    with TemporaryDirectory() as tmp:
        csv_path = Path(tmp) / "summary.csv"

        rows = [
            {"speed": 100, "current": 2.5},
            {"speed": 200, "current": 3.0},
        ]

        write_summary_csv(csv_path, rows)

        assert csv_path.exists()

        contents = csv_path.read_text(encoding="utf-8")
        assert "speed" in contents
        assert "current" in contents
        assert "100" in contents
        assert "200" in contents


def test_write_summary_csv_empty() -> None:
    """Writing an empty CSV should create an empty file."""
    with TemporaryDirectory() as tmp:
        csv_path = Path(tmp) / "empty.csv"

        write_summary_csv(csv_path, [])

        assert csv_path.exists()
        assert csv_path.read_text(encoding="utf-8") == ""
