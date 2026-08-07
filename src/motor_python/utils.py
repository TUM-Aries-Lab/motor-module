"""Configure the logger."""

import csv
import sys
from collections.abc import Iterator, Sequence
from contextlib import contextmanager
from datetime import datetime
from pathlib import Path
from typing import Any

from loguru import logger

from motor_python import definitions
from motor_python.definitions import (
    DATE_FORMAT,
    DEFAULT_LOG_FILENAME,
    DEFAULT_LOG_LEVEL,
    ENCODING,
    LOG_DIR,
    MotorSpec,
)


def create_timestamped_filepath(suffix: str, output_dir: Path, prefix: str) -> Path:
    """Generate a timestamped filename.

    :param suffix: Suffix to append to the timestamped filename.
    :param output_dir: Output directory.
    :param prefix: Prefix to append to the timestamped filename.
    :return: Path to the timestamped filename.
    """
    timestamp = datetime.now().strftime(DATE_FORMAT)
    filepath = output_dir / f"{prefix}_{timestamp}.{suffix}"
    filepath.parent.mkdir(parents=True, exist_ok=True)  # create dirs if missing
    filepath.touch(exist_ok=True)  # create empty file (don't overwrite)
    return filepath


def setup_logger(
    filename: str = DEFAULT_LOG_FILENAME,
    stderr_level: str = DEFAULT_LOG_LEVEL,
    log_level: str = DEFAULT_LOG_LEVEL,
    log_dir: Path | None = None,
) -> Path:
    """Configure the logger.

    :param filename: Name of the file to create.
    :param stderr_level: Logging level to use.
    :param log_level: Logging level to use.
    :param log_dir: Logging directory to use.
    :return: Path to the created logfile.
    """
    logger.remove()

    if log_dir is None:
        log_filepath = LOG_DIR
    else:
        log_filepath = log_dir
    filepath_with_time = create_timestamped_filepath(
        output_dir=log_filepath, prefix=filename, suffix="log"
    )
    logger.add(sys.stderr, level=stderr_level)
    logger.add(filepath_with_time, level=log_level, encoding=ENCODING, enqueue=True)
    logger.info(f"Logging to '{filepath_with_time}'.")
    return filepath_with_time


def erpm_to_degrees_per_second(
    erpm: int | float,
    motor_spec: MotorSpec | None = None,
) -> float:
    """Convert Electrical RPM (ERPM) to output-shaft degrees per second.

    Formula: ERPM * 360 / (60 * pole_pairs * gear_ratio)
    :param erpm: Electrical RPM
    :param motor_spec: Optional motor hardware profile for modularity
    :return: Output-shaft degrees per second
    """
    motor_spec = definitions.CURRENT_MOTOR_SPEC if motor_spec is None else motor_spec
    return (
        abs(float(erpm))
        * 6.0
        / (float(motor_spec.pole_pairs) * float(motor_spec.gear_ratio))
    )


class CsvStreamWriter:
    """Small wrapper for writing dict rows to a CSV file with automatic flush."""

    def __init__(
        self, path: Path, fieldnames: Sequence[str], *, encoding: str = "utf-8"
    ) -> None:
        self.path = path
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.handle = path.open("w", newline="", encoding=encoding)
        self.writer = csv.DictWriter(self.handle, fieldnames=list(fieldnames))
        self.writer.writeheader()

    def writerow(self, row: dict[str, Any]) -> None:
        """Write a single row to the CSV file and flush."""
        self.writer.writerow(row)
        self.handle.flush()

    def writerows(self, rows: Sequence[dict[str, Any]]) -> None:
        """Write multiple rows to the CSV file and flush."""
        self.writer.writerows(rows)
        self.handle.flush()

    def close(self) -> None:
        """Close the CSV file."""
        self.handle.close()

    def __enter__(self) -> "CsvStreamWriter":
        """Return self for use in a context manager."""
        return self

    def __exit__(self, exc_type: object, exc: object, tb: object) -> None:
        """Close the CSV file on exit."""
        self.close()


@contextmanager
def open_csv_writer(
    path: Path,
    fieldnames: Sequence[str],
    *,
    encoding: str = "utf-8",
) -> Iterator[CsvStreamWriter]:
    """Open a CSV file for streaming dict rows with a shared header."""
    writer = CsvStreamWriter(path, fieldnames, encoding=encoding)
    try:
        yield writer
    finally:
        writer.close()


def write_summary_csv(path: Path, rows: Sequence[dict[str, Any]]) -> None:
    """Write summary rows to CSV."""
    if not rows:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("", encoding="utf-8")
        return
    fieldnames = list(rows[0].keys())
    with open_csv_writer(path, fieldnames) as writer:
        writer.writerows(rows)
