"""MotorManager --> for multiple CubeMars AK60-6 CAN motors.

The manager creates one `CubeMarsAK606v3CAN` instance per motor ID and
provides label-based access, lifecycle helpers, and context-manager cleanup.
_closed and _motors are accessed without locks, so this class is not thread-safe. Use a lock later if you need to access it from multiple threads.
"""

from __future__ import annotations

from collections.abc import Iterator, Mapping

from loguru import logger

from motor_python.cube_mars_motor_can import CubeMarsAK606v3CAN


class MotorManager:
    """Manages multiple motors on a single CAN bus."""

    def __init__(
        self,
        motor_ids: list[int],
        interface: str = "can0",
        labels: Mapping[str, int] | None = None,
    ) -> None:
        # Checking if all motor IDs are unique
        if len(motor_ids) != len(set(motor_ids)):
            raise ValueError("motor_ids contains duplicate CAN IDs")

        self.interface = interface
        self._motors: dict[
            int, CubeMarsAK606v3CAN
        ] = {}  # Maps CAN ID to motor instance
        self._label_to_id: dict[str, int] = {}  # Maps label to CAN ID
        self._closed = False

        if labels is not None:
            self._label_to_id = dict(labels)
            for label, motor_id in self._label_to_id.items():
                if motor_id not in motor_ids:
                    raise ValueError(
                        f"Label '{label}' references unknown motor ID 0x{motor_id:02X}"
                    )

        for motor_id in motor_ids:
            self._motors[motor_id] = CubeMarsAK606v3CAN(
                motor_can_id=motor_id,
                interface=interface,
            )

    def __getitem__(self, key: int | str) -> CubeMarsAK606v3CAN:
        """Return a motor instance by CAN ID or label."""
        if isinstance(key, str):  # Label-based access
            if key not in self._label_to_id:
                raise KeyError(f"Unknown motor label: {key}")
            return self._motors[self._label_to_id[key]]

        if isinstance(key, int):  # CAN ID-based access
            if key not in self._motors:
                raise KeyError(f"Unknown motor CAN ID: 0x{key:02X}")
            return self._motors[key]

        raise KeyError(f"Unsupported motor key type: {type(key)}")

    def __len__(self) -> int:
        """Return the number of managed motors."""
        return len(self._motors)

    def __iter__(self) -> Iterator[CubeMarsAK606v3CAN]:
        """Iterate over managed motor instances."""
        return iter(self._motors.values())

    def __contains__(self, key: object) -> bool:
        """Return whether the manager contains a motor by ID or label."""
        if isinstance(key, str):  # Label-based check
            return key in self._label_to_id
        if isinstance(key, int):  # CAN ID-based check
            return key in self._motors
        return False

    @classmethod
    def discover(cls, interface: str = "can0") -> MotorManager:
        """Discover available CAN motors on the configured SocketCAN interface."""
        discovered_ids: list[int] = []

        for motor_id in range(1, 17):  # scan CAN IDs 0x01 to 0x10 (1-16)
            try:
                with CubeMarsAK606v3CAN(
                    motor_can_id=motor_id, interface=interface
                ) as motor:
                    if motor.check_communication():
                        discovered_ids.append(motor_id)
                # Exiting the "with" block will automatically close the motor connection (BaseMotor.__exit__)
            except Exception as exc:
                logger.debug(f"Motor discovery failed for ID 0x{motor_id:02X}: {exc}")

        if not discovered_ids:
            raise RuntimeError(f"No CAN motors discovered on interface '{interface}'")

        logger.info("Discovered {} motors: {}", len(discovered_ids), discovered_ids)
        return cls(motor_ids=discovered_ids, interface=interface)

    def enable_all(self) -> None:
        """Enable all managed motors."""
        for motor in self._motors.values():
            motor.enable_motor()

    def disable_all(self) -> None:
        """Disable all managed motors."""
        for motor in self._motors.values():
            motor.disable_motor()

    def stop_all(self) -> None:
        """Stop all managed motors and collect any stop errors."""
        errors: list[Exception] = []
        for motor in self._motors.values():
            try:
                motor.stop()
            except Exception as exc:
                motor_id = getattr(motor, "motor_can_id", None)
                if isinstance(motor_id, int):
                    logger.exception(f"Motor 0x{motor_id:02X} failed to stop")
                else:
                    logger.exception("Motor failed to stop")
                errors.append(exc)

        if errors:
            raise ExceptionGroup("MotorManager.stop_all failed", errors)

    def check_all(self) -> dict[int, bool]:
        """Check communication for all managed motors."""
        return {
            motor_id: motor.check_communication()
            for motor_id, motor in self._motors.items()
        }

    def close(self) -> None:
        """Close all managed motors and release the CAN interface."""
        if self._closed:  # Already closed, do nothing
            return

        errors: list[Exception] = []
        for motor in self._motors.values():
            try:
                motor.close()
            except Exception as exc:
                motor_id = getattr(motor, "motor_can_id", None)
                if isinstance(motor_id, int):
                    logger.exception(f"Motor 0x{motor_id:02X} failed to close")
                else:
                    logger.exception("Motor failed to close")
                errors.append(exc)

        self._closed = True  # Mark as closed to prevent future operations
        if errors:
            raise ExceptionGroup("MotorManager.close failed", errors)

    def __enter__(self) -> MotorManager:
        """Context manager entry - returns self for use within the block."""
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc_value: BaseException | None,
        traceback: object | None,
    ) -> None:
        """Context manager exit - ensures all motors are closed."""
        self.stop_all()  # Attempt to stop all motors first
        self.close()
