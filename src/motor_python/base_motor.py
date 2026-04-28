"""Abstract base class for CubeMars AK60-6 motor controllers.

Provides the shared interface, safety checks, and higher-level methods
that are common to both the UART and CAN implementations.  Subclasses
must implement the abstract transport-layer methods.
"""

from __future__ import annotations

import abc
import time
from dataclasses import dataclass
from typing import Self

import numpy as np
from loguru import logger

from motor_python.definitions import (
    MOTOR_DEFAULTS,
    MOTOR_LIMITS,
    TendonAction,
)
from motor_python.utils import erpm_to_degrees_per_second

# Error code descriptions from CubeMars CAN protocol spec (section 4.3.1)
CAN_ERROR_CODES: dict[int, str] = {
    0: "No fault",
    1: "Motor over-temperature",
    2: "Over-current",
    3: "Over-voltage",
    4: "Under-voltage",
    5: "Encoder fault",
    6: "MOSFET over-temperature",
    7: "Motor lock-up",
}


@dataclass
class MotorState:
    """Motor feedback data (position, speed, current, temp, error).

    Replaces CANMotorFeedback so both UART and CAN implementations can
    return identical telemetry objects.
    """

    position_degrees: float  # Motor position in degrees
    speed_erpm: int  # Electrical speed in ERPM
    current_amps: float  # Phase current in amps
    temperature_celsius: int  # Driver board temperature in °C
    error_code: int  # Fault code (0 = OK)

    @property
    def error_description(self) -> str:
        """Human-readable error description."""
        return CAN_ERROR_CODES.get(
            self.error_code, f"Unknown error ({self.error_code})"
        )


class BaseMotor(abc.ABC):
    """Abstract motor controller for CubeMars AK60-6.

    Concrete subclasses (UART / CAN) implement the ``_connect``,
    ``_send_command_*`` and ``_receive_*`` methods.  Everything else —
    velocity safety checks, tendon helpers, context-manager protocol,
    movement estimation — lives here exactly once.
    """

    def __init__(self) -> None:
        self.connected: bool = False
        self.communicating: bool = False
        self._consecutive_no_response: int = 0
        self._consecutive_invalid_response: int = 0
        self._max_no_response: int = MOTOR_DEFAULTS.max_no_response_attempts

    # ------------------------------------------------------------------
    # Abstract transport-layer methods (must be implemented by subclasses)
    # ------------------------------------------------------------------

    @abc.abstractmethod
    def _connect(self) -> None:
        """Establish the physical connection (serial / CAN bus)."""

    @abc.abstractmethod
    def set_position(self, position_degrees: float) -> None:
        """Move the motor to a target angle and hold it there.

        :param position_degrees: Target angle in degrees.
        """
        raise NotImplementedError("set_position is not implemented")

    def get_position(self) -> float | None:
        """Return the current motor angle in degrees.

        Reads from the most recent motor status.
        """
        status = self.get_status()
        return status.position_degrees if status else None

    @abc.abstractmethod
    def set_origin(self, permanent: bool = False) -> None:
        """Set the current position as the zero reference.

        :param permanent: If True, save to non-volatile memory.
        """

    @abc.abstractmethod
    def get_status(self) -> MotorState | None:
        """Read and return the full motor state."""

    @abc.abstractmethod
    def check_communication(self) -> bool:
        """Verify the motor is alive and responding.

        :return: True if the motor responds within the retry window.
        """

    @abc.abstractmethod
    def stop(self) -> None:
        """Stop the motor immediately and release the windings."""

    @abc.abstractmethod
    def _stop_motor_transport(self) -> None:
        """Transport-specific cleanup when closing (stop motor, release bus/serial)."""

    @abc.abstractmethod
    def _soft_start(self, direction: int) -> None:
        """Pre-spin motor with gentle current to pass the noisy low-speed zone.

        :param direction: 1 for forward, -1 for reverse.
        """

    @abc.abstractmethod
    def _send_velocity_command(self, velocity_erpm: int) -> None:
        """Send the transport-specific velocity command after safety checks.

        :param velocity_erpm: Validated and clamped velocity in ERPM.
        """

    # ------------------------------------------------------------------
    # Unified Advanced Control Methods (Stubbed by default)
    # ------------------------------------------------------------------

    def enable_motor(self) -> None:  # noqa: B027
        """Power on the motor and enter active mode.

        Required for CAN before sending commands. UART typically ignores this.
        """
        pass

    def disable_motor(self) -> None:
        """Power off the motor so it coasts."""
        self.stop()

    def set_current(self, current_amps: float) -> None:
        """Command a specific motor torque via direct current control."""
        raise NotImplementedError("set_current is not supported or implemented")

    def set_brake_current(self, current_amps: float) -> None:
        """Hold the motor shaft in place using brake current."""
        raise NotImplementedError("set_brake_current is not supported or implemented")

    def set_duty_cycle(self, duty: float) -> None:
        """Apply a raw PWM voltage to the motor windings."""
        raise NotImplementedError("set_duty_cycle is not supported or implemented")

    def set_position_velocity_accel(
        self,
        position_degrees: float,
        velocity_erpm: int,
        accel_erpm_per_sec: int = 0,
    ) -> None:
        """Move to a target angle with controlled speed and acceleration."""
        raise NotImplementedError(
            "set_position_velocity_accel is not supported or implemented"
        )

    def enable_mit_mode(self) -> None:
        """Switch the motor from Servo mode to MIT impedance mode."""
        raise NotImplementedError("MIT mode is only supported on CAN transport")

    def disable_mit_mode(self) -> None:
        """Exit MIT mode and cut motor drive output."""
        raise NotImplementedError("MIT mode is only supported on CAN transport")

    def set_mit_mode(
        self,
        pos_rad: float,
        vel_rad_s: float = 0.0,
        kp: float = 0.0,
        kd: float = 0.0,
        torque_ff_nm: float = 0.0,
    ) -> None:
        """Send an impedance control command using the MIT actuator protocol."""
        raise NotImplementedError("MIT mode is only supported on CAN transport")

    # ------------------------------------------------------------------
    # Unified Telemetry Methods
    # ------------------------------------------------------------------

    def get_temperature(self) -> int | None:
        """Return the motor driver board temperature in °C."""
        status = self.get_status()
        return status.temperature_celsius if status else None

    def get_current(self) -> float | None:
        """Return the phase current draw in amps."""
        status = self.get_status()
        return status.current_amps if status else None

    def get_speed(self) -> int | None:
        """Return the current speed in electrical RPM."""
        status = self.get_status()
        return status.speed_erpm if status else None

    def get_motor_data(self) -> dict | None:
        """Return all motor telemetry as a dictionary."""
        status = self.get_status()
        if not status:
            return None
        return {
            "position_degrees": status.position_degrees,
            "speed_erpm": status.speed_erpm,
            "current_amps": status.current_amps,
            "temperature_celsius": status.temperature_celsius,
            "error_code": status.error_code,
            "error_description": status.error_description,
        }

    # ------------------------------------------------------------------
    # Concrete shared methods
    # ------------------------------------------------------------------

    def set_velocity(self, velocity_erpm: int) -> None:
        """Set motor velocity in electrical RPM.

        :param velocity_erpm: Target velocity in ERPM.  Negative = reverse.
        """
        velocity_erpm_int = int(velocity_erpm)

        # Velocity 0 means stop
        if velocity_erpm_int == 0:
            self.stop()
            return

        # Clamp to protocol limits — subclass may further narrow the range.
        velocity_erpm = int(
            np.clip(
                velocity_erpm_int,
                MOTOR_LIMITS.min_velocity_electrical_rpm,
                MOTOR_LIMITS.max_velocity_electrical_rpm,
            )
        )
        if velocity_erpm != velocity_erpm_int:
            logger.warning(
                f"Velocity {velocity_erpm_int} ERPM clamped to {velocity_erpm} ERPM"
            )

        # Soft-start: pre-spin with current to avoid noisy low-speed zone
        direction = 1 if velocity_erpm > 0 else -1
        self._pre_velocity_hook(velocity_erpm, direction)

        # Hand off to the transport layer
        self._send_velocity_command(velocity_erpm)

    def _pre_velocity_hook(self, velocity_erpm: int, direction: int) -> None:
        """Call hook before the velocity command is sent.

        Subclasses may override to e.g. skip soft-start when already in
        velocity mode (CAN) or always soft-start (UART).  Default
        implementation always soft-starts.
        """
        self._soft_start(direction)

    def control_exosuit_tendon(
        self,
        action: TendonAction,
        velocity_erpm: int = MOTOR_LIMITS.default_tendon_velocity_erpm,
    ) -> None:
        """Control exosuit tendon using safe velocity commands.

        :param action: TendonAction.PULL, RELEASE, or STOP.
        :param velocity_erpm: Speed for PULL / RELEASE in ERPM.
        :raises ValueError: If action is not a valid TendonAction.
        """
        if action == TendonAction.PULL:
            logger.info(f"Pulling tendon at {velocity_erpm} ERPM")
            self.set_velocity(velocity_erpm=abs(velocity_erpm))
        elif action == TendonAction.RELEASE:
            logger.info(f"Releasing tendon at {velocity_erpm} ERPM")
            self.set_velocity(velocity_erpm=-abs(velocity_erpm))
        elif action == TendonAction.STOP:
            logger.info("Stopping tendon motion")
            self.stop()
        else:
            raise ValueError(
                f"Invalid action {action}. Use TendonAction.PULL, "
                f"TendonAction.RELEASE, or TendonAction.STOP"
            )

    def _estimate_movement_time(
        self, target_degrees: float, motor_speed_erpm: int
    ) -> float:
        """Estimate time needed to reach target position at given speed.

        :param target_degrees: Target position in degrees.
        :param motor_speed_erpm: Motor speed in ERPM (absolute value used).
        :return: Estimated travel time in seconds.
        """
        if motor_speed_erpm == 0:
            return 0.0

        current_position = self._get_current_position_for_estimate()

        degrees_per_second = erpm_to_degrees_per_second(motor_speed_erpm)
        distance = abs(target_degrees - current_position)
        return distance / degrees_per_second

    def _get_current_position_for_estimate(self) -> float:
        """Return current position as float for movement estimation.

        Subclasses override to obtain position from their transport layer.
        Default returns 0.0.
        """
        return 0.0

    def move_to_position_with_speed(
        self,
        target_degrees: float,
        motor_speed_erpm: int,
    ) -> None:
        """Drive to a target angle at a given speed, then hold.

        Uses velocity control to move toward the target, waits for
        the estimated travel time, then switches to position hold.

        :param target_degrees: Target angle in degrees.
        :param motor_speed_erpm: Travel speed in ERPM.
        """
        current_pos = self._get_current_position_for_estimate()
        direction = 1 if target_degrees > current_pos else -1

        self.set_velocity(velocity_erpm=motor_speed_erpm * direction)

        estimated_time = self._estimate_movement_time(target_degrees, motor_speed_erpm)
        time.sleep(min(estimated_time, MOTOR_LIMITS.max_movement_time))

        self.set_position(target_degrees)
        logger.info(
            f"Reached position: {target_degrees:.1f}° at {motor_speed_erpm} ERPM"
        )

    def close(self) -> None:
        """Stop the motor and release the connection.

        Safe to call multiple times.  Called automatically by ``__exit__``.
        """
        self._stop_motor_transport()

    def __enter__(self) -> Self:
        """Context manager support."""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        """Context manager exit."""
        self.close()

    def get_timing_stats(self) -> dict:
        """Return Refresh Loop Statistics.

        Reports:
        - Loop timing statistics (mean dt, std dt, min dt, max dt, and the effective Hz (1/mean_dt))
        - Jitter count (Counts how many intervals exceed 2x the expected period)
        - Cumulative send failures and missed feedback frames (never resets)
        - CAN error counter deltas (current tx_err/rx_err vs values at start)
        - TX pacing metrics (if available)

        :return: Dictionary with timing statistics, or minimal dict if unavailable.
        """
        stats = {
            "method": "get_timing_stats",
            "available": False,
        }

        # Trying to get loop timing from refresh timestamps
        refresh_timestamps = getattr(self, "_refresh_timestamps", None)
        if refresh_timestamps is not None:
            timestamps = np.array(refresh_timestamps)
            if len(timestamps) > 1:
                expected_period = (
                    1.0 / getattr(self, "_refresh_interval", 0.01)
                    if hasattr(self, "_refresh_interval")
                    else 0.01
                )
                dts = np.diff(timestamps)
                threshold = 2.0 * expected_period
                jitter_count = np.sum(
                    dts > threshold
                )  # Counting intervals that exceed the expected threshold

                stats.update(
                    {
                        "available": True,
                        "loop_period_expected_s": expected_period,
                        "loop_period_mean_s": float(np.mean(dts)),
                        "loop_period_std_s": float(np.std(dts)),
                        "loop_period_min_s": float(np.min(dts)),
                        "loop_period_max_s": float(np.max(dts)),
                        "loop_effective_hz": 1.0 / float(np.mean(dts)),
                        "loop_intervals_total": len(dts),
                        "loop_jitter_count": int(jitter_count),
                        "loop_jitter_ratio": float(
                            jitter_count / len(dts)
                        )  # delete if unnecessary
                        if len(dts) > 0
                        else 0.0,
                    }
                )

        # Cumulative send failures
        cumulative_send_failures = getattr(
            self, "_cumulative_refresh_send_failures", None
        )
        if cumulative_send_failures is not None:
            stats["cumulative_send_failures"] = cumulative_send_failures

        # Cumulative missed feedback
        cumulative_no_feedback = getattr(self, "_cumulative_refresh_no_feedback", None)
        if cumulative_no_feedback is not None:
            stats["cumulative_missed_feedback"] = cumulative_no_feedback

        # CAN error counter deltas
        initial_can_state = getattr(self, "_initial_can_state", None)
        last_can_state_cache = getattr(self, "_last_can_state_cache", None)
        if initial_can_state is not None and last_can_state_cache is not None:
            stats["can_tx_err_initial"] = initial_can_state.get("tx_err", 0)
            stats["can_tx_err_final"] = last_can_state_cache.get("tx_err", 0)
            stats["can_tx_err_delta"] = last_can_state_cache.get(
                "tx_err", 0
            ) - initial_can_state.get("tx_err", 0)
            stats["can_rx_err_initial"] = initial_can_state.get("rx_err", 0)
            stats["can_rx_err_final"] = last_can_state_cache.get("rx_err", 0)
            stats["can_rx_err_delta"] = last_can_state_cache.get(
                "rx_err", 0
            ) - initial_can_state.get("rx_err", 0)

        # TX pacing metrics tells how often pacing delays were needed and how much time was spent sleeping to pace transmissions.
        tx_pace_sleep_count = getattr(self, "_tx_pace_sleep_count", None)
        if tx_pace_sleep_count is not None:
            stats["tx_pace_sleep_count"] = tx_pace_sleep_count
        tx_pace_sleep_time_s = getattr(self, "_tx_pace_sleep_time_s", None)
        if tx_pace_sleep_time_s is not None:
            stats["tx_pace_sleep_time_s"] = tx_pace_sleep_time_s

        return stats
