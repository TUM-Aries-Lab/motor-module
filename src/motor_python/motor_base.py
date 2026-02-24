"""Abstract base class defining the common motor controller interface.

Both UART (CubeMarsAK606v3) and CAN (CubeMarsAK606v3CAN) implement this
interface so user code can swap between protocols without changing call sites.

Usage examples::

    # Works identically with UART or CAN
    motor: AbstractMotorController = CubeMarsAK606v3CAN()
    motor: AbstractMotorController = CubeMarsAK606v3()

    motor.enable_motor()
    motor.set_velocity(10000)
    motor.control_exosuit_tendon(TendonAction.PULL)
    motor.stop()
"""

from abc import ABC, abstractmethod

from motor_python.definitions import MOTOR_DEFAULTS, MOTOR_LIMITS, TendonAction


class AbstractMotorController(ABC):
    """Abstract base class for AK60-6 motor controllers.

    Defines the complete public interface shared by the UART and CAN
    implementations.  Any code that uses only these methods works with
    either backend.
    """

    # Concrete classes must set these two flags
    connected: bool
    communicating: bool

    # ------------------------------------------------------------------
    # Connection lifecycle
    # ------------------------------------------------------------------

    @abstractmethod
    def enable_motor(self) -> None:
        """Activate the motor and allow control commands.

        For CAN: sends the Servo-mode power-on frame.
        For UART: no-op (UART protocol has no explicit enable).

        :return: None
        """

    @abstractmethod
    def disable_motor(self) -> None:
        """Deactivate the motor (safe power-off).

        For CAN: sends the power-off frame.
        For UART: delegates to stop() which zeros current.

        :return: None
        """

    @abstractmethod
    def check_communication(self) -> bool:
        """Verify the motor is responding.

        :return: True if motor replies, False otherwise.
        """

    @abstractmethod
    def close(self) -> None:
        """Stop the motor and release the connection (serial port or CAN bus).

        :return: None
        """

    # ------------------------------------------------------------------
    # Status / feedback
    # ------------------------------------------------------------------

    @abstractmethod
    def get_status(self) -> object:
        """Read and log the full motor status.

        UART returns raw status bytes; CAN returns a CANMotorFeedback
        dataclass.  Both implementations log all fields at INFO level.

        :return: Status data (type differs by implementation).
        """

    @abstractmethod
    def get_position(self) -> float | None:
        """Return current motor position in degrees.

        :return: Position in degrees, or None if not available.
        """

    # ------------------------------------------------------------------
    # Control commands
    # ------------------------------------------------------------------

    @abstractmethod
    def set_position(self, position_degrees: float) -> None:
        """Move motor to the specified position.

        :param position_degrees: Target position in degrees.
        :return: None
        """

    @abstractmethod
    def set_velocity(
        self, velocity_erpm: int, allow_low_speed: bool = False
    ) -> None:
        """Spin the motor at the given electrical RPM.

        Speeds in 1–4999 ERPM cause voltage oscillations (firmware PID
        issue). Use allow_low_speed=True only when you know what you are
        doing.

        :param velocity_erpm: Target ERPM (negative = reverse direction).
        :param allow_low_speed: Bypass the 5000 ERPM safety floor.
        :return: None
        :raises ValueError: If speed is 1–4999 ERPM and allow_low_speed=False.
        """

    @abstractmethod
    def set_current(self, current_amps: float) -> None:
        """Command a specific phase current (torque control).

        :param current_amps: Target current in amps (negative = reverse torque).
        :return: None
        """

    @abstractmethod
    def set_duty_cycle(self, duty: float) -> None:
        """Apply a PWM duty cycle directly to the motor windings.

        Not all hardware backends support this.  UART will raise
        NotImplementedError.

        :param duty: Fraction from -1.0 (full reverse) to 1.0 (full forward).
        :return: None
        :raises NotImplementedError: If the backend does not support duty-cycle control.
        """

    @abstractmethod
    def set_origin(self, permanent: bool = False) -> None:
        """Define the current position as the new zero reference.

        Not all hardware backends support this.  UART will raise
        NotImplementedError.

        :param permanent: True saves origin to EEPROM, False is volatile.
        :return: None
        :raises NotImplementedError: If the backend does not support set-origin.
        """

    @abstractmethod
    def set_position_velocity_accel(
        self,
        position_degrees: float,
        velocity_erpm: int,
        accel_erpm_per_sec: int = 0,
    ) -> None:
        """Move to a position with explicit velocity and acceleration limits.

        Produces a smooth trapezoidal motion profile.  Not all hardware
        backends support this.  UART will raise NotImplementedError.

        :param position_degrees: Target position in degrees.
        :param velocity_erpm: Maximum speed during the move in ERPM.
        :param accel_erpm_per_sec: Maximum acceleration in ERPM/s (0 = firmware default).
        :return: None
        :raises NotImplementedError: If the backend does not support this mode.
        """

    # ------------------------------------------------------------------
    # High-level helpers
    # ------------------------------------------------------------------

    @abstractmethod
    def move_to_position_with_speed(
        self,
        target_degrees: float,
        motor_speed_erpm: int,
        step_delay: float = MOTOR_DEFAULTS.step_delay,
    ) -> None:
        """Velocity-drive to a target position then hold with position control.

        :param target_degrees: Target position in degrees.
        :param motor_speed_erpm: Speed to travel at in ERPM (absolute value).
        :param step_delay: Delay between incremental steps (UART only).
        :return: None
        """

    @abstractmethod
    def control_exosuit_tendon(
        self,
        action: TendonAction,
        velocity_erpm: int = MOTOR_LIMITS.default_tendon_velocity_erpm,
    ) -> None:
        """High-level exosuit tendon command.

        :param action: TendonAction.PULL, TendonAction.RELEASE, or TendonAction.STOP.
        :param velocity_erpm: Speed for PULL/RELEASE (default 10 000 ERPM).
        :return: None
        :raises ValueError: If action is not a valid TendonAction.
        """

    @abstractmethod
    def stop(self) -> None:
        """Stop motion immediately by zeroing current (windings released).

        :return: None
        """

    # ------------------------------------------------------------------
    # Context manager protocol
    # ------------------------------------------------------------------

    @abstractmethod
    def __enter__(self) -> "AbstractMotorController":
        """Context manager entry — returns self.

        :return: Self.
        """

    @abstractmethod
    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        """Context manager exit — calls close().

        :return: None
        """
