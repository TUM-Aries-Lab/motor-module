"""CAN protocol mode definitions for CubeMars extended CAN IDs."""


class CANControlMode:
    """CAN extended ID control modes.

    Extended arbitration ID format: ``(mode << 8) | motor_id``.
    This project now implements Force Control Mode (MIT, mode 0x08) as the
    primary command path for AK60-6.

    Legacy mode IDs are kept as constants for interoperability/reference.
    """

    DUTY_CYCLE = 0x00  # Duty cycle control  — PWM voltage fraction
    CURRENT_LOOP = 0x01  # IQ current control  — direct torque
    CURRENT_BRAKE = 0x02  # Brake current mode  — hold with current
    VELOCITY_LOOP = 0x03  # Velocity loop        — ERPM setpoint
    POSITION_LOOP = 0x04  # Position loop        — degree setpoint
    SET_ORIGIN = 0x05  # Set origin          — define new zero
    POSITION_VELOCITY = 0x06  # Pos + vel + acc profile
    MIT_MODE = 0x08  # Force Control Mode (MIT protocol, extended frame)


class CANOriginMode:
    """Byte payloads for the SET_ORIGIN command (0x05 mode)."""

    TEMPORARY = 0x01  # Resets on next power cycle
    PERMANENT = 0x02  # Saved to internal EEPROM
