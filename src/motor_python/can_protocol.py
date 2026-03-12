"""CAN Protocol Control Modes definitions."""


class CANControlMode:
    """CAN extended ID control modes — CubeMars AK60-6 spec section 4.4.1.

    Extended arb_id format: (mode << 8) | motor_id
    E.g. for motor_id=0x03: duty=0x0003, velocity=0x0303, position=0x0403

    Verified against spec table (motor ID 0x68 examples):
      Duty cycle  0x0068 — data: int32(duty × 100 000)
      Current     0x0168 — data: int32(amps × 1 000)
      Brk current 0x0268 — data: int32(amps × 1 000)  (same payload as current)
      Velocity    0x0368 — data: int32(ERPM direct)
      Position    0x0468 — data: int32(deg × 10 000)
      Pos-Vel     0x0668 — data: int32 pos + int16(ERPM/10) + int16(acc/10)
      MIT         0x0868 — bit-packed (pos/vel/kp/kd/torque), see MIT protocol
    """

    DUTY_CYCLE = 0x00  # Duty cycle control  — PWM voltage fraction
    CURRENT_LOOP = 0x01  # IQ current control  — direct torque
    CURRENT_BRAKE = 0x02  # Brake current mode  — hold with current
    VELOCITY_LOOP = 0x03  # Velocity loop        — ERPM setpoint
    POSITION_LOOP = 0x04  # Position loop        — degree setpoint
    SET_ORIGIN = 0x05  # Set origin          — define new zero
    POSITION_VELOCITY = 0x06  # Pos + vel + acc profile
    MIT_MODE = 0x08  # MIT impedance/actuator protocol (bit-packed 64-bit frame)
    #                    Enable:  FF FF FF FF FF FF FF FF  (arb_id = motor_id)
    #                    Disable: FF FF FF FF FF FF FF FE  (arb_id = motor_id)
    #                    Payload encoding — all big-endian bit-fields (8 bytes total):
    #                      [63:48] pos  uint16  range [-12.5, 12.5] rad   → 0..65535
    #                      [47:36] vel  uint12  range [-45.0, 45.0] rad/s → 0..4095
    #                      [35:24] kp   uint12  range [0, 500] Nm/rad     → 0..4095
    #                      [23:12] kd   uint12  range [0, 5] Nms/rad      → 0..4095
    #                      [11:0]  tau  uint12  range [-18, 18] Nm        → 0..4095
    #                    See _pack_mit_frame() and set_mit_mode() for Python helpers.


class CANOriginMode:
    """Byte payloads for the SET_ORIGIN command (0x05 mode)."""

    TEMPORARY = 0x01  # Resets on next power cycle
    PERMANENT = 0x02  # Saved to internal EEPROM
