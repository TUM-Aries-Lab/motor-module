"""Helpers for packing and unpacking CAN MIT protocol frames."""


def float_to_uint(value: float, v_min: float, v_max: float, n_bits: int) -> int:
    """Convert a float literal to unsigned integer for bit-packing.

    Replicates CubeMars documentation C macro:
    int float_to_uint(float x, float x_min, float x_max, int bits)

    :param value: Float value to convert.
    :param v_min: Minimum expected range.
    :param v_max: Maximum expected range.
    :param n_bits: Number of bits for the output integer.
    :return: Unsigned integer representation.
    """
    span = v_max - v_min
    offset = v_min
    # Cap value to range
    value = min(value, v_max)
    value = max(value, v_min)
    # Map [v_min, v_max] to [0, (2^n_bits) - 1]
    return int(((value - offset) * ((1 << n_bits) - 1)) / span)


def uint_to_float(raw: int, v_min: float, v_max: float, n_bits: int) -> float:
    """Convert an unpacked unsigned integer back to float.

    :param raw: Extracted unsigned integer.
    :param v_min: Minimum expected range.
    :param v_max: Maximum expected range.
    :param n_bits: Number of bits in the integer.
    :return: Reconstructed float value.
    """
    span = v_max - v_min
    offset = v_min
    return float(raw) * span / float((1 << n_bits) - 1) + offset


def pack_mit_frame(
    p_des: float, v_des: float, kp: float, kd: float, t_ff: float
) -> bytes:
    """Pack MIT mode parameters into an 8-byte payload.

    Bit allocations:
      position  (p_des): 16 bits
      velocity  (v_des): 12 bits
      stiffness    (kp): 12 bits
      damping      (kd): 12 bits
      feedforward (t_ff): 12 bits

    :param p_des: Target position in radians.
    :param v_des: Target velocity in rad/s.
    :param kp: Position stiffness (Nm/rad).
    :param kd: Velocity damping (Nm s/rad).
    :param t_ff: Feedforward torque (Nm).
    :return: 8-byte payload ready to send.
    """
    p_int = float_to_uint(p_des, -12.5, 12.5, 16)
    v_int = float_to_uint(v_des, -45.0, 45.0, 12)
    kp_int = float_to_uint(kp, 0.0, 500.0, 12)
    kd_int = float_to_uint(kd, 0.0, 5.0, 12)
    t_int = float_to_uint(t_ff, -18.0, 18.0, 12)

    buf = bytearray(8)
    # [15:0] position (16 bits)
    buf[0] = p_int >> 8
    buf[1] = p_int & 0xFF

    # [11:4] velocity top 8 bits
    buf[2] = v_int >> 4
    # [3:0] vel lower 4 bits + [11:8] kp top 4 bits
    buf[3] = ((v_int & 0xF) << 4) | (kp_int >> 8)

    # [7:0] kp lower 8 bits
    buf[4] = kp_int & 0xFF

    # [11:4] kd top 8 bits
    buf[5] = kd_int >> 4
    # [3:0] kd lower 4 bits + [11:8] torque top 4 bits
    buf[6] = ((kd_int & 0xF) << 4) | (t_int >> 8)

    # [7:0] torque lower 8 bits
    buf[7] = t_int & 0xFF

    return bytes(buf)
