#!/usr/bin/env python3
"""CAN reliability diagnostic for CubeMars AK60-6.

Tests each phase of the connection sequence independently:
  1. CAN bus state (ERROR-ACTIVE vs PASSIVE vs BUS-OFF)
  2. Raw recv with no filters (do we see motor feedback IDs at all?)
  3. Raw recv with driver filters (do filters block frames?)
  4. Enable command to response timing
  5. Multiple enable/get_status cycles through the Python driver

Run:
    python scripts/diagnose_can.py --motor-id 0x03 --interface can0
"""
# ruff: noqa: T201, PLC0415

import argparse
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import can

ENABLE_FRAME = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC])
IMU_FLOOD_STD_ID = 0x0088
CAN_MASK_EXTENDED = 0x1FFFFFFF
CAN_MASK_STANDARD = 0x7FF
DEFAULT_INTERFACE = "can0"
DEFAULT_MOTOR_ID = 0x03


@dataclass(frozen=True)
class DiagnosticConfig:
    """Runtime settings for the CAN diagnostic script."""

    motor_id: int
    interface: str
    raw_duration: float
    filtered_duration: float
    enable_attempts: int
    status_attempts: int
    skip_reset: bool


def parse_int(value: str) -> int:
    """Parse decimal or hex integer values."""
    return int(value, 0)


def build_feedback_ids(motor_id: int) -> set[int]:
    """Build expected feedback arbitration IDs for the selected motor ID."""
    return {
        motor_id,
        motor_id + 1,  # firmware variant seen on bench
        0x2900 | motor_id,  # canonical extended feedback id
        0x2900,  # enable-response frame seen on some firmware
        0x0080 | motor_id,
    }


def build_driver_filters(motor_id: int) -> list[dict[str, int | bool]]:
    """Mirror CubeMarsAK606v3CAN._connect() kernel filter setup."""
    return [
        {"can_id": 0x2900 | motor_id, "can_mask": CAN_MASK_EXTENDED, "extended": True},
        {"can_id": 0x2900, "can_mask": CAN_MASK_EXTENDED, "extended": True},
        {"can_id": motor_id + 1, "can_mask": CAN_MASK_EXTENDED, "extended": True},
        {"can_id": motor_id, "can_mask": CAN_MASK_EXTENDED, "extended": True},
        {"can_id": 0x0080 | motor_id, "can_mask": CAN_MASK_EXTENDED, "extended": True},
        {"can_id": motor_id + 1, "can_mask": CAN_MASK_STANDARD, "extended": False},
    ]


def drain_bus(bus: can.BusABC, max_duration_s: float = 0.05, max_frames: int = 2048) -> int:
    """Drain queued frames with bounded work to avoid infinite loops."""
    drained = 0
    deadline = time.monotonic() + max_duration_s
    while drained < max_frames and time.monotonic() < deadline:
        frame = bus.recv(timeout=0.0)
        if frame is None:
            break
        drained += 1
    return drained


def get_can_state(interface: str) -> dict[str, int | str]:
    """Parse CAN interface state from ip command output."""
    result = subprocess.run(
        ["ip", "-details", "-statistics", "link", "show", interface],
        capture_output=True,
        check=False,
        text=True,
        timeout=5,
    )
    output = result.stdout
    state = "UNKNOWN"
    tx_err = rx_err = 0
    for line in output.splitlines():
        if "state" in line and "berr-counter" in line:
            for part in line.split():
                if part in ("ERROR-ACTIVE", "ERROR-PASSIVE", "ERROR-WARNING", "BUS-OFF"):
                    state = part
            if "tx" in line:
                idx = line.index("tx")
                tx_err = int(line[idx:].split()[1])
            if "rx" in line:
                idx = line.index("rx")
                rx_err = int(line[idx:].split()[1].rstrip(")"))
    return {"state": state, "tx_err": tx_err, "rx_err": rx_err}


def reset_interface(interface: str) -> None:
    """Reset the CAN interface with project-standard settings."""
    subprocess.run(
        ["sudo", "ip", "link", "set", interface, "down"],
        check=False,
        timeout=5,
    )
    subprocess.run(
        [
            "sudo",
            "ip",
            "link",
            "set",
            interface,
            "up",
            "type",
            "can",
            "bitrate",
            "1000000",
            "berr-reporting",
            "on",
            "restart-ms",
            "100",
        ],
        check=False,
        timeout=5,
    )
    time.sleep(0.5)


def test_raw_recv_no_filter(config: DiagnosticConfig) -> dict[str, float | int | None]:
    """Open a raw CAN socket with no filters to inspect all frame IDs."""
    feedback_ids = build_feedback_ids(config.motor_id)
    print(f"\n{'-' * 60}")
    print(f"Test 1: Raw recv, no filters ({config.raw_duration}s)")
    print(f"{'-' * 60}")
    bus = can.interface.Bus(channel=config.interface, interface="socketcan")
    frames: dict[int, int] = {}
    error_frames: dict[int, int] = {}
    first_motor_at = None
    t0 = time.monotonic()
    deadline = t0 + config.raw_duration
    while time.monotonic() < deadline:
        msg = bus.recv(timeout=0.1)
        if msg is None:
            continue
        if msg.is_error_frame:
            error_frames[msg.arbitration_id] = (
                error_frames.get(msg.arbitration_id, 0) + 1
            )
            continue
        frames[msg.arbitration_id] = frames.get(msg.arbitration_id, 0) + 1
        if msg.arbitration_id in feedback_ids and first_motor_at is None:
            first_motor_at = time.monotonic() - t0
    bus.shutdown()

    total = sum(frames.values())
    motor_frames = sum(cnt for aid, cnt in frames.items() if aid in feedback_ids)
    flood_0088 = frames.get(IMU_FLOOD_STD_ID, 0)
    print(f"  Total frames received : {total}")
    print(f"  Error frames          : {sum(error_frames.values())}")
    if error_frames:
        print(
            "  Error IDs             : "
            f"{[(hex(k), v) for k, v in sorted(error_frames.items())]}"
        )
    print(
        "  Motor feedback frames : "
        f"{motor_frames}  (IDs: {[hex(k) for k in sorted(frames) if k in feedback_ids]})"
    )
    print(f"  0x0088 flood frames   : {flood_0088}")
    print(
        "  Other frame IDs       : "
        f"{[hex(k) for k in sorted(frames) if k not in feedback_ids and k != IMU_FLOOD_STD_ID]}"
    )
    if first_motor_at is not None:
        print(f"  First motor frame at  : {first_motor_at * 1000:.0f} ms")
    else:
        print(f"  WARNING: no motor frames received in {config.raw_duration}s")
    return {
        "total": total,
        "motor": motor_frames,
        "flood_0088": flood_0088,
        "error_frames": sum(error_frames.values()),
        "first_motor_ms": first_motor_at * 1000 if first_motor_at else None,
    }


def test_raw_recv_with_filter(config: DiagnosticConfig) -> dict[str, float | int | None]:
    """Open CAN socket with the driver filters and inspect what passes."""
    print(f"\n{'-' * 60}")
    print(f"Test 2: Raw recv, with driver filters ({config.filtered_duration}s)")
    print(f"{'-' * 60}")
    bus = can.interface.Bus(
        channel=config.interface,
        interface="socketcan",
        can_filters=build_driver_filters(config.motor_id),
        ignore_rx_error_frames=True,
    )
    frames: dict[int, int] = {}
    first_frame_at = None
    t0 = time.monotonic()
    deadline = t0 + config.filtered_duration
    while time.monotonic() < deadline:
        msg = bus.recv(timeout=0.1)
        if msg is None:
            continue
        frames[msg.arbitration_id] = frames.get(msg.arbitration_id, 0) + 1
        if first_frame_at is None:
            first_frame_at = time.monotonic() - t0
    bus.shutdown()

    total = sum(frames.values())
    print(f"  Total frames received : {total}")
    print(f"  Frame IDs             : {[(hex(k), v) for k, v in sorted(frames.items())]}")
    if first_frame_at is not None:
        print(f"  First frame at        : {first_frame_at * 1000:.0f} ms")
    else:
        print(f"  WARNING: no frames passed filter in {config.filtered_duration}s")
    return {"total": total, "first_ms": first_frame_at * 1000 if first_frame_at else None}


def test_enable_and_response(config: DiagnosticConfig) -> dict[str, float | list[dict]]:
    """Send enable commands and measure response timing."""
    feedback_ids = build_feedback_ids(config.motor_id)
    print(f"\n{'-' * 60}")
    print(f"Test 3: Enable -> response timing ({config.enable_attempts} attempts)")
    print(f"{'-' * 60}")
    bus = can.interface.Bus(
        channel=config.interface,
        interface="socketcan",
        can_filters=build_driver_filters(config.motor_id),
        ignore_rx_error_frames=True,
    )
    results = []
    for i in range(config.enable_attempts):
        _ = drain_bus(bus, max_duration_s=0.08, max_frames=4096)

        msg = can.Message(
            arbitration_id=config.motor_id,
            data=ENABLE_FRAME,
            is_extended_id=True,
        )
        t_send = time.monotonic()
        try:
            bus.send(msg, timeout=0.1)
        except can.CanError as exc:
            results.append({"ok": False, "latency_ms": None, "pos": None})
            print(f"  [{i + 1}] FAIL  send error: {exc}")
            time.sleep(0.3)
            continue

        reply = None
        deadline = t_send + 1.0
        while time.monotonic() < deadline:
            frame = bus.recv(timeout=0.1)
            if frame and frame.arbitration_id in feedback_ids:
                reply = frame
                break

        if reply:
            latency_ms = (time.monotonic() - t_send) * 1000
            pos = int.from_bytes(reply.data[0:2], "big", signed=True) * 0.1
            results.append({"ok": True, "latency_ms": latency_ms, "pos": pos})
            print(
                f"  [{i + 1}] OK  response in {latency_ms:.1f} ms  "
                f"pos={pos:.1f} deg  id=0x{reply.arbitration_id:04X}"
            )
        else:
            results.append({"ok": False, "latency_ms": None, "pos": None})
            print(f"  [{i + 1}] FAIL  no response within 1s")
        time.sleep(0.3)

    bus.shutdown()
    ok_count = sum(1 for result in results if result["ok"])
    print(f"\n  Success rate: {ok_count}/{config.enable_attempts}")
    if ok_count > 0:
        latencies = [result["latency_ms"] for result in results if result["ok"]]
        assert latencies
        print(
            f"  Latency: min={min(latencies):.1f}ms  "
            f"max={max(latencies):.1f}ms  avg={sum(latencies) / len(latencies):.1f}ms"
        )
    return {"success_rate": ok_count / config.enable_attempts, "results": results}


def test_get_status_loop(config: DiagnosticConfig) -> dict[str, float]:
    """Emulate the enable + get_status loop from the main driver."""
    print(f"\n{'-' * 60}")
    print(f"Test 4: Full driver enable + get_status ({config.status_attempts} attempts)")
    print(f"{'-' * 60}")
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
    from motor_python.cube_mars_motor_can import CubeMarsAK606v3CAN

    results = []
    for i in range(config.status_attempts):
        motor = None
        try:
            motor = CubeMarsAK606v3CAN(
                motor_can_id=config.motor_id,
                interface=config.interface,
            )
            motor.enable_motor()
            time.sleep(0.3)
            feedback = motor.get_status()
            if feedback:
                results.append({"ok": True, "pos": feedback.position_degrees})
                print(
                    f"  [{i + 1}] OK  pos={feedback.position_degrees:.1f} deg  "
                    f"temp={feedback.temperature_celsius} C"
                )
            else:
                results.append({"ok": False, "pos": None})
                print(f"  [{i + 1}] FAIL  get_status() returned None")
            motor.disable_motor()
        except Exception as exc:
            results.append({"ok": False, "pos": None})
            print(f"  [{i + 1}] FAIL  exception: {exc}")
        finally:
            if motor is not None:
                try:
                    motor.close()
                except Exception as exc:
                    print(f"  [{i + 1}] WARN  close() failed: {exc}")
        time.sleep(0.5)

    ok_count = sum(1 for result in results if result["ok"])
    print(f"\n  Success rate: {ok_count}/{config.status_attempts}")
    return {"success_rate": ok_count / config.status_attempts}


def parse_args() -> DiagnosticConfig:
    """Parse command line arguments for diagnostics."""
    parser = argparse.ArgumentParser(description="CAN reliability diagnostic")
    parser.add_argument(
        "--motor-id",
        type=parse_int,
        default=DEFAULT_MOTOR_ID,
        help="Motor CAN ID (decimal or hex, default: 0x03).",
    )
    parser.add_argument(
        "--interface",
        default=DEFAULT_INTERFACE,
        help="SocketCAN interface name (default: can0).",
    )
    parser.add_argument(
        "--raw-duration",
        type=float,
        default=2.0,
        help="Duration in seconds for raw unfiltered listen (default: 2.0).",
    )
    parser.add_argument(
        "--filtered-duration",
        type=float,
        default=2.0,
        help="Duration in seconds for filtered listen (default: 2.0).",
    )
    parser.add_argument(
        "--enable-attempts",
        type=int,
        default=5,
        help="Enable/response attempts for Test 3 (default: 5).",
    )
    parser.add_argument(
        "--status-attempts",
        type=int,
        default=10,
        help="Enable/get_status attempts for Test 4 (default: 10).",
    )
    parser.add_argument(
        "--skip-reset",
        action="store_true",
        help="Do not reset can0 when bus state is not ERROR-ACTIVE.",
    )
    args = parser.parse_args()
    return DiagnosticConfig(
        motor_id=args.motor_id,
        interface=args.interface,
        raw_duration=args.raw_duration,
        filtered_duration=args.filtered_duration,
        enable_attempts=args.enable_attempts,
        status_attempts=args.status_attempts,
        skip_reset=args.skip_reset,
    )


def main(config: DiagnosticConfig) -> None:
    """Run the full diagnostic suite."""
    print("=" * 60)
    print("  CAN Reliability Diagnostic")
    print("=" * 60)
    print(f"  Interface : {config.interface}")
    print(f"  Motor ID  : 0x{config.motor_id:02X}")
    print(f"  IMU flood : 0x{IMU_FLOOD_STD_ID:03X} (standard ID)")

    state = get_can_state(config.interface)
    print(
        f"\n  CAN state: {state['state']}  "
        f"(tx_err={state['tx_err']} rx_err={state['rx_err']})"
    )
    if state["state"] != "ERROR-ACTIVE":
        if config.skip_reset:
            print("  WARNING: bus is not ERROR-ACTIVE and reset is skipped.")
        else:
            print("  Bus is not ERROR-ACTIVE; resetting interface...")
            reset_interface(config.interface)
            state = get_can_state(config.interface)
            print(
                f"  After reset: {state['state']}  "
                f"(tx_err={state['tx_err']} rx_err={state['rx_err']})"
            )

    test1 = test_raw_recv_no_filter(config)
    test2 = test_raw_recv_with_filter(config)
    test3 = test_enable_and_response(config)
    test4 = test_get_status_loop(config)

    raw_mode = (
        "broadcasting"
        if test1["motor"] > 0
        else ("none" if test1["total"] == 0 else "response-only / other-IDs")
    )
    filtered_mode = (
        "broadcasting"
        if test2["total"] > 0
        else ("none" if test1["error_frames"] > 0 else "response-only (normal)")
    )

    print(f"\n{'=' * 60}")
    print("  SUMMARY")
    print(f"{'=' * 60}")
    print(f"  Bus state             : {state['state']}")
    print(
        f"  Raw motor frames      : {test1['motor']} in {config.raw_duration:g}s  "
        f"({raw_mode})"
    )
    print(f"  Raw error frames      : {test1['error_frames']} in {config.raw_duration:g}s")
    print(
        f"  Filtered frames       : {test2['total']} in {config.filtered_duration:g}s  "
        f"({filtered_mode})"
    )
    print(f"  Enable->response      : {test3['success_rate'] * 100:.0f}%")
    print(f"  Full driver get_status: {test4['success_rate'] * 100:.0f}%")

    if test3["success_rate"] == 0:
        print("\n  ROOT CAUSE: motor is not responding to commands at all.")
        print("  -> Check motor power and UART cable disconnection first.")
    elif test3["success_rate"] < 1.0:
        print("\n  ROOT CAUSE: enable command response is intermittent.")
        print("  -> Check timing and CAN bus error state during retries.")
    elif test4["success_rate"] < 1.0:
        print("\n  ROOT CAUSE: driver path issue in enable -> get_status flow.")
        print("  -> Check _capture_response timeout and pending feedback logic.")
    else:
        print("\n  OK: all tests passed - CAN is working reliably right now.")
        if test1["motor"] == 0:
            print("  INFO: motor appears to run in response-only mode.")
    print(f"{'=' * 60}")


if __name__ == "__main__":
    main(parse_args())
