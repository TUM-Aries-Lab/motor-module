"""CAN Network Utilities for Linux (Jetson).

Three operations on the interface, in increasing order of violence:

* :func:`get_can_state` reads the controller's error state and counters;
* :func:`ensure_can_interface` raises the link if it is down, and leaves it
  strictly alone if it is already up at the right bitrate;
* :func:`reset_can_interface` reloads the kernel module, which is the only
  thing that clears latched error counters on the Orin's mttcan controller.

Reach for the lightest one that answers the problem. Reconfiguring a working
bus is not free: taking the link down and up resets the controller and can
drop a motor into BUS-OFF.
"""

import shutil
import subprocess
import time

from loguru import logger

from motor_python.definitions import CAN_DEFAULTS


def get_can_state(interface: str = "can0") -> dict:
    """Parse CAN controller state from the kernel via `ip` command.

    :param interface: CAN interface name (e.g. 'can0').
    :return: Dict with keys 'state', 'tx_err', 'rx_err'.
    """
    try:
        result = subprocess.run(
            ["ip", "-details", "-statistics", "link", "show", interface],
            capture_output=True,
            text=True,
            timeout=5,
        )
        output = result.stdout
        state = "UNKNOWN"
        tx_err = rx_err = 0
        for line in output.splitlines():
            if "state" in line and "berr-counter" in line:
                for part in line.split():
                    if part in (
                        "ERROR-ACTIVE",
                        "ERROR-PASSIVE",
                        "ERROR-WARNING",
                        "BUS-OFF",
                    ):
                        state = part
                if "tx" in line:
                    idx = line.index("tx")
                    tx_err = int(line[idx:].split()[1])
                if "rx" in line:
                    idx = line.index("rx")
                    rx_err = int(line[idx:].split()[1].rstrip(")"))
        return {"state": state, "tx_err": tx_err, "rx_err": rx_err}
    except Exception as e:
        logger.debug(f"Could not read CAN state: {e}")
        return {"state": "UNKNOWN", "tx_err": 0, "rx_err": 0}


def reset_can_interface(interface: str = "can0", bitrate: int = 1000000) -> bool:
    """Fully reset the CAN interface by reloading the mttcan kernel module.

    ``ip link set down/up`` does NOT reset TX/RX error counters on the
    Jetson Orin Nano mttcan controller and causes a CAN bus disruption
    that can push the motor into BUS-OFF (completely silent).  This method
    instead:

    1. Takes the interface down.
    2. Unloads the mttcan module — the bus is silent during unload, giving
       the motor's CAN controller time to see the 128×11 recessive bits
       required by the CAN spec to recover from BUS-OFF.
    3. Reloads the module and brings the interface back up.

    :param interface: CAN interface name (e.g. 'can0').
    :param bitrate: Desired bitrate (e.g. 1000000).
    :return: True if successful, False if the commands fail (e.g. no sudo).
    """
    logger.warning(
        f"Performing full kernel-level reset of {interface} to clear error states..."
    )
    cmds = [
        f"sudo ip link set {interface} down",
        "sudo rmmod mttcan",
        "sleep 0.5",  # Let bus stay recessive so nodes can recover
        "sudo modprobe mttcan",
        # Keep runtime reset behavior aligned with setup_can.sh.
        f"sudo ip link set {interface} up type can bitrate {bitrate} berr-reporting on restart-ms 100",
        f"sudo ip link set {interface} txqueuelen 1000",
    ]
    try:
        for cmd in cmds:
            if cmd == "sleep 0.5":
                time.sleep(0.5)
            else:
                subprocess.run(cmd.split(), check=True)
            time.sleep(0.1)
        logger.success(f"CAN interface {interface} reset successfully.")
        return True
    except subprocess.CalledProcessError as e:
        logger.error(f"Failed to reset CAN interface: {e}")
        return False


def _run(command: list[str], timeout: float) -> subprocess.CompletedProcess | None:
    """Run a command, returning None if it could not be run at all.

    :param command: Argument vector.
    :param timeout: Seconds to wait before giving up.
    :return: The finished process, or None if it could not run.
    """
    try:
        # check=False: a non-zero exit is information here, not an exception.
        # The caller reads returncode and reports it with the interface's name.
        return subprocess.run(
            command, capture_output=True, text=True, timeout=timeout, check=False
        )
    except (OSError, subprocess.SubprocessError) as err:
        logger.debug(f"Could not run {' '.join(command)}: '{err}'.")
        return None


def read_can_interface(
    interface: str = CAN_DEFAULTS.interface,
) -> tuple[bool, bool, int | None]:
    """Report what the kernel currently thinks of a CAN interface.

    :param interface: CAN interface name (e.g. 'can0').
    :return: ``(exists, is_up, bitrate)``; bitrate is None when unreadable.
    :rtype: tuple[bool, bool, int | None]
    """
    result = _run(
        ["ip", "-details", "link", "show", interface],
        CAN_DEFAULTS.can_command_timeout_s,
    )
    if result is None or result.returncode != 0:
        return False, False, None

    output = result.stdout
    # The flags live in angle brackets on the first line. "UP" also appears in
    # "state UP" further along, but the flag is the authoritative one -- an
    # interface can carry the UP flag while its state reads UNKNOWN, which is
    # normal for CAN.
    first_line = output.splitlines()[0] if output else ""
    is_up = "UP" in first_line.split("<")[-1].split(">")[0].split(",")

    bitrate = None
    fields = output.split()
    if "bitrate" in fields:
        try:
            bitrate = int(fields[fields.index("bitrate") + 1])
        except (IndexError, ValueError):
            bitrate = None
    return True, is_up, bitrate


def ensure_can_interface(
    interface: str = CAN_DEFAULTS.interface,
    bitrate: int = CAN_DEFAULTS.bitrate,
) -> bool:
    """Make sure a CAN interface is up at the given bitrate, raising it if not.

    Callers otherwise have to remember ``sudo ./setup_can.sh`` after every power
    cycle, and forgetting it presents as a motor that will not answer -- which
    sends you to the wiring for a fault that is one command away.

    **Idempotent**: an interface already up at the right bitrate is left
    untouched. That is the point of the check rather than a shortcut through
    it, because bringing a working link down and up resets the controller and
    can drop a motor into BUS-OFF.

    This does not reload the ``mttcan`` module; see
    :func:`reset_can_interface` for that, which is the right tool once error
    counters have latched and the wrong one to use unasked at startup.

    :param interface: CAN interface name (e.g. 'can0').
    :param bitrate: Desired bitrate in bits/sec.
    :return: True if the interface is usable afterwards.
    :rtype: bool
    """
    if shutil.which("ip") is None:
        logger.debug(
            "No 'ip' command, so this is not a Linux host. Skipping CAN setup."
        )
        return False

    exists, is_up, current_bitrate = read_can_interface(interface)
    if not exists:
        logger.error(
            f"No CAN interface '{interface}'. The mttcan kernel module is "
            f"probably not loaded; run setup_can.sh once, which modprobes it."
        )
        return False

    if is_up and current_bitrate == bitrate:
        logger.info(
            f"CAN interface '{interface}' is already up at {bitrate} bps. "
            f"Leaving it alone."
        )
        return True

    reason = "down" if not is_up else f"at {current_bitrate} bps rather than {bitrate}"
    logger.warning(f"CAN interface '{interface}' is {reason}. Bringing it up.")
    return _bring_up_can_interface(interface, bitrate)


def _bring_up_can_interface(interface: str, bitrate: int) -> bool:
    """Configure and raise the interface.

    ``sudo -n`` throughout: a control process must not stop at a password
    prompt, so a host without passwordless sudo fails immediately and says so
    rather than hanging until something times out.

    :param interface: CAN interface name.
    :param bitrate: Desired bitrate in bits/sec.
    :return: True if the interface came up at the requested bitrate.
    :rtype: bool
    """
    timeout = CAN_DEFAULTS.can_command_timeout_s
    commands = [
        # Down first: "up type can bitrate ..." is rejected on an interface that
        # is already up, which is the case when only the bitrate is wrong.
        ["sudo", "-n", "ip", "link", "set", interface, "down"],
        [
            "sudo",
            "-n",
            "ip",
            "link",
            "set",
            interface,
            "up",
            "type",
            "can",
            "bitrate",
            str(bitrate),
            "berr-reporting",
            "on",
            "restart-ms",
            str(CAN_DEFAULTS.can_restart_ms),
        ],
        [
            "sudo",
            "-n",
            "ip",
            "link",
            "set",
            interface,
            "txqueuelen",
            str(CAN_DEFAULTS.can_tx_queue_length),
        ],
    ]

    for command in commands:
        result = _run(command, timeout)
        if result is None or result.returncode != 0:
            detail = (result.stderr or result.stdout).strip() if result else "not run"
            logger.error(
                f"Could not bring up '{interface}': '{detail}'. Run "
                f"scripts/allow_can_bringup_without_password.sh once to fix this "
                f"permanently, or setup_can.sh with sudo for just this boot."
            )
            return False

    _, is_up, current_bitrate = read_can_interface(interface)
    if not (is_up and current_bitrate == bitrate):
        logger.error(
            f"'{interface}' did not come up as asked: up={is_up}, "
            f"bitrate={current_bitrate}."
        )
        return False

    logger.success(f"CAN interface '{interface}' up at {bitrate} bps.")
    return True
