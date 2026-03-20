"""CAN Network Utilities for Linux (Jetson)."""

import subprocess
import time

from loguru import logger


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
