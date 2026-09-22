"""Tests for raising the CAN interface.

The parsing is tested against real ``ip -details link show can0`` output from
the Jetson, because that is the part most likely to be quietly wrong: a
mis-read "up" turns the idempotent path into one that takes a working bus down
and back up on every start.
"""

import subprocess
from types import SimpleNamespace

import pytest

from motor_python import can_utils
from motor_python.definitions import CAN_DEFAULTS

# Captured before the autouse fixtures replace it, so the one test that needs
# the real runner can still reach it.
REAL_RUN = can_utils._run

UP_AT_1M = (
    "3: can0: <NOARP,UP,LOWER_UP,ECHO> mtu 16 qdisc pfifo_fast state UP mode DEFAULT "
    "group default qlen 1000\n"
    "    link/can  promiscuity 0 minmtu 0 maxmtu 0 \n"
    "    can state ERROR-ACTIVE restart-ms 100 \n"
    "    bitrate 1000000 sample-point 0.750 \n"
    "    tq 25 prop-seg 14 phase-seg1 15 phase-seg2 10 sjw 1\n"
)
DOWN = (
    "3: can0: <NOARP,ECHO> mtu 16 qdisc pfifo_fast state DOWN mode DEFAULT "
    "group default qlen 10\n"
    "    link/can  promiscuity 0 minmtu 0 maxmtu 0 \n"
    "    can <BERR-REPORTING> state STOPPED restart-ms 100 \n"
    "    bitrate 1000000 sample-point 0.750 \n"
)
UP_AT_500K = UP_AT_1M.replace("bitrate 1000000", "bitrate 500000")


def _ip_returning(output: str, returncode: int = 0):
    """Build a fake _run that answers every `ip show` with this output."""

    def fake(command, timeout):
        if "show" in command:
            return SimpleNamespace(returncode=returncode, stdout=output, stderr="")
        return SimpleNamespace(returncode=0, stdout="", stderr="")

    return fake


@pytest.fixture(autouse=True)
def _pretend_ip_exists(monkeypatch):
    """Every test assumes a Linux host; the no-ip case is tested explicitly."""
    monkeypatch.setattr(can_utils.shutil, "which", lambda _: "/usr/bin/ip")


def test_reads_an_interface_that_is_up():
    """Real Jetson output for a healthy can0."""
    exists, is_up, bitrate = can_utils.read_can_interface()

    # Parsed from the canned output via the patched runner below.
    assert (exists, is_up, bitrate) == (True, True, 1_000_000)


@pytest.fixture(autouse=True)
def _default_ip_output(monkeypatch):
    """Default every test to the healthy output; individual tests override."""
    monkeypatch.setattr(can_utils, "_run", _ip_returning(UP_AT_1M))


def test_reads_an_interface_that_is_down(monkeypatch):
    """A stopped interface must not read as up just because 'UP' appears elsewhere."""
    monkeypatch.setattr(can_utils, "_run", _ip_returning(DOWN))

    exists, is_up, bitrate = can_utils.read_can_interface()

    assert (exists, is_up) == (True, False)
    assert bitrate == 1_000_000


def test_a_healthy_interface_is_left_completely_alone(monkeypatch):
    """The point of the check: never disturb a bus that is already working.

    Taking the link down and up resets the controller and can drop a motor into
    BUS-OFF, so an unnecessary bring-up is worse than no bring-up at all.
    """
    calls = []

    def recording(command, timeout):
        calls.append(command)
        return SimpleNamespace(returncode=0, stdout=UP_AT_1M, stderr="")

    monkeypatch.setattr(can_utils, "_run", recording)

    assert can_utils.ensure_can_interface() is True
    assert all("sudo" not in command for command in calls)


def test_a_down_interface_is_brought_up(monkeypatch):
    """Down first, then up with the bitrate, then the queue length."""
    calls = []
    outputs = [DOWN, UP_AT_1M]

    def fake(command, timeout):
        calls.append(command)
        if "show" in command:
            return SimpleNamespace(
                returncode=0, stdout=outputs.pop(0) if outputs else UP_AT_1M, stderr=""
            )
        return SimpleNamespace(returncode=0, stdout="", stderr="")

    monkeypatch.setattr(can_utils, "_run", fake)

    assert can_utils.ensure_can_interface() is True

    configuring = [c for c in calls if "sudo" in c]
    assert len(configuring) == 3
    assert configuring[0][-1] == "down"
    assert "bitrate" in configuring[1]
    assert str(CAN_DEFAULTS.bitrate) in configuring[1]
    assert "restart-ms" in configuring[1]
    assert "txqueuelen" in configuring[2]
    # -n on every one, or a missing sudo password stalls the control process.
    assert all(c[1] == "-n" for c in configuring)


def test_a_wrong_bitrate_is_reconfigured(monkeypatch):
    """Up but at the wrong speed is not usable, and must be corrected."""
    outputs = [UP_AT_500K, UP_AT_1M]
    calls = []

    def fake(command, timeout):
        calls.append(command)
        if "show" in command:
            return SimpleNamespace(
                returncode=0, stdout=outputs.pop(0) if outputs else UP_AT_1M, stderr=""
            )
        return SimpleNamespace(returncode=0, stdout="", stderr="")

    monkeypatch.setattr(can_utils, "_run", fake)

    assert can_utils.ensure_can_interface() is True
    assert any("sudo" in command for command in calls)


def test_a_missing_interface_is_reported_not_created(monkeypatch):
    """Without the kernel module there is nothing to raise."""
    monkeypatch.setattr(can_utils, "_run", _ip_returning("", returncode=1))

    assert can_utils.ensure_can_interface() is False


def test_sudo_refusing_is_reported_rather_than_hanging(monkeypatch):
    """A host without passwordless sudo must fail, not wait at a prompt."""

    def fake(command, timeout):
        if "show" in command:
            return SimpleNamespace(returncode=0, stdout=DOWN, stderr="")
        return SimpleNamespace(
            returncode=1, stdout="", stderr="sudo: a password is required"
        )

    monkeypatch.setattr(can_utils, "_run", fake)

    assert can_utils.ensure_can_interface() is False


def test_it_does_nothing_off_linux(monkeypatch):
    """No `ip` means a developer machine, and nothing to do."""
    monkeypatch.setattr(can_utils.shutil, "which", lambda _: None)

    assert can_utils.ensure_can_interface() is False


def test_a_hanging_command_does_not_propagate(monkeypatch):
    """A timeout is answered with None, not an exception into the caller."""

    def explode(*_args, **_kwargs):
        raise subprocess.TimeoutExpired(cmd="ip", timeout=5.0)

    monkeypatch.setattr(can_utils.subprocess, "run", explode)

    assert REAL_RUN(["ip", "link"], 5.0) is None
