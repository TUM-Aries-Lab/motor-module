#!/bin/bash
# Let the exosuit raise can0 without a sudo password.
#
# motor_python.can_utils.ensure_can_interface runs `sudo -n ip link set can0
# ...` when the
# interface is down. The -n is deliberate: a control process must never stop at
# a password prompt. Without a sudoers rule it therefore gives up and asks you
# to run setup_can.sh by hand, which is the thing the bring-up exists to avoid.
#
# Run once per machine:
#     sudo ./scripts/allow_can_bringup_without_password.sh
#
# The rule is narrow on purpose. It permits `ip link set can0 <anything>` and
# nothing else: not other interfaces, not `ip route`, not `ip addr`. Reading the
# interface needs no privilege at all, so `ip link show` is not granted -- the
# bring-up already calls that without sudo.
#
# To undo:
#     sudo rm /etc/sudoers.d/exosuit-can

set -euo pipefail

RULE_FILE=/etc/sudoers.d/exosuit-can
INTERFACE=can0

if [ "$EUID" -ne 0 ]; then
    echo "ERROR: run with sudo -- this writes to /etc/sudoers.d/." >&2
    exit 1
fi

# The invoking user, not root, is the one that needs the rule.
TARGET_USER="${SUDO_USER:-}"
if [ -z "$TARGET_USER" ]; then
    echo "ERROR: could not tell who invoked sudo. Run as: sudo $0" >&2
    exit 1
fi

# sudoers matches on the absolute path, and `ip` lives in different places
# across distributions, so it is resolved rather than assumed.
IP_PATH="$(command -v ip || true)"
if [ -z "$IP_PATH" ]; then
    echo "ERROR: no 'ip' command found." >&2
    exit 1
fi

RULE="$TARGET_USER ALL=(root) NOPASSWD: $IP_PATH link set $INTERFACE *"

# Validated in a temporary file first. A malformed line in /etc/sudoers.d/ can
# break sudo for every user on the machine, and this one is being written by a
# script rather than by visudo.
TEMP_FILE="$(mktemp)"
trap 'rm -f "$TEMP_FILE"' EXIT
echo "$RULE" > "$TEMP_FILE"
chmod 0440 "$TEMP_FILE"

if ! visudo -c -f "$TEMP_FILE" > /dev/null; then
    echo "ERROR: the generated rule is not valid sudoers syntax. Nothing written." >&2
    echo "       rule was: $RULE" >&2
    exit 1
fi

install -m 0440 -o root -g root "$TEMP_FILE" "$RULE_FILE"
echo "Wrote $RULE_FILE:"
echo "  $RULE"

# And confirm sudo as a whole still parses, including the file just added.
if ! visudo -c > /dev/null; then
    echo "ERROR: sudoers no longer validates. Removing the rule." >&2
    rm -f "$RULE_FILE"
    exit 1
fi

echo ""
echo "Check it took effect, as $TARGET_USER and not as root:"
echo "  sudo -n -l | grep $INTERFACE"
echo ""
echo "exosuit-python will now raise $INTERFACE by itself when it is down."
