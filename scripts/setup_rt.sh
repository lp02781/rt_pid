#!/usr/bin/env bash
# Grant this user permission to use real-time scheduling and to lock memory.
#
# Without this, sched_setscheduler(SCHED_FIFO) fails with EPERM and the loop
# silently falls back to the normal scheduler. Run once, then log out and back
# in -- PAM only applies rlimits at login, so a new terminal is not enough.
#
#   sudo ./scripts/setup_rt.sh

set -euo pipefail

if [[ $EUID -ne 0 ]]; then
  echo "run me with sudo" >&2
  exit 1
fi

# SUDO_USER is the human who invoked sudo; $USER here would just be root.
TARGET_USER="${SUDO_USER:-$USER}"
LIMITS_FILE="/etc/security/limits.d/99-rt-pid.conf"

# The 'realtime' group is the conventional place for this on Debian/Ubuntu.
if ! getent group realtime >/dev/null; then
  groupadd realtime
  echo "created group 'realtime'"
fi

if ! id -nG "$TARGET_USER" | grep -qw realtime; then
  usermod -aG realtime "$TARGET_USER"
  echo "added '$TARGET_USER' to group 'realtime'"
fi

# rtprio 99  -> allowed to request any SCHED_FIFO priority
# memlock unlimited -> allowed to mlockall() the whole process
# nice -20   -> useful for the non-RT helper threads
cat > "$LIMITS_FILE" <<'LIMITS'
@realtime   -   rtprio      99
@realtime   -   memlock     unlimited
@realtime   -   nice        -20
LIMITS

echo "wrote $LIMITS_FILE"
echo
echo "Now LOG OUT and LOG BACK IN, then verify with:"
echo "    ulimit -r      # should print 99"
echo "    ulimit -l      # should print unlimited"
