#!/bin/sh
# Apply the EC fan curve from the config file. hwmon numbering is not stable
# across boots, so locate the device by its name.
#
# Deliberately not using `set -e`: a helper that skips an unset config value has
# to be able to return non-zero without killing the script.

CONF=${1:-/etc/zimacube-fan-curve.conf}

die() { echo "$*" >&2; exit 1; }

[ -r "$CONF" ] || die "no readable config at $CONF"
# shellcheck disable=SC1090
. "$CONF" || die "could not parse $CONF"

find_hwmon() {
  for entry in /sys/class/hwmon/hwmon*; do
    [ -r "$entry/name" ] || continue
    [ "$(cat "$entry/name")" = "zimacube_ec" ] || continue
    echo "$entry"
    return 0
  done
  return 1
}

hwmon=$(find_hwmon)
if [ -z "$hwmon" ]; then
  modprobe zimacube_ec_fan 2>/dev/null
  # the platform device and hwmon registration are not instantaneous
  attempt=0
  while [ $attempt -lt 20 ]; do
    hwmon=$(find_hwmon) && break
    attempt=$((attempt + 1))
    sleep 0.1
  done
fi

if [ -z "$hwmon" ]; then
  echo "zimacube_ec hwmon device not found." >&2
  if ! lsmod | grep -q '^zimacube_ec_fan'; then
    echo "  the zimacube_ec_fan module is not loaded, and modprobe did not find it." >&2
    echo "  install it first: make install   (or the dkms install steps)" >&2
  else
    echo "  the module is loaded but did not register hwmon -- check dmesg." >&2
  fi
  exit 1
fi

fail=0
set_reg() {
  # $1 = attribute, $2 = value. An empty value means "leave as BIOS set it".
  [ -n "$2" ] || return 0
  if ! printf '%s\n' "$2" > "$hwmon/$1" 2>/dev/null; then
    echo "failed to write $2 to $hwmon/$1" >&2
    fail=1
  fi
}

# The EC re-reads the curve registers on every control tick, so the curve is
# live while it is being written: a channel already in Automatic starts acting
# on a half-updated curve the moment one write fails. Refusing to switch to
# Automatic afterwards does not help, because the channel is usually there
# already. The only honest answer is to put back what was in effect.
CURVE_ATTRS="pwm1_auto_point1_temp pwm1_auto_point2_temp pwm1_auto_point1_pwm pwm1_slope
pwm2_auto_point1_temp pwm2_auto_point2_temp pwm2_auto_point1_pwm pwm2_slope"

snapshot=$(mktemp) || die "could not create a temporary file for the rollback snapshot"
snapshot_ready=0

cleanup() { rm -f "$snapshot"; }

restore_curve() {
  rc=0
  while read -r attr value; do
    printf '%s\n' "$value" > "$hwmon/$attr" 2>/dev/null || rc=1
  done < "$snapshot"
  return "$rc"
}

force_full_speed() {
  rc=0
  printf '0\n' > "$hwmon/pwm1_enable" 2>/dev/null || rc=1
  printf '0\n' > "$hwmon/pwm2_enable" 2>/dev/null || rc=1
  return "$rc"
}

# Shared by the failure path and the signal path, because a half-applied curve
# is equally dangerous however we got there. Put the previous curve back; if
# even that fails, escalate to full speed, which is loud but is the only state
# that cannot overheat the drives. Returns 0 only when the resulting state is
# one we can vouch for.
recover_curve() {
  if restore_curve; then
    echo "previous curve restored" >&2
    return 0
  fi
  echo "rollback failed" >&2
  if force_full_speed; then
    echo "both channels forced to full speed" >&2
    return 0
  fi
  echo "the channels could not be forced to full speed -- fan state is unknown, check $hwmon by hand" >&2
  return 1
}

# A trap on a signal does not end the script in POSIX sh: the handler runs and
# execution simply carries on. Left at cleanup-only that would delete the
# snapshot and keep writing, so an interrupt in the middle of the curve would
# leave exactly the half-applied state the rollback exists to prevent. Recover
# here, then exit explicitly with the conventional 128 + signal status.
on_signal() {
  if [ "$snapshot_ready" = 1 ]; then
    echo "interrupted while applying the curve; recovering" >&2
    recover_curve
  fi
  cleanup
  exit "$1"
}

trap cleanup EXIT
trap 'on_signal 130' INT
trap 'on_signal 143' TERM
trap 'on_signal 129' HUP

for attr in $CURVE_ATTRS; do
  value=$(cat "$hwmon/$attr" 2>/dev/null) || die "could not read $hwmon/$attr"
  printf '%s %s\n' "$attr" "$value" >> "$snapshot"
done
snapshot_ready=1

# temperatures are millidegrees in sysfs, whole degrees in the config
set_reg pwm1_auto_point1_temp "${CPU_START_TEMP:+$((CPU_START_TEMP * 1000))}"
set_reg pwm1_auto_point2_temp "${CPU_FULL_TEMP:+$((CPU_FULL_TEMP * 1000))}"
set_reg pwm1_auto_point1_pwm  "$CPU_START_PWM"
set_reg pwm1_slope            "$CPU_SLOPE"

set_reg pwm2_auto_point1_temp "${SYS_START_TEMP:+$((SYS_START_TEMP * 1000))}"
set_reg pwm2_auto_point2_temp "${SYS_FULL_TEMP:+$((SYS_FULL_TEMP * 1000))}"
set_reg pwm2_auto_point1_pwm  "$SYS_START_PWM"
set_reg pwm2_slope            "$SYS_SLOPE"

if [ "$fail" != 0 ]; then
  echo "curve incomplete; recovering" >&2
  recover_curve
  die "one or more curve registers could not be written"
fi

# hand control back to the EC only once the whole curve is in place
set_reg pwm1_enable 2
set_reg pwm2_enable 2

[ "$fail" = 0 ] || die "curve written, but the channels could not be switched to automatic"
echo "applied to $hwmon"
