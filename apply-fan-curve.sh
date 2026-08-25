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
	for d in /sys/class/hwmon/hwmon*; do
		[ -r "$d/name" ] || continue
		[ "$(cat "$d/name")" = "zimacube_ec" ] || continue
		echo "$d"
		return 0
	done
	return 1
}

H=$(find_hwmon)
if [ -z "$H" ]; then
	modprobe zimacube_ec_fan 2>/dev/null
	# the platform device and hwmon registration are not instantaneous
	i=0
	while [ $i -lt 20 ]; do
		H=$(find_hwmon) && break
		i=$((i + 1))
		sleep 0.1
	done
fi

if [ -z "$H" ]; then
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
	if ! printf '%s\n' "$2" > "$H/$1" 2>/dev/null; then
		echo "failed to write $2 to $H/$1" >&2
		fail=1
	fi
}

# temperatures are millidegrees in sysfs, whole degrees in the config
set_reg pwm1_auto_point1_temp "${CPU_START_TEMP:+$((CPU_START_TEMP * 1000))}"
set_reg pwm1_auto_point2_temp "${CPU_FULL_TEMP:+$((CPU_FULL_TEMP * 1000))}"
set_reg pwm1_auto_point1_pwm  "$CPU_START_PWM"
set_reg pwm1_slope            "$CPU_SLOPE"

set_reg pwm2_auto_point1_temp "${SYS_START_TEMP:+$((SYS_START_TEMP * 1000))}"
set_reg pwm2_auto_point2_temp "${SYS_FULL_TEMP:+$((SYS_FULL_TEMP * 1000))}"
set_reg pwm2_auto_point1_pwm  "$SYS_START_PWM"
set_reg pwm2_slope            "$SYS_SLOPE"

# hand control back to the EC with the new curve
set_reg pwm1_enable 2
set_reg pwm2_enable 2

[ "$fail" = 0 ] || die "one or more registers could not be written"
echo "applied to $H"
