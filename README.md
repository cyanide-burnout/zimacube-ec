# zimacube-ec-fan

Linux hwmon driver for the ITE IT5570E embedded controller on IceWhale **ZimaCube** boards.
Two independent fan channels, three temperatures, the EC's own fan curve exposed as
writable attributes, and the front power LED as a standard LED class device.

Verified on: ZimaCube Pro, BIOS 5.24 (12/10/2025), Debian 13, kernel 6.12.

The register offsets and the behaviour of the controller's own fan curve are documented in
the comments at the top of `zimacube_ec_fan.c`.

## Why this exists

`passiveEndeavour/it5570-fan` gets the *access methods* right — that project is where this
one started — but its register map belongs to a different board. On a ZimaCube it reads
offsets 0x22/0x23 as "fan RPM high/low", when those are actually the two fan **mode** bytes,
both set to 2 (Automatic). The result is a fan that permanently reports
`(2<<8)|2 = 514 RPM`. Its PWM offsets 0x0E/0x0F are not referenced anywhere in this EC's
firmware, so its writes are silent no-ops.

The lesson generalises: a matching Super I/O chip ID says nothing about the EC RAM layout.
This driver therefore refuses to bind on unknown boards — see *Safety gates*.

## Install

```sh
sudo make dkms
sudo make reload
```

That is the whole thing. DKMS keeps the module alive across kernel upgrades; without it the
module has to be rebuilt by hand after every one.

`make dkms` is also the update path — edit the source and run it again, it rebuilds in place.

It works on exactly the version in `dkms.conf` and never touches any other, so bumping
`PACKAGE_VERSION` installs the new build *alongside* the old one rather than replacing it.
Both then stay registered, and with `AUTOINSTALL="yes"` both get rebuilt on every kernel
upgrade, racing for the same module path. After a bump, clear the old one out:

```sh
sudo make dkms-purge
sudo make dkms
```

Note that `make dkms-remove` cannot do this: it also reads the version from `dkms.conf`, so
once the file says 0.4 it will remove 0.4, not the 0.3 you meant. Only `dkms-purge` walks
every registered version. To drop one by hand:
`sudo dkms remove -m zimacube-ec-fan -v 0.3 --all`. Check what is left with
`dkms status -m zimacube-ec-fan`.

| target | what it does |
|---|---|
| `make dkms` | install or update the module via DKMS, plus the support files below |
| `make reload` | reload the module and show the last kernel messages |
| `make install` | plain install without DKMS — breaks on the next kernel upgrade |
| `make install-support` | only the support files, no module |
| `make dkms-remove` | remove the version currently named in `dkms.conf`, not whatever is installed |
| `make dkms-purge` | remove every registered version — the one to use after a version bump |
| `make uninstall` | remove the autoload rule, the apply script and the unit — keeps the curve config and the module. Disable the unit first, see below |

The support files are an autoload rule in `/etc/modules-load.d`, the curve config at
`/etc/zimacube-fan-curve.conf` (never overwritten if it already exists), the apply script,
and a systemd unit.

To have the curve applied at boot:

```sh
sudo systemctl daemon-reload
sudo systemctl enable --now zimacube-fan-curve
```

Removing it again is the same three steps in reverse:

```sh
sudo systemctl disable --now zimacube-fan-curve
sudo make uninstall
sudo systemctl daemon-reload
```

Both bookends matter. Disabling first is what removes the symlink in
`multi-user.target.wants`; delete the unit file while it is still enabled and that symlink is
left pointing at nothing, which systemd reports on every operation afterwards. The reload at
the end is what makes systemd forget the unit it has already parsed.

If `make uninstall` already ran with the unit enabled, `systemctl disable zimacube-fan-curve`
still clears the stale symlink; should it refuse because the unit file is gone, remove
`/etc/systemd/system/multi-user.target.wants/zimacube-fan-curve.service` by hand and reload.

Leaving that unit disabled is fine — the controller then keeps whatever curve BIOS Setup
configured, and the driver gives you readings plus on-demand manual control.

## Attributes

| attribute | meaning |
|---|---|
| `fan1_input`, `fan2_input` | CPU / system fan speed, RPM |
| `pwm1`, `pwm2` | duty, 0–255, rescaled to the EC register by `fullscale` (1:1 at the default 255) |
| `pwm[12]_enable` | 0 = full speed, 1 = manual at the current duty, 2 = the EC's own curve |
| `pwm[12]_auto_point1_temp` | curve start temperature |
| `pwm[12]_auto_point2_temp` | full-speed temperature |
| `pwm[12]_auto_point1_pwm` | duty at the start temperature |
| `pwm[12]_slope` | duty units added per °C |
| `temp1_input` | CPU package — the input to both curves |
| `temp2_input` | board thermistor |
| `temp3_input` | max of the two, as the EC computes it |

The curve the EC runs is `duty = start_pwm + (temp − start_temp) × slope`, forced to 255 at
`auto_point2_temp`, and slew-limited to one step per control tick.

`pwm` is the hwmon 0–255 scale; what reaches the EC is `pwm × fullscale / 255`, where
`fullscale` is a module parameter. At its default of 255 that is 1:1; set it to 122 and a
full 255 lands on the value BIOS caps its own fields at. Values of 0 or above 255 are
ignored — the duty register is 8-bit and a larger one would wrap.

Writing `1` to `pwm[12]_enable` means "hold the speed the fan runs at now", so the driver has
to know that speed, and refuses with `EIO` rather than guess:

* the duty must come from a live read of the PWM register, or from a cached one that came
  from a live read and is still inside the refresh window — a duty inferred from the mode
  alone does not count;
* a stopped fan reads back as 0, and manual mode never looks at temperature again, so the
  value is floored at the channel's own start PWM — the duty the EC itself uses to get a
  stopped fan turning. If that register is 0 too, the write is refused.

The reason is written to the kernel log. Writing `pwm1`/`pwm2` instead sets the duty
outright and switches the channel to manual on the way, with no floor applied — that value
is yours, not a guess, so the driver does not second-guess it.

`/sys/kernel/debug/zimacube_ec_fan/regs` dumps the decoded EC RAM with the curve's predicted
duty next to the live one — the fastest way to check that the map applies to your board.

## Power LED

On ZimaCube Pro, the driver exposes the front power LED as
`/sys/class/leds/zimacube::power`. This interface has been verified on the machine.

| attribute | meaning |
|---|---|
| `brightness` | `0` = off, `1` = steady on; reads `1` while the LED blinks |
| `trigger` | any kernel trigger — `timer`, `heartbeat`, `disk-activity`, … |
| `hw_blink` | `none`, `slow`, `medium`, `fast` — the EC blinks the LED on its own |

```sh
echo heartbeat | sudo tee /sys/class/leds/zimacube::power/trigger
echo slow      | sudo tee /sys/class/leds/zimacube::power/hw_blink
echo 1         | sudo tee /sys/class/leds/zimacube::power/brightness
```

Kernel triggers blink in software, one EC write per edge. `hw_blink` costs nothing after the
write, but its three rates are fixed by the EC and have not been timed, which is why they are
not offered through the LED core's `blink_set`: that interface has to report the delays it
actually applies. Writing `hw_blink` detaches any trigger first, and `none` leaves the LED
steady on. Writing `brightness` ends a hardware blink.

The LED is registered only on a verified ZimaCube Pro, and only if the register holds one of
the five known states at load time. Otherwise the fans still bind and the kernel log says
why the LED did not. On unload the LED is put back the way it was found. Other ZimaCube
models drive their LED from a Super I/O GPIO rather than the EC, which this driver does not
touch.

## Safety gates

Binding is gated twice, because getting this wrong means writing duty values into registers
that mean something else entirely:

1. **DMI.** `ZimaCube Pro` binds silently. Other `IceWhale` + `ZimaCube` boards bind with a
   warning that the layout is unverified there. Anything else is refused.
2. **Content.** Fan modes must be 0–4, start/full temperature pairs ordered and ≤127 °C,
   speeds under 20000 RPM, temperatures ≤110 °C. A board that merely shares the IT5570
   chip ID will almost certainly fail one of these.

`force=1` skips both. On an unknown layout it can drive the fans to an arbitrary duty, so
check the debugfs dump below before trusting anything it reports.

Raw EC writes through `debugfs/ec_raw` are **compiled out by default** — a loaded gun
pointed at a controller that also owns thermal and power state. Enable deliberately:

```sh
make CFLAGS_zimacube_ec_fan.o=-DZC_ALLOW_EC_WRITE
```

## Two things worth knowing

**BIOS manual mode is capped at 48 %, the auto curve is not.** Setup's duty fields accept at
most 122 of 255 — roughly 1761 RPM on the CPU fan against 3450 measured at full duty. The
computed `start_pwm + slope × Δt` is *not* clamped to 122, so the stock curve does reach 255
at 79 °C. What this driver adds is the full manual range, plus start PWMs and slopes outside
the ranges Setup offers.

**`temp1` is a lagged reading.** It trails the real package temperature in both directions —
measured 6 °C low while heating, 3 °C high while cooling, exact once settled. That lag stacks
with the duty slew limiter, so the fans are slow both to spin up when load arrives and to
quiet down when it leaves. Compensate with a steeper `slope` rather than a lower
`auto_point1_temp`.

## Credits

* [passiveEndeavour/it5570-fan](https://github.com/passiveEndeavour/it5570-fan).
  Its Super I/O detection and indirect-window access patterns are correct and were the
  starting point here; only its register map does not apply to this board.
* [IceWhaleTech/ZimaCube-PowerLED](https://github.com/IceWhaleTech/ZimaCube-PowerLED).
  The vendor's Power LED utility provided the ZimaCube Pro LED control values.

## Provenance

This driver was written for interoperability: to make the fans and power LED of hardware its
author owns controllable from Linux, where no such support existed.

What is published here is confined to that purpose: the driver itself, and the register
offsets it cannot function without. No firmware is redistributed, nor any disassembly of it,
nor any account of the controller's internal implementation.

Not affiliated with or endorsed by IceWhale Technology, AMI, or ITE Tech. Product names
identify the supported hardware and nothing more.

## Licence

GPL-2.0-only. See [LICENSE](LICENSE).
