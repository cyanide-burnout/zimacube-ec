// SPDX-License-Identifier: GPL-2.0-only
/*
 * IceWhale ZimaCube (ITE IT5570E EC) hwmon driver — two independent fan channels.
 *
 * The controller exposes its fan registers through the standard ACPI EC window on
 * ports 0x62/0x66 (command 0x80 reads a byte, 0x81 writes one). The layout is
 * board-specific: a matching Super I/O chip ID says nothing about it, hence the
 * DMI and content checks in zc_init().
 *
 * Channel 0 = CPU fan, channel 1 = system fan.
 *
 * The controller re-reads 0x22..0x2D on every control tick, so writes take effect
 * immediately and nothing has to be handed over first. It never writes those
 * registers itself; after a cold boot they hold what the BIOS put there.
 *
 * Curve run by the controller when a channel is in mode 2:
 *
 *   duty = start_pwm + (EC[0x70] - start_temp) * slope
 *
 * forced to full scale once EC[0x70] reaches full_temp, dropped to 0 once it
 * falls ~5 degC below start_temp, and slew-limited to one step per control tick
 * so it ramps rather than jumps. A channel sitting at duty 0 with a non-zero
 * target is kicked to start_pwm first. `slope` is a plain integer multiplier in
 * duty units per degC -- the "0.125 .. 15.875 PWM" labels in BIOS Setup do not
 * describe what the controller computes.
 *
 * EC[0x70] is a lagged reading: measured 6 degC below the coretemp package
 * temperature while heating, 3 degC above while cooling, equal once settled. That
 * lag stacks with the slew limiter, so the fans are slow both to spin up and to
 * quiet down. Compensate with a steeper slope, not a lower start temperature.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/acpi.h>
#include <linux/debugfs.h>
#include <linux/dmi.h>
#include <linux/delay.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>

#define DRVNAME			"zimacube_ec_fan"

/* ---- Super I/O (chip detection + indirect register window) ---------------- */
#define SIO_ADDR		0x4e
#define SIO_DATA		0x4f
#define IT5570_CHIPID		0x5570

/* ITE indirect memory access: SIO cfg reg 0x2e selects a sub-register, 0x2f is data */
#define SIO_IMA_SELECT		0x2e
#define SIO_IMA_DATA		0x2f
#define SIO_IMA_ADDR_LO		0x10
#define SIO_IMA_ADDR_HI		0x11
#define SIO_IMA_XFER		0x12

/* ---- ACPI EC register window, ports 0x62/0x66 ---------------------------- */
#define EC_SYS_FAN_MODE		0x22	/* 0=off 1=manual 2=auto 3=full     */
#define EC_CPU_FAN_MODE		0x23
#define EC_SYS_SLOPE		0x24	/* duty units per degC              */
#define EC_SYS_START_PWM	0x25	/* 0..122                          */
#define EC_SYS_START_TEMP	0x26	/* degC                            */
#define EC_SYS_FULL_TEMP	0x27	/* degC                            */
#define EC_CPU_SLOPE		0x28
#define EC_CPU_START_PWM	0x29
#define EC_CPU_START_TEMP	0x2a
#define EC_CPU_FULL_TEMP	0x2b
#define EC_SYS_MANUAL_PWM	0x2c	/* 0..122                          */
#define EC_CPU_MANUAL_PWM	0x2d
#define EC_TEMP_CTRL		0x70	/* CPU package temp, drives curves */
#define EC_TEMP_MAX		0x71	/* max(0x70, 0x72)                 */
#define EC_TEMP_BOARD		0x72	/* board/aux thermistor            */
#define EC_CPU_RPM_HI		0x76	/* big-endian 16-bit, already RPM  */
#define EC_CPU_RPM_LO		0x77
#define EC_SYS_RPM_HI		0x78
#define EC_SYS_RPM_LO		0x79
#define EC_CPU_TEMP10_LO	0xa1	/* little-endian 16-bit, degC * 10  */
#define EC_CPU_TEMP10_HI	0xa2

/*
 * Live PWM duty, reachable only through the Super I/O indirect window: the
 * controller does not mirror it into the ACPI EC space.
 */
#define REG_PWM_SYS_DUTY	0x1803
#define REG_PWM_CPU_DUTY	0x1809

/* EC firmware fan modes */
#define FAN_MODE_OFF		0
#define FAN_MODE_MANUAL		1
#define FAN_MODE_AUTO		2
#define FAN_MODE_FULL		3
/* mode 4 exists in BIOS Setup but the controller does not decode it — never write it */

/*
 * The BIOS Setup page caps every duty field at 122, but the EC applies the value
 * to the PWM duty register unchanged (measured: EC 0x2D = 61 -> duty 61,
 * 122 -> duty 122), and mode 3 drives the register to 255. So the real
 * scale is a plain 8-bit 0..255 and 122 is a UI limit only -- which means Linux
 * can command duties, start PWMs and slopes that BIOS Setup cannot express.
 */
#define EC_PWM_SCALE		255	/* duty register full scale */
#define EC_BIOS_PWM_CAP		122	/* what BIOS Setup allows */
#define HWMON_PWM_MAX		255

#define CACHE_TTL		(HZ / 2)

struct chan_regs {
	u8 mode, slope, start_pwm, start_temp, full_temp, manual_pwm;
	u8 rpm_hi, rpm_lo;
	u16 duty_reg;
	const char *label;
};

static const struct chan_regs chan[2] = {
	[0] = {	/* CPU fan */
		.mode = EC_CPU_FAN_MODE,   .slope = EC_CPU_SLOPE,
		.start_pwm = EC_CPU_START_PWM, .start_temp = EC_CPU_START_TEMP,
		.full_temp = EC_CPU_FULL_TEMP, .manual_pwm = EC_CPU_MANUAL_PWM,
		.rpm_hi = EC_CPU_RPM_HI,   .rpm_lo = EC_CPU_RPM_LO,
		.duty_reg = REG_PWM_CPU_DUTY,
		.label = "CPU Fan",
	},
	[1] = {	/* System fan */
		.mode = EC_SYS_FAN_MODE,   .slope = EC_SYS_SLOPE,
		.start_pwm = EC_SYS_START_PWM, .start_temp = EC_SYS_START_TEMP,
		.full_temp = EC_SYS_FULL_TEMP, .manual_pwm = EC_SYS_MANUAL_PWM,
		.rpm_hi = EC_SYS_RPM_HI,   .rpm_lo = EC_SYS_RPM_LO,
		.duty_reg = REG_PWM_SYS_DUTY,
		.label = "System Fan",
	},
};

struct zc_data {
	struct mutex lock;		/* serialises EC + SIO access */
	u8  mode_at_probe[2];		/* restored on unload */
	bool mode_saved;
	unsigned long updated;
	bool valid;

	u16 rpm[2];
	u8  mode[2];
	u8  duty[2];			/* 0..EC_PWM_SCALE, derived from hardware */
	u8  fullscale;			/* duty value meaning 100 %, 0 if unknown */
	u8  temp_ctrl, temp_max, temp_board;
	u16 temp_cpu10;
};

static bool force;
module_param(force, bool, 0444);
MODULE_PARM_DESC(force,
	"Bind even if the DMI data does not match a known-good board. The EC RAM layout this driver writes is board-specific: a different IT5570 machine will have a different map, so forcing can drive the fans to an arbitrary duty. Only set this if you have verified the layout yourself.");

static bool sio_ima_ok = true;
module_param_named(sio_ima, sio_ima_ok, bool, 0444);
MODULE_PARM_DESC(sio_ima, "Use the Super I/O indirect window to read live PWM duty (default on)");

static unsigned int fullscale = EC_PWM_SCALE;
module_param(fullscale, uint, 0644);
MODULE_PARM_DESC(fullscale,
	"PWM duty register value that means 100% (default 255, measured via Full Speed mode). Set to 122 to stay inside the range BIOS Setup uses.");

/* ------------------------------------------------------------------ Super I/O */

static void sio_enter(void)
{
	outb(0x87, SIO_ADDR);
	outb(0x01, SIO_ADDR);
	outb(0x55, SIO_ADDR);
	outb(0xaa, SIO_ADDR);
}

static void sio_leave(void)
{
	outb(0x02, SIO_ADDR);
	outb(0x02, SIO_DATA);
}

static u8 sio_inb(u8 reg)
{
	outb(reg, SIO_ADDR);
	return inb(SIO_DATA);
}

static void sio_outb(u8 reg, u8 val)
{
	outb(reg, SIO_ADDR);
	outb(val, SIO_DATA);
}

/*
 * Read one byte through the ITE indirect memory-access window. Must be called
 * with sio_enter() already done and the 0x4e/0x4f region held.
 */
static u8 sio_ind_read(u16 addr)
{
	sio_outb(SIO_IMA_SELECT, SIO_IMA_ADDR_HI);
	sio_outb(SIO_IMA_DATA, addr >> 8);
	sio_outb(SIO_IMA_SELECT, SIO_IMA_ADDR_LO);
	sio_outb(SIO_IMA_DATA, addr & 0xff);
	sio_outb(SIO_IMA_SELECT, SIO_IMA_XFER);
	/* select the data sub-register, then read it (outb takes value, port) */
	outb(SIO_IMA_DATA, SIO_ADDR);
	return inb(SIO_DATA);
}

/* Read a batch of indirect-window addresses in one Super I/O session. */
static int sio_ind_read_batch(const u16 *addr, u8 *out, int n)
{
	int i;

	if (!sio_ima_ok)
		return -ENODEV;

	if (!request_muxed_region(SIO_ADDR, 2, DRVNAME))
		return -EBUSY;

	sio_enter();
	for (i = 0; i < n; i++)
		out[i] = sio_ind_read(addr[i]);
	sio_leave();

	release_region(SIO_ADDR, 2);
	return 0;
}

/*
 * Read the live duty of both channels.
 *
 * The duty registers are on the same 0..fullscale scale the EC applies to the
 * BIOS values, so no rescaling is done here.
 */
static int read_live_duty(struct zc_data *d)
{
	static const u16 addr[2] = { REG_PWM_CPU_DUTY, REG_PWM_SYS_DUTY };
	u8 v[2];
	unsigned int fs = fullscale ? fullscale : EC_PWM_SCALE;
	int ret, i;

	ret = sio_ind_read_batch(addr, v, 2);
	if (ret)
		return ret;

	d->fullscale = fs;
	for (i = 0; i < 2; i++)
		d->duty[i] = min_t(unsigned int, v[i], fs);
	return 0;
}

static int sio_chipid(u16 *id)
{
	if (!request_muxed_region(SIO_ADDR, 2, DRVNAME))
		return -EBUSY;

	sio_enter();
	*id = (sio_inb(0x20) << 8) | sio_inb(0x21);
	sio_leave();

	release_region(SIO_ADDR, 2);
	return 0;
}

/* ----------------------------------------------------------------- EC access */

static int ec_r(u8 off, u8 *val)
{
	return ec_read(off, val);
}

static int ec_w(u8 off, u8 val)
{
	return ec_write(off, val);
}

static int ec_r16be(u8 hi_off, u16 *val)
{
	u8 hi, lo;
	int ret;

	ret = ec_r(hi_off, &hi);
	if (ret)
		return ret;
	ret = ec_r(hi_off + 1, &lo);
	if (ret)
		return ret;
	*val = (hi << 8) | lo;
	return 0;
}

static int zc_update(struct zc_data *d)
{
	int i, ret = 0;
	u8 lo, hi;

	mutex_lock(&d->lock);
	if (d->valid && time_before(jiffies, d->updated + CACHE_TTL))
		goto out;

	for (i = 0; i < 2; i++) {
		ret = ec_r16be(chan[i].rpm_hi, &d->rpm[i]);
		if (ret)
			goto out;
		ret = ec_r(chan[i].mode, &d->mode[i]);
		if (ret)
			goto out;
	}

	ret = ec_r(EC_TEMP_CTRL, &d->temp_ctrl);
	if (ret)
		goto out;
	ret = ec_r(EC_TEMP_MAX, &d->temp_max);
	if (ret)
		goto out;
	ret = ec_r(EC_TEMP_BOARD, &d->temp_board);
	if (ret)
		goto out;

	/* CPU temperature in tenths of a degree — note: little-endian here, unlike RPM */
	ret = ec_r(EC_CPU_TEMP10_LO, &lo);
	if (ret)
		goto out;
	ret = ec_r(EC_CPU_TEMP10_HI, &hi);
	if (ret)
		goto out;
	d->temp_cpu10 = (hi << 8) | lo;

	if (read_live_duty(d)) {
		/*
		 * No indirect window: fall back to what we told the EC to do.
		 * In auto mode we cannot know the duty, so report the configured
		 * manual value only when the channel is actually in manual mode.
		 */
		d->fullscale = 0;
		for (i = 0; i < 2; i++) {
			u8 v = 0;

			if (d->mode[i] == FAN_MODE_MANUAL)
				ec_r(chan[i].manual_pwm, &v);
			else if (d->mode[i] == FAN_MODE_FULL)
				v = fullscale ? fullscale : EC_PWM_SCALE;
			d->duty[i] = v;
		}
	}

	d->updated = jiffies;
	d->valid = true;
out:
	mutex_unlock(&d->lock);
	return ret;
}

static inline long ec_to_hwmon_pwm(u8 ec)
{
	unsigned int fs = fullscale ? fullscale : EC_PWM_SCALE;

	return DIV_ROUND_CLOSEST(min_t(unsigned int, ec, fs) * HWMON_PWM_MAX, fs);
}

static inline u8 hwmon_to_ec_pwm(long pwm)
{
	unsigned int fs = fullscale ? fullscale : EC_PWM_SCALE;

	pwm = clamp_val(pwm, 0, HWMON_PWM_MAX);
	return DIV_ROUND_CLOSEST(pwm * fs, HWMON_PWM_MAX);
}

/* -------------------------------------------------------------------- hwmon */

static umode_t zc_is_visible(const void *drvdata, enum hwmon_sensor_types type,
			     u32 attr, int channel)
{
	switch (type) {
	case hwmon_fan:
		return 0444;
	case hwmon_temp:
		return 0444;
	case hwmon_pwm:
		switch (attr) {
		case hwmon_pwm_input:
		case hwmon_pwm_enable:
		case hwmon_pwm_auto_channels_temp:
			return 0644;
		default:
			return 0;
		}
	default:
		return 0;
	}
}

static int zc_read(struct device *dev, enum hwmon_sensor_types type, u32 attr,
		   int channel, long *val)
{
	struct zc_data *d = dev_get_drvdata(dev);
	int ret = zc_update(d);

	if (ret)
		return ret;

	switch (type) {
	case hwmon_fan:
		*val = d->rpm[channel];
		return 0;

	case hwmon_pwm:
		switch (attr) {
		case hwmon_pwm_input:
			*val = ec_to_hwmon_pwm(d->duty[channel]);
			return 0;
		case hwmon_pwm_enable:
			switch (d->mode[channel]) {
			case FAN_MODE_MANUAL:	*val = 1; break;
			case FAN_MODE_AUTO:	*val = 2; break;
			case FAN_MODE_FULL:	*val = 0; break;
			case FAN_MODE_OFF:	*val = 1; break;
			default:
				/*
				 * BIOS "Silent" (4) is not decoded by the EC, so the
				 * channel is effectively uncontrolled. Report that.
				 */
				*val = 0;
				break;
			}
			return 0;
		case hwmon_pwm_auto_channels_temp:
			*val = 1;	/* both curves are driven by temp1 (EC 0x70) */
			return 0;
		default:
			return -EOPNOTSUPP;
		}

	case hwmon_temp:
		switch (channel) {
		case 0: *val = d->temp_cpu10 * 100; return 0;	/* decidegrees */
		case 1: *val = d->temp_board * 1000; return 0;
		case 2: *val = d->temp_max * 1000; return 0;
		default: return -EOPNOTSUPP;
		}

	default:
		return -EOPNOTSUPP;
	}
}

static int zc_write(struct device *dev, enum hwmon_sensor_types type, u32 attr,
		    int channel, long val)
{
	struct zc_data *d = dev_get_drvdata(dev);
	int ret;

	if (type != hwmon_pwm)
		return -EOPNOTSUPP;

	mutex_lock(&d->lock);

	switch (attr) {
	case hwmon_pwm_input: {
		u8 ec = hwmon_to_ec_pwm(val);

		ret = ec_w(chan[channel].manual_pwm, ec);
		if (!ret)
			ret = ec_w(chan[channel].mode, FAN_MODE_MANUAL);
		break;
	}
	case hwmon_pwm_enable:
		switch (val) {
		case 0:	/* no control: run flat out */
			ret = ec_w(chan[channel].mode, FAN_MODE_FULL);
			break;
		case 1:	/* manual: freeze at the duty the fan is actually running at */
			if (read_live_duty(d))
				d->duty[channel] = d->valid ? d->duty[channel] : 60;
			ret = ec_w(chan[channel].manual_pwm, d->duty[channel]);
			if (!ret)
				ret = ec_w(chan[channel].mode, FAN_MODE_MANUAL);
			break;
		case 2:	/* hand back to the EC's own curve */
			ret = ec_w(chan[channel].mode, FAN_MODE_AUTO);
			break;
		default:
			ret = -EINVAL;
		}
		break;
	default:
		ret = -EOPNOTSUPP;
	}

	if (!ret)
		d->valid = false;
	mutex_unlock(&d->lock);
	return ret;
}

static int zc_read_string(struct device *dev, enum hwmon_sensor_types type,
			  u32 attr, int channel, const char **str)
{
	static const char * const temp_labels[] = { "CPU", "Board", "Max" };

	if (type == hwmon_fan && attr == hwmon_fan_label) {
		*str = chan[channel].label;
		return 0;
	}
	if (type == hwmon_temp && attr == hwmon_temp_label &&
	    channel < ARRAY_SIZE(temp_labels)) {
		*str = temp_labels[channel];
		return 0;
	}
	return -EOPNOTSUPP;
}

static const struct hwmon_channel_info * const zc_info[] = {
	HWMON_CHANNEL_INFO(fan,
		HWMON_F_INPUT | HWMON_F_LABEL,
		HWMON_F_INPUT | HWMON_F_LABEL),
	HWMON_CHANNEL_INFO(pwm,
		HWMON_PWM_INPUT | HWMON_PWM_ENABLE | HWMON_PWM_AUTO_CHANNELS_TEMP,
		HWMON_PWM_INPUT | HWMON_PWM_ENABLE | HWMON_PWM_AUTO_CHANNELS_TEMP),
	HWMON_CHANNEL_INFO(temp,
		HWMON_T_INPUT | HWMON_T_LABEL,
		HWMON_T_INPUT | HWMON_T_LABEL,
		HWMON_T_INPUT | HWMON_T_LABEL),
	NULL
};

static const struct hwmon_ops zc_ops = {
	.is_visible	= zc_is_visible,
	.read		= zc_read,
	.read_string	= zc_read_string,
	.write		= zc_write,
};

static const struct hwmon_chip_info zc_chip_info = {
	.ops	= &zc_ops,
	.info	= zc_info,
};

/* ------------------------------------------- curve attributes (custom group) */

static struct zc_data *attr_data(struct device *dev)
{
	return dev_get_drvdata(dev);
}

static ssize_t curve_show(struct device *dev, struct device_attribute *da,
			  char *buf)
{
	struct sensor_device_attribute_2 *a = to_sensor_dev_attr_2(da);
	struct zc_data *d = attr_data(dev);
	u8 off, v;
	int ret;

	switch (a->nr) {
	case 0: off = chan[a->index].start_temp; break;
	case 1: off = chan[a->index].full_temp;  break;
	case 2: off = chan[a->index].start_pwm;  break;
	case 3: off = chan[a->index].slope;      break;
	default: return -EINVAL;
	}

	mutex_lock(&d->lock);
	ret = ec_r(off, &v);
	mutex_unlock(&d->lock);
	if (ret)
		return ret;

	if (a->nr <= 1)			/* temperatures, in millidegrees */
		return sysfs_emit(buf, "%d\n", v * 1000);
	if (a->nr == 2)			/* start PWM, in hwmon 0..255 units */
		return sysfs_emit(buf, "%ld\n", ec_to_hwmon_pwm(v));
	return sysfs_emit(buf, "%u\n", v);	/* slope, raw duty units per degC */
}

static ssize_t curve_store(struct device *dev, struct device_attribute *da,
			   const char *buf, size_t count)
{
	struct sensor_device_attribute_2 *a = to_sensor_dev_attr_2(da);
	struct zc_data *d = attr_data(dev);
	long in;
	u8 off, v;
	int ret;

	ret = kstrtol(buf, 10, &in);
	if (ret)
		return ret;

	switch (a->nr) {
	case 0:
		off = chan[a->index].start_temp;
		v = clamp_val(in / 1000, 0, 127);
		break;
	case 1:
		off = chan[a->index].full_temp;
		v = clamp_val(in / 1000, 0, 127);
		break;
	case 2:
		off = chan[a->index].start_pwm;
		v = hwmon_to_ec_pwm(in);
		break;
	case 3:
		off = chan[a->index].slope;
		v = clamp_val(in, 0, 255);
		break;
	default:
		return -EINVAL;
	}

	mutex_lock(&d->lock);
	ret = ec_w(off, v);
	d->valid = false;
	mutex_unlock(&d->lock);

	return ret ? ret : count;
}

static SENSOR_DEVICE_ATTR_2_RW(pwm1_auto_point1_temp, curve, 0, 0);
static SENSOR_DEVICE_ATTR_2_RW(pwm1_auto_point2_temp, curve, 1, 0);
static SENSOR_DEVICE_ATTR_2_RW(pwm1_auto_point1_pwm,  curve, 2, 0);
static SENSOR_DEVICE_ATTR_2_RW(pwm1_slope,            curve, 3, 0);
static SENSOR_DEVICE_ATTR_2_RW(pwm2_auto_point1_temp, curve, 0, 1);
static SENSOR_DEVICE_ATTR_2_RW(pwm2_auto_point2_temp, curve, 1, 1);
static SENSOR_DEVICE_ATTR_2_RW(pwm2_auto_point1_pwm,  curve, 2, 1);
static SENSOR_DEVICE_ATTR_2_RW(pwm2_slope,            curve, 3, 1);

static struct attribute *zc_curve_attrs[] = {
	&sensor_dev_attr_pwm1_auto_point1_temp.dev_attr.attr,
	&sensor_dev_attr_pwm1_auto_point2_temp.dev_attr.attr,
	&sensor_dev_attr_pwm1_auto_point1_pwm.dev_attr.attr,
	&sensor_dev_attr_pwm1_slope.dev_attr.attr,
	&sensor_dev_attr_pwm2_auto_point1_temp.dev_attr.attr,
	&sensor_dev_attr_pwm2_auto_point2_temp.dev_attr.attr,
	&sensor_dev_attr_pwm2_auto_point1_pwm.dev_attr.attr,
	&sensor_dev_attr_pwm2_slope.dev_attr.attr,
	NULL
};
ATTRIBUTE_GROUPS(zc_curve);


/* ------------------------------------------------------------------- debugfs */

static struct dentry *zc_debugfs;

static const u16 dump_duty[] = { REG_PWM_CPU_DUTY, REG_PWM_SYS_DUTY };

static const char *mode_name(u8 m)
{
	switch (m) {
	case FAN_MODE_OFF:	return "off";
	case FAN_MODE_MANUAL:	return "manual";
	case FAN_MODE_AUTO:	return "automatic";
	case FAN_MODE_FULL:	return "full speed";
	case 4:			return "SILENT (not implemented by EC!)";
	default:		return "unknown";
	}
}

static int zc_regs_show(struct seq_file *sf, void *unused)
{
	u8 ec[0x100];
	u8 duty[ARRAY_SIZE(dump_duty)];
	bool have_duty;
	int i, ch;

	memset(ec, 0, sizeof(ec));
	for (i = 0x20; i <= 0x37; i++)
		ec_r(i, &ec[i]);
	for (i = 0x70; i <= 0x79; i++)
		ec_r(i, &ec[i]);
	ec_r(0xa1, &ec[0xa1]);
	ec_r(0xa2, &ec[0xa2]);

	have_duty = sio_ind_read_batch(dump_duty, duty, ARRAY_SIZE(dump_duty)) == 0;

	seq_puts(sf, "ACPI EC register window\n");
	for (i = 0x20; i <= 0x30; i += 0x10) {
		int j;

		seq_printf(sf, "  %02x:", i);
		for (j = 0; j < 16 && i + j <= 0x37; j++)
			seq_printf(sf, " %02x", ec[i + j]);
		seq_puts(sf, "\n");
	}
	seq_puts(sf, "  70:");
	for (i = 0x70; i <= 0x79; i++)
		seq_printf(sf, " %02x", ec[i]);
	seq_puts(sf, "\n");

	for (ch = 0; ch < 2; ch++) {
		const struct chan_regs *c = &chan[ch];
		u8 mode = ec[c->mode];
		int start = ec[c->start_pwm], slope = ec[c->slope];
		int st = ec[c->start_temp], ft = ec[c->full_temp];
		int t = ec[EC_TEMP_CTRL], pred;
		unsigned int fs = fullscale ? fullscale : EC_PWM_SCALE;

		seq_printf(sf, "\n%s (channel %d)\n", c->label, ch);
		seq_printf(sf, "  mode        0x%02x = %u (%s)\n",
			   c->mode, mode, mode_name(mode));
		seq_printf(sf, "  manual pwm  0x%02x = %u/%u\n",
			   c->manual_pwm, ec[c->manual_pwm], fs);
		seq_printf(sf, "  start pwm   0x%02x = %u/%u\n", c->start_pwm, start, fs);
		seq_printf(sf, "  slope       0x%02x = %u\n", c->slope, slope);
		seq_printf(sf, "  start temp  0x%02x = %u C\n", c->start_temp, st);
		seq_printf(sf, "  full temp   0x%02x = %u C\n", c->full_temp, ft);
		seq_printf(sf, "  rpm         0x%02x = %u\n",
			   c->rpm_hi, (ec[c->rpm_hi] << 8) | ec[c->rpm_lo]);

		if (t >= ft)
			pred = fs;
		else if (t < st)
			pred = -1;
		else
			pred = start + (t - st) * slope;

		if (pred < 0)
			seq_puts(sf, "  auto curve  T below start temp -> holds previous\n");
		else
			seq_printf(sf, "  auto curve  predicts %d/%u at T=%d C%s\n",
				   pred, fs, t,
				   pred > EC_BIOS_PWM_CAP ? "  (above the BIOS UI cap of 122)" : "");

		if (have_duty) {
			u8 dcr = duty[ch];

			seq_printf(sf, "  live duty   %u/%u", dcr, fs);
			if (mode == FAN_MODE_AUTO && pred >= 0)
				seq_printf(sf, "  (prediction %d, delta %d)",
					   pred, (int)dcr - pred);
			seq_puts(sf, "\n");
		}
	}

	seq_printf(sf, "\ntemps: ctrl(0x70)=%u C  max(0x71)=%u C  board(0x72)=%u C  cpu*10(0xA1)=%u\n",
		   ec[0x70], ec[0x71], ec[0x72], (ec[0xa2] << 8) | ec[0xa1]);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(zc_regs);



/*
 * Raw access to the ACPI EC window, for probing registers that have no mirror in
 * the hwmon ABI.
 *
 * Reading dumps 0x00..0xFF.
 *
 * The write side pokes an embedded controller that also owns thermal and power
 * state, and there is no safe generic offset list to validate against, so it is
 * compiled out unless you ask for it:
 *
 *     make CFLAGS_zimacube_ec_fan.o=-DZC_ALLOW_EC_WRITE
 *
 * It takes "<offset> <value>" in hex, is root-only and debugfs-only, and logs
 * every write.
 */
static int zc_ec_raw_show(struct seq_file *sf, void *unused)
{
	int i, j;

	for (i = 0; i < 0x100; i += 16) {
		seq_printf(sf, "%02x:", i);
		for (j = 0; j < 16; j++) {
			u8 v;

			if (ec_r(i + j, &v))
				seq_puts(sf, " --");
			else
				seq_printf(sf, " %02x", v);
		}
		seq_puts(sf, "\n");
	}
	return 0;
}

static int zc_ec_raw_open(struct inode *inode, struct file *file)
{
	return single_open(file, zc_ec_raw_show, inode->i_private);
}

#ifdef ZC_ALLOW_EC_WRITE
static ssize_t zc_ec_raw_write(struct file *file, const char __user *ubuf,
			       size_t len, loff_t *ppos)
{
	struct zc_data *d = file_inode(file)->i_private;
	char buf[32];
	unsigned int off, val;
	int ret;

	if (len >= sizeof(buf))
		return -EINVAL;
	if (copy_from_user(buf, ubuf, len))
		return -EFAULT;
	buf[len] = '\0';

	if (sscanf(buf, "%x %x", &off, &val) != 2)
		return -EINVAL;
	if (off > 0xff || val > 0xff)
		return -EINVAL;

	pr_warn("debugfs write: EC RAM 0x%02x = 0x%02x\n", off, val);

	mutex_lock(&d->lock);
	ret = ec_w(off, val);
	d->valid = false;
	mutex_unlock(&d->lock);

	return ret ? ret : len;
}
#define ZC_EC_RAW_MODE	0600
#else
#define zc_ec_raw_write	NULL
#define ZC_EC_RAW_MODE	0400
#endif

static const struct file_operations zc_ec_raw_fops = {
	.owner		= THIS_MODULE,
	.open		= zc_ec_raw_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
	.write		= zc_ec_raw_write,
};

/* ---------------------------------------------------------------- DMI gating */

/*
 * The register layout this driver writes to is board-specific, and a matching
 * Super I/O chip ID says nothing about it -- that assumption is exactly why an
 * earlier IT5570 driver read a pair of fan mode bytes as "514 RPM" here. So bind
 * only on boards that have actually been checked.
 */
/* Verified: the register map in this driver was confirmed on this exact board. */
static const struct dmi_system_id zc_dmi_verified[] = {
	{
		.ident = "IceWhale ZimaCube Pro",
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "IceWhale"),
			DMI_MATCH(DMI_BOARD_NAME, "ZimaCube Pro"),
		},
	},
	{ }
};

/* Same family, layout not verified -- bind, but say so. */
static const struct dmi_system_id zc_dmi_family[] = {
	{
		.ident = "IceWhale ZimaCube (unverified variant)",
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "IceWhale"),
			DMI_MATCH(DMI_BOARD_NAME, "ZimaCube"),
		},
	},
	{ }
};

static void zc_print_dmi(void)
{
	pr_info("  sys_vendor=\"%s\" board_name=\"%s\" bios_version=\"%s\"\n",
		dmi_get_system_info(DMI_SYS_VENDOR) ?: "?",
		dmi_get_system_info(DMI_BOARD_NAME) ?: "?",
		dmi_get_system_info(DMI_BIOS_VERSION) ?: "?");
}

static int zc_check_dmi(void)
{
	const struct dmi_system_id *id;

	id = dmi_first_match(zc_dmi_verified);
	if (id) {
		pr_info("matched verified board: %s\n", id->ident);
		return 0;
	}

	id = dmi_first_match(zc_dmi_family);
	if (id) {
		pr_warn("%s: the EC RAM layout was verified on ZimaCube Pro only.\n",
			id->ident);
		pr_warn("  Check /sys/kernel/debug/%s/regs before trusting pwm writes.\n",
			DRVNAME);
		zc_print_dmi();
		return 0;
	}

	pr_err("not a known board, refusing to bind.\n");
	zc_print_dmi();
	pr_err("  A matching Super I/O chip ID does not imply a matching EC RAM layout.\n");
	pr_err("  force=1 overrides this; read the module parameter description first.\n");
	return -ENODEV;
}

/*
 * Content-based sanity check.
 *
 * Stronger than DMI: if this is the expected layout, the bytes have to look like
 * it. A board that merely shares the IT5570 chip ID will almost certainly fail at
 * least one of these, which stops us from writing duty values into whatever those
 * offsets mean over there.
 */
static int zc_sanity_check(void)
{
	u8 m[2], st[2], ft[2], t70, t71, t72;
	u16 rpm[2];
	int i, ret;

	for (i = 0; i < 2; i++) {
		ret = ec_r(chan[i].mode, &m[i]);
		if (ret)
			return ret;
		ret = ec_r(chan[i].start_temp, &st[i]);
		if (ret)
			return ret;
		ret = ec_r(chan[i].full_temp, &ft[i]);
		if (ret)
			return ret;
		ret = ec_r16be(chan[i].rpm_hi, &rpm[i]);
		if (ret)
			return ret;
	}
	ret = ec_r(EC_TEMP_CTRL, &t70);
	if (!ret)
		ret = ec_r(EC_TEMP_MAX, &t71);
	if (!ret)
		ret = ec_r(EC_TEMP_BOARD, &t72);
	if (ret)
		return ret;

	for (i = 0; i < 2; i++) {
		if (m[i] > 4) {
			pr_err("EC[0x%02x] = %u is not a valid fan mode\n",
			       chan[i].mode, m[i]);
			return -ENODEV;
		}
		if (st[i] > 127 || ft[i] > 127 || st[i] > ft[i]) {
			pr_err("EC[0x%02x]/[0x%02x] = %u/%u are not a sane temperature pair\n",
			       chan[i].start_temp, chan[i].full_temp, st[i], ft[i]);
			return -ENODEV;
		}
		if (rpm[i] > 20000) {
			pr_err("EC[0x%02x] = %u is not a plausible fan speed\n",
			       chan[i].rpm_hi, rpm[i]);
			return -ENODEV;
		}
	}
	if (t70 > 110 || t72 > 110) {
		pr_err("EC[0x70]/[0x72] = %u/%u are not plausible temperatures\n",
		       t70, t72);
		return -ENODEV;
	}

	/* EC[0x71] == max(EC[0x70], EC[0x72]); a race can break it, so only warn. */
	if (t71 != max(t70, t72))
		pr_warn("EC[0x71] = %u but max(EC[0x70], EC[0x72]) = %u -- layout may differ\n",
			t71, max(t70, t72));

	pr_info("layout check passed: modes %u/%u, temps %u/%u/%u C, fans %u/%u RPM\n",
		m[0], m[1], t70, t71, t72, rpm[0], rpm[1]);
	return 0;
}

/* ----------------------------------------------------------------- platform */

static struct platform_device *zc_pdev;

static int zc_probe(struct platform_device *pdev)
{
	struct zc_data *d;
	struct device *hwmon;

	d = devm_kzalloc(&pdev->dev, sizeof(*d), GFP_KERNEL);
	if (!d)
		return -ENOMEM;

	mutex_init(&d->lock);
	platform_set_drvdata(pdev, d);

	hwmon = devm_hwmon_device_register_with_info(&pdev->dev, "zimacube_ec",
						    d, &zc_chip_info,
						    zc_curve_groups);
	if (IS_ERR(hwmon))
		return PTR_ERR(hwmon);

	if (!ec_r(chan[0].mode, &d->mode_at_probe[0]) &&
	    !ec_r(chan[1].mode, &d->mode_at_probe[1]))
		d->mode_saved = true;

	zc_debugfs = debugfs_create_dir(DRVNAME, NULL);
	debugfs_create_file("regs", 0400, zc_debugfs, d, &zc_regs_fops);
	debugfs_create_file("ec_raw", ZC_EC_RAW_MODE, zc_debugfs, d,
			    &zc_ec_raw_fops);

	if (zc_update(d) == 0)
		dev_info(&pdev->dev,
			 "CPU %d.%d C, board %d C | CPU fan %u RPM (%u%%, mode %u) | SYS fan %u RPM (%u%%, mode %u)%s\n",
			 d->temp_cpu10 / 10, d->temp_cpu10 % 10, d->temp_board,
			 d->rpm[0], d->duty[0] * 100 / (fullscale ?: EC_PWM_SCALE), d->mode[0],
			 d->rpm[1], d->duty[1] * 100 / (fullscale ?: EC_PWM_SCALE), d->mode[1],
			 d->fullscale ? "" : " [live duty unavailable]");
	return 0;
}

static struct platform_driver zc_driver = {
	.driver = { .name = DRVNAME },
	.probe	= zc_probe,
};

static int __init zc_init(void)
{
	u16 id = 0;
	int ret;

	if (!force) {
		ret = zc_check_dmi();
		if (ret)
			return ret;
	} else {
		pr_warn("force=1: binding without a DMI match; the EC RAM layout may not apply to this board\n");
	}

	ret = sio_chipid(&id);
	if (ret)
		return ret;
	if (id != IT5570_CHIPID) {
		pr_info("Super I/O chip ID 0x%04x is not IT5570\n", id);
		return -ENODEV;
	}
	pr_info("found ITE IT5570/IT5570E (ID 0x%04x)\n", id);

	ret = zc_sanity_check();
	if (ret) {
		if (!force) {
			pr_err("EC RAM does not look like the expected layout, refusing to bind\n");
			return ret;
		}
		pr_warn("force=1: binding despite a failed layout check\n");
	}

	ret = platform_driver_register(&zc_driver);
	if (ret)
		return ret;

	zc_pdev = platform_device_register_simple(DRVNAME, -1, NULL, 0);
	if (IS_ERR(zc_pdev)) {
		platform_driver_unregister(&zc_driver);
		return PTR_ERR(zc_pdev);
	}
	return 0;
}

static void __exit zc_exit(void)
{
	struct zc_data *d = platform_get_drvdata(zc_pdev);
	int i;

	/*
	 * Put the fans back the way we found them rather than forcing Automatic:
	 * the user may deliberately have selected Manual or Full Speed in BIOS.
	 */
	for (i = 0; i < 2; i++)
		ec_w(chan[i].mode,
		     (d && d->mode_saved) ? d->mode_at_probe[i] : FAN_MODE_AUTO);

	debugfs_remove_recursive(zc_debugfs);
	platform_device_unregister(zc_pdev);
	platform_driver_unregister(&zc_driver);
}

module_init(zc_init);
module_exit(zc_exit);

MODULE_DESCRIPTION("ZimaCube Pro 2 ITE IT5570E dual-fan hwmon driver");
MODULE_LICENSE("GPL v2");
