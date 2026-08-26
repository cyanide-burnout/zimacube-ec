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

#define pr_fmt(fmt)  KBUILD_MODNAME ": " fmt

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

#define DRVNAME  "zimacube_ec_fan"

/* ---- Super I/O (chip detection + indirect register window) ---------------- */
#define SIO_ADDR       0x4e
#define SIO_DATA       0x4f
#define IT5570_CHIPID  0x5570

/* ITE indirect memory access: SIO cfg reg 0x2e selects a sub-register, 0x2f is data */
#define SIO_IMA_SELECT   0x2e
#define SIO_IMA_DATA     0x2f
#define SIO_IMA_ADDR_LO  0x10
#define SIO_IMA_ADDR_HI  0x11
#define SIO_IMA_XFER     0x12

/* ---- ACPI EC register window, ports 0x62/0x66 ---------------------------- */
#define EC_SYS_FAN_MODE    0x22  /* 0=off 1=manual 2=auto 3=full */
#define EC_CPU_FAN_MODE    0x23
#define EC_SYS_SLOPE       0x24  /* duty units per degC */
#define EC_SYS_START_PWM   0x25  /* 0..122 */
#define EC_SYS_START_TEMP  0x26  /* degC */
#define EC_SYS_FULL_TEMP   0x27  /* degC */
#define EC_CPU_SLOPE       0x28
#define EC_CPU_START_PWM   0x29
#define EC_CPU_START_TEMP  0x2a
#define EC_CPU_FULL_TEMP   0x2b
#define EC_SYS_MANUAL_PWM  0x2c  /* 0..122 */
#define EC_CPU_MANUAL_PWM  0x2d
#define EC_TEMP_CTRL       0x70  /* CPU package temp, drives curves */
#define EC_TEMP_MAX        0x71  /* max(0x70, 0x72) */
#define EC_TEMP_BOARD      0x72  /* board/aux thermistor */
#define EC_CPU_RPM_HI      0x76  /* big-endian 16-bit, already RPM */
#define EC_CPU_RPM_LO      0x77
#define EC_SYS_RPM_HI      0x78
#define EC_SYS_RPM_LO      0x79
#define EC_CPU_TEMP10_LO   0xa1  /* little-endian 16-bit, degC * 10 */
#define EC_CPU_TEMP10_HI   0xa2

/*
 * Live PWM duty, reachable only through the Super I/O indirect window: the
 * controller does not mirror it into the ACPI EC space.
 */
#define REG_PWM_SYS_DUTY  0x1803
#define REG_PWM_CPU_DUTY  0x1809

/* EC firmware fan modes */
#define FAN_MODE_OFF     0
#define FAN_MODE_MANUAL  1
#define FAN_MODE_AUTO    2
#define FAN_MODE_FULL    3
/* mode 4 exists in BIOS Setup but the controller does not decode it — never write it */

/*
 * The BIOS Setup page caps every duty field at 122, but the EC applies the value
 * to the PWM duty register unchanged (measured: EC 0x2D = 61 -> duty 61,
 * 122 -> duty 122), and mode 3 drives the register to 255. So the real
 * scale is a plain 8-bit 0..255 and 122 is a UI limit only -- which means Linux
 * can command duties, start PWMs and slopes that BIOS Setup cannot express.
 */
#define EC_PWM_SCALE     255  /* duty register full scale */
#define EC_BIOS_PWM_CAP  122  /* what BIOS Setup allows */
#define HWMON_PWM_MAX    255

#define CACHE_TTL  (HZ / 2)

struct chan_regs
{
  u8 mode, slope, start_pwm, start_temp, full_temp, manual_pwm;
  u8 rpm_hi, rpm_lo;
  u16 duty_reg;
  const char *label;
};

static const struct chan_regs channels[2] =
{
  [0] =
  {  /* CPU fan */
    .mode       = EC_CPU_FAN_MODE,
    .slope      = EC_CPU_SLOPE,
    .start_pwm  = EC_CPU_START_PWM,
    .start_temp = EC_CPU_START_TEMP,
    .full_temp  = EC_CPU_FULL_TEMP,
    .manual_pwm = EC_CPU_MANUAL_PWM,
    .rpm_hi     = EC_CPU_RPM_HI,
    .rpm_lo     = EC_CPU_RPM_LO,
    .duty_reg   = REG_PWM_CPU_DUTY,
    .label      = "CPU Fan",
  },
  [1] =
  {  /* System fan */
    .mode       = EC_SYS_FAN_MODE,
    .slope      = EC_SYS_SLOPE,
    .start_pwm  = EC_SYS_START_PWM,
    .start_temp = EC_SYS_START_TEMP,
    .full_temp  = EC_SYS_FULL_TEMP,
    .manual_pwm = EC_SYS_MANUAL_PWM,
    .rpm_hi     = EC_SYS_RPM_HI,
    .rpm_lo     = EC_SYS_RPM_LO,
    .duty_reg   = REG_PWM_SYS_DUTY,
    .label      = "System Fan",
  },
};

struct zc_data
{
  struct mutex lock;     /* serialises EC + SIO access */
  u8  mode_at_probe[2];  /* restored on unload */
  bool mode_saved;
  unsigned long updated;
  bool valid;

  u16 speeds[2];
  u8  mode[2];
  u8  duty[2];     /* 0..EC_PWM_SCALE, derived from hardware */
  u8  full_scale;  /* duty value meaning 100 %, 0 if unknown */
  u8  temp_ctrl, temp_max, temp_board;
  u16 temp_cpu10;
};

static bool force;
module_param(force, bool, 0444);
MODULE_PARM_DESC(force, "Bind even if the DMI data does not match a known-good board. The EC RAM layout this driver writes is board-specific: a different IT5570 machine will have a different map, so forcing can drive the fans to an arbitrary duty. Only set this if you have verified the layout yourself.");

static bool sio_ima_ok = true;
module_param_named(sio_ima, sio_ima_ok, bool, 0444);
MODULE_PARM_DESC(sio_ima, "Use the Super I/O indirect window to read live PWM duty (default on)");

static unsigned int fullscale = EC_PWM_SCALE;
module_param(fullscale, uint, 0644);
MODULE_PARM_DESC(fullscale, "PWM duty register value that means 100% (default 255, measured via Full Speed mode). Set to 122 to stay inside the range BIOS Setup uses. Values of 0 or above 255 are ignored: the duty registers are 8-bit, so a larger value would wrap on write and silently under-drive the fan.");

/*
 * fullscale is writable at runtime and every use of it ends up in an 8-bit EC
 * duty register. A value above 255 would wrap there -- 300 turns a requested
 * 100% into 300 & 0xff = 44, i.e. 17% -- so clamp rather than trust it.
 *
 * Sample it exactly once: a concurrent sysfs write between a check and a
 * separate return would hand the caller the very value the check rejected,
 * and the compiler is free to reload a plain global anyway.
 */
static inline unsigned int pwm_full_scale(void)
{
  unsigned int value = READ_ONCE(fullscale);

  if ((!value) || (value > EC_PWM_SCALE))
    return EC_PWM_SCALE;
  return value;
}

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

static void sio_outb(u8 reg, u8 value)
{
  outb(reg, SIO_ADDR);
  outb(value, SIO_DATA);
}

/*
 * Read one byte through the ITE indirect memory-access window. Must be called
 * with sio_enter() already done and the 0x4e/0x4f region held.
 */
static u8 sio_ind_read(u16 address)
{
  sio_outb(SIO_IMA_SELECT, SIO_IMA_ADDR_HI);
  sio_outb(SIO_IMA_DATA, address >> 8);
  sio_outb(SIO_IMA_SELECT, SIO_IMA_ADDR_LO);
  sio_outb(SIO_IMA_DATA, address & 0xff);
  sio_outb(SIO_IMA_SELECT, SIO_IMA_XFER);
  /* select the data sub-register, then read it (outb takes value, port) */
  outb(SIO_IMA_DATA, SIO_ADDR);
  return inb(SIO_DATA);
}

/* Read a batch of indirect-window addresses in one Super I/O session. */
static int sio_ind_read_batch(const u16 *addresses, u8 *out, int count)
{
  int index;

  if (!sio_ima_ok)
    return -ENODEV;

  if (!request_muxed_region(SIO_ADDR, 2, DRVNAME))
    return -EBUSY;

  sio_enter();
  for (index = 0; index < count; index++)
    out[index] = sio_ind_read(addresses[index]);
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
static int read_live_duty(struct zc_data *state)
{
  static const u16 addresses[2] = { REG_PWM_CPU_DUTY, REG_PWM_SYS_DUTY };
  unsigned int full_scale       = pwm_full_scale();
  int result, index;
  u8 duties[2];

  result = sio_ind_read_batch(addresses, duties, 2);
  if (result)
    return result;

  state->full_scale = full_scale;
  for (index = 0; index < 2; index++)
    state->duty[index] = min_t(unsigned int, duties[index], full_scale);
  return 0;
}

static int sio_chipid(u16 *chip_id)
{
  if (!request_muxed_region(SIO_ADDR, 2, DRVNAME))
    return -EBUSY;

  sio_enter();
  *chip_id = (sio_inb(0x20) << 8) | sio_inb(0x21);
  sio_leave();

  release_region(SIO_ADDR, 2);
  return 0;
}

/* ----------------------------------------------------------------- EC access */

static int read_ec_reg(u8 offset, u8 *value)
{
  return ec_read(offset, value);
}

static int write_ec_reg(u8 offset, u8 value)
{
  return ec_write(offset, value);
}

static int read_ec_reg16(u8 high_offset, u16 *value)
{
  u8 high, low;
  int result;

  result = read_ec_reg(high_offset, &high);
  if (result)
    return result;
  result = read_ec_reg(high_offset + 1, &low);
  if (result)
    return result;
  *value = (high << 8) | low;
  return 0;
}

static int zc_update(struct zc_data *state)
{
  int index, result = 0;
  u8 low, high;

  mutex_lock(&state->lock);
  if ((state->valid) && time_before(jiffies, state->updated + CACHE_TTL))
    goto out;

  for (index = 0; index < 2; index++)
  {
    result = read_ec_reg16(channels[index].rpm_hi, &state->speeds[index]);
    if (result)
      goto out;
    result = read_ec_reg(channels[index].mode, &state->mode[index]);
    if (result)
      goto out;
  }

  result = read_ec_reg(EC_TEMP_CTRL, &state->temp_ctrl);
  if (result)
    goto out;
  result = read_ec_reg(EC_TEMP_MAX, &state->temp_max);
  if (result)
    goto out;
  result = read_ec_reg(EC_TEMP_BOARD, &state->temp_board);
  if (result)
    goto out;

  /* CPU temperature in tenths of a degree — note: little-endian here, unlike RPM */
  result = read_ec_reg(EC_CPU_TEMP10_LO, &low);
  if (result)
    goto out;
  result = read_ec_reg(EC_CPU_TEMP10_HI, &high);
  if (result)
    goto out;
  state->temp_cpu10 = (high << 8) | low;

  if (read_live_duty(state))
  {
    /*
     * No indirect window: fall back to what we told the EC to do.
     * In auto mode we cannot know the duty, so report the configured
     * manual value only when the channel is actually in manual mode.
     */
    state->full_scale = 0;
    for (index = 0; index < 2; index++)
    {
      u8 value = 0;

      if (state->mode[index] == FAN_MODE_MANUAL)
        read_ec_reg(channels[index].manual_pwm, &value);
      else if (state->mode[index] == FAN_MODE_FULL)
        value = pwm_full_scale();
      state->duty[index] = value;
    }
  }

  state->updated = jiffies;
  state->valid = true;
out:
  mutex_unlock(&state->lock);
  return result;
}

static inline long ec_to_hwmon_pwm(u8 duty)
{
  unsigned int full_scale = pwm_full_scale();

  return DIV_ROUND_CLOSEST(min_t(unsigned int, duty, full_scale) * HWMON_PWM_MAX, full_scale);
}

static inline u8 hwmon_to_ec_pwm(long pwm)
{
  unsigned int full_scale = pwm_full_scale();

  pwm = clamp_val(pwm, 0, HWMON_PWM_MAX);
  return DIV_ROUND_CLOSEST(pwm * full_scale, HWMON_PWM_MAX);
}

/* -------------------------------------------------------------------- hwmon */

static umode_t zc_is_visible(const void *drvdata, enum hwmon_sensor_types type,
                             u32 attr, int channel)
{
  switch (type)
  {
    case hwmon_fan:
      return 0444;
    case hwmon_temp:
      return 0444;
    case hwmon_pwm:
      switch (attr)
      {
        case hwmon_pwm_input:
        case hwmon_pwm_enable:
          return 0644;
        /*
         * Both curves are fed by one EC register (0x70), and nothing in the
         * EC lets a channel be pointed at another sensor. Reporting the file
         * as writable would promise an operation that cannot be honoured.
         */
        case hwmon_pwm_auto_channels_temp:
          return 0444;
        default:
          return 0;
      }
    default:
      return 0;
  }
}

static int zc_read(struct device *dev, enum hwmon_sensor_types type, u32 attr,
                   int channel, long *value)
{
  struct zc_data *state = dev_get_drvdata(dev);
  int result = zc_update(state);

  if (result)
    return result;

  switch (type)
  {
    case hwmon_fan:
      *value = state->speeds[channel];
      return 0;

    case hwmon_pwm:
      switch (attr)
      {
        case hwmon_pwm_input:
          *value = ec_to_hwmon_pwm(state->duty[channel]);
          return 0;
        case hwmon_pwm_enable:
          switch (state->mode[channel])
          {
            case FAN_MODE_MANUAL:  *value = 1; break;
            case FAN_MODE_AUTO:    *value = 2; break;
            case FAN_MODE_FULL:    *value = 0; break;
            case FAN_MODE_OFF:     *value = 1; break;
            default:
              /*
               * Mode 4 is not decoded by
               * the controller, so the
               * channel is uncontrolled.
               */
              *value = 0;
              break;
          }
          return 0;
        case hwmon_pwm_auto_channels_temp:
          /* both curves are driven by temp1 */
          *value = 1;
          return 0;
        default:
          return -EOPNOTSUPP;
      }

    case hwmon_temp:
      switch (channel)
      {
        case 0:
          /* tenths of a degree, hwmon wants millidegrees */
          *value = state->temp_cpu10 * 100;
          return 0;

        case 1:
          *value = state->temp_board * 1000;
          return 0;

        case 2:
          *value = state->temp_max * 1000;
          return 0;
        default:  return -EOPNOTSUPP;
      }

    default:
      return -EOPNOTSUPP;
  }
}

static int zc_write(struct device *dev, enum hwmon_sensor_types type, u32 attr,
                    int channel, long value)
{
  const struct chan_regs *regs = &channels[channel];
  struct zc_data *state = dev_get_drvdata(dev);
  int result;

  if (type != hwmon_pwm)
    return -EOPNOTSUPP;

  mutex_lock(&state->lock);

  switch (attr)
  {
    case hwmon_pwm_input:
    {
      u8 duty = hwmon_to_ec_pwm(value);

      result = write_ec_reg(regs->manual_pwm, duty);
      if (!result)
        result = write_ec_reg(regs->mode, FAN_MODE_MANUAL);
      break;
    }

    case hwmon_pwm_enable:
      switch (value)
      {
        /* no control at all: run flat out */
        case 0:
          result = write_ec_reg(regs->mode, FAN_MODE_FULL);
          break;

        /* manual: freeze at the duty the fan runs at now */
        case 1:
        {
          unsigned int duty  = 0;
          u8 start_pwm       = 0;
          bool have_start    = (!read_ec_reg(regs->start_pwm, &start_pwm));
          bool live_now      = (!read_live_duty(state));
          /*
           * Falling back on the cache needs two things that state->valid does
           * not give on its own.
           *
           * Where the value came from: without the indirect window zc_update
           * fills duty[] from the mode alone and leaves 0 for a channel in
           * Automatic, marking that by clearing full_scale. Treating that 0
           * as a measurement would be claiming knowledge we do not have.
           *
           * How old it is: valid stays set until something explicitly clears
           * it, so on its own it would let an arbitrarily old reading pass as
           * "the duty the fan runs at now". Apply the same freshness window
           * zc_update uses.
           */
          bool cache_usable  = (state->valid) && (state->full_scale) &&
                               time_before(jiffies, state->updated + CACHE_TTL);
          bool have_live     = live_now || cache_usable;

          /*
           * "Freeze where the fan is" needs to know where the fan is. When
           * the indirect window does not answer and the cache is cold, that
           * is simply unknown, and picking a number would command an
           * arbitrary speed behind the user's back. Fail instead.
           *
           * A stopped fan also reads back as 0, which the EC parks a channel
           * at routinely: in Automatic it drops the target to 0 once the
           * control temperature falls below start_temp - 5. Carrying that 0
           * into the manual register latches the fan off for good -- manual
           * never looks at temperature again, and the EC's kick-start only
           * fires on the automatic target, not on this one.
           *
           * So refuse outright when the duty is unknown, and otherwise floor
           * the measured value at the channel's own start PWM -- the duty the
           * EC itself uses to get a stopped fan turning.
           */
          if (!have_live)
          {
            dev_warn(dev, "pwm%d_enable=1 refused: the current duty is unknown\n", channel + 1);
            result = -EIO;
            break;
          }

          /*
           * The floor belongs to a duty we actually measured. Applying it to
           * an unknown one would be the same invention in a different
           * disguise, which is why that case is rejected above rather than
           * quietly promoted to start_pwm.
           */
          duty = state->duty[channel];
          if (have_start && (duty < start_pwm))
            duty = start_pwm;

          if (!duty)
          {
            dev_warn(dev, "pwm%d_enable=1 refused: fan is stopped and no usable start PWM\n", channel + 1);
            result = -EIO;
            break;
          }

          result = write_ec_reg(regs->manual_pwm, duty);
          if (!result)
            result = write_ec_reg(regs->mode, FAN_MODE_MANUAL);
          break;
        }

        /* hand back to the controller's own curve */
        case 2:
          result = write_ec_reg(regs->mode, FAN_MODE_AUTO);
          break;

        default:
          result = -EINVAL;
      }
      break;

    default:
      result = -EOPNOTSUPP;
  }

  if (!result)
    state->valid = false;
  mutex_unlock(&state->lock);
  return result;
}

static int zc_read_string(struct device *dev, enum hwmon_sensor_types type,
                          u32 attr, int channel, const char **str)
{
  static const char * const temp_labels[] = { "CPU", "Board", "Max" };

  bool labelled = (channel >= 0) && (channel < (int)ARRAY_SIZE(temp_labels));

  if ((type == hwmon_fan) && (attr == hwmon_fan_label))
  {
    *str = channels[channel].label;
    return 0;
  }

  if ((type == hwmon_temp) && (attr == hwmon_temp_label) && labelled)
  {
    *str = temp_labels[channel];
    return 0;
  }

  return -EOPNOTSUPP;
}

static const struct hwmon_channel_info * const zc_info[] =
{
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

static const struct hwmon_ops zc_ops =
{
  .is_visible  = zc_is_visible,
  .read        = zc_read,
  .read_string = zc_read_string,
  .write       = zc_write,
};

static const struct hwmon_chip_info zc_chip_info =
{
  .ops  = &zc_ops,
  .info = zc_info,
};

/* ------------------------------------------- curve attributes (custom group) */

static struct zc_data *attr_data(struct device *dev)
{
  return dev_get_drvdata(dev);
}

static ssize_t curve_show(struct device *dev, struct device_attribute *dev_attr,
                          char *buf)
{
  struct sensor_device_attribute_2 *attr = to_sensor_dev_attr_2(dev_attr);
  struct zc_data *state                  = attr_data(dev);
  u8 offset, value;
  int result;

  switch (attr->nr)
  {
    case 0:   offset = channels[attr->index].start_temp; break;
    case 1:   offset = channels[attr->index].full_temp;  break;
    case 2:   offset = channels[attr->index].start_pwm;  break;
    case 3:   offset = channels[attr->index].slope;      break;
    default:  return -EINVAL;
  }

  mutex_lock(&state->lock);
  result = read_ec_reg(offset, &value);
  mutex_unlock(&state->lock);
  if (result)
    return result;

  /* temperatures are reported in millidegrees */
  if (attr->nr <= 1)
    return sysfs_emit(buf, "%d\n", value * 1000);

  /* start PWM in hwmon 0..255 units */
  if (attr->nr == 2)
    return sysfs_emit(buf, "%ld\n", ec_to_hwmon_pwm(value));

  /* slope, raw duty units per degC */
  return sysfs_emit(buf, "%u\n", value);
}

static ssize_t curve_store(struct device *dev, struct device_attribute *dev_attr,
                           const char *buf, size_t count)
{
  struct sensor_device_attribute_2 *attr = to_sensor_dev_attr_2(dev_attr);
  struct zc_data *state = attr_data(dev);
  u8 offset, value;
  int result;
  long input;

  result = kstrtol(buf, 10, &input);
  if (result)
    return result;

  switch (attr->nr)
  {
    case 0:
      offset = channels[attr->index].start_temp;
      value  = clamp_val(input / 1000, 0, 127);
      break;
    case 1:
      offset = channels[attr->index].full_temp;
      value  = clamp_val(input / 1000, 0, 127);
      break;
    case 2:
      offset = channels[attr->index].start_pwm;
      value  = hwmon_to_ec_pwm(input);
      break;
    case 3:
      offset = channels[attr->index].slope;
      value  = clamp_val(input, 0, 255);
      break;
    default:
      return -EINVAL;
  }

  mutex_lock(&state->lock);
  result = write_ec_reg(offset, value);
  state->valid = false;
  mutex_unlock(&state->lock);

  return result ? result : count;
}

static SENSOR_DEVICE_ATTR_2_RW(pwm1_auto_point1_temp, curve, 0, 0);
static SENSOR_DEVICE_ATTR_2_RW(pwm1_auto_point2_temp, curve, 1, 0);
static SENSOR_DEVICE_ATTR_2_RW(pwm1_auto_point1_pwm,  curve, 2, 0);
static SENSOR_DEVICE_ATTR_2_RW(pwm1_slope,            curve, 3, 0);
static SENSOR_DEVICE_ATTR_2_RW(pwm2_auto_point1_temp, curve, 0, 1);
static SENSOR_DEVICE_ATTR_2_RW(pwm2_auto_point2_temp, curve, 1, 1);
static SENSOR_DEVICE_ATTR_2_RW(pwm2_auto_point1_pwm,  curve, 2, 1);
static SENSOR_DEVICE_ATTR_2_RW(pwm2_slope,            curve, 3, 1);

static struct attribute *zc_curve_attrs[] =
{
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

static const char *mode_name(u8 mode)
{
  switch (mode)
  {
    case FAN_MODE_OFF:     return "off";
    case FAN_MODE_MANUAL:  return "manual";
    case FAN_MODE_AUTO:    return "automatic";
    case FAN_MODE_FULL:    return "full speed";
    case 4:                return "SILENT (not implemented by EC!)";
    default:               return "unknown";
  }
}

static int zc_regs_show(struct seq_file *output, void *unused)
{
  u8 window[0x100];
  u8 duties[ARRAY_SIZE(dump_duty)];
  bool have_duty;
  int index, channel_index;

  memset(window, 0, sizeof(window));
  for (index = 0x20; index <= 0x37; index++)
    read_ec_reg(index, &window[index]);
  for (index = 0x70; index <= 0x79; index++)
    read_ec_reg(index, &window[index]);
  read_ec_reg(0xa1, &window[0xa1]);
  read_ec_reg(0xa2, &window[0xa2]);

  have_duty = (sio_ind_read_batch(dump_duty, duties, ARRAY_SIZE(dump_duty)) == 0);

  seq_puts(output, "ACPI EC register window\n");
  for (index = 0x20; index <= 0x30; index += 0x10)
  {
    int column;

    seq_printf(output, "  %02x:", index);
    for (column = 0; (column < 16) && ((index + column) <= 0x37); column++)
      seq_printf(output, " %02x", window[index + column]);
    seq_puts(output, "\n");
  }
  seq_puts(output, "  70:");
  for (index = 0x70; index <= 0x79; index++)
    seq_printf(output, " %02x", window[index]);
  seq_puts(output, "\n");

  for (channel_index = 0; channel_index < 2; channel_index++)
  {
    const struct chan_regs *regs = &channels[channel_index];
    u8 mode                      = window[regs->mode];
    int start                    = window[regs->start_pwm];
    int slope                    = window[regs->slope];
    int start_temp               = window[regs->start_temp];
    int full_temp                = window[regs->full_temp];
    int temperature              = window[EC_TEMP_CTRL];
    unsigned int full_scale      = pwm_full_scale();
    int predicted;

    seq_printf(output, "\n%s (channel %d)\n",              regs->label, channel_index);
    seq_printf(output, "  mode        0x%02x = %u (%s)\n", regs->mode, mode, mode_name(mode));
    seq_printf(output, "  manual pwm  0x%02x = %u/%u\n",   regs->manual_pwm, window[regs->manual_pwm], full_scale);
    seq_printf(output, "  start pwm   0x%02x = %u/%u\n",   regs->start_pwm, start, full_scale);
    seq_printf(output, "  slope       0x%02x = %u\n",      regs->slope, slope);
    seq_printf(output, "  start temp  0x%02x = %u C\n",    regs->start_temp, start_temp);
    seq_printf(output, "  full temp   0x%02x = %u C\n",    regs->full_temp, full_temp);
    seq_printf(output, "  rpm         0x%02x = %u\n",      regs->rpm_hi, (window[regs->rpm_hi] << 8) | window[regs->rpm_lo]);

    if (temperature >= full_temp)
      predicted = full_scale;
    else if (temperature < start_temp)
      predicted = -1;
    else
      predicted = start + (temperature - start_temp) * slope;

    if (predicted < 0)
      seq_puts(output, "  auto curve  T below start temp -> holds previous\n");
    else
      seq_printf(output, "  auto curve  predicts %d/%u at T=%d C%s\n",
                 predicted, full_scale, temperature,
                 (predicted > EC_BIOS_PWM_CAP) ?
                 "  (above the BIOS UI cap of 122)" : "");

    if (have_duty)
    {
      u8 duty = duties[channel_index];

      seq_printf(output, "  live duty   %u/%u", duty, full_scale);
      if ((mode == FAN_MODE_AUTO) && (predicted >= 0))
        seq_printf(output, "  (prediction %d, delta %d)",
                   predicted, (int)duty - predicted);
      seq_puts(output, "\n");
    }
  }

  seq_printf(output, "\ntemps: ctrl(0x70)=%u C  max(0x71)=%u C  board(0x72)=%u C  cpu*10(0xA1)=%u\n",
             window[0x70], window[0x71], window[0x72], (window[0xa2] << 8) | window[0xa1]);
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
static int zc_ec_raw_show(struct seq_file *output, void *unused)
{
  int index, column;

  for (index = 0; index < 0x100; index += 16)
  {
    seq_printf(output, "%02x:", index);
    for (column = 0; column < 16; column++)
    {
      u8 value;

      if (read_ec_reg(index + column, &value))
        seq_puts(output, " --");
      else
        seq_printf(output, " %02x", value);
    }
    seq_puts(output, "\n");
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
  struct zc_data *state = file_inode(file)->i_private;
  char buf[32];
  unsigned int offset, value;
  int result;

  if (len >= sizeof(buf))
    return -EINVAL;
  if (copy_from_user(buf, ubuf, len))
    return -EFAULT;
  buf[len] = '\0';

  if (sscanf(buf, "%x %x", &offset, &value) != 2)
    return -EINVAL;
  if ((offset > 0xff) || (value > 0xff))
    return -EINVAL;

  pr_warn("debugfs write: EC RAM 0x%02x = 0x%02x\n", offset, value);

  mutex_lock(&state->lock);
  result = write_ec_reg(offset, value);
  state->valid = false;
  mutex_unlock(&state->lock);

  return result ? result : len;
}
#define ZC_EC_RAW_MODE  0600
#else
#define zc_ec_raw_write  NULL
#define ZC_EC_RAW_MODE   0400
#endif

static const struct file_operations zc_ec_raw_fops =
{
  .owner   = THIS_MODULE,
  .open    = zc_ec_raw_open,
  .read    = seq_read,
  .llseek  = seq_lseek,
  .release = single_release,
  .write   = zc_ec_raw_write,
};

/* ---------------------------------------------------------------- DMI gating */

/*
 * The register layout this driver writes to is board-specific, and a matching
 * Super I/O chip ID says nothing about it -- that assumption is exactly why an
 * earlier IT5570 driver read a pair of fan mode bytes as "514 RPM" here. So bind
 * only on boards that have actually been checked.
 *
 * zc_dmi_verified: layouts confirmed on the exact board.
 * zc_dmi_family:   same family, unverified -- bind, but warn.
 */
static const struct dmi_system_id zc_dmi_verified[] =
{
  {
    .ident   = "IceWhale ZimaCube Pro",
    .matches =
    {
      DMI_MATCH(DMI_SYS_VENDOR, "IceWhale"),
      DMI_MATCH(DMI_BOARD_NAME, "ZimaCube Pro"),
    },
  },
  { }
};

static const struct dmi_system_id zc_dmi_family[] =
{
  {
    .ident   = "IceWhale ZimaCube (unverified variant)",
    .matches =
    {
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
  const struct dmi_system_id *match;

  match = dmi_first_match(zc_dmi_verified);
  if (match)
  {
    pr_info("matched verified board: %s\n", match->ident);
    return 0;
  }

  match = dmi_first_match(zc_dmi_family);
  if (match)
  {
    pr_warn("%s: the EC RAM layout was verified on ZimaCube Pro only.\n", match->ident);
    pr_warn("  Check /sys/kernel/debug/%s/regs before trusting pwm writes.\n", DRVNAME);
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
  u8 modes[2], start_temps[2], full_temps[2];
  u8 control_temp, max_temp, board_temp;
  u16 speeds[2];
  int index, result;

  for (index = 0; index < 2; index++)
  {
    result = read_ec_reg(channels[index].mode, &modes[index]);
    if (result)
      return result;
    result = read_ec_reg(channels[index].start_temp, &start_temps[index]);
    if (result)
      return result;
    result = read_ec_reg(channels[index].full_temp, &full_temps[index]);
    if (result)
      return result;
    result = read_ec_reg16(channels[index].rpm_hi, &speeds[index]);
    if (result)
      return result;
  }
  result = read_ec_reg(EC_TEMP_CTRL, &control_temp);
  if (!result)
    result = read_ec_reg(EC_TEMP_MAX, &max_temp);
  if (!result)
    result = read_ec_reg(EC_TEMP_BOARD, &board_temp);
  if (result)
    return result;

  for (index = 0; index < 2; index++)
  {
    if (modes[index] > 4)
    {
      pr_err("EC[0x%02x] = %u is not a valid fan mode\n",
             channels[index].mode, modes[index]);
      return -ENODEV;
    }
    if ((start_temps[index] > 127) || (full_temps[index] > 127) ||
        (start_temps[index] > full_temps[index]))
    {
      pr_err("EC[0x%02x]/[0x%02x] = %u/%u are not a sane temperature pair\n",
             channels[index].start_temp, channels[index].full_temp,
             start_temps[index], full_temps[index]);
      return -ENODEV;
    }
    if (speeds[index] > 20000)
    {
      pr_err("EC[0x%02x] = %u is not a plausible fan speed\n",
             channels[index].rpm_hi, speeds[index]);
      return -ENODEV;
    }
  }
  if ((control_temp > 110) || (board_temp > 110))
  {
    pr_err("EC[0x70]/[0x72] = %u/%u are not plausible temperatures\n",
           control_temp, board_temp);
    return -ENODEV;
  }

  /* EC[0x71] == max(EC[0x70], EC[0x72]); a race can break it, so only warn. */
  if (max_temp != max(control_temp, board_temp))
    pr_warn("EC[0x71] = %u but max(EC[0x70], EC[0x72]) = %u -- layout may differ\n",
            max_temp, max(control_temp, board_temp));

  pr_info("layout check passed: modes %u/%u, temps %u/%u/%u C, fans %u/%u RPM\n",
          modes[0], modes[1], control_temp, max_temp, board_temp, speeds[0], speeds[1]);
  return 0;
}

/* ----------------------------------------------------------------- platform */

static struct platform_device *zc_pdev;

static int zc_probe(struct platform_device *pdev)
{
  struct zc_data *state;
  struct device *hwmon;
  int failed;

  state = devm_kzalloc(&pdev->dev, sizeof(*state), GFP_KERNEL);
  if (!state)
    return -ENOMEM;

  mutex_init(&state->lock);
  platform_set_drvdata(pdev, state);

  hwmon = devm_hwmon_device_register_with_info(&pdev->dev, "zimacube_ec",
                                               state, &zc_chip_info,
                                               zc_curve_groups);
  if (IS_ERR(hwmon))
    return PTR_ERR(hwmon);

  failed  = read_ec_reg(channels[0].mode, &state->mode_at_probe[0]);
  failed |= read_ec_reg(channels[1].mode, &state->mode_at_probe[1]);
  state->mode_saved = (failed == 0);

  zc_debugfs = debugfs_create_dir(DRVNAME, NULL);
  debugfs_create_file("regs", 0400, zc_debugfs, state, &zc_regs_fops);
  debugfs_create_file("ec_raw", ZC_EC_RAW_MODE, zc_debugfs, state,
                      &zc_ec_raw_fops);

  if (zc_update(state) == 0)
    dev_info(&pdev->dev,
             "CPU %d.%d C, board %d C | CPU fan %u RPM (%u%%, mode %u) | SYS fan %u RPM (%u%%, mode %u)%s\n",
             state->temp_cpu10 / 10, state->temp_cpu10 % 10, state->temp_board,
             state->speeds[0], state->duty[0] * 100 / EC_PWM_SCALE, state->mode[0],
             state->speeds[1], state->duty[1] * 100 / EC_PWM_SCALE, state->mode[1],
             state->full_scale ? "" : " [live duty unavailable]");
  return 0;
}

static struct platform_driver zc_driver =
{
  .driver = { .name = DRVNAME },
  .probe  = zc_probe,
};

static int __init zc_init(void)
{
  u16 chip_id = 0;
  int result;

  if (!force)
  {
    result = zc_check_dmi();
    if (result)
      return result;
  }
  else
  {
    pr_warn("force=1: binding without a DMI match; the EC RAM layout may not apply to this board\n");
  }

  result = sio_chipid(&chip_id);
  if (result)
    return result;
  if (chip_id != IT5570_CHIPID)
  {
    pr_info("Super I/O chip ID 0x%04x is not IT5570\n", chip_id);
    return -ENODEV;
  }
  pr_info("found ITE IT5570/IT5570E (ID 0x%04x)\n", chip_id);

  result = zc_sanity_check();
  if (result)
  {
    if (!force)
    {
      pr_err("EC RAM does not look like the expected layout, refusing to bind\n");
      return result;
    }
    pr_warn("force=1: binding despite a failed layout check\n");
  }

  result = platform_driver_register(&zc_driver);
  if (result)
    return result;

  zc_pdev = platform_device_register_simple(DRVNAME, -1, NULL, 0);
  if (IS_ERR(zc_pdev))
  {
    platform_driver_unregister(&zc_driver);
    return PTR_ERR(zc_pdev);
  }
  return 0;
}

static void __exit zc_exit(void)
{
  struct zc_data *state = platform_get_drvdata(zc_pdev);
  int index;

  /*
   * Put the fans back the way we found them rather than forcing Automatic:
   * the user may deliberately have selected Manual or Full Speed in BIOS.
   */
  for (index = 0; index < 2; index++)
    write_ec_reg(channels[index].mode,
                 (state && state->mode_saved) ?
                 state->mode_at_probe[index] : FAN_MODE_AUTO);

  debugfs_remove_recursive(zc_debugfs);
  platform_device_unregister(zc_pdev);
  platform_driver_unregister(&zc_driver);
}

module_init(zc_init);
module_exit(zc_exit);

MODULE_DESCRIPTION("ZimaCube Pro 2 ITE IT5570E dual-fan hwmon driver");
MODULE_LICENSE("GPL v2");
