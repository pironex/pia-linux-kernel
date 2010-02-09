/*
 * drivers/media/video/mt9t111.c
 *
 * mt9t111 sensor driver
 *
 * Copyright (C) 2009 Leopard Imaging
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/i2c.h>
#include <linux/delay.h>
#include <media/v4l2-int-device.h>

#include <media/mt9t111.h>
#include "mt9t111_reg.h"

#define USE_RAW // YCbCr mode does not work yet
//#define COLOR_BAR // Create a Color bar test pattern, Blue, Green, Red, Grey

#define SENSOR_DETECTED		1
#define SENSOR_NOT_DETECTED	0

static void mt9t111_loaddefault(struct i2c_client *client);

/* 
* as a place holder for further development
*/
static void debug_dummy(char *in_msg)
{
	
}

/* list of image formats supported by mt9t111 sensor */
const static struct v4l2_fmtdesc mt9t111_formats[] = {
#ifdef USE_RAW
	{
		.description    = "RAW ",
		.pixelformat    = V4L2_PIX_FMT_SGRBG10,
	},
#else
	{
		.description    = "YUV 422 ",
		.pixelformat    = V4L2_PIX_FMT_YUYV,
	},
#endif
};

#define NUM_CAPTURE_FORMATS ARRAY_SIZE(mt9t111_formats)

/*
 * Array of image sizes supported by MT9T111.  These must be ordered from
 * smallest image size to largest.
 */
const static struct capture_size mt9t111_sizes[] = {
	{  640, 480 },	
//	{ 2048, 1536}		
};

#define NUM_CAPTURE_SIZE ARRAY_SIZE(mt9t111_sizes)


const struct v4l2_fract mt9t111_frameintervals[] = {
	{  .numerator = 1, .denominator = 10 }
};

#define NUM_CAPTURE_FRAMEINTERVALS ARRAY_SIZE(mt9t111_frameintervals)

/**
 * struct mt9t111_sensor - main structure for storage of sensor information
 * @pdata: access functions and data for platform level information
 * @v4l2_int_device: V4L2 device structure structure
 * @i2c_client: iic client device structure
 * @pix: V4L2 pixel format information structure
 * @timeperframe: time per frame expressed as V4L fraction
 * @scaler:
 * @ver: mt9t111 chip version
 * @fps: frames per second value
 */
struct mt9t111_sensor {
	const struct mt9t111_platform_data *pdata;
	struct v4l2_int_device *v4l2_int_device;
	struct i2c_client *i2c_client;
	struct v4l2_pix_format pix;
	struct v4l2_fract timeperframe;
	int scaler;
	int ver;
	int fps;
	int state;
};

static struct mt9t111_sensor mt9t111 = {
	.timeperframe = {
		.numerator = 1,
		.denominator = 10,
	},
	.state = SENSOR_NOT_DETECTED,
};

/**
 * mt9t111_read_reg - Read a value from a register in an mt9t111 sensor device
 * @client: i2c driver client structure
 * @data_length: length of data to be read
 * @reg: register address / offset
 * @val: stores the value that gets read
 *
 * Read a value from a register in an mt9t111 sensor device.
 * The value is returned in 'val'.
 * Returns zero if successful, or non-zero otherwise.
 */
static int
mt9t111_read_reg(struct i2c_client *client, u16 reg, u16 *val)
{
	struct i2c_msg msg[1];
	u8 data[4];
	int err;

	msg->addr = client->addr;
	msg->flags = 0;
	msg->len = 2;
	msg->buf = data;	
	data[0] = (reg & 0xff00) >> 8;
	data[1] = (reg & 0x00ff);
	err = i2c_transfer(client->adapter, msg, 1);
	if (err >= 0) {			
		msg->flags = I2C_M_RD;			
		msg->len = 2;	/* 2 byte read */			
		err = i2c_transfer(client->adapter, msg, 1);			
		if (err >= 0) {				
			*val = ((data[0] & 0x00ff) << 8)    
				| (data[1] & 0x00ff);	
			return 0;
		}
	}
	return err;
}

/**
 * mt9t111_write_reg - Write a value to a register in an mt9t111 sensor device
 * @client: i2c driver client structure
 * @data_length: length of data to be read
 * @reg: register address / offset
 * @val: value to be written to specified register
 *
 * Write a value to a register in an mt9t111 sensor device.
 * Returns zero if successful, or non-zero otherwise.
 */
static int
mt9t111_write_reg(struct i2c_client *client, u16 reg, u16 val)
{
	struct i2c_msg msg[1];
	u8 data[20];
	int err;

	msg->addr = client->addr;
	msg->flags = 0;
	msg->len = 4;
	msg->buf = data;
	data[0] = (u8)((reg & 0xff00) >> 8);
	data[1] = (u8)(reg & 0x00ff);	
	data[2] = (u8)((val & 0xff00) >> 8);
	data[3] = (u8)(val & 0x00ff);
	err = i2c_transfer(client->adapter, msg, 1);

	return err;
}

/**
 * mt9t111_write_regs - Write registers to an mt9t111 sensor device
 * @client: i2c driver client structure
 * @reg_in: pointer to registers to write
 * @cnt: the number of registers 
 *
 * Write registers .
 * Returns zero if successful, or non-zero otherwise.
 */
static int
mt9t111_write_regs(struct i2c_client *client, mt9t111_regs *reg_in, int cnt)
{
	int err = 0;
	int i;
	mt9t111_regs *reg = reg_in;
	
	for (i=0;i<cnt;i++) {
		if (reg->delay_time == 0) {
			err |= mt9t111_write_reg(client, reg->addr, reg->data);
		} else if (reg->addr != 0 || reg->data != 0) {
			err |= mt9t111_write_reg(client, reg->addr, reg->data);
			mdelay(reg->delay_time);
		} else 
			mdelay(reg->delay_time);
			
		if (err < 0) {
			dev_warn(&client->dev, "write reg error, addr = 0x%x, data = 0x%x \n", \
				reg->addr, reg->data);
			return err;
		}
		reg++;
	}
	return err;
}

/**
 * mt9t111_detect - Detect if an mt9t111 is present, and if so which revision
 * @client: pointer to the i2c client driver structure
 *
 * Detect if an mt9t111 is present
 * Returns a negative error number if no device is detected, or the
 * non-negative value of the version ID register if a device is detected.
 */
static int
mt9t111_detect(struct i2c_client *client)
{
	u16 val;

	/* chip ID is at address 0 */
	if (mt9t111_read_reg(client, MT9T111_CHIP_ID, &val) < 0)
		return -ENODEV;
	dev_info(&client->dev, "model id detected 0x%x\n", val);
	
	if (val != MT9T111_CHIP_ID_VALUE) {
		dev_warn(&client->dev, "model id mismatch received 0x%x expecting 0x%x\n",
			val, MT9T111_CHIP_ID_VALUE);

		return -ENODEV;
	}

	return 0;

}

/**
 * mt9t111_configure - Configure the mt9t111 for the specified image mode
 * @s: pointer to standard V4L2 device structure
 *
 * Configure the mt9t111 for a specified image size, pixel format, and frame
 * period.  xclk is the frequency (in Hz) of the xclk input to the mt9t111.
 * fper is the frame period (in seconds) expressed as a fraction.
 * Returns zero if successful, or non-zero otherwise.
 * The actual frame period is returned in fper.
 */
static int mt9t111_configure(struct v4l2_int_device *s)
{
	debug_dummy("debug_dummy -- to set imager mode");

	return 0;
}

/**
 * ioctl_enum_framesizes - V4L2 sensor if handler for vidioc_int_enum_framesizes
 * @s: pointer to standard V4L2 device structure
 * @frms: pointer to standard V4L2 framesizes enumeration structure
 *
 * Returns possible framesizes depending on choosen pixel format
 **/
static int ioctl_enum_framesizes(struct v4l2_int_device *s,
					struct v4l2_frmsizeenum *frms)
{
	int ifmt;

	for (ifmt = 0; ifmt < NUM_CAPTURE_FORMATS; ifmt++) {
		if (frms->pixel_format == mt9t111_formats[ifmt].pixelformat)
			break;
	}
	/* Is requested pixelformat not found on sensor? */
	if (ifmt == NUM_CAPTURE_FORMATS)
		return -EINVAL;

	/* Do we already reached all discrete framesizes? */
	if (frms->index >= NUM_CAPTURE_SIZE)
		return -EINVAL;

	frms->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	frms->discrete.width = mt9t111_sizes[frms->index].width;
	frms->discrete.height = mt9t111_sizes[frms->index].height;

	return 0;

}

static int ioctl_enum_frameintervals(struct v4l2_int_device *s,
					struct v4l2_frmivalenum *frmi)
{
	int ifmt;

printk(KERN_INFO "entering ioctl_enum_frameintervals\n");
printk(KERN_INFO "index = %d, pixel_format = 0x%x, width = %d, height = %d\n",
			frmi->index, frmi->pixel_format, frmi->width, frmi->height);
printk(KERN_INFO "mt9t111 format = 0x%x\n",	mt9t111_formats[0].pixelformat);

	if (frmi->index >= NUM_CAPTURE_FRAMEINTERVALS)
		return -EINVAL;
	
	for (ifmt = 0; ifmt < NUM_CAPTURE_FORMATS; ifmt++) {
		if (frmi->pixel_format == mt9t111_formats[ifmt].pixelformat)
			break;
	}
	/* Is requested pixelformat not found on sensor? */
	if (ifmt == NUM_CAPTURE_FORMATS)
		return -EINVAL;

	frmi->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	frmi->discrete.numerator =
				mt9t111_frameintervals[frmi->index].numerator;
	frmi->discrete.denominator =
				mt9t111_frameintervals[frmi->index].denominator;
	return 0;
}

/**
 * ioctl_init - V4L2 sensor interface handler for VIDIOC_INT_INIT
 * @s: pointer to standard V4L2 device structure
 *
 * Initialize the sensor device (call mt9t111_configure())
 */
static int ioctl_init(struct v4l2_int_device *s)
{
	return 0;
}

/**
 * ioctl_dev_exit - V4L2 sensor interface handler for vidioc_int_dev_exit_num
 * @s: pointer to standard V4L2 device structure
 *
 * Delinitialise the dev. at slave detach.  The complement of ioctl_dev_init.
 */
static int ioctl_dev_exit(struct v4l2_int_device *s)
{
	return 0;
}

/**
 * ioctl_dev_init - V4L2 sensor interface handler for vidioc_int_dev_init_num
 * @s: pointer to standard V4L2 device structure
 *
 * Initialise the device when slave attaches to the master.  Returns 0 if
 * mt9t111 device could be found, otherwise returns appropriate error.
 */
static int ioctl_dev_init(struct v4l2_int_device *s)
{
	return 0;
}

/**
 * ioctl_s_power - V4L2 sensor interface handler for vidioc_int_s_power_num
 * @s: pointer to standard V4L2 device structure
 * @on: power state to which device is to be set
 *
 * Sets devices power state to requrested state, if possible.
 */
static int ioctl_s_power(struct v4l2_int_device *s, enum v4l2_power on)
{
	struct mt9t111_sensor *sensor = s->priv;
	struct i2c_client *c = sensor->i2c_client;
	int rval;

	if ((on == V4L2_POWER_STANDBY) && (sensor->state == SENSOR_DETECTED))
		debug_dummy("debug_dummy -- put to standby\n");

	if (on != V4L2_POWER_ON)
		debug_dummy("debug_dummy -- stop master clock\n");
	else
		debug_dummy("debug_dummy -- enable clock\n");;

	rval = sensor->pdata->power_set(on);
	if (rval < 0) {
		dev_err(&c->dev, "Unable to set the power state: " "mt9t111"
								" sensor\n");
		//sensor->pdata->set_xclk(0);
		return rval;
	}

	if ((on == V4L2_POWER_ON) && (sensor->state == SENSOR_DETECTED))
		mt9t111_configure(s);

	if ((on == V4L2_POWER_ON) && (sensor->state == SENSOR_NOT_DETECTED)) {
		rval = mt9t111_detect(c);
		if (rval < 0) {
			dev_err(&c->dev, "Unable to detect " "mt9t111"
								" sensor\n");
			sensor->state = SENSOR_NOT_DETECTED;
			return rval;
		}
		mt9t111_loaddefault(c);
		sensor->state = SENSOR_DETECTED;
		sensor->ver = rval;
		pr_info("mt9t111" " chip version 0x%02x detected\n",
								sensor->ver);
	}
	return 0;
}

/**
 * ioctl_g_priv - V4L2 sensor interface handler for vidioc_int_g_priv_num
 * @s: pointer to standard V4L2 device structure
 * @p: void pointer to hold sensor's private data address
 *
 * Returns device's (sensor's) private data area address in p parameter
 */
static int ioctl_g_priv(struct v4l2_int_device *s, void *p)
{
	struct mt9t111_sensor *sensor = s->priv;

	return sensor->pdata->priv_data_set(p);
}

/**
 * ioctl_s_parm - V4L2 sensor interface handler for VIDIOC_S_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_S_PARM ioctl structure
 *
 * Configures the sensor to use the input parameters, if possible.  If
 * not possible, reverts to the old parameters and returns the
 * appropriate error code.
 */
static int ioctl_s_parm(struct v4l2_int_device *s,
			     struct v4l2_streamparm *a)
{
	//TODO: set paramters
	debug_dummy("debug_dummy -- VIDIOC_S_PARM ");
	return 0;
}

/**
 * ioctl_g_parm - V4L2 sensor interface handler for VIDIOC_G_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_G_PARM ioctl structure
 *
 * Returns the sensor's video CAPTURE parameters.
 */
static int ioctl_g_parm(struct v4l2_int_device *s,
			     struct v4l2_streamparm *a)
{
	struct mt9t111_sensor *sensor = s->priv;
	struct v4l2_captureparm *cparm = &a->parm.capture;

	if (a->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	memset(a, 0, sizeof(*a));
	a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	cparm->capability = V4L2_CAP_TIMEPERFRAME;
	cparm->timeperframe = sensor->timeperframe;

	return 0;
}

/**
 * ioctl_g_fmt_cap - V4L2 sensor interface handler for ioctl_g_fmt_cap
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 v4l2_format structure
 *
 * Returns the sensor's current pixel format in the v4l2_format
 * parameter.
 */
static int ioctl_g_fmt_cap(struct v4l2_int_device *s,
				struct v4l2_format *f)
{
	struct mt9t111_sensor *sensor = s->priv;
	f->fmt.pix = sensor->pix;

	return 0;
}

/**
 * ioctl_try_fmt_cap - Implement the CAPTURE buffer VIDIOC_TRY_FMT ioctl
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 VIDIOC_TRY_FMT ioctl structure
 *
 * Implement the VIDIOC_TRY_FMT ioctl for the CAPTURE buffer type.  This
 * ioctl is used to negotiate the image capture size and pixel format
 * without actually making it take effect.
 */
static int ioctl_try_fmt_cap(struct v4l2_int_device *s,
			     struct v4l2_format *f)
{
	struct v4l2_pix_format *pix = &f->fmt.pix;
	struct mt9t111_sensor *sensor = s->priv;
	struct v4l2_pix_format *pix2 = &sensor->pix;

	pix->width = 640;
	pix->height = 480;
#ifdef USE_RAW	
	pix->pixelformat = V4L2_PIX_FMT_SGRBG10;
	pix->bytesperline = pix->width; 
	pix->colorspace =  V4L2_COLORSPACE_SRGB;
#else
	pix->pixelformat = V4L2_PIX_FMT_YUYV;
	pix->bytesperline = pix->width * 2;  
	pix->colorspace = V4L2_COLORSPACE_JPEG;
#endif
	pix->field = V4L2_FIELD_NONE;

	pix->sizeimage = pix->bytesperline * pix->height;
	pix->priv = 0;
	*pix2 = *pix;
	return 0;
}

/**
 * ioctl_s_fmt_cap - V4L2 sensor interface handler for VIDIOC_S_FMT ioctl
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 VIDIOC_S_FMT ioctl structure
 *
 * If the requested format is supported, configures the HW to use that
 * format, returns error code if format not supported or HW can't be
 * correctly configured.
 */
static int ioctl_s_fmt_cap(struct v4l2_int_device *s,
				struct v4l2_format *f)
{
	struct mt9t111_sensor *sensor = s->priv;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	int rval;

	rval = ioctl_try_fmt_cap(s, f);
	if (!rval)
		sensor->pix = *pix;

	return rval;
}

/**
 * ioctl_enum_fmt_cap - Implement the CAPTURE buffer VIDIOC_ENUM_FMT ioctl
 * @s: pointer to standard V4L2 device structure
 * @fmt: standard V4L2 VIDIOC_ENUM_FMT ioctl structure
 *
 * Implement the VIDIOC_ENUM_FMT ioctl for the CAPTURE buffer type.
 */
static int ioctl_enum_fmt_cap(struct v4l2_int_device *s,
				   struct v4l2_fmtdesc *fmt)
{
	int index = fmt->index;
	enum v4l2_buf_type type = fmt->type;

	memset(fmt, 0, sizeof(*fmt));
	fmt->index = index;
	fmt->type = type;

	switch (fmt->type) {
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		if (index >= NUM_CAPTURE_FORMATS)
			return -EINVAL;
	break;
	default:
		return -EINVAL;
	}

	fmt->flags = mt9t111_formats[index].flags;
	strlcpy(fmt->description, mt9t111_formats[index].description,
					sizeof(fmt->description));
	fmt->pixelformat = mt9t111_formats[index].pixelformat;

	return 0;
}

/**
 * ioctl_s_ctrl - V4L2 sensor interface handler for VIDIOC_S_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_S_CTRL ioctl structure
 *
 * If the requested control is supported, sets the control's current
 * value in HW (and updates the video_control[] array).  Otherwise,
 * returns -EINVAL if the control is not supported.
 */
static int ioctl_s_ctrl(struct v4l2_int_device *s,
			     struct v4l2_control *vc)
{
	debug_dummy("debug_dummy -- s ctrl\n");
	return 0;
}

/**
 * ioctl_g_ctrl - V4L2 sensor interface handler for VIDIOC_G_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_G_CTRL ioctl structure
 *
 * If the requested control is supported, returns the control's current
 * value from the video_control[] array.  Otherwise, returns -EINVAL
 * if the control is not supported.
 */
static int ioctl_g_ctrl(struct v4l2_int_device *s,
			     struct v4l2_control *vc)
{
	debug_dummy("debug_dummy -- g ctrl\n");	
	return 0;
}

/**
 * ioctl_queryctrl - V4L2 sensor interface handler for VIDIOC_QUERYCTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @qc: standard V4L2 VIDIOC_QUERYCTRL ioctl structure
 *
 * If the requested control is supported, returns the control information
 * from the video_control[] array.  Otherwise, returns -EINVAL if the
 * control is not supported.
 */
static int ioctl_queryctrl(struct v4l2_int_device *s,
				struct v4l2_queryctrl *qc)
{
	debug_dummy("debug_dummy -- query ctrl\n");	
	return-EINVAL;
}

/**
 * ioctl_s_routing - V4L2 decoder interface handler for VIDIOC_S_INPUT ioctl
 * @s: pointer to standard V4L2 device structure
 * @index: number of the input
 *
 * If index is valid, selects the requested input. Otherwise, returns -EINVAL if
 * the input is not supported or there is no active signal present in the
 * selected input.
 */
static int ioctl_s_routing(struct v4l2_int_device *s,
				struct v4l2_routing *route)
{
	return 0;
}

/**
 * ioctl_g_ifparm - V4L2 decoder interface handler for vidioc_int_g_ifparm_num
 * @s: pointer to standard V4L2 device structure
 * @p: pointer to standard V4L2 vidioc_int_g_ifparm_num ioctl structure
 *
 * Gets slave interface parameters.
 * Calculates the required xclk value to support the requested
 * clock parameters in p. This value is returned in the p
 * parameter.
 */
static int ioctl_g_ifparm(struct v4l2_int_device *s, struct v4l2_ifparm *p)
{
	struct mt9t111_sensor *sensor = s->priv;
	int rval;

	if (p == NULL)
		return -EINVAL;

	if (NULL == sensor->pdata->ifparm)
		return -EINVAL;

	rval = sensor->pdata->ifparm(p);
	if (rval) {
		v4l_err(sensor->i2c_client, "g_ifparm.Err[%d]\n", rval);
		return rval;
	}

	p->u.ycbcr.clock_curr = 40*1000000; // temporal value

	return 0;
}


static struct v4l2_int_ioctl_desc mt9t111_ioctl_desc[] = {
	{ .num = vidioc_int_enum_framesizes_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_enum_framesizes },
	{ .num = vidioc_int_enum_frameintervals_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_enum_frameintervals },
	{ .num = vidioc_int_dev_init_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_dev_init },
	{ .num = vidioc_int_dev_exit_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_dev_exit },
	{ .num = vidioc_int_s_power_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_s_power },
	{ .num = vidioc_int_g_priv_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_g_priv },
  	{vidioc_int_g_ifparm_num, 
  	  .func = (v4l2_int_ioctl_func*) ioctl_g_ifparm},
	{ .num = vidioc_int_init_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_init },
	{ .num = vidioc_int_enum_fmt_cap_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_enum_fmt_cap },
	{ .num = vidioc_int_try_fmt_cap_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_try_fmt_cap },
	{ .num = vidioc_int_g_fmt_cap_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_g_fmt_cap },
	{ .num = vidioc_int_s_fmt_cap_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_s_fmt_cap },
	{ .num = vidioc_int_g_parm_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_g_parm },
	{ .num = vidioc_int_s_parm_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_s_parm },
	{ .num = vidioc_int_queryctrl_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_queryctrl },
	{ .num = vidioc_int_g_ctrl_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_g_ctrl },
	{ .num = vidioc_int_s_ctrl_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_s_ctrl },
	{.num = vidioc_int_s_video_routing_num,
		.func = (v4l2_int_ioctl_func *) ioctl_s_routing},	  
};

static void mt9t111_refresh(struct i2c_client *client){	
	int i;	
	unsigned short value;		
	// MCU_ADDRESS [SEQ_CMD] -- refresh	
	mt9t111_write_reg(client, 0x098E,	    0x8400);	
	mt9t111_write_reg(client, 0x0990,	    0x0006);	
	for (i=0;i<100;i++){		
		mt9t111_write_reg(client, 0x098E, 0x8400);			
		mt9t111_read_reg(client,0x0990,&value);				
		if ( value == 0)			
			break;		
		mdelay(5);	
	}
}

#ifdef COLOR_BAR
static void mt9t111_color_bar(struct i2c_client *client)
{
	mt9t111_write_reg(client, 0x3210, 0x01B0); // disable lens correction

	mt9t111_write_reg(client, 0x098E, 0x6003);
	mt9t111_write_reg(client, 0x0990, 0x0100);
	mt9t111_write_reg(client, 0x098E, 0x6025);
	mt9t111_write_reg(client, 0x0990, 0x0003);
}
#endif

static void mt9t111_bayer_format(struct i2c_client *client)
{
	mt9t111_write_regs(client, bayer_pattern_regs, sizeof(bayer_pattern_regs)/sizeof(mt9t111_regs));
}

static void mt9t111_enable_pll(struct i2c_client *client)
{
	int i;
	unsigned short value; 

	mt9t111_write_regs(client, pll_regs1, sizeof(pll_regs1)/sizeof(mt9t111_regs));
	for (i=0;i<100;i++){
		mt9t111_read_reg(client,0x0014,&value);
		if (( value & 0x8000) != 0)
			break;
		mdelay(2);
	}
	mt9t111_write_regs(client, pll_regs2, sizeof(pll_regs2)/sizeof(mt9t111_regs));
}


static void mt9t111_loaddefault(struct i2c_client *client)
{
	mt9t111_write_reg(client, 0x001A, 0x0219);
	mt9t111_write_reg(client, 0x001A, 0x0218);

	mt9t111_enable_pll(client);
	mt9t111_write_regs(client, def_regs1, sizeof(def_regs1)/sizeof(mt9t111_regs));
	mt9t111_write_regs(client, patch_rev6, sizeof(patch_rev6)/sizeof(mt9t111_regs));
	mt9t111_write_regs(client, def_regs2, sizeof(def_regs2)/sizeof(mt9t111_regs));

#ifdef USE_RAW
	mt9t111_bayer_format(client);
#endif

#ifdef COLOR_BAR
	mt9t111_color_bar(client);
#endif

	mt9t111_refresh(client);
}

static struct v4l2_int_slave mt9t111_slave = {
	.ioctls = mt9t111_ioctl_desc,
	.num_ioctls = ARRAY_SIZE(mt9t111_ioctl_desc),
};

static struct v4l2_int_device mt9t111_int_device = {
	.module = THIS_MODULE,
	.name = "mt9t111",
	.priv = &mt9t111,
	.type = v4l2_int_type_slave,
	.u = {
		.slave = &mt9t111_slave,
	},
};

/**
 * mt9t111_probe - sensor driver i2c probe handler
 * @client: i2c driver client device structure
 *
 * Register sensor as an i2c client device and V4L2
 * device.
 */
static int
mt9t111_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct mt9t111_sensor *sensor = &mt9t111;
	int err;

	if (i2c_get_clientdata(client))
		return -EBUSY;

	sensor->pdata = client->dev.platform_data;

	if (!sensor->pdata) {
		dev_err(&client->dev, "no platform data?\n");
		return -ENODEV;
	}

	sensor->v4l2_int_device = &mt9t111_int_device;
	sensor->i2c_client = client;

	i2c_set_clientdata(client, sensor);

	sensor->pix.width = 640;
	sensor->pix.height = 480;
#ifdef USE_RAW	
	sensor->pix.pixelformat = V4L2_PIX_FMT_SGRBG10;
#else
	sensor->pix.pixelformat = V4L2_PIX_FMT_YUYV;
#endif
	err = v4l2_int_device_register(sensor->v4l2_int_device);
	if (err)
		i2c_set_clientdata(client, NULL);
	return err;
}

/**
 * mt9t111_remove - sensor driver i2c remove handler
 * @client: i2c driver client device structure
 *
 * Unregister sensor as an i2c client device and V4L2
 * device.  Complement of mt9t111_probe().
 */
static int __exit
mt9t111_remove(struct i2c_client *client)
{
	struct mt9t111_sensor *sensor = i2c_get_clientdata(client);

	if (!client->adapter)
		return -ENODEV;	/* our client isn't attached */

	v4l2_int_device_unregister(sensor->v4l2_int_device);
	i2c_set_clientdata(client, NULL);

	return 0;
}

static const struct i2c_device_id mt9t111_id[] = {
	{ "mt9t111", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, mt9t111_id);

static struct i2c_driver mt9t111sensor_i2c_driver = {
	.driver = {
		.name = "mt9t111",
		.owner = THIS_MODULE,
	},
	.probe = mt9t111_probe,
	.remove = __exit_p(mt9t111_remove),
	.id_table = mt9t111_id,
};

/**
 * mt9t111sensor_init - sensor driver module_init handler
 *
 * Registers driver as an i2c client driver.  Returns 0 on success,
 * error code otherwise.
 */
static int __init mt9t111sensor_init(void)
{
printk(KERN_INFO "entering mt9t111sensor_init\n");
	return i2c_add_driver(&mt9t111sensor_i2c_driver);
}
module_init(mt9t111sensor_init);

/**
 * mt9t111sensor_cleanup - sensor driver module_exit handler
 *
 * Unregisters/deletes driver as an i2c client driver.
 * Complement of mt9t111sensor_init.
 */
static void __exit mt9t111sensor_cleanup(void)
{
	i2c_del_driver(&mt9t111sensor_i2c_driver);
}
module_exit(mt9t111sensor_cleanup);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("mt9t111 camera sensor driver");
