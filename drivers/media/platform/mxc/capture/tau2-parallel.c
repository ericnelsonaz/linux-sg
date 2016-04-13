/* FLIR Tau2 parallel camera sensor
 *
 * Copyright (C) 2016 Nelson Integration LLC
 * Author: Eric Nelson <eric@nelint.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

/*!
 * @file tau2-parallel.c
 *
 * @brief FLIR Tau2 parallel camera sensor
 *
 * @ingroup Camera
 */
#define DEBUG

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fsl_devices.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/types.h>
#include <linux/videodev2.h>
#include <media/v4l2-chip-ident.h>
#include "mxc_v4l2_capture.h"

#define DRIVER_NAME     "tau2-parallel"

/***********************************************************************
 * IOCTL Functions from v4l2_int_ioctl_desc.
 ***********************************************************************/

/*!
 * ioctl_g_ifparm - V4L2 sensor interface handler for vidioc_int_g_ifparm_num
 * s: pointer to standard V4L2 device structure
 * p: pointer to standard V4L2 vidioc_int_g_ifparm_num ioctl structure
 *
 * Gets slave interface parameters.
 * Calculates the required xclk value to support the requested
 * clock parameters in p.  This value is returned in the p
 * parameter.
 *
 * vidioc_int_g_ifparm returns platform-specific information about the
 * interface settings used by the sensor.
 *
 * Called on open.
 */
static int ioctl_g_ifparm(struct v4l2_int_device *s, struct v4l2_ifparm *p)
{
	pr_info("%s\n", __func__);

	memset(p, 0, sizeof(*p));

	p->u.bt656.clock_curr = 24000000;
	p->if_type = V4L2_IF_TYPE_BT656;
	p->u.bt656.mode = V4L2_IF_TYPE_BT656_MODE_NOBT_16BIT;
	p->u.bt656.clock_min = p->u.bt656.clock_curr;
	p->u.bt656.clock_max = p->u.bt656.clock_curr;
	p->u.bt656.bt_sync_correct = 1;  /* Indicate external vsync */

	return 0;
}

/*!
 * ioctl_g_parm - V4L2 sensor interface handler for VIDIOC_G_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_G_PARM ioctl structure
 *
 * Returns the sensor's video CAPTURE parameters.
 */
static int ioctl_g_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	pr_info("%s\n", __func__);

	switch (a->type) {
		/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE: {
		struct sensor_data *sensor = s->priv;
		a->parm.capture.capability = sensor->streamcap.capability;
		a->parm.capture.capturemode = sensor->streamcap.capturemode;
		a->parm.capture.timeperframe.numerator = 1;
		a->parm.capture.timeperframe.denominator = 60;
		break;
	}
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		pr_info("ioctl_g_parm:type is unknown %d\n", a->type);
		break;
	}
	return 0;
}

/*!
 * ioctl_s_parm - V4L2 sensor interface handler for VIDIOC_S_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_S_PARM ioctl structure
 *
 * Configures the sensor to use the input parameters, if possible.  If
 * not possible, reverts to the old parameters and returns the
 * appropriate error code.
 *
 * This driver cannot change these settings.
 */
static int ioctl_s_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	int ret = 0;

	pr_info("%s\n", __func__);

	switch (a->type) {
	/* This is the only case currently handled. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		break;

	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		pr_info("   type is unknown - %d\n", a->type);
		ret = -EINVAL;
		break;
	}
	return ret;
}

/*!
 * ioctl_g_fmt_cap - V4L2 sensor interface handler for ioctl_g_fmt_cap
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 v4l2_format structure
 *
 * Returns the sensor's current pixel format in the v4l2_format
 * parameter.
 */
static int ioctl_g_fmt_cap(struct v4l2_int_device *s, struct v4l2_format *f)
{
	struct sensor_data *sensor = s->priv;

	pr_info("%s: type %d: sensor %p\n", __func__, f->type, sensor);

	switch (f->type) {
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		f->fmt.pix = sensor->pix;
		pr_info("%s: %dx%d, pixelfmt 0x%08x, %ubpl, %u bytes/frame, colorspace %d\n",
			__func__,
			sensor->pix.width, sensor->pix.height,
			sensor->pix.pixelformat, sensor->pix.bytesperline,
			sensor->pix.sizeimage, sensor->pix.colorspace);
		break;

	case V4L2_BUF_TYPE_SENSOR:
		f->fmt.spix = sensor->spix;
		pr_info("%s: left=%d, top=%d, %dx%d\n", __func__,
			sensor->spix.left, sensor->spix.top,
			sensor->spix.swidth, sensor->spix.sheight);
		break;

	default:
		break;
	}

	return 0;
}

/*!
 * ioctl_queryctrl - V4L2 sensor interface handler for VIDIOC_QUERYCTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @qc: standard V4L2 VIDIOC_QUERYCTRL ioctl structure
 *
 * If the requested control is supported, returns the control information
 * from the video_control[] array.  Otherwise, returns -EINVAL if the
 * control is not supported.
 */
static int ioctl_queryctrl(struct v4l2_int_device *s, struct v4l2_queryctrl *qc)
{
	pr_info("%s\n", __func__);
	return -EINVAL;
}

/*!
 * ioctl_g_ctrl - V4L2 sensor interface handler for VIDIOC_G_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_G_CTRL ioctl structure
 *
 * If the requested control is supported, returns the control's current
 * value from the video_control[] array.  Otherwise, returns -EINVAL
 * if the control is not supported.
 */
static int ioctl_g_ctrl(struct v4l2_int_device *s, struct v4l2_control *vc)
{
	int ret = 0;

	pr_info("%s:%s: %x\n", __FILE__, __func__, vc->id);

	return ret;
}

/*!
 * ioctl_s_ctrl - V4L2 sensor interface handler for VIDIOC_S_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_S_CTRL ioctl structure
 *
 * If the requested control is supported, sets the control's current
 * value in HW (and updates the video_control[] array).  Otherwise,
 * returns -EINVAL if the control is not supported.
 */
static int ioctl_s_ctrl(struct v4l2_int_device *s, struct v4l2_control *vc)
{
	pr_info("%s:%s: %x\n", __FILE__, __func__, vc->id);

	return 0;
}

/*!
 * ioctl_enum_framesizes - V4L2 sensor interface handler for
 *			   VIDIOC_ENUM_FRAMESIZES ioctl
 * @s: pointer to standard V4L2 device structure
 * @fsize: standard V4L2 VIDIOC_ENUM_FRAMESIZES ioctl structure
 *
 * Return 0 if successful, otherwise -EINVAL.
 */
static int ioctl_enum_framesizes(struct v4l2_int_device *s,
				 struct v4l2_frmsizeenum *fsize)
{
	struct sensor_data *sensor = s->priv;

	fsize->pixel_format = V4L2_PIX_FMT_Y16;
	fsize->discrete.width = sensor->pix.width;
	fsize->discrete.height = sensor->pix.height;

	pr_info("%s: %04x: %ux%u\n", __func__, fsize->pixel_format, fsize->discrete.width, fsize->discrete.height );
	return 0;
}

/*!
 * ioctl_g_chip_ident - V4L2 sensor interface handler for
 *			VIDIOC_DBG_G_CHIP_IDENT ioctl
 * @s: pointer to standard V4L2 device structure
 * @id: pointer to int
 *
 * Return 0.
 */
static int ioctl_g_chip_ident(struct v4l2_int_device *s, int *id)
{
	pr_info("%s\n", __func__);

	((struct v4l2_dbg_chip_ident *)id)->match.type =
	    V4L2_CHIP_MATCH_I2C_DRIVER;
	strcpy(((struct v4l2_dbg_chip_ident *)id)->match.name, DRIVER_NAME);

	return 0;
}

/*!
 * ioctl_enum_fmt_cap - V4L2 sensor interface handler for VIDIOC_ENUM_FMT
 * @s: pointer to standard V4L2 device structure
 * @fmt: pointer to standard V4L2 fmt description structure
 *
 * Return 0.
 */
static int ioctl_enum_fmt_cap(struct v4l2_int_device *s,
			      struct v4l2_fmtdesc *fmt)
{
	pr_info("%s: %d\n", __func__, fmt->index);
	if (fmt->index > 0)
		return -EINVAL;
	fmt->pixelformat = V4L2_PIX_FMT_Y16;

	return 0;
}

/*!
 * ioctl_dev_init - V4L2 sensor interface handler for vidioc_int_dev_init_num
 * @s: pointer to standard V4L2 device structure
 *
 * Initialise the device when slave attaches to the master.
 */
static int ioctl_dev_init(struct v4l2_int_device *s)
{
	pr_info("%s\n", __func__);
	return 0;
}

/*!
 * This structure defines all the ioctls for this module.
 */
static struct v4l2_int_ioctl_desc tau2_ioctl_desc[] = {

	{vidioc_int_dev_init_num, (v4l2_int_ioctl_func *) ioctl_dev_init},
	{vidioc_int_g_ifparm_num, (v4l2_int_ioctl_func *) ioctl_g_ifparm},

	/*!
	 * VIDIOC_ENUM_FMT ioctl for the CAPTURE buffer type.
	 */
	{vidioc_int_enum_fmt_cap_num,
	 (v4l2_int_ioctl_func *) ioctl_enum_fmt_cap},
	{vidioc_int_g_fmt_cap_num, (v4l2_int_ioctl_func *) ioctl_g_fmt_cap},
	{vidioc_int_g_parm_num, (v4l2_int_ioctl_func *) ioctl_g_parm},
	{vidioc_int_s_parm_num, (v4l2_int_ioctl_func *) ioctl_s_parm},
	{vidioc_int_queryctrl_num, (v4l2_int_ioctl_func *) ioctl_queryctrl},
	{vidioc_int_g_ctrl_num, (v4l2_int_ioctl_func *) ioctl_g_ctrl},
	{vidioc_int_s_ctrl_num, (v4l2_int_ioctl_func *) ioctl_s_ctrl},
	{vidioc_int_enum_framesizes_num,
	 (v4l2_int_ioctl_func *) ioctl_enum_framesizes},
	{vidioc_int_g_chip_ident_num,
	 (v4l2_int_ioctl_func *) ioctl_g_chip_ident},
};

static struct v4l2_int_slave tau2_slave = {
	.ioctls = tau2_ioctl_desc,
	.num_ioctls = ARRAY_SIZE(tau2_ioctl_desc),
};

static struct v4l2_int_device tau2_int_device = {
	.module = THIS_MODULE,
	.name = DRIVER_NAME,
	.type = v4l2_int_type_slave,
	.u = {
	      .slave = &tau2_slave,
	      },
};

/*!
 * probe function.
 *
 *  @param *plat - platform device descriptor.
 *
 *  @return	Error code indicating success or failure.
 */
static int tau2_probe(struct platform_device *plat)
{
	int ret = 0;
	struct pinctrl *pinctrl;
	struct sensor_data *sensor;

	pr_info("%s\n", __func__);

	pinctrl = devm_pinctrl_get_select_default(&plat->dev);
	if (IS_ERR(pinctrl))
		dev_err(&plat->dev, "no pinctrl\n");

	sensor = devm_kzalloc(&plat->dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return PTR_ERR(sensor);

	ret = of_property_read_u32(plat->dev.of_node, "ipu", &sensor->ipu_id);
	if (ret) {
		dev_err(&plat->dev, "ipu missing or invalid\n");
		goto bail;
	}

	ret = of_property_read_u32(plat->dev.of_node, "csi", &sensor->csi);
	if (ret) {
		dev_err(&plat->dev, "csi missing or invalid\n");
		goto bail;
	}

	sensor->pix.pixelformat = V4L2_PIX_FMT_Y16;
	sensor->pix.width = 336;
	sensor->spix.swidth = 665;
	sensor->spix.left = 0;
	sensor->pix.height = 256;
	sensor->spix.sheight = 256;
	sensor->spix.top = 0;
	sensor->streamcap.capability = V4L2_MODE_HIGHQUALITY |
	    V4L2_CAP_TIMEPERFRAME;
	sensor->pix.bytesperline = (2 * sensor->pix.width);
	sensor->pix.sizeimage = sensor->pix.bytesperline * sensor->pix.height;
	pr_err("%s: %u bytes/image\n", __func__, sensor->pix.sizeimage);
	sensor->streamcap.capturemode = 0;
	sensor->streamcap.timeperframe.denominator = 60;
	sensor->streamcap.timeperframe.numerator = 1;

	tau2_int_device.priv = sensor;

	ret = v4l2_int_device_register(&tau2_int_device);

	dev_err(&plat->dev, "%s: %d\n", __func__, ret);

bail:
	return ret;
}

static int tau2_remove(struct platform_device *plat)
{
	pr_info("%s\n", __func__);
	v4l2_int_device_unregister(&tau2_int_device);
	return 0;
}

static const struct of_device_id tau2_match[] = {
        { .compatible = "flir,tau2-parallel" },
        {},
};
MODULE_DEVICE_TABLE(of, tau2_match);

struct platform_driver tau2_driver = {
	.driver = {
                .name           = DRIVER_NAME,
                .of_match_table = of_match_ptr(tau2_match),
        },
	.probe = tau2_probe,
	.remove = tau2_remove,
};

static int __init mod_init(void)
{
	return platform_driver_probe(&tau2_driver, tau2_probe);
}

static void __exit mod_exit(void)
{
	platform_driver_unregister(&tau2_driver);
}

module_init(mod_init);
module_exit(mod_exit);

MODULE_DESCRIPTION("tau2 camera sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
