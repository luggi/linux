/*
 * Copyright (C) 2012-2015 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/v4l2-mediabus.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>

#define ov7740_VOLTAGE_ANALOG               2800000
#define ov7740_VOLTAGE_DIGITAL_CORE         1500000
#define ov7740_VOLTAGE_DIGITAL_IO           1800000

#define MIN_FPS 15
#define MAX_FPS 30
#define DEFAULT_FPS 30

#define ov7740_XCLK_MIN 6000000
#define ov7740_XCLK_MAX 27000000

#define OV7740_CHIP_ID_HIGH_BYTE	    0x1C
#define OV7740_CHIP_ID_HIGH_BYTE_READ   0x7F
#define OV7740_CHIP_ID_LOW_BYTE		    0x1D
#define OV7740_CHIP_ID_LOW_BYTE_READ	0xA2


enum ov7740_mode {
	ov7740_mode_MIN = 0,
	ov7740_mode_VGA_640_480 = 0,
	ov7740_mode_QVGA_320_240 = 1,
	ov7740_mode_MAX = 1
};

enum ov7740_frame_rate {
	ov7740_15_fps,
	ov7740_30_fps,
	ov7740_60_fps
};

static int ov7740_framerates[] = {
		[ov7740_15_fps] = 15,
		[ov7740_30_fps] = 30,
		[ov7740_60_fps] = 60,
};

struct ov7740_datafmt {
	u32	code;
	enum v4l2_colorspace		colorspace;
};

struct reg_value {
	u8 u8RegAddr;
	u8 u8Val;
	u8 u8Mask;
	u32 u32Delay_ms;
};

struct ov7740_mode_info {
	enum ov7740_mode mode;
	u32 width;
	u32 height;
	struct reg_value *init_data_ptr;
	u32 init_data_size;
};

struct ov7740 {
	struct v4l2_subdev		subdev;
	struct i2c_client *i2c_client;
	struct v4l2_pix_format pix;
	const struct ov7740_datafmt	*fmt;
	struct v4l2_captureparm streamcap;
	bool on;

	/* control settings */
	int brightness;
	int hue;
	int contrast;
	int saturation;
	int red;
	int green;
	int blue;
	int ae_mode;

	u32 mclk;
	u8 mclk_source;
	struct clk *sensor_clk;
	int csi;

	void (*io_init)(void);
};

/*!
 * Maintains the information on the current state of the sesor.
 */
static struct ov7740 ov7740_data;
static int pwn_gpio, rst_gpio;
static int prev_sysclk;
static int AE_Target = 52, night_mode;
static int prev_HTS;
static int AE_high, AE_low;

static struct reg_value ov7740_setting_15fps_QVGA_320_240[] = {
        {0x12,0x80, 0 , 10},
        {0x13,0x00, 0 , 0},

        {0x11,0x03, 0 , 0}, // clock divider = 4
#ifdef USE_BT656 /* BT656 0x12=>0x20 ou 0x31 */
        {0x12,0x20, 0 , 0},
#else
#ifdef USE_RAW
        {0x12,0x01, 0 , 0},
#else
        {0x12,0x00, 0 , 0}, /* YUYV */
#endif /* USE_RAW */
#endif /* USE_BT656 */
        {0xd5,0x10, 0 , 0},

        {0x0c,0x12, 0 , 0}, /* swap to YUYV */
        {0x0d,0x34, 0 , 0},
        {0x0e,0xe3, 0 , 0}, /* Output driving to 4x */

        {0x17,0x25, 0 , 0},
        {0x18,0xa0, 0 , 0},
        {0x19,0x03, 0 , 0},
        {0x1a,0xf0, 0 , 0},
        {0x1b,0x89, 0 , 0},
        {0x1e,0x13, 0 , 0},
        {0x22,0x03, 0 , 0},
        {0x29,0x17, 0 , 0},
        {0x2b,0xf8, 0 , 0},
        {0x2c,0x01, 0 , 0},
        {0x31,0x50, 0 , 0},
        {0x32,0x78, 0 , 0},
        {0x33,0xc4, 0 , 0},

        {0x3a,0xb4, 0 , 0},
        {0x36,0x3f, 0 , 0},

        {0x04,0x60, 0 , 0},
        {0x27,0x80, 0 , 0},
        {0x3d,0x0f, 0 , 0},
        {0x3e,0x82, 0 , 0},
        {0x3f,0x40, 0 , 0},
        {0x40,0x7f, 0 , 0},
        {0x41,0x6a, 0 , 0},
        {0x42,0x29, 0 , 0},
        {0x44,0xe5, 0 , 0},
        {0x45,0x41, 0 , 0},
        {0x47,0x42, 0 , 0},
        {0x48,0x00, 0 , 0},
        {0x49,0x61, 0 , 0},
        {0x4a,0xa1, 0 , 0},
        {0x4b,0x46, 0 , 0},
        {0x4c,0x18, 0 , 0},
        {0x4d,0x50, 0 , 0},
        {0x4e,0x13, 0 , 0},
        {0x64,0x00, 0 , 0},
        {0x67,0x88, 0 , 0},
        {0x68,0x1a, 0 , 0},

        {0x14,0x38, 0 , 0},
        {0x24,0x3c, 0 , 0},
        {0x25,0x30, 0 , 0},
        {0x26,0x72, 0 , 0},
        {0x50,0x4c, 0 , 0},
        {0x51,0x3f, 0 , 0},
        {0x52,0x00, 0 , 0},
        {0x53,0x00, 0 , 0},
        {0x20,0x00, 0 , 0},
        {0x21,0x57, 0 , 0},
        {0x38,0x14, 0 , 0},
        {0xe9,0x00, 0 , 0},
        {0x56,0x55, 0 , 0},
        {0x57,0xff, 0 , 0},
        {0x58,0xff, 0 , 0},
        {0x59,0xff, 0 , 0},
        {0x5f,0x04, 0 , 0},
        {0x13,0xff, 0 , 0},

        {0x80,0x7d, 0 , 0},
        {0x81,0x3f, 0 , 0},/* YUV422 enabled */
        {0x82,0x3f, 0 , 0},
        {0x83,0x03, 0 , 0},
        {0x38,0x11, 0 , 0},
        {0x84,0x70, 0 , 0},
        {0x85,0x00, 0 , 0},
        {0x86,0x03, 0 , 0},
        {0x87,0x01, 0 , 0},
        {0x88,0x05, 0 , 0},
        {0x89,0x30, 0 , 0},
        {0x8d,0x30, 0 , 0},
        {0x8f,0x85, 0 , 0},
        {0x93,0x30, 0 , 0},
        {0x95,0x85, 0 , 0},
        {0x99,0x30, 0 , 0},
        {0x9b,0x85, 0 , 0},

        {0x9c,0x08, 0 , 0},
        {0x9d,0x12, 0 , 0},
        {0x9e,0x23, 0 , 0},
        {0x9f,0x45, 0 , 0},
        {0xa0,0x55, 0 , 0},
        {0xa1,0x64, 0 , 0},
        {0xa2,0x72, 0 , 0},
        {0xa3,0x7f, 0 , 0},
        {0xa4,0x8b, 0 , 0},
        {0xa5,0x95, 0 , 0},
        {0xa6,0xa7, 0 , 0},
        {0xa7,0xb5, 0 , 0},
        {0xa8,0xcb, 0 , 0},
        {0xa9,0xdd, 0 , 0},
        {0xaa,0xec, 0 , 0},
        {0xab,0x1a, 0 , 0},

        {0xce,0x78, 0 , 0},
        {0xcf,0x6e, 0 , 0},
        {0xd0,0x0a, 0 , 0},
        {0xd1,0x0c, 0 , 0},
        {0xd2,0x84, 0 , 0},
        {0xd3,0x90, 0 , 0},
        {0xd4,0x1e, 0 , 0},

        {0x5a,0x24, 0 , 0},
        {0x5b,0x1f, 0 , 0},
        {0x5c,0x88, 0 , 0},
        {0x5d,0x60, 0 , 0},

        {0xac,0x6e, 0 , 0},
        {0xbe,0xff, 0 , 0},
        {0xbf,0x00, 0 , 0},
        {0xe5,0x8c, 0 , 0},

        {0x70,0x00, 0 , 0},
        {0x71,0x34, 0 , 0},
        {0x74,0x28, 0 , 0},
        {0x75,0x98, 0 , 0},
        {0x76,0x00, 0 , 0},
        {0x77,0x08, 0 , 0},
        {0x78,0x01, 0 , 0},
        {0x79,0xc2, 0 , 0},
        {0x7d,0x02, 0 , 0},
        {0x7a,0x9c, 0 , 0},
        {0x7b,0x40, 0 , 0},
        {0xec,0x02, 0 , 0},
        {0x7c,0x0c, 0 , 0},
};

static struct reg_value ov7740_setting_30fps_QVGA_320_240[] = {
        {0x12,0x80, 0 , 10},
        {0x13,0x00, 0 , 0},

        {0x11,0x01, 0 , 0}, // clock divider = 2
#ifdef USE_BT656 /* BT656 0x12=>0x20 ou 0x31 */
        {0x12,0x20, 0 , 0},
#else
#ifdef USE_RAW
        {0x12,0x01, 0 , 0},
#else
        {0x12,0x00, 0 , 0}, /* YUYV */
#endif /* USE_RAW */
#endif /* USE_BT656 */
        {0xd5,0x10, 0 , 0},

        {0x0c,0x12, 0 , 0}, /* swap to YUYV */
        {0x0d,0x34, 0 , 0},
        {0x0e,0xe3, 0 , 0}, /* Output driving to 4x */

        {0x17,0x25, 0 , 0},
        {0x18,0xa0, 0 , 0},
        {0x19,0x03, 0 , 0},
        {0x1a,0xf0, 0 , 0},
        {0x1b,0x89, 0 , 0},
        {0x1e,0x13, 0 , 0},
        {0x22,0x03, 0 , 0},
        {0x29,0x17, 0 , 0},
        {0x2b,0xf8, 0 , 0},
        {0x2c,0x01, 0 , 0},
        {0x31,0x50, 0 , 0},
        {0x32,0x78, 0 , 0},
        {0x33,0xc4, 0 , 0},

        {0x3a,0xb4, 0 , 0},
        {0x36,0x3f, 0 , 0},

        {0x04,0x60, 0 , 0},
        {0x27,0x80, 0 , 0},
        {0x3d,0x0f, 0 , 0},
        {0x3e,0x82, 0 , 0},
        {0x3f,0x40, 0 , 0},
        {0x40,0x7f, 0 , 0},
        {0x41,0x6a, 0 , 0},
        {0x42,0x29, 0 , 0},
        {0x44,0xe5, 0 , 0},
        {0x45,0x41, 0 , 0},
        {0x47,0x42, 0 , 0},
        {0x48,0x00, 0 , 0},
        {0x49,0x61, 0 , 0},
        {0x4a,0xa1, 0 , 0},
        {0x4b,0x46, 0 , 0},
        {0x4c,0x18, 0 , 0},
        {0x4d,0x50, 0 , 0},
        {0x4e,0x13, 0 , 0},
        {0x64,0x00, 0 , 0},
        {0x67,0x88, 0 , 0},
        {0x68,0x1a, 0 , 0},

        {0x14,0x38, 0 , 0},
        {0x24,0x3c, 0 , 0},
        {0x25,0x30, 0 , 0},
        {0x26,0x72, 0 , 0},
        {0x50,0x97, 0 , 0},
        {0x51,0x7e, 0 , 0},
        {0x52,0x00, 0 , 0},
        {0x53,0x00, 0 , 0},
        {0x20,0x00, 0 , 0},
        {0x21,0x23, 0 , 0},
        {0x38,0x14, 0 , 0},
        {0xe9,0x00, 0 , 0},
        {0x56,0x55, 0 , 0},
        {0x57,0xff, 0 , 0},
        {0x58,0xff, 0 , 0},
        {0x59,0xff, 0 , 0},
        {0x5f,0x04, 0 , 0},
        {0x13,0xff, 0 , 0},

        {0x80,0x7d, 0 , 0},
        {0x81,0x3f, 0 , 0},/* YUV422 enabled */
        {0x82,0x3f, 0 , 0},
        {0x83,0x03, 0 , 0},
        {0x38,0x11, 0 , 0},
        {0x84,0x70, 0 , 0},
        {0x85,0x00, 0 , 0},
        {0x86,0x03, 0 , 0},
        {0x87,0x01, 0 , 0},
        {0x88,0x05, 0 , 0},
        {0x89,0x30, 0 , 0},
        {0x8d,0x30, 0 , 0},
        {0x8f,0x85, 0 , 0},
        {0x93,0x30, 0 , 0},
        {0x95,0x85, 0 , 0},
        {0x99,0x30, 0 , 0},
        {0x9b,0x85, 0 , 0},

        {0x9c,0x08, 0 , 0},
        {0x9d,0x12, 0 , 0},
        {0x9e,0x23, 0 , 0},
        {0x9f,0x45, 0 , 0},
        {0xa0,0x55, 0 , 0},
        {0xa1,0x64, 0 , 0},
        {0xa2,0x72, 0 , 0},
        {0xa3,0x7f, 0 , 0},
        {0xa4,0x8b, 0 , 0},
        {0xa5,0x95, 0 , 0},
        {0xa6,0xa7, 0 , 0},
        {0xa7,0xb5, 0 , 0},
        {0xa8,0xcb, 0 , 0},
        {0xa9,0xdd, 0 , 0},
        {0xaa,0xec, 0 , 0},
        {0xab,0x1a, 0 , 0},

        {0xce,0x78, 0 , 0},
        {0xcf,0x6e, 0 , 0},
        {0xd0,0x0a, 0 , 0},
        {0xd1,0x0c, 0 , 0},
        {0xd2,0x84, 0 , 0},
        {0xd3,0x90, 0 , 0},
        {0xd4,0x1e, 0 , 0},

        {0x5a,0x24, 0 , 0},
        {0x5b,0x1f, 0 , 0},
        {0x5c,0x88, 0 , 0},
        {0x5d,0x60, 0 , 0},

        {0xac,0x6e, 0 , 0},
        {0xbe,0xff, 0 , 0},
        {0xbf,0x00, 0 , 0},
        {0xe5,0x8c, 0 , 0},

        {0x70,0x00, 0 , 0},
        {0x71,0x34, 0 , 0},
        {0x74,0x28, 0 , 0},
        {0x75,0x98, 0 , 0},
        {0x76,0x00, 0 , 0},
        {0x77,0x08, 0 , 0},
        {0x78,0x01, 0 , 0},
        {0x79,0xc2, 0 , 0},
        {0x7d,0x02, 0 , 0},
        {0x7a,0x9c, 0 , 0},
        {0x7b,0x40, 0 , 0},
        {0xec,0x02, 0 , 0},
        {0x7c,0x0c, 0 , 0},

};
static struct reg_value ov7740_setting_60fps_QVGA_320_240[] = {
        {0x12,0x80, 0 , 10},
        {0x13,0x00, 0 , 0},

        {0x11,0x00, 0 , 0}, // clock divider = 1
#ifdef USE_BT656 /* BT656 0x12=>0x20 ou 0x31 */
        {0x12,0x20, 0 , 0},
#else
#ifdef USE_RAW
        {0x12,0x01, 0 , 0},
#else
        {0x12,0x00, 0 , 0}, /* YUYV */
#endif /* USE_RAW */
#endif /* USE_BT656 */
        {0xd5,0x10, 0 , 0},

        {0x0c,0x12, 0 , 0}, /* swap to YUYV */
        {0x0d,0x34, 0 , 0},
        {0x0e,0xe3, 0 , 0}, /* Output driving to 4x */

        {0x17,0x25, 0 , 0},
        {0x18,0xa0, 0 , 0},
        {0x19,0x03, 0 , 0},
        {0x1a,0xf0, 0 , 0},
        {0x1b,0x89, 0 , 0},
        {0x1e,0x13, 0 , 0},
        {0x22,0x03, 0 , 0},
        {0x29,0x17, 0 , 0},
        {0x2b,0xf8, 0 , 0},
        {0x2c,0x01, 0 , 0},
        {0x31,0x50, 0 , 0},
        {0x32,0x78, 0 , 0},
        {0x33,0xc4, 0 , 0},

        {0x3a,0xb4, 0 , 0},
        {0x36,0x3f, 0 , 0},

        {0x04,0x60, 0 , 0},
        {0x27,0x80, 0 , 0},
        {0x3d,0x0f, 0 , 0},
        {0x3e,0x82, 0 , 0},
        {0x3f,0x40, 0 , 0},
        {0x40,0x7f, 0 , 0},
        {0x41,0x6a, 0 , 0},
        {0x42,0x29, 0 , 0},
        {0x44,0xe5, 0 , 0},
        {0x45,0x41, 0 , 0},
        {0x47,0x42, 0 , 0},
        {0x48,0x00, 0 , 0},
        {0x49,0x61, 0 , 0},
        {0x4a,0xa1, 0 , 0},
        {0x4b,0x46, 0 , 0},
        {0x4c,0x18, 0 , 0},
        {0x4d,0x50, 0 , 0},
        {0x4e,0x13, 0 , 0},
        {0x64,0x00, 0 , 0},
        {0x67,0x88, 0 , 0},
        {0x68,0x1a, 0 , 0},

        {0x14,0x38, 0 , 0},
        {0x24,0x3c, 0 , 0},
        {0x25,0x30, 0 , 0},
        {0x26,0x72, 0 , 0},
        {0x50,0x97, 0 , 0},
        {0x51,0x7e, 0 , 0},
        {0x52,0x00, 0 , 0},
        {0x53,0x00, 0 , 0},
        {0x20,0x00, 0 , 0},
        {0x21,0x23, 0 , 0},
        {0x38,0x14, 0 , 0},
        {0xe9,0x00, 0 , 0},
        {0x56,0x55, 0 , 0},
        {0x57,0xff, 0 , 0},
        {0x58,0xff, 0 , 0},
        {0x59,0xff, 0 , 0},
        {0x5f,0x04, 0 , 0},
        {0x13,0xff, 0 , 0},

        {0x80,0x7d, 0 , 0},
        {0x81,0x3f, 0 , 0},/* YUV422 enabled */
        {0x82,0x3f, 0 , 0},
        {0x83,0x03, 0 , 0},
        {0x38,0x11, 0 , 0},
        {0x84,0x70, 0 , 0},
        {0x85,0x00, 0 , 0},
        {0x86,0x03, 0 , 0},
        {0x87,0x01, 0 , 0},
        {0x88,0x05, 0 , 0},
        {0x89,0x30, 0 , 0},
        {0x8d,0x30, 0 , 0},
        {0x8f,0x85, 0 , 0},
        {0x93,0x30, 0 , 0},
        {0x95,0x85, 0 , 0},
        {0x99,0x30, 0 , 0},
        {0x9b,0x85, 0 , 0},

        {0x9c,0x08, 0 , 0},
        {0x9d,0x12, 0 , 0},
        {0x9e,0x23, 0 , 0},
        {0x9f,0x45, 0 , 0},
        {0xa0,0x55, 0 , 0},
        {0xa1,0x64, 0 , 0},
        {0xa2,0x72, 0 , 0},
        {0xa3,0x7f, 0 , 0},
        {0xa4,0x8b, 0 , 0},
        {0xa5,0x95, 0 , 0},
        {0xa6,0xa7, 0 , 0},
        {0xa7,0xb5, 0 , 0},
        {0xa8,0xcb, 0 , 0},
        {0xa9,0xdd, 0 , 0},
        {0xaa,0xec, 0 , 0},
        {0xab,0x1a, 0 , 0},

        {0xce,0x78, 0 , 0},
        {0xcf,0x6e, 0 , 0},
        {0xd0,0x0a, 0 , 0},
        {0xd1,0x0c, 0 , 0},
        {0xd2,0x84, 0 , 0},
        {0xd3,0x90, 0 , 0},
        {0xd4,0x1e, 0 , 0},

        {0x5a,0x24, 0 , 0},
        {0x5b,0x1f, 0 , 0},
        {0x5c,0x88, 0 , 0},
        {0x5d,0x60, 0 , 0},

        {0xac,0x6e, 0 , 0},
        {0xbe,0xff, 0 , 0},
        {0xbf,0x00, 0 , 0},
        {0xe5,0x8c, 0 , 0},

        {0x70,0x00, 0 , 0},
        {0x71,0x34, 0 , 0},
        {0x74,0x28, 0 , 0},
        {0x75,0x98, 0 , 0},
        {0x76,0x00, 0 , 0},
        {0x77,0x08, 0 , 0},
        {0x78,0x01, 0 , 0},
        {0x79,0xc2, 0 , 0},
        {0x7d,0x02, 0 , 0},
        {0x7a,0x9c, 0 , 0},
        {0x7b,0x40, 0 , 0},
        {0xec,0x02, 0 , 0},
        {0x7c,0x0c, 0 , 0},

};
static struct reg_value ov7740_setting_15fps_VGA_640_480[] = {
        {0x12,0x80, 0 , 10},
        {0x13,0x00, 0 , 0},

        {0x11,0x03, 0 , 0},
#ifdef USE_BT656 /* BT656 0x12=>0x20 ou 0x31 */
        {0x12,0x20, 0 , 0},
#else
#ifdef USE_RAW
        {0x12,0x01, 0 , 0},
#else
        {0x12,0x00, 0 , 0}, /* YUYV */
#endif /* USE_RAW */
#endif /* USE_BT656 */
        {0xd5,0x10, 0 , 0},

        {0x0c,0x12, 0 , 0}, /* swap to YUYV */
        {0x0d,0x34, 0 , 0},
        {0x0e,0xe3, 0 , 0}, /* Output driving to 4x */

        {0x17,0x25, 0 , 0},
        {0x18,0xa0, 0 , 0},
        {0x19,0x03, 0 , 0},
        {0x1a,0xf0, 0 , 0},
        {0x1b,0x89, 0 , 0},
        {0x1e,0x13, 0 , 0},
        {0x22,0x03, 0 , 0},
        {0x29,0x17, 0 , 0},
        {0x2b,0xf8, 0 , 0},
        {0x2c,0x01, 0 , 0},
        {0x31,0xa0, 0 , 0},
        {0x32,0xf0, 0 , 0},
        {0x33,0xc4, 0 , 0},

        {0x3a,0xb4, 0 , 0},
        {0x36,0x3f, 0 , 0},

        {0x04,0x60, 0 , 0},
        {0x27,0x80, 0 , 0},
        {0x3d,0x0f, 0 , 0},
        {0x3e,0x82, 0 , 0},
        {0x3f,0x40, 0 , 0},
        {0x40,0x7f, 0 , 0},
        {0x41,0x6a, 0 , 0},
        {0x42,0x29, 0 , 0},
        {0x44,0xe5, 0 , 0},
        {0x45,0x41, 0 , 0},
        {0x47,0x42, 0 , 0},
        {0x48,0x00, 0 , 0},
        {0x49,0x61, 0 , 0},
        {0x4a,0xa1, 0 , 0},
        {0x4b,0x46, 0 , 0},
        {0x4c,0x18, 0 , 0},
        {0x4d,0x50, 0 , 0},
        {0x4e,0x13, 0 , 0},
        {0x64,0x00, 0 , 0},
        {0x67,0x88, 0 , 0},
        {0x68,0x1a, 0 , 0},

        {0x14,0x38, 0 , 0},
        {0x24,0x3c, 0 , 0},
        {0x25,0x30, 0 , 0},
        {0x26,0x72, 0 , 0},
        {0x50,0x4c, 0 , 0},
        {0x51,0x3f, 0 , 0},
        {0x52,0x00, 0 , 0},
        {0x53,0x00, 0 , 0},
        {0x20,0x00, 0 , 0},
        {0x21,0x57, 0 , 0},
        {0x38,0x14, 0 , 0},
        {0xe9,0x00, 0 , 0},
        {0x56,0x55, 0 , 0},
        {0x57,0xff, 0 , 0},
        {0x58,0xff, 0 , 0},
        {0x59,0xff, 0 , 0},
        {0x5f,0x04, 0 , 0},
        {0x13,0xff, 0 , 0},

        {0x80,0x7d, 0 , 0},
        {0x81,0x3f, 0 , 0},/* YUV422 enabled */
        {0x82,0x32, 0 , 0},
        {0x83,0x03, 0 , 0},
        {0x38,0x11, 0 , 0},
        {0x84,0x70, 0 , 0},
        {0x85,0x00, 0 , 0},
        {0x86,0x03, 0 , 0},
        {0x87,0x01, 0 , 0},
        {0x88,0x05, 0 , 0},
        {0x89,0x30, 0 , 0},
        {0x8d,0x30, 0 , 0},
        {0x8f,0x85, 0 , 0},
        {0x93,0x30, 0 , 0},
        {0x95,0x85, 0 , 0},
        {0x99,0x30, 0 , 0},
        {0x9b,0x85, 0 , 0},

        {0x9c,0x08, 0 , 0},
        {0x9d,0x12, 0 , 0},
        {0x9e,0x23, 0 , 0},
        {0x9f,0x45, 0 , 0},
        {0xa0,0x55, 0 , 0},
        {0xa1,0x64, 0 , 0},
        {0xa2,0x72, 0 , 0},
        {0xa3,0x7f, 0 , 0},
        {0xa4,0x8b, 0 , 0},
        {0xa5,0x95, 0 , 0},
        {0xa6,0xa7, 0 , 0},
        {0xa7,0xb5, 0 , 0},
        {0xa8,0xcb, 0 , 0},
        {0xa9,0xdd, 0 , 0},
        {0xaa,0xec, 0 , 0},
        {0xab,0x1a, 0 , 0},

        {0xce,0x78, 0 , 0},
        {0xcf,0x6e, 0 , 0},
        {0xd0,0x0a, 0 , 0},
        {0xd1,0x0c, 0 , 0},
        {0xd2,0x84, 0 , 0},
        {0xd3,0x90, 0 , 0},
        {0xd4,0x1e, 0 , 0},

        {0x5a,0x24, 0 , 0},
        {0x5b,0x1f, 0 , 0},
        {0x5c,0x88, 0 , 0},
        {0x5d,0x60, 0 , 0},

        {0xac,0x6e, 0 , 0},
        {0xbe,0xff, 0 , 0},
        {0xbf,0x00, 0 , 0},

        {0x70,0x00, 0 , 0},
        {0x71,0x34, 0 , 0},
        {0x74,0x28, 0 , 0},
        {0x75,0x98, 0 , 0},
        {0x76,0x00, 0 , 0},
        {0x77,0x08, 0 , 0},
        {0x78,0x01, 0 , 0},
        {0x79,0xc2, 0 , 0},
        {0x7d,0x02, 0 , 0},
        {0x7a,0x9c, 0 , 0},
        {0x7b,0x40, 0 , 0},
        {0xec,0x82, 0 , 0},
        {0x7c,0x0c, 0 , 0},
};

//default one
static struct reg_value ov7740_setting_30fps_VGA_640_480[] = {

        {0x12,0x80, 0 , 10},
        {0x13,0x00, 0 , 0},

        {0x11,0x01, 0 , 0},
#ifdef USE_BT656 /* BT656 0x12=>0x20 ou 0x31 */
        {0x12,0x20, 0 , 0},
#else
#ifdef USE_RAW
        {0x12,0x01, 0 , 0},
#else
        {0x12,0x00, 0 , 0}, /* YUYV */
#endif /* USE_RAW */
#endif /* USE_BT656 */
        {0xd5,0x10, 0 , 0},

        {0x0c,0x12, 0 , 0}, /* swap to YUYV */
        {0x0d,0x34, 0 , 0},
        {0x0e,0xe3, 0 , 0}, /* Output driving to 4x */

        {0x17,0x25, 0 , 0},
        {0x18,0xa0, 0 , 0},
        {0x19,0x03, 0 , 0},
        {0x1a,0xf0, 0 , 0},
        {0x1b,0x89, 0 , 0},
        {0x1e,0x13, 0 , 0},
        {0x22,0x03, 0 , 0},
        {0x29,0x17, 0 , 0},
        {0x2b,0xf8, 0 , 0},
        {0x2c,0x01, 0 , 0},
        {0x31,0xa0, 0 , 0},
        {0x32,0xf0, 0 , 0},
        {0x33,0xc4, 0 , 0},

        {0x3a,0xb4, 0 , 0},
        {0x36,0x3f, 0 , 0},

        {0x04,0x60, 0 , 0},
        {0x27,0x80, 0 , 0},
        {0x3d,0x0f, 0 , 0},
        {0x3e,0x82, 0 , 0},
        {0x3f,0x40, 0 , 0},
        {0x40,0x7f, 0 , 0},
        {0x41,0x6a, 0 , 0},
        {0x42,0x29, 0 , 0},
        {0x44,0xe5, 0 , 0},
        {0x45,0x41, 0 , 0},
        {0x47,0x42, 0 , 0},
        {0x48,0x00, 0 , 0},
        {0x49,0x61, 0 , 0},
        {0x4a,0xa1, 0 , 0},
        {0x4b,0x46, 0 , 0},
        {0x4c,0x18, 0 , 0},
        {0x4d,0x50, 0 , 0},
        {0x4e,0x13, 0 , 0},
        {0x64,0x00, 0 , 0},
        {0x67,0x88, 0 , 0},
        {0x68,0x1a, 0 , 0},

        {0x14,0x38, 0 , 0},
        {0x24,0x3c, 0 , 0},
        {0x25,0x30, 0 , 0},
        {0x26,0x72, 0 , 0},
        {0x50,0x97, 0 , 0},
        {0x51,0x7e, 0 , 0},
        {0x52,0x00, 0 , 0},
        {0x53,0x00, 0 , 0},
        {0x20,0x00, 0 , 0},
        {0x21,0x23, 0 , 0},
        {0x38,0x14, 0 , 0},
        {0xe9,0x00, 0 , 0},
        {0x56,0x55, 0 , 0},
        {0x57,0xff, 0 , 0},
        {0x58,0xff, 0 , 0},
        {0x59,0xff, 0 , 0},
        {0x5f,0x04, 0 , 0},
        {0x13,0xff, 0 , 0},

        {0x80,0x7d, 0 , 0},
        {0x81,0x3f, 0 , 0},/* YUV422 enabled */
        {0x82,0x32, 0 , 0},
        {0x83,0x03, 0 , 0},
        {0x38,0x11, 0 , 0},
        {0x84,0x70, 0 , 0},
        {0x85,0x00, 0 , 0},
        {0x86,0x03, 0 , 0},
        {0x87,0x01, 0 , 0},
        {0x88,0x05, 0 , 0},
        {0x89,0x30, 0 , 0},
        {0x8d,0x30, 0 , 0},
        {0x8f,0x85, 0 , 0},
        {0x93,0x30, 0 , 0},
        {0x95,0x85, 0 , 0},
        {0x99,0x30, 0 , 0},
        {0x9b,0x85, 0 , 0},

        {0x9c,0x08, 0 , 0},
        {0x9d,0x12, 0 , 0},
        {0x9e,0x23, 0 , 0},
        {0x9f,0x45, 0 , 0},
        {0xa0,0x55, 0 , 0},
        {0xa1,0x64, 0 , 0},
        {0xa2,0x72, 0 , 0},
        {0xa3,0x7f, 0 , 0},
        {0xa4,0x8b, 0 , 0},
        {0xa5,0x95, 0 , 0},
        {0xa6,0xa7, 0 , 0},
        {0xa7,0xb5, 0 , 0},
        {0xa8,0xcb, 0 , 0},
        {0xa9,0xdd, 0 , 0},
        {0xaa,0xec, 0 , 0},
        {0xab,0x1a, 0 , 0},

        {0xce,0x78, 0 , 0},
        {0xcf,0x6e, 0 , 0},
        {0xd0,0x0a, 0 , 0},
        {0xd1,0x0c, 0 , 0},
        {0xd2,0x84, 0 , 0},
        {0xd3,0x90, 0 , 0},
        {0xd4,0x1e, 0 , 0},

        {0x5a,0x24, 0 , 0},
        {0x5b,0x1f, 0 , 0},
        {0x5c,0x88, 0 , 0},
        {0x5d,0x60, 0 , 0},

        {0xac,0x6e, 0 , 0},
        {0xbe,0xff, 0 , 0},
        {0xbf,0x00, 0 , 0},

        {0x70,0x00, 0 , 0},
        {0x71,0x34, 0 , 0},
        {0x74,0x28, 0 , 0},
        {0x75,0x98, 0 , 0},
        {0x76,0x00, 0 , 0},
        {0x77,0x08, 0 , 0},
        {0x78,0x01, 0 , 0},
        {0x79,0xc2, 0 , 0},
        {0x7d,0x02, 0 , 0},
        {0x7a,0x9c, 0 , 0},
        {0x7b,0x40, 0 , 0},
        {0xec,0x82, 0 , 0},
        {0x7c,0x0c, 0 , 0},

};

static struct ov7740_mode_info ov7740_mode_info_data[2][ov7740_mode_MAX + 1] = {
		{
				{ov7740_mode_VGA_640_480,      640,  480,
						ov7740_setting_15fps_VGA_640_480,
						ARRAY_SIZE(ov7740_setting_15fps_VGA_640_480)},
				{ov7740_mode_QVGA_320_240,     320,  240,
						ov7740_setting_15fps_QVGA_320_240,
						ARRAY_SIZE(ov7740_setting_15fps_QVGA_320_240)},

		},
		{
				{ov7740_mode_VGA_640_480,      640,  480,
						ov7740_setting_30fps_VGA_640_480,
						ARRAY_SIZE(ov7740_setting_30fps_VGA_640_480)},
				{ov7740_mode_QVGA_320_240,     320,  240,
						ov7740_setting_30fps_QVGA_320_240,
						ARRAY_SIZE(ov7740_setting_30fps_QVGA_320_240)},

		},
};

static struct regulator *io_regulator;
static struct regulator *core_regulator;
static struct regulator *analog_regulator;

static int ov7740_probe(struct i2c_client *adapter,
						const struct i2c_device_id *device_id);
static int ov7740_remove(struct i2c_client *client);

static s32 ov7740_read_reg(u16 reg, u8 *val);
static s32 ov7740_write_reg(u8 reg, u8 val);

static const struct i2c_device_id ov7740_id[] = {
		{"ov7740", 0},
		{},
};

MODULE_DEVICE_TABLE(i2c, ov7740_id);

static struct i2c_driver ov7740_i2c_driver = {
		.driver = {
				.owner = THIS_MODULE,
				.name  = "ov7740",
		},
		.probe  = ov7740_probe,
		.remove = ov7740_remove,
		.id_table = ov7740_id,
};

static const struct ov7740_datafmt ov7740_colour_fmts[] = {
		{MEDIA_BUS_FMT_YUYV8_2X8, V4L2_COLORSPACE_JPEG},
};

static struct ov7740 *to_ov7740(const struct i2c_client *client)
{
	return container_of(i2c_get_clientdata(client), struct ov7740, subdev);
}

/* Find a data format by a pixel code in an array */
static const struct ov7740_datafmt
*ov7740_find_datafmt(u32 code)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ov7740_colour_fmts); i++)
		if (ov7740_colour_fmts[i].code == code)
			return ov7740_colour_fmts + i;

	return NULL;
}

static inline void ov7740_power_down(int enable)
{
	gpio_set_value_cansleep(pwn_gpio, enable);

	msleep(2);
}

static inline void ov7740_reset(void)
{
	/* camera reset */
	gpio_set_value_cansleep(rst_gpio, 1);
}

static int ov7740_regulator_enable(struct device *dev)
{
	int ret = 0;

	io_regulator = devm_regulator_get(dev, "DOVDD");
	if (!IS_ERR(io_regulator)) {
		regulator_set_voltage(io_regulator,
							  ov7740_VOLTAGE_DIGITAL_IO,
							  ov7740_VOLTAGE_DIGITAL_IO);
		ret = regulator_enable(io_regulator);
		if (ret) {
			dev_err(dev, "set io voltage failed\n");
			return ret;
		} else {
			dev_dbg(dev, "set io voltage ok\n");
		}
	} else {
		io_regulator = NULL;
		dev_warn(dev, "cannot get io voltage\n");
	}

	core_regulator = devm_regulator_get(dev, "DVDD");
	if (!IS_ERR(core_regulator)) {
		regulator_set_voltage(core_regulator,
							  ov7740_VOLTAGE_DIGITAL_CORE,
							  ov7740_VOLTAGE_DIGITAL_CORE);
		ret = regulator_enable(core_regulator);
		if (ret) {
			dev_err(dev, "set core voltage failed\n");
			return ret;
		} else {
			dev_dbg(dev, "set core voltage ok\n");
		}
	} else {
		core_regulator = NULL;
		dev_warn(dev, "cannot get core voltage\n");
	}

	analog_regulator = devm_regulator_get(dev, "AVDD");
	if (!IS_ERR(analog_regulator)) {
		regulator_set_voltage(analog_regulator,
							  ov7740_VOLTAGE_ANALOG,
							  ov7740_VOLTAGE_ANALOG);
		ret = regulator_enable(analog_regulator);
		if (ret) {
			dev_err(dev, "set analog voltage failed\n");
			return ret;
		} else {
			dev_dbg(dev, "set analog voltage ok\n");
		}
	} else {
		analog_regulator = NULL;
		dev_warn(dev, "cannot get analog voltage\n");
	}

	return ret;
}

static s32 ov7740_write_reg(u8 reg, u8 val)
{
	return i2c_smbus_write_byte_data(ov7740_data.i2c_client, reg, val);
}

static s32 ov7740_read_reg(u16 reg, u8 *val)
{
	u8 u8RdVal = 0;

	u8RdVal = i2c_smbus_read_byte_data(ov7740_data.i2c_client, reg);

	*val = u8RdVal;

	return u8RdVal;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int ov7740_get_register(struct v4l2_subdev *sd,
					struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
	u8 val;

	if (reg->reg & ~0xffff)
		return -EINVAL;

	reg->size = 1;

	ret = ov7740_read_reg(reg->reg, &val);
	if (!ret)
		reg->val = (__u64)val;

	return ret;
}

static int ov7740_set_register(struct v4l2_subdev *sd,
					const struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	if (reg->reg & ~0xffff || reg->val & ~0xff)
		return -EINVAL;

	return ov7740_write_reg(reg->reg, reg->val);
}
#endif

static void ov7740_soft_reset(void)
{
	/* software reset */
	ov7740_write_reg(0x12, 0x80);

	/* delay at least 5ms */
	msleep(10);
}


/* calculate sysclk */
static int ov7740_get_sysclk(void)
{
	int xvclk = ov7740_data.mclk / 10000;
	int sysclk;
	int temp1, temp2;
	int Multiplier, PreDiv, VCO, SysDiv, Pll_rdiv, Bit_div2x, sclk_rdiv;
	int sclk_rdiv_map[] = {1, 2, 4, 8};
	u8 regval = 0;

	temp1 = ov7740_read_reg(0x3034, &regval);
	temp2 = temp1 & 0x0f;
	if (temp2 == 8 || temp2 == 10) {
		Bit_div2x = temp2 / 2;
	} else {
		pr_err("ov7740: unsupported bit mode %d\n", temp2);
		return -1;
	}

	temp1 = ov7740_read_reg(0x3035, &regval);
	SysDiv = temp1 >> 4;
	if (SysDiv == 0)
		SysDiv = 16;

	temp1 = ov7740_read_reg(0x3036, &regval);
	Multiplier = temp1;
	temp1 = ov7740_read_reg(0x3037, &regval);
	PreDiv = temp1 & 0x0f;
	Pll_rdiv = ((temp1 >> 4) & 0x01) + 1;

	temp1 = ov7740_read_reg(0x3108, &regval);
	temp2 = temp1 & 0x03;

	sclk_rdiv = sclk_rdiv_map[temp2];
	VCO = xvclk * Multiplier / PreDiv;
	sysclk = VCO / SysDiv / Pll_rdiv * 2 / Bit_div2x / sclk_rdiv;

	return sysclk;
}

/* read HTS from register settings */
static int ov7740_get_HTS(void)
{
	int HTS;
	u8 temp = 0;

	HTS = ov7740_read_reg(0x18, &temp);
	HTS = (HTS<<2) + ((ov7740_read_reg(0x16, &temp) & 0x18) >> 3);
	return HTS;
}

/* read VTS from register settings */
static int ov7740_get_VTS(void)
{
	int VTS;
	u8 temp = 0;

	VTS = ov7740_read_reg(0x1a, &temp);
	VTS = (VTS<<8) + ((ov7740_read_reg(0x16, &temp) & 0x20) >> 5);

	return VTS;
}

/* write VTS to registers */
static int ov7740_set_VTS(int VTS)
{
	int temp;
    u8 temp_reg;

	temp = VTS >> 1;
	ov7740_write_reg(0x1a, temp);


    ov7740_read_reg(0x16, &temp_reg);
	temp = (VTS & 0x01) << 5;
    temp_reg &= ~(0x20);
    temp_reg |= (temp & 0x20);
	ov7740_write_reg(0x16, temp_reg);
	return 0;
}

/* read shutter, in number of line period */
static int ov7740_get_shutter(void)
{
	int shutter;
	u8 regval;

	shutter = (u32)(ov7740_read_reg(0x10, &regval)) | (u32)(ov7740_read_reg(0x0f, &regval)<<8);

	return shutter;
}

/* write shutter, in number of line period */
static int ov7740_set_shutter(int shutter)
{
	int temp;

	shutter = shutter & 0xffff;
	temp = shutter & 0xff;
	ov7740_write_reg(0x10, temp);

	temp = shutter & 0xff00;
	temp = temp>>8;
	ov7740_write_reg(0x0f, temp);

	return 0;
}

/* read gain, 16 = 1x */
static int ov7740_get_gain16(void)
{
	int gain16;
	u8 regval;

	gain16 = ov7740_read_reg(0x350a, &regval) & 0x03;
	gain16 = (gain16<<8) + ov7740_read_reg(0x350b, &regval);

	return gain16;
}

/* write gain, 16 = 1x @todo: implement this for ov7740*/
static int ov7740_set_gain16(int gain16)
{
	int temp;

	return 0;
}

/* get banding filter value */
static int ov7740_get_light_freq(void)
{
	int temp, temp1, light_frequency;
	u8 regval;

	temp = ov7740_read_reg(0x3c01, &regval);
	if (temp & 0x80) {
		/* manual */
		temp1 = ov7740_read_reg(0x3c00, &regval);
		if (temp1 & 0x04) {
			/* 50Hz */
			light_frequency = 50;
		} else {
			/* 60Hz */
			light_frequency = 60;
		}
	} else {
		/* auto */
		temp1 = ov7740_read_reg(0x3c0c, &regval);
		if (temp1 & 0x01) {
			/* 50Hz */
			light_frequency = 50;
		} else {
			/* 60Hz */
			light_frequency = 60;
		}
	}

	return light_frequency;
}

static void ov7740_set_bandingfilter(void)
{
	int prev_VTS;
	int band_step60, max_band60, band_step50, max_band50;

	/* read preview PCLK */
	prev_sysclk = ov7740_get_sysclk();

	/* read preview HTS */
	prev_HTS = ov7740_get_HTS();

	/* read preview VTS */
	prev_VTS = ov7740_get_VTS();

	/* calculate banding filter */
	/* 60Hz */
	band_step60 = prev_sysclk * 100/prev_HTS * 100/120;
	ov7740_write_reg(0x3a0a, (band_step60 >> 8));
	ov7740_write_reg(0x3a0b, (band_step60 & 0xff));

	max_band60 = (int)((prev_VTS-4)/band_step60);
	ov7740_write_reg(0x3a0d, max_band60);

	/* 50Hz */
	band_step50 = prev_sysclk * 100/prev_HTS;
	ov7740_write_reg(0x3a08, (band_step50 >> 8));
	ov7740_write_reg(0x3a09, (band_step50 & 0xff));

	max_band50 = (int)((prev_VTS-4)/band_step50);
	ov7740_write_reg(0x3a0e, max_band50);
}

/* stable in high */
static int ov7740_set_AE_target(int target)
{
	int fast_high, fast_low;

	AE_low = target * 23 / 25; /* 0.92 */
	AE_high = target * 27 / 25; /* 1.08 */
	fast_high = AE_high << 1;

	if (fast_high > 255)
		fast_high = 255;
	fast_low = AE_low >> 1;

	ov7740_write_reg(0x3a0f, AE_high);
	ov7740_write_reg(0x3a10, AE_low);
	ov7740_write_reg(0x3a1b, AE_high);
	ov7740_write_reg(0x3a1e, AE_low);
	ov7740_write_reg(0x3a11, fast_high);
	ov7740_write_reg(0x3a1f, fast_low);

	return 0;
}

/* enable = 0 to turn off night mode
   enable = 1 to turn on night mode */
static int ov7740_set_night_mode(int enable)
{
	u8 mode;

	ov7740_read_reg(0x3a00, &mode);

	if (enable) {
		/* night mode on */
		mode |= 0x04;
		ov7740_write_reg(0x3a00, mode);
	} else {
		/* night mode off */
		mode &= 0xfb;
		ov7740_write_reg(0x3a00, mode);
	}

	return 0;
}

/* enable = 0 to turn off AEC/AGC
   enable = 1 to turn on AEC/AGC */
static void ov7740_turn_on_AE_AG(int enable)
{
	u8 ae_ag_ctrl;

	ov7740_read_reg(0x3503, &ae_ag_ctrl);
	if (enable) {
		/* turn on auto AE/AG */
		ae_ag_ctrl = ae_ag_ctrl & ~(0x03);
	} else {
		/* turn off AE/AG */
		ae_ag_ctrl = ae_ag_ctrl | 0x03;
	}
	ov7740_write_reg(0x3503, ae_ag_ctrl);
}

/* download ov7740 settings to sensor through i2c */
static int ov7740_download_firmware(struct reg_value *pModeSetting, s32 ArySize)
{
	register u32 Delay_ms = 0;
	register u16 RegAddr = 0;
	register u8 Mask = 0;
	register u8 Val = 0;
	u8 RegVal = 0;
	int i, retval = 0;

	for (i = 0; i < ArySize; ++i, ++pModeSetting) {
		Delay_ms = pModeSetting->u32Delay_ms;
		RegAddr = pModeSetting->u8RegAddr;
		Val = pModeSetting->u8Val;
		Mask = pModeSetting->u8Mask;

		if (Mask) {
			retval = ov7740_read_reg(RegAddr, &RegVal);
			if (retval < 0)
				goto err;

			RegVal &= ~(u8)Mask;
			Val &= Mask;
			Val |= RegVal;
		}

		retval = ov7740_write_reg(RegAddr, Val);
		if (retval < 0)
			goto err;

		if (Delay_ms)
			msleep(Delay_ms);
	}
	err:
	return retval;
}

static int ov7740_init_mode(void)
{
	struct reg_value *pModeSetting = NULL;
	int ArySize = 0, retval = 0;

	ov7740_soft_reset();

	pModeSetting = ov7740_setting_30fps_QVGA_320_240;
	ArySize = ARRAY_SIZE(ov7740_setting_30fps_QVGA_320_240);
	retval = ov7740_download_firmware(pModeSetting, ArySize);
	if (retval < 0)
		goto err;

	/* change driver capability to 2x according to validation board.
	 * if the image is not stable, please increase the driver strength.
	 */
	//ov7740_set_bandingfilter();
	//ov7740_set_AE_target(AE_Target);
	//ov7740_set_night_mode(night_mode);

	/* skip 9 vysnc: start capture at 10th vsync */
	msleep(300);

	/* turn off night mode */
	night_mode = 0;
	ov7740_data.pix.width = 640;
	ov7740_data.pix.height = 480;
	err:
	return retval;
}


/*!
 * ov7740_s_power - V4L2 sensor interface handler for VIDIOC_S_POWER ioctl
 * @s: pointer to standard V4L2 device structure
 * @on: indicates power mode (on or off)
 *
 * Turns the power on or off, depending on the value of on and returns the
 * appropriate error code.
 */
static int ov7740_s_power(struct v4l2_subdev *sd, int on)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov7740 *sensor = to_ov7740(client);

	if (on)
		clk_enable(ov7740_data.sensor_clk);
	else
		clk_disable(ov7740_data.sensor_clk);

	sensor->on = on;

	return 0;
}

/*!
 * ov7740_g_parm - V4L2 sensor interface handler for VIDIOC_G_PARM ioctl
 * @s: pointer to standard V4L2 sub device structure
 * @a: pointer to standard V4L2 VIDIOC_G_PARM ioctl structure
 *
 * Returns the sensor's video CAPTURE parameters.
 */
static int ov7740_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *a)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov7740 *sensor = to_ov7740(client);
	struct v4l2_captureparm *cparm = &a->parm.capture;
	int ret = 0;

	switch (a->type) {
		/* This is the only case currently handled. */
		case V4L2_BUF_TYPE_VIDEO_CAPTURE:
			memset(a, 0, sizeof(*a));
			a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
			cparm->capability = sensor->streamcap.capability;
			cparm->timeperframe = sensor->streamcap.timeperframe;
			cparm->capturemode = sensor->streamcap.capturemode;
			ret = 0;
			break;

			/* These are all the possible cases. */
		case V4L2_BUF_TYPE_VIDEO_OUTPUT:
		case V4L2_BUF_TYPE_VIDEO_OVERLAY:
		case V4L2_BUF_TYPE_VBI_CAPTURE:
		case V4L2_BUF_TYPE_VBI_OUTPUT:
		case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
		case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
			ret = -EINVAL;
			break;

		default:
			pr_debug("   type is unknown - %d\n", a->type);
			ret = -EINVAL;
			break;
	}

	return ret;
}

/*!
 * ov7740_s_parm - V4L2 sensor interface handler for VIDIOC_S_PARM ioctl
 * @s: pointer to standard V4L2 sub device structure
 * @a: pointer to standard V4L2 VIDIOC_S_PARM ioctl structure
 *
 * Configures the sensor to use the input parameters, if possible.  If
 * not possible, reverts to the old parameters and returns the
 * appropriate error code.
 */
static int ov7740_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *a)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov7740 *sensor = to_ov7740(client);
	struct v4l2_fract *timeperframe = &a->parm.capture.timeperframe;
	u32 tgt_fps;	/* target frames per secound */
	enum ov7740_frame_rate frame_rate;
	int ret = 0;

	switch (a->type) {
		/* This is the only case currently handled. */
		case V4L2_BUF_TYPE_VIDEO_CAPTURE:
			/* Check that the new frame rate is allowed. */
			if ((timeperframe->numerator == 0) ||
				(timeperframe->denominator == 0)) {
				timeperframe->denominator = DEFAULT_FPS;
				timeperframe->numerator = 1;
			}

			tgt_fps = timeperframe->denominator /
					  timeperframe->numerator;

			if (tgt_fps > MAX_FPS) {
				timeperframe->denominator = MAX_FPS;
				timeperframe->numerator = 1;
			} else if (tgt_fps < MIN_FPS) {
				timeperframe->denominator = MIN_FPS;
				timeperframe->numerator = 1;
			}

			/* Actual frame rate we use */
			tgt_fps = timeperframe->denominator /
					  timeperframe->numerator;

			if (tgt_fps == 15)
				frame_rate = ov7740_15_fps;
			else if (tgt_fps == 30)
				frame_rate = ov7740_30_fps;
			else {
				pr_err(" The camera frame rate is not supported!\n");
				goto error;
			}

			sensor->streamcap.timeperframe = *timeperframe;
			sensor->streamcap.capturemode = a->parm.capture.capturemode;

			break;

			/* These are all the possible cases. */
		case V4L2_BUF_TYPE_VIDEO_OUTPUT:
		case V4L2_BUF_TYPE_VIDEO_OVERLAY:
		case V4L2_BUF_TYPE_VBI_CAPTURE:
		case V4L2_BUF_TYPE_VBI_OUTPUT:
		case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
		case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
			pr_debug("   type is not " \
			"V4L2_BUF_TYPE_VIDEO_CAPTURE but %d\n",
					 a->type);
			ret = -EINVAL;
			break;

		default:
			pr_debug("   type is unknown - %d\n", a->type);
			ret = -EINVAL;
			break;
	}

	error:
	return ret;
}

static int ov7740_try_fmt(struct v4l2_subdev *sd,
						  struct v4l2_mbus_framefmt *mf)
{
	const struct ov7740_datafmt *fmt = ov7740_find_datafmt(mf->code);

	if (!fmt) {
		mf->code	= ov7740_colour_fmts[0].code;
		mf->colorspace	= ov7740_colour_fmts[0].colorspace;
	}

	mf->field	= V4L2_FIELD_NONE;

	return 0;
}

static int ov7740_s_fmt(struct v4l2_subdev *sd,
						struct v4l2_mbus_framefmt *mf)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov7740 *sensor = to_ov7740(client);

	/* MIPI CSI could have changed the format, double-check */
	if (!ov7740_find_datafmt(mf->code))
		return -EINVAL;

	ov7740_try_fmt(sd, mf);
	sensor->fmt = ov7740_find_datafmt(mf->code);

	return 0;
}

static int ov7740_g_fmt(struct v4l2_subdev *sd,
						struct v4l2_mbus_framefmt *mf)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov7740 *sensor = to_ov7740(client);

	const struct ov7740_datafmt *fmt = sensor->fmt;

	mf->code	= fmt->code;
	mf->colorspace	= fmt->colorspace;
	mf->field	= V4L2_FIELD_NONE;

	return 0;
}

static int ov7740_enum_fmt(struct v4l2_subdev *sd, unsigned int index,
						   u32 *code)
{
	if (index >= ARRAY_SIZE(ov7740_colour_fmts))
		return -EINVAL;

	*code = ov7740_colour_fmts[index].code;
	return 0;
}

/*!
 * ov7740_enum_framesizes - V4L2 sensor interface handler for
 *			   VIDIOC_ENUM_FRAMESIZES ioctl
 * @s: pointer to standard V4L2 device structure
 * @fsize: standard V4L2 VIDIOC_ENUM_FRAMESIZES ioctl structure
 *
 * Return 0 if successful, otherwise -EINVAL.
 */
static int ov7740_enum_framesizes(struct v4l2_subdev *sd,
								  struct v4l2_subdev_pad_config *cfg,
								  struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index > ov7740_mode_MAX)
		return -EINVAL;

	fse->max_width =
			max(ov7740_mode_info_data[0][fse->index].width,
				ov7740_mode_info_data[1][fse->index].width);
	fse->min_width = fse->max_width;
	fse->max_height =
			max(ov7740_mode_info_data[0][fse->index].height,
				ov7740_mode_info_data[1][fse->index].height);
	fse->min_height = fse->max_height;
	return 0;
}

/*!
 * ov7740_enum_frameintervals - V4L2 sensor interface handler for
 *			       VIDIOC_ENUM_FRAMEINTERVALS ioctl
 * @s: pointer to standard V4L2 device structure
 * @fival: standard V4L2 VIDIOC_ENUM_FRAMEINTERVALS ioctl structure
 *
 * Return 0 if successful, otherwise -EINVAL.
 */
static int ov7740_enum_frameintervals(struct v4l2_subdev *sd,
									  struct v4l2_subdev_pad_config *cfg,
									  struct v4l2_subdev_frame_interval_enum *fie)
{
	int i, j, count;

	if (fie->index < 0 || fie->index > ov7740_mode_MAX)
		return -EINVAL;

	if (fie->width == 0 || fie->height == 0 ||
		fie->code == 0) {
		pr_warning("Please assign pixel format, width and height.\n");
		return -EINVAL;
	}

	fie->interval.numerator = 1;

	count = 0;
	for (i = 0; i < ARRAY_SIZE(ov7740_mode_info_data); i++) {
		for (j = 0; j < (ov7740_mode_MAX + 1); j++) {
			if (fie->width == ov7740_mode_info_data[i][j].width
				&& fie->height == ov7740_mode_info_data[i][j].height
				&& ov7740_mode_info_data[i][j].init_data_ptr != NULL) {
				count++;
			}
			if (fie->index == (count - 1)) {
				fie->interval.denominator =
						ov7740_framerates[i];
				return 0;
			}
		}
	}

	return -EINVAL;
}

static int ov7740_set_clk_rate(void)
{
	u32 tgt_xclk;	/* target xclk */
	int ret;

	/* mclk */
	tgt_xclk = ov7740_data.mclk;
	tgt_xclk = min(tgt_xclk, (u32)ov7740_XCLK_MAX);
	tgt_xclk = max(tgt_xclk, (u32)ov7740_XCLK_MIN);
	ov7740_data.mclk = tgt_xclk;

	pr_debug("   Setting mclk to %d MHz\n", tgt_xclk / 1000000);
	ret = clk_set_rate(ov7740_data.sensor_clk, ov7740_data.mclk);
	if (ret < 0)
		pr_debug("set rate filed, rate=%d\n", ov7740_data.mclk);
	return ret;
}

/*!
 * dev_init - V4L2 sensor init
 * @s: pointer to standard V4L2 device structure
 *
 */
static int init_device(void)
{
	u32 tgt_xclk;	/* target xclk */
	u32 tgt_fps;	/* target frames per secound */
	enum ov7740_frame_rate frame_rate;
	int ret;

	ov7740_data.on = true;

	/* mclk */
	tgt_xclk = ov7740_data.mclk;

	/* Default camera frame rate is set in probe */
	tgt_fps = ov7740_data.streamcap.timeperframe.denominator /
			  ov7740_data.streamcap.timeperframe.numerator;

	if (tgt_fps == 15)
		frame_rate = ov7740_15_fps;
	else if (tgt_fps == 30)
		frame_rate = ov7740_30_fps;
	else
		return -EINVAL; /* Only support 15fps or 30fps now. */

	ret = ov7740_init_mode();

	return ret;
}

static struct v4l2_subdev_video_ops ov7740_subdev_video_ops = {
		.g_parm = ov7740_g_parm,
		.s_parm = ov7740_s_parm,

		.s_mbus_fmt	= ov7740_s_fmt,
		.g_mbus_fmt	= ov7740_g_fmt,
		.try_mbus_fmt	= ov7740_try_fmt,
		.enum_mbus_fmt	= ov7740_enum_fmt,
};

static const struct v4l2_subdev_pad_ops ov7740_subdev_pad_ops = {
		.enum_frame_size       = ov7740_enum_framesizes,
		.enum_frame_interval   = ov7740_enum_frameintervals,
};

static struct v4l2_subdev_core_ops ov7740_subdev_core_ops = {
		.s_power	= ov7740_s_power,
#ifdef CONFIG_VIDEO_ADV_DEBUG
.g_register	= ov7740_get_register,
	.s_register	= ov7740_set_register,
#endif
};

static struct v4l2_subdev_ops ov7740_subdev_ops = {
		.core	= &ov7740_subdev_core_ops,
		.video	= &ov7740_subdev_video_ops,
		.pad	= &ov7740_subdev_pad_ops,
};

/*!
 * ov7740 I2C probe function
 *
 * @param adapter            struct i2c_adapter *
 * @return  Error code indicating success or failure
 */
static int ov7740_probe(struct i2c_client *client,
						const struct i2c_device_id *id)
{
	struct pinctrl *pinctrl;
	struct device *dev = &client->dev;
	int retval;
	u8 chip_id_high, chip_id_low;

	/* ov7740 pinctrl */
	pinctrl = devm_pinctrl_get_select_default(dev);
	if (IS_ERR(pinctrl)) {
		dev_err(dev, "setup pinctrl failed\n");
		return PTR_ERR(pinctrl);
	}

	/* request power down pin */
	pwn_gpio = of_get_named_gpio(dev->of_node, "pwn-gpios", 0);
	if (!gpio_is_valid(pwn_gpio)) {
		dev_err(dev, "no sensor pwdn pin available\n");
		return -ENODEV;
	}
	retval = devm_gpio_request_one(dev, pwn_gpio, GPIOF_OUT_INIT_HIGH,
								   "ov7740_pwdn");
	if (retval < 0)
		return retval;

	/* request reset pin */
	rst_gpio = of_get_named_gpio(dev->of_node, "rst-gpios", 0);
	if (!gpio_is_valid(rst_gpio)) {
		dev_err(dev, "no sensor reset pin available\n");
		return -EINVAL;
	}
	retval = devm_gpio_request_one(dev, rst_gpio, GPIOF_OUT_INIT_HIGH,
								   "ov7740_reset");
	if (retval < 0)
		return retval;

	/* Set initial values for the sensor struct. */
	memset(&ov7740_data, 0, sizeof(ov7740_data));
	ov7740_data.sensor_clk = devm_clk_get(dev, "csi_mclk");
	if (IS_ERR(ov7740_data.sensor_clk)) {
		dev_err(dev, "get mclk failed\n");
		return PTR_ERR(ov7740_data.sensor_clk);
	}

	retval = of_property_read_u32(dev->of_node, "mclk",
								  &ov7740_data.mclk);
	if (retval) {
		dev_err(dev, "mclk frequency is invalid\n");
		return retval;
	}

	retval = of_property_read_u32(dev->of_node, "mclk_source",
								  (u32 *) &(ov7740_data.mclk_source));
	if (retval) {
		dev_err(dev, "mclk_source invalid\n");
		return retval;
	}

	retval = of_property_read_u32(dev->of_node, "csi_id",
								  &(ov7740_data.csi));
	if (retval) {
		dev_err(dev, "csi_id invalid\n");
		return retval;
	}

	/* Set mclk rate before clk on */
	ov7740_set_clk_rate();

	clk_prepare_enable(ov7740_data.sensor_clk);

	ov7740_data.io_init = ov7740_reset;
	ov7740_data.i2c_client = client;
	ov7740_data.pix.pixelformat = V4L2_PIX_FMT_YUYV;
	ov7740_data.pix.width = 640;
	ov7740_data.pix.height = 480;
	ov7740_data.streamcap.capability = V4L2_MODE_HIGHQUALITY |
									   V4L2_CAP_TIMEPERFRAME;
	ov7740_data.streamcap.capturemode = 0;
	ov7740_data.streamcap.timeperframe.denominator = DEFAULT_FPS;
	ov7740_data.streamcap.timeperframe.numerator = 1;

	ov7740_regulator_enable(&client->dev);

	ov7740_reset();

	ov7740_power_down(0);

	retval = ov7740_read_reg(OV7740_CHIP_ID_HIGH_BYTE, &chip_id_high);
	if (retval < 0 || chip_id_high != OV7740_CHIP_ID_HIGH_BYTE_READ) {
		clk_disable_unprepare(ov7740_data.sensor_clk);
		pr_warning("camera ov7740 is not found_hb\n");
        pr_warning("%i expected but got %i",OV7740_CHIP_ID_HIGH_BYTE_READ,retval);
		return -ENODEV;
	}
	retval = ov7740_read_reg(OV7740_CHIP_ID_LOW_BYTE, &chip_id_low);
	if (retval < 0 || chip_id_low != OV7740_CHIP_ID_LOW_BYTE_READ) {
		clk_disable_unprepare(ov7740_data.sensor_clk);
		pr_warning("camera ov7740 is not found_lb\n");
        pr_warning("%i expected but got %i",OV7740_CHIP_ID_LOW_BYTE_READ,retval);
		return -ENODEV;
	}

	retval = init_device();
	if (retval < 0) {
		clk_disable_unprepare(ov7740_data.sensor_clk);
		pr_warning("camera ov7740 init failed\n");
		ov7740_power_down(1);
		return retval;
	}

	clk_disable(ov7740_data.sensor_clk);

	v4l2_i2c_subdev_init(&ov7740_data.subdev, client, &ov7740_subdev_ops);

	retval = v4l2_async_register_subdev(&ov7740_data.subdev);
	if (retval < 0)
		dev_err(&client->dev,
				"%s--Async register failed, ret=%d\n", __func__, retval);

	pr_info("camera ov7740, is found\n");
	return retval;
}

/*!
 * ov7740 I2C detach function
 *
 * @param client            struct i2c_client *
 * @return  Error code indicating success or failure
 */
static int ov7740_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);

	v4l2_async_unregister_subdev(sd);

	clk_unprepare(ov7740_data.sensor_clk);

	ov7740_power_down(1);

	if (analog_regulator)
		regulator_disable(analog_regulator);

	if (core_regulator)
		regulator_disable(core_regulator);

	if (io_regulator)
		regulator_disable(io_regulator);

	return 0;
}

module_i2c_driver(ov7740_i2c_driver);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("ov7740 Camera Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
MODULE_ALIAS("CSI");
