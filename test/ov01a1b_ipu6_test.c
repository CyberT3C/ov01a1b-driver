// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2020-2022 Intel Corporation.
// Copyright (c) 2025 Niklas Bartz
//
// OV01A1B camera driver
// Based on ov01a10 driver by Intel Corporation
// OV01A1B Power Test Module - Based on OV01A10 power sequence
//



#include <linux/version.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/acpi.h>
#include <linux/delay.h>
#include <linux/pm_runtime.h>

#include <linux/regulator/consumer.h>
#include <linux/gpio/consumer.h>
#include <linux/clk.h>
#include <linux/unaligned.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>   


#define DRIVER_NAME "ov01a1b"

#define CHIP_ID                         0x560141
#define REG_CHIP_ID		        0x300a

#define OV01A1B_LINK_FREQ_400MHZ	400000000ULL
#define OV01A1B_SCLK			40000000LL
//#define OV01A1B_MCLK			19200000
//#define OV01A1B_DATA_LANES		1
//#define OV01A1B_IR_DEPTH		10

#define OV01A1B_REG_MODE_SELECT         0x0100
#define OV01A1B_MODE_STANDBY            0x00
#define OV01A1B_MODE_STREAMING          0x01

/* vertical-timings from sensor */
#define OV01A1B_REG_VTS			0x380e
#define OV01A1B_VTS_DEF_640		0x0200
#define OV01A1B_VTS_MIN			0x0200
#define OV01A1B_VTS_MAX			0xffff


/* Common OmniVision I2C addresses to try */

/*
Power-On:                     Power-Off:
1. Regulators ON       →      4. Regulators OFF
2. Clock ON           →      3. Clock OFF  
3. Powerdown = 0      →      2. Powerdown = 1
4. Reset 1→0          →      1. Reset = 1
5. Wait for boot      →      nop 
*/

/* seuqnce for v4l2 
ov01a1b_probe()
    ├── Hardware init (GPIOs, clocks, regulators)
    ├── Power on
    ├── Check chip ID
    ├── v4l2_i2c_subdev_init()
    ├── Initialize controls (v4l2_ctrl_handler)
    ├── Media entity init
    ├── v4l2_async_register_subdev()
    └── Power off (optional)
*/

struct ov01a1b_reg {
	u16 address;
	u8 val;
};

struct ov01a1b_reg_list {
	u32 num_of_regs;
	const struct ov01a1b_reg *regs;
};

struct ov01a1b_link_freq_config {
	const struct ov01a1b_reg_list reg_list;
};


struct ov01a1b_mode {
	/* Frame width in pixels */
	u32 width;

	/* Frame height in pixels */
	u32 height;

	/* Horizontal timining size */
	u32 hts;

	/* Default vertical timining size */
	u32 vts_def;

	/* Min vertical timining size */
	u32 vts_min;

	/* Link frequency needed for this resolution */
	u32 link_freq_index;

	/* Sensor register settings for this resolution */
	const struct ov01a1b_reg_list reg_list;
};


static const struct ov01a1b_reg sensor_ir_init[] = {
	{0x0103, 0x01},
	{0x0302, 0x00},
	{0x0303, 0x06},
	{0x0304, 0x01},
	{0x0305, 0x90},
	{0x0306, 0x00},
	{0x0308, 0x01},
	{0x0309, 0x00},
	{0x030c, 0x01},
	{0x0322, 0x01},
	{0x0323, 0x06},
	{0x0324, 0x01},
	{0x0325, 0x68},
	{0x3002, 0xa1},
	{0x301e, 0xf0},
	{0x3022, 0x01},
	{0x3501, 0x03},
	{0x3502, 0x78},
	{0x3504, 0x0c},
	{0x3508, 0x01},
	{0x3509, 0x00},
	{0x3601, 0xc0},
	{0x3603, 0x71},
	{0x3610, 0x68},
	{0x3611, 0x86},
	{0x3640, 0x10},
	{0x3641, 0x80},
	{0x3642, 0xdc},
	{0x3646, 0x55},
	{0x3647, 0x57},
	{0x364b, 0x00},
	{0x3653, 0x10},
	{0x3655, 0x00},
	{0x3656, 0x00},
	{0x365f, 0x0f},
	{0x3661, 0x45},
	{0x3662, 0x24},
	{0x3663, 0x11},
	{0x3664, 0x07},
	{0x3709, 0x34},
	{0x370b, 0x6f},
	{0x3714, 0x22},
	{0x371b, 0x27},
	{0x371c, 0x67},
	{0x371d, 0xa7},
	{0x371e, 0xe7},
	{0x3730, 0x81},
	{0x3733, 0x10},
	{0x3734, 0x40},
	{0x3737, 0x04},
	{0x3739, 0x1c},
	{0x3767, 0x00},
	{0x376c, 0x81},
	{0x3772, 0x14},
	{0x37c2, 0x04},
	{0x37d8, 0x03},
	{0x37d9, 0x0c},
	{0x37e0, 0x00},
	{0x37e1, 0x08},
	{0x37e2, 0x10},
	{0x37e3, 0x04},
	{0x37e4, 0x04},
	{0x37e5, 0x03},
	{0x37e6, 0x04},
	{0x3b00, 0x00},
	{0x3c80, 0x00},
	{0x3c88, 0x02},
	{0x3c8c, 0x07},
	{0x3c8d, 0x40},
	{0x3cc7, 0x80},
	{0x4000, 0xc3},
	{0x4001, 0xe0},
	{0x4003, 0x40},
	{0x4008, 0x02},
	{0x4009, 0x19},
	{0x400a, 0x01},
	{0x400b, 0x6c},
	{0x4011, 0x00},
	{0x4041, 0x00},
	{0x4300, 0xff},  /* IR/Monochrome mode */
	{0x4301, 0x00},
	{0x4302, 0x0f},
	{0x4503, 0x00},
	{0x4601, 0x50},
	{0x481f, 0x34},
	{0x4825, 0x33},
	{0x4837, 0x14},
	{0x4881, 0x40},
	{0x4883, 0x01},
	{0x4890, 0x00},
	{0x4901, 0x00},
	{0x4902, 0x00},
	{0x4b00, 0x2a},
	{0x4b0d, 0x00},
	{0x450a, 0x04},
	{0x450b, 0x00},
	{0x5000, 0x65},
	{0x5004, 0x00},
	{0x5080, 0x40},
	{0x5200, 0x18},
};

/* Dell-specified IR resolution 640x400 @ 30fps */
static const struct ov01a1b_reg sensor_640x400_ir_setting[] = {
	/* Window settings */
	{0x3800, 0x00},
	{0x3801, 0x00},
	{0x3802, 0x00},
	{0x3803, 0x00},
	{0x3804, 0x05},
	{0x3805, 0x0f},
	{0x3806, 0x03},
	{0x3807, 0x2f},
	{0x3808, 0x02},  /* H_OUTPUT_SIZE = 640 */
	{0x3809, 0x80},
	{0x380a, 0x01},  /* V_OUTPUT_SIZE = 400 */
	{0x380b, 0x90},
	{0x380c, 0x03},  /* HTS = 768 */
	{0x380d, 0x00},
	{0x380e, 0x02},  /* VTS = 512 for ~30fps */
	{0x380f, 0x00},
	{0x3810, 0x00},
	{0x3811, 0x04},
	{0x3812, 0x00},
	{0x3813, 0x04},
	{0x3814, 0x03},  /* H_INC = 3 (binning) */
	{0x3815, 0x03},  /* V_INC = 3 (binning) */
	{0x3816, 0x03},
	{0x3817, 0x03},
	{0x3820, 0xa0},  /* Format control with IR mode bits */
	{0x3822, 0x03},
	{0x3832, 0x28},
	{0x3833, 0x10},
	{0x373d, 0x24},
	{0x4300, 0xff},  /* Ensure IR mode */
};

/* Native resolution 1280x800 for calibration/alignment */
static const struct ov01a1b_reg sensor_1280x800_ir_setting[] = {
	{0x3800, 0x00},
	{0x3801, 0x00},
	{0x3802, 0x00},
	{0x3803, 0x00},
	{0x3804, 0x05},
	{0x3805, 0x0f},
	{0x3806, 0x03},
	{0x3807, 0x2f},
	{0x3808, 0x05},  /* H_OUTPUT_SIZE = 1280 */
	{0x3809, 0x00},
	{0x380a, 0x03},  /* V_OUTPUT_SIZE = 800 */
	{0x380b, 0x20},
	{0x380c, 0x02},  /* HTS = 744 */
	{0x380d, 0xe8},
	{0x380e, 0x03},  /* VTS = 896 */
	{0x380f, 0x80},
	{0x3810, 0x00},
	{0x3811, 0x08},
	{0x3812, 0x00},
	{0x3813, 0x08},
	{0x3814, 0x01},  /* No binning */
	{0x3815, 0x01},
	{0x3816, 0x01},
	{0x3817, 0x01},
	{0x3820, 0xa0},
	{0x3822, 0x03},
	{0x3832, 0x28},
	{0x3833, 0x10},
	{0x373d, 0x24},
	{0x4300, 0xff},  /* IR mode */
};





enum {
	OV01A1B_LINK_FREQ_400MHZ_INDEX,
};


static const struct ov01a1b_reg mipi_data_rate_720mbps[] = {
};  
  
static const s64 link_freq_menu_items[] = {
	OV01A1B_LINK_FREQ_400MHZ,
};

  static const struct ov01a1b_link_freq_config link_freq_configs[] = {
    [OV01A1B_LINK_FREQ_400MHZ_INDEX] = {
        .reg_list = {
            .num_of_regs = ARRAY_SIZE(mipi_data_rate_720mbps),
            .regs = mipi_data_rate_720mbps,
        }
    },
};



struct ov01a1b {
    // v4l2 layer
    struct v4l2_subdev sd;
    struct v4l2_ctrl_handler ctrl_handler;    
    // controls
    struct v4l2_ctrl *link_freq;
    struct v4l2_ctrl *pixel_rate;
    struct v4l2_ctrl *vblank;
    struct v4l2_ctrl *hblank;
    struct v4l2_ctrl *exposure;


    // media
    const struct ov01a1b_mode *cur_mode;
    bool streaming;
    
    // async calls
    struct mutex mutex;

    // hardware
    struct clk *xvclk;
    struct gpio_desc *reset_gpio;
    struct gpio_desc *powerdown_gpio;
    struct regulator_bulk_data supplies[3];
    u32 xvclk_freq;
};

static inline struct ov01a1b *to_ov01a1b(struct v4l2_subdev *subdev)
{
	return container_of(subdev, struct ov01a1b, sd);
}






static const struct ov01a1b_mode supported_modes[] = {
	{
		/* Primary mode: Dell-specified 640x400 @ 30fps for Windows Hello */
		.width = 640,
		.height = 400,
		.hts = 768,
		.vts_def = OV01A1B_VTS_DEF_640,
		.vts_min = OV01A1B_VTS_MIN,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(sensor_640x400_ir_setting),
			.regs = sensor_640x400_ir_setting,
		},
		.link_freq_index = OV01A1B_LINK_FREQ_400MHZ_INDEX,
	},
	{
		/* Native resolution for calibration/alignment */
		.width = 1280,
		.height = 800,
		.hts = 744,
		.vts_def = 896,
		.vts_min = 896,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(sensor_1280x800_ir_setting),
			.regs = sensor_1280x800_ir_setting,
		},
		.link_freq_index = OV01A1B_LINK_FREQ_400MHZ_INDEX,
	},
};

 


static int ov01a1b_write_register(struct ov01a1b *ov01a1b, u16 reg, u16 len, u32 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&ov01a1b->sd);
	u8 buf[6];
	int ret = 0;

	if (len > 4)
		return -EINVAL;

	put_unaligned_be16(reg, buf);
	put_unaligned_be32(val << 8 * (4 - len), buf + 2);

	ret = i2c_master_send(client, buf, len + 2);
	if (ret != len + 2)
		return ret < 0 ? ret : -EIO;

	return 0;

}

static int ov01a1b_write_register_list(struct ov01a1b *ov01a1b,
				  const struct ov01a1b_reg_list *r_list)
{
	struct i2c_client *client = v4l2_get_subdevdata(&ov01a1b->sd);
	unsigned int i;
	int ret = 0;

	for (i = 0; i < r_list->num_of_regs; i++) {
		ret = ov01a1b_write_register(ov01a1b, r_list->regs[i].address, 1,
					r_list->regs[i].val);
		if (ret) {
			dev_err_ratelimited(&client->dev,
					    "write reg 0x%4.4x return err = %d",
					    r_list->regs[i].address, ret);
			return ret;
		}
	}

	return 0;
}

static int ov01a1b_read_register(struct ov01a1b *ov01a1b, u16 reg, u16 len, u32 *val)
{
	if (len > 4)
	    return -EINVAL;

	struct i2c_client *client = v4l2_get_subdevdata(&ov01a1b->sd);
	struct i2c_msg msgs[2];
	u8 addr_buffer[2];
	u8 data_buffer[4] = {};
	int ret = 0;


	put_unaligned_be16(reg, addr_buffer);
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = sizeof(addr_buffer);
	msgs[0].buf = addr_buffer;
	msgs[1].addr = client->addr; 
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	// msgs[1].buf = &data_buf;
	msgs[1].buf = &data_buffer[4 - len];
        //msgs[1].buf = data_buffer;
        

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return ret < 0 ? ret : -EIO;

        
        dev_info(&client->dev, "debug output before get unaligned be32");
	*val = get_unaligned_be32(data_buffer);
        
       // switch(len) {
       // case 1: *val = data_buffer[0]; break;
       // case 2: *val = get_unaligned_be16(data_buffer); break;
       // case 3: *val = (data_buffer[0] << 16) | (data_buffer[1] << 8) | data_buffer[2]; break;
       // case 4: *val = get_unaligned_be32(data_buffer); break;
       // }    
    
    
    	return 0;
    }
    
    
    
static int read_chip_id(struct ov01a1b *ov01a1b, u32 *val)
{
    struct i2c_client *client = v4l2_get_subdevdata(&ov01a1b->sd);
    struct device *dev = &client->dev;
    
    return ov01a1b_read_register(ov01a1b, REG_CHIP_ID, 3, val);
}

static int ov01a1b_check_i2c_address(struct ov01a1b *ov01a1b)
{
    struct i2c_client *client = v4l2_get_subdevdata(&ov01a1b->sd);
    struct device *dev = &client->dev;
    u32 id_value;
    int ret;

    ret = read_chip_id(ov01a1b, &id_value);
    if(ret){
        return ret;
    }

    if (id_value == CHIP_ID) {
    	dev_info(dev, "OV01A1B IR camera detected (chip id: 0x%06x)\n", id_value);
        return 0; 
    }
    
    return -EIO;
}


static int ov01a1b_power_off_sequenz(struct ov01a1b *ov01a1b)
{

    struct i2c_client *client = v4l2_get_subdevdata(&ov01a1b->sd);
    struct device *dev = &client->dev;

    dev_info(dev, "startign power off sequence...\n");
    
    /* Step 1: Assert reset (put sensor in reset) */
    if (ov01a1b->reset_gpio)
        gpiod_set_value_cansleep(ov01a1b->reset_gpio, 1);

    /* Small delay to ensure reset is registered */
    usleep_range(1000, 1500);
    
    /* Step 2: Assert powerdown */
    if (ov01a1b->powerdown_gpio)
        gpiod_set_value_cansleep(ov01a1b->powerdown_gpio, 1);
    
    /* Step 3: Disable clock */
    clk_disable_unprepare(ov01a1b->xvclk);
    
    /* Step 4: Disable regulators (reverse order!) */
    regulator_bulk_disable(ARRAY_SIZE(ov01a1b->supplies),
                          ov01a1b->supplies);

    // give the chip time to stop
    msleep(50);

    if(ov01a1b_check_i2c_address(ov01a1b) == 0) {
        dev_err(dev, "ERROR: Sensor still responding after power off!\n");
        return -ENODEV;
    }
    return 0;
}


static int ov01a1b_power_on_sequence(struct ov01a1b *ov01a1b)
{
    struct i2c_client *client = v4l2_get_subdevdata(&ov01a1b->sd);
    struct device *dev = &client->dev;
    int ret;
    
    dev_info(dev, "Starting OV01A10-style power sequence...\n");
    
    /* Step 1: Enable regulators (if available) */
    if (ov01a1b->supplies[0].consumer) {
        dev_info(dev, "Enabling regulators...\n");
        ret = regulator_bulk_enable(ARRAY_SIZE(ov01a1b->supplies), 
                                    ov01a1b->supplies);
        if (ret < 0) {
            dev_err(dev, "Failed to enable regulators: %d\n", ret);
            /* Continue anyway, might not be required */
        } else {
            /* Wait for voltages to stabilize */
            usleep_range(1000, 1500);
        }
    }
    
    /* Step 2: Enable clock (if available) */
    if (ov01a1b->xvclk) {
        dev_info(dev, "Enabling XVCLK at %u Hz...\n", ov01a1b->xvclk_freq);
        
        ret = clk_set_rate(ov01a1b->xvclk, ov01a1b->xvclk_freq);
        if (ret < 0) {
            dev_warn(dev, "Failed to set xvclk rate: %d\n", ret);
        }
        
        ret = clk_prepare_enable(ov01a1b->xvclk);
        if (ret < 0) {
            dev_err(dev, "Failed to enable xvclk: %d\n", ret);
            /* Continue anyway */
        } else {
            /* Wait for clock to stabilize */
            usleep_range(1000, 1500);
        }
    } else {
        dev_warn(dev, "No clock found - sensor might need external clock!\n");
    }
    
    /* Step 3: Release powerdown (if available) */
    if (ov01a1b->powerdown_gpio) {
        dev_info(dev, "Releasing powerdown...\n");
        gpiod_set_value_cansleep(ov01a1b->powerdown_gpio, 0);
        usleep_range(1000, 1500);
    }
    
    /* Step 4: Toggle reset (if available) */
    if (ov01a1b->reset_gpio) {
        dev_info(dev, "Toggling reset...\n");
        /* Assert reset */
        gpiod_set_value_cansleep(ov01a1b->reset_gpio, 1);
        usleep_range(1000, 1500);
        /* Deassert reset */
        gpiod_set_value_cansleep(ov01a1b->reset_gpio, 0);
        /* Wait for sensor to boot - this is critical! */
        msleep(20);
    }
    
    /* Step 5: Additional delay for sensor initialization */
    dev_info(dev, "Waiting for sensor initialization...\n");
    msleep(10);
    
    /* Step 6: test address */
    dev_info(dev, "Probing I2C address...\n");
    ret = ov01a1b_check_i2c_address(ov01a1b);
    if (ret) {
        return ret;
    }
    
    return 0;

}

///////////////////////////////////////////////
/////////////////////////////////////////////
///
///
static void ov01a1b_update_pad_format(const struct ov01a1b_mode *mode,
                                      struct v4l2_mbus_framefmt *fmt)
{
        fmt->width = mode->width;
        fmt->height = mode->height;
        /* IR sensor outputs monochrome data */
        fmt->code = MEDIA_BUS_FMT_Y10_1X10;    
        fmt->field = V4L2_FIELD_NONE;         
}




static int ov01a1b_set_format(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *sd_state,
			      struct v4l2_subdev_format *fmt)
{
	struct ov01a1b *ov01a1b = to_ov01a1b(sd);
	const struct ov01a1b_mode *mode;
	s32 vblank_def, h_blank;

	mode = v4l2_find_nearest_size(supported_modes,
				      ARRAY_SIZE(supported_modes), width,
				      height, fmt->format.width,
				      fmt->format.height);

	mutex_lock(&ov01a1b->mutex);
	ov01a1b_update_pad_format(mode, &fmt->format);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		*v4l2_subdev_state_get_format(sd_state, fmt->pad) = fmt->format;
	} else {
		ov01a1b->cur_mode = mode;
		__v4l2_ctrl_s_ctrl(ov01a1b->link_freq, mode->link_freq_index);
		__v4l2_ctrl_s_ctrl_int64(ov01a1b->pixel_rate, OV01A1B_SCLK);

		/* Update limits and set FPS to default */
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(ov01a1b->vblank,
					 mode->vts_min - mode->height,
					 OV01A1B_VTS_MAX - mode->height, 1,
					 vblank_def);
		__v4l2_ctrl_s_ctrl(ov01a1b->vblank, vblank_def);
		h_blank = mode->hts - mode->width;
		__v4l2_ctrl_modify_range(ov01a1b->hblank, h_blank, h_blank, 1,
					 h_blank);
	}
	mutex_unlock(&ov01a1b->mutex);

	return 0;
}

static int ov01a1b_get_format(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *sd_state,
			      struct v4l2_subdev_format *fmt)
{
	struct ov01a1b *ov01a1b = to_ov01a1b(sd);

	mutex_lock(&ov01a1b->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
		fmt->format = *v4l2_subdev_state_get_format(
							  sd_state, fmt->pad);
	else
		ov01a1b_update_pad_format(ov01a1b->cur_mode, &fmt->format);

	mutex_unlock(&ov01a1b->mutex);

	return 0;
}

static int ov01a1b_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index > 0)
		return -EINVAL;

	/* IR sensor outputs monochrome data */
	code->code = MEDIA_BUS_FMT_Y10_1X10;

	return 0;
}


static int ov01a1b_enum_frame_size(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *sd_state,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	if (fse->code != MEDIA_BUS_FMT_Y10_1X10)
		return -EINVAL;

	fse->min_width = supported_modes[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height = supported_modes[fse->index].height;
	fse->max_height = fse->min_height;

	return 0;
}




static int ov01a1b_stop_streaming(struct ov01a1b *ov01a1b)
{
    struct i2c_client *client = v4l2_get_subdevdata(&ov01a1b->sd);
    int ret = 0;
    
    ret = ov01a1b_write_register(ov01a1b, OV01A1B_REG_MODE_SELECT, 1, OV01A1B_MODE_STANDBY);
    if (ret)
    	dev_err(&client->dev, "failed to stop streaming");
    return ret;
}


static int ov01a1b_start_streaming(struct ov01a1b *ov01a1b)
{
        struct i2c_client *client = v4l2_get_subdevdata(&ov01a1b->sd);
	const struct ov01a1b_reg_list *reg_list;
	struct ov01a1b_reg_list init_list = {
		.num_of_regs = ARRAY_SIZE(sensor_ir_init),
		.regs = sensor_ir_init,
	};
	int link_freq_index;
	int ret = 0;

	/* Apply common IR initialization */
	ret = ov01a1b_write_register_list(ov01a1b, &init_list);
	if (ret) {
		dev_err(&client->dev, "failed to set IR init registers");
		return ret;
	}

	link_freq_index = ov01a1b->cur_mode->link_freq_index;
	reg_list = &link_freq_configs[link_freq_index].reg_list;
	ret = ov01a1b_write_register_list(ov01a1b, reg_list);
	if (ret) {
		dev_err(&client->dev, "failed to set plls");
		return ret;
	}

	/* Apply mode specific settings */
	reg_list = &ov01a1b->cur_mode->reg_list;
	ret = ov01a1b_write_register_list(ov01a1b, reg_list);
	if (ret) {
		dev_err(&client->dev, "failed to set mode");
		return ret;
	}

	ret = __v4l2_ctrl_handler_setup(ov01a1b->sd.ctrl_handler);
	if (ret)
		return ret;

	ret = ov01a1b_write_register(ov01a1b, OV01A1B_REG_MODE_SELECT, 1,
				OV01A1B_MODE_STREAMING);
	if (ret)
		dev_err(&client->dev, "failed to start streaming");

	return ret;
}


static int ov01a1b_set_stream(struct v4l2_subdev *sd, int enable)
{
        struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov01a1b *ov01a1b = to_ov01a1b(sd);
	int ret = 0;

	if (ov01a1b->streaming == enable)
		return 0;

	mutex_lock(&ov01a1b->mutex);
	if (enable) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			mutex_unlock(&ov01a1b->mutex);
			return ret;
		}

		ret = ov01a1b_start_streaming(ov01a1b);
		if (ret) {
			enable = 0;
			ov01a1b_stop_streaming(ov01a1b);
			pm_runtime_put(&client->dev);
		}
	} else {
		ov01a1b_stop_streaming(ov01a1b);
		pm_runtime_put(&client->dev);
	}

	ov01a1b->streaming = enable;
	mutex_unlock(&ov01a1b->mutex);

	return ret;
}

static int ov01a1b_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov01a1b *ov01a1b = to_ov01a1b(sd);
	int ret;

	mutex_lock(&ov01a1b->mutex);
	if (ov01a1b->streaming)
		ov01a1b_stop_streaming(ov01a1b);

        ret = ov01a1b_power_off_sequenz(ov01a1b);
	mutex_unlock(&ov01a1b->mutex);

	return ret;
}

static int ov01a1b_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov01a1b *ov01a1b = to_ov01a1b(sd);
	int ret = 0;

	mutex_lock(&ov01a1b->mutex);
        ret = ov01a1b_power_on_sequence(ov01a1b);
        if(ret) {
                goto exit;
        }

	if (!ov01a1b->streaming) {
                ov01a1b_power_off_sequenz(ov01a1b);
		goto exit;
        }

	ret = ov01a1b_start_streaming(ov01a1b);
	if (ret) {
		ov01a1b->streaming = false;
		ov01a1b_stop_streaming(ov01a1b);
	}

exit:
	mutex_unlock(&ov01a1b->mutex);
	return ret;
}
static const struct v4l2_subdev_video_ops ov01a1b_video_ops = {
	.s_stream = ov01a1b_set_stream,
};

static const struct v4l2_subdev_pad_ops ov01a1b_pad_ops = {
	.set_fmt = ov01a1b_set_format,
	.get_fmt = ov01a1b_get_format,
	.enum_mbus_code = ov01a1b_enum_mbus_code,
	.enum_frame_size = ov01a1b_enum_frame_size,
};

static const struct v4l2_subdev_ops ov01a1b_subdev_ops = {
	.video = &ov01a1b_video_ops,
	.pad = &ov01a1b_pad_ops,
};



static int ov01a1b_probe(struct i2c_client *client)
{
    struct device *dev = &client->dev;
    struct ov01a1b *ov01a1b;
    int ret;

  // only target platform   
#if LINUX_VERSION_CODE > KERNEL_VERSION(6, 15, 3)
#if IS_ENABLED(CONFIG_INTEL_SKL_INT3472)

    dev_info(dev, "\n=== OV01A1B Power Test Probe ===\n");
    dev_info(dev, "Chip ID: 0x%02x\n", REG_CHIP_ID);
    dev_info(dev, "Adapter: %s\n", client->adapter->name);
    dev_info(dev, "Client address: 0x%02x\n", client->addr);

#else
#error "CONFIG_INTEL_SKL_INT3472 must be enabled."
#endif
#else
#error "KERNEL_VERSION must greater than 6.15.3 ."
#endif

    ov01a1b = devm_kzalloc(&client->dev, sizeof(*ov01a1b), GFP_KERNEL);
    if (!ov01a1b)
    	return -ENOMEM;
    
    v4l2_i2c_subdev_init(&ov01a1b->sd, client, &ov01a1b_subdev_ops);
    
    /* Try to get clock - based on OV01A10 */
    ov01a1b->xvclk = devm_clk_get(dev, "xvclk");
    if (IS_ERR(ov01a1b->xvclk)) {
        dev_info(dev, "No xvclk found, trying default names...\n");
        ov01a1b->xvclk = devm_clk_get(dev, NULL);
        if (IS_ERR(ov01a1b->xvclk)) {
            ov01a1b->xvclk = NULL;
            dev_warn(dev, "No clock found - continuing without\n");
        }
    }
    
    /* Default to 19.2 MHz like OV01A10 */
    ov01a1b->xvclk_freq = 19200000;
    
    /* Try to get regulators - based on OV01A10 */
    ov01a1b->supplies[0].supply = "avdd";
    ov01a1b->supplies[1].supply = "dovdd";
    ov01a1b->supplies[2].supply = "dvdd";
    
    ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(ov01a1b->supplies),
                                  ov01a1b->supplies);
    if (ret < 0) {
        dev_info(dev, "Cannot get regulators, trying alternatives...\n");
        
        /* Try alternative names */
        ov01a1b->supplies[0].supply = "vana";
        ov01a1b->supplies[1].supply = "vio";
        ov01a1b->supplies[2].supply = "vdig";
        
        ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(ov01a1b->supplies),
                                      ov01a1b->supplies);
        if (ret < 0) {
            dev_warn(dev, "No regulators found - continuing without\n");
            ov01a1b->supplies[0].consumer = NULL;
        }
    }
    
    /* Try to get GPIOs */
    ov01a1b->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
    if (IS_ERR(ov01a1b->reset_gpio)) {
        ov01a1b->reset_gpio = NULL;
        dev_warn(dev, "No reset GPIO found\n");
    }
    
    ov01a1b->powerdown_gpio = devm_gpiod_get_optional(dev, "powerdown", 
                                                     GPIOD_OUT_HIGH);
    if (IS_ERR(ov01a1b->powerdown_gpio)) {
        ov01a1b->powerdown_gpio = NULL;
        dev_warn(dev, "No powerdown GPIO found\n");
    }
    
    /* Try alternative GPIO names */
    if (!ov01a1b->reset_gpio) {
        ov01a1b->reset_gpio = devm_gpiod_get_optional(dev, "xshutdown", 
                                                     GPIOD_OUT_HIGH);
    }
    if (!ov01a1b->powerdown_gpio) {
        ov01a1b->powerdown_gpio = devm_gpiod_get_optional(dev, "pwdn", 
                                                         GPIOD_OUT_HIGH);
    }
    
    /* Execute power on sequence */
    ret = ov01a1b_power_on_sequence(ov01a1b);
    if (ret) {
        goto probe_error_ret;
    }
  
    mutex_init(&ov01a1b->mutex);
//    ov01a1b->cur_mode = &supported_modes[0];
//    ret = ov01a1b_init_controls(ov01a1b);
//    if (ret) {
//    	dev_err(&client->dev, "failed to init controls: %d", ret);
//    	goto error_handler_free;
//    }


//    /* Execute power off sequence */
//    ret = ov01a1b_power_off_sequenz(ov01a1b);
//    if(ret ){
//        return -ENODEV;
//    }
//
//    dev_info(dev, "Good: Sensor not responding after power off\n");
//    dev_info(dev, "\n=== Test Complete ===\n");
//
    // Runtime PM init and activate    
    pm_runtime_set_autosuspend_delay(dev, 1000); // 1000 ms Timeout
    pm_runtime_use_autosuspend(dev);
    pm_runtime_set_active(dev); // device -> "active" 
    pm_runtime_enable(dev); 
    
    /* Always return error to avoid binding */
    // TEST SETTING
    // return -ENODEV;
    return 0;
//error_media_entity:
//    media_entity_cleanup(&ov01a1b->sd.entity);
//error_handler_free:
//    v4l2_ctrl_handler_free(&ov01a1b->ctrl_handler);
//    mutex_destroy(&ov01a1b->mutex);
probe_error_ret:
	return ret;
}

static void ov01a1b_remove(struct i2c_client *client)
{
    dev_info(&client->dev, "OV01A1B power test remove\n");

    struct v4l2_subdev *sd = i2c_get_clientdata(client);
    struct device *dev = &client->dev;
    struct ov01a1b *ov01a1b = to_ov01a1b(sd);
    
    pm_runtime_get_sync(dev);
    pm_runtime_disable(dev);
    pm_runtime_put_noidle(dev);
    ov01a1b_suspend(dev);
    
    v4l2_async_unregister_subdev(sd);
    media_entity_cleanup(&sd->entity);
    v4l2_ctrl_handler_free(sd->ctrl_handler);
    mutex_destroy(&ov01a1b->mutex);

}

static const struct acpi_device_id ov01a1b_acpi_ids[] = {
    { "OVTI01AB", 0 },
    { }
};
MODULE_DEVICE_TABLE(acpi, ov01a1b_acpi_ids);

static const struct i2c_device_id ov01a1b_id[] = {
    { "ov01a1b", 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, ov01a1b_id);

static struct i2c_driver ov01a1b_driver = {
    .driver = {
        .name = DRIVER_NAME,
        .acpi_match_table = ov01a1b_acpi_ids,
    },
    .probe = ov01a1b_probe,
    .remove = ov01a1b_remove,
    .id_table = ov01a1b_id,
};

// this struct is mapping the power operations
static const struct dev_pm_ops ov01a1b_pm_ops = {
    // callback, wake up from "idle"
    .runtime_resume   = ov01a1b_resume,


    // callback, set to "idle"
    .runtime_suspend  = ov01a1b_suspend,

    // Optional, maybe for the future 
    // .runtime_idle     = ..., 

    // system Suspend/Hibernate, not runtime PM.
    .suspend          = ov01a1b_suspend,
    .resume           = ov01a1b_resume,
};

module_i2c_driver(ov01a1b_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Niklas Bartz");
MODULE_DESCRIPTION("OV01A1B power sequence test based on OV01A10");

