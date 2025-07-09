// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2020-2022 Intel Corporation.
// Copyright (c) 2025 Niklas Bartz
//
// OV01A1B camera driver
// Based on ov01a10 driver by Intel Corporation
// OV01A1B Power Test Module - Based on OV01A10 power sequence

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/acpi.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio/consumer.h>
#include <linux/clk.h>

#define DRIVER_NAME "ov01a1b_power_test"
#define CHIP_ID 0x560141

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
static const u8 address = 0x10;


struct ov01a1b {
    struct i2c_client *client;
    struct clk *xvclk;
    struct gpio_desc *reset_gpio;
    struct gpio_desc *powerdown_gpio;
    struct regulator_bulk_data supplies[3];
    u32 xvclk_freq;
};



static int ov01a1b_check_i2c_address(struct i2c_client *client, u8 addr)
{
    struct i2c_msg msg[2];
    u8 reg_addr[2];
    u8 chip_id[3];
    int ret;
    
    /* Read chip ID registers 0x300a, 0x300b, 0x300c */
    reg_addr[0] = 0x30;
    reg_addr[1] = 0x0a;
    
    msg[0].addr = addr;
    msg[0].flags = 0;
    msg[0].len = 2;
    msg[0].buf = reg_addr;
    
    msg[1].addr = addr;
    msg[1].flags = I2C_M_RD;
    msg[1].len = 3;  /* Read 3 bytes at once */
    msg[1].buf = chip_id;
    
    ret = i2c_transfer(client->adapter, msg, 2);
    if (ret < 0) {
        return ret;
    }
    
    if (ret == 2) {
        dev_info(&client->dev, 
                 "Chip ID at address 0x%02x: 0x%02x%02x%02x\n", 
                 addr, chip_id[0], chip_id[1], chip_id[2]);
        
        /* Check if this matches expected OV01A1B ID */
        if (chip_id[0] == 0x56 && chip_id[1] == 0x01 && chip_id[2] == 0x41) {
            dev_info(&client->dev, "*** OV01A1B detected! ***\n");
        }
    }
    
    return 0;
}

static void ov01a1b_power_off(struct ov01a1b *power)
{

    struct device *dev = &power->client->dev;
    dev_info(dev, "startign power off sequence...\n");
    
    /* Step 1: Assert reset (put sensor in reset) */
    if (power->reset_gpio)
        gpiod_set_value_cansleep(power->reset_gpio, 1);

    /* Small delay to ensure reset is registered */
    usleep_range(1000, 1500);
    
    /* Step 2: Assert powerdown */
    if (power->powerdown_gpio)
        gpiod_set_value_cansleep(power->powerdown_gpio, 1);
    
    /* Step 3: Disable clock */
    clk_disable_unprepare(power->xvclk);
    
    /* Step 4: Disable regulators (reverse order!) */
    regulator_bulk_disable(ARRAY_SIZE(power->supplies),
                          power->supplies);

    dev_info(dev, "Power off complete\n");
}


static int ov01a1b_power_on_sequence(struct ov01a1b *power)
{
    struct device *dev = &power->client->dev;
    int ret;
    
    dev_info(dev, "Starting OV01A10-style power sequence...\n");
    
    /* Step 1: Enable regulators (if available) */
    if (power->supplies[0].consumer) {
        dev_info(dev, "Enabling regulators...\n");
        ret = regulator_bulk_enable(ARRAY_SIZE(power->supplies), 
                                    power->supplies);
        if (ret < 0) {
            dev_err(dev, "Failed to enable regulators: %d\n", ret);
            /* Continue anyway, might not be required */
        } else {
            /* Wait for voltages to stabilize */
            usleep_range(1000, 1500);
        }
    }
    
    /* Step 2: Enable clock (if available) */
    if (power->xvclk) {
        dev_info(dev, "Enabling XVCLK at %u Hz...\n", power->xvclk_freq);
        
        ret = clk_set_rate(power->xvclk, power->xvclk_freq);
        if (ret < 0) {
            dev_warn(dev, "Failed to set xvclk rate: %d\n", ret);
        }
        
        ret = clk_prepare_enable(power->xvclk);
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
    if (power->powerdown_gpio) {
        dev_info(dev, "Releasing powerdown...\n");
        gpiod_set_value_cansleep(power->powerdown_gpio, 0);
        usleep_range(1000, 1500);
    }
    
    /* Step 4: Toggle reset (if available) */
    if (power->reset_gpio) {
        dev_info(dev, "Toggling reset...\n");
        /* Assert reset */
        gpiod_set_value_cansleep(power->reset_gpio, 1);
        usleep_range(1000, 1500);
        /* Deassert reset */
        gpiod_set_value_cansleep(power->reset_gpio, 0);
        /* Wait for sensor to boot - this is critical! */
        msleep(20);
    }
    
    /* Step 5: Additional delay for sensor initialization */
    dev_info(dev, "Waiting for sensor initialization...\n");
    msleep(10);
    
    /* Step 6: test address */
    dev_info(dev, "Probing I2C address...\n");
    ret = ov01a1b_check_i2c_address(power->client, address);
    if (ret == 0) {
        dev_info(dev, "Found device at address 0x%02x!\n", address);
    }
    
    return 0;
}

static int ov01a1b_power_test_probe(struct i2c_client *client)
{
    struct device *dev = &client->dev;
    struct ov01a1b *power;
    int ret;
    
    dev_info(dev, "\n=== OV01A1B Power Test Probe ===\n");
    dev_info(dev, "Client address: 0x%02x\n", client->addr);
    dev_info(dev, "Adapter: %s\n", client->adapter->name);
    
    power = devm_kzalloc(dev, sizeof(*power), GFP_KERNEL);
    if (!power)
        return -ENOMEM;
    
    power->client = client;
    
    /* Try to get clock - based on OV01A10 */
    power->xvclk = devm_clk_get(dev, "xvclk");
    if (IS_ERR(power->xvclk)) {
        dev_info(dev, "No xvclk found, trying default names...\n");
        power->xvclk = devm_clk_get(dev, NULL);
        if (IS_ERR(power->xvclk)) {
            power->xvclk = NULL;
            dev_warn(dev, "No clock found - continuing without\n");
        }
    }
    
    /* Default to 19.2 MHz like OV01A10 */
    power->xvclk_freq = 19200000;
    
    /* Try to get regulators - based on OV01A10 */
    power->supplies[0].supply = "avdd";
    power->supplies[1].supply = "dovdd";
    power->supplies[2].supply = "dvdd";
    
    ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(power->supplies),
                                  power->supplies);
    if (ret < 0) {
        dev_info(dev, "Cannot get regulators, trying alternatives...\n");
        
        /* Try alternative names */
        power->supplies[0].supply = "vana";
        power->supplies[1].supply = "vio";
        power->supplies[2].supply = "vdig";
        
        ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(power->supplies),
                                      power->supplies);
        if (ret < 0) {
            dev_warn(dev, "No regulators found - continuing without\n");
            power->supplies[0].consumer = NULL;
        }
    }
    
    /* Try to get GPIOs */
    power->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
    if (IS_ERR(power->reset_gpio)) {
        power->reset_gpio = NULL;
        dev_warn(dev, "No reset GPIO found\n");
    }
    
    power->powerdown_gpio = devm_gpiod_get_optional(dev, "powerdown", 
                                                     GPIOD_OUT_HIGH);
    if (IS_ERR(power->powerdown_gpio)) {
        power->powerdown_gpio = NULL;
        dev_warn(dev, "No powerdown GPIO found\n");
    }
    
    /* Try alternative GPIO names */
    if (!power->reset_gpio) {
        power->reset_gpio = devm_gpiod_get_optional(dev, "xshutdown", 
                                                     GPIOD_OUT_HIGH);
    }
    if (!power->powerdown_gpio) {
        power->powerdown_gpio = devm_gpiod_get_optional(dev, "pwdn", 
                                                         GPIOD_OUT_HIGH);
    }
    
    /* Execute power on sequence */
    ret = ov01a1b_power_on_sequence(power);


    ret = ov01a1b_check_i2c_address(power->client, address);
    if (ret == 0) {
        dev_info(dev, "Device is working at address 0x%02x!\n", 
                 address);
    }
    

    /* Execute power off sequence */
    ov01a1b_power_off(power);
    msleep(10);
    /* Try to read chip ID - should fail */

    ret = ov01a1b_check_i2c_address(power->client, address);
    if (ret == 0) {
      dev_err(dev, "ERROR: Sensor still responding after power off!\n");
    } else {
        dev_info(dev, "Good: Sensor not responding after power off\n");
    }

    dev_info(dev, "\n=== Test Complete ===\n");
    
    /* Always return error to avoid binding */
    return -ENODEV;
}

static void ov01a1b_power_test_remove(struct i2c_client *client)
{
    dev_info(&client->dev, "OV01A1B power test remove\n");
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

static struct i2c_driver ov01a1b_power_test_driver = {
    .driver = {
        .name = DRIVER_NAME,
        .acpi_match_table = ov01a1b_acpi_ids,
    },
    .probe = ov01a1b_power_test_probe,
    .remove = ov01a1b_power_test_remove,
    .id_table = ov01a1b_id,
};

module_i2c_driver(ov01a1b_power_test_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Niklas Bartz");
MODULE_DESCRIPTION("OV01A1B power sequence test based on OV01A10");

