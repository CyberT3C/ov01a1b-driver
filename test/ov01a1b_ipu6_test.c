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

#if NOT IS_ENABLED(CONFIG_INTEL_SKL_INT3472)
# only target platform
#error "CONFIG_INTEL_SKL_INT3472 must be enabled."
#endif

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


struct ov01a1b {
    // struct v4l2_subdev sd;
    //

    // hardware
    u8 address;
    struct i2c_client *client;
    struct clk *xvclk;
    struct gpio_desc *reset_gpio;
    struct gpio_desc *powerdown_gpio;
    struct regulator_bulk_data supplies[3];
    u32 xvclk_freq;
};

static int write_register(struct ov01a1b *ov01a1b, u16 reg, u8 val)
{
    u8 buf[3] = { reg >> 8, reg & 0xff, val };
    
    return i2c_master_send(ov01a1b->client, buf, 3);
}

static int read_register(struct ov01a1b *ov01a1b, u16 reg, u8 *val)
{
    struct i2c_msg msgs[2];
    u8 reg_buf[2] = { reg >> 8, reg & 0xff };
    
    msgs[0].addr = ov01a1b->address;
    msgs[0].flags = 0;
    msgs[0].len = 2;
    msgs[0].buf = reg_buf;
    
    msgs[1].addr = ov01a1b->address;
    msgs[1].flags = I2C_M_RD;
    msgs[1].len = 1;
    msgs[1].buf = val;
    
    return i2c_transfer(ov01a1b->client->adapter, msgs, 2);
}

// Test basic sensor registers
static void test_sensor_registers(struct ov01a1b *sensor)
{
    u8 val;

    struct device *dev = &sensor->client->dev;

    dev_info(dev, "\n=== Basic Register Test ===\n");
   

    // I know
    read_register(sensor, 0x3001, &val);
    dev_info(dev, "Reg 0x3001 = 0x%02x\n", val);
    
    read_register(sensor, 0x3002, &val);
    dev_info(dev, "Reg 0x3002 = 0x%02x\n", val);


    // Check for monochrome/RAW format
    read_register(sensor, 0x4300, &val);  // Format control
    dev_info(dev, "Format control (0x4300) = 0x%02x\n", val);
    
    // Check MIPI lanes configuration
    read_register(sensor, 0x4800, &val);
    dev_info(dev, "MIPI control (0x4800) = 0x%02x\n", val);
    // 0x04 might mean 1-lane MIPI (common for low-res sensors)
    
    // Check for IR-specific features
    read_register(sensor, 0x5000, &val);
    dev_info(dev, "ISP control (0x5000) = 0x%02x\n", val);
    // 0x75 from your dump might have IR-specific bits
}

static void test_sensor_registers_extended(struct ov01a1b *sensor)
{
    u8 val;
    struct device *dev = &sensor->client->dev;
    
    dev_info(dev, "\n=== Extended Register Test ===\n");
    
    // Chip ID registers (already working)
    read_register(sensor, 0x300a, &val);
    dev_info(dev, "Chip ID High (0x300a) = 0x%02x\n", val);
    read_register(sensor, 0x300b, &val);
    dev_info(dev, "Chip ID Mid (0x300b) = 0x%02x\n", val);
    read_register(sensor, 0x300c, &val);
    dev_info(dev, "Chip ID Low (0x300c) = 0x%02x\n", val);
    
    // System control registers
    read_register(sensor, 0x0100, &val);  // Mode select
    dev_info(dev, "Mode select (0x0100) = 0x%02x\n", val);
    read_register(sensor, 0x0103, &val);  // Software reset
    dev_info(dev, "Software reset (0x0103) = 0x%02x\n", val);
    
    // PLL registers (important for clock config)
    read_register(sensor, 0x0300, &val);
    dev_info(dev, "PLL1 ctrl (0x0300) = 0x%02x\n", val);
    read_register(sensor, 0x0301, &val);
    dev_info(dev, "PLL1 ctrl (0x0301) = 0x%02x\n", val);
    read_register(sensor, 0x0302, &val);
    dev_info(dev, "PLL1 ctrl (0x0302) = 0x%02x\n", val);
    read_register(sensor, 0x0303, &val);
    dev_info(dev, "PLL1 ctrl (0x0303) = 0x%02x\n", val);
    
    // Timing registers
    read_register(sensor, 0x3800, &val);
    dev_info(dev, "H_START high (0x3800) = 0x%02x\n", val);
    read_register(sensor, 0x3801, &val);
    dev_info(dev, "H_START low (0x3801) = 0x%02x\n", val);
    read_register(sensor, 0x3802, &val);
    dev_info(dev, "V_START high (0x3802) = 0x%02x\n", val);
    read_register(sensor, 0x3803, &val);
    dev_info(dev, "V_START low (0x3803) = 0x%02x\n", val);
    
    // Output size
    read_register(sensor, 0x3808, &val);
    dev_info(dev, "H_OUTPUT_SIZE high (0x3808) = 0x%02x\n", val);
    read_register(sensor, 0x3809, &val);
    dev_info(dev, "H_OUTPUT_SIZE low (0x3809) = 0x%02x\n", val);
    read_register(sensor, 0x380a, &val);
    dev_info(dev, "V_OUTPUT_SIZE high (0x380a) = 0x%02x\n", val);
    read_register(sensor, 0x380b, &val);
    dev_info(dev, "V_OUTPUT_SIZE low (0x380b) = 0x%02x\n", val);
    
    // Total size (for FPS calculation)
    read_register(sensor, 0x380c, &val);
    dev_info(dev, "HTS high (0x380c) = 0x%02x\n", val);
    read_register(sensor, 0x380d, &val);
    dev_info(dev, "HTS low (0x380d) = 0x%02x\n", val);
    read_register(sensor, 0x380e, &val);
    dev_info(dev, "VTS high (0x380e) = 0x%02x\n", val);
    read_register(sensor, 0x380f, &val);
    dev_info(dev, "VTS low (0x380f) = 0x%02x\n", val);
    
    // Exposure control
    read_register(sensor, 0x3500, &val);
    dev_info(dev, "Exposure[19:16] (0x3500) = 0x%02x\n", val);
    read_register(sensor, 0x3501, &val);
    dev_info(dev, "Exposure[15:8] (0x3501) = 0x%02x\n", val);
    read_register(sensor, 0x3502, &val);
    dev_info(dev, "Exposure[7:0] (0x3502) = 0x%02x\n", val);
    
    // Gain control
    read_register(sensor, 0x3508, &val);
    dev_info(dev, "Gain high (0x3508) = 0x%02x\n", val);
    read_register(sensor, 0x3509, &val);
    dev_info(dev, "Gain low (0x3509) = 0x%02x\n", val);
    
    // MIPI control
    read_register(sensor, 0x4800, &val);
    dev_info(dev, "MIPI ctrl (0x4800) = 0x%02x\n", val);
    read_register(sensor, 0x4801, &val);
    dev_info(dev, "MIPI ctrl (0x4801) = 0x%02x\n", val);
    
    // Test pattern
    read_register(sensor, 0x5080, &val);
    dev_info(dev, "Test pattern (0x5080) = 0x%02x\n", val);
    
    dev_info(dev, "=== End Extended Register Test ===\n");
}

// Helper function to dump a register range
static void dump_register_range(struct ov01a1b *sensor, u16 start, u16 end)
{
    struct device *dev = &sensor->client->dev;
    u8 val;
    int i;
    
    dev_info(dev, "\nDumping registers 0x%04x - 0x%04x:\n", start, end);
    for (i = start; i <= end; i++) {
        if (read_register(sensor, i, &val) == 2) {
            dev_info(dev, "  0x%04x = 0x%02x\n", i, val);
        }
    }
}

// Try to find undocumented features
static void probe_sensor_capabilities(struct ov01a1b *sensor)
{
    struct device *dev = &sensor->client->dev;
    u8 val;
    
    dev_info(dev, "\n=== Probing Sensor Capabilities ===\n");
    
    // Check for common OmniVision register ranges
    dump_register_range(sensor, 0x3000, 0x3010);  // System control
    dump_register_range(sensor, 0x3800, 0x3810);  // Timing
    dump_register_range(sensor, 0x5000, 0x5010);  // ISP control
    
    // Try to identify supported modes by checking certain registers
    dev_info(dev, "\nChecking for mode-specific registers...\n");
    
    // Some sensors have mode registers at 0x3820-0x3821
    read_register(sensor, 0x3820, &val);
    dev_info(dev, "Format1 (0x3820) = 0x%02x (bit 1: vflip, bit 2: hflip)\n", val);
    read_register(sensor, 0x3821, &val);
    dev_info(dev, "Format2 (0x3821) = 0x%02x\n", val);
}

static void search_for_ir_modes(struct ov01a1b *sensor)
{
    struct device *dev = &sensor->client->dev;
    u8 val;
    
    dev_info(dev, "\n=== Searching for IR/Face Recognition Modes ===\n");
    
    // Check binning/skipping registers
    read_register(sensor, 0x3814, &val);
    dev_info(dev, "H_INC (0x3814) = 0x%02x (horizontal subsampling)\n", val);
    read_register(sensor, 0x3815, &val);
    dev_info(dev, "V_INC (0x3815) = 0x%02x (vertical subsampling)\n", val);
    
    // Check for binning configuration
    read_register(sensor, 0x3820, &val);
    dev_info(dev, "Format1 (0x3820) = 0x%02x\n", val);
    dev_info(dev, "  - Bit 0: Binning H = %d\n", val & 0x01);
    dev_info(dev, "  - Bit 1: Binning V = %d\n", (val >> 1) & 0x01);
    dev_info(dev, "  - Bit 7-6: Special mode = %d\n", (val >> 6) & 0x03);
    
    // Check ISP window size (might show actual output)
    read_register(sensor, 0x5680, &val);
    dev_info(dev, "ISP H_SIZE high (0x5680) = 0x%02x\n", val);
    read_register(sensor, 0x5681, &val);
    dev_info(dev, "ISP H_SIZE low (0x5681) = 0x%02x\n", val);
    read_register(sensor, 0x5682, &val);
    dev_info(dev, "ISP V_SIZE high (0x5682) = 0x%02x\n", val);
    read_register(sensor, 0x5683, &val);
    dev_info(dev, "ISP V_SIZE low (0x5683) = 0x%02x\n", val);
    
    // Check for IR-specific registers
    dev_info(dev, "\nChecking IR-specific features:\n");
    
    // Common IR control registers in OmniVision sensors
    read_register(sensor, 0x5300, &val);
    dev_info(dev, "IR control (0x5300) = 0x%02x\n", val);
    read_register(sensor, 0x5301, &val);
    dev_info(dev, "IR control (0x5301) = 0x%02x\n", val);
    
    // Check if there's a mode select register
    read_register(sensor, 0x3708, &val);
    dev_info(dev, "Mode control? (0x3708) = 0x%02x\n", val);
}

// Test different resolutions by writing registers
static int test_resolution_mode(struct ov01a1b *sensor, u16 width, u16 height)
{
    struct device *dev = &sensor->client->dev;
    u8 val;
    int ret;
    
    dev_info(dev, "\n=== Testing %dx%d mode ===\n", width, height);
    
    // Try to set output size
    ret = write_register(sensor, 0x3808, (width >> 8) & 0xff);
    ret |= write_register(sensor, 0x3809, width & 0xff);
    ret |= write_register(sensor, 0x380a, (height >> 8) & 0xff);
    ret |= write_register(sensor, 0x380b, height & 0xff);
    
    if (ret < 0) {
        dev_err(dev, "Failed to write resolution registers\n");
        return ret;
    }
    
    // Read back to verify
    msleep(10);
    read_register(sensor, 0x3808, &val);
    dev_info(dev, "H_OUTPUT_SIZE high readback = 0x%02x\n", val);
    read_register(sensor, 0x3809, &val);
    dev_info(dev, "H_OUTPUT_SIZE low readback = 0x%02x\n", val);
    read_register(sensor, 0x380a, &val);
    dev_info(dev, "V_OUTPUT_SIZE high readback = 0x%02x\n", val);
    read_register(sensor, 0x380b, &val);
    dev_info(dev, "V_OUTPUT_SIZE low readback = 0x%02x\n", val);
    
    return 0;
}

// Calculate and display timing information
static void analyze_timing(struct ov01a1b *sensor)
{
    struct device *dev = &sensor->client->dev;
    u8 val_h, val_l;
    u16 hts, vts, h_output, v_output;
    u32 pixel_rate, fps;
    
    dev_info(dev, "\n=== Timing Analysis ===\n");
    
    // Read current values
    read_register(sensor, 0x380c, &val_h);
    read_register(sensor, 0x380d, &val_l);
    hts = (val_h << 8) | val_l;
    
    read_register(sensor, 0x380e, &val_h);
    read_register(sensor, 0x380f, &val_l);
    vts = (val_h << 8) | val_l;
    
    read_register(sensor, 0x3808, &val_h);
    read_register(sensor, 0x3809, &val_l);
    h_output = (val_h << 8) | val_l;
    
    read_register(sensor, 0x380a, &val_h);
    read_register(sensor, 0x380b, &val_l);
    v_output = (val_h << 8) | val_l;
    
    dev_info(dev, "HTS (Horizontal Total Size): %d\n", hts);
    dev_info(dev, "VTS (Vertical Total Size): %d\n", vts);
    dev_info(dev, "Output resolution: %dx%d\n", h_output, v_output);
    
    // Assuming 19.2MHz clock (from your power sequence)
    // This is simplified - actual calculation depends on PLL settings
    pixel_rate = 19200000; // This would need PLL calculation
    fps = pixel_rate / (hts * vts);
    
    dev_info(dev, "Estimated FPS (assuming direct clock): %d\n", fps);
}

// Try to find supported sensor modes
static void probe_sensor_modes(struct ov01a1b *sensor)
{
    struct device *dev = &sensor->client->dev;
    
    dev_info(dev, "\n=== Probing Common IR Sensor Modes ===\n");
    
    // First analyze current timing
    analyze_timing(sensor);
    
    // Search for IR-specific configurations
    search_for_ir_modes(sensor);
    
    // Test writing different resolutions
    // Note: We'll do a soft reset after to restore defaults
    test_resolution_mode(sensor, 400, 400);  // Expected IR mode
    test_resolution_mode(sensor, 640, 480);  // Common VGA
    test_resolution_mode(sensor, 320, 240);  // QVGA
    
    // Soft reset to restore defaults
    dev_info(dev, "\nPerforming soft reset to restore defaults...\n");
    write_register(sensor, 0x0103, 0x01);
    msleep(10);
}

static int ov01a1b_check_i2c_address(struct i2c_client *client, u8 addr)
{
    struct i2c_msg msg[2];
    u8 reg_addr[2];
    u8 chip_id[3];
    u32 id_value;
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
    msg[1].len = 3;             // Read 3 bytes at once
    msg[1].buf = chip_id;
    
    ret = i2c_transfer(client->adapter, msg, 2);
    if (ret != 2) {
        return ret < 0 ? ret : -EIO;
    }
    
    id_value = (chip_id[0] << 16) | (chip_id[1] << 8) | chip_id[2];
    dev_info(&client->dev, "Chip ID at address 0x%02x: 0x%06x\n", addr, id_value);
    
    if (id_value == CHIP_ID) {
        dev_info(&client->dev, "*** OV01A1B detected! ***\n");
        return 1; // chip found
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
    ret = ov01a1b_check_i2c_address(power->client, power->address);
    if (ret == 0) {
        dev_info(dev, "Found device at address 0x%02x!\n", power->address);
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

    power->address = 0x10;
    ret = ov01a1b_check_i2c_address(power->client, power->address);
    if (ret == 0) {
        dev_info(dev, "Device is working at address 0x%02x!\n", power->address);
    }
  
    // test and get informations
    test_sensor_registers(power);
    test_sensor_registers_extended(power);
    probe_sensor_capabilities(power);
    search_for_ir_modes(power);
    probe_sensor_modes(power);

    /* Execute power off sequence */
    ov01a1b_power_off(power);
    msleep(10);
    /* Try to read chip ID - should fail */

    ret = ov01a1b_check_i2c_address(power->client, power->address);
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

