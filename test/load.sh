#!/usr/bin/env bash

MODULE_PATH="result/lib/modules/6.15.4/kernel/drivers/media/i2c"
sudo insmod "$MODULE_PATH/ov01a1b_power_test.ko"

