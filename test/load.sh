#!/usr/bin/env bash

MODULE_PATH="result/lib/modules/6.15.5/kernel/drivers/media/i2c"
sudo insmod "$MODULE_PATH/$1.ko" dyndbg=+p

