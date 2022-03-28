/*
 * Copyright (c) 2022 Hisanori Yamaoku
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_AM2301B_AM2301B_H_
#define ZEPHYR_DRIVERS_SENSOR_AM2301B_AM2301B_H_

#include <device.h>
#include <sys/util.h>
#include <zephyr/types.h>

struct am2301b_data {
	uint32_t temperature;
	uint32_t humidity;
};

struct am2301b_config {
	struct i2c_dt_spec i2c;
};

#define AM2301B_REG_SOFT_RESET                 0xBA
#define AM2301B_STATUS_INIT_COMPLETE           0x18

#define AM2301B_REG_STATUS                     0x71
#define AM2301B_MASK_MEASUREMENT_BUSY          0x80

#define AM2301B_WAIT_POWER_ON                  40U
#define AM2301B_WAIT_INIT                      10U
#define AM2301B_WAIT_MEASUREMENT               80U

#endif /* ZEPHYR_DRIVERS_SENSOR_AM2301B_AM2301B_H_ */
