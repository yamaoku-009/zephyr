/*
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_AM2301B_AM2301B_H_
#define ZEPHYR_DRIVERS_SENSOR_AM2301B_AM2301B_H_

#include <device.h>
#include <sys/util.h>
#include <zephyr/types.h>

#define AM2301B_I2C_ADDRESS                     0x38

#define AM2301B_TIME_POWER_ON                   40U

/* Status */
#define AM2301B_REG_STATUS                      0x71
#define AM2301B_MASK_MESURMENT_BUSY             0x80
#define AM2301B_MASK_MESURMENT_CAL_EN           0x04

#define AM2301B_INIT_COMPLETE_STATUS            0x18

/* INIT */
#define AM2301B_REG_INITIALIZE                  0xBE
#define AM2301B_OP_INIT_DATA_1                  0x08
#define AM2301B_OP_INIT_DATA_2                  0x00
#define AM2301B_TIME_INIT_WAIT                  10U

/* Mesaument command */
#define AM2301B_REG_MESURMENT                   0xAC
#define AM2301B_OP_MESURMENT_1                  0x33
#define AM2301B_OP_MESURMENT_2                  0x00
#define AM2301B_TIME_MEASURMENT                 80U

#define AM2301B_LEN_MESURMENT_DATA              7U

/* b[7]: busy, 0: not busy , 1: busy */
#define AM2301B_MASK_MES_STATUS_BUSY            0x80

#define AM2301B_REG_SOFT_RESET                  0xBA
#define AM2301B_TIME_RESET                      20U

#define AM2301B_MEASUREMENT_WAIT_MAX_COUNT      20U
#define AM2301B_MEASUREMENT_WAIT_TIME           10U

#endif /* ZEPHYR_DRIVERS_SENSOR_AM2301B_AM2301B_H_ */