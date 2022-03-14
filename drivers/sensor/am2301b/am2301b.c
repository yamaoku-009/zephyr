/* am2301b.c - Driver for Aosong AM2301B Humidity and Temperature Module */
/*
 * Copyright (c) 2022 Hisanori Yamaoku
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT aosong_am2301b

#include <device.h>
#include <devicetree.h>
#include <drivers/i2c.h>
#include <drivers/sensor.h>
#include <logging/log.h>

#ifdef CONFIG_AM2301B_CRC_CHECK
#include <sys/crc.h>
#endif /* CONFIG_AM2301B_CRC_CHECK */

#include "am2301b.h"

LOG_MODULE_REGISTER(AM2301B, CONFIG_SENSOR_LOG_LEVEL);

#define AM2301B_SENSOR_FIXEDPOINT_BASE  0x100000
#define AM2301B_LEN_MEASURMENT_DATA              7U

/* AM2301B Registers */
#define AM2301B_REG_SOFT_RESET                  0xBA
#define AM2301B_INIT_COMPLETE_STATUS            0x18

#define AM2301B_REG_STATUS                      0x71
#define AM2301B_MASK_MEASUREMENT_BUSY           0x80

struct am2301b_data {
	uint32_t temperture;
	uint32_t humidity;
};

struct am2301b_config {
	struct i2c_dt_spec i2c;
};

const static uint8_t am2301b_cmd_initialize[3] = { 0xBE, 0x08, 0x00 };
const static uint8_t am2301b_cmd_mesurment[3] = { 0xAC, 0x33, 0x00 };

#ifdef CONFIG_AM2301B_CRC_CHECK
static inline bool am2301b_check_crc(uint8_t *data)
{
	return crc8(data, 6, 0x31, 0xFF, false) != data[6];
}
#endif /* CONFIG_AM2301B_CRC_CHECK */

static int am2301b_fetch_measument_data(const struct am2301b_config *cfg, struct am2301b_data *data)
{
	uint8_t buf[AM2301B_LEN_MEASURMENT_DATA];

	if (i2c_write_dt(&cfg->i2c, am2301b_cmd_mesurment, sizeof(am2301b_cmd_mesurment))) {
		LOG_ERR("A communication error was detected during measurement.");
		return -EIO;
	}

	k_sleep(K_MSEC(AM2301B_TIME_MEASURMENT));

	if (i2c_read_dt(&cfg->i2c, buf, sizeof(buf))) {
		LOG_ERR("A communication error was detected during measurement.");
		return -EIO;
	}

	if (buf[0] & AM2301B_MASK_MEASUREMENT_BUSY) {
		LOG_ERR("A timeout was detected during measurement.");
		return -EBUSY;
	}

#ifdef CONFIG_AM2301B_CRC_CHECK
	if (am2301b_check_crc(buf)) {
		LOG_ERR("crc mismatch");
		return -EIO;
	}
#endif /* CONFIG_AM2301B_CRC_CHECK */

	data->humidity = ((uint32_t)buf[1] << 12) | ((uint32_t)buf[2] << 4) | (buf[3] >> 4);
	data->temperture = (uint32_t)(buf[3] & 0x0f) << 16 | ((uint32_t)buf[4] << 8) | buf[5];

	return 0;
}

static int am2301b_sample_fetch(const struct device *dev,
				enum sensor_channel chan)
{
	const struct am2301b_config *cfg = dev->config;
	struct am2301b_data *data = dev->data;

	switch (chan) {
	case SENSOR_CHAN_ALL:
	case SENSOR_CHAN_AMBIENT_TEMP:
	case SENSOR_CHAN_HUMIDITY:
		return am2301b_fetch_measument_data(cfg, data);
	default:
		return -ENOTSUP;
	}
}

static inline void am2301b_convert_temperture_value(struct sensor_value *val, uint32_t sensor_raw)
{
	const static int32_t power20_of_2 = 0x100000;
	int32_t t = sensor_raw * 200 - power20_of_2 * 50;

	val->val1 = t / power20_of_2;
	/* To prevent overflow, instead of multiplying by 1000000 to convert sensor_value,
	   we split it into multiplying by 1000 twice. */
	val->val2 = ((t % power20_of_2) * 1000 /  power20_of_2) * 1000;
}

static inline void am2301b_convert_humidity_value(struct sensor_value *val, uint32_t sensor_raw)
{
	const static int32_t power20_of_2 = 0x100000;
	uint32_t t = sensor_raw * 100;

	val->val1 = t / power20_of_2;
	/* To prevent overflow, instead of multiplying by 1000000 to convert sensor_value,
	   we split it into multiplying by 1000 twice. */
	val->val2 = ((t % power20_of_2) * 1000 /  power20_of_2) * 1000;
}

static int am2301b_channel_get(const struct device *dev,
			       enum sensor_channel chan,
			       struct sensor_value *val)
{
	struct am2301b_data *data = (struct am2301b_data *)dev->data;

	switch (chan) {
	case SENSOR_CHAN_AMBIENT_TEMP:
		am2301b_convert_temperture_value(val, data->temperture);
		return 0;

	case SENSOR_CHAN_HUMIDITY:
		am2301b_convert_humidity_value(val, data->humidity);
		return 0;

	default:
		return -ENOTSUP;
	}
}

static int am2301b_init(const struct device *dev)
{
	const struct am2301b_config *cfg = dev->config;
	uint8_t status;

	if (!device_is_ready(cfg->i2c.bus)) {
		LOG_ERR("device is not ready.");
		return -ENODEV;
	}

	k_sleep(K_MSEC(AM2301B_TIME_POWER_ON));

	if (i2c_write_dt(&cfg->i2c, am2301b_cmd_initialize, sizeof(am2301b_cmd_initialize))) {
		LOG_ERR("Failed to send the initialization command.");
		return -EIO;
	}

	k_sleep(K_MSEC(AM2301B_TIME_INIT_WAIT));

	if (i2c_reg_read_byte_dt(&cfg->i2c, AM2301B_REG_STATUS, &status)) {
		LOG_ERR("Failed to receive status.");
		return -EIO;
	}

	if (status != AM2301B_INIT_COMPLETE_STATUS) {
		LOG_ERR("Detected timeout on reset.");
		return -EBUSY;
	}

	return 0;
}

static const struct sensor_driver_api am2301b_driver_api = {
	.sample_fetch = am2301b_sample_fetch,
	.channel_get = am2301b_channel_get,
};

#define AM2301B_INST(inst)						      \
	static struct am2301b_data am2301b_data_##inst;			      \
	static const struct am2301b_config am2301b_config_##inst = {	      \
		.i2c = I2C_DT_SPEC_INST_GET(inst),			      \
	};								      \
	DEVICE_DT_INST_DEFINE(inst, am2301b_init, NULL, &am2301b_data_##inst, \
			      &am2301b_config_##inst, POST_KERNEL,	      \
			      CONFIG_SENSOR_INIT_PRIORITY, &am2301b_driver_api);

DT_INST_FOREACH_STATUS_OKAY(AM2301B_INST)
