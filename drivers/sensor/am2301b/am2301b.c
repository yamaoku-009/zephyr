/*
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

#define AM2301B_SENSOR_FIXEDPOINT_BASE  (0x100000)

LOG_MODULE_REGISTER(AM2301B, CONFIG_SENSOR_LOG_LEVEL);

struct am2301b_data {
	uint32_t temp;
	uint32_t humidity;
};

struct am2301b_config {
	const struct device *i2c_dev;
	uint8_t i2c_addr;
};

const static uint8_t am2301b_cmd_initialize[3] = { AM2301B_REG_INITIALIZE, AM2301B_OP_INIT_DATA_1, AM2301B_OP_INIT_DATA_2 };
const static uint8_t am2301b_cmd_mesurment[3] = { AM2301B_REG_MESURMENT, AM2301B_OP_MESURMENT_1, AM2301B_OP_MESURMENT_2 };

static inline int am2301b_write_cmd(struct am2301b_config *cfg, const uint8_t *data, const size_t len )
{
	return i2c_write(cfg->i2c_dev, data, len, cfg->i2c_addr);
}

static inline int am2301b_read_status(struct am2301b_config *cfg, uint8_t *status)
{
	return i2c_reg_read_byte(cfg->i2c_dev, cfg->i2c_addr, AM2301B_REG_STATUS, status);
}

static inline int am2301b_read_mesaument(struct am2301b_config *cfg, uint8_t *data, int len)
{
	return i2c_read(cfg->i2c_dev, data, len, cfg->i2c_addr);
}

#ifdef CONFIG_AM2301B_CRC_CHECK
static uint8_t am2301b_compute_crc(uint8_t *data, int len)
{
	return crc8(data, len, 0x31, 0xFF, false);
}
#endif /* CONFIG_AM2301B_CRC_CHECK */

static int am2301b_device_measurment(struct am2301b_config *cfg, uint8_t *data, int len)
{
	int ret, wait_count;

	ret = am2301b_write_cmd(cfg,am2301b_cmd_mesurment,sizeof(am2301b_cmd_mesurment));
	if (ret != 0) {
		LOG_ERR("measurment command write failed.");
		return -EBUSY;
	}

	k_msleep(AM2301B_TIME_MEASURMENT);
	for ( wait_count = 0 ; wait_count < AM2301B_MEASUREMENT_WAIT_MAX_COUNT ; wait_count++ ) {
		if (am2301b_read_mesaument(cfg, data, len) == 0) {
			if ((data[0] & AM2301B_MASK_MESURMENT_BUSY) != AM2301B_MASK_MESURMENT_BUSY) {
				break;
			}
		}

		k_msleep(AM2301B_MASK_MESURMENT_BUSY);
	}

	if (wait_count == AM2301B_MEASUREMENT_WAIT_MAX_COUNT) {
		LOG_ERR("measurment: timeout.");
		return -EBUSY;
	}

	return 0;
}

static int am2301b_get_measument_raw(struct am2301b_config *cfg, struct am2301b_data *data)
{
	uint8_t buf[7];
#ifdef CONFIG_AM2301B_CRC_CHECK
    uint8_t crc;
#endif /* CONFIG_AM2301B_CRC_CHECK */
	int ret;

	ret = am2301b_device_measurment(cfg, buf, sizeof(buf));
	if (ret != 0) {
		return ret;
	}

#ifdef CONFIG_AM2301B_CRC_CHECK
	crc = am2301b_compute_crc(buf, 6);
	if (buf[6] != crc) {
		LOG_ERR("CRC MISMATCH:0x%02x(Computed: 0x%02x)", buf[6], crc);
		return -EIO;
	}
#endif /* CONFIG_AM2301B_CRC_CHECK */

	data->humidity = ((uint32_t)buf[1] << 12) | ((uint32_t)buf[2] << 4) | (buf[3] >> 4);
	data->temp = (uint32_t)(buf[3] & 0x0f) << 16 | ((uint32_t)buf[4] << 8) | buf[5];

	return 0;
}

static int am2301b_device_reset(struct am2301b_config *cfg)
{
	uint8_t status;

	if (am2301b_read_status(cfg, &status) != 0) {
		LOG_ERR("reset: read status failed.");
		return -EBUSY;
	}

	if (status == AM2301B_INIT_COMPLETE_STATUS) {
		return 0;
	}

	/* Start initilaize */
	if (am2301b_write_cmd(cfg,am2301b_cmd_initialize,sizeof(am2301b_cmd_initialize)) != 0) {
		LOG_ERR("reset: send init command failed.");
		return -EBUSY;
	}

	k_msleep(AM2301B_TIME_INIT_WAIT);
	if (am2301b_read_status(cfg, &status) != 0) {
		LOG_ERR("reset: init : read status failed.");
		return -EBUSY;
	}

	if (status != AM2301B_INIT_COMPLETE_STATUS) {
		LOG_ERR("reset: can't init device");
		return -EBUSY;
	}

	return 0;
}

static int am2301b_sample_fetch(const struct device *dev,
				enum sensor_channel chan)
{
	struct am2301b_data *data = (struct am2301b_data *)dev->data;
	struct am2301b_config *cfg = (struct am2301b_config *)dev->config;

	switch (chan) {
	case SENSOR_CHAN_ALL:
	case SENSOR_CHAN_AMBIENT_TEMP:
	case SENSOR_CHAN_HUMIDITY:
		return am2301b_get_measument_raw(cfg, data);
	default:
		return -ENOTSUP;
	}
}

/**
 * @brief Convert Fixed-point sensor value to sensor_value
 *
 * @param val Output sensor value
 * @param sensor_raw sensor raw value
 * @param base_val Base value
 */
static inline void am2301b_convert_sensor_value(struct sensor_value *val, uint32_t sensor_raw, int32_t mul, int32_t base_val)
{
	/* Input format: 20bit (Q20) */
	uint32_t t = sensor_raw * mul + base_val * AM2301B_SENSOR_FIXEDPOINT_BASE;
	int32_t val_1, val_2;

	val_1 = t / AM2301B_SENSOR_FIXEDPOINT_BASE;
	val_2 = (((t - (val_1 * AM2301B_SENSOR_FIXEDPOINT_BASE)) * 1000) / AM2301B_SENSOR_FIXEDPOINT_BASE) * 1000;

	val->val1 = val_1;
	val->val2 = val_2;
}

static int am2301b_channel_get(const struct device *dev,
			       enum sensor_channel chan,
			       struct sensor_value *val)
{
	struct am2301b_data *data = (struct am2301b_data *)dev->data;

	switch (chan) {
	case SENSOR_CHAN_AMBIENT_TEMP:
		am2301b_convert_sensor_value(val, data->temp, 200, -50);
		return 0;

	case SENSOR_CHAN_HUMIDITY:
		am2301b_convert_sensor_value(val, data->humidity, 100, 0);
		return 0;

	default:
		return -ENOTSUP;
	}
}

int am2301b_init(const struct device *dev)
{
	struct am2301b_config *cfg = (struct am2301b_config *)dev->config;
	int ret;

	if (!device_is_ready(cfg->i2c_dev)) {
		LOG_ERR("I2C device is not ready.");
		return -ENODEV;
	}

	k_msleep(AM2301B_TIME_POWER_ON);
	ret = am2301b_device_reset(cfg);
	if (!ret) {
		return ret;
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
		.i2c_dev = DEVICE_DT_GET(DT_INST_BUS(inst)),		      \
		.i2c_addr = AM2301B_I2C_ADDRESS,			      \
	};								      \
	DEVICE_DT_INST_DEFINE(inst, am2301b_init, NULL, &am2301b_data_##inst, \
			      &am2301b_config_##inst, POST_KERNEL,	      \
			      CONFIG_SENSOR_INIT_PRIORITY, &am2301b_driver_api);

DT_INST_FOREACH_STATUS_OKAY(AM2301B_INST)
