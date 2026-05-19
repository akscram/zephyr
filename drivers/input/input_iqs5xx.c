/*
 * Copyright (c) 2024 Ilia Kharin
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT azoteq_iqs5xx

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/init.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(input_iqs5xx, CONFIG_INPUT_LOG_LEVEL);

#define IQS5XX_SHOW_RESET_WAIT_TIMEOUT_US 5000
#define IQS5XX_SHOW_RESET_WAIT_DELAY_US 500

/*
 * Direct-addressable memory map registers are available via I2C where each
 * register has a 16-bit address.
 */

/* Product Number is 2 bytes register */
#define IQS5XX_REG_PRODUCT_NUMBER	0x0000 /* R */
/* Project Number is 2 bytes register */
#define IQS5XX_REG_PROJECT_NUMBER	0x0002 /* R */
#define IQS5XX_REG_SYSTEM_INFO_0	0x000F /* R */
#define IQS5XX_REG_NUMBER_OF_FINGERS	0x0011 /* R */
#define IQS5XX_REG_RELATIVE_X		0x0012 /* R */
#define IQS5XX_REG_RELATIVE_Y		0x0014 /* R */
#define IQS5XX_REG_SYSTEM_CONTROL_0	0x0431 /* R/W */
#define IQS5XX_REG_SYSTEM_CONFIG_1	0x058F /* R/W */
#define IQS5XX_REG_FILTER_SETTINGS_0	0x0632 /* R/W */
#define IQS5XX_REG_XY_CONFIG_0		0x0669 /* R/W */
#define IQS5XX_REG_X_RESOLUTION		0x066E /* R/W */
#define IQS5XX_REG_Y_RESOLUTION		0x0670 /* R/W */
#define IQS5XX_REG_END_COMMUNICATION	0xEEEE /* W */

/* Product Number Register */
#define IQS5XX_PRODUCT_NUMBER_550	0x28
#define IQS5XX_PRODUCT_NUMBER_572	0x3A
#define IQS5XX_PRODUCT_NUMBER_525	0x34

/* Project Number Register */
#define IQS5XX_PROJECT_NUMBER_B000	0x0F

/* System Info 0 Register */
#define IQS5XX_SYSTEM_INFO_0_SHOW_RESET 7

/* System Control 0 Register */
#define IQS5XX_SYSTEM_CONTROL_0_ACK_RESET 7

/* System Config 1 Register */
#define IQS5XX_SYSTEM_CONFIG_1_PROXY_EVENT	7
#define IQS5XX_SYSTEM_CONFIG_1_TOUCH_EVENT	6
#define IQS5XX_SYSTEM_CONFIG_1_SNAP_EVENT	5
#define IQS5XX_SYSTEM_CONFIG_1_ALP_PROXY_EVENT	4
#define IQS5XX_SYSTEM_CONFIG_1_REATI_EVENT	3
#define IQS5XX_SYSTEM_CONFIG_1_TP_EVENT		2
#define IQS5XX_SYSTEM_CONFIG_1_GESTURE_EVENT	1
#define IQS5XX_SYSTEM_CONFIG_1_EVENT_MODE	0

/* Filter Settings 0 Register */
#define IQS5XX_FILTER_SETTINGS_0_ALP_COUNT_FILTER	3
#define IQS5XX_FILTER_SETTINGS_0_IIR_SELECT		2
#define IQS5XX_FILTER_SETTINGS_0_MAV_FILTER		1
#define IQS5XX_FILTER_SETTINGS_0_IIR_FILTER		0

/* XY Config 0 Register */
#define IQS5XX_XY_CONFIG_0_PALM_REJECT		3
#define IQS5XX_XY_CONFIG_0_SWITCH_XY_AXIS	2
#define IQS5XX_XY_CONFIG_0_FLIP_Y		1
#define IQS5XX_XY_CONFIG_0_FLIP_X		0

struct iqs5xx_config {
	struct i2c_dt_spec i2c;
	struct gpio_dt_spec rdy_gpio;
	struct gpio_dt_spec reset_gpio;
	uint16_t x_resolution;
	uint16_t y_resolution;
	bool palm_reject;
	bool switch_xy;
	bool flip_x;
	bool flip_y;
};

struct iqs5xx_data {
	const struct device *dev;
	struct gpio_callback dr_cb_data;
	struct k_work work;
	uint8_t def[1742];
};

static int iqs5xx_handle_interrupt(const struct device *dev)
{
	const struct iqs5xx_config *config = dev->config;

	uint8_t rc;
	uint8_t reg[3] = {0};
	uint8_t values[8] = {0};

	// Read data
	sys_put_be16(IQS5XX_REG_NUMBER_OF_FINGERS, reg);
	rc = i2c_write_read_dt(&config->i2c, reg, 2, values, 1);
	if (rc < 0) {
		LOG_ERR("Failed to read number of fingers: %d", rc);
		return rc;
	}

	uint8_t number_of_fingers = values[0];
	LOG_DBG("Number of fingers: %d", number_of_fingers);
	if (values[0] > 0) {
		sys_put_be16(IQS5XX_REG_RELATIVE_X, reg);
		rc = i2c_write_read_dt(&config->i2c, reg, 2, values, 8);
		if (rc < 0) {
			LOG_ERR("Failed to read X and Y positions: %d", rc);
			return rc;
		}
		int16_t rel_x = (int16_t)(values[0] << 8) | values[1];
		int16_t rel_y = (int16_t)(values[2] << 8) | values[3];

		input_report_rel(dev, INPUT_REL_X, rel_x, false, K_FOREVER);
		input_report_rel(dev, INPUT_REL_Y, rel_y, true, K_FOREVER);

		uint16_t abs_x = (uint16_t)(values[4] << 8) | values[5];
		uint16_t abs_y = (uint16_t)(values[6] << 8) | values[7];

		LOG_INF("Absolute coordinates X/Y: %d:%d", abs_x, abs_y);
	}

	// End communication
	sys_put_be16(IQS5XX_REG_END_COMMUNICATION, reg);
	reg[2] = 0;
	rc = i2c_write_dt(&config->i2c, reg, 3);
	if (rc < 0) {
		LOG_ERR("Failed to end communication: %d", rc);
		return rc;
	}

	return 0;
}

static void iqs5xx_data_ready_gpio_callback(const struct device *dev, struct gpio_callback *cb,
					    uint32_t pins)
{
	struct iqs5xx_data *drv_data = CONTAINER_OF(cb, struct iqs5xx_data, dr_cb_data);

	k_work_submit(&drv_data->work);
}

static void iqs5xx_work_cb(struct k_work *work)
{
	struct iqs5xx_data *drv_data = CONTAINER_OF(work, struct iqs5xx_data, work);

	iqs5xx_handle_interrupt(drv_data->dev);
}

int iqs5xx_init_interrupt(const struct device *dev)
{
	struct iqs5xx_data *drv_data = dev->data;
	const struct iqs5xx_config *config = dev->config;
	const struct gpio_dt_spec *rdy_gpio = &config->rdy_gpio;

	int rc;

	drv_data->dev = dev;
	drv_data->work.handler = iqs5xx_work_cb;

	rc = gpio_pin_interrupt_configure_dt(rdy_gpio, GPIO_INT_EDGE_TO_ACTIVE);
	if (rc) {
		LOG_ERR("Failed to configured interrupt for %s/%d",
			rdy_gpio->port->name, rdy_gpio->pin);
		return rc;
	}

	gpio_init_callback(&drv_data->dr_cb_data, iqs5xx_data_ready_gpio_callback,
			   BIT(rdy_gpio->pin));

	rc = gpio_add_callback(rdy_gpio->port, &drv_data->dr_cb_data);
	if (rc) {
		LOG_ERR("Failed to configured interrupt for %s/%d",
			rdy_gpio->port->name, rdy_gpio->pin);
		return rc;
	}

	return 0;
}

static int iqs5xx_init(const struct device *dev)
{
	const struct iqs5xx_config *config = dev->config;

	uint8_t reg[4] = {0};
	uint8_t values[2] = {0};

	int rc;

	if (!i2c_is_ready_dt(&config->i2c)) {
		LOG_ERR("I2C bus %s is not ready", config->i2c.bus->name);
		return -ENODEV;
	}

	/* Configure GPIO pin for RDY signal */
	if (!gpio_is_ready_dt(&config->rdy_gpio)) {
		LOG_ERR("GPIO device %s/%d is not ready",
			config->rdy_gpio.port->name, config->rdy_gpio.pin);
		return -ENODEV;
	}

	rc = gpio_pin_configure_dt(&config->rdy_gpio, GPIO_INPUT);
	if (rc) {
		LOG_ERR("Failed to configure %s/%d as input",
			config->rdy_gpio.port->name, config->rdy_gpio.pin);
		return rc;
	}

	// Hardware reset
	if (!gpio_is_ready_dt(&config->reset_gpio)) {
		LOG_ERR("GPIO device %s/%d is not ready",
			config->reset_gpio.port->name, config->reset_gpio.pin);
		return -ENODEV;
	}

	rc = gpio_pin_configure_dt(&config->reset_gpio, GPIO_OUTPUT_ACTIVE);
	if (rc < 0) {
		LOG_ERR("Failed to set reset GPIO %s/%d pin active",
			config->reset_gpio.port->name, config->reset_gpio.pin);
		return rc;
	}

	// The reset pulse width must be minimum 20 ns. Thus, holding
	// for 100 us which should be enough.
	k_sleep(K_USEC(100));

	rc = gpio_pin_set_dt(&config->reset_gpio, 0);
	if (rc < 0) {
		LOG_ERR("Failed to set reset GPIO %s/%d pin inactive",
			config->reset_gpio.port->name, config->reset_gpio.pin);
		return rc;
	}

	// Sleep for 4 ms and then wait for the reset indication flag
	// represented by the asserted SHOW_RESET bit
	k_sleep(K_MSEC(4));

	sys_put_be16(IQS5XX_REG_SYSTEM_INFO_0, reg);
	if (!WAIT_FOR((i2c_write_read_dt(&config->i2c, reg, 2, values, 1) == 0
			&& IS_BIT_SET(values[0], IQS5XX_SYSTEM_INFO_0_SHOW_RESET)),
			IQS5XX_SHOW_RESET_WAIT_TIMEOUT_US,
			k_sleep(K_USEC(IQS5XX_SHOW_RESET_WAIT_DELAY_US)))) {
		LOG_ERR("Failed to wait for SHOW_RESET after reset");
		return -EIO;
	}

	// Acknowledge the reset indication flag by setting the ACK_RESET flag
	sys_put_be16(IQS5XX_REG_SYSTEM_CONTROL_0, reg);
	reg[2] = BIT(IQS5XX_SYSTEM_CONTROL_0_ACK_RESET);
	rc = i2c_write_dt(&config->i2c, reg, 3);
	if (rc < 0) {
		LOG_ERR("Failed to acknowledge the reset flag: %d", rc);
		return rc;
	}

	// Read product number and check if it is supported
	sys_put_be16(IQS5XX_REG_PRODUCT_NUMBER, reg);
	rc = i2c_write_read_dt(&config->i2c, reg, 2, values, 2);
	if (rc < 0) {
		LOG_ERR("Failed to read product number: %d", rc);
		return rc;
	}
	uint16_t product_number = sys_get_be16(values);
	LOG_DBG("Product number: %d", product_number);
	if (product_number != IQS5XX_PRODUCT_NUMBER_550
		&& product_number != IQS5XX_PRODUCT_NUMBER_572
		&& product_number != IQS5XX_PRODUCT_NUMBER_525) {
		LOG_ERR("Product number not supported: %d", product_number);
		return -EINVAL;
	}

	// Read project number and check if it is supported
	sys_put_be16(IQS5XX_REG_PROJECT_NUMBER, reg);
	rc = i2c_write_read_dt(&config->i2c, reg, 2, values, 2);
	if (rc < 0) {
		LOG_ERR("Failed to read project number: %d", rc);
		return rc;
	}
	uint16_t project_number = sys_get_be16(values);
	LOG_DBG("Project number: %d", project_number);
	if (project_number != IQS5XX_PROJECT_NUMBER_B000) {
		LOG_ERR("Project number not supported: %d", project_number);
		return -EINVAL;
	}

	// Configure sending interrupts only on touch events
	sys_put_be16(IQS5XX_REG_SYSTEM_CONFIG_1, reg);
	reg[2] = (BIT(IQS5XX_SYSTEM_CONFIG_1_EVENT_MODE)
			| BIT(IQS5XX_SYSTEM_CONFIG_1_TOUCH_EVENT)
			| BIT(IQS5XX_SYSTEM_CONFIG_1_TP_EVENT));
	rc = i2c_write_dt(&config->i2c, reg, 3);
	if (rc < 0) {
		LOG_ERR("Failed to configuring event mode: %d", rc);
		return rc;
	}

	// Configure X resolution
	sys_put_be16(IQS5XX_REG_X_RESOLUTION, reg);
	sys_put_be16(config->x_resolution, &reg[2]);
	rc = i2c_write_dt(&config->i2c, reg, 3);
	if (rc < 0) {
		LOG_ERR("Failed to configuring X resolution: %d", rc);
		return rc;
	}

	// Configure Y resolution
	sys_put_be16(IQS5XX_REG_Y_RESOLUTION, reg);
	sys_put_be16(config->y_resolution, &reg[2]);
	rc = i2c_write_dt(&config->i2c, reg, 3);
	if (rc < 0) {
		LOG_ERR("Failed to configuring Y resolution: %d", rc);
		return rc;
	}

	sys_put_be16(IQS5XX_REG_XY_CONFIG_0, reg);
	reg[2] = 0;
	WRITE_BIT(reg[2], IQS5XX_XY_CONFIG_0_PALM_REJECT, config->palm_reject);
	WRITE_BIT(reg[2], IQS5XX_XY_CONFIG_0_SWITCH_XY_AXIS, config->switch_xy);
	WRITE_BIT(reg[2], IQS5XX_XY_CONFIG_0_FLIP_Y, config->flip_y);
	WRITE_BIT(reg[2], IQS5XX_XY_CONFIG_0_FLIP_X, config->flip_x);
	rc = i2c_write_dt(&config->i2c, reg, 3);
	if (rc < 0) {
		LOG_ERR("Failed to write XY configuration: %d", rc);
		return rc;
	}

	sys_put_be16(IQS5XX_REG_FILTER_SETTINGS_0, reg);
	reg[2] = 0;
	WRITE_BIT(reg[2], IQS5XX_FILTER_SETTINGS_0_IIR_SELECT, 0);
	WRITE_BIT(reg[2], IQS5XX_FILTER_SETTINGS_0_MAV_FILTER, 1);
	WRITE_BIT(reg[2], IQS5XX_FILTER_SETTINGS_0_IIR_FILTER, 1);
	rc = i2c_write_dt(&config->i2c, reg, 3);
	if (rc < 0) {
		LOG_ERR("Failed to write filter settings: %d", rc);
		return rc;
	}
	rc = iqs5xx_init_interrupt(dev);
	if (rc) {
		LOG_ERR("Failed to initialize an interrupt");
		return rc;
	}

	// End communication
	sys_put_be16(IQS5XX_REG_END_COMMUNICATION, reg);
	reg[2] = 0;
	rc = i2c_write_dt(&config->i2c, reg, 3);
	if (rc < 0) {
		LOG_ERR("Failed to end communication: %d", rc);
		return rc;
	}

	return 0;
}

#define IQS5XX_DEFINE(inst)									\
	static const struct iqs5xx_config iqs5xx_config_##inst = {				\
		.i2c = I2C_DT_SPEC_INST_GET(inst),						\
		.rdy_gpio = GPIO_DT_SPEC_INST_GET(inst, rdy_gpios),				\
		.reset_gpio = GPIO_DT_SPEC_INST_GET(inst, reset_gpios),				\
		.x_resolution = DT_INST_PROP(inst, x_resolution),				\
		.y_resolution = DT_INST_PROP(inst, y_resolution),				\
		.palm_reject = DT_INST_PROP(inst, palm_reject),					\
		.switch_xy = DT_INST_PROP(inst, switch_xy),					\
		.flip_x = DT_INST_PROP(inst, flip_x),						\
		.flip_y = DT_INST_PROP(inst, flip_y),						\
	};											\
	static struct iqs5xx_data iqs5xx_data_##inst;						\
	DEVICE_DT_INST_DEFINE(inst, iqs5xx_init, NULL, &iqs5xx_data_##inst,			\
			      &iqs5xx_config_##inst, POST_KERNEL, CONFIG_INPUT_INIT_PRIORITY,	\
			      NULL);								\

DT_INST_FOREACH_STATUS_OKAY(IQS5XX_DEFINE)
