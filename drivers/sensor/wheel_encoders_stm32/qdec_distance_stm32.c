#define DT_DRV_COMPAT st_stm32_qdec_distance

/** @file
 * @brief STM32 family Quadrature Decoder (QDEC) driver.
 */

#include <errno.h>

#include <zephyr/sys/__assert.h>
#include <zephyr/sys/util.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control/stm32_clock_control.h>
#include <zephyr/logging/log.h>

#include <stm32_ll_tim.h>
#include <math.h>

LOG_MODULE_REGISTER(qdec_stm32_distance, CONFIG_SENSOR_LOG_LEVEL);

#define HALF_INT32_MAX INT32_MAX/2

/* Device constant configuration parameters */
struct qdec_stm32_dev_cfg {
	const struct pinctrl_dev_config *pin_config;
	struct stm32_pclken pclken;
	TIM_TypeDef *timer_inst;
	bool is_input_polarity_inverted;
	uint8_t input_filtering_level;
	uint32_t counts_per_revolution;
};

/* Device run time data */
struct qdec_stm32_dev_data {
	uint16_t ticks;
	int64_t count;
};

static int qdec_stm32_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct qdec_stm32_dev_data *dev_data = dev->data;
	const struct qdec_stm32_dev_cfg *dev_cfg = dev->config;
	uint32_t curr_ticks;
	uint32_t last_ticks = dev_data->ticks;

	if ((chan != SENSOR_CHAN_ALL) && (chan != SENSOR_CHAN_ROTATION)) {
		return -ENOTSUP;
	}

	curr_ticks = LL_TIM_GetCounter(dev_cfg->timer_inst);
	dev_data->ticks = curr_ticks;
	int64_t diff = (int64_t)curr_ticks - last_ticks;
	if(llabs(diff) > UINT16_MAX/2 && diff > 0) {
		diff -= UINT16_MAX;
	}
	if (llabs(diff) > UINT16_MAX/2 && diff < 0) {
		diff += UINT16_MAX;
	}

	dev_data->count += diff;

	return 0;
}

static int qdec_stm32_get(const struct device *dev, enum sensor_channel chan,
			struct sensor_value *val)
{
	struct qdec_stm32_dev_data *const dev_data = dev->data;

	if (chan == SENSOR_CHAN_ALL){
		val->val1 = (dev_data->count) / (int64_t)pow(10, 6);
		int64_t mod = llabs(dev_data->count) % (int64_t)pow(10,6);

		if (dev_data->count < 0)
			val->val2 = -mod; // Here the val2 is ticks instead of accuracy
		else val->val2 = mod;
	}
	else {
		return -ENOTSUP;
	}

	return 0;
}

static int qdec_stm32_initialize(const struct device *dev)
{
	const struct qdec_stm32_dev_cfg *const dev_cfg = dev->config;

	int retval;
	LL_TIM_ENCODER_InitTypeDef init_props;
	uint32_t max_counter_value;

	retval = pinctrl_apply_state(dev_cfg->pin_config, PINCTRL_STATE_DEFAULT);
	if (retval < 0) {
		return retval;
	}

	if (!device_is_ready(DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE))) {
		LOG_ERR("Clock control device not ready");
		return -ENODEV;
	}

	retval = clock_control_on(DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE),
			     (clock_control_subsys_t)&dev_cfg->pclken);
	if (retval < 0) {
		LOG_ERR("Could not initialize clock");
		return retval;
	}

	if (dev_cfg->counts_per_revolution < 1) {
		LOG_ERR("Invalid number of counts per revolution (%d)",
			dev_cfg->counts_per_revolution);
		return -EINVAL;
	}

	LL_TIM_ENCODER_StructInit(&init_props);

	if (dev_cfg->is_input_polarity_inverted) {
		init_props.IC1ActiveInput = LL_TIM_IC_POLARITY_FALLING;
		init_props.IC2ActiveInput = LL_TIM_IC_POLARITY_FALLING;
	}

	init_props.IC1Filter = dev_cfg->input_filtering_level * LL_TIM_IC_FILTER_FDIV1_N2;
	init_props.IC2Filter = dev_cfg->input_filtering_level * LL_TIM_IC_FILTER_FDIV1_N2;

	/* Ensure that the counter will always count up to a multiple of counts_per_revolution */
	if (IS_TIM_32B_COUNTER_INSTANCE(dev_cfg->timer_inst)) {
		max_counter_value = UINT32_MAX;
	} else {
		max_counter_value = UINT16_MAX;
	}
	LL_TIM_SetAutoReload(dev_cfg->timer_inst, max_counter_value);

	if (LL_TIM_ENCODER_Init(dev_cfg->timer_inst, &init_props) != SUCCESS) {
		LOG_ERR("Initalization failed");
		return -EIO;
	}

	LL_TIM_EnableCounter(dev_cfg->timer_inst);

	return 0;
}

static const struct sensor_driver_api qdec_stm32_driver_api = {
	.sample_fetch = qdec_stm32_fetch,
	.channel_get = qdec_stm32_get,
};

#define QDEC_STM32_INIT(n)								\
	PINCTRL_DT_INST_DEFINE(n);							\
	static const struct qdec_stm32_dev_cfg qdec##n##_stm32_config = {		\
		.pin_config = PINCTRL_DT_INST_DEV_CONFIG_GET(n),			\
		.timer_inst = ((TIM_TypeDef *)DT_REG_ADDR(DT_INST_PARENT(n))),		\
		.pclken = {								\
			.bus = DT_CLOCKS_CELL(DT_INST_PARENT(n), bus),			\
			.enr = DT_CLOCKS_CELL(DT_INST_PARENT(n), bits)			\
		},									\
		.is_input_polarity_inverted = DT_INST_PROP(n, st_input_polarity_inverted),	\
		.input_filtering_level = DT_INST_PROP(n, st_input_filter_level),		\
		.counts_per_revolution = DT_INST_PROP(n, st_counts_per_revolution),		\
	};										\
											\
	static struct qdec_stm32_dev_data qdec##n##_stm32_data;				\
											\
	SENSOR_DEVICE_DT_INST_DEFINE(n, qdec_stm32_initialize, NULL,			\
				&qdec##n##_stm32_data, &qdec##n##_stm32_config,		\
				POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,		\
				&qdec_stm32_driver_api);

DT_INST_FOREACH_STATUS_OKAY(QDEC_STM32_INIT)
