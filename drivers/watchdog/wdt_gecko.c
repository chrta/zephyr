/*
 * Copyright (c) 2019 Interay Solutions B.V.
 * Copyright (c) 2019 Oane Kingma
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <soc.h>
#include <drivers/watchdog.h>
#include <em_wdog.h>
#include <em_cmu.h>

#define LOG_LEVEL CONFIG_WDT_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(wdt_gecko);

/* Timeout period in cycles */
static const u32_t timeout_in_cycles_tbl[16] = {
	9, 17, 33, 65, 129, 257, 513, 1025, 2049, 4097,
	8193, 16385, 32769, 65537, 131073, 262145
};

/* Device constant configuration parameters */
struct wdt_gecko_cfg {
	WDOG_TypeDef *base;
	void (*irq_cfg_func)(void);
};

struct wdt_gecko_data {
	wdt_callback_t callback;
	WDOG_Init_TypeDef wdog_config;
	bool timeout_valid;
};

#define DEV_NAME(dev) ((dev)->config->name)
#define DEV_DATA(dev) \
	((struct wdt_gecko_data *)(dev)->driver_data)
#define DEV_CFG(dev) \
	((struct wdt_gecko_cfg *)(dev)->config->config_info)

static int wdt_gecko_convert_timeout(u32_t timeout)
{
	int idx = 0;

	/* When using ULFRCO (default), 1 cycle is 1 ms +/- 12% */
	while (idx < ARRAY_SIZE(timeout_in_cycles_tbl)) {
		if (timeout > timeout_in_cycles_tbl[idx]) {
			idx++;
			continue;
		}

		/* Found the (rounded up?) value of cycles for
		 *  supplied timeout
		 */
		break;
	}

	return idx;
}

static int wdt_gecko_convert_window(u32_t window, u32_t period)
{
	int idx = 0;
	u32_t incr_val, comp_val;

	incr_val = period / 8;
	comp_val = 0; /* Initially 0, disable */

	/* As we're using fixed max window setting of 75% to allow for
	 * an early warning interrupt, we will only allow a minimum
	 * window setting of 62.5% (= 5 * 12.5%)
	 */
	while (idx < 5) {
		if (window > comp_val) {
			comp_val += incr_val;
			idx++;
			continue;
		}

		break;
	}

	return idx;
}

static int wdt_gecko_setup(struct device *dev, u8_t options)
{
	const struct wdt_gecko_cfg *config = DEV_CFG(dev);
	struct wdt_gecko_data *data = DEV_DATA(dev);
	WDOG_TypeDef *wdog = config->base;

	if (!data->timeout_valid) {
		LOG_ERR("No valid timeouts installed");
		return -EINVAL;
	}

	data->wdog_config.em2Run =
		(options & WDT_OPT_PAUSE_IN_SLEEP) == 0U;
	data->wdog_config.em3Run =
		(options & WDT_OPT_PAUSE_IN_SLEEP) == 0U;

	data->wdog_config.debugRun =
		(options & WDT_OPT_PAUSE_HALTED_BY_DBG) == 0U;

	if (data->callback != NULL) {
		/* Interrupt mode for window */
		/* Enable timeout and early warning interrupt */
		WDOGn_IntEnable(wdog, WDOG_IEN_TOUT | WDOG_IF_WARN);
	} else {
		/* Disable timeout and early warning interrupt */
		WDOGn_IntDisable(wdog, WDOG_IEN_TOUT | WDOG_IF_WARN);
	}

	/* Watchdog is started after initialization */
	WDOGn_Init(wdog, &data->wdog_config);
	LOG_DBG("Setup the watchdog");

	return 0;
}

static int wdt_gecko_disable(struct device *dev)
{
	const struct wdt_gecko_cfg *config = DEV_CFG(dev);
	struct wdt_gecko_data *data = DEV_DATA(dev);
	WDOG_TypeDef *wdog = config->base;

	WDOGn_Enable(wdog, false);
	data->timeout_valid = false;
	LOG_DBG("Disabled the watchdog");

	return 0;
}

static int wdt_gecko_install_timeout(struct device *dev,
				     const struct wdt_timeout_cfg *cfg)
{
	struct wdt_gecko_data *data = DEV_DATA(dev);
	WDOG_Init_TypeDef init_defaults = WDOG_INIT_DEFAULT;

	if (data->timeout_valid) {
		LOG_ERR("No more timeouts can be installed");
		return -ENOMEM;
	}

	if ((cfg->window.max < timeout_in_cycles_tbl[0]) ||
		(cfg->window.max > timeout_in_cycles_tbl[
			ARRAY_SIZE(timeout_in_cycles_tbl) - 1])) {
		LOG_ERR("Upper limit timeout out of range");
		return -EINVAL;
	}

	data->wdog_config = init_defaults;

	data->wdog_config.perSel = (WDOG_PeriodSel_TypeDef)
		wdt_gecko_convert_timeout(cfg->window.max);

	if (cfg->window.min) {
		/* Window mode. Use rounded up timeout value to
		 * calculate minimum window setting.
		 */
		data->wdog_config.winSel = (WDOG_WinSel_TypeDef)
			wdt_gecko_convert_window(cfg->window.min,
				timeout_in_cycles_tbl[
				data->wdog_config.perSel]);
	} else {
		/* Normal mode */
		data->wdog_config.winSel = wdogIllegalWindowDisable;
	}

	/* Using fixed 75% value for early warning interrupt */
	data->wdog_config.warnSel = wdogWarnTime75pct;

	/* Set mode of watchdog and callback */
	switch (cfg->flags) {
	case WDT_FLAG_RESET_SOC:
	case WDT_FLAG_RESET_CPU_CORE:
		data->wdog_config.resetDisable = false;
		LOG_DBG("Configuring reset CPU/SoC mode\n");
		break;

	case WDT_FLAG_RESET_NONE:
		data->wdog_config.resetDisable = true;
		LOG_DBG("Configuring non-reset mode\n");
		break;

	default:
		LOG_ERR("Unsupported watchdog config flag");
		return -EINVAL;
	}

	data->callback = cfg->callback;
	data->timeout_valid = true;

	return 0;
}

static int wdt_gecko_feed(struct device *dev, int channel_id)
{
	const struct wdt_gecko_cfg *config = DEV_CFG(dev);
	WDOG_TypeDef *wdog = config->base;

	if (channel_id != 0) {
		LOG_ERR("Invalid channel id");
		return -EINVAL;
	}

	WDOGn_Feed(wdog);
	LOG_DBG("Fed the watchdog");

	return 0;
}

static void wdt_gecko_isr(void *arg)
{
	struct device *dev = (struct device *)arg;
	const struct wdt_gecko_cfg *config = DEV_CFG(dev);
	struct wdt_gecko_data *data = DEV_DATA(dev);
	WDOG_TypeDef *wdog = config->base;
	u32_t flags;

	/* Clear IRQ flags */
	flags = WDOGn_IntGet(wdog);
	WDOGn_IntClear(wdog, flags);

	if (data->callback) {
		data->callback(dev, 0);
	}
}

static int wdt_gecko_init(struct device *dev)
{
	const struct wdt_gecko_cfg *config = DEV_CFG(dev);

#ifdef CONFIG_WDT_DISABLE_AT_BOOT
	/* Ignore any errors */
	wdt_gecko_disable(dev);
#endif

	/* Enable ULFRCO (1KHz) oscillator */
	CMU_OscillatorEnable(cmuOsc_ULFRCO, true, false);

	/* Enable IRQs */
	config->irq_cfg_func();

	LOG_INF("Device %s initialized", DEV_NAME(dev));

	return 0;
}

static const struct wdt_driver_api wdt_gecko_driver_api = {
	.setup = wdt_gecko_setup,
	.disable = wdt_gecko_disable,
	.install_timeout = wdt_gecko_install_timeout,
	.feed = wdt_gecko_feed,
};

#define GECKO_WDT_INIT(index)						\
									\
	static void wdt_gecko_cfg_func_##index(void);			\
									\
	static const struct wdt_gecko_cfg wdt_gecko_cfg_##index = {	\
		.base = (WDOG_TypeDef *)				\
			DT_INST_##index##_SILABS_GECKO_WDOG_BASE_ADDRESS,\
		.irq_cfg_func = wdt_gecko_cfg_func_##index,		\
	};								\
	static struct wdt_gecko_data wdt_gecko_data_##index;		\
									\
	DEVICE_AND_API_INIT(wdt_##index,				\
				DT_INST_##index##_SILABS_GECKO_WDOG_LABEL,\
				&wdt_gecko_init, &wdt_gecko_data_##index,\
				&wdt_gecko_cfg_##index, POST_KERNEL,	\
				CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,	\
				&wdt_gecko_driver_api);			\
									\
	static void wdt_gecko_cfg_func_##index(void)			\
	{								\
		IRQ_CONNECT(DT_INST_##index##_SILABS_GECKO_WDOG_IRQ_0,	\
			DT_INST_##index##_SILABS_GECKO_WDOG_IRQ_0_PRIORITY,\
			wdt_gecko_isr, DEVICE_GET(wdt_##index), 0);	\
		irq_enable(DT_INST_##index##_SILABS_GECKO_WDOG_IRQ_0);	\
	}

#ifdef DT_INST_0_SILABS_GECKO_WDOG
GECKO_WDT_INIT(0)
#endif /* DT_INST_0_SILABS_GECKO_WDOG */

#ifdef DT_INST_1_SILABS_GECKO_WDOG
GECKO_WDT_INIT(1)
#endif /* DT_INST_1_SILABS_GECKO_WDOG */
