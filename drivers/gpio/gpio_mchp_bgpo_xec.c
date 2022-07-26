/*
 * Copyright (c) 2022 Silicom Connectivity Solutions
 *
 * Initial contents based on gpio_mchp_xec_v2.c which is:
 * Copyright (c) 2019 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT microchip_xec_bgpo

#include <errno.h>
#include <device.h>
#include <drivers/gpio.h>
#include <soc.h>
#include <arch/arm/aarch32/cortex_m/cmsis.h>

#include "gpio_utils.h"


struct bgpo_xec_data {
	/* gpio_driver_data needs to be first */
	struct gpio_driver_data common;
	/* port ISR callback routine address */
	sys_slist_t callbacks;
};

struct bgpo_xec_config {
	/* gpio_driver_config needs to be first */
	struct gpio_driver_config common;
	uintptr_t data_addr;
	uintptr_t power_addr;
	uintptr_t reset_addr;
	uint32_t mask;
};

static inline uintptr_t pin_parin_addr(const struct device *dev)
{
	const struct bgpo_xec_config *config = dev->config;

	return config->data_addr;
}

static inline uintptr_t pin_parout_addr(const struct device *dev)
{
	const struct bgpo_xec_config *config = dev->config;

	return config->data_addr;
}

static inline uintptr_t pin_power_addr(const struct device *dev)
{
	const struct bgpo_xec_config *config = dev->config;

	return config->power_addr;
}

/*
 * Use Zephyr system API to implement
 * reg32(addr) = (reg32(addr) & ~mask) | (val & mask)
 */
static inline void xec_mask_write32(uintptr_t addr, uint32_t mask, uint32_t val)
{
	uint32_t r = (sys_read32(addr) & ~mask) | (val & mask);

	sys_write32(r, addr);
}

/*
 */
static int bgpo_xec_configure(const struct device *dev,
			      gpio_pin_t pin, gpio_flags_t flags)
{
	const struct bgpo_xec_config *config = dev->config;
	uintptr_t pout_addr = pin_parout_addr(dev);
	uintptr_t power_addr = pin_power_addr(dev);

	/* Validate pin number range in terms of current port */
	if ((config->mask & BIT(pin)) == 0) {
		return -EINVAL;
	}

	sys_set_bit(power_addr, pin);

	if ((flags & GPIO_OUTPUT) != 0U) {
		if ((flags & GPIO_OUTPUT_INIT_HIGH) != 0U) {
			sys_set_bit(pout_addr, pin);
		} else if ((flags & GPIO_OUTPUT_INIT_LOW) != 0U) {
			sys_clear_bit(pout_addr, pin);
		}
	}

	return 0;
}

static int bgpo_xec_port_set_masked_raw(const struct device *dev,
					uint32_t mask,
					uint32_t value)
{
	uintptr_t pout_addr = pin_parout_addr(dev);

	xec_mask_write32(pout_addr, mask, value);

	return 0;
}

static int bgpo_xec_port_set_bits_raw(const struct device *dev, uint32_t mask)
{
	uintptr_t pout_addr = pin_parout_addr(dev);

	sys_write32(sys_read32(pout_addr) | mask, pout_addr);

	return 0;
}

static int bgpo_xec_port_clear_bits_raw(const struct device *dev,
					uint32_t mask)
{
	uintptr_t pout_addr = pin_parout_addr(dev);

	sys_write32(sys_read32(pout_addr) & ~mask, pout_addr);

	return 0;
}

static int bgpo_xec_port_toggle_bits(const struct device *dev, uint32_t mask)
{
	uintptr_t pout_addr = pin_parout_addr(dev);

	sys_write32(sys_read32(pout_addr) ^ mask, pout_addr);

	return 0;
}

static int bgpo_xec_port_get_raw(const struct device *dev, uint32_t *value)
{
	uintptr_t pin_addr = pin_parin_addr(dev);

	*value = sys_read32(pin_addr);

	return 0;
}

/* GPIO driver official API table */
static const struct gpio_driver_api bgpo_xec_driver_api = {
	.pin_configure = bgpo_xec_configure,
	.port_get_raw = bgpo_xec_port_get_raw,
	.port_set_masked_raw = bgpo_xec_port_set_masked_raw,
	.port_set_bits_raw = bgpo_xec_port_set_bits_raw,
	.port_clear_bits_raw = bgpo_xec_port_clear_bits_raw,
	.port_toggle_bits = bgpo_xec_port_toggle_bits,
};

#define XEC_BGPO_PORT(n)						\
	static int bgpo_xec_port_init_##n(const struct device *dev)	\
	{								\
		const struct bgpo_xec_config *config = dev->config;	\
									\
		sys_write32(0,config->power_addr);			\
		return 0;						\
	}								\
									\
	static struct bgpo_xec_data bgpo_xec_port_data_##n;		\
									\
	static const struct bgpo_xec_config xec_bgpo_config_##n = {	\
		.common = {						\
			.port_pin_mask =				\
				GPIO_PORT_PIN_MASK_FROM_DT_INST(n),	\
		},							\
		.data_addr = (uintptr_t)DT_INST_REG_ADDR_BY_IDX(n, 0),	\
		.power_addr = (uintptr_t)DT_INST_REG_ADDR_BY_IDX(n, 0) + 0x4,	\
		.reset_addr = (uintptr_t)DT_INST_REG_ADDR_BY_IDX(n, 0) + 0x8,\
		.mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(n),			\
	};								\
									\
	DEVICE_DT_INST_DEFINE(n, bgpo_xec_port_init_##n, NULL,		\
		&bgpo_xec_port_data_##n, &xec_bgpo_config_##n,		\
		POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,	\
		&bgpo_xec_driver_api);

DT_INST_FOREACH_STATUS_OKAY(XEC_BGPO_PORT)
