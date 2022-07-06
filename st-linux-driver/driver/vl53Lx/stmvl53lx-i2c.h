
/* SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause */
/******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 This file is part of VL53LX and is dual licensed,
 either GPL-2.0+
 or 'BSD 3-clause "New" or "Revised" License' , at your option.
 ******************************************************************************
 */

/**  @file stmvl53lx-i2c.h
 * Linux kernel i2c/cci  wrapper for  ST VL53LX sensor i2c interface
 **/

#ifndef STMVL53LX_I2C_H
#define STMVL53LX_I2C_H
#include <linux/types.h>
#include "stmvl53lx.h"

struct i2c_data {
	struct i2c_client *client;
	/** back link to driver for interrupt and clean-up */
	struct stmvl53lx_data *vl53lx_data;

	/* reference counter */
	struct kref ref;

	/*!< if null no regulator use for power ctrl */
	struct regulator *vdd;

	/*!< power enable gpio number
	 *
	 * if -1 no gpio if vdd not avl pwr is not controllable
	 */
	int pwren_gpio;

	/*!< xsdn reset (low active) gpio number to device
	 *
	 *  -1  mean none assume no "resetable"
	 */
	int xsdn_gpio;

	/*!< intr gpio number to device
	 *
	 *  intr is active/low negative edge by default
	 *
	 *  -1  mean none assume use polling
	 *  @warning if the dev tree and intr gpio is require please adapt code
	 */
	int intr_gpio;

	/*!< device boot i2c register address
	 *
	 * boot_reg is the value of device i2c address after it is bring out
	 * of reset.
	 */
	int boot_reg;

	/*!< is set if above irq gpio got acquired */
	struct i2d_data_flags_t {
		unsigned pwr_owned:1; /*!< set if pwren gpio is owned*/
		unsigned xsdn_owned:1; /*!< set if sxdn  gpio is owned*/
		unsigned intr_owned:1; /*!< set if intr  gpio is owned*/
		unsigned intr_started:1; /*!< set if irq is hanlde  */
	} io_flag;

	/** the irq vectore assigned to gpio
	 * -1 if no irq hanled
	 */
	int irq;

	struct msgtctrl_t {
		unsigned unhandled_irq_vec:1;
	} msg_flag;
};

int stmvl53lx_init_i2c(void);
void __exit stmvl53lx_exit_i2c(void *arg);
int stmvl53lx_power_up_i2c(void *arg);
int stmvl53lx_power_down_i2c(void *arg);
int stmvl53lx_reset_release_i2c(void *arg);
int stmvl53lx_reset_hold_i2c(void *arg);
void stmvl53lx_clean_up_i2c(void);
int stmvl53lx_start_intr(void *object, int *poll_mode);
void *stmvl53lx_get(void *arg);
void stmvl53lx_put(void *arg);

#endif /* STMVL53LX_I2C_H */
