/*
 * me302c_spc_pwr_set.c: ASUS ME302C hardware power related initialization code
 *
 * (C) Copyright 2012 ASUS Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/gpio.h>
#include <linux/HWVersion.h>
#define PIN_3VSUS_SYNC  45
#define PIN_5VSUS_SYNC  159
#define PIN_5VSUS_EN    160

extern int Read_PROJ_ID(void);
extern int Read_HW_ID(void);
static int PROJ_ID;
static int HW_ID;

static int __init me302c_specific_gpio_setting_init()
{
	PROJ_ID = Read_PROJ_ID();
	HW_ID = Read_HW_ID();
	gpio_request(PIN_3VSUS_SYNC, "P_+3VSO_SYNC_5");
	if (PROJ_ID == PROJ_ID_ME302C || PROJ_ID == PROJ_ID_ME372CG || PROJ_ID == PROJ_ID_GEMINI){
		gpio_request(PIN_5VSUS_SYNC, "P_+5VSO_SYNC_EN");
		gpio_request(PIN_5VSUS_EN, "P_+5VSO_EN_10");
		gpio_direction_output(PIN_5VSUS_EN, 1);
		gpio_direction_output(PIN_5VSUS_SYNC, 0);
	}
	if (PROJ_ID == PROJ_ID_ME372CL && HW_ID == HW_ID_ER){
		gpio_request(PIN_5VSUS_EN, "P_+5VSO_EN_10");
		gpio_direction_output(PIN_5VSUS_EN, 1);
	}
	gpio_direction_output(PIN_3VSUS_SYNC, 0);

	return 0;

}

module_init(me302c_specific_gpio_setting_init);
