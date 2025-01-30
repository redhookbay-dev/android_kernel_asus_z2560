/*
 * platform_gc0339_raw.c: GC0339_RAW with iCatch 7002A ISP platform data initilization file
 *
 * (C) Copyright 2013 ASUSTeK COMPUTER INC
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/atomisp_platform.h>
#include <asm/intel-mid.h>
#include <asm/intel_scu_ipcutil.h>
#include <media/v4l2-subdev.h>
#include <linux/regulator/consumer.h>
#include "platform_camera.h"
#include "platform_gc0339_raw.h"


#define VPROG1_VAL 1800000

static int camera_power_2p8_en;
static int camera_reset;
static int camera_power_down;
static int camera_vprog1_on;

static struct regulator *vprog1_reg;


static int gc0339_raw_flisclk_ctrl(struct v4l2_subdev *sd, int flag)
{
	static const unsigned int clock_khz = 19200; //Intel just can support 19.2MHz/9.6MHz/4.8MHz 
	int ret = 0;
	v4l2_err(sd, "%s: ++\n",__func__);
	ret = intel_scu_ipc_osc_clk(OSC_CLK_CAM1, flag ? clock_khz : 0);
	return ret;
}


static int gc0339_raw_gpio_ctrl(struct v4l2_subdev *sd, int flag)
{
	return 0;
}



static int gc0339_raw_power_ctrl(struct v4l2_subdev *sd, int flag)
{
	int reg_err;
	int ret;
	printk("%s: ++\n",__func__);

	//printk("HW ID:%d\n", HW_ID);
	switch (HW_ID){
		case HW_ID_SR1:
		case HW_ID_SR2:
			if (camera_power_2p8_en < 0) {
				ret = camera_sensor_gpio(110, GP_CAMERA_2_8V,
							 GPIOF_DIR_OUT, 0);
				if (ret < 0){
					printk("%s not available.\n", GP_CAMERA_2_8V);
					return ret;
				}
				camera_power_2p8_en = ret;
				printk("%s = %d\n",GP_CAMERA_2_8V,ret);
			}
			if (camera_power_down < 0) {
                                ret = camera_sensor_gpio(160, GP_CAMERA_1_PDWN,
                                                         GPIOF_DIR_OUT, 0);
                                if (ret < 0){
                                        printk("%s not available.\n", GP_CAMERA_1_PDWN);
                                        return ret;
                                }
                                camera_power_down = 160;
                                printk("%s = %d\n",GP_CAMERA_1_PDWN,ret);
                        }
			if (camera_reset < 0) {
				ret = camera_sensor_gpio(176, GP_CAMERA_1_RESET,
							 GPIOF_DIR_OUT, 0);
				if (ret < 0){
					printk("%s not available.\n", GP_CAMERA_1_RESET);
					return ret;
				}
				printk("%s = %d\n",GP_CAMERA_1_RESET,ret);
				camera_reset = ret;
			}
			break;
		case HW_ID_ER:
		case HW_ID_PR:
		case HW_ID_MP:
		default:
			if (camera_power_2p8_en < 0) {
				ret = camera_sensor_gpio(110, GP_CAMERA_2_8V,
							 GPIOF_DIR_OUT, 0);
				if (ret < 0){
					printk("%s not available.\n", GP_CAMERA_2_8V);
					return ret;
				}
				camera_power_2p8_en = ret;
				printk("%s = %d\n",GP_CAMERA_2_8V,ret);
			}
			if (camera_power_down < 0) {
                                ret = camera_sensor_gpio(160, GP_CAMERA_1_PDWN,
                                                         GPIOF_DIR_OUT, 0);
                                if (ret < 0){
                                        printk("%s not available.\n", GP_CAMERA_1_PDWN);
                                        return ret;
                                }
                                camera_power_down = 160;
                                printk("%s = %d\n",GP_CAMERA_1_PDWN,ret);
                        }
			if (camera_reset < 0) {
				ret = camera_sensor_gpio(176, GP_CAMERA_1_RESET,
							 GPIOF_DIR_OUT, 0);
				if (ret < 0){
					printk("%s not available.\n", GP_CAMERA_1_RESET);
					return ret;
				}
				printk("%s = %d\n",GP_CAMERA_1_RESET,ret);
				camera_reset = ret;
			}
			break;
	}

	if (flag){
		//pull high reset first
		if (camera_power_2p8_en >= 0){
			gpio_set_value(camera_power_2p8_en, 1);
			printk("<<< AVDD_SENSOR 2.8V = 1\n");
			msleep(10);
		}

		//turn on power VDD_SEN VDD_HOST 1.8V
		if (!camera_vprog1_on) {
			camera_vprog1_on = 1;
			reg_err = regulator_enable(vprog1_reg);
			if (reg_err) {
				printk(KERN_ALERT "Failed to enable regulator vprog1\n");
				return reg_err;
			}
			printk("<<< VDD_SEN VDD_HOST 1.8V = 1\n");
			msleep(10);
		}

		//turn on MCLK
		gc0339_raw_flisclk_ctrl(sd, 1);
		msleep(10); 

		//pull low power down
		if (camera_power_down >= 0){
                        gpio_set_value(camera_power_down, 0);
                        printk("<<< camera_power_down = 0\n");
                        msleep(10);
                }
		//pull high reset first
		if (camera_reset >= 0){
			gpio_set_value(camera_reset, 0);
			msleep(20);
			gpio_set_value(camera_reset, 1);
			printk("<<< camera_reset = 1\n");
			msleep(10);
		}
	}else{
		//pull low reset first
		if (camera_reset >= 0){
			gpio_set_value(camera_reset, 0);
			printk("<<< camera_reset = 0\n");
			msleep(10);
		}

		if (camera_power_down >= 0){
                        gpio_set_value(camera_power_down, 0);
                        printk("<<< camera_power_down = 0\n");
                        msleep(10);
                }

		//turn off power VDD_SEN VDD_HOST 1.8V
		if (camera_vprog1_on) {
			camera_vprog1_on = 0;
			reg_err = regulator_disable(vprog1_reg);
			if (reg_err) {
				printk(KERN_ALERT "Failed to disable regulator vprog1\n");
				return reg_err;
			}
			printk("<<< VDD_SEN VDD_HOST 1.8V = 0\n");
			msleep(10);
		}
		//pull low reset first
		if (camera_power_2p8_en >= 0){
			gpio_set_value(camera_power_2p8_en, 0);
			gpio_free(camera_power_2p8_en);
			printk("<<< camera_reset = 0\n");
			msleep(10);
		}

		//turn off MCLK
		gc0339_raw_flisclk_ctrl(sd, 0);
		msleep(10); 
	}
		
	return 0;
}

static int gc0339_raw_csi_configure(struct v4l2_subdev *sd, int flag)
{
	/* soc sensor, there is no raw bayer order (set to -1) */
	return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_SECONDARY, 1,
		ATOMISP_INPUT_FORMAT_RAW_10, atomisp_bayer_order_grbg,
		flag);
}

static int gc0339_raw_platform_init(struct i2c_client *client)
{
	int ret;

	printk("%s: ++\n", __func__);
	//VPROG1 for VDD_HOST and VDD_SEN, 1.8V
	vprog1_reg = regulator_get(&client->dev, "vprog1");
	if (IS_ERR(vprog1_reg)) {
		dev_err(&client->dev, "vprog1 failed\n");
		return PTR_ERR(vprog1_reg);
	}
	ret = regulator_set_voltage(vprog1_reg, VPROG1_VAL, VPROG1_VAL);
	if (ret) {
		dev_err(&client->dev, "vprog1 set failed\n");
		regulator_put(vprog1_reg);
	}

	return ret;
}

static int gc0339_raw_platform_deinit(void)
{
	regulator_put(vprog1_reg);

}

static struct camera_sensor_platform_data gc0339_raw_sensor_platform_data = {
	.gpio_ctrl	 = gc0339_raw_gpio_ctrl,
	.flisclk_ctrl	 = gc0339_raw_flisclk_ctrl,
	.power_ctrl	 = gc0339_raw_power_ctrl,
	.csi_cfg	 = gc0339_raw_csi_configure,
	.platform_init   = gc0339_raw_platform_init,
	.platform_deinit = gc0339_raw_platform_deinit,
};

void *gc0339_raw_platform_data(void *info)
{
	camera_power_2p8_en = -1;
	camera_reset = -1;
	camera_power_down = -1;
	return &gc0339_raw_sensor_platform_data;
}
