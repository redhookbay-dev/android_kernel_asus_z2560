/*
 * Copyright (c) 2012, uPI Semiconductor Corp. All Rights Reserved.
 */

#ifndef __UPI_ug31xx_GAUGE_H
#define __UPI_ug31xx_GAUGE_H

#define UG31XX_DEV_NAME        "ug31xx-gauge"

#define	UPI_DEBUG_STRING	(512)

typedef enum {
 	UG31XX_DRV_NOT_READY = 0,
 	UG31XX_DRV_INIT_OK,
 	UG31XX_DRV_SUSPEND,
} ug31xx_drv_status_t;

typedef enum {
  UG31XX_GPIO_1 = 0,
  UG31XX_GPIO_2,
  UG31XX_GPIO_3,
  UG31XX_GPIO_4,
} ug31xx_gpio_idx_t;

typedef enum {
  UG31XX_GPIO_STS_LOW = 0,
  UG31XX_GPIO_STS_HIGH,
  UG31XX_GPIO_STS_UNKNOWN,
} ug31xx_gpio_status_t;

#if !defined(CONFIG_PF400CG) && !defined(CONFIG_ME175CG)
#define UPI_UG31XX_SHELL_AP         ("/system/bin/upi_gg_ctl")
#define	UPI_UG31XX_BACKUP_FILE		  ("/sdcard/upi_gg")
#define UPI_UG31XX_BACKUP_SUSPEND   ("/sdcard/upi_table")
#else
#define UPI_UG31XX_SHELL_AP         ("/factory/upi_gg_ctl")
#define	UPI_UG31XX_BACKUP_FILE		  ("/factory/upi_gg")
#define UPI_UG31XX_BACKUP_SUSPEND   ("/factory/upi_table")
#endif

#define	UPI_UG31XX_MODULE_READY		  (1)
#define	UPI_UG31XX_MODULE_NOT_READY	(0)
#define	UPI_UG31XX_BATTERY_REMOVED	(1)
#define	UPI_UG31XX_BATTERY_INSERTED	(0)
#define	UPI_UG31XX_ALARM_STATUS_UV	(1<<0)
#define	UPI_UG31XX_ALARM_STATUS_UET	(1<<1)
#define	UPI_UG31XX_ALARM_STATUS_OET	(1<<2)
#define UPI_UG31XX_NTC_NORMAL       (0)
#define UPI_UG31XX_NTC_OPEN         (1)
#define UPI_UG31XX_NTC_SHORT        (2)

#ifndef _LKM_OPTIONS_

#define _LKM_OPTIONS_
#define LKM_OPTIONS_FORCE_RESET             (1<<0)
#define LKM_OPTIONS_ENABLE_SUSPEND_DATA_LOG (1<<1)
#define LKM_OPTIONS_ENABLE_DEBUG_LOG        (3<<2)
  #define LKM_OPTIONS_DEBUG_ERROR           (0<<2)
  #define LKM_OPTIONS_DEBUG_INFO            (1<<2)
  #define LKM_OPTIONS_DEBUG_NOTICE          (2<<2)
  #define LKM_OPTIONS_DEBUG_DEBUG           (3<<2)
#define LKM_OPTIONS_ENABLE_REVERSE_CURRENT  (1<<4)
#define LKM_OPTIONS_ADJUST_DESIGN_CAPACITY  (1<<5)
#define LKM_OPTIONS_DISABLE_BACHUP_FILE     (1<<6)

#endif  ///< end of _LKM_OPTIONS_

struct ug31xx_module_interface {
	int (*initial)(char *ggb);
	int (*uninitial)(void);
	int (*suspend)(char dc_in);
	int (*resume)(void);
	int (*shutdown)(void);
	int (*update)(void);
	int (*reset)(char *ggb);

	int (*get_voltage)(void);
	int (*get_voltage_now)(void);
	int (*get_current)(void);
	int (*get_current_now)(void);
	int (*get_external_temperature)(void);
	int (*get_external_temperature_now)(void);
	int (*get_internal_temperature)(void);
	int (*get_internal_temperature_now)(void);
	int (*get_remaining_capacity)(void);
	int (*get_full_charge_capacity)(void);
	int (*get_relative_state_of_charge)(void);
	char * (*get_version)(void);
	int (*get_polling_time)(void);
	int (*get_module_ready)(void);
	int (*get_battery_removed)(void);
	int (*get_alarm_status)(void);
	int (*get_charge_termination_current)(void);
	int (*get_full_charge_status)(void);
	int (*get_design_capacity)(void);
	int (*get_rsense)(void);
	int (*get_predict_rsoc)(void);
	int (*get_gpio)(ug31xx_gpio_idx_t gpio);
	int (*get_cycle_count)(void);
	int (*get_avg_external_temperature)(void);
	int (*get_ntc_status)(void);
	unsigned char * (*get_backup_buffer)(int *size);
	unsigned char (*get_backup_daemon_cntl)(void);
	unsigned char (*get_backup_daemon_period)(void);
	int (*get_update_interval)(void);
	int (*get_update_time)(void);
	int (*get_board_offset)(void);
	int (*get_delta_q)(void);

	int (*set_backup_file)(char enable);
	int (*set_charger_full)(char is_full);
	int (*set_charge_termination_current)(int curr);
	int (*set_battery_temp_external)(void);
	int (*set_battery_temp_internal)(void);
	int (*set_rsense)(int rsense);
	int (*set_backup_file_name)(char *filename, int length);
	int (*set_suspend_file_name)(char *filename, int length);
	int (*set_options)(unsigned char options);
	int (*set_gpio)(ug31xx_gpio_idx_t gpio, int status);
	int (*set_shell_ap)(char *apname, int length);
	int (*set_backup_daemon_cntl)(unsigned char cntl);
	int (*set_capacity_suspend_mode)(char in_suspend);

	int (*chk_backup_file)(void);
	int (*enable_save_data)(char enable);
	int (*change_to_pri_batt)(char *ggb, char pri_batt);
	int (*ug31xx_i2c_read)(unsigned short addr, unsigned char *data);
	int (*ug31xx_i2c_write)(unsigned short addr, unsigned char *data);
	int (*reset_cycle_count)(void);
	int (*adjust_cell_table)(unsigned short design_capacity);
	int (*calibrate_offset)(void);
};

enum {
	PWR_SUPPLY_BATTERY = 0,
	PWR_SUPPLY_AC,
	PWR_SUPPLY_USB
};

enum {
	UG31XX_NO_CABLE = 0,
	UG31XX_USB_PC_CABLE = 1,
	UG31XX_PAD_POWER = 2,
	UG31XX_AC_ADAPTER_CABLE = 3
};

enum {
  UG31XX_CHARGER_NO_DETECTS_FULL = 0,
  UG31XX_CHARGER_DETECTS_FULL,
  UG31XX_TAPER_REACHED,
};

extern struct ug31xx_module_interface ug31_module;

#endif /*__UPI_ug31xx_GAUGE_H */
