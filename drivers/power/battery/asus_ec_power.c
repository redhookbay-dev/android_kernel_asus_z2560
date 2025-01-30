#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/power_supply.h>
#include <linux/wakelock.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/usb/penwell_otg.h>
#include <linux/gpio.h>
#include <asm/intel-mid.h>
#include <linux/debugfs.h>
#include <linux/proc_fs.h>
#include <linux/HWVersion.h>
#include <linux/switch.h>

#include "asus_ec_power.h"
#include "util.h"
#include <linux/microp_notify.h>
#include <linux/microp_api.h>

#define DRIVER_NAME "asus_ec_power"
#define EC_FAIL_MAX_COUNT               3
#define FIRST_RUN_TIME                  3
#define HIGH_POLLING_TIME               60
#define NORMAL_POLLING_TIME		30
#define LOW_POLLING_TIME		10
#define CRITICAL_POLLING_TIME   	5
#define READ_FAIL_POLLING_TIME		60
#define RETRY_COUNT 3
extern int Read_HW_ID(void);
extern int entry_mode;

extern int register_microp_notifier(struct notifier_block *nb);
extern int unregister_microp_notifier(struct notifier_block *nb);

struct asus_pad_device_info {
    struct delayed_work ec_report_work;
    struct workqueue_struct *ec_report_work_q;
    wait_queue_head_t  charger_status_event;
    wait_queue_head_t  gaugefw_status_event;
    bool low_battery;

    //return to system ++
    int pad_ac_status;
    int pre_pad_ac_status;
    int pad_bat_status;
    int pad_bat_temperature;
    int pad_bat_percentage;
    int pad_bat_capacity_level;
    int pad_bat_present;
    int pad_bat_voltage;
    int pad_bat_current;
    //return to system --

    int ec_suspend_status;
    int ec_fail_count;
    int base_fail_count;
};
static struct asus_pad_device_info *asus_pad_device;
DEFINE_MUTEX(ec_info_lock);

static int asus_ec_power_get_battery_property(struct power_supply *psy,
                                           enum power_supply_property psp,
                                           union power_supply_propval *val);
static int asus_ec_power_get_ac_property(struct power_supply *psy,
                                      enum power_supply_property psp,
                                      union power_supply_propval *val);

static enum power_supply_property asus_ec_power_ac_props[] = {
    POWER_SUPPLY_PROP_ONLINE,
};

static enum power_supply_property asus_ec_power_props[] = {
    POWER_SUPPLY_PROP_STATUS,
    POWER_SUPPLY_PROP_HEALTH,
    POWER_SUPPLY_PROP_PRESENT,
    POWER_SUPPLY_PROP_TECHNOLOGY,
    POWER_SUPPLY_PROP_VOLTAGE_NOW,
    POWER_SUPPLY_PROP_CURRENT_NOW,
    POWER_SUPPLY_PROP_CAPACITY,
    POWER_SUPPLY_PROP_CAPACITY_LEVEL,
    POWER_SUPPLY_PROP_TEMP,
};

static char *asus_ec_power_ac_supplied_to[] = {
    "battery",
    "pad_bat",
};

static struct power_supply asus_ec_power_supplies[] = {
    {
        .name = "pad_bat",
        .type = POWER_SUPPLY_TYPE_PAD_BAT,
        .properties = asus_ec_power_props,
        .num_properties = ARRAY_SIZE(asus_ec_power_props),
        .get_property = asus_ec_power_get_battery_property,
    },
    {
        .name = "pad_ac",
        .type = POWER_SUPPLY_TYPE_PAD_AC,
        .supplied_to = asus_ec_power_ac_supplied_to,
        .num_supplicants = ARRAY_SIZE(asus_ec_power_ac_supplied_to),
        .properties = asus_ec_power_ac_props,
        .num_properties = ARRAY_SIZE(asus_ec_power_ac_props),
        .get_property = asus_ec_power_get_ac_property,
    },
};

bool pad_pwr_supply()
{
    bool ret;

    /* return TRUE if gpio for pad insertion detected
       and charger received the valid external power
    */
    ret = !gpio_get_value(get_gpio_by_name("PAD_PLUG_IN_N")) &&
          !gpio_get_value(get_gpio_by_name("CHG_INOK#"));

    if (ret)
        pr_info("%s: TRUE\n", __func__);
    else
        pr_info("%s: FALSE\n", __func__);

    return ret;
}

#ifndef ME372CG_USER_BUILD
int pad_power_toggle_on_read(char *page, char **start, off_t off,
                    int count, int *eof, void *date)
{
    pr_info("%s:\n", __func__);

    /* recovery pad power supply - detect Pad present */
    if (!gpio_get_value(get_gpio_by_name("PAD_PLUG_IN_N")))
        AX_MicroP_set_VBusPower(1);
    return 0;
}
int pad_power_toggle_on_write(struct file *file, const char *buffer,
                    unsigned long count, void *data)
{ return 0; }
int pad_power_toggle_off_read(char *page, char **start, off_t off,
                    int count, int *eof, void *date)
{
    pr_info("%s:\n", __func__);

    /* cut off pad power supply */
    if (pad_pwr_supply())
        AX_MicroP_set_VBusPower(0);
    return 0;
}
int pad_power_toggle_off_write(struct file *file, const char *buffer,
                    unsigned long count, void *data)
{ return 0; }

int init_pad_power_toggle_on(void)
{
    struct proc_dir_entry *entry=NULL;

    entry = create_proc_entry("recpp", 0666, NULL);
    if (!entry) {
        pr_info("Unable to create pad_power_toggle_on\n");
        return -EINVAL;
    }
    entry->read_proc = pad_power_toggle_on_read;
    entry->write_proc = pad_power_toggle_on_write;

    return 0;
}
int init_pad_power_toggle_off(void)
{
    struct proc_dir_entry *entry=NULL;

    entry = create_proc_entry("cutpp", 0666, NULL);
    if (!entry) {
        pr_info("Unable to create pad_power_toggle_off\n");
        return -EINVAL;
    }
    entry->read_proc = pad_power_toggle_off_read;
    entry->write_proc = pad_power_toggle_off_write;

    return 0;
}
#else
int init_pad_power_toggle_on(void) { return 0; }
int init_pad_power_toggle_off(void) { return 0; }
#endif

static int get_ec_power_info(void)
{
    int ret, ret1, chrging_sta;
    u8 pad_bat_info[6];
    u8 pad_bat_info_soc[2];
    /* voltage and current from pad battery */
    u16 vol, cur;
    int _cur, tempr;
    int pad_charging_type;

    char pad_chrgr_str[3][8] = {
        "NONE",
        "PAD_AC",
        "PAD_USB",
    };
    char pad_chrging_str[3][5] = {
        "DISC",
        "CHRG",
        "FULL",
    };

    ret = AX_MicroP_getBatteryInfo(pad_bat_info);
    if (ret < 0) {
        pr_err("<EC_PWR> fail to get EC power battery info\n");
    }
    else {
        vol = pad_bat_info[1] << 8 | pad_bat_info[0];
        cur = pad_bat_info[3] << 8 | pad_bat_info[2];
        _cur = (int)(s16)cur;
        tempr = (int)(s8)pad_bat_info[4];

        ret = AX_MicroP_getBatterySoc(pad_bat_info_soc);
        if (ret < 0) {
            pr_err("<EC_PWR> fail to get EC power battery info soc\n");
        }
        else {
            ret = AX_MicroP_get_USBDetectStatus(Batt_P01);
            if (ret < 0) {
                pr_err("<EC_PWR> fail to get EC power charger type\n");
            }
            else {
                ret1 = AX_MicroP_get_ChargingStatus(Batt_P01);
                if (ret < 0) {
                    pr_err("<EC_PWR> fail to get EC power batt charging status\n");
                }
                else {
                    pad_charging_type = ret;

                    /* pad batt charging status */
                    switch(ret1) {
                    case P01_CHARGING_NO:
                        chrging_sta = POWER_SUPPLY_STATUS_DISCHARGING;
                        break;
                    case P01_CHARGING_ONGOING:
                        chrging_sta = POWER_SUPPLY_STATUS_CHARGING;
                        break;
                    case P01_CHARGING_FULL:
                        chrging_sta = POWER_SUPPLY_STATUS_FULL;
                        break;
                    }

                    pr_info("<EC_PWR> C:%u%%(%u%%) V:%umV C:%dmA T:%dC M:%d S:%s(%s) %s\n",
                        pad_bat_info_soc[1], pad_bat_info_soc[0],
                        vol,
                        _cur,
                        tempr,
                        pad_bat_info[5],
                        pad_chrgr_str[pad_charging_type],
                        pad_chrging_str[ret1],
                        pad_pwr_supply() ? "PAD POWER SUPPLYING..." :
                                           "*MISSING PAD POWER!*");

                    mutex_lock(&ec_info_lock);
                    asus_pad_device->pad_bat_status = chrging_sta;
                    asus_pad_device->pad_bat_voltage = vol;
                    asus_pad_device->pad_bat_current = _cur;
                    asus_pad_device->pad_bat_temperature = tempr;
                    asus_pad_device->pad_bat_percentage = pad_bat_info_soc[1];
                    mutex_unlock(&ec_info_lock);
                }
            }
        }
    }
    return ret;
}

static int ec_power_changed_all()
{
    int ret, pad_present, pad_ac_present;

    mutex_lock(&ec_info_lock);
    pad_present = asus_pad_device->pad_bat_present;
    pad_ac_present = asus_pad_device->pad_ac_status;
    mutex_unlock(&ec_info_lock);

    if (!pad_present) {
        pr_info("<EC_PWR> pad is not present!\n");
        return ret;
    }

    ret = get_ec_power_info();
    if (ret < 0) {
        pr_err("<EC_PWR> failed to get EC power info per 30s!\n");
        return ret;
    }

    if (pad_present)
        power_supply_changed(&asus_ec_power_supplies[0]);
    if (pad_ac_present)
        power_supply_changed(&asus_ec_power_supplies[1]);

    return ret;
}

static int ec_power_report(struct notifier_block *nb, unsigned long event, void *ptr)
{
    int ret;

    pr_info("------------------- %s -------------------: %d\n",
        __func__, event);

    if (!asus_pad_device) {
        pr_info("<EC_PWR> ec power driver not ready\n");
        return NOTIFY_OK;
    }

    switch (event) {
    case P01_ADD:
        mutex_lock(&ec_info_lock);
        asus_pad_device->pad_bat_present = 1;
        mutex_unlock(&ec_info_lock);
        ret = get_ec_power_info();
        power_supply_changed(&asus_ec_power_supplies[0]);
        break;

    case P01_REMOVE:
        mutex_lock(&ec_info_lock);
        asus_pad_device->pad_ac_status = 0;
        asus_pad_device->pad_bat_present = 0;
        mutex_unlock(&ec_info_lock);
        power_supply_changed(&asus_ec_power_supplies[1]);
        break;

    case P01_BATTERY_POWER_BAD:
        break;
    case P01_BATTERY_TO_CHARGING:
        break;
    case P01_BATTERY_TO_NON_CHARGING:
        break;
    case PAD_PHONEJACK_IN:
        break;
    case PAD_PHONEJACK_OUT:
        break;
    case P01_VOLUP_KEY_PRESSED:
        break;
    case P01_VOLUP_KEY_RELEASED:
        break;
    case P01_VOLDN_KEY_PRESSED:
        break;
    case P01_VOLDN_KEY_RELEASED:
        break;
    case P01_PWR_KEY_PRESSED:
        break;
    case P01_PWR_KEY_RELEASED:
        break;
    case P01_LIGHT_SENSOR:
        break;

    case P01_AC_IN:
        mutex_lock(&ec_info_lock);
        asus_pad_device->pad_ac_status = 1;
        asus_pad_device->pad_bat_present = 1;
        mutex_unlock(&ec_info_lock);
        ret = get_ec_power_info();
        power_supply_changed(&asus_ec_power_supplies[1]);
        break;

    case P01_USB_IN:
        ret = get_ec_power_info();
        break;

    case P01_AC_USB_OUT:
        mutex_lock(&ec_info_lock);
        asus_pad_device->pad_ac_status = 0;
        mutex_unlock(&ec_info_lock);
        ret = get_ec_power_info();
        power_supply_changed(&asus_ec_power_supplies[1]);
        break;

    case P01_DEAD:
        break;
    case PAD_UPDATE_FINISH:
        break;
    case PAD_EXTEND_CAP_SENSOR:
        break;
    case PAD_USB_OTG_ENABLE:
        break;
    case P01_LOW_BATTERY:
        ret = get_ec_power_info();
        power_supply_changed(&asus_ec_power_supplies[0]);
        break;
    }

    return NOTIFY_OK;
}
static struct notifier_block ec_power_notifier = {
    .notifier_call = ec_power_report,
};

static int asus_battery_driver_ready = 0;

static int asus_ec_power_get_ac_property(struct power_supply *psy,
                                      enum power_supply_property psp,
                                      union power_supply_propval *val)
{
    int ret = 0;

    mutex_lock(&ec_info_lock);

    switch (psp) {
    case POWER_SUPPLY_PROP_ONLINE:
        if (psy->type == POWER_SUPPLY_TYPE_PAD_AC) {
            val->intval = asus_pad_device->pad_ac_status;
        }
        else {
            ret = -EINVAL;
        }
        break;
    default:
        ret = -EINVAL;
        break;
    }

    mutex_unlock(&ec_info_lock);

    return ret;
}

static int asus_ec_power_get_battery_property(struct power_supply *psy,
                                           enum power_supply_property psp,
                                           union power_supply_propval *val)
{
    mutex_lock(&ec_info_lock);

    switch (psp) {
    case POWER_SUPPLY_PROP_STATUS:
        val->intval = asus_pad_device->pad_bat_status;
        break;
    case POWER_SUPPLY_PROP_HEALTH:
        val->intval = POWER_SUPPLY_HEALTH_GOOD;
        break;
    case POWER_SUPPLY_PROP_PRESENT:
        val->intval = asus_pad_device->pad_bat_present;
        break;
    case POWER_SUPPLY_PROP_TECHNOLOGY:
        val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
        break;
    case POWER_SUPPLY_PROP_VOLTAGE_NOW:
        val->intval = asus_pad_device->pad_bat_voltage;
        break;
    case POWER_SUPPLY_PROP_CURRENT_NOW:
        val->intval = asus_pad_device->pad_bat_current;
        break;
    case POWER_SUPPLY_PROP_CAPACITY:
        val->intval = asus_pad_device->pad_bat_percentage;
        break;
    case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
        val->intval = asus_pad_device->pad_bat_capacity_level;
        break;
    case POWER_SUPPLY_PROP_TEMP:
        val->intval = asus_pad_device->pad_bat_temperature;
        break;
    default:
        dbg_e("%s: some properties(%d) deliberately report errors.\n",__func__,psp);
        mutex_unlock(&ec_info_lock);
        return -EINVAL;
    }

    mutex_unlock(&ec_info_lock);

    return 0;
}

static int ec_report_worker(struct work_struct work)
{
    ec_power_changed_all();

    queue_delayed_work(asus_pad_device->ec_report_work_q,
        &asus_pad_device->ec_report_work,
        30*HZ);
}

static int ec_routine_report_init()
{
    int ret;

    pr_info("<EC_PWR> %s\n", __func__);

    INIT_DELAYED_WORK(&asus_pad_device->ec_report_work,
        ec_report_worker);
    asus_pad_device->ec_report_work_q =
        create_singlethread_workqueue("ec_pwr_report_wq");
    if (!asus_pad_device->ec_report_work_q) {
        pr_info("<EC_PWR> fail to create \"ec_pwr_report_wq\"");
        return -ENOMEM;
    }

    queue_delayed_work(asus_pad_device->ec_report_work_q,
        &asus_pad_device->ec_report_work,
        30*HZ);

    return 0;
}

static int __devinit asus_ec_power_probe(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    int ret;

    pr_info("++++++++++++++++ %s ++++++++++++++++\n", __func__);

    asus_pad_device = kzalloc(sizeof(struct asus_pad_device_info), GFP_KERNEL);
    if (!asus_pad_device) {
        ret = -ENOMEM;
        goto pad_dev_alloc_fail;
    }
    memset(asus_pad_device, 0, sizeof(*asus_pad_device));

    //Init
    asus_pad_device->pad_bat_present = 0;
    asus_pad_device->pad_bat_status = 3;
    asus_pad_device->pad_bat_percentage = 50;
    asus_pad_device->pad_bat_temperature = 270;
    asus_pad_device->pad_bat_voltage = 4000;
    asus_pad_device->pad_bat_current = 0;
    asus_pad_device->ec_suspend_status = 0;
    asus_pad_device->low_battery = false;

#if 0
    asus_pad_device->manufacture = BATT_LG;
    asus_pad_device->inok_gpio = get_gpio_by_name(CHARGER_INOK_NAME);
    asus_pad_device->batt_low_gpio = get_gpio_by_name(BATT_LOW_NAME);
#ifdef TX201LA_ENG_BUILD
    asus_pad_device->limit_charger_disable = true;
#endif
    init_waitqueue_head(&asus_pad_device->charger_status_event);
    init_waitqueue_head(&asus_pad_device->gaugefw_status_event);

    /* init wake lock in COS */
    wake_lock_init(&wakelock, WAKE_LOCK_SUSPEND, "asus_battery_power_wakelock");
    wake_lock_init(&wakelock_t, WAKE_LOCK_SUSPEND, "asus_battery_power_wakelock_timeout");

        /* prevent system from entering s3 in COS while AC charger is connected */
    if (entry_mode == 4) {
        if (cable_status == AC_IN) {
            if (!wake_lock_active(&wakelock)) {
                dbg_i(" %s: asus_battery_power_wakelock -> wake lock\n", __func__);
                wake_lock(&wakelock);
            }
        }
    }

    if (asus_pad_device->inok_gpio >=0) {
        ret = gpio_request_one(asus_pad_device->inok_gpio, GPIOF_IN, "CHG_INOK");
        if (ret <0) {
            dbg_e("request INOK gpio fail!\n");
            goto request_inok_gpio_fail;
        }
        asus_pad_device->inok_irq = gpio_to_irq(asus_pad_device->inok_gpio);
        ret = request_threaded_irq(asus_pad_device->inok_irq, asus_battery_inok_interrupt,
                                        asus_battery_inok_interrupt,
                                        IRQF_PERCPU | IRQF_NO_SUSPEND | IRQF_FORCE_RESUME |
					IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
					"CHG_INOK",dev);
        if (ret <0) {
            dbg_e("request INOK gpio as irq fail!\n");
            goto request_inok_irq_fail;
        }
    }

    if (asus_pad_device->batt_low_gpio >=0) {
        ret = gpio_request_one(asus_pad_device->batt_low_gpio, GPIOF_IN, BATT_LOW_NAME);
        asus_pad_device->batt_low_irq = gpio_to_irq(asus_pad_device->batt_low_gpio);
        if (ret <0) {
            dbg_e("request BATT_LOW gpio fail!\n");
            goto request_battlow_gpio_fail;
        }
        ret = request_threaded_irq(asus_pad_device->batt_low_irq, asus_battery_low_interrupt,
                                        asus_battery_low_interrupt,
                                        IRQF_TRIGGER_FALLING, BATT_LOW_NAME,dev);
        if (ret <0) {
            dbg_e("request battery low gpio as irq fail!\n");
            goto request_battlow_irq_fail;
        }
    }


    ret = power_supply_register(dev, &asus_pad_batt_supplies[PAD_BATTERY]);
    if (ret) {
        dbg_e("%s: failed to register %s\n", __func__,
                asus_pad_batt_supplies[PAD_BATTERY].name);
	goto batt_reg_fail;
    }

    ret = power_supply_register(dev, &asus_pad_batt_supplies[PAD_AC]);
    if (ret) {
        dbg_e("%s: failed to register %s\n", __func__,
                asus_pad_batt_supplies[PAD_AC].name);
        goto ac_reg_fail;
    }

    ret = power_supply_register(dev, &asus_pad_batt_supplies[PAD_USB]);
    if (ret) {
        pr_err("%s: failed to register %s\n", __func__,
                asus_pad_batt_supplies[PAD_USB].name);
        goto usb_reg_fail;
    }

    ret = power_supply_register(dev, &asus_pad_batt_supplies[BASE_BATTERY]);
    if (ret) {
        dbg_e("%s: failed to register %s\n", __func__,
                asus_pad_batt_supplies[BASE_BATTERY].name);
        goto base_batt_reg_fail;
    }

    ret = power_supply_register(dev, &asus_pad_batt_supplies[BASE_AC]);
    if (ret) {
        dbg_e("%s: failed to register %s\n", __func__,
                asus_pad_batt_supplies[BASE_AC].name);
        goto base_ac_reg_fail;
    }

    bat_ret = asus_pad_battery_get_info();
    if ( bat_ret == RET_GAUGE_FAIL || bat_ret == RET_EC_FAIL) {
        dbg_e("%s:fail to get battery info\n",__func__);
    }

    register_dock_attach_notifier(&attach_dock_notifier);
    register_dock_detach_notifier(&detach_dock_notifier);

    ite_gauge_stop_polling();
    asus_pad_device->fw_cfg_ver = bq27520_asus_battery_dev_read_fw_cfg_version();
    if ( asus_pad_device->fw_cfg_ver == ERROR_CODE_I2C_FAILURE)
        asus_pad_device->fw_cfg_ver = 0;
    dbg_i("fw_cfg_version:%d\n",asus_pad_device->fw_cfg_ver);
    asus_pad_device->chem_id = bq27520_asus_battery_dev_read_chemical_id();
    if ( asus_pad_device->chem_id == ERROR_CODE_I2C_FAILURE)
        asus_pad_device->chem_id = 0;
    dbg_i("chemical_id=%d\n", asus_pad_device->chem_id);
    asus_pad_device->df_ver = bq27520_asus_battery_dev_read_df();
    if ( asus_pad_device->df_ver == ERROR_CODE_I2C_FAILURE)
        asus_pad_device->df_ver = 0;
    dbg_i("df:%d\n",asus_pad_device->df_ver);
    ite_gauge_start_polling();

    queue_delayed_work(battery_work_queue, &battery_poll_data_work, FIRST_RUN_TIME*HZ);
#endif

    ret = power_supply_register(dev, &asus_ec_power_supplies[0]);
    if (ret) {
        dbg_e("%s: failed to register %s\n", __func__,
                asus_ec_power_supplies[0].name);
        goto ec_power_reg_bat_fail;
    }

    ret = power_supply_register(dev, &asus_ec_power_supplies[1]);
    if (ret) {
        dbg_e("%s: failed to register %s\n", __func__,
                asus_ec_power_supplies[1].name);
        goto ec_power_reg_ac_fail;
    }

    ret = init_pad_power_toggle_on();
    if (ret)
        goto ec_power_reg_ac_fail;
    ret = init_pad_power_toggle_off();
    if (ret)
        goto ec_power_reg_ac_fail;

    asus_battery_driver_ready = 1;
    register_microp_notifier(&ec_power_notifier);

    ret = ec_routine_report_init();
    if (ret < 0)
        goto ec_power_rountine_fail;

    return 0;

ec_power_rountine_fail:
ec_power_reg_ac_fail:
    power_supply_unregister(&asus_ec_power_supplies[0]);
ec_power_reg_bat_fail:
pad_dev_alloc_fail:
    kfree(asus_pad_device);
    return ret;
}

static int __devexit asus_battery_ec_remove(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;

#if 0
    wake_lock_destroy(&wakelock);
    wake_lock_destroy(&wakelock_t);
#endif

    return 0;
}

static int asus_pad_batt_suspend(struct platform_device *pdev, pm_message_t state)
{
    dbg_i("asus_pad_batt_suspend. \n");
    asus_pad_device->ec_suspend_status = 1;
    return 0;
}

static int asus_pad_batt_resume(struct platform_device *pdev)
{
    asus_pad_device->ec_suspend_status = 0;
#if 0
    asus_update_all(0.5);
#endif
    return 0;
}


static const struct platform_device_id asus_battery_ec_table[] = {
    {DRIVER_NAME, 1},
};

static const struct dev_pm_ops ec_pm_ops = {
    .suspend = asus_pad_batt_suspend,
    .resume = asus_pad_batt_resume,
};

static struct platform_driver asus_battery_ec_driver = {
    .driver = {
        .name  = DRIVER_NAME,
        .owner = THIS_MODULE,
        .pm    = &ec_pm_ops,
	},
    .probe      = asus_ec_power_probe,
    .remove     = __devexit_p(asus_battery_ec_remove),
    .id_table   = asus_battery_ec_table,
};

static int __init asus_ec_power_init(void)
{
    int rc;

    if (entry_mode == 5) return -1;
    pr_info("++++++++++++++++ %s ++++++++++++++++\n", __func__);

    rc = platform_driver_register(&asus_battery_ec_driver);
    if (rc < 0) {
        dbg_e("%s: FAIL: platform_driver_register. rc = %d\n", __func__, rc);
        goto register_fail;
    }

    return 0;

register_fail:
    return rc;
}

static void __exit asus_ec_power_exit(void)
{
    int i;
    unregister_microp_notifier(&ec_power_notifier);
#if 0
    unregister_dock_attach_notifier(&attach_dock_notifier);
    unregister_dock_detach_notifier(&detach_dock_notifier);
    for (i = 0; i < ARRAY_SIZE(asus_pad_batt_supplies); i++)
        power_supply_changed(&asus_pad_batt_supplies[i]);
    dbg_i("%s: 'changed' event sent, sleeping for 10 seconds...\n",
            __func__);
    ssleep(10);

    for (i = 0; i < ARRAY_SIZE(asus_pad_batt_supplies); i++)
        power_supply_unregister(&asus_pad_batt_supplies[i]);
#endif
    kfree(asus_pad_device);
#if 0
    destroy_workqueue(battery_work_queue);
#endif
    platform_driver_unregister(&asus_battery_ec_driver);
    asus_battery_driver_ready = 0;
}

module_init(asus_ec_power_init);
module_exit(asus_ec_power_exit);

MODULE_AUTHOR("ASUS BSP");
MODULE_DESCRIPTION("ec power driver");
MODULE_LICENSE("GPL v2");
