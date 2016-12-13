/*
 * Fake Power supply driver for android.
 */

#include <linux/module.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/types.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <asm/io.h>

static struct platform_device *fake_battery_pdev;

static int ac_online		= 1;
static int battery_status	= POWER_SUPPLY_STATUS_CHARGING;
static int battery_capacity	= 100;

static int fake_battery_bat_get_property(struct power_supply *psy,
					 enum power_supply_property psp,
					 union power_supply_propval *val)
{
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = battery_status;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = POWER_SUPPLY_HEALTH_GOOD;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = battery_capacity;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = 20;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = 5;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int fake_battery_ac_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = ac_online;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static enum power_supply_property fake_battery_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CAPACITY,
};

static enum power_supply_property fake_battery_ac_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static struct power_supply fake_battery_bat = {
	.name = "battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties 	=  fake_battery_battery_props,
	.num_properties = ARRAY_SIZE(fake_battery_battery_props),
	.get_property	= fake_battery_bat_get_property,
	.use_for_apm 	= 1,
};

static struct power_supply fake_battery_ac = {
	.name = "ac",
	.type = POWER_SUPPLY_TYPE_MAINS,
	.properties 	=  fake_battery_ac_props,
	.num_properties = ARRAY_SIZE(fake_battery_ac_props),
	.get_property	= fake_battery_ac_get_property,
};

static struct power_supply fake_battery_usb = {
	.name = "usb",
	.type = POWER_SUPPLY_TYPE_MAINS,
	.properties 	=  fake_battery_ac_props,
	.num_properties = ARRAY_SIZE(fake_battery_ac_props),
	.get_property	= fake_battery_ac_get_property,
};

#define MAX_KEYLENGTH 25
struct battery_property_map {
	int value;
	char const *key;
};

static struct battery_property_map map_online[] = {
	{ 0,  "off"  },
	{ 1,  "on" },
	{ -1, NULL  },
};

static struct battery_property_map map_status[] = {
	{ POWER_SUPPLY_STATUS_CHARGING,     "charging"     },
	{ POWER_SUPPLY_STATUS_DISCHARGING,  "discharging"  },
	{ POWER_SUPPLY_STATUS_NOT_CHARGING, "not-charging" },
	{ POWER_SUPPLY_STATUS_FULL,         "full"         },
	{ -1,                               NULL           },
};

static int map_get_value(struct battery_property_map *map, const char *key,
				int def_val)
{
	char buf[MAX_KEYLENGTH];
	int cr;

	strncpy(buf, key, MAX_KEYLENGTH);
	buf[MAX_KEYLENGTH - 1] = '\0';

	cr = strnlen(buf, MAX_KEYLENGTH) - 1;
	if (buf[cr] == '\n')
		buf[cr] = '\0';

	while (map->key) {
		if (strncasecmp(map->key, buf, MAX_KEYLENGTH) == 0)
			return map->value;
		map++;
	}

	return def_val;
}

static const char *map_get_key(struct battery_property_map *map, int value,
				const char *def_key)
{
	while (map->key) {
		if (map->value == value)
			return map->key;
		map++;
	}

	return def_key;
}


static int param_set_ac_online(const char *key, const struct kernel_param *kp)
{
	ac_online = map_get_value(map_online, key, ac_online);
	power_supply_changed(&fake_battery_ac);
	return 0;
}

static int param_get_ac_online(char *buffer, const struct kernel_param *kp)
{
	strcpy(buffer, map_get_key(map_online, ac_online, "unknown"));
	return strlen(buffer);
}

static struct kernel_param_ops param_ops_ac_online = {
	.set = param_set_ac_online,
	.get = param_get_ac_online,
};

static int param_set_battery_status(const char *key, 
				    const struct kernel_param *kp)
{
	battery_status = map_get_value(map_status, key, battery_status);
	power_supply_changed(&fake_battery_bat);
	return 0;
}

static int param_get_battery_status(char *buffer, const struct kernel_param *kp)
{
	strcpy(buffer, map_get_key(map_status, battery_status, "unknown"));
	return strlen(buffer);
}

static struct kernel_param_ops param_ops_battery_status = {
	.set = param_set_battery_status,
	.get = param_get_battery_status,
};

static int param_set_battery_capacity(const char *key, 
				      const struct kernel_param *kp)
{
	int tmp;

	if (1 != sscanf(key, "%d", &tmp))
		return -EINVAL;

	battery_capacity = tmp;
	power_supply_changed(&fake_battery_bat);
	return 0;
}

#define param_get_battery_capacity param_get_int

static struct kernel_param_ops param_ops_battery_capacity = {
	.set = param_set_battery_capacity,
	.get = param_get_battery_capacity,
};

#define param_check_ac_online(name, p) __param_check(name, p, void);
#define param_check_battery_status(name, p) __param_check(name, p, void);
#define param_check_battery_capacity(name, p) __param_check(name, p, void);

module_param(ac_online, ac_online, 0644);
MODULE_PARM_DESC(ac_online, "AC charging state <on|off>");

module_param(battery_status, battery_status, 0644);
MODULE_PARM_DESC(battery_status,
	"battery status <charging|discharging|not-charging|full>");

module_param(battery_capacity, battery_capacity, 0644);
MODULE_PARM_DESC(battery_capacity, "battery capacity (percentage)");

static int __init fake_battery_init(void)
{
	int ret = 0;
	
	fake_battery_pdev = platform_device_register_simple("battery", 0, NULL, 0);
	if (IS_ERR(fake_battery_pdev))
		return PTR_ERR(fake_battery_pdev);

	ret = power_supply_register(&fake_battery_pdev->dev, &fake_battery_bat);
	if (ret)
		goto bat_failed;

	ret = power_supply_register(&fake_battery_pdev->dev, &fake_battery_ac);
	if (ret)
		goto ac_failed;

	ret = power_supply_register(&fake_battery_pdev->dev, &fake_battery_usb);
	if (ret)
		goto usb_failed;

	goto success;

bat_failed:
	power_supply_unregister(&fake_battery_bat);
ac_failed:
	power_supply_unregister(&fake_battery_ac);
usb_failed:
	power_supply_unregister(&fake_battery_usb);
	platform_device_unregister(fake_battery_pdev);
success:
	return ret;
}

static void __exit fake_battery_exit(void)
{
	power_supply_unregister(&fake_battery_bat);
	power_supply_unregister(&fake_battery_ac);
	power_supply_unregister(&fake_battery_usb);
	platform_device_unregister(fake_battery_pdev);
}

module_init(fake_battery_init);
module_exit(fake_battery_exit);
MODULE_AUTHOR("jean-baptiste.dubois@parrot.com");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Fake Battery driver for Android");

