// SPDX-License-Identifier: GPL-2.0

#include <linux/errno.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/interrupt.h>

#define PM8916_PERPH_TYPE 0x04
#define PM8916_BMS_VM_TYPE 0x020D

#define PM8916_BMS_VM_STATUS1 0x08
#define PM8916_BMS_VM_FSM_STATE(x) ((x & 0b00111000) >> 3)
#define PM8916_BMS_VM_FSM_STATE_S2 0x2


#define PM8916_BMS_VM_EN_CTL 0x46
#define PM8916_BMS_ENABLED BIT(7)

#define PM8916_BMS_VM_FIFO_LENGTH_CTL 0x47
#define PM8916_BMS_VM_S1_SAMPLE_INTERVAL_CTL 0x55
#define PM8916_BMS_VM_S2_SAMPLE_INTERVAL_CTL 0x56
#define PM8916_BMS_VM_S3_S7_OCV_DATA0 0x6A
#define PM8916_BMS_VM_BMS_FIFO_REG_0_LSB 0xC0

// NOTE: downstream has a comment saying that using 1 fifo is broken in hardware
#define PM8916_BMS_VM_FIFO_COUNT 2 // 2 .. 8

struct pm8916_bms_vm_battery {
	struct device *dev;
	struct power_supply_desc desc;
	struct power_supply *battery;
	struct power_supply_battery_info info;
	struct regmap *regmap;
	unsigned int reg;
	unsigned int boot_ocv;
	unsigned int fake_ocv;
	unsigned int vbat_now;
};


static int pm8916_bms_vm_battery_get_property(struct power_supply *psy,
					 enum power_supply_property psp,
					 union power_supply_propval *val)
{
	struct pm8916_bms_vm_battery *bat = power_supply_get_drvdata(psy);
	struct power_supply_battery_info *info = &bat->info;
	int ret = 0, supplied;
	unsigned int tmp = 0;
	unsigned int vbat = 0;

	switch (psp) {
		case POWER_SUPPLY_PROP_VOLTAGE_NOW:
			ret = regmap_bulk_read(bat->regmap,
				bat->reg + PM8916_BMS_VM_BMS_FIFO_REG_0_LSB, &tmp, 2);

			if (ret)
				return ret;

			tmp *= 300;

			if (tmp == 19660500)
				vbat = bat->boot_ocv;
			else
				vbat = tmp - 100000; // FIXME Offset needs explaination
			break;

		default:
			break;
	}

	switch (psp) {
		case POWER_SUPPLY_PROP_STATUS:
			// Maybe not the best way but I just assume that if at least
			// one psu is online then it is charging
			supplied = power_supply_is_system_supplied();

			if (supplied < 0)
				return supplied;
			else if (supplied) {
				ret = regmap_write(bat->regmap, bat->reg + PM8916_BMS_VM_STATUS1, 0);
				ret = regmap_read(bat->regmap, bat->reg + PM8916_BMS_VM_STATUS1, &tmp);

				if (PM8916_BMS_VM_FSM_STATE(tmp) == PM8916_BMS_VM_FSM_STATE_S2)
					val->intval = POWER_SUPPLY_STATUS_FULL;
				else
					val->intval = POWER_SUPPLY_STATUS_CHARGING;
			}
			else
				val->intval = POWER_SUPPLY_STATUS_DISCHARGING;

			return 0;

		case POWER_SUPPLY_PROP_HEALTH:
			if (bat->vbat_now < info->voltage_min_design_uv)
				val->intval = POWER_SUPPLY_HEALTH_DEAD;
			else if (bat->vbat_now > info->voltage_max_design_uv)
				val->intval = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
			else
				val->intval = POWER_SUPPLY_HEALTH_GOOD;
			return 0;

		case POWER_SUPPLY_PROP_CAPACITY:
			val->intval = power_supply_batinfo_ocv2cap(info, bat->fake_ocv, 20);

			return 0;

		case POWER_SUPPLY_PROP_VOLTAGE_NOW:
			val->intval = bat->vbat_now;

			return 0;

		case POWER_SUPPLY_PROP_VOLTAGE_BOOT:
			val->intval = bat->boot_ocv;
			return 0;

		case POWER_SUPPLY_PROP_VOLTAGE_OCV:
			val->intval = bat->fake_ocv;
			return 0;

		case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
			val->intval = info->voltage_min_design_uv;
			return 0;

		case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
			val->intval = info->voltage_max_design_uv;
			return 0;

		default:
			return -EINVAL;
	};
}

static enum power_supply_property pm8916_bms_vm_battery_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_BOOT,
	POWER_SUPPLY_PROP_VOLTAGE_OCV,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CAPACITY,
};

static irqreturn_t pm8916_bms_vm_fifo_update_done_irq(int irq, void *data)
{
	struct pm8916_bms_vm_battery *bat = data;
	//struct power_supply_battery_info *info = &bat->info;
	int i, delta;
	unsigned int tmp = 0;
	int supplied;

	// FIXME Yes, this is bad
	// Should probably just bulk_read into array of u16 and schedule work
	// This will also help with automatic "recalibration" when bat is charged

	dev_warn(bat->dev, " ** Interrupt **\n");

	supplied = power_supply_is_system_supplied();
	if (supplied < 0)
		return IRQ_HANDLED;

	for (i = 0; i < PM8916_BMS_VM_FIFO_COUNT; i++) {
		tmp = 0;

		regmap_bulk_read(bat->regmap,
				bat->reg + PM8916_BMS_VM_BMS_FIFO_REG_0_LSB + i * 2, &tmp, 2);

		tmp = tmp * 300 - 100000;

		delta = tmp - bat->vbat_now;

		if ((supplied && delta > 0) || (!supplied && delta < 0))
			if (abs(delta) < 25000) // 0.025v
				bat->fake_ocv += delta;

		dev_warn(bat->dev, "  %d) %07u (d = %07d) [%s]\n", i, tmp, delta, (supplied ? "S" : "n"));

		bat->vbat_now = tmp;
	}

	power_supply_changed(bat->battery);

	return IRQ_HANDLED;
}

static int pm8916_bms_vm_battery_probe_dt(struct pm8916_bms_vm_battery *bat)
{
	struct device *dev = bat->dev;
	struct device_node *np = dev->of_node;

	int ret;
	unsigned int tmp;

	of_property_read_u32(np, "qcom,bms-vm-s1-sample-interval", &tmp);
	tmp = min_t(u32, tmp, 2550) / 10;
	ret = regmap_write(bat->regmap,
			   bat->reg + PM8916_BMS_VM_S1_SAMPLE_INTERVAL_CTL, tmp);

	of_property_read_u32(np, "qcom,bms-vm-s2-sample-interval", &tmp);
	tmp = min_t(u32, tmp, 2550) / 10;
	ret = regmap_write(bat->regmap,
			   bat->reg + PM8916_BMS_VM_S2_SAMPLE_INTERVAL_CTL, tmp);


	ret = regmap_write(bat->regmap, bat->reg + PM8916_BMS_VM_FIFO_LENGTH_CTL,
			   PM8916_BMS_VM_FIFO_COUNT << 4 | PM8916_BMS_VM_FIFO_COUNT);

	ret = regmap_write(bat->regmap,
			   bat->reg + PM8916_BMS_VM_EN_CTL, PM8916_BMS_ENABLED);

	ret = regmap_bulk_read(bat->regmap,
			       bat->reg + PM8916_BMS_VM_S3_S7_OCV_DATA0, &tmp, 2);
	bat->boot_ocv = bat->fake_ocv = bat->vbat_now = tmp * 300;


	return ret;

}

static int pm8916_bms_vm_battery_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct pm8916_bms_vm_battery *bat;
	struct power_supply_config psy_cfg = {};
	struct power_supply_desc *desc;
	int ret, irq;
	unsigned int tmp;

	bat = devm_kzalloc(dev, sizeof(*bat), GFP_KERNEL);
	if (!bat)
		return -ENOMEM;

	bat->dev = dev;

	bat->regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!bat->regmap)
		return -ENODEV;

	of_property_read_u32(dev->of_node, "reg", &bat->reg);

	irq = platform_get_irq(pdev, 0);
	if (irq > 0) {
		ret = devm_request_irq(dev, irq, pm8916_bms_vm_fifo_update_done_irq,
				       0, "pm8916_vm_bms", bat);
		if (ret)
			return ret;
	} else {
		if (irq == -EPROBE_DEFER)
			return -EPROBE_DEFER;
	}

	ret = regmap_bulk_read(bat->regmap, bat->reg + PM8916_PERPH_TYPE, &tmp, 2); // FIXME
	if (ret)  {
		dev_err(dev, "Unable to communicate with device: %d\n", ret);
		return ret;
	}

	if (tmp != PM8916_BMS_VM_TYPE) {
		dev_err(dev, "Device reported wrong type: 0x%X\n", tmp);
		return -ENODEV;
	}

	ret = pm8916_bms_vm_battery_probe_dt(bat);
	if (ret) {
		dev_err(dev, "Error while parsing device tree: %d\n", ret);
		return ret;
	}

	desc = &bat->desc;
	desc->name = "pm8916-bms-vm";
	desc->type = POWER_SUPPLY_TYPE_BATTERY;
	desc->properties = pm8916_bms_vm_battery_properties;
	desc->num_properties = ARRAY_SIZE(pm8916_bms_vm_battery_properties);
	desc->get_property = pm8916_bms_vm_battery_get_property;
	psy_cfg.drv_data = bat;
	psy_cfg.of_node = dev->of_node;

	bat->battery = devm_power_supply_register(dev, desc, &psy_cfg);
	if (IS_ERR(bat->battery)) {
		dev_err(dev, "Unable to register battery\n");
		return PTR_ERR(bat->battery);
	}

	ret = power_supply_get_battery_info(bat->battery, &bat->info);
	if (ret) {
		dev_err(dev, "Unable to get battery info: %d\n", ret);
		return ret;
	}

	dev_warn(dev, "Expect more Magic!\n");

	return 0;
}

static const struct of_device_id pm8916_bms_vm_battery_of_match[] = {
	{ .compatible = "qcom,pm8916-bms-vm", },
	{ },
};
MODULE_DEVICE_TABLE(of, pm8916_bms_vm_battery_of_match);

static struct platform_driver pm8916_bms_vm_battery_driver = {
	.driver = {
		.name = "pm8916-bms-vm",
		.of_match_table = of_match_ptr(pm8916_bms_vm_battery_of_match),
	},
	.probe = pm8916_bms_vm_battery_probe,
};
module_platform_driver(pm8916_bms_vm_battery_driver);

MODULE_DESCRIPTION("pm8916 BMS-VM driver");
MODULE_AUTHOR("Nikita Travkin <nikitos.tr@gmail.com>");
MODULE_LICENSE("GPL");
