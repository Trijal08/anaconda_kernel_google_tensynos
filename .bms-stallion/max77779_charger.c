/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2023-2025 Google LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/debugfs.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/spmi.h>
#include <linux/thermal.h>

#include "google_bms.h"
#include "google_bms_usecase.h"
#include "max77779.h"
#include "max77779_charger.h"

#define BATOILO_DET_30US 0x4
#define MAX77779_DEFAULT_MODE	MAX77779_CHGR_MODE_ALL_OFF
#define CHG_TERM_VOLT_DEBOUNCE	200
#define MAX77779_OTG_5000_MV 5000
#define GS201_OTG_DEFAULT_MV MAX77779_OTG_5000_MV

/* CHG_DETAILS_01:CHG_DTLS */
#define CHGR_DTLS_DEAD_BATTERY_MODE			0x00
#define CHGR_DTLS_FAST_CHARGE_CONST_CURRENT_MODE	0x01
#define CHGR_DTLS_FAST_CHARGE_CONST_VOLTAGE_MODE	0x02
#define CHGR_DTLS_TOP_OFF_MODE				0x03
#define CHGR_DTLS_DONE_MODE				0x04
#define CHGR_DTLS_TIMER_FAULT_MODE			0x06
#define CHGR_DTLS_DETBAT_HIGH_SUSPEND_MODE		0x07
#define CHGR_DTLS_OFF_MODE				0x08
#define CHGR_DTLS_OFF_HIGH_TEMP_MODE			0x0a
#define CHGR_DTLS_OFF_WATCHDOG_MODE			0x0b
#define CHGR_DTLS_OFF_JEITA				0x0c
#define CHGR_DTLS_OFF_TEMP				0x0d

#define CHGR_CHG_CNFG_12_VREG_4P6V			0x1
#define CHGR_CHG_CNFG_12_VREG_4P7V			0x2

#define WCIN_INLIM_T					(5000)
#define WCIN_INLIM_HEADROOM_MA				(50000)
#define WCIN_INLIM_STEP_MV				(25000)
#define MAX77779_GPIO_WCIN_INLIM_EN			0
#define MAX77779_NUM_GPIOS				1

#define WCIN_INLIM_VOTER				"WCIN_INLIM"

#define MAX77779_CHG_NUM_REGS (MAX77779_CHG_CUST_TM - MAX77779_CHG_CHGIN_I_ADC_L + 1)

#define MAX77779_DEFAULT_CV_MARGIN			(20000)
#define MAX77779_DEFAULT_CV_DEBOUNCE			(10000)
#define MAX77779_DEFAULT_CV_DCR				(30)

/*
 * int[0]
 *  CHG_INT_AICL_I	(0x1 << 7)
 *  CHG_INT_CHGIN_I	(0x1 << 6)
 *  CHG_INT_WCIN_I	(0x1 << 5)
 *  CHG_INT_CHG_I	(0x1 << 4)
 *  CHG_INT_BAT_I	(0x1 << 3)
 *  CHG_INT_INLIM_I	(0x1 << 2)
 *  CHG_INT_THM2_I	(0x1 << 1)
 *  CHG_INT_BYP_I	(0x1 << 0)
 *
 * int[1]
 *  CHG_INT2_INSEL_I		(0x1 << 7)
 *  CHG_INT2_COP_LIMIT_WD_I	(0x1 << 6)
 *  CHG_INT2_COP_ALERT_I	(0x1 << 5)
 *  CHG_INT2_COP_WARN_I		(0x1 << 4)
 *  CHG_INT2_CHG_STA_CC_I	(0x1 << 3)
 *  CHG_INT2_CHG_STA_CV_I	(0x1 << 2)
 *  CHG_INT2_CHG_STA_TO_I	(0x1 << 1)
 *  CHG_INT2_CHG_STA_DONE_I	(0x1 << 0)
 *
 *
 * these 3 cause unnecessary chatter at EOC due to the interaction between
 * the CV and the IIN loop:
 *   MAX77779_CHG_INT2_MASK_CHG_STA_CC_M |
 *   MAX77779_CHG_INT2_MASK_CHG_STA_CV_M |
 *   MAX77779_CHG_INT_MASK_CHG_M
 *
 * NOTE: don't use this to write to the interrupt mask register. Read/write the
 * MAX77779_CHG_INT_MASK because external interrupt handlers can mask/unmask their
 * own bits.
 *
 * This array only contains the internally handled interrupts. It doesn't take into
 * account externally registered interrupts
 */
static u8 max77779_int_mask[MAX77779_CHG_INT_COUNT] = {
	~(MAX77779_CHG_INT_CHGIN_I_MASK |
	  MAX77779_CHG_INT_WCIN_I_MASK |
	  MAX77779_CHG_INT_BAT_I_MASK |
	  MAX77779_CHG_INT_THM2_I_MASK),
	(u8)~(MAX77779_CHG_INT2_INSEL_I_MASK |
	  MAX77779_CHG_INT2_CHG_STA_TO_I_MASK |
	  MAX77779_CHG_INT2_CHG_STA_DONE_I_MASK)
};

static int max77779_is_limited(struct max77779_chgr_data *data);
static int max77779_wcin_current_now(struct max77779_chgr_data *data, int *iic);
static int max77779_current_check_mode(struct max77779_chgr_data *data);

static inline int max77779_reg_read(struct max77779_chgr_data *data, uint8_t reg,
				    uint8_t *val)
{
	int ret, ival;

	ret = regmap_read(data->regmap, reg, &ival);
	if (ret == 0)
		*val = 0xFF & ival;

	return ret;
}

static bool max77779_chg_is_protected(uint8_t reg)
{
	switch(reg) {
	case MAX77779_CHG_CNFG_01:
	case MAX77779_CHG_CNFG_03:
	case MAX77779_CHG_CNFG_07 ... MAX77779_CHG_CNFG_08:
	case MAX77779_CHG_CNFG_13 ... MAX77779_BAT_OILO2_CNFG_3:
	case MAX77779_CHG_CUST_TM:
		return true;
	default:
		return false;
	}
}

/*
 * 1 if changed, 0 if not changed or not protected, or < 0 on error
 * Must call this function with prot disabled, do write IO, then call this function
 * with prot enabled
 */
static int max77779_chg_prot(struct max77779_chgr_data *data, uint8_t reg, int count, bool enable)
{
	const u8 value = enable ? 0 : MAX77779_CHG_CNFG_06_CHGPROT_MASK;
	bool changed, is_protected = false;
	int ret, i;

	if (count < 1)
		return -EINVAL;

	for (i = 0; i < count; i++) {
		if (is_protected)
			break;
		is_protected |= max77779_chg_is_protected(reg + i);
	}

	if (!is_protected)
		return 0;

	if (!enable)
		mutex_lock(&data->prot_lock);
	ret = regmap_update_bits_check(data->regmap, MAX77779_CHG_CNFG_06,
				       MAX77779_CHG_CNFG_06_CHGPROT_MASK,
				       value,
				       &changed);
	if (ret)
		dev_err(data->dev, "error modifying protection bits reg:0x%x count:%d "
			"enable:%d ret:%d\n", reg, count, enable, ret);
	if (enable || ret)
		mutex_unlock(&data->prot_lock);

	return ret ? ret : changed;
}

static inline int max77779_reg_write(struct max77779_chgr_data *data, uint8_t reg,
				     uint8_t val)
{
	int ret, prot;

	prot = max77779_chg_prot(data, reg, 1, false);
	if (prot < 0)
		return prot;

	ret = regmap_write(data->regmap, reg, val);

	prot = max77779_chg_prot(data, reg, 1, true);
	if (prot < 0)
		return prot;

	return ret;
}

static inline int max77779_readn(struct max77779_chgr_data *data, uint8_t reg,
				 uint8_t *val, int count)
{
	return regmap_bulk_read(data->regmap, reg, val, count);
}

static inline int max77779_writen(struct max77779_chgr_data *data, uint8_t reg, /* NOTYPO */
				  const uint8_t *val, int count)
{
	int ret, prot;

	prot = max77779_chg_prot(data, reg, count, false);
	if (prot < 0)
		return prot;

	ret = regmap_bulk_write(data->regmap, reg, val, count);

	prot = max77779_chg_prot(data, reg, count, true);
	if (prot < 0)
		return prot;

	return ret;
}

static inline int max77779_reg_update(struct max77779_chgr_data *data,
				      uint8_t reg, uint8_t msk, uint8_t val)
{
	int ret, prot;

	prot = max77779_chg_prot(data, reg, 1, false);
	if (prot < 0)
		return prot;

	ret = regmap_write_bits(data->regmap, reg, msk, val); /* forces update */

	prot = max77779_chg_prot(data, reg, 1, true);
	if (prot < 0)
		return prot;

	return ret;
}

static inline int max77779_reg_update_verify(struct max77779_chgr_data *data,
					     uint8_t reg, uint8_t msk, uint8_t val)
{
	int ret;
	uint8_t tmp;

	ret = max77779_reg_update(data, reg, msk, val);
	if (ret)
		return ret;

	ret = max77779_reg_read(data, reg, &tmp);
	if (ret)
		return ret;

	return ((tmp & msk) == val) ? 0 : -EINVAL;
}

static int max77779_chg_mode_write_locked(struct max77779_chgr_data *data,
					  enum max77779_charger_modes mode)
{
	/* The io lock should be held before you call this to protect the mode register */
	return max77779_reg_update(data, MAX77779_CHG_CNFG_00,
				   MAX77779_CHG_CNFG_00_MODE_MASK,
				   mode);
}

static int max77779_init_check(struct max77779_chgr_data *data)
{
	int ret = 0;

	pm_runtime_get_sync(data->dev);
	if (!data->init_complete)
		ret = -EAGAIN;
	pm_runtime_put_sync(data->dev);

	return ret;
}

/* ----------------------------------------------------------------------- */
int max77779_external_chg_reg_read(struct device *dev, uint8_t reg, uint8_t *val)
{
	struct max77779_chgr_data *data = dev_get_drvdata(dev);

	if (!data || !data->regmap)
		return -ENODEV;

	if (max77779_init_check(data))
		return -EAGAIN;

	return max77779_reg_read(data, reg, val);
}
EXPORT_SYMBOL_GPL(max77779_external_chg_reg_read);

int max77779_external_chg_reg_write(struct device *dev, uint8_t reg, uint8_t val)
{
	struct max77779_chgr_data *data = dev_get_drvdata(dev);

	if (!data || !data->regmap)
		return -ENODEV;

	if (max77779_init_check(data))
		return -EAGAIN;

	return max77779_reg_write(data, reg, val);
}
EXPORT_SYMBOL_GPL(max77779_external_chg_reg_write);

int max77779_external_chg_reg_update(struct device *dev, u8 reg, u8 mask, u8 value)
{
	struct max77779_chgr_data *data = dev_get_drvdata(dev);

	if (!data || !data->regmap)
		return -ENODEV;

	if (max77779_init_check(data))
		return -EAGAIN;

	return max77779_reg_update(data, reg, mask, value);
}
EXPORT_SYMBOL_GPL(max77779_external_chg_reg_update);

int max77779_external_chg_mode_write(struct device *dev, enum max77779_charger_modes mode)
{
	int ret;
	struct max77779_chgr_data *data = dev_get_drvdata(dev);

	if (!data)
		return -ENODEV;

	/* Protect mode register */
	mutex_lock(&data->io_lock);
	ret = max77779_chg_mode_write_locked(data, mode);
	mutex_unlock(&data->io_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(max77779_external_chg_mode_write);

int max77779_external_chg_insel_write(struct device *dev, u8 mask, u8 value)
{
	int ret;
	struct max77779_chgr_data *data = dev_get_drvdata(dev);

	mutex_lock(&data->io_lock);
	ret = max77779_external_chg_reg_update(dev, MAX77779_CHG_CNFG_12, mask, value);
	mutex_unlock(&data->io_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(max77779_external_chg_insel_write);

int max77779_external_chg_insel_read(struct device *dev, u8 *value)
{
	return max77779_external_chg_reg_read(dev, MAX77779_CHG_CNFG_12, value);
}
EXPORT_SYMBOL_GPL(max77779_external_chg_insel_read);

/* ----------------------------------------------------------------------- */

static struct device* max77779_get_i2c_dev(struct device_node *dn)
{
	struct i2c_client *client;

	client = of_find_i2c_device_by_node(dn);

	return client ? &client->dev : NULL;
}

static struct device* max77779_get_spmi_dev(struct device_node *dn)
{
	struct spmi_device *sdev;

	sdev = spmi_device_from_of(dn);

	return sdev ? &sdev->dev : NULL;
}

struct device* max77779_get_dev(struct device *dev, const char* name)
{
	struct device_node *dn;
	struct device* d;

	dn = of_parse_phandle(dev->of_node, name, 0);
	if (!dn)
		return NULL;

	d = max77779_get_i2c_dev(dn);
	if (d) {
		goto ret;
	}

	d = max77779_get_spmi_dev(dn);

ret:
	of_node_put(dn);
	return d;
}
EXPORT_SYMBOL_GPL(max77779_get_dev);

/* Modified version of irq_of_parse_and_map */
int max77779_irq_of_parse_and_map(struct device_node *dev, int index)
{
	struct of_phandle_args oirq;
	int ret;

	ret = of_irq_parse_one(dev, index, &oirq);
	if (ret)
		return 0;

	ret = irq_create_of_mapping(&oirq);

	return !ret ? -EPROBE_DEFER : ret;
}
EXPORT_SYMBOL_GPL(max77779_irq_of_parse_and_map);

static struct power_supply* max77779_get_fg_psy(struct max77779_chgr_data *chg)
{
	if (!chg->fg_psy)
		chg->fg_psy = power_supply_get_by_name("max77779fg");
	if (!chg->fg_psy)
		chg->fg_psy = power_supply_get_by_name("dualbatt");

	return chg->fg_psy;
}

static int max77779_read_vbatt(struct max77779_chgr_data *data, int *vbatt)
{
	union power_supply_propval val;
	struct power_supply *fg_psy;
	int ret = 0;

	fg_psy = max77779_get_fg_psy(data);
	if (!fg_psy) {
		dev_err(data->dev, "Couldn't get fg_psy\n");
		return -EIO;
	}

	ret = power_supply_get_property(fg_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, &val);
	if (ret < 0)
		dev_err(data->dev, "Couldn't get VOLTAGE_NOW, ret=%d\n", ret);
	else
		*vbatt = val.intval;

	return ret;
}

static int max77779_read_ibat(struct max77779_chgr_data *data, int *ibat)
{
	union power_supply_propval val;
	struct power_supply *fg_psy;
	int ret = 0;

	fg_psy = max77779_get_fg_psy(data);
	if (!fg_psy) {
		dev_err(data->dev, "Couldn't get fg_psy\n");
		return -EIO;
	}

	ret = power_supply_get_property(fg_psy, POWER_SUPPLY_PROP_CURRENT_NOW, &val);
	if (ret < 0)
		dev_err(data->dev, "Couldn't get VOLTAGE_NOW, ret=%d\n", ret);
	else
		*ibat = val.intval;

	return ret;
}

#define MAX77779_WCIN_RAW_TO_UV 625

static int max77779_read_wcin(struct max77779_chgr_data *data, int *vbyp)
{
	u16 tmp;
	int ret;

	ret = max77779_readn(data, MAX77779_CHG_WCIN_V_ADC_L, (uint8_t*)&tmp, 2);
	if (ret) {
		pr_err("Failed to read %x\n", MAX77779_CHG_WCIN_V_ADC_L);
		return ret;
	}

	*vbyp = tmp * MAX77779_WCIN_RAW_TO_UV;
	return 0;
}

/* ----------------------------------------------------------------------- */

/* set WDTEN in CHG_CNFG_15 (0xCB), tWD = 80s */
static int max77779_wdt_enable(struct max77779_chgr_data *data, bool enable)
{
	return max77779_reg_update_verify(data, MAX77779_CHG_CNFG_15,
					  MAX77779_CHG_CNFG_15_WDTEN_MASK,
					  _max77779_chg_cnfg_15_wdten_set(0, enable));
}

/* First step to convert votes to a usecase and a setting for mode */
static int max77779_foreach_callback(void *data, const char *reason,
				     void *vote)
{
	struct max77779_foreach_cb_data *cb_data = data;
	int mode = _bms_usecase_mode_get((long)vote); /* max77779_mode is an int election */

	switch (mode) {
	/* Direct raw modes last come fist served */
	case MAX77779_CHGR_MODE_ALL_OFF:
	case MAX77779_CHGR_MODE_BUCK_ON:
	case MAX77779_CHGR_MODE_CHGR_BUCK_ON:
	case MAX77779_CHGR_MODE_BOOST_UNO_ON:
	case MAX77779_CHGR_MODE_BOOST_ON:
	case MAX77779_CHGR_MODE_OTG_BOOST_ON:
	case MAX77779_CHGR_MODE_BUCK_BOOST_UNO_ON:
	case MAX77779_CHGR_MODE_CHGR_BUCK_BOOST_UNO_ON:
	case MAX77779_CHGR_MODE_OTG_BUCK_BOOST_ON:
	case MAX77779_CHGR_MODE_CHGR_OTG_BUCK_BOOST_ON:
		pr_debug("%s: RAW vote=0x%x\n", __func__, mode);
		if (cb_data->use_raw)
			break;
		cb_data->raw_value = mode;
		cb_data->reason = reason;
		cb_data->use_raw = true;
		break;

	/* SYSTEM modes can add complex transactions */

	/* MAX77779: on disconnect */
	case GBMS_CHGR_MODE_STBY_ON:
		if (!cb_data->stby_on)
			cb_data->reason = reason;
		pr_debug("%s: STBY_ON %s vote=0x%x\n",
			 __func__, reason ? reason : "<>", mode);
		cb_data->stby_on += 1;
		break;
	/* USB+WLCIN, factory only */
	case GBMS_CHGR_MODE_USB_WLC_RX:
		pr_debug("%s: USB_WLC_RX %s vote=0x%x\n",
			 __func__, reason ? reason : "<>", mode);
		if (!cb_data->usb_wlc)
			cb_data->reason = reason;
		cb_data->usb_wlc += 1;
		break;

	/* input_suspend => 0 ilim */
	case GBMS_CHGR_MODE_CHGIN_OFF:
		if (!cb_data->chgin_off)
			cb_data->reason = reason;
		pr_debug("%s: CHGIN_OFF %s vote=0x%x\n", __func__,
			 reason ? reason : "<>", mode);
		cb_data->chgin_off += 1;
		break;
	/* input_suspend => DC_SUSPEND */
	case GBMS_CHGR_MODE_WLCIN_OFF:
		if (!cb_data->wlcin_off)
			cb_data->reason = reason;
		pr_debug("%s: WLCIN_OFF %s vote=0x%x\n", __func__,
			 reason ? reason : "<>", mode);
		cb_data->wlcin_off += 1;
		break;
	/* MAX77779: charging on via CC_MAX (needs inflow, buck_on on) */
	case GBMS_CHGR_MODE_CHGR_BUCK_ON:
		if (!cb_data->chgr_on)
			cb_data->reason = reason;
		pr_debug("%s: CHGR_BUCK_ON %s vote=0x%x\n", __func__,
			 reason ? reason : "<>", mode);
		cb_data->chgr_on += 1;
		break;

	/* USB: present, charging controlled via GBMS_CHGR_MODE_CHGR_BUCK_ON */
	case GBMS_USB_BUCK_ON:
		if (!cb_data->buck_on)
			cb_data->reason = reason;
		pr_debug("%s: BUCK_ON %s vote=0x%x\n", __func__,
			 reason ? reason : "<>", mode);
		cb_data->buck_on += 1;
		break;
	/* USB: OTG, source, fast role swap case */
	case GBMS_USB_OTG_FRS_ON:
		if (!cb_data->frs_on)
			cb_data->reason = reason;
		pr_debug("%s: FRS_ON vote=0x%x\n", __func__, mode);
		cb_data->frs_on += 1;
		break;
	/* USB: boost mode, source, normally external boost */
	case GBMS_USB_OTG_ON:
		if (!cb_data->otg_on)
			cb_data->reason = reason;
		pr_debug("%s: OTG_ON %s vote=0x%x\n", __func__,
			 reason ? reason : "<>", mode);
		cb_data->otg_on += 1;
		break;
	/* DC Charging: mode=0, set CP_EN */
	case GBMS_CHGR_MODE_CHGR_DC_USB:
		if (!cb_data->dc_on)
			cb_data->reason = reason;
		pr_debug("%s: DC_ON_USB vote=0x%x\n", __func__, mode);
		cb_data->dc_on = GBMS_CHGR_SEL_USB;
		break;
	case GBMS_CHGR_MODE_CHGR_DC_WLC:
		if (!cb_data->dc_on)
			cb_data->reason = reason;
		pr_debug("%s: DC_ON_WLC vote=0x%x\n", __func__, mode);
		cb_data->dc_on = GBMS_CHGR_SEL_WIRELESS;
		break;
	/* WLC Tx */
	case GBMS_CHGR_MODE_WLC_TX:
		if (!cb_data->wlc_tx)
			cb_data->reason = reason;
		pr_debug("%s: WLC_TX vote=%x\n", __func__, mode);
		cb_data->wlc_tx += 1;
		break;

	/* WLC_RX */
	case GBMS_CHGR_MODE_WLC_RX:
		if (!cb_data->wlc_rx)
			cb_data->reason = reason;
		pr_debug("%s: WLC_RX vote=%x\n", __func__, mode);
		cb_data->wlc_rx += 1;
		break;
	case GBMS_CHGR_MODE_FWUPDATE_BOOST_ON:
		pr_debug("%s: FWUPDATE vote=%x\n", __func__, mode);
		cb_data->fwupdate_on |= GSU_MODE_FWUPDATE_MASK;
		break;
	case GBMS_CHGR_MODE_WLC_FWUPDATE:
		pr_debug("%s: WLC FWUPDATE vote=%x\n", __func__, mode);
		cb_data->fwupdate_on |= GSU_MODE_WLC_FWUPDATE_MASK;
		break;
	case GBMS_POGO_VIN:
		if (!cb_data->pogo_vin)
			cb_data->reason = reason;
		pr_debug("%s: POGO VIN vote=%x\n", __func__, mode);
		cb_data->pogo_vin += 1;
		break;
	case GBMS_POGO_VOUT:
		if (!cb_data->pogo_vout)
			cb_data->reason = reason;
		pr_debug("%s: POGO VOUT vote=%x\n", __func__, mode);
		cb_data->pogo_vout += 1;
		break;
	default:
		pr_err("mode=%x not supported\n", mode);
		break;
	}

	return 0;
}

/*
 * It could use cb_data->charge_done to turn off charging.
 * TODO: change chgr_on=>2 to (cc_max && chgr_ena)
 */
static bool cb_data_is_chgr_on(const struct max77779_foreach_cb_data *cb_data)
{
	return cb_data->stby_on ? 0 : (cb_data->chgr_on >= 2);
}

/*
 * Case	USB_chg USB_otg	WLC_chg	WLC_TX	PMIC_Charger	Ext_B	Name
 * -------------------------------------------------------------------------------------
 * 7	0	1	1	0	IF-PMIC-WCIN	1	USB_OTG_WLC_RX
 * 9	0	1	0	0	0		1	USB_OTG
 * 10   0       1       0       0       OTG_5V          0	USB_OTG_FRS
 * -------------------------------------------------------------------------------------
 * Ext_Boost = 0 off, 1 = OTG 5V
 * WLC_chg = 0 off, 1 = on, 2 = PPS
 *
 * NOTE: do not call with (cb_data->wlc_rx && cb_data->wlc_tx)
 */
static int max77779_get_otg_usecase(struct max77779_foreach_cb_data *cb_data,
				struct max77779_usecase_data *uc_data)
{
	const int chgr_on = cb_data_is_chgr_on(cb_data);
	bool dc_on = cb_data->dc_on; /* && !cb_data->charge_done */
	int usecase;
	u8 mode;

	/* invalid, cannot do OTG stuff with USB power */
	if (cb_data->buck_on) {
		pr_err("%s: buck_on with OTG\n", __func__);
		return -EINVAL;
	}

	if (cb_data->pogo_vout) {
		usecase = GSU_MODE_USB_OTG_POGO_VOUT;
		mode = MAX77779_CHGR_MODE_BOOST_UNO_ON;
	} else if (!cb_data->wlc_rx && !cb_data->wlc_tx) {
		/* 9: USB_OTG or  10: USB_OTG_FRS */
		if (cb_data->frs_on) {
			usecase = GSU_MODE_USB_OTG_FRS;
			mode = MAX77779_CHGR_MODE_OTG_BOOST_ON;
		} else {
			usecase = GSU_MODE_USB_OTG;
			if (!IS_ERR_OR_NULL(uc_data->ext_bst_ctl))
				mode = MAX77779_CHGR_MODE_ALL_OFF;
			else
				mode = MAX77779_CHGR_MODE_OTG_BOOST_ON;
		}

		dc_on = false;
	} else if (cb_data->wlc_tx) {
		/* GSU_MODE_USB_OTG_WLC_TX not supported */
		return -EINVAL;
	} else if (cb_data->wlc_rx) {
		if (dc_on) {
			usecase = GSU_MODE_USB_OTG_WLC_DC;
			mode = MAX77779_CHGR_MODE_ALL_OFF;
		} else if (chgr_on) {
			usecase = GSU_MODE_USB_OTG_WLC_RX_CHARGE_ENABLED;
			if (!IS_ERR_OR_NULL(uc_data->ext_bst_ctl))
				mode = MAX77779_CHGR_MODE_CHGR_BUCK_ON;
			else
				mode = MAX77779_CHGR_MODE_CHGR_OTG_BUCK_BOOST_ON;
		} else {
			usecase = GSU_MODE_USB_OTG_WLC_RX;
			if (!IS_ERR_OR_NULL(uc_data->ext_bst_ctl))
				mode = MAX77779_CHGR_MODE_BUCK_ON;
			else
				mode = MAX77779_CHGR_MODE_CHGR_OTG_BUCK_BOOST_ON;
		}
	} else {
		return -EINVAL;
	}

	cb_data->reg = _max77779_chg_cnfg_00_cp_en_set(cb_data->reg, dc_on);
	cb_data->reg = _max77779_chg_cnfg_00_mode_set(cb_data->reg, mode);
	return usecase;
}

/*
 * Determines the use case to switch to. This is device/system dependent and
 * will likely be factored to a separate file (compile module).
 */
int max77779_get_usecase(struct max77779_foreach_cb_data *cb_data,
			 struct max77779_usecase_data *uc_data)
{
	struct max77779_chgr_data *data = dev_get_drvdata(uc_data->dev);
	const int buck_on = cb_data->chgin_off ? 0 : cb_data->buck_on;
	const int chgr_on = cb_data_is_chgr_on(cb_data);
	bool wlc_tx = cb_data->wlc_tx != 0;
	bool wlc_rx = cb_data->wlc_rx != 0;
	int dc_on = cb_data->dc_on; /* && !cb_data->charge_done */
	int usecase;
	u8 mode;

	/* consistency check, TOD: add more */
	if (wlc_tx) {
		if (wlc_rx) {
			pr_err("%s: wlc_tx and wlc_rx\n", __func__);
			return -EINVAL;
		}

		if (cb_data->otg_on) {
			pr_warn("%s: no wlc_tx with otg_on for now\n", __func__);
			wlc_tx = 0;
			cb_data->wlc_tx = 0;
		}
	}

	/* GSU_MODE_USB_OTG_WLC_DC not supported*/
	if (dc_on && !uc_data->otg_wlc_dc_en && cb_data->wlc_rx)
		cb_data->otg_on = 0;

	/* OTG modes override the others, might need to move under usb_wlc */
	if (cb_data->otg_on || cb_data->frs_on)
		return max77779_get_otg_usecase(cb_data, uc_data);

	/* USB will disable wlc_rx, tx */
	if (cb_data->buck_on && !uc_data->dcin_is_dock) {
		wlc_rx = false;
		wlc_tx = false;
		cb_data->wlc_tx = 0;
	}

	/* buck_on is wired, wlc_rx is wireless, might still need rTX */
	if (cb_data->usb_wlc) {
		/* USB+WLC for factory and testing */
		usecase = GSU_MODE_USB_WLC_RX;
		mode = MAX77779_CHGR_MODE_CHGR_BUCK_ON;
	} else if (cb_data->pogo_vout) {
		if (!buck_on) {
			mode = MAX77779_CHGR_MODE_ALL_OFF;
			usecase = GSU_MODE_POGO_VOUT;
		} else if (chgr_on) {
			mode = MAX77779_CHGR_MODE_CHGR_BUCK_ON;
			usecase = GSU_MODE_USB_CHG_POGO_VOUT;
		} else {
			mode = MAX77779_CHGR_MODE_BUCK_ON;
			usecase = GSU_MODE_USB_CHG_POGO_VOUT;
		}
	} else if (!buck_on && !wlc_rx) {
		mode = MAX77779_CHGR_MODE_ALL_OFF;

		if (cb_data->buck_on) {
			usecase = GSU_MODE_STANDBY_BUCK_ON;
		} else if (wlc_tx) { /* Rtx using the internal battery */
			usecase = GSU_MODE_WLC_TX;
			mode = MAX77779_CHGR_MODE_BOOST_UNO_ON;
		} else {
			usecase = GSU_MODE_STANDBY;
		}
		dc_on = false;
	} else if (wlc_tx) {
		/* above checks that buck_on is false */
		usecase = GSU_MODE_WLC_TX;
		mode = MAX77779_CHGR_MODE_BOOST_UNO_ON;
	} else if (wlc_rx) {
		dc_on = (dc_on == GBMS_CHGR_SEL_WIRELESS);
		/* will be in mode 4 if in stby unless dc is enabled */
		if (chgr_on) {
			mode = MAX77779_CHGR_MODE_CHGR_BUCK_ON;
			usecase = GSU_MODE_WLC_RX_CHARGE_ENABLED;
		} else {
			mode = MAX77779_CHGR_MODE_BUCK_ON;
			usecase = GSU_MODE_WLC_RX;
		}

		/* wired input should be disabled here */
		if (dc_on) {
			mode = MAX77779_CHGR_MODE_ALL_OFF;
			usecase = GSU_MODE_WLC_DC;
		}

		if (uc_data->dcin_is_dock)
			usecase = GSU_MODE_DOCK;

		if (data->wlc_spoof && uc_data->wlc_spoof_vbyp) {
			mode = MAX77779_CHGR_MODE_BOOST_ON;
			usecase = GSU_MODE_WLC_RX_SPOOFED;
		}
	} else {
		dc_on = (dc_on == GBMS_CHGR_SEL_USB);
		/* MODE_BUCK_ON is inflow */
		if (chgr_on) {
			mode = MAX77779_CHGR_MODE_CHGR_BUCK_ON;
			usecase = GSU_MODE_USB_CHG_CHARGE_ENABLED;
		} else {
			mode = MAX77779_CHGR_MODE_BUCK_ON;
			usecase = GSU_MODE_USB_CHG;
		}

		/*
		 * NOTE: OTG cases handled in max77779_get_otg_usecase()
		 * NOTE: usecases with !(buck|wlc)_on same as.
		 * NOTE: mode=0 if standby, mode=5 if charging, mode=0xa on otg
		 * TODO: handle rTx + DC and some more.
		 */
		if (dc_on && wlc_rx) {
			/* WLC_DC->WLC_DC+USB -> ignore dc_on */
		} else if (dc_on) {
			if (uc_data->chrg_byp_en)
				mode = MAX77779_CHGR_MODE_ALLOW_BYP;
			else
				mode = MAX77779_CHGR_MODE_ALL_OFF;
			usecase = GSU_MODE_USB_DC;
		} else if (cb_data->stby_on && !chgr_on) {
			mode = MAX77779_CHGR_MODE_ALL_OFF;
			usecase = cb_data->buck_on ? GSU_MODE_STANDBY_BUCK_ON : GSU_MODE_STANDBY;
		}

	}

	if (wlc_tx)
		dc_on = false;

	/* reg might be ignored later */
	cb_data->reg = _max77779_chg_cnfg_00_cp_en_set(cb_data->reg, !!dc_on);
	cb_data->reg = _max77779_chg_cnfg_00_mode_set(cb_data->reg, mode);

	return usecase;
}

static int max77779_wcin_is_valid(struct max77779_chgr_data *data);
static int max77779_wcin_is_online(struct max77779_chgr_data *data);

static bool max77779_usecase_is_debounce(struct max77779_chgr_data *data,
					 struct max77779_foreach_cb_data *cb_data)
{
	bool debounce;

	debounce = data->uc_data.mode_cb_debounce && !(cb_data->chgr_on == 2) &&
		!cb_data->stby_on && !cb_data->use_raw && !cb_data->fwupdate_on &&
		(!cb_data->chgin_off || (cb_data->chgin_off && cb_data->wlc_rx)) &&
		(!cb_data->wlcin_off || (cb_data->wlcin_off && cb_data->buck_on)) &&
		(!cb_data->otg_on || (cb_data->otg_on && cb_data->wlc_rx)) &&
		!cb_data->usb_wlc && !cb_data->frs_on && !cb_data->wlc_tx && !cb_data->pogo_vin &&
		!cb_data->pogo_vout;

	if (!debounce)
		data->uc_data.mode_cb_debounce = false;

	return debounce;
}

/*
 * I am using a the comparator_none, need scan all the votes to determine
 * the actual.
 *
 * This function should only return errors before bms_usecase_get_new_node
 * otherwise, set entry->ret to the error and handle in
 * max77779_charger_post_election_work
 */
static int max77779_mode_callback(struct gvotable_election *el,
				  const char *trigger, void *value)
{
	struct max77779_chgr_data *data = gvotable_get_data(el);
	struct bms_usecase_data *bms_uc_data = &data->uc_data.usecase_data;
	const int from_use_case = bms_usecase_get_usecase(bms_uc_data);
	struct max77779_foreach_cb_data *cb_data;
	int use_case, ret = 0;
	bool nope = false, rerun = false;
	u8 reg = 0;
	struct bms_usecase_entry *entry;
	struct max77779_usecase_data *uc_data = &data->uc_data;
	bool wlc_in_standby;

	__pm_stay_awake(data->usecase_wake_lock);
	mutex_lock(&data->mode_lock);

	use_case = from_use_case;

	if (max77779_init_check(data)) {
		schedule_delayed_work(&data->mode_rerun_work, msecs_to_jiffies(50));
		rerun = true;
		ret = -EAGAIN;
		goto unlock_done;
	}

	entry = bms_usecase_get_new_node(bms_uc_data,
			gvotable_get_most_recent_reason(el),
			GVOTABLE_PTR_TO_INT(gvotable_get_most_recent_vote(el)));
	if (!entry) {
		dev_err(data->dev, "%s couldn't allocate entry\n", __func__);
		ret = -ENOMEM;
		goto unlock_done;
	}

	cb_data = (struct max77779_foreach_cb_data *)entry->cb_data;
	cb_data->reason = trigger;

	/* no caching */
	ret = max77779_reg_read(data, MAX77779_CHG_CNFG_00, &reg);
	if (ret < 0) {
		entry->mode_cb_ret = ret;
		dev_err(data->dev, "cannot read CNFG_00 (%d)\n", ret);
		ret = 0;
		goto unlock_done;
	}

	/* Need to switch to MW (turn off dc_on) and enforce no charging  */
	cb_data->charge_done = data->charge_done;

	/* this is the last vote of the election */
	cb_data->reg = reg;	/* current */
	cb_data->el = el;	/* election */

	cb_data->wlcin_off = !!data->wcin_input_suspend;

	pr_debug("%s: wcin_is_online=%d data->wcin_input_suspend=%d data->wlc_spoof=%d\n", __func__,
		  max77779_wcin_is_online(data), data->wcin_input_suspend, data->wlc_spoof);

	/* now scan all the reasons, accumulate in cb_data */
	gvotable_election_for_each(el, max77779_foreach_callback, cb_data);

	wlc_in_standby = cb_data->wlc_rx && data->wcin_input_suspend;
	uc_data->wlc_standby_changed = wlc_in_standby != uc_data->wlc_in_standby;
	if (uc_data->wlc_standby_changed)
		uc_data->wlc_in_standby = cb_data->wlc_rx && data->wcin_input_suspend;
	cb_data->wlc_rx = (cb_data->wlc_rx && !data->wcin_input_suspend) ||
			  (data->wlc_spoof && uc_data->wlc_spoof_vbyp);

	/* Block wlc_rx for POGO_VIN if it is from POGO_VOUT */
	cb_data->wlc_rx = cb_data->wlc_rx &&
			 from_use_case != GSU_MODE_POGO_VOUT &&
			 from_use_case != GSU_MODE_USB_CHG_POGO_VOUT &&
			 from_use_case != GSU_MODE_USB_OTG_POGO_VOUT;

	nope = !cb_data->use_raw && !cb_data->stby_on && !cb_data->dc_on &&
	       !cb_data->chgr_on && !cb_data->buck_on &&
	       !cb_data->otg_on && !cb_data->frs_on && !cb_data->wlc_tx &&
	       !cb_data->wlc_rx && !cb_data->wlcin_off && !cb_data->chgin_off &&
	       !cb_data->usb_wlc && !cb_data->fwupdate_on &&
	       !cb_data->pogo_vout && !cb_data->pogo_vin;
	if (nope) {
		entry->mode_cb_ret = BMS_USECASE_NO_HOPS;
		dev_dbg(data->dev, "%s: nope callback\n", __func__);
		goto unlock_done;
	}

	dev_info(data->dev, "%s:%s full=%d raw=%d stby_on=%d, dc_on=%d, chgr_on=%d, buck_on=%d,"
		" otg_on=%d, wlc_tx=%d wlc_rx=%d usb_wlc=%d"
		" chgin_off=%d wlcin_off=%d frs_on=%d fwupdate=%d"
		" pogo_vout=%d, pogo_vin=%d\n",
		__func__, trigger ? trigger : "<>",
		data->charge_done, cb_data->use_raw, cb_data->stby_on, cb_data->dc_on,
		cb_data->chgr_on, cb_data->buck_on, cb_data->otg_on,
		cb_data->wlc_tx, cb_data->wlc_rx, cb_data->usb_wlc,
		cb_data->chgin_off, cb_data->wlcin_off, cb_data->frs_on, cb_data->fwupdate_on,
		cb_data->pogo_vout, cb_data->pogo_vin);

	if (max77779_usecase_is_debounce(data, cb_data)) {
		entry->mode_cb_ret = BMS_USECASE_NO_HOPS;
		dev_dbg(data->dev, "%s: DEBOUNCE callback\n", __func__);
		goto unlock_done;
	}

	/* just use raw "as is", no changes to switches etc */
	if (unlikely(cb_data->fwupdate_on)) {
		cb_data->reason = MAX77779_REASON_FIRMWARE;

		if (GSU_MODE_FWUPDATE_MASK & cb_data->fwupdate_on) {
			cb_data->reg = MAX77779_CHGR_MODE_BOOST_ON;
			use_case = GSU_MODE_FWUPDATE;
		} else {
			cb_data->reg = MAX77779_CHGR_MODE_BOOST_UNO_ON;
			use_case = GSU_MODE_WLC_FWUPDATE;
		}
	} else if (cb_data->use_raw) {
		cb_data->reg = cb_data->raw_value;
		use_case = GSU_RAW_MODE;
	} else {
		bool use_internal_bst;

		/* insel needs it, otg usecases needs it */
		if (uc_data->init_done <= 0) {
			uc_data->init_done = gs201_setup_usecases(uc_data,
						data->dev->of_node);
			gs201_dump_usecasase_config(uc_data);
		}

		/*
		 * force FRS if ext boost or NBC is not enabled
		 * TODO: move to setup_usecase
		 */
		use_internal_bst = uc_data->vin_is_valid < 0 &&
				   IS_ERR_OR_NULL(uc_data->ext_bst_ctl);
		if (cb_data->otg_on && use_internal_bst)
			cb_data->frs_on = cb_data->otg_on;

		/* figure out next use case if not in raw mode */
		use_case = max77779_get_usecase(cb_data, uc_data);
		if (use_case < 0) {
			entry->mode_cb_ret = use_case;
			dev_err(data->dev, "no valid use case %d\n", use_case);
			goto unlock_done;
		}
	}

	bms_usecase_entry_set_usecase(entry, use_case);

unlock_done:
	if (use_case >= 0 && rerun)
		dev_info(data->dev, "%s:%s vote before resume complete\n",
			 __func__, trigger ? trigger : "<>");

	return ret ? ret : entry->id;
}

static void max77779_mode_rerun_work(struct work_struct *work)
{
	struct max77779_chgr_data *data = container_of(work, struct max77779_chgr_data,
						       mode_rerun_work.work);

	gvotable_run_election(data->mode_votable, true);

	return;
}

static int max77779_get_charge_enabled(struct max77779_chgr_data *data,
				       int *enabled)
{
	int ret;
	const void *vote = (const void *)0;

	ret = gvotable_get_current_vote(data->mode_votable, &vote);
	if (ret < 0)
		return ret;

	switch ((uintptr_t)vote) {
	case MAX77779_CHGR_MODE_CHGR_BUCK_ON:
	case MAX77779_CHGR_MODE_CHGR_BUCK_BOOST_UNO_ON:
	case MAX77779_CHGR_MODE_CHGR_OTG_BUCK_BOOST_ON:
		*enabled = 1;
		break;
	default:
		*enabled = 0;
		break;
	}

	return ret;
}

/* reset charge_done if needed on cc_max!=0 and on charge_disable(false) */
static int max77779_enable_sw_recharge(struct max77779_chgr_data *data,
				       bool force)
{
	const bool charge_done = data->charge_done;
	bool needs_restart = force || data->charge_done;
	uint8_t reg;
	int ret;

	if (max77779_init_check(data))
		return -EAGAIN;

	if (!needs_restart) {
		ret = max77779_reg_read(data, MAX77779_CHG_DETAILS_01, &reg);
		needs_restart = (ret < 0) ||
				_max77779_chg_details_01_chg_dtls_get(reg) == CHGR_DTLS_DONE_MODE;
		if (!needs_restart)
			return 0;
	}

	/* This: will not trigger the usecase state machine */
	mutex_lock(&data->io_lock);
	ret = max77779_reg_read(data, MAX77779_CHG_CNFG_00, &reg);
	if (ret == 0)
		ret = max77779_chg_mode_write_locked(data, MAX77779_CHGR_MODE_ALL_OFF);
	if (ret == 0)
		ret = max77779_chg_mode_write_locked(data, reg);
	mutex_unlock(&data->io_lock);

	data->charge_done = false;

	dev_dbg(data->dev, "%s charge_done=%d->0, reg=%hhx (%d)\n", __func__,
		charge_done, reg, ret);

	return ret;
}

static int max77779_higher_headroom_enable(struct max77779_chgr_data *data, bool flag)
{
	int ret = 0;
	u8 reg, reg_rd;
	const u8 val = flag ? CHGR_CHG_CNFG_12_VREG_4P7V : CHGR_CHG_CNFG_12_VREG_4P6V;

	mutex_lock(&data->io_lock);
	ret = max77779_reg_read(data, MAX77779_CHG_CNFG_12, &reg);
	if (ret < 0)
		goto done;

	reg_rd = reg;

	reg = _max77779_chg_cnfg_12_vchgin_reg_set(reg, val);
	ret = max77779_reg_write(data, MAX77779_CHG_CNFG_12, reg);

done:
	mutex_unlock(&data->io_lock);

	dev_dbg(data->dev, "%s: val: %#02x, reg: %#02x -> %#02x (%d)\n", __func__,
		val, reg_rd, reg, ret);

	return ret;
}

/* called from gcpm and for CC_MAX == 0 */
static int max77779_set_charge_enabled(struct max77779_chgr_data *data,
				       int enabled, const char *reason)
{
	/* ->charge_done is reset in max77779_enable_sw_recharge() */
	pr_debug("%s %s enabled=%d\n", __func__, reason, enabled);

	return gvotable_cast_long_vote(data->mode_votable, reason,
				       GBMS_CHGR_MODE_CHGR_BUCK_ON, enabled);
}

/* google_charger on disconnect */
static int max77779_set_charge_disable(struct max77779_chgr_data *data,
				       int enabled, const char *reason)
{
	/* make sure charging is restarted on enable */
	if (enabled) {
		int ret;

		ret = max77779_enable_sw_recharge(data, false);
		if (ret < 0)
			dev_err(data->dev, "%s cannot re-enable charging (%d)\n",
				__func__, ret);

		ret = max77779_higher_headroom_enable(data, false); /* reset on plug/unplug */
		if (ret)
			dev_err_ratelimited(data->dev, "%s error disabling higher headroom,"
					    "ret:%d\n", __func__, ret);
	}

	return gvotable_cast_long_vote(data->mode_votable, reason,
				       GBMS_CHGR_MODE_STBY_ON, enabled);
}

static int max77779_chgin_input_suspend(struct max77779_chgr_data *data,
					bool enabled, const char *reason)
{
	const int old_value = data->chgin_input_suspend;
	int ret;

	dev_dbg(data->dev, "%s enabled=%d->%d reason=%s\n", __func__,
		 data->chgin_input_suspend, enabled, reason);

	data->chgin_input_suspend = enabled; /* the callback might use this */
	ret = gvotable_cast_long_vote(data->mode_votable, "CHGIN_SUSP",
				      GBMS_CHGR_MODE_CHGIN_OFF, enabled);
	if (ret < 0)
		data->chgin_input_suspend = old_value; /* restored */

	return ret;
}

static int max77779_wcin_input_suspend(struct max77779_chgr_data *data,
				       bool enabled, const char *reason)
{
	const int old_value = data->wcin_input_suspend;
	const uint32_t vote = _bms_usecase_meta_async_set(GBMS_CHGR_MODE_WLCIN_OFF, true);
	const void *val = (const void *)0;
	int ret = 0, icl;

	dev_dbg(data->dev, "%s %s vote=%d", __func__, reason, enabled);

	ret = gvotable_get_current_vote(data->dc_suspend_votable, &val);
	icl = gvotable_get_current_int_vote(data->dc_icl_votable);
	if (ret == 0) {
		data->wcin_input_suspend = (uintptr_t)val > 0 || icl == 0;
		dev_dbg(data->dev, "%s wcin_input_suspend=%d(dc_suspend=%lu,icl_suspend=%d)",
			 __func__, data->wcin_input_suspend, (uintptr_t)val, icl == 0);
	}

	dev_dbg(data->dev, "%s enabled=%d->%d reason=%s vote:%d\n", __func__,
		 old_value, data->wcin_input_suspend, reason, vote);

	ret = gvotable_cast_long_vote(data->mode_votable, reason, vote, enabled);
	if (ret < 0)
		data->wcin_input_suspend = old_value; /* restore */

	return ret;
}

static int max77779_set_regulation_voltage(struct max77779_chgr_data *data,
					   int voltage_uv)
{
	u8 value;

	if (voltage_uv >= 4550000)
		value = 0x37;
	else if (voltage_uv < 4000000)
		value = 0x38 + (voltage_uv - 3800000) / 100000;
	else
		value = (voltage_uv - 4000000) / 10000;

	value = VALUE2FIELD(MAX77779_CHG_CNFG_04_CHG_CV_PRM, value);
	return max77779_reg_update(data, MAX77779_CHG_CNFG_04,
				   MAX77779_CHG_CNFG_04_CHG_CV_PRM_MASK,
				   value);
}

static int max77779_get_regulation_voltage_uv(struct max77779_chgr_data *data,
					      int *voltage_uv)
{
	u8 value;
	int ret;

	ret = max77779_reg_read(data, MAX77779_CHG_CNFG_04, &value);
	if (ret < 0)
		return ret;

	if  (value < 0x38)
		*voltage_uv = (4000 + value * 10) * 1000;
	else if (value == 0x38)
		*voltage_uv = 3800 * 1000;
	else if (value == 0x39)
		*voltage_uv = 3900 * 1000;
	else
		return -EINVAL;

	return 0;
}

static int max77779_enable_cop(struct max77779_chgr_data *data, bool enable)
{

	return max77779_reg_update(data, MAX77779_CHG_COP_CTRL,
				   MAX77779_CHG_COP_CTRL_COP_EN_MASK,
				   _max77779_chg_cop_ctrl_cop_en_set(0, enable));
}

static bool max77779_is_cop_enabled(struct max77779_chgr_data *data)
{
	u8 value;
	int ret;

	ret = max77779_reg_read(data, MAX77779_CHG_COP_CTRL, &value);
	return (ret == 0) && _max77779_chg_cop_ctrl_cop_en_get(value);
}

/* Accepts current in uA */
static int max77779_set_cop_warn(struct max77779_chgr_data *data, uint32_t max_value)
{
	int ret;
	const uint32_t cc_max = max_value;

	max_value *= MAX77779_COP_SENSE_RESISTOR_VAL;
	max_value /= 1000; /* Convert to uV */

	if (max_value > 0xFFFF) {
		dev_err(data->dev, "Setting COP warn value too large val:%u\n", max_value);
		return -EINVAL;
	}

	ret = max77779_writen(data, MAX77779_CHG_COP_WARN_L, /* NOTYPO */
			     (uint8_t*)&max_value, 2);
	if (ret) {
		dev_err(data->dev, "Error writing MAX77779_CHG_COP_WARN_L ret:%d", ret);
		return ret;
	}

	data->cop_warn = cc_max;

	return ret;
}

static int max77779_get_cop_warn(struct max77779_chgr_data *data, uint32_t *max_value)
{
	int ret;
	u16 temp;

	ret = max77779_readn(data, MAX77779_CHG_COP_WARN_L, (uint8_t*)&temp, 2);
	if (ret) {
		dev_err(data->dev, "Error reading MAX77779_CHG_COP_WARN_L ret:%d", ret);
		return ret;
	}

	*max_value = temp * 1000 / MAX77779_COP_SENSE_RESISTOR_VAL;

	return ret;
}

/* Accepts current in uA */
static int max77779_set_cop_limit(struct max77779_chgr_data *data, uint32_t max_value)
{
	int ret;

	max_value *= MAX77779_COP_SENSE_RESISTOR_VAL;
	max_value /= 1000; /* Convert to uV */

	if (max_value > 0xFFFF) {
		dev_err(data->dev, "Setting COP limit value too large val:%u\n", max_value);
		return -EINVAL;
	}

	ret = max77779_writen(data, MAX77779_CHG_COP_LIMIT_L, /* NOTYPO */
			     (uint8_t*)&max_value, 2);
	if (ret) {
		dev_err(data->dev, "Error writing MAX77779_CHG_COP_LIMIT_L ret:%d", ret);
		return ret;
	}

	return ret;
}

static int max77779_get_cop_limit(struct max77779_chgr_data *data, uint32_t *max_value)
{
	int ret;
	u16 temp;

	ret = max77779_readn(data, MAX77779_CHG_COP_LIMIT_L, (uint8_t*)&temp, 2);
	if (ret) {
		dev_err(data->dev, "Error reading MAX77779_CHG_COP_LIMIT_L ret:%d", ret);
		return ret;
	}

	*max_value = temp * 1000 / MAX77779_COP_SENSE_RESISTOR_VAL;

	return ret;
}

static void max77779_cop_enable_work(struct work_struct *work)
{
	struct max77779_chgr_data *data = container_of(work, struct max77779_chgr_data,
						       cop_enable_work.work);

	max77779_enable_cop(data, true);
}

static int max77779_cop_config(struct max77779_chgr_data * data)
{
	int ret;

	max77779_set_cop_warn(data, MAX77779_COP_MAX_VALUE);

	/* TODO: b/293487608 Support COP limit */
	/* Setting limit to MAX to not trip */
	ret = max77779_set_cop_limit(data, MAX77779_COP_MAX_VALUE);
	if (ret < 0)
		dev_err(data->dev, "Error setting COP limit to max\n");

	return ret;
}

/* set charging current to 0 to disable charging (MODE=0) */
static int max77779_set_charger_current_max_ua(struct max77779_chgr_data *data,
					       int current_ua)
{
	const int disabled = current_ua == 0;
	u8 value, reg;
	int ret;
	bool cp_enabled;
	uint32_t new_cop_warn;

	if (current_ua < 0)
		return 0;

	/* ilim=0 -> switch to mode 0 and suspend charging */
	if  (current_ua == 0)
		value = 0x0;
	else if (current_ua <= 200000)
		value = 0x03;
	else if (current_ua >= 4000000)
		value = 0x3c;
	else
		value = 0x3 + (current_ua - 200000) / 66670;

	ret = max77779_reg_read(data, MAX77779_CHG_CNFG_00, &reg);
	if (ret < 0) {
		dev_err(data->dev, "cannot read CHG_CNFG_00 (%d)\n", ret);
		return ret;
	}

	new_cop_warn = current_ua * MAX77779_COP_WARN_THRESHOLD / 100;

	/* Don't trigger COP in discharge */
	if (new_cop_warn == 0)
		new_cop_warn = MAX77779_COP_MAX_VALUE;

	if (data->cop_warn <= new_cop_warn) {
		ret = max77779_set_cop_warn(data, new_cop_warn);
		if (ret < 0)
			dev_err(data->dev, "cannot set cop warn (%d)\n", ret);

		msleep(MAX77779_COP_MIN_DEBOUNCE_TIME_MS);
	}

	cp_enabled = _max77779_chg_cnfg_00_cp_en_get(reg);
	if (cp_enabled)
		goto update_reg;

	/*
	 * cc_max > 0 might need to restart charging: the usecase state machine
	 * will be triggered in max77779_set_charge_enabled()
	 */
	if (current_ua) {
		ret = max77779_enable_sw_recharge(data, false);
		if (ret < 0)
			dev_err(data->dev, "cannot re-enable charging (%d)\n", ret);
	}
update_reg:
	value = VALUE2FIELD(MAX77779_CHG_CNFG_02_CHGCC, value);
	ret = max77779_reg_update(data, MAX77779_CHG_CNFG_02,
				   MAX77779_CHG_CNFG_02_CHGCC_MASK,
				   value);
	if (ret == 0)
		ret = max77779_set_charge_enabled(data, !disabled, "CC_MAX");

	if (data->cop_warn > new_cop_warn) {
		msleep(MAX77779_COP_MIN_DEBOUNCE_TIME_MS);

		ret = max77779_set_cop_warn(data, new_cop_warn);
		if (ret < 0)
			dev_err(data->dev, "cannot set cop warn (%d)\n", ret);
	}

	return ret;
}

static int max77779_get_charger_current_max_ua(struct max77779_chgr_data *data,
					       int *current_ua)
{
	u8 value;
	int ret;

	ret = max77779_reg_read(data, MAX77779_CHG_CNFG_02,
				&value);
	if (ret < 0)
		return ret;

	/* TODO: fix the rounding */
	value = VALUE2FIELD(MAX77779_CHG_CNFG_02_CHGCC, value);

	/* ilim=0 -> mode 0 with charging suspended */
	if (value == 0)
		*current_ua = 0;
	else if (value < 3)
		*current_ua = 133 * 1000;
	else if (value >= 0x3C)
		*current_ua = 4000 * 1000;
	else
		*current_ua = 133000 + (value - 2) * 66670;

	return 0;
}

/* enable autoibus and charger mode */
static int max77779_chgin_set_ilim_max_ua(struct max77779_chgr_data *data,
					  int ilim_ua)
{
	const bool suspend = ilim_ua == 0;
	u8 value;
	int ret;

	/* TODO: disable charging */
	if (ilim_ua < 0)
		return 0;

	if (ilim_ua == 0)
		value = 0x00;
	else if (ilim_ua > 3200000)
		value = 0x7f;
	else
		value = 0x04 + (ilim_ua - 125000) / 25000;

	value = VALUE2FIELD(MAX77779_CHG_CNFG_09_NO_AUTOIBUS, 1) |
		VALUE2FIELD(MAX77779_CHG_CNFG_09_CHGIN_ILIM, value);
	ret = max77779_reg_update(data, MAX77779_CHG_CNFG_09,
					MAX77779_CHG_CNFG_09_NO_AUTOIBUS_MASK |
					MAX77779_CHG_CNFG_09_CHGIN_ILIM_MASK,
					value);
	if (ret == 0)
		ret = max77779_chgin_input_suspend(data, suspend, "ILIM");

	return ret;
}

static int max77779_chgin_get_ilim_max_ua(struct max77779_chgr_data *data,
					  int *ilim_ua)
{
	int icl, ret;
	u8 value;

	ret = max77779_reg_read(data, MAX77779_CHG_CNFG_09, &value);
	if (ret < 0)
		return ret;

	value = FIELD2VALUE(MAX77779_CHG_CNFG_09_CHGIN_ILIM, value);
	if (value == 0)
		icl = 0;
	else if (value > 3)
		icl = 100 + (value - 3) * 25;
	else
		icl = 100;

	*ilim_ua = icl * 1000;

	if (data->chgin_input_suspend)
		*ilim_ua = 0;

	return 0;
}

static int max77779_set_topoff_current_max_ma(struct max77779_chgr_data *data,
					       int current_ma)
{
	u8 value;
	int ret;

	if (current_ma < 0)
		return 0;

	if (current_ma <= 150)
		value = 0x0;
	else if (current_ma >= 500)
		value = 0x7;
	else
		value = (current_ma - 150) / 50;

	value = VALUE2FIELD(MAX77779_CHG_CNFG_03_TO_ITH, value);
	ret = max77779_reg_update(data, MAX77779_CHG_CNFG_03,
				   MAX77779_CHG_CNFG_03_TO_ITH_MASK,
				   value);

	return ret;
}

static int max77779_set_topoff_timer(struct max77779_chgr_data *data,
				     int timer)
{
	u8 value;
	int ret;

	if (timer < 0)
		return 0;

	if (timer < 600)
		value = 0x0;
	else if (timer >= 4200)
		value = 0x7;
	else
		value = ((timer - 600) / 600) + 1;

	value = VALUE2FIELD(MAX77779_CHG_CNFG_03_TO_TIME, value);
	ret = max77779_reg_update(data, MAX77779_CHG_CNFG_03,
				   MAX77779_CHG_CNFG_03_TO_TIME_MASK,
				   value);

	return ret;
}

static int max77779_wcin_set_ilim_max_ua(struct max77779_chgr_data *data,
					 int ilim_ua)
{
	u8 value;
	int ret;

	if (ilim_ua < 0)
		return -EINVAL;

	if (ilim_ua == 0)
		value = 0x00;
	else if (ilim_ua <= 100000)
		value = 0x03;
	else
		value = 0x4 + (ilim_ua - 125000) / 25000;

	value = VALUE2FIELD(MAX77779_CHG_CNFG_10_WCIN_ILIM, value);
	ret = max77779_reg_update(data, MAX77779_CHG_CNFG_10,
					MAX77779_CHG_CNFG_10_WCIN_ILIM_MASK,
					value);

	/* Legacy: DC_ICL doesn't suspend on ilim_ua == 0 (it should) */

	return ret;
}

static int max77779_wcin_get_ilim_max_ua(struct max77779_chgr_data *data,
					 int *ilim_ua)
{
	int ret;
	u8 value;

	ret = max77779_reg_read(data, MAX77779_CHG_CNFG_10, &value);
	if (ret < 0)
		return ret;

	value = FIELD2VALUE(MAX77779_CHG_CNFG_10_WCIN_ILIM, value);
	if (value == 0)
		*ilim_ua = 0;
	else if (value < 4)
		*ilim_ua = 100000;
	else
		*ilim_ua = 125000 + (value - 4) * 25000;

	if (data->wcin_input_suspend)
		*ilim_ua = 0;

	return 0;
}

/* default is no suspend, any valid vote will suspend  */
static int max77779_dc_suspend_vote_callback(struct gvotable_election *el,
					     const char *reason, void *value)
{
	struct max77779_chgr_data *data = gvotable_get_data(el);
	int ret, suspend = (long)value > 0;

	/* will trigger a CHARGER_MODE callback */
	ret = max77779_wcin_input_suspend(data, suspend, "DC_SUSPEND");
	if (ret < 0)
		return 0;

	pr_debug("%s: DC_SUSPEND reason=%s, value=%ld suspend=%d (%d)\n",
		 __func__, reason ? reason : "", (long)value, suspend, ret);

	return 0;
}

static int max77779_real_dc_icl(int dc_icl)
{
	return (dc_icl / MAX77779_DC_ICL_STEP) * MAX77779_DC_ICL_STEP;
}

static int max77779_dcicl_callback(struct gvotable_election *el,
				   const char *reason,
				   void *value)
{
	struct max77779_chgr_data *data = gvotable_get_data(el);
	const bool suspend = (long)value == 0;
	int ret;
	struct max77779_usecase_data *uc_data = &data->uc_data;

	pr_debug("%s: DC_ICL reason=%s, value=%ld suspend=%d\n",
		 __func__, reason ? reason : "", (long)value, suspend);

	if (max77779_real_dc_icl((long)value) != max77779_real_dc_icl(data->dc_icl))
		data->wcin_inlim_flag = -1;

	data->dc_icl = (long)value;
	/* doesn't trigger a CHARGER_MODE */
	ret = max77779_wcin_set_ilim_max_ua(data, data->dc_icl);
	if (ret < 0)
		dev_err(data->dev, "cannot set dc_icl=%d (%d)\n",
			data->dc_icl, ret);

	/* will trigger a CHARGER_MODE callback */
	if (uc_data->wlc_spoof_vbyp)
		gvotable_cast_bool_vote(data->wlc_spoof_votable, "WLC",
					suspend && (strcmp(reason, REASON_MDIS) == 0));

	ret = max77779_wcin_input_suspend(data, suspend, "DC_ICL");
	if (ret < 0)
		dev_err(data->dev, "cannot set suspend=%d (%d)\n",
			suspend, ret);

	return 0;
}

static int max77779_wlc_spoof_callback(struct gvotable_election *el,
				       const char *reason, void *value)
{
	struct max77779_chgr_data *data = gvotable_get_data(el);
	int spoof = (long)value > 0;
	bool wlc_rx;

	wlc_rx = (max77779_wcin_is_online(data) && !data->wcin_input_suspend);

	data->wlc_spoof = spoof && wlc_rx;

	pr_info("%s:wlc_spoof=%d\n", __func__, data->wlc_spoof);

	return 0;
}

static void max77779_inlim_irq_en(struct max77779_chgr_data *data, bool en)
{
	int ret;
	uint16_t intb_mask;

	mutex_lock(&data->io_lock);

	ret = max77779_readn(data, MAX77779_CHG_INT_MASK, (uint8_t*)&intb_mask, 2);
	if (ret < 0) {
		dev_err(data->dev, "Unable to read interrupt mask (%d)\n", ret);
		goto unlock_out;
	}

	if (en) {
		max77779_int_mask[0] &= ~MAX77779_CHG_INT_INLIM_I_MASK;
		intb_mask &= ~MAX77779_CHG_INT_INLIM_I_MASK;
	} else {
		max77779_int_mask[0] |= MAX77779_CHG_INT_INLIM_I_MASK;
		intb_mask |= MAX77779_CHG_INT_INLIM_I_MASK;
	}
	ret = max77779_writen(data, MAX77779_CHG_INT_MASK, /* NOTYPO */
			      (uint8_t*)&intb_mask, sizeof(intb_mask));
	if (ret < 0)
		dev_err(data->dev, "%s: cannot set irq_mask (%d)\n", __func__, ret);

unlock_out:
	mutex_unlock(&data->io_lock);
}

static void max77779_wcin_inlim_work(struct work_struct *work)
{
	struct max77779_chgr_data *data = container_of(work, struct max77779_chgr_data,
						       wcin_inlim_work.work);
	int iwcin, wcin_soft_icl, dc_icl_prev, inlim, ret;
	union power_supply_propval volt_now;

	mutex_lock(&data->wcin_inlim_lock);

	if (data->wcin_ema < 0 || data->wcin_ema_disable) {
		if (max77779_wcin_current_now(data, &iwcin))
			goto done;
	} else {
		iwcin = data->wcin_ema;
	}
	if (!data->dc_icl_votable) {
		mutex_unlock(&data->wcin_inlim_lock);
		dev_err(data->dev, "Could not get votable: DC_ICL\n");
		return;
	}

	dc_icl_prev = data->dc_icl;

	inlim = max77779_is_limited(data);

	if (data->wcin_soft_icl == 0) {
		/* initial setting */
		data->wcin_inlim_flag = -1;
		wcin_soft_icl = iwcin + data->wcin_inlim_headroom;
		goto vote;
	}
	if (!inlim) {
		data->wcin_inlim_debounce_count = data->wcin_inlim_debounce_thresh;
		data->wcin_inlim_flag = 0;
		if (dc_icl_prev <= iwcin + data->wcin_inlim_headroom)
			wcin_soft_icl = dc_icl_prev;
		else
			wcin_soft_icl = iwcin + data->wcin_inlim_headroom;
		goto vote;
	}
	wcin_soft_icl = dc_icl_prev;
	if (data->wcin_inlim_flag == 0) {
		data->wcin_inlim_debounce_count -= 1;
		if (data->wcin_inlim_debounce_count <= 0) {
			dev_dbg(data->dev, "inlim remove debounce");
			data->wcin_inlim_flag = 1;
			wcin_soft_icl = dc_icl_prev + data->wcin_inlim_step;
		} else {
			dev_dbg(data->dev, "inlim debounce: %d checks left",
				data->wcin_inlim_debounce_count);
		}
	} else {
		data->wcin_inlim_flag = 1;
		wcin_soft_icl = dc_icl_prev + data->wcin_inlim_step;
	}
vote:
	ret = power_supply_get_property(data->wcin_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, &volt_now);
	gvotable_cast_int_vote(data->dc_icl_votable, WCIN_INLIM_VOTER, wcin_soft_icl, true);
	if (data->wcin_soft_icl == 0 ||
	    (max77779_real_dc_icl(wcin_soft_icl) != max77779_real_dc_icl(data->wcin_soft_icl) &&
	     max77779_real_dc_icl(wcin_soft_icl) != max77779_real_dc_icl(dc_icl_prev))) {
		dev_info(data->dev, "%s: iwcin: %d, soft_icl: %d->%d, prev_dc_icl: %d, limited: %d, inlim_flag: %d volt_now: %d (%d)\n",
		__func__, iwcin, data->wcin_soft_icl, wcin_soft_icl, dc_icl_prev,
		inlim, data->wcin_inlim_flag, volt_now.intval, ret);
	} else {
		dev_dbg(data->dev, "iwcin: %d, soft_icl: %d->%d, prev_dc_icl: %d, limited: %d, inlim_flag: %d volt_now: %d (%d)\n",
		iwcin, data->wcin_soft_icl, wcin_soft_icl, dc_icl_prev,
		inlim, data->wcin_inlim_flag, volt_now.intval, ret);
	}
	data->wcin_soft_icl = wcin_soft_icl;

done:
	mutex_unlock(&data->wcin_inlim_lock);
	schedule_delayed_work(&data->wcin_inlim_work, msecs_to_jiffies(data->wcin_inlim_t));
}

static void max77779_wcin_ema_work(struct work_struct *work)
{
	struct max77779_chgr_data *data = container_of(work, struct max77779_chgr_data,
						       wcin_ema_work.work);
	int iwcin_now;

	mutex_lock(&data->wcin_inlim_lock);
	if (data->wcin_ema == -2 || !data->wcin_inlim_en || data->wcin_ema_disable) {
		mutex_unlock(&data->wcin_inlim_lock);
		return;
	}
	if (max77779_wcin_current_now(data, &iwcin_now))
		goto done;

	if (data->wcin_ema == -1) {
		data->wcin_ema = iwcin_now;
		goto done;
	}
	data->wcin_ema = (data->wcin_ema_alpha * iwcin_now / 1000) +
			 (((1000 - data->wcin_ema_alpha) * data->wcin_ema) / 1000);
done:
	mutex_unlock(&data->wcin_inlim_lock);
	if (!data->wcin_ema_disable)
		schedule_delayed_work(&data->wcin_ema_work, msecs_to_jiffies(data->wcin_ema_t));

}

static void max77779_wcin_inlim_work_en(struct max77779_chgr_data *data, bool en)
{
	mutex_lock(&data->wcin_inlim_lock);
	if (en) {
		if (!data->wcin_ema_disable && data->wcin_ema != -2) {
			data->wcin_ema = -1;
			mod_delayed_work(system_wq, &data->wcin_ema_work, 0);
		}
		mod_delayed_work(system_wq, &data->wcin_inlim_work, 0);
	} else {
		max77779_inlim_irq_en(data, false);
		cancel_delayed_work(&data->wcin_inlim_work);
		cancel_delayed_work(&data->wcin_ema_work);
		if (data->wcin_ema > 0)
			data->wcin_ema = -1;
		data->wcin_soft_icl = 0;
		if (data->dc_icl_votable)
			gvotable_cast_int_vote(data->dc_icl_votable, WCIN_INLIM_VOTER,
						data->wcin_soft_icl, false);
	}
	mutex_unlock(&data->wcin_inlim_lock);
}

#if IS_ENABLED(CONFIG_GPIOLIB)
static int max77779_gpio_get_direction(struct gpio_chip *chip, unsigned int offset)
{
	return GPIOF_DIR_OUT;
}

static int max77779_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	return 0;
}

static void max77779_gpio_set(struct gpio_chip *chip, unsigned int offset, int value)
{
	struct max77779_chgr_data *data = gpiochip_get_data(chip);
	int ret = 0;

	switch (offset) {
	case MAX77779_GPIO_WCIN_INLIM_EN:
		data->wcin_inlim_en = !!value;
		max77779_wcin_inlim_work_en(data, data->wcin_inlim_en);
		break;

	default:
		ret = -EINVAL;
		break;
	}

	dev_dbg(data->dev, "%s: GPIO offset=%d value=%d ret:%d\n", __func__, offset, value, ret);

	if (ret < 0)
		dev_warn(data->dev, "GPIO%d: value=%d ret:%d\n", offset, value, ret);
}

static void max77779_gpio_init(struct max77779_chgr_data *data)
{
	data->gpio.owner = THIS_MODULE;
	data->gpio.label = "max77779_gpio";
	data->gpio.get_direction = max77779_gpio_get_direction;
	data->gpio.get = max77779_gpio_get;
	data->gpio.set = max77779_gpio_set;
	data->gpio.base = -1;
	data->gpio.ngpio = MAX77779_NUM_GPIOS;
	data->gpio.can_sleep = true;
}
#endif

/*************************
 * WCIN PSY REGISTRATION   *
 *************************/
static enum power_supply_property max77779_wcin_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
};

static int max77779_wcin_is_valid(struct max77779_chgr_data *data)
{
	uint8_t val = 0;
	uint8_t wcin_dtls;
	int ret = 0;

	if (data->uc_data.pogo_vout_en >= 0) {
		val = max77779_current_check_mode(data);
		ret = gpiod_get_value_cansleep(data->uc_data.pogo_vout_en);
	}

	if (val == MAX77779_CHGR_MODE_BOOST_UNO_ON || ret > 0)
		return 0;

	ret = max77779_reg_read(data, MAX77779_CHG_DETAILS_00, &val);
	if (ret < 0)
		return ret;
	wcin_dtls = _max77779_chg_details_00_wcin_dtls_get(val);
	return wcin_dtls == 0x2 || wcin_dtls == 0x3;
}

static inline int max77779_wcin_is_online(struct max77779_chgr_data *data)
{
	return max77779_wcin_is_valid(data);
}

/* TODO: make this configurable */
static struct power_supply* max77779_get_wlc_psy(struct max77779_chgr_data *chg)
{
	if (!chg->wlc_psy)
		chg->wlc_psy = power_supply_get_by_name("wireless");
	return chg->wlc_psy;
}

static int max77779_wcin_voltage_max(struct max77779_chgr_data *chg,
				     union power_supply_propval *val)
{
	struct power_supply *wlc_psy;
	int rc;

	if (!max77779_wcin_is_valid(chg)) {
		val->intval = 0;
		return 0;
	}

	wlc_psy = max77779_get_wlc_psy(chg);
	if (!wlc_psy)
		return max77779_get_regulation_voltage_uv(chg, &val->intval);

	rc = power_supply_get_property(wlc_psy, POWER_SUPPLY_PROP_VOLTAGE_MAX, val);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't get VOLTAGE_MAX, rc=%d\n", rc);
		return rc;
	}

	return rc;
}

static int max77779_wcin_voltage_now(struct max77779_chgr_data *chg,
				     union power_supply_propval *val)
{
	struct power_supply *wlc_psy;
	int rc;

	if (!max77779_wcin_is_valid(chg)) {
		val->intval = 0;
		return 0;
	}

	wlc_psy = max77779_get_wlc_psy(chg);
	if (!wlc_psy)
		return max77779_read_wcin(chg, &val->intval);

	rc = power_supply_get_property(wlc_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, val);
	if (rc < 0)
		dev_err(chg->dev, "Couldn't get VOLTAGE_NOW, rc=%d\n", rc);

	return rc;
}

#define MAX77779_WCIN_RAW_TO_UA	166

static int max77779_current_check_mode(struct max77779_chgr_data *data)
{
	int ret;
	u8 reg;

	ret = max77779_reg_read(data, MAX77779_CHG_CNFG_00, &reg);
	if (ret < 0)
		return ret;

	return _max77779_chg_cnfg_00_mode_get(reg);
}

/* current is valid only when charger mode is one of the following */
static bool max77779_current_check_chgin_mode(struct max77779_chgr_data *data)
{
	u8 reg;

	reg = max77779_current_check_mode(data);

	return reg == 1 || reg == 4 || reg == 5 || reg == 6 || reg == 7 || reg == 0xc || reg == 0xd;
}

/* current is valid only when charger mode is one of the following */
static bool max77779_current_check_wcin_mode(struct max77779_chgr_data *data)
{
	u8 reg;

	reg = max77779_current_check_mode(data);

	return reg == 0x4 || reg == 0x5 || reg == 0xe || reg == 0xf;
}

/* only valid in mode e, f */
static int max77779_wcin_current_now(struct max77779_chgr_data *data, int *iic)
{
	u16 tmp;
	int ret;

	ret = max77779_readn(data, MAX77779_CHG_WCIN_I_ADC_L, (uint8_t*)&tmp, 2);
	if (ret) {
		pr_err("Failed to read %x\n", MAX77779_CHG_WCIN_I_ADC_L);
		return ret;
	}

	*iic = tmp * MAX77779_WCIN_RAW_TO_UA;
	return 0;
}

static int max77779_wcin_get_prop(struct power_supply *psy,
				  enum power_supply_property psp,
				  union power_supply_propval *val)
{
	struct max77779_chgr_data *chgr = power_supply_get_drvdata(psy);
	int rc = 0;

	if (max77779_init_check(chgr))
		return -EAGAIN;

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = max77779_wcin_is_valid(chgr);
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = max77779_wcin_is_online(chgr);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		rc = max77779_wcin_voltage_now(chgr, val);
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		rc = max77779_wcin_get_ilim_max_ua(chgr, &val->intval);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		rc = max77779_wcin_voltage_max(chgr, val);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = 0;
		if (!max77779_wcin_is_online(chgr) || !max77779_current_check_wcin_mode(chgr))
			break;
		rc = max77779_wcin_current_now(chgr, &val->intval);
		break;
	default:
		return -EINVAL;
	}
	if (rc < 0) {
		pr_debug("Couldn't get prop %d rc = %d\n", psp, rc);
		return -ENODATA;
	}
	return 0;
}

static int max77779_wcin_set_prop(struct power_supply *psy,
				  enum power_supply_property psp,
				  const union power_supply_propval *val)
{
	struct max77779_chgr_data *chgr = power_supply_get_drvdata(psy);
	int rc = 0;

	if (max77779_init_check(chgr))
		return -EAGAIN;

	switch (psp) {
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		rc = max77779_wcin_set_ilim_max_ua(chgr, val->intval);
		pr_debug("%s: DC_ICL=%d (%d)\n", __func__, val->intval, rc);
		break;
	default:
		return -EINVAL;
	}

	return rc;
}

static int max77779_wcin_prop_is_writeable(struct power_supply *psy,
					   enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		return 1;
	default:
		break;
	}

	return 0;
}

static int max77779_gbms_wcin_get_prop(struct power_supply *psy,
				       enum gbms_property psp,
				       union gbms_propval *val)
{
	struct max77779_chgr_data *chgr = power_supply_get_drvdata(psy);

	if (max77779_init_check(chgr))
		return -EAGAIN;

	pr_debug("%s: route to max77779_wcin_get_prop, psp:%d\n", __func__, psp);
	return -ENODATA;
}

static int max77779_gbms_wcin_set_prop(struct power_supply *psy,
				       enum gbms_property psp,
				       const union gbms_propval *val)
{
	struct max77779_chgr_data *chgr = power_supply_get_drvdata(psy);
	int rc = 0;

	if (max77779_init_check(chgr))
		return -EAGAIN;

	switch (psp) {
	/* called from google_cpm when switching chargers */
	case GBMS_PROP_CHARGING_ENABLED:
		rc = max77779_set_charge_enabled(chgr, val->prop.intval > 0,
						 "DC_PSP_ENABLED");
		pr_debug("%s: charging_enabled=%d (%d)\n",
			__func__, val->prop.intval > 0, rc);
		break;
	default:
		pr_debug("%s: route to max77779_wcin_set_prop, psp:%d\n", __func__, psp);
		return -ENODATA;
	}

	return rc;
}

static int max77779_gbms_wcin_prop_is_writeable(struct power_supply *psy,
						enum gbms_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_CURRENT_MAX:
	case GBMS_PROP_CHARGING_ENABLED:
		return 1;
	default:
		break;
	}

	return 0;
}

static struct gbms_desc max77779_wcin_psy_desc = {
	.psy_dsc.name = "dc",
	.psy_dsc.type = POWER_SUPPLY_TYPE_UNKNOWN,
	.psy_dsc.properties = max77779_wcin_props,
	.psy_dsc.num_properties = ARRAY_SIZE(max77779_wcin_props),
	.psy_dsc.get_property = max77779_wcin_get_prop,
	.psy_dsc.set_property = max77779_wcin_set_prop,
	.psy_dsc.property_is_writeable = max77779_wcin_prop_is_writeable,
	.get_property = max77779_gbms_wcin_get_prop,
	.set_property = max77779_gbms_wcin_set_prop,
	.property_is_writeable = max77779_gbms_wcin_prop_is_writeable,
	.forward = true,
};

static int max77779_init_wcin_psy(struct max77779_chgr_data *data)
{
	struct power_supply_config wcin_cfg = {};
	struct device *dev = data->dev;
	const char *name;
	int ret;

	wcin_cfg.drv_data = data;
	wcin_cfg.of_node = dev->of_node;

	if (of_property_read_bool(dev->of_node, "max77779,dc-psy-type-wireless"))
		max77779_wcin_psy_desc.psy_dsc.type = POWER_SUPPLY_TYPE_WIRELESS;

	ret = of_property_read_string(dev->of_node, "max77779,dc-psy-name", &name);
	if (ret == 0) {
		max77779_wcin_psy_desc.psy_dsc.name = devm_kstrdup(dev, name, GFP_KERNEL);
		if (!max77779_wcin_psy_desc.psy_dsc.name)
			return -ENOMEM;
	}

	data->wcin_psy = devm_power_supply_register(data->dev,
					&max77779_wcin_psy_desc.psy_dsc, &wcin_cfg);
	if (IS_ERR(data->wcin_psy))
		return PTR_ERR(data->wcin_psy);

	return 0;
}

static int max77779_chgin_is_online(struct max77779_chgr_data *data)
{
	uint8_t val;
	int ret = max77779_reg_read(data, MAX77779_CHG_DETAILS_00, &val);

	return (ret == 0) && (_max77779_chg_details_00_chgin_dtls_get(val) == 0x2 ||
		_max77779_chg_details_00_chgin_dtls_get(val) == 0x3);
}

/*
 * NOTE: could also check aicl to determine whether the adapter is, in fact,
 * at fault. Possibly qualify this with battery voltage as subpar adapters
 * are likely to flag AICL when the battery is at high voltage.
 */
static int max77779_is_limited(struct max77779_chgr_data *data)
{
	int ret;
	u8 value;

	ret = max77779_reg_read(data, MAX77779_CHG_INT_OK, &value);
	return (ret == 0) && _max77779_chg_int_ok_inlim_ok_get(value) == 0;
}

/* WCIN || CHGIN present, valid  && CHGIN FET is closed */
static int max77779_is_online(struct max77779_chgr_data *data)
{
	uint8_t val;
	int ret;

	ret = max77779_reg_read(data, MAX77779_CHG_DETAILS_00, &val);
	return (ret == 0) && ((_max77779_chg_details_00_chgin_dtls_get(val) == 0x2)||
	       (_max77779_chg_details_00_chgin_dtls_get(val) == 0x3) ||
	       (_max77779_chg_details_00_wcin_dtls_get(val) == 0x2) ||
	       (_max77779_chg_details_00_wcin_dtls_get(val) == 0x3));
}

static int max77779_get_charge_type(struct max77779_chgr_data *data)
{
	int ret;
	uint8_t reg;

	if (!max77779_is_online(data))
		return POWER_SUPPLY_CHARGE_TYPE_NONE;

	ret = max77779_reg_read(data, MAX77779_CHG_DETAILS_01, &reg);
	if (ret < 0)
		return POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;

	switch(_max77779_chg_details_01_chg_dtls_get(reg)) {
	case CHGR_DTLS_DEAD_BATTERY_MODE:
		return POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
	case CHGR_DTLS_FAST_CHARGE_CONST_CURRENT_MODE:
		return POWER_SUPPLY_CHARGE_TYPE_FAST;
	case CHGR_DTLS_FAST_CHARGE_CONST_VOLTAGE_MODE:
	case CHGR_DTLS_TOP_OFF_MODE:
		return POWER_SUPPLY_CHARGE_TYPE_TAPER_EXT;

	case CHGR_DTLS_DONE_MODE:
	case CHGR_DTLS_TIMER_FAULT_MODE:
	case CHGR_DTLS_DETBAT_HIGH_SUSPEND_MODE:
	case CHGR_DTLS_OFF_MODE:
	case CHGR_DTLS_OFF_HIGH_TEMP_MODE:
	case CHGR_DTLS_OFF_WATCHDOG_MODE:
		return POWER_SUPPLY_CHARGE_TYPE_NONE;
	default:
		break;
	}

	return POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
}

static bool max77779_is_full(struct max77779_chgr_data *data)
{
	int vlimit = data->chg_term_voltage;
	int ret, vbatt = 0;

	/* Not in the last voltage index */
	if (vlimit == 0)
		return false;

	/*
	 * Set voltage level to leave CHARGER_DONE (BATT_RL_STATUS_DISCHARGE)
	 * and enter BATT_RL_STATUS_RECHARGE. It sets STATUS_DISCHARGE again
	 * once CHARGER_DONE flag set (return true here)
	 */
	ret = max77779_read_vbatt(data, &vbatt);
	if (ret == 0)
		vbatt = vbatt / 1000;

	if (data->charge_done)
		vlimit -= data->chg_term_volt_debounce;

	/* true when chg_term_voltage==0, false if read error (vbatt==0) */
	return vbatt >= vlimit;
}

static int max77779_get_status(struct max77779_chgr_data *data)
{
	union power_supply_propval prop;
	struct power_supply *wlc_psy;
	uint8_t val;
	int ret;

	if (!max77779_is_online(data)) {
		wlc_psy = max77779_get_wlc_psy(data);
		if (wlc_psy) {
			ret = power_supply_get_property(wlc_psy, POWER_SUPPLY_PROP_PRESENT, &prop);
			if (ret == 0 && prop.intval > 0)
				return POWER_SUPPLY_STATUS_NOT_CHARGING;
		}
		return POWER_SUPPLY_STATUS_DISCHARGING;
	}

	/*
	 * EOC can be made sticky returning POWER_SUPPLY_STATUS_FULL on
	 * ->charge_done. Also need a check on max77779_is_full() or
	 * google_charger will fail to restart charging.
	 */
	ret = max77779_reg_read(data, MAX77779_CHG_DETAILS_01, &val);
	if (ret < 0)
		return POWER_SUPPLY_STATUS_UNKNOWN;

	switch (_max77779_chg_details_01_chg_dtls_get(val)) {
	case CHGR_DTLS_DEAD_BATTERY_MODE:
	case CHGR_DTLS_FAST_CHARGE_CONST_CURRENT_MODE:
	case CHGR_DTLS_FAST_CHARGE_CONST_VOLTAGE_MODE:
	case CHGR_DTLS_TOP_OFF_MODE:
		return POWER_SUPPLY_STATUS_CHARGING;
	case CHGR_DTLS_DONE_MODE:
		/* same as POWER_SUPPLY_PROP_CHARGE_DONE */
		if (!max77779_is_full(data))
			data->charge_done = false;
		if (data->charge_done)
			return POWER_SUPPLY_STATUS_FULL;
		return POWER_SUPPLY_STATUS_NOT_CHARGING;
	case CHGR_DTLS_TIMER_FAULT_MODE:
	case CHGR_DTLS_DETBAT_HIGH_SUSPEND_MODE:
	case CHGR_DTLS_OFF_MODE:
	case CHGR_DTLS_OFF_HIGH_TEMP_MODE:
	case CHGR_DTLS_OFF_WATCHDOG_MODE:
		return POWER_SUPPLY_STATUS_NOT_CHARGING;
	default:
		break;
	}

	return POWER_SUPPLY_STATUS_UNKNOWN;
}

static bool max77779_debounce_cv(struct max77779_chgr_data *data, int vbatt,
				 union gbms_charger_state *chg_state)
{
	return (data->prev_chg_type == POWER_SUPPLY_CHARGE_TYPE_TAPER_EXT &&
		chg_state->f.chg_type == POWER_SUPPLY_CHARGE_TYPE_FAST &&
		data->prev_vbatt != 0 &&
		vbatt >=  data->prev_vbatt - data->fv_cv_debounce);
}

static int max77779_get_chg_chgr_state(struct max77779_chgr_data *data,
				       union gbms_charger_state *chg_state)
{
	int usb_present, usb_valid, dc_present, dc_valid;
	const char *source = "";
	uint8_t dtls, cnfg, cp_enabled = 0;
	int vbatt, icl = 0;
	int rc;

	chg_state->v = 0;
	chg_state->f.chg_status = max77779_get_status(data);
	chg_state->f.chg_type = max77779_get_charge_type(data);
	chg_state->f.flags = gbms_gen_chg_flags(chg_state->f.chg_status,
						chg_state->f.chg_type);

	rc = max77779_reg_read(data, MAX77779_CHG_CNFG_00, &cnfg);
	if (rc == 0) {
		cp_enabled = _max77779_chg_cnfg_00_cp_en_get(cnfg);
		rc = max77779_reg_read(data, MAX77779_CHG_DETAILS_02,
					&dtls);
	}

	/* present when connected, valid when FET is closed */
	/* chgin_sts and wcin_sts not valid in direct charger 4:1 mode */
	usb_present = (rc == 0) && max77779_chgin_is_online(data);
	if (!cp_enabled)
		usb_valid = usb_present && _max77779_chg_details_02_chgin_sts_get(dtls);
	else
		usb_valid = usb_present;

	/* present if in field, valid when FET is closed */
	dc_present = (rc == 0) && max77779_wcin_is_online(data);
	if (!cp_enabled)
		dc_valid = dc_present && _max77779_chg_details_02_wcin_sts_get(dtls);
	else
		dc_valid = dc_present;

	rc = max77779_read_vbatt(data, &vbatt);
	if (rc == 0)
		chg_state->f.vchrg = vbatt / 1000;

	if (chg_state->f.chg_status == POWER_SUPPLY_STATUS_DISCHARGING)
		goto exit_done;

	/* Disable tier matching in wlc */
	if (data->wlc_inlim_cv && dc_valid)
		chg_state->f.vchrg = 0;

	rc = max77779_is_limited(data);
	if (rc > 0)
		chg_state->f.flags |= GBMS_CS_FLAG_ILIM;

	if (data->wlc_inlim_cv && dc_valid) {
		if (chg_state->f.chg_type == POWER_SUPPLY_CHARGE_TYPE_FAST) {
			int fv_uv;
			int ibat;
			int delta;

			rc = max77779_get_regulation_voltage_uv(data, &fv_uv);
			max77779_read_ibat(data, &ibat);
			delta = max(data->fv_cv_margin, (ibat / 1000) * data->fv_cv_dcr);
			if (rc == 0 && vbatt > (fv_uv - delta)) {
				chg_state->f.chg_type = POWER_SUPPLY_CHARGE_TYPE_TAPER_EXT;
				chg_state->f.flags &= ~GBMS_CS_FLAG_CC;
				chg_state->f.flags |= GBMS_CS_FLAG_CV;
				dev_info(data->dev, "Fake CV in CC\n");
			}
		} else if (max77779_debounce_cv(data, vbatt, chg_state)) {
			chg_state->f.chg_type = POWER_SUPPLY_CHARGE_TYPE_TAPER_EXT;
			chg_state->f.flags &= ~GBMS_CS_FLAG_CC;
			chg_state->f.flags |= GBMS_CS_FLAG_CV;
			dev_info(data->dev, "Debounce CV\n");
		}
	}

	data->prev_vbatt = vbatt;
	data->prev_chg_type = chg_state->f.chg_type;

	/* TODO: b/ handle input MUX corner cases */
	if (usb_valid) {
		max77779_chgin_get_ilim_max_ua(data, &icl);
		/* TODO: 'u' only when in sink */
		if (!dc_present)
			source = "U";
		 else if (dc_valid)
			source = "UW";
		 else
			source = "Uw";

	} else if (dc_valid) {
		max77779_wcin_get_ilim_max_ua(data, &icl);

		/* TODO: 'u' only when in sink */
		source = usb_present ? "uW" : "W";
	} else if (usb_present && dc_present) {
		source = "uw";
	} else if (usb_present) {
		source = "u";
	} else if (dc_present) {
		source = "w";
	}

	chg_state->f.icl = icl / 1000;

exit_done:
	pr_debug("MSC_PCS chg_state=%lx [0x%x:%d:%d:%d:%d] chg=%s\n",
		 (unsigned long)chg_state->v,
		 chg_state->f.flags,
		 chg_state->f.chg_type,
		 chg_state->f.chg_status,
		 chg_state->f.vchrg,
		 chg_state->f.icl,
		 source);

	return 0;
}

#define MAX77779_CHGIN_RAW_TO_UA	166

/* only valid in mode 1, 5, 6, 7, c, d */
static int max77779_chgin_current_now(struct max77779_chgr_data *data, int *iic)
{
	u16 tmp;
	int ret;

	ret = max77779_readn(data, MAX77779_CHG_CHGIN_I_ADC_L, (uint8_t*)&tmp, 2);
	if (ret) {
		pr_err("Failed to read %x\n", MAX77779_CHG_CHGIN_I_ADC_L);
		return ret;
	}

	*iic = tmp * MAX77779_CHGIN_RAW_TO_UA;
	return 0;
}

static int max77779_wd_tickle(struct max77779_chgr_data *data)
{
	int ret;

	/* Protect mode register */
	mutex_lock(&data->io_lock);

	ret = max77779_reg_update(data, MAX77779_CHG_CNFG_00,
				  MAX77779_CHG_CNFG_00_WDTCLR_MASK,
				  _max77779_chg_cnfg_00_wdtclr_set(0, 0x1));
	if (ret < 0)
		dev_err(data->dev, "WD Tickle failed %d\n", ret);

	mutex_unlock(&data->io_lock);

	return ret;
}

/* online is used from DC charging to tickle the watchdog (if enabled) */
static int max77779_set_online(struct max77779_chgr_data *data, bool online)
{
	int ret = 0;

	if (data->wden) {
		ret = max77779_wd_tickle(data);
		if (ret < 0)
			pr_err("cannot tickle the watchdog\n");
	}

	if (data->online != online) {
		ret = gvotable_cast_long_vote(data->mode_votable, "OFFLINE",
					      GBMS_CHGR_MODE_STBY_ON, !online);
		data->online = online;
	}

	return ret;
}

#define MAX77779_CHG_TERM_VOL_TOLERANCE 50 /* 50mV */

static int max77779_set_chg_term_voltage(struct max77779_chgr_data *data, int fv_uv)
{
	int vlimit = 0, msc_last = 0;

	if (!data->msc_last_votable)
		data->msc_last_votable = gvotable_election_get_handle("MSC_LAST");
	if (!data->msc_last_votable)
		return -EINVAL;

	msc_last = gvotable_get_current_int_vote(data->msc_last_votable);
	if (msc_last == 1)
		vlimit = (fv_uv / 1000) - MAX77779_CHG_TERM_VOL_TOLERANCE;
	data->chg_term_voltage = vlimit;

	return msc_last;
}

static int max77779_psy_set_property(struct power_supply *psy,
				     enum power_supply_property psp,
				     const union power_supply_propval *pval)
{
	struct max77779_chgr_data *data = power_supply_get_drvdata(psy);
	int ret = 0;
	bool changed = false;

	if (max77779_init_check(data))
		return -EAGAIN;

	switch (psp) {
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		ret = max77779_chgin_set_ilim_max_ua(data, pval->intval);
		pr_debug("%s: icl=%d (%d)\n", __func__, pval->intval, ret);
		break;
	/* Charge current is set to 0 to EOC */
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
	{
		u8 reg, mode;

		ret = max77779_reg_read(data, MAX77779_CHG_CNFG_00, &reg);
		if (ret)
			break;

		mode = _max77779_chg_cnfg_00_mode_get(reg);

		if ((pval->intval > 0 && !_max77779_chg_cnfg_00_cp_en_get(reg)
		   && (!mode || mode == MAX77779_CHGR_MODE_BUCK_ON))
		   || pval->intval != data->cc_max) {
			ret = max77779_set_charger_current_max_ua(data, pval->intval);
			data->cc_max = pval->intval;
			pr_debug("%s: charge_current=%d (%d)\n",
				 __func__, pval->intval, ret);
		   }
	}
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		if (data->uc_data.input_uv != pval->intval)
			changed = true;
		data->uc_data.input_uv = pval->intval;
		pr_debug("%s: input_voltage=%d\n", __func__, pval->intval);
		if (changed)
			power_supply_changed(data->psy);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
	{
		int msc_last, timer;

		ret = max77779_set_regulation_voltage(data, pval->intval);
		pr_debug("%s: charge_voltage=%d (%d)\n",
			__func__, pval->intval, ret);
		if (ret)
			break;
		msc_last = max77779_set_chg_term_voltage(data, pval->intval);
		if (max77779_is_online(data) && msc_last == 1)
			ret = max77779_higher_headroom_enable(data, true);

		/*
		 * set TO_TIME to 10 min before entering the last tier and
		 * change it back to 30 sec when entering the last tier
		 */
		timer = msc_last == 1 ? 30 : 600;
		ret = max77779_set_topoff_timer(data, timer);
		pr_info("%s: timer=%d (%d)\n",
			 __func__, timer, ret);
	}
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		ret = max77779_set_online(data, pval->intval != 0);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
		ret = max77779_set_topoff_current_max_ma(data, pval->intval);
		pr_debug("%s: topoff_current=%d (%d)\n",
			__func__, pval->intval, ret);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	if (ret == 0 && data->wden)
		max77779_wd_tickle(data);


	return ret;
}

static int max77779_read_current_now(struct max77779_chgr_data *data, int *intval)
{
	int ret = 0;

	if (max77779_wcin_is_online(data) && max77779_current_check_wcin_mode(data))
		ret = max77779_wcin_current_now(data, intval);
	else if (max77779_chgin_is_online(data) && max77779_current_check_chgin_mode(data))
		ret = max77779_chgin_current_now(data, intval);
	else
		*intval = 0;

	return ret;
}

static inline int reg_to_deg_mcel(s16 val)
{
	/* LSB: 1/256°C */
	return ((s64) val * 1000) >> 8;
}

static int max77779_read_thm2_temp(struct max77779_chgr_data *data, int *intval)
{
	int ret;
	s16 regval;
	u8 jeita_flags;

	ret = max77779_reg_update(data, MAX77779_CHG_JEITA_CTRL,
				  MAX77779_CHG_JEITA_CTRL_THM2_TEMP_FORCE_MASK,
				  MAX77779_CHG_JEITA_CTRL_THM2_TEMP_FORCE_MASK);
	if (ret) {
		dev_warn(data->dev, "%s: error setting THM2_TEMP_FORCE (%d)\n", __func__, ret);
		return -EINVAL;
	}

	ret = max77779_reg_read(data, MAX77779_CHG_JEITA_FLAGS, &jeita_flags);
	if (ret) {
		dev_warn(data->dev, "%s: error reading jeita flags (%d)\n", __func__, ret);
		return -EINVAL;
	}

	if (!_max77779_chg_jeita_flags_thm2_temp_on_get(jeita_flags)) {
		dev_info(data->dev, "%s: thm2 temp not ready\n", __func__);
		return -EINVAL;
	}

	ret = max77779_readn(data, MAX77779_CHG_THM2_TEMP_L, (u8 *)&regval, 2);
	if (ret) {
		dev_warn(data->dev, "%s: error reading THM2_TEMP (%d)\n", __func__, ret);
		return -EINVAL;
	}

	*intval = reg_to_deg_mcel(regval);
	return 0;
}

static int max77779_vs_thm2_tz_get(struct thermal_zone_device *tzd, int *vs)
{
	struct max77779_chgr_data *data = tzd->devdata;
	int ret;

	if (!vs)
		return -EINVAL;

	ret = max77779_read_thm2_temp(data, vs);
	return ret;
}

static struct thermal_zone_device_ops max77779_vs_thm2_tz_ops = {
	.get_temp = max77779_vs_thm2_tz_get,
};

static int max77779_psy_get_property(struct power_supply *psy,
				     enum power_supply_property psp,
				     union power_supply_propval *pval)
{
	struct max77779_chgr_data *data = power_supply_get_drvdata(psy);
	int rc, ret = 0;

	if (max77779_init_check(data))
		return -EAGAIN;

	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		pval->intval = max77779_get_charge_type(data);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		ret = max77779_get_charger_current_max_ua(data, &pval->intval);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		pval->intval = data->uc_data.input_uv;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
		ret = max77779_get_regulation_voltage_uv(data, &pval->intval);
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		pval->intval = max77779_is_online(data);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		pval->intval = max77779_is_online(data);
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		ret = max77779_chgin_get_ilim_max_ua(data, &pval->intval);
		break;
	case POWER_SUPPLY_PROP_STATUS:
		pval->intval = max77779_get_status(data);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		rc = max77779_read_vbatt(data, &pval->intval);
		if (rc < 0)
			pval->intval = rc;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		rc = max77779_read_current_now(data, &pval->intval);
		if (rc < 0)
			pval->intval = rc;
		break;
	default:
		pr_debug("property (%d) unsupported.\n", psp);
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int max77779_psy_is_writeable(struct power_supply *psy,
				 enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
	case POWER_SUPPLY_PROP_VOLTAGE_MAX: /* input voltage limit */
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
	case POWER_SUPPLY_PROP_CURRENT_MAX:
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
		return 1;
	default:
		break;
	}

	return 0;
}

static int max77779_gbms_psy_set_property(struct power_supply *psy,
					  enum gbms_property psp,
					  const union gbms_propval *pval)
{
	struct max77779_chgr_data *data = power_supply_get_drvdata(psy);
	int ret = 0;

	if (max77779_init_check(data))
		return -EAGAIN;

	switch (psp) {
	/* called from google_cpm when switching chargers */
	case GBMS_PROP_CHARGING_ENABLED:
		ret = max77779_set_charge_enabled(data, pval->prop.intval,
						  "PSP_ENABLED");
		pr_debug("%s: charging_enabled=%d (%d)\n",
			__func__, pval->prop.intval, ret);
		break;
	/* called from google_charger on disconnect */
	case GBMS_PROP_CHARGE_DISABLE:
		ret = max77779_set_charge_disable(data, pval->prop.intval,
						  "PSP_DISABLE");
		pr_debug("%s: charge_disable=%d (%d)\n",
			__func__, pval->prop.intval, ret);
		break;
	case GBMS_PROP_TAPER_CONTROL:
		break;
	default:
		pr_debug("%s: route to max77779_psy_set_property, psp:%d\n", __func__, psp);
		ret = -ENODATA;
		break;
	}

	if (ret == 0 && data->wden)
		max77779_wd_tickle(data);


	return ret;
}

static int max77779_gbms_psy_get_property(struct power_supply *psy,
					  enum gbms_property psp,
					  union gbms_propval *pval)
{
	struct max77779_chgr_data *data = power_supply_get_drvdata(psy);
	union gbms_charger_state chg_state;
	int rc, ret = 0;

	if (max77779_init_check(data))
		return -EAGAIN;

	switch (psp) {
	case GBMS_PROP_CHARGE_DISABLE:
		rc = max77779_get_charge_enabled(data, &pval->prop.intval);
		if (rc == 0)
			pval->prop.intval = !pval->prop.intval;
		else
			pval->prop.intval = rc;
		break;
	case GBMS_PROP_CHARGING_ENABLED:
		ret = max77779_get_charge_enabled(data, &pval->prop.intval);
		break;
	case GBMS_PROP_CHARGE_CHARGER_STATE:
		rc = max77779_get_chg_chgr_state(data, &chg_state);
		if (rc == 0)
			pval->int64val = chg_state.v;
		break;
	case GBMS_PROP_INPUT_CURRENT_LIMITED:
		pval->prop.intval = max77779_is_limited(data);
		break;
	case GBMS_PROP_TAPER_CONTROL:
		ret = 0;
		break;
	default:
		pr_debug("%s: route to max77779_psy_get_property, psp:%d\n", __func__, psp);
		ret = -ENODATA;
		break;
	}

	return ret;
}

static int max77779_gbms_psy_is_writeable(struct power_supply *psy,
					  enum gbms_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
	case POWER_SUPPLY_PROP_VOLTAGE_MAX: /* input voltage limit */
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
	case POWER_SUPPLY_PROP_CURRENT_MAX:
	case GBMS_PROP_CHARGING_ENABLED:
	case GBMS_PROP_CHARGE_DISABLE:
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
	case GBMS_PROP_TAPER_CONTROL:
		return 1;
	default:
		break;
	}

	return 0;
}

/*
 * TODO: POWER_SUPPLY_PROP_RERUN_AICL, POWER_SUPPLY_PROP_TEMP
 * POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX
 * POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX
 */
static enum power_supply_property max77779_psy_props[] = {
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,		/* input max_voltage */
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX,
};

static struct gbms_desc max77779_psy_desc = {
	.psy_dsc.name = "max77779-charger",
	.psy_dsc.type = POWER_SUPPLY_TYPE_UNKNOWN,
	.psy_dsc.properties = max77779_psy_props,
	.psy_dsc.num_properties = ARRAY_SIZE(max77779_psy_props),
	.psy_dsc.get_property = max77779_psy_get_property,
	.psy_dsc.set_property = max77779_psy_set_property,
	.psy_dsc.property_is_writeable = max77779_psy_is_writeable,
	.get_property = max77779_gbms_psy_get_property,
	.set_property = max77779_gbms_psy_set_property,
	.property_is_writeable = max77779_gbms_psy_is_writeable,
	.forward = true,
};

static ssize_t show_fship_dtls(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct max77779_chgr_data *data = dev_get_drvdata(dev);
	static char *fship_reason[] = {"None", "PWRONB1", "PWRONB1", "PWR"};
	u8 pmic_rd;
	int ret;

	if (data->fship_dtls != -1)
		goto exit_done;

	if (max77779_init_check(data))
		return -EAGAIN;

	if (!data->pmic_dev) {
		data->pmic_dev = max77779_get_dev(data->dev, MAX77779_PMIC_OF_NAME);
		if (!data->pmic_dev) {
			dev_err(dev, "Error finding pmic\n");
			return -EIO;
		}
	}

	mutex_lock(&data->io_lock);
	ret = max77779_external_pmic_reg_read(data->pmic_dev, MAX77779_PMIC_INT_MASK, &pmic_rd);
	if (ret < 0)
		goto unlock;

	if (_max77779_pmic_int_mask_fship_not_rd_get(pmic_rd)) {
		u8 fship_dtls;

		ret = max77779_reg_read(data, MAX77779_CHG_DETAILS_04,
					&fship_dtls);
		if (ret < 0)
			goto unlock;

		data->fship_dtls =
			_max77779_chg_details_04_fship_exit_dtls_get(fship_dtls);

		pmic_rd = _max77779_pmic_int_mask_fship_not_rd_set(pmic_rd, 1);
		ret = max77779_external_pmic_reg_write(data->pmic_dev, MAX77779_PMIC_INT_MASK, pmic_rd);
		if (ret < 0)
			pr_err("FSHIP: cannot update RD (%d)\n", ret);

	} else {
		data->fship_dtls = 0;
	}
unlock:
	mutex_unlock(&data->io_lock);

	if (ret)
		return ret;
exit_done:
	return scnprintf(buf, PAGE_SIZE, "%d %s\n", data->fship_dtls,
			 fship_reason[data->fship_dtls]);
}

static DEVICE_ATTR(fship_dtls, 0444, show_fship_dtls, NULL);

/* -- BCL ------------------------------------------------------------------ */

static int vdroop2_ok_get(void *d, u64 *val)
{
	struct max77779_chgr_data *data = d;
	int ret = 0;
	u8 chg_dtls1;

	if (max77779_init_check(data))
		return -EAGAIN;

	ret = max77779_reg_read(data, MAX77779_CHG_DETAILS_01, &chg_dtls1);
	if (ret < 0)
		return -ENODEV;

	*val = _max77779_chg_details_01_vdroop2_ok_get(chg_dtls1);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(vdroop2_ok_fops, vdroop2_ok_get, NULL, "%llu\n");

static int vdp1_stp_bst_get(void *d, u64 *val)
{
	struct max77779_chgr_data *data = d;
	int ret = 0;
	u8 chg_cnfg17;

	if (max77779_init_check(data))
		return -EAGAIN;

	ret = max77779_reg_read(data, MAX77779_CHG_CNFG_17, &chg_cnfg17);
	if (ret < 0)
		return -ENODEV;

	*val = _max77779_chg_cnfg_17_vdp1_stp_bst_get(chg_cnfg17);
	return 0;
}

static int vdp1_stp_bst_set(void *d, u64 val)
{
	struct max77779_chgr_data *data = d;
	const u8 vdp1_stp_bst = (val > 0)? 0x1 : 0x0;

	if (max77779_init_check(data))
		return -EAGAIN;

	return max77779_reg_update(data, MAX77779_CHG_CNFG_17,
				   MAX77779_CHG_CNFG_17_VDP1_STP_BST_MASK,
				   _max77779_chg_cnfg_17_vdp1_stp_bst_set(0, vdp1_stp_bst));
}

DEFINE_SIMPLE_ATTRIBUTE(vdp1_stp_bst_fops, vdp1_stp_bst_get, vdp1_stp_bst_set, "%llu\n");

static int vdp2_stp_bst_get(void *d, u64 *val)
{
	struct max77779_chgr_data *data = d;
	int ret = 0;
	u8 chg_cnfg17;

	if (max77779_init_check(data))
		return -EAGAIN;

	ret = max77779_reg_read(data, MAX77779_CHG_CNFG_17, &chg_cnfg17);
	if (ret < 0)
		return -ENODEV;

	*val = _max77779_chg_cnfg_17_vdp2_stp_bst_get(chg_cnfg17);
	return 0;
}

static int vdp2_stp_bst_set(void *d, u64 val)
{
	struct max77779_chgr_data *data = d;
	const u8 vdp2_stp_bst = (val > 0)? 0x1 : 0x0;

	if (max77779_init_check(data))
		return -EAGAIN;

	return max77779_reg_update(data, MAX77779_CHG_CNFG_17,
				   MAX77779_CHG_CNFG_17_VDP2_STP_BST_MASK,
				   _max77779_chg_cnfg_17_vdp2_stp_bst_set(0, vdp2_stp_bst));
}

DEFINE_SIMPLE_ATTRIBUTE(vdp2_stp_bst_fops, vdp2_stp_bst_get, vdp2_stp_bst_set, "%llu\n");

/* -- charge control ------------------------------------------------------ */

static int charger_restart_set(void *d, u64 val)
{
	struct max77779_chgr_data *data = d;
	int ret;

	ret = max77779_enable_sw_recharge(data, !!val);
	dev_info(data->dev, "triggered recharge(force=%d) %d\n", !!val, ret);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(charger_restart_fops, NULL, charger_restart_set, "%llu\n");

/* -- debug --------------------------------------------------------------- */

static int max77779_chg_debug_reg_read(void *d, u64 *val)
{
	struct max77779_chgr_data *data = d;
	u8 reg = 0;
	int ret;

	if (max77779_init_check(data))
		return -EAGAIN;

	ret = max77779_reg_read(data, data->debug_reg_address, &reg);
	if (ret)
		return ret;

	*val = reg;
	return 0;
}

static int max77779_chg_debug_reg_write(void *d, u64 val)
{
	struct max77779_chgr_data *data = d;
	u8 reg = (u8) val;

	if (max77779_init_check(data))
		return -EAGAIN;

	pr_warn("debug write reg 0x%x, 0x%x", data->debug_reg_address, reg);
	return max77779_reg_write(data, data->debug_reg_address, reg);
}
DEFINE_SIMPLE_ATTRIBUTE(debug_reg_rw_fops, max77779_chg_debug_reg_read,
			max77779_chg_debug_reg_write, "%02llx\n");

static int max77779_chg_debug_thm2_temp_read(void *d, u64 *val)
{
	struct max77779_chgr_data *data = d;
	int temp;
	int ret;

	if (max77779_init_check(data))
		return -EAGAIN;

	ret = max77779_read_thm2_temp(data, &temp);
	if (ret)
		return ret;

	*val = temp;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(debug_thm2_temp_fops, max77779_chg_debug_thm2_temp_read, NULL, "%llu\n");

static int max77779_chg_debug_cop_warn_read(void *d, u64 *val)
{
	struct max77779_chgr_data *data = d;
	uint32_t reg = 0;
	int ret;

	if (max77779_init_check(data))
		return -EAGAIN;

	ret = max77779_get_cop_warn(data, &reg);
	if (ret == 0)
		*val = reg;

	return ret;
}

static int max77779_chg_debug_cop_warn_write(void *d, u64 val)
{
	struct max77779_chgr_data *data = d;

	if (max77779_init_check(data))
		return -EAGAIN;

	return max77779_set_cop_warn(data, val);
}
DEFINE_SIMPLE_ATTRIBUTE(debug_cop_warn_fops, max77779_chg_debug_cop_warn_read,
			max77779_chg_debug_cop_warn_write, "%llu\n");

static int max77779_chg_debug_cop_limit_read(void *d, u64 *val)
{
	struct max77779_chgr_data *data = d;
	uint32_t reg = 0;
	int ret;

	if (max77779_init_check(data))
		return -EAGAIN;

	ret = max77779_get_cop_limit(data, &reg);
	if (ret == 0)
		*val = reg;

	return ret;
}

static int max77779_chg_debug_cop_limit_write(void *d, u64 val)
{
	struct max77779_chgr_data *data = d;

	if (max77779_init_check(data))
		return -EAGAIN;

	return max77779_set_cop_limit(data, val);
}
DEFINE_SIMPLE_ATTRIBUTE(debug_cop_limit_fops, max77779_chg_debug_cop_limit_read,
			max77779_chg_debug_cop_limit_write, "%llu\n");

static int max77779_chg_debug_cop_is_enabled(void *d, u64 *val)
{
	struct max77779_chgr_data *data = d;

	if (max77779_init_check(data))
		return -EAGAIN;

	*val = max77779_is_cop_enabled(data);

	return 0;
}

static int max77779_chg_debug_cop_enable(void *d, u64 val)
{
	struct max77779_chgr_data *data = d;

	if (max77779_init_check(data))
		return -EAGAIN;

	return max77779_enable_cop(data, val);
}
DEFINE_SIMPLE_ATTRIBUTE(debug_cop_enable_fops, max77779_chg_debug_cop_is_enabled,
			max77779_chg_debug_cop_enable, "%llu\n");

static ssize_t registers_dump_show(struct device *dev, struct device_attribute *attr,
				   char *buf)
{
	struct max77779_chgr_data *data = dev_get_drvdata(dev);
	int ret, i;
	int offset = 0;

	if (!data->regmap) {
		pr_err("Failed to read, no regmap\n");
		return -EIO;
	}

	for (i = 0; i < MAX77779_CHG_NUM_REGS; i++) {
		u8 tmp;
		u32 reg_address = i + MAX77779_CHG_CHGIN_I_ADC_L;

		if (!max77779_chg_is_reg(dev, reg_address))
			continue;

		ret = max77779_reg_read(data, reg_address, &tmp);
		if (ret < 0) {
			dev_err(dev, "[%s]: Failed to dump ret:%d\n", __func__, ret);
			break;
		}

		ret = sysfs_emit_at(buf, offset, "%02x: %02x\n", reg_address, tmp);
		if (!ret) {
			dev_err(dev, "[%s]: Not all registers printed. last:%x\n", __func__,
				reg_address - 1);
			break;
		}
		offset += ret;
	}

	return ret < 0 ? ret : offset;
}
static DEVICE_ATTR_RO(registers_dump);

static int dbg_init_fs(struct max77779_chgr_data *data)
{
	int ret;

	ret = device_create_file(data->dev, &dev_attr_fship_dtls);
	if (ret != 0)
		pr_err("Failed to create fship_dtls, ret=%d\n", ret);

	ret = device_create_file(data->dev, &dev_attr_registers_dump);
	if (ret != 0)
		dev_warn(data->dev, "Failed to create registers_dump, ret=%d\n", ret);

	data->de = debugfs_create_dir("max77779_chg", 0);
	if (IS_ERR_OR_NULL(data->de))
		return -EINVAL;

	debugfs_create_atomic_t("insel_cnt", 0644, data->de, &data->insel_cnt);
	debugfs_create_bool("insel_clear", 0644, data->de, &data->insel_clear);

	debugfs_create_atomic_t("early_topoff_cnt", 0644, data->de,
				&data->early_topoff_cnt);

	/* BCL */
	debugfs_create_file("vdroop2_ok", 0400, data->de, data,
			    &vdroop2_ok_fops);
	debugfs_create_file("vdp1_stp_bst", 0600, data->de, data,
			    &vdp1_stp_bst_fops);
	debugfs_create_file("vdp2_stp_bst", 0600, data->de, data,
			    &vdp2_stp_bst_fops);

	debugfs_create_file("chg_restart", 0600, data->de, data,
			    &charger_restart_fops);

	debugfs_create_file("cop_warn", 0444, data->de, data, &debug_cop_warn_fops);
	debugfs_create_file("cop_limit", 0444, data->de, data, &debug_cop_limit_fops);
	debugfs_create_file("cop_enable", 0444, data->de, data, &debug_cop_enable_fops);

	debugfs_create_u32("address", 0600, data->de, &data->debug_reg_address);
	debugfs_create_file("data", 0600, data->de, data, &debug_reg_rw_fops);
	debugfs_create_file("thm2_temp", 0600, data->de, data, &debug_thm2_temp_fops);

	debugfs_create_u32("inlim_period", 0600, data->de, &data->wcin_inlim_t);
	debugfs_create_u32("inlim_headroom", 0600, data->de, &data->wcin_inlim_headroom);
	debugfs_create_u32("inlim_step", 0600, data->de, &data->wcin_inlim_step);
	debugfs_create_bool("iwcin_ema_disable", 0600, data->de, &data->wcin_ema_disable);
	debugfs_create_u32("iwcin_interval", 0600, data->de, &data->wcin_ema_t);
	debugfs_create_u32("iwcin_ema_alpha", 0600, data->de, &data->wcin_ema_alpha);
	debugfs_create_u32("wcin_inlim_debounce", 0600, data->de,
			   &data->wcin_inlim_debounce_thresh);
	debugfs_create_u32("fv_cv_margin", 0600, data->de, &data->fv_cv_margin);
	debugfs_create_u32("fv_cv_debounce", 0600, data->de, &data->fv_cv_debounce);
	debugfs_create_u32("fv_cv_dcr", 0600, data->de, &data->fv_cv_dcr);
	return 0;
}

bool max77779_chg_is_reg(struct device *dev, unsigned int reg)
{
	switch(reg) {
	case MAX77779_CHG_CHGIN_I_ADC_L ... MAX77779_CHG_JEITA_FLAGS:
	case MAX77779_CHG_COP_CTRL ... MAX77779_CHG_COP_LIMIT_H:
	case MAX77779_CHG_INT ... MAX77779_CHG_INT2:
	case MAX77779_CHG_INT_MASK ... MAX77779_CHG_INT2_MASK:
	case MAX77779_CHG_INT_OK ... MAX77779_BAT_OILO2_CNFG_3:
	case MAX77779_CHG_CUST_TM :
		return true;
	default:
		return false;
	}
}
EXPORT_SYMBOL_GPL(max77779_chg_is_reg);

static void max77779_check_fet_work(struct work_struct *work)
{
	int ret;
	uint8_t chg_dtls, wcin_dtls, cur_wlc_sel = 0;
	struct max77779_chgr_data *data = container_of(work, struct max77779_chgr_data,
						       check_fet_work.work);
	mutex_lock(&data->io_lock);

	ret = max77779_reg_read(data, MAX77779_CHG_DETAILS_00, &chg_dtls);
	if (ret < 0)
		goto unlock;

	ret = max77779_external_chg_insel_read(data->dev, &cur_wlc_sel);
	if (ret < 0)
		goto unlock;

	wcin_dtls = _max77779_chg_details_00_wcin_dtls_get(chg_dtls);
	if ((wcin_dtls == 0x1) && (_max77779_chg_cnfg_12_wcinsel_get(cur_wlc_sel) == 1)) {

		dev_info(data->dev, "Detected fet stuck: Toggling wcin\n");
		ret = max77779_external_chg_reg_update(data->dev, MAX77779_CHG_CNFG_12,
						       MAX77779_CHG_CNFG_12_WCINSEL_MASK,
						       ~MAX77779_CHG_CNFG_12_WCINSEL);
		if (ret < 0)
			goto unlock;

		usleep_range(10 * USEC_PER_MSEC, 15 * USEC_PER_MSEC);

		ret = max77779_external_chg_reg_update(data->dev, MAX77779_CHG_CNFG_12,
						       MAX77779_CHG_CNFG_12_WCINSEL_MASK,
						       MAX77779_CHG_CNFG_12_WCINSEL);
	}

unlock:
	mutex_unlock(&data->io_lock);
}

static irqreturn_t max77779_chgr_irq(int irq, void *d)
{
	struct max77779_chgr_data *data = d;
	u8 chg_int[MAX77779_CHG_INT_COUNT] = { 0 };
	u8 chg_int_clr[MAX77779_CHG_INT_COUNT];
	bool broadcast;
	int ret;

	if (max77779_init_check(data)) {
		dev_warn_ratelimited(data->dev, "%s: irq skipped, irq%d\n", __func__, irq);
		return IRQ_NONE;
	}

	ret = max77779_readn(data, MAX77779_CHG_INT, chg_int, 2);
	if (ret < 0) {
		dev_err_ratelimited(data->dev, "%s i2c error reading INT, IRQ_NONE\n", __func__);
		return IRQ_NONE;
	}

	if ((chg_int[0] & ~max77779_int_mask[0]) == 0 &&
	    (chg_int[1] & ~max77779_int_mask[1]) == 0)
		return IRQ_NONE;
	/*
	 * Only clear the interrupts that are masked. The other interrupts will
	 * be routed to other drivers to handle via the chrg interrupt controller.
	 */
	chg_int_clr[0] = chg_int[0] & ~max77779_int_mask[0];
	chg_int_clr[1] = chg_int[1] & ~max77779_int_mask[1];

	ret = max77779_writen(data, MAX77779_CHG_INT, /* NOTYPO */
                              chg_int_clr, 2);
	if (ret < 0) {
		dev_err_ratelimited(data->dev, "%s i2c error writing INT, IRQ_NONE\n", __func__);
		return IRQ_NONE;
	}

	dev_info_ratelimited(data->dev, "%s INT : %02x %02x\n", __func__, chg_int[0], chg_int[1]);

	/* No need to monitor wcin_inlim when on USB */
	if (chg_int[0] & MAX77779_CHG_INT_CHGIN_I_MASK) {
		if (max77779_chgin_is_online(data))
			max77779_wcin_inlim_work_en(data, false);
		else if (data->wcin_inlim_en)
			max77779_wcin_inlim_work_en(data, true);
	}

	/* always broadcast battery events */
	broadcast = chg_int[0] & MAX77779_CHG_INT_BAT_I_MASK;

	if (chg_int[1] & MAX77779_CHG_INT2_INSEL_I_MASK) {
		pr_debug("%s: INSEL insel_auto_clear=%d (%d)\n", __func__,
			 data->insel_clear, data->insel_clear ? ret : 0);
		atomic_inc(&data->insel_cnt);
	}

	if (chg_int[1] & MAX77779_CHG_INT2_CHG_STA_TO_I_MASK) {
		pr_debug("%s: TOP_OFF\n", __func__);

		if (!max77779_is_full(data)) {
			/*
			 * on small adapter  might enter top-off far from the
			 * last charge tier due to system load.
			 * TODO: check inlim (maybe) and rewrite fv_uv
			 */
			atomic_inc(&data->early_topoff_cnt);
		}

	}

	if (chg_int[0] & MAX77779_CHG_INT_INLIM_I_MASK) {
		int inlim = max77779_is_limited(data);

		pr_debug("%s: INLIM limited: %d\n", __func__, inlim);
		data->wcin_inlim_flag = inlim;
		/* Only turn it off if it got limited */
		if (inlim)
			max77779_inlim_irq_en(data, false);
	}

	if (chg_int[1] & MAX77779_CHG_INT2_CHG_STA_CC_I_MASK)
		pr_debug("%s: CC_MODE\n", __func__);

	if (chg_int[1] & MAX77779_CHG_INT2_CHG_STA_CV_I_MASK)
		pr_debug("%s: CV_MODE\n", __func__);

	if (chg_int[1] & MAX77779_CHG_INT2_CHG_STA_DONE_I_MASK) {
		const bool charge_done = data->charge_done;

		/* reset on disconnect or toggles of enable/disable */
		if (max77779_is_full(data))
			data->charge_done = true;
		broadcast = true;

		pr_debug("%s: CHARGE DONE charge_done=%d->%d\n", __func__,
			 charge_done, data->charge_done);
	}

	/* wired input is changed */
	if (chg_int[0] & MAX77779_CHG_INT_CHGIN_I_MASK) {
		pr_debug("%s: CHGIN charge_done=%d\n", __func__, data->charge_done);

		data->charge_done = false;
		broadcast = true;

		if (data->chgin_psy)
			power_supply_changed(data->chgin_psy);
	}

	/* wireless input is changed */
	if (chg_int[0] & MAX77779_CHG_INT_WCIN_I_MASK) {
		pr_debug("%s: WCIN charge_done=%d\n", __func__, data->charge_done);

		data->charge_done = false;
		broadcast = true;

		mod_delayed_work(system_wq, &data->check_fet_work, msecs_to_jiffies(60));

		if (data->wcin_psy)
			power_supply_changed(data->wcin_psy);
	}

	/* THM2 is changed */
	if (chg_int[0] & MAX77779_CHG_INT_THM2_I_MASK) {
		uint8_t int_ok;
		bool thm2_sts;

		ret = max77779_reg_read(data, MAX77779_CHG_INT_OK, &int_ok);
		if (ret == 0) {
			thm2_sts = (_max77779_chg_int_ok_thm2_ok_get(int_ok))? false : true;

			if (thm2_sts != data->thm2_sts) {
				pr_info("%s: THM2 %d->%d\n", __func__, data->thm2_sts, thm2_sts);
				if (!thm2_sts) {
					pr_info("%s: THM2 run recover...\n", __func__);
					ret = max77779_reg_update(data, MAX77779_CHG_CNFG_13,
						MAX77779_CHG_CNFG_13_THM2_HW_CTRL_MASK, 0);
					if (ret == 0)
						ret = max77779_reg_update(data,
							MAX77779_CHG_CNFG_13,
							MAX77779_CHG_CNFG_13_THM2_HW_CTRL_MASK,
							MAX77779_CHG_CNFG_13_THM2_HW_CTRL_MASK);
				}
				data->thm2_sts = thm2_sts;
			}
		}
	}

	/* someting is changed */
	if (data->psy && broadcast)
		power_supply_changed(data->psy);

	return IRQ_HANDLED;
}

static bool max77779_chrg_irq_is_internal(uint16_t intsrc_sts)
{
	return (((intsrc_sts & 0xff) & ~max77779_int_mask[0]) != 0) ||
	       ((((intsrc_sts & 0xff00) >> 8) & ~max77779_int_mask[1]) != 0);
}

/*
 * Interrupts handled:
 * 0 = BYP_I
 * 1 = THM2_I
 * 2 = INLIM_I
 * 3 = BAT_I
 * 4 = CHG_I
 * 5 = WCIN_I
 * 6 = CHGIN_I
 * 7 = AICL_I
 * 8 = CHG_STA_DONE_I
 * 9 = CHG_STA_TO_I
 * 10 = CHG_STA_CV_I
 * 11 = CHG_STA_CC_I
 * 12 = COP_WARN_I
 * 13 = COP_ALERT_I
 * 14 = COP_LIMIT_WD_I
 * 15 = INSEL_I
 */
static irqreturn_t max77779_chg_irq_handler(int irq, void *ptr)
{
	struct max77779_chgr_data *data = ptr;
	int sub_irq;
	u16 intsrc_sts;
	int offset, ret = IRQ_NONE;
	u16 irq_handled = 0;

	if (max77779_init_check(data)) {
		dev_warn_ratelimited(data->dev, "%s: irq skipped, irq%d\n", __func__, irq);
		return IRQ_NONE;
	}

	ret = max77779_readn(data, MAX77779_CHG_INT, (uint8_t*)&intsrc_sts, 2);
	if (ret) {
		dev_err_ratelimited(data->dev, "%s: read error %d\n", __func__, ret);
		return IRQ_NONE;
	}

	pr_debug("max77779_chg_irq_handler INT: %02x %02x\n",
		(intsrc_sts & 0xff), (intsrc_sts & 0xff00) >> 8);

	for (offset = 0; offset < MAX77779_CHG_NUM_IRQS; offset++)
	{
		if (intsrc_sts & (1 << offset)) {
			sub_irq = irq_find_mapping(data->domain, offset);
			if (sub_irq && !(data->mask & (1 << offset))) {
				irq_handled |= (1 << offset);
				handle_nested_irq(sub_irq);
			}
		}
	}

	ret = max77779_writen(data, MAX77779_CHG_INT, (uint8_t*)&irq_handled, 2); /* NOTYPO */
	if (ret) {
		dev_err_ratelimited(data->dev, "%s: write error %d\n", __func__, ret);
		return IRQ_NONE;
	}

	if (!data->disable_internal_irq_handler && max77779_chrg_irq_is_internal(intsrc_sts))
		ret = max77779_chgr_irq(irq, ptr);

	return irq_handled ? IRQ_HANDLED : ret;
}

/*
 * mode callback will return the entry_id that corresponds to the specific used entry
 */
static int max77779_charger_post_election_work(struct gvotable_election *el, void *d, int ret)
{
	struct max77779_chgr_data *data = (struct max77779_chgr_data *)d;
	struct bms_usecase_data *bms_uc_data = &data->uc_data.usecase_data;
	struct bms_usecase_entry *entry;
	bool blocking;

	/* error occurred, just return out */
	if (ret < 0) {
		dev_err(data->dev, "Error in mode_callback ret:%d\n", ret);
		goto unlock;
	}

	entry = bms_usecase_get_entry(bms_uc_data, ret);
	if (!entry) {
		dev_err(data->dev, "Error retrieving entry %d\n", ret);
		ret = -EINVAL;
		goto unlock;
	}

	if (entry->mode_cb_ret < 0) {
		bms_usecase_free_node(bms_uc_data, entry, BMS_USECASE_FREE_ENTRY);
		ret = entry->mode_cb_ret == BMS_USECASE_NO_HOPS ? 0 : entry->mode_cb_ret;
		goto unlock;
	}

	blocking = !_bms_usecase_meta_async_get(entry->value);
	bms_usecase_add_tail(bms_uc_data, entry);

	mutex_unlock(&data->mode_lock);

	schedule_delayed_work(&data->usecase_work, 0);

	if (blocking) {
		ret = bms_usecase_down(entry);

		/*
		 * If wait is interrupted (ret < 0), check if the node is still
		 * in the queue.
		 * 1. If in queue: Set status=ORPHAN so worker frees it (FREE_ALL).
		 * 2. If not in queue: Worker is done, we must free it (FREE_ENTRY).
		 */
		if (ret < 0) {
			mutex_lock(&bms_uc_data->queue_lock);
			/* Check if list node is still attached */
			if (klist_node_attached(&entry->list_node)) {
				/* Use ORPHAN status to handle ownership transfer safely */
				entry->status = BMS_USECASE_STATUS_ORPHAN;
				mutex_unlock(&bms_uc_data->queue_lock);
				dev_warn(data->dev, "Blocking wait interrupted, orphaning node %d\n",
					 entry->id);

				/* Return error to caller as requested */
				ret = -EAGAIN;
				goto relax;
			}
			mutex_unlock(&bms_uc_data->queue_lock);
		} else {
			ret = entry->uc_work_ret;
		}
		bms_usecase_free_node(bms_uc_data, entry, BMS_USECASE_FREE_ENTRY);
	}

	goto relax;
unlock:
	mutex_unlock(&data->mode_lock);
relax:
	__pm_relax(data->usecase_wake_lock);
	return ret;
}

static int max77779_setup_votables(struct max77779_chgr_data *data)
{
	int ret;

	/* votes might change mode */
	data->mode_votable = gvotable_create_int_election(NULL, NULL,
					max77779_mode_callback,
					data);
	if (!data->mode_votable) {
		ret = -ENXIO;
		dev_err(data->dev, "no mode votable\n");
		return ret;
	}

	gvotable_set_vote2str(data->mode_votable, gvotable_v2s_uint);
	gvotable_register_post_election_work(data->mode_votable, data,
					     max77779_charger_post_election_work);
	/* will use gvotable_get_default() when available */
	gvotable_set_default(data->mode_votable, (void *)GBMS_CHGR_MODE_STBY_ON);
	gvotable_election_set_name(data->mode_votable, GBMS_MODE_VOTABLE);

	/* Wireless charging, DC name is for compat */
	data->dc_suspend_votable =
		gvotable_create_bool_election(NULL,
					     max77779_dc_suspend_vote_callback,
					     data);
	if (!data->dc_suspend_votable) {
		ret = -ENXIO;
		dev_err(data->dev, "no dc_suspend votable\n");
		goto unreg_mode_votable;
	}

	gvotable_set_vote2str(data->dc_suspend_votable, gvotable_v2s_int);
	gvotable_election_set_name(data->dc_suspend_votable, "DC_SUSPEND");

	data->dc_icl_votable =
		gvotable_create_int_election(NULL, gvotable_comparator_int_min,
					     max77779_dcicl_callback,
					     data);
	if (!data->dc_icl_votable) {
		ret = -ENXIO;
		dev_err(data->dev, "no dc_icl votable\n");
		goto unreg_dc_suspend_votable;
	}

	gvotable_set_vote2str(data->dc_icl_votable, gvotable_v2s_uint);
	gvotable_set_default(data->dc_icl_votable, (void *)700000);
	gvotable_election_set_name(data->dc_icl_votable, "DC_ICL");
	gvotable_use_default(data->dc_icl_votable, true);

	data->wlc_spoof_votable =
		gvotable_create_bool_election(NULL,
					      max77779_wlc_spoof_callback,
					      data);
	if (!data->wlc_spoof_votable) {
		ret = -ENXIO;
		dev_err(data->dev, "no wlc_spoof votable\n");
		goto unreg_dcicl_votable;
	}

	gvotable_set_vote2str(data->wlc_spoof_votable, gvotable_v2s_int);
	gvotable_election_set_name(data->wlc_spoof_votable, "WLC_SPOOF");

	return 0;

unreg_dcicl_votable:
	gvotable_destroy_election(data->dc_icl_votable);
unreg_dc_suspend_votable:
	gvotable_destroy_election(data->dc_suspend_votable);
unreg_mode_votable:
	gvotable_destroy_election(data->mode_votable);

	return ret;
}

/* CHG_INT Interrupts */
static void max77779_chg_irq_mask(struct irq_data *d)
{
	struct max77779_chgr_data *data = irq_data_get_irq_chip_data(d);

	data->mask |= BIT(d->hwirq);
	data->mask_u |= BIT(d->hwirq);
}

static void max77779_chg_irq_unmask(struct irq_data *d)
{
	struct max77779_chgr_data *data = irq_data_get_irq_chip_data(d);
	const u8 mask = MAX77779_CHG_INT2_COP_WARN_I_MASK |
			MAX77779_CHG_INT2_COP_ALERT_I_MASK |
			MAX77779_CHG_INT2_COP_LIMIT_WD_I_MASK;
	/*
	 * COP is enabled if a driver registers a COP related interrupt with this driver.
	 * COP warn INT: COP warn interrupt will throttle cc_max to charge pump
	 * COP limit INT: COP limit will set mode to 0 and disable charge pump
	 * COP limit watchdog INT: If watchdog is not pet after 80s, set mode to 0
	 * and disable charge pump
	 */
	if ((d->hwirq > 8) && ((1 << (d->hwirq - 8)) & mask))
		schedule_delayed_work(&data->cop_enable_work, 0);

	data->mask &= ~BIT(d->hwirq);
	data->mask_u |= BIT(d->hwirq);
}

static void max77779_chg_irq_disable(struct irq_data *d)
{
	max77779_chg_irq_mask(d);
}

static void max77779_chg_irq_enable(struct irq_data *d)
{
	max77779_chg_irq_unmask(d);
}

static int max77779_chg_set_irq_type(struct irq_data *d, unsigned int type)
{
	struct max77779_chgr_data *data = irq_data_get_irq_chip_data(d);

	switch (type) {
	case IRQF_TRIGGER_NONE:
	case IRQF_TRIGGER_RISING:
	case IRQF_TRIGGER_FALLING:
	case IRQF_TRIGGER_HIGH:
	case IRQF_TRIGGER_LOW:
		data->trig_type &= (0xf << (d->hwirq * 4));
		data->trig_type |= (type << (d->hwirq * 4));
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static void max77779_chg_bus_lock(struct irq_data *d)
{
	struct max77779_chgr_data *data = irq_data_get_irq_chip_data(d);

	mutex_lock(&data->irq_lock);
}

static void max77779_chg_bus_sync_unlock(struct irq_data *d)
{
	struct max77779_chgr_data *data = irq_data_get_irq_chip_data(d);
	uint16_t intb_mask, offset, value;
	int err;

	mutex_lock(&data->io_lock);

	if (!data->mask_u)
		goto unlock_out;

	err = max77779_readn(data, MAX77779_CHG_INT_MASK, (uint8_t*)&intb_mask, 2);
	if (err < 0) {
		dev_err(data->dev, "Unable to read interrupt mask (%d)\n", err);
		goto unlock_out;
	}

	while (data->mask_u) {
		offset = __ffs(data->mask_u);
		value = !!(data->mask & (1 << offset));

		intb_mask &= ~(1 << offset);
		intb_mask |= value << offset;

		/* clear pending updates */
		data->mask_u &= ~(1 << offset);
	}

	err = max77779_writen(data, MAX77779_CHG_INT_MASK, /* NOTYPO */
			      (uint8_t*)&intb_mask, 2);
	if (err < 0)
		dev_err(data->dev, "Unable to write interrupt mask (%d)\n", err);


 unlock_out:
	mutex_unlock(&data->io_lock);
	mutex_unlock(&data->irq_lock);
}

static struct irq_chip max77779_chg_irq_chip = {
	.name = "max77779_chg_irq",
	.irq_enable = max77779_chg_irq_enable,
	.irq_disable = max77779_chg_irq_disable,
	.irq_mask = max77779_chg_irq_mask,
	.irq_unmask = max77779_chg_irq_unmask,
	.irq_set_type = max77779_chg_set_irq_type,
	.irq_bus_lock = max77779_chg_bus_lock,
	.irq_bus_sync_unlock = max77779_chg_bus_sync_unlock,
};

static int max77779_chg_irq_setup(struct max77779_chgr_data *data)
{
	struct device *dev = data->dev;
	int i, irq;

	mutex_init(&data->irq_lock);

	data->disable_internal_irq_handler =
		of_property_read_bool(dev->of_node, "max77779,disable-internal-irq-handler");

	data->domain = irq_domain_add_linear(dev->of_node, MAX77779_CHG_NUM_IRQS,
					     &irq_domain_simple_ops, data);
	if (!data->domain) {
		dev_err(data->dev, "Unable to get irq domain\n");
		return -ENODEV;
	}

	for (i = 0; i < MAX77779_CHG_NUM_IRQS; i++) {
		irq = irq_create_mapping(data->domain, i);

		if (!irq) {
			dev_err(dev, "failed irq create map\n");
			return -EINVAL;
		}
		irq_set_chip_data(irq, data);
		irq_set_chip_and_handler(irq, &max77779_chg_irq_chip,
				handle_simple_irq);
	}

	return 0;
}

/*
 * Initialization requirements
 * struct max77779_chgr_data *data
 * - dev
 * - regmap
 * - irq_int
 */
int max77779_charger_init(struct max77779_chgr_data *data)
{
	struct power_supply_config chgr_psy_cfg = { 0 };
	struct device *dev = data->dev;
	const char *tmp, *thm2_tz_name;
	u32 usb_otg_mv;
	int ret = 0;
	u8 ping;
#if IS_ENABLED(CONFIG_GPIOLIB)
	struct device_node *dp;
#endif
	ret = max77779_reg_read(data, MAX77779_CHG_CNFG_00, &ping);
	if (ret < 0)
		return -ENODEV;

	/* TODO: PING or read HW version from PMIC */
	data->fship_dtls = -1;
	data->wden = false; /* TODO: read from DT */
	data->mask = 0xFFFFFFFF;
	mutex_init(&data->io_lock);
	mutex_init(&data->prot_lock);
	mutex_init(&data->wcin_inlim_lock);
	mutex_init(&data->mode_lock);
	atomic_set(&data->insel_cnt, 0);
	atomic_set(&data->early_topoff_cnt, 0);
	data->wcin_ema = -1;
	data->wcin_ema_t = MAX77779_WCIN_EMA_TIME_MS;
	data->wcin_ema_alpha = MAX77779_WCIN_EMA_ALPHA_THOUSANDTHS;

	INIT_DELAYED_WORK(&data->cop_enable_work, max77779_cop_enable_work);
	INIT_DELAYED_WORK(&data->wcin_inlim_work, max77779_wcin_inlim_work);
	INIT_DELAYED_WORK(&data->wcin_ema_work, max77779_wcin_ema_work);
	INIT_DELAYED_WORK(&data->usecase_work, max77779_usecase_work);
	INIT_DELAYED_WORK(&data->check_fet_work, max77779_check_fet_work);

	data->usecase_wake_lock = wakeup_source_register(NULL, "max77779-usecase");
	if (!data->usecase_wake_lock) {
		dev_err(dev, "Failed to register wakeup source\n");
		ret = -ENODEV;
		goto free_locks;
	}

	ret = max77779_cop_config(data);
	if (ret < 0)
		dev_warn(dev, "Error configuring COP\n");

	ret = max77779_chg_irq_setup(data);
	if (ret < 0)
		dev_warn(dev, "Error configuring CHG SUB-IRQ Handler\n");

	/* NOTE: only one instance */
	ret = of_property_read_string(dev->of_node, "max77779,psy-name", &tmp);
	if (ret == 0)
		max77779_psy_desc.psy_dsc.name = devm_kstrdup(dev, tmp, GFP_KERNEL);

	chgr_psy_cfg.drv_data = data;
	chgr_psy_cfg.supplied_to = NULL;
	chgr_psy_cfg.num_supplicants = 0;
	data->psy = devm_power_supply_register(dev, &max77779_psy_desc.psy_dsc,
		&chgr_psy_cfg);
	if (IS_ERR(data->psy)) {
		dev_err(dev, "Failed to register psy rc = %ld\n",
			PTR_ERR(data->psy));
		ret = PTR_ERR(data->psy);
		goto unreg_wake_lock;
	}

	ret = dbg_init_fs(data);
	if (ret < 0)
		dev_warn(dev, "Failed to initialize debug fs\n");

	ret = max77779_wdt_enable(data, data->wden);
	if (ret < 0)
		dev_warn(dev, "wd enable=%d failed %d\n", data->wden, ret);

	/* disable fast charge safety timer */
	ret = max77779_reg_update(data, MAX77779_CHG_CNFG_01,
				  MAX77779_CHG_CNFG_01_FCHGTIME_MASK,
				  MAX77779_CHG_CNFG_01_FCHGTIME_CLEAR);
	if (ret < 0)
		dev_warn(dev, "disable fast charge safety timer failed %d\n", ret);

	if (of_property_read_bool(dev->of_node, "google,max77779-thm2-monitor")) {
		/* enable THM2 monitor at 60 degreeC */
		ret = max77779_reg_update(data, MAX77779_CHG_CNFG_13,
					  MAX77779_CHG_CNFG_13_THM2_HW_CTRL_MASK |
					  MAX77779_CHG_CNFG_13_USB_TEMP_THR_MASK,
					  0xA);
		if (ret < 0)
			dev_warn(dev, "enable THM2 monitor failed %d\n", ret);
	} else if (!of_property_read_bool(dev->of_node, "max77779,usb-mon")) {
		/* b/193355117 disable THM2 monitoring */
		ret = max77779_reg_update(data, MAX77779_CHG_CNFG_13,
					  MAX77779_CHG_CNFG_13_THM2_HW_CTRL_MASK |
					  MAX77779_CHG_CNFG_13_USB_TEMP_THR_MASK,
					  0);
		if (ret < 0)
			dev_warn(dev, "disable THM2 monitoring failed %d\n", ret);
	}

	data->otg_changed = false;

	ret = of_property_read_u32(dev->of_node, "max77779,chg-term-volt-debounce",
				   &data->chg_term_volt_debounce);
	if (ret < 0)
		data->chg_term_volt_debounce = CHG_TERM_VOLT_DEBOUNCE;

	ret = of_property_read_u32(dev->of_node, "max77779,usb-otg-mv", &usb_otg_mv);
	if (ret)
		dev_warn(dev, "usb-otg-mv not found, using default\n");

	ret = max77779_otg_vbyp_mv_to_code(&data->uc_data.otg_value, ret ?
					   GS201_OTG_DEFAULT_MV : usb_otg_mv);
	if (ret < 0) {
		dev_dbg(dev, "Invalid value of USB OTG voltage, set to 5000\n");
		data->uc_data.otg_value = MAX77779_CHG_CNFG_11_OTG_VBYP_5000MV;
	}

	data->uc_data.dcin_is_dock = of_property_read_bool(dev->of_node, "max77779,dcin-is-dock");

	ret = of_property_read_u32(dev->of_node, "max77779,wcin-inlim-period", &data->wcin_inlim_t);
	if (ret < 0)
		data->wcin_inlim_t = WCIN_INLIM_T;

	ret = of_property_read_u32(dev->of_node, "max77779,wcin-inlim-headroom",
				   &data->wcin_inlim_headroom);
	if (ret < 0)
		data->wcin_inlim_headroom = WCIN_INLIM_HEADROOM_MA;

	ret = of_property_read_u32(dev->of_node, "max77779,wcin_inlim_step", &data->wcin_inlim_step);
	if (ret < 0)
		data->wcin_inlim_step = WCIN_INLIM_STEP_MV;

	ret = of_property_read_u32(dev->of_node, "max77779,wcin_inlim_debounce_thresh",
				   &data->wcin_inlim_debounce_thresh);
	if (ret < 0)
		data->wcin_inlim_debounce_thresh = MAX77779_WCIN_INLIM_DEFAULT_DEBOUNCE;

	ret = of_property_read_string(dev->of_node, "max77779,thm2_tz_name", &thm2_tz_name);
	if (ret == 0) {
		data->chg_vs_thm2_tz = thermal_zone_device_register(thm2_tz_name, 0, 0,
								    data,
								    &max77779_vs_thm2_tz_ops,
								    NULL, 0, 0);
		if (IS_ERR(data->chg_vs_thm2_tz)) {
			pr_err("chg_vs_thm2_tz name: %s register failed (%ld)\n",
				thm2_tz_name, PTR_ERR(data->chg_vs_thm2_tz));
		} else {
			thermal_zone_device_update(data->chg_vs_thm2_tz, THERMAL_DEVICE_UP);
		}
	}

	data->wlc_inlim_cv = of_property_read_bool(dev->of_node, "max77779,wlc-inlim-cv");

	ret = of_property_read_u32(dev->of_node, "max77779,fv-cv-margin",
				   &data->fv_cv_margin);
	if (ret < 0) {
		dev_dbg(dev, "Using default fv_cv_margin of %d\n", MAX77779_DEFAULT_CV_MARGIN);
		data->fv_cv_margin = MAX77779_DEFAULT_CV_MARGIN;
	}

	ret = of_property_read_u32(dev->of_node, "max77779,fv-cv-debounce",
		&data->fv_cv_debounce);
	if (ret < 0) {
		dev_dbg(dev, "Using default fv_cv_debounce of %d\n", MAX77779_DEFAULT_CV_DEBOUNCE);
		data->fv_cv_debounce = MAX77779_DEFAULT_CV_DEBOUNCE;
	}

	ret = of_property_read_u32(dev->of_node, "max77779,fv-cv-dcr",
		&data->fv_cv_dcr);
	if (ret < 0) {
		dev_dbg(dev, "Using default fv_cv_dcr of %d\n", MAX77779_DEFAULT_CV_DCR);
		data->fv_cv_dcr = MAX77779_DEFAULT_CV_DCR;
	}

	dev_info(dev, "wlc-inlim-cv: %d, fv-cv-margin: %d, fv-cv-debounce: %d, fv-cv-dcr: %d\n",
		data->wlc_inlim_cv, data->fv_cv_margin, data->fv_cv_debounce, data->fv_cv_dcr);

	data->init_complete = 1;
#if IS_ENABLED(CONFIG_GPIOLIB)
	max77779_gpio_init(data);
	data->gpio.parent = dev;
	dp = of_find_node_by_name(dev->of_node, data->gpio.label);
	if (!dp)
		dev_warn(dev, "Failed to find %s DT node\n", data->gpio.label);
	data->gpio.fwnode = of_node_to_fwnode(dp);
	of_node_put(dp);
	ret = devm_gpiochip_add_data(dev, &data->gpio, data);
	dev_dbg(dev, "%d GPIOs registered ret: %d\n", data->gpio.ngpio, ret);
#endif

	/* CHARGER_MODE needs this (initialized to -EPROBE_DEFER) */
	ret = gs201_setup_usecases(&data->uc_data, NULL);
	if (ret) {
		dev_err(dev, "Error, could not setup usecase ret:%d\n", ret);
		return ret;
	}
	INIT_DELAYED_WORK(&data->mode_rerun_work, max77779_mode_rerun_work);

	/* other drivers (ex tcpci) need this. */
	ret = max77779_setup_votables(data);
	if (ret < 0)
		goto unreg_wake_lock;

	ret = max77779_init_wcin_psy(data);
	if (ret < 0)
		dev_warn(dev, "Couldn't register dc power supply (%d)\n", ret);

	/* Init last by probe */
	if (data->irq_int) {
		ret = devm_request_threaded_irq(data->dev, data->irq_int, NULL,
						max77779_chg_irq_handler,
						IRQF_TRIGGER_LOW |
						IRQF_SHARED |
						IRQF_ONESHOT,
						"max77779_charger",
						data);
		if (ret == 0) {
			uint16_t intb_mask;

			/* might cause the isr to be called */
			max77779_chg_irq_handler(-1, data);

			mutex_lock(&data->io_lock);

			ret = max77779_readn(data, MAX77779_CHG_INT_MASK, (uint8_t*)&intb_mask, 2);
			if (ret < 0) {
				dev_err(data->dev, "Unable to read interrupt mask (%d)\n", ret);
				goto unlock;
			}

			intb_mask &= (max77779_int_mask[0] | (max77779_int_mask[1] << 8));

			ret = max77779_writen(data, MAX77779_CHG_INT_MASK, /* NOTYPO */
					      (uint8_t*)&intb_mask, sizeof(intb_mask));
			if (ret < 0)
				dev_warn(dev, "cannot set irq_mask (%d)\n", ret);
unlock:
			mutex_unlock(&data->io_lock);

			device_init_wakeup(data->dev, true);
			ret = enable_irq_wake(data->irq_int);
			if (ret)
				dev_err(data->dev, "Error enabling irq wake ret:%d\n", ret);
		}
	}

	dev_info(dev, "registered as %s\n", max77779_psy_desc.psy_dsc.name);
	return 0;

unreg_wake_lock:
	wakeup_source_unregister(data->usecase_wake_lock);
	mutex_destroy(&data->irq_lock);
free_locks:
	mutex_destroy(&data->io_lock);
	mutex_destroy(&data->prot_lock);
	mutex_destroy(&data->wcin_inlim_lock);
	mutex_destroy(&data->mode_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(max77779_charger_init);

void max77779_charger_remove(struct max77779_chgr_data *data)
{
	disable_irq_wake(data->irq_int);
	device_init_wakeup(data->dev, false);

	debugfs_remove(data->de);

	gvotable_destroy_election(data->mode_votable);
	gvotable_destroy_election(data->dc_suspend_votable);
	gvotable_destroy_election(data->dc_icl_votable);
	gvotable_destroy_election(data->wlc_spoof_votable);

	cancel_delayed_work(&data->cop_enable_work);
	cancel_delayed_work(&data->wcin_inlim_work);
	cancel_delayed_work(&data->usecase_work);
	wakeup_source_unregister(data->usecase_wake_lock);

	mutex_destroy(&data->io_lock);
	mutex_destroy(&data->prot_lock);
	mutex_destroy(&data->wcin_inlim_lock);
	mutex_destroy(&data->mode_lock);

	mutex_destroy(&data->irq_lock);
	bms_usecase_remove(&data->uc_data.usecase_data);
}
EXPORT_SYMBOL_GPL(max77779_charger_remove);

#if IS_ENABLED(CONFIG_PM)
int max77779_charger_pm_suspend(struct device *dev)
{
	struct max77779_chgr_data *data = dev_get_drvdata(dev);

	pm_runtime_get_sync(data->dev);
	dev_dbg(data->dev, "%s\n", __func__);
	cancel_delayed_work_sync(&data->wcin_ema_work);
	data->wcin_ema = -2;
	pm_runtime_put_sync(data->dev);

	return 0;
}
EXPORT_SYMBOL_GPL(max77779_charger_pm_suspend);

int max77779_charger_pm_resume(struct device *dev)
{
	struct max77779_chgr_data *data = dev_get_drvdata(dev);

	pm_runtime_get_sync(data->dev);
	mutex_lock(&data->wcin_inlim_lock);
	data->wcin_inlim_debounce_count = 0;
	if (data->wcin_inlim_en && !data->wcin_ema_disable) {
		data->wcin_ema = -1;
		mod_delayed_work(system_wq, &data->wcin_ema_work, 0);
	}
	mutex_unlock(&data->wcin_inlim_lock);
	dev_dbg(data->dev, "%s\n", __func__);
	pm_runtime_put_sync(data->dev);

	return 0;
}
EXPORT_SYMBOL_GPL(max77779_charger_pm_resume);
#endif

MODULE_DESCRIPTION("Maxim 77779 Charger Driver");
MODULE_AUTHOR("Prasanna Prapancham <prapancham@google.com>");
MODULE_LICENSE("GPL");
