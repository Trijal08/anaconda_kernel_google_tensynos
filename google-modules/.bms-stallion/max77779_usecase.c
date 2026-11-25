/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2023 Google, LLC
 *
 */


#include <linux/kernel.h>
#include <linux/printk.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include "google_bms.h"
#include "max77779.h"
#include "max77779_charger.h"

static int gs201_otg_enable(struct max77779_usecase_data *uc_data, bool enable);
/* ----------------------------------------------------------------------- */

int gs201_wlc_en(struct max77779_usecase_data *uc_data, enum wlc_state_t state)
{
	const int wlc_on = state == WLC_ENABLED;
	int ret;

	pr_debug("%s: wlc_en=%d wlc_on=%d wlc_state=%d\n", __func__,
		 (IS_ERR_OR_NULL(uc_data->wlc_en)
		 ? (int)PTR_ERR(uc_data->wlc_en)
		 : desc_to_gpio(uc_data->wlc_en)),
		 wlc_on, state);

	if (IS_ERR_OR_NULL(uc_data->wlc_en))
		return 0;

	if (state == WLC_SPOOFED && uc_data->wlc_spoof_vbyp > 0) {
		ret = max77779_external_chg_reg_write(uc_data->dev, MAX77779_CHG_CNFG_11,
					     uc_data->wlc_spoof_vbyp);
		pr_debug("%s: MAX77779_CHG_CNFG_11 write to %02x (ret = %d)\n",
			 __func__, uc_data->wlc_spoof_vbyp, ret);
	}

	if (uc_data->slow_wlc_ilim)
		max77779_external_chg_reg_update(uc_data->dev, MAX77779_CHG_CNFG_10,
					MAX77779_CHG_CNFG_10_CHGIN_ILIM_SPEED_MASK,
					_max77779_chg_cnfg_10_chgin_ilim_speed_set(0, wlc_on));

	if (state == WLC_SPOOFED && !IS_ERR_OR_NULL(uc_data->wlc_spoof_gpio))
		gpiod_set_value_cansleep(uc_data->wlc_spoof_gpio, 1);
	gpiod_set_value_cansleep(uc_data->wlc_en, wlc_on);

	return 0;
}

/* RTX reverse wireless charging */
static int gs201_wlc_tx_enable(struct max77779_usecase_data *uc_data, int use_case,
			       bool enable)
{
	int ret = 0;

	pr_debug("%s: use_case:%d enable:%d\n", __func__, use_case, enable);

	if (!enable) {
		ret = max77779_external_chg_reg_write(uc_data->dev, MAX77779_CHG_CNFG_11, 0x0);
		if (ret < 0)
			pr_err("%s: fail to reset MAX77779_CHG_REVERSE_BOOST_VOUT\n",
				__func__);

		ret = gs201_wlc_en(uc_data, WLC_DISABLED);
		if (ret < 0)
			pr_err("%s: cannot disable WLC (%d)\n", __func__, ret);

		return ret;
	}

	ret = gs201_wlc_en(uc_data, WLC_ENABLED);
	if (ret < 0)
		pr_err("%s: cannot enable WLC (%d)\n", __func__, ret);

	if (!IS_ERR_OR_NULL(uc_data->rtx_ready))
		gpiod_set_value_cansleep(uc_data->rtx_ready, 1);

	return ret;
}
static int gs201_wlc_tx_config(struct max77779_usecase_data *uc_data, int use_case)
{
	u8 val;
	int ret = 0;

	/* We need to configure max77779 */
	if (use_case == GSU_MODE_WLC_TX) {
		ret = max77779_external_chg_reg_write(uc_data->dev,
							MAX77779_CHG_CNFG_11,
							MAX77779_CHG_REVERSE_BOOST_VOUT_7V);
		if (ret < 0)
			pr_err("fail to configure MAX77779_CHG_REVERSE_BOOST_VOUT\n");
	} else {
		ret = max77779_external_chg_reg_write(uc_data->dev,
							MAX77779_CHG_CNFG_11,
							0x0);
		if (ret < 0)
			pr_err("fail to reset MAX77779_CHG_REVERSE_BOOST_VOUT\n");
	}
	/* Set WCSM to 1.4A */
	ret = max77779_external_chg_reg_read(uc_data->dev, MAX77779_CHG_CNFG_05, &val);
	if (ret < 0)
		pr_err("%s: fail to read MAX77779_CHG_CNFG_05 ret:%d\n", __func__, ret);

	ret = max77779_external_chg_reg_write(uc_data->dev, MAX77779_CHG_CNFG_05,
		_max77779_chg_cnfg_05_wcsm_ilim_set(val,
				MAX77779_CHG_CNFG_05_WCSM_ILIM_1400_MA));
	if (ret < 0) {
		pr_err("%s: fail to write MAX77779_CHG_CNFG_05 ret:%d\n", __func__, ret);
		return ret;
	}

	return ret;
}

static int gs201_wlc_fw_update_enable(struct max77779_usecase_data *uc_data, bool enable)
{
	return max77779_external_chg_reg_write(uc_data->dev,
					       MAX77779_CHG_CNFG_11,
					       enable ? MAX77779_CHG_REVERSE_BOOST_VOUT_6V : 0x0);
}

static int gs201_otg_update_ilim(struct max77779_usecase_data *uc_data, int enable)
{
	u8 ilim;

	if (uc_data->otg_orig == uc_data->otg_ilim)
		return 0;

	if (enable) {
		int rc;

		rc = max77779_external_chg_reg_read(uc_data->dev, MAX77779_CHG_CNFG_05,
					   	    &uc_data->otg_orig);
		if (rc < 0) {
			pr_err("%s: cannot read otg_ilim (%d), use default\n",
			       __func__, rc);
			uc_data->otg_orig = MAX77779_CHG_CNFG_05_OTG_ILIM_1500MA;
		} else {
			uc_data->otg_orig &= MAX77779_CHG_CNFG_05_OTG_ILIM_MASK;
		}

		ilim = uc_data->otg_ilim;
	} else {
		ilim = uc_data->otg_orig;
	}

	return max77779_external_chg_reg_update(uc_data->dev, MAX77779_CHG_CNFG_05,
						MAX77779_CHG_CNFG_05_OTG_ILIM_MASK,
						ilim);
}

static int gs201_pogo_vout_enable(struct max77779_usecase_data *uc_data,
				  bool enable, bool otg)
{
	pr_debug("%s: enable: %d, otg: %d\n", __func__, enable, otg);

	if (enable && otg) {
		if (!IS_ERR_OR_NULL(uc_data->pogo_vout_en))
			gpiod_set_value_cansleep(uc_data->pogo_vout_en, 0);

		if (!IS_ERR_OR_NULL(uc_data->ext_bst_ctl))
			gpiod_set_value_cansleep(uc_data->ext_bst_ctl, 1);

		return 0;
	}
	if (!otg && !IS_ERR_OR_NULL(uc_data->ext_bst_ctl))
		gpiod_set_value_cansleep(uc_data->ext_bst_ctl, 0);

	if (enable) {
		if (!IS_ERR_OR_NULL(uc_data->bst_on))
			gpiod_set_value_cansleep(uc_data->bst_on, true);

		if (!IS_ERR_OR_NULL(uc_data->pogo_vout_en))
			gpiod_set_value_cansleep(uc_data->pogo_vout_en, true);
	} else {
		if (!IS_ERR_OR_NULL(uc_data->pogo_vout_en))
			gpiod_set_value_cansleep(uc_data->pogo_vout_en, false);

		if (!IS_ERR_OR_NULL(uc_data->bst_on))
			gpiod_set_value_cansleep(uc_data->bst_on, false);
	}

	return 0;
}

/* enable/disable soft-start */
static int gs201_ramp_bypass(struct max77779_usecase_data *uc_data, bool enable)
{
	const u8 value = enable ? MAX77779_CHG_CNFG_00_BYPV_RAMP_BYPASS_MASK : 0;

	return max77779_external_chg_reg_update(uc_data->dev, MAX77779_CHG_CNFG_00,
						MAX77779_CHG_CNFG_00_BYPV_RAMP_BYPASS_MASK,
						value);
}

/* cleanup from every usecase */
static void gs201_force_standby(struct max77779_usecase_data *uc_data)
{
	const u8 insel_mask = MAX77779_CHG_CNFG_12_CHGINSEL_MASK |
			      MAX77779_CHG_CNFG_12_WCINSEL_MASK;
	const u8 insel_value = MAX77779_CHG_CNFG_12_CHGINSEL |
			       MAX77779_CHG_CNFG_12_WCINSEL;
	int ret;

	pr_debug("%s: recovery\n", __func__);

	ret = gs201_ramp_bypass(uc_data, false);
	if (ret < 0)
		pr_err("%s: cannot reset ramp_bypass (%d)\n",
			__func__, ret);

	ret = gs201_pogo_vout_enable(uc_data, false, false);
	if (ret < 0)
		pr_err("%s: cannot tun off pogo_vout (%d)\n", __func__, ret);

	ret = max77779_external_chg_mode_write(uc_data->dev, MAX77779_CHGR_MODE_ALL_OFF);
	if (ret < 0)
		pr_err("%s: cannot reset mode register (%d)\n",
			__func__, ret);

	ret = max77779_external_chg_insel_write(uc_data->dev, insel_mask, insel_value);
	if (ret < 0)
		pr_err("%s: cannot reset insel (%d)\n",
			__func__, ret);

	uc_data->ext_bst_on = false;
	gs201_otg_enable(uc_data, false);

	if (!IS_ERR_OR_NULL(uc_data->rtx_ready))
		gpiod_set_value_cansleep(uc_data->rtx_ready, 0);
}

static int gs201_otg_mode(struct max77779_usecase_data *uc_data, int to)
{
	int ret = -EINVAL;

	pr_debug("%s: to=%d\n", __func__, to);

	if (to == GSU_MODE_USB_OTG) {
		ret = max77779_external_chg_mode_write(uc_data->dev,
						       MAX77779_CHGR_MODE_ALL_OFF);
	}

	return ret;
}

/*
 * This must follow different paths depending on the platforms.
 *
 * NOTE: the USB stack expects VBUS to be on after voting for the usecase.
 */
static int gs201_otg_enable_frs(struct max77779_usecase_data *uc_data, bool enable)
{
	int ret;

	ret = gs201_otg_update_ilim(uc_data, enable);
	if (ret < 0)
		dev_warn(uc_data->dev, "%s: cannot update otg ilim ret:%d\n", __func__, ret);

	return ret;
}

static int gs201_otg_enable(struct max77779_usecase_data *uc_data, bool enable)
{
	pr_debug("%s: enable:%d\n", __func__, enable);

	if (enable) {
		if (!IS_ERR_OR_NULL(uc_data->bst_on))
			gpiod_set_value_cansleep(uc_data->bst_on, 1);

		usleep_range(5 * USEC_PER_MSEC, 5 * USEC_PER_MSEC + 100);

		if (!IS_ERR_OR_NULL(uc_data->ext_bst_ctl))
			gpiod_set_value_cansleep(uc_data->ext_bst_ctl, 1);

	} else {
		if (!IS_ERR_OR_NULL(uc_data->ext_bst_ctl))
			gpiod_set_value_cansleep(uc_data->ext_bst_ctl, 0);

		usleep_range(5 * USEC_PER_MSEC, 5 * USEC_PER_MSEC + 100);

		/*
		 * if ext_rx_otg is enabled, turn off external boost
		 * except when wlcrx/otg are using it
		 */
		if (!IS_ERR_OR_NULL(uc_data->bst_on))
			gpiod_set_value_cansleep(uc_data->bst_on, uc_data->ext_bst_on);
	}

	return 0;
}

static int gs201_wlcrx_ext_bst_enable(struct max77779_usecase_data *uc_data, bool enable)
{
	if (!uc_data->ext_rx_otg)
		return 0;

	pr_debug("%s: enable:%d\n", __func__, enable);

	if (!IS_ERR_OR_NULL(uc_data->bst_on))
		gpiod_set_value_cansleep(uc_data->bst_on, enable);

	return 0;
}

static bool wlcrx_otg_ext_bst_enabled(struct max77779_usecase_data *uc_data,
				      const int use_case, const int from_uc)
{
	if (!uc_data->ext_rx_otg)
		return false;

	switch(use_case) {
	case GSU_MODE_USB_OTG:
	case GSU_MODE_WLC_RX:
	case GSU_MODE_WLC_RX_CHARGE_ENABLED:
	case GSU_MODE_WLC_RX_SPOOFED:
		if (from_uc == GSU_MODE_STANDBY ||
		    from_uc == GSU_MODE_STANDBY_BUCK_ON ||
		    from_uc == GSU_MODE_USB_OTG_WLC_RX ||
		    from_uc == GSU_MODE_USB_OTG_WLC_RX_CHARGE_ENABLED)
			return true;
	break;

	case GSU_MODE_USB_OTG_WLC_RX:
	case GSU_MODE_USB_OTG_WLC_RX_CHARGE_ENABLED:
		if (from_uc == GSU_MODE_USB_OTG ||
		    from_uc == GSU_MODE_WLC_RX ||
		    from_uc == GSU_MODE_WLC_RX_CHARGE_ENABLED ||
		    from_uc == GSU_MODE_WLC_RX_SPOOFED)
			return true;
	break;

	default:
		return false;
	}

	return false;
}

/*
 * Case	USB_chg USB_otg	WLC_chg	WLC_TX	PMIC_Charger	Name
 * -------------------------------------------------------------------------------------
 * 7	0	1	1	0	IF-PMIC-WCIN	USB_OTG_WLC_RX
 * 9	0	1	0	0	0		USB_OTG / USB_OTG_FRS
 * -------------------------------------------------------------------------------------
 * WLC_chg = 0 off, 1 = on, 2 = PPS
 *
 * NOTE: do not call with (cb_data->wlc_rx && cb_data->wlc_tx)
 */

static int gs201_standby_to_otg(struct max77779_usecase_data *uc_data, int use_case)
{
	int ret;

	ret = gs201_otg_enable(uc_data, true);

	if (ret == 0)
		usleep_range(5 * USEC_PER_MSEC, 5 * USEC_PER_MSEC + 100);
	/*
	 * Assumption: gs201_to_usecase() will write back cached values to
	 * CHG_CNFG_00.Mode. At the moment, the cached value at
	 * max77779_mode_callback is 0. If the cached value changes to something
	 * other than 0, then, the code has to be revisited.
	 */

	return ret;
}

/* was b/179816224 WLC_RX -> WLC_RX + OTG (Transition #10) */
static int gs201_wlcrx_to_wlcrx_otg(struct max77779_usecase_data *uc_data)
{
	pr_warn("%s: disabled\n", __func__);
	return 0;
}

static int gs201_to_otg_usecase(struct max77779_usecase_data *uc_data, int use_case,
				int from_uc)
{
	int ret = 0;

	switch (from_uc) {
	/* 9: stby to USB OTG */
	/* 10: stby to USB_OTG_FRS */
	case GSU_MODE_STANDBY:
	case GSU_MODE_STANDBY_BUCK_ON:
		if (use_case != GSU_MODE_USB_OTG_FRS) {
			ret = gs201_standby_to_otg(uc_data, use_case);
			if (ret < 0) {
				dev_err(uc_data->dev, "%s: cannot enable OTG ret:%d\n", __func__,
					ret);
				return ret;
			}
		}
	break;

	case GSU_MODE_USB_CHG:
	case GSU_MODE_USB_CHG_CHARGE_ENABLED:
		/* need to go through stby out of this */
		if (use_case != GSU_MODE_USB_OTG && use_case != GSU_MODE_USB_OTG_FRS)
			return -EINVAL;
		else if (use_case == GSU_MODE_USB_OTG)
			ret = gs201_otg_enable(uc_data, true);
	break;


	case GSU_MODE_WLC_TX:
	break;

	case GSU_MODE_WLC_RX:
	case GSU_MODE_WLC_RX_CHARGE_ENABLED:
	case GSU_MODE_WLC_RX_SPOOFED:
	case GSU_MODE_DOCK:
	case GSU_MODE_WLC_DC:
		if (use_case == GSU_MODE_USB_OTG_WLC_RX ||
		    use_case == GSU_MODE_USB_OTG_WLC_RX_CHARGE_ENABLED ||
		    use_case == GSU_MODE_USB_OTG_WLC_DC) {
			if (uc_data->rx_otg_en) {
				ret = gs201_standby_to_otg(uc_data, use_case);
			} else {
				ret = gs201_wlcrx_to_wlcrx_otg(uc_data);
			}
		}
	break;

	case GSU_MODE_USB_OTG:
	break;
	case GSU_MODE_USB_OTG_WLC_RX:
	case GSU_MODE_USB_OTG_WLC_RX_CHARGE_ENABLED:
	case GSU_MODE_USB_OTG_WLC_DC:
		if (use_case == GSU_MODE_USB_OTG_FRS)
			return -EINVAL;
	break;
	case GSU_MODE_USB_OTG_FRS:
		if (use_case == GSU_MODE_USB_OTG_WLC_RX ||
		    use_case == GSU_MODE_USB_OTG_WLC_RX_CHARGE_ENABLED ||
		    use_case == GSU_MODE_USB_OTG_WLC_DC)
			return -EINVAL;
	break;
	case GSU_MODE_POGO_VOUT:
		if (use_case == GSU_MODE_USB_OTG_POGO_VOUT) {
			ret = max77779_external_chg_reg_write(uc_data->dev,
							      MAX77779_CHG_CNFG_00,
							      MAX77779_CHGR_MODE_BOOST_UNO_ON);

			msleep(40);
			ret = gs201_pogo_vout_enable(uc_data, true, true);
		}
	break;
	case GSU_MODE_USB_OTG_POGO_VOUT:
	break;
	default:
		return -ENOTSUPP;
	}

	return ret;
}

/* handles the transition data->use_case ==> use_case */
int gs201_to_usecase(struct max77779_usecase_data *uc_data, int use_case, int from_uc)
{
	bool rtx_avail = false;
	int ret = 0;

	/* read ext bst state for wlcrx/otg usecases */
	uc_data->ext_bst_on = uc_data->wlc_in_standby ||
			      wlcrx_otg_ext_bst_enabled(uc_data, use_case, from_uc);

	switch (use_case) {
	case GSU_MODE_USB_OTG:
	case GSU_MODE_USB_OTG_FRS:
	case GSU_MODE_USB_OTG_WLC_RX:
	case GSU_MODE_USB_OTG_WLC_RX_CHARGE_ENABLED:
	case GSU_MODE_USB_OTG_POGO_VOUT:
	case GSU_MODE_USB_OTG_WLC_DC:
		ret = gs201_to_otg_usecase(uc_data, use_case, from_uc);
		break;
	case GSU_MODE_WLC_TX:
		rtx_avail = true;
		ret = gs201_wlc_tx_config(uc_data, use_case);
		break;
	case GSU_MODE_WLC_DC:
	case GSU_MODE_WLC_RX:
	case GSU_MODE_WLC_RX_CHARGE_ENABLED:
	case GSU_MODE_WLC_RX_SPOOFED:
	case GSU_MODE_DOCK:
		if (from_uc == GSU_MODE_USB_OTG_WLC_RX ||
		    from_uc == GSU_MODE_USB_OTG_WLC_RX_CHARGE_ENABLED ||
		    from_uc == GSU_MODE_USB_OTG_WLC_DC) {
			if (uc_data->ext_otg_only)
				ret = gs201_otg_enable(uc_data, false);
			else
				ret = gs201_otg_mode(uc_data, GSU_MODE_USB_OTG);
		} else if (from_uc == GSU_MODE_STANDBY ||
			   from_uc == GSU_MODE_STANDBY_BUCK_ON) {
			ret = gs201_wlcrx_ext_bst_enable(uc_data, true);
		}
		break;
	case GSU_MODE_USB_CHG:
	case GSU_MODE_USB_CHG_CHARGE_ENABLED:
		if (from_uc == GSU_MODE_USB_CHG_POGO_VOUT) {
			ret = gs201_pogo_vout_enable(uc_data, false, false);
			if (ret < 0)
				pr_err("%s: cannot tun off pogo_vout (%d)\n", __func__, ret);
		}
		fallthrough;
	case GSU_MODE_USB_DC:
		rtx_avail = false;
		break;
	case GSU_MODE_STANDBY:
	case GSU_MODE_STANDBY_BUCK_ON:
		/* from POGO_VOUT to STBY */
		if (from_uc == GSU_MODE_POGO_VOUT ||
		    from_uc == GSU_MODE_USB_OTG_POGO_VOUT ||
		    from_uc == GSU_MODE_USB_CHG_POGO_VOUT) {
			ret = gs201_pogo_vout_enable(uc_data, false, false);
			if (ret < 0)
				pr_err("%s: cannot tun off pogo_vout (%d)\n", __func__, ret);
		} else if (from_uc == GSU_MODE_USB_OTG_FRS) {
			gs201_otg_enable_frs(uc_data, false);
		} else if (bms_usecase_is_uc_otg(from_uc)) {
			gs201_otg_enable(uc_data, false);
		} else if (from_uc == GSU_MODE_WLC_FWUPDATE) {
			gs201_wlc_fw_update_enable(uc_data, false);
		} else if (from_uc == GSU_MODE_WLC_RX ||
			   from_uc == GSU_MODE_WLC_RX_CHARGE_ENABLED ||
			   from_uc == GSU_MODE_WLC_RX_SPOOFED ||
			   from_uc == GSU_MODE_STANDBY ||
			   from_uc == GSU_MODE_STANDBY_BUCK_ON) {
			/*
			 * WLC_RX State:
			 *
			 * - WLC_RX -> STANDBY:
			 * - Turn external boost OFF unless already in STANDBY state.
			 *
			 * - STANDBY -> STANDBY:
			 * - External boost is ON when connected to WLC, OFF when disconnected.
			 */
			gs201_wlcrx_ext_bst_enable(uc_data, uc_data->wlc_in_standby);
		}

		if ((from_uc == GSU_MODE_WLC_TX) && !IS_ERR_OR_NULL(uc_data->rtx_ready))
			gpiod_set_value_cansleep(uc_data->rtx_ready, 0);

		/* rtx not avail for GSU_MODE_STANDBY_BUCK_ON */
		if (use_case == GSU_MODE_STANDBY_BUCK_ON)
			break;
		fallthrough;
	case GSU_RAW_MODE:
		/* just write the value to the register (it's in stby) */
		rtx_avail = true;
		break;
	case GSU_MODE_USB_WLC_RX:
		break;
	case GSU_MODE_WLC_FWUPDATE:
		rtx_avail = false;
		gs201_wlc_fw_update_enable(uc_data, true);
		break;
	case GSU_MODE_POGO_VOUT:
		ret = gs201_pogo_vout_enable(uc_data, true, false);

		/* wait for ext boost ready */
		if (ret == 0 && from_uc == GSU_MODE_USB_OTG_POGO_VOUT)
			msleep(4);
		break;
	case GSU_MODE_USB_CHG_POGO_VOUT:
		ret = gs201_pogo_vout_enable(uc_data, true, false);
		break;
	default:
		break;
	}

	if (!IS_ERR_OR_NULL(uc_data->rtx_available))
		gpiod_set_value_cansleep(uc_data->rtx_available, rtx_avail);

	return ret;
}

/* finish usecase configuration after max77779 mode register is set */
int gs201_finish_usecase(struct max77779_usecase_data *uc_data, int use_case, int from_uc)
{
	int ret = 0;

	switch (use_case) {
	case GSU_MODE_WLC_TX:
		/* p9412 will not be in RX when powered from EXT */
		ret = gs201_wlc_tx_enable(uc_data, use_case, true);
		if (ret < 0)
			return ret;
		break;
	case GSU_MODE_USB_OTG_FRS:
		if (from_uc != GSU_MODE_USB_OTG_FRS)
			gs201_otg_enable_frs(uc_data, true);
		break;
	default:
		if (from_uc == GSU_MODE_WLC_TX) {
			/* p9412 is already off from insel */
			ret = gs201_wlc_tx_enable(uc_data, use_case, false);
			if (ret < 0)
				return ret;

			ret = gs201_wlc_en(uc_data, WLC_ENABLED); /* re-enable wlc in case of rx */
			if (ret < 0)
				return ret;
		}
		break;
	}

	return ret;
}

#define cb_data_is_inflow_off(cb_data) \
	((cb_data)->chgin_off && (cb_data)->wlcin_off)

/*
 * adjust *INSEL (only one source can be enabled at a given time)
 * NOTE: providing compatibility with input_suspend makes this more complex
 * that it needs to be.
 * TODO(b/) sequoia has back to back FETs to isolate WLC from USB
 * and we likely don't need all this logic here.
 */
static int max77779_set_insel(struct max77779_chgr_data *data,
			      struct max77779_usecase_data *uc_data,
			      const struct max77779_foreach_cb_data *cb_data,
			      int from_uc, int use_case)
{
	const u8 insel_mask = MAX77779_CHG_CNFG_12_CHGINSEL_MASK |
			      MAX77779_CHG_CNFG_12_WCINSEL_MASK;
	int wlc_on = cb_data->wlc_tx && !cb_data->dc_on;
	bool force_wlc = false;
	u8 insel_value = 0;
	int ret;

	if (cb_data->usb_wlc) {
		insel_value |= MAX77779_CHG_CNFG_12_WCINSEL;
		force_wlc = true;
	} else if (cb_data_is_inflow_off(cb_data)) {
		/*
		 * input_suspend masks both inputs but must still allow
		 * TODO: use a separate use case for usb + wlc
		 */
		 force_wlc = true;
	} else if (cb_data->buck_on && !cb_data->chgin_off) {
		insel_value |= MAX77779_CHG_CNFG_12_CHGINSEL;
	} else if (cb_data->wlc_rx && !cb_data->wlcin_off) {

		/* always disable WLC when USB is present */
		if (!cb_data->buck_on)
			insel_value |= MAX77779_CHG_CNFG_12_WCINSEL;
		else
			force_wlc = true;

	} else {
		/* disconnected, do not enable chgin if in input_suspend */
		if (!cb_data->chgin_off)
			insel_value |= MAX77779_CHG_CNFG_12_CHGINSEL;

		/* disconnected, do not enable wlc_in if in input_suspend */
		if (!cb_data->buck_on && (!cb_data->wlcin_off || cb_data->wlc_tx))
			insel_value |= MAX77779_CHG_CNFG_12_WCINSEL;

		force_wlc = true;
	}

	if (cb_data->pogo_vout) {
		/* always disable WCIN when pogo power out */
		insel_value &= ~MAX77779_CHG_CNFG_12_WCINSEL;
	} else if (cb_data->pogo_vin && !cb_data->wlcin_off) {
		/* always disable USB when Dock is present */
		insel_value &= ~MAX77779_CHG_CNFG_12_CHGINSEL;
		insel_value |= MAX77779_CHG_CNFG_12_WCINSEL;
	}

	if (from_uc != use_case || force_wlc || wlc_on) {
		enum wlc_state_t state;
		wlc_on = wlc_on || (insel_value & MAX77779_CHG_CNFG_12_WCINSEL) != 0;

		/* b/182973431 disable WLC_IC while CHGIN, rtx will enable WLC later */
		if (wlc_on)
			state = WLC_ENABLED;
		else if (data->wlc_spoof)
			state = WLC_SPOOFED;
		else
			state = WLC_DISABLED;

		ret = gs201_wlc_en(uc_data, state);

		if (ret < 0)
			pr_err("%s: error wlc_en=%d ret:%d\n", __func__,
			       wlc_on, ret);
	} else {
		u8 value = 0;

		wlc_on = max77779_external_chg_insel_read(uc_data->dev, &value);
		if (wlc_on == 0)
			wlc_on = (value & MAX77779_CHG_CNFG_12_WCINSEL) != 0;
	}

	/* changing [CHGIN|WCIN]_INSEL: works when protection is disabled  */
	ret = max77779_external_chg_insel_write(uc_data->dev, insel_mask, insel_value);

	pr_debug("%s: usecase=%d->%d mask=%x insel=%x wlc_on=%d force_wlc=%d (%d)\n",
		 __func__, from_uc, use_case, insel_mask, insel_value, wlc_on,
		 force_wlc, ret);

	return ret;
}

/* switch to a use case, handle the transitions */
static int max77779_set_usecase(struct max77779_chgr_data *data,
				struct max77779_foreach_cb_data *cb_data,
				int use_case)
{
	struct max77779_usecase_data *uc_data = &data->uc_data;
	const int from_uc = bms_usecase_get_usecase(&uc_data->usecase_data);
	int ret;

	/* Need this only for usecases that control the switches */
	if (uc_data->init_done <= 0) {
		uc_data->psy = data->psy;
		uc_data->init_done = gs201_setup_usecases(uc_data, data->dev->of_node);
	}

	/* always fix/adjust insel (solves multiple input_suspend) */
	ret = max77779_set_insel(data, uc_data, cb_data, from_uc, use_case);
	if (ret < 0) {
		dev_err(data->dev, "use_case=%d->%d set_insel failed ret:%d\n",
			from_uc, use_case, ret);
		return ret;
	}

	/* usbchg+wlctx will call _set_insel() multiple times. */
	if (from_uc == use_case && !uc_data->wlc_standby_changed)
		goto exit_done;

	/* transition from data->use_case to use_case */
	ret = gs201_to_usecase(uc_data, use_case, from_uc);
	if (ret < 0) {
		dev_err(data->dev, "use_case=%d->%d to_usecase failed ret:%d\n",
			from_uc, use_case, ret);
		return ret;
	}

exit_done:

	/* Protect mode register */
	mutex_lock(&data->io_lock);

	/* finally set mode register */
	ret = max77779_external_chg_reg_write(data->dev, MAX77779_CHG_CNFG_00, cb_data->reg);
	pr_debug("%s: CHARGER_MODE=%x ret:%x\n", __func__, cb_data->reg, ret);
	if (ret < 0) {
		dev_err(data->dev,  "use_case=%d->%d CNFG_00=%x failed ret:%d\n",
			from_uc, use_case, cb_data->reg, ret);
		mutex_unlock(&data->io_lock);
		return ret;
	}
	mutex_unlock(&data->io_lock);

	ret = gs201_finish_usecase(uc_data, use_case, from_uc);
	if (ret < 0 && ret != -EAGAIN)
		dev_err(data->dev, "Error finishing usecase config ret:%d\n", ret);

	return ret;
}

void max77779_usecase_work(struct work_struct *work)
{
	struct max77779_chgr_data *data = container_of(work, struct max77779_chgr_data,
						       usecase_work.work);
	struct bms_usecase_data *bms_uc_data = &data->uc_data.usecase_data;
	struct max77779_foreach_cb_data *cb_data;
	struct klist_iter iter;
	struct klist_node *node;
	bool reschedule = false;
	int hops, ret, from_uc, to_uc;
	int err = 0;

	__pm_stay_awake(data->usecase_wake_lock);

	bms_usecase_work_lock(bms_uc_data);

	klist_iter_init(&bms_uc_data->queue, &iter);

	node = bms_usecase_queue_next(bms_uc_data, &iter);
	while (node) {
		struct bms_usecase_entry *entry =
			container_of(node, struct bms_usecase_entry, list_node);

		cb_data = (struct max77779_foreach_cb_data *)entry->cb_data;

		hops = bms_usecase_add_hops(bms_uc_data, entry);
		if (hops < 0) {
			dev_err(data->dev, "Error adding hops (%d)\n", hops);
			err = hops;
			break;
		}
		if (hops) {
			reschedule = true;
			break;
		}

		from_uc = bms_usecase_get_usecase(bms_uc_data);
		to_uc = entry->usecase;

		dev_info(data->dev, "%s:%s use_case=%s(%d)->%s(%d) CHG_CNFG_00=%x->%x node:%d\n",
			__func__, cb_data->reason ? cb_data->reason : "<>",
			bms_usecase_to_str(from_uc), from_uc,
			bms_usecase_to_str(to_uc), to_uc,
			bms_usecase_get_reg(bms_uc_data), cb_data->reg, entry->id);

		if (from_uc != to_uc)
			bms_usecase_uc_setup_notify(bms_uc_data, from_uc, to_uc);

		/* state machine that handle transition between states */
		ret = max77779_set_usecase(data, cb_data, to_uc);
		if (ret < 0) {
			struct max77779_usecase_data *uc_data = &data->uc_data;

			dev_err(data->dev, "Error setting usecase (%d)\n", ret);

			if (ret == -EAGAIN) {
				reschedule = true;
				break;
			}

			err = ret;
			gs201_force_standby(uc_data);

			cb_data->reg = MAX77779_CHGR_MODE_ALL_OFF;
			cb_data->reason = "error";
			entry->usecase = GSU_MODE_STANDBY;
			to_uc = GSU_MODE_STANDBY;
		}

		/* the election is an int election */
		if (!cb_data->reason)
			cb_data->reason = "<>";

		/* this changes the trigger */
		ret = gvotable_election_set_result(cb_data->el, cb_data->reason,
						   (void *)(uintptr_t)cb_data->reg);
		if (ret < 0)
			dev_err(data->dev, "cannot update election %d\n", ret);

		/* set usecase */
		bms_usecase_set(bms_uc_data, entry->usecase, cb_data->reg);

		if (!!err)
			break;

		node = bms_usecase_queue_next(bms_uc_data, &iter);

		/* usecase entry data is invalid after this point */
		/* Mutex locks in bms_usecase_free_node() */
		bms_usecase_free_node(bms_uc_data, entry,
				      !_bms_usecase_meta_async_get(entry->value));

		if (from_uc != to_uc)
			bms_usecase_uc_changed_notify(bms_uc_data, from_uc, to_uc);
	}

	klist_iter_exit(&iter);

	if (!!err) {
		bms_usecase_clear_queue(bms_uc_data, err);
		schedule_delayed_work(&data->mode_rerun_work, msecs_to_jiffies(50));
	}

	if (reschedule) {
		dev_info(data->dev, "Rescheduling\n");
		schedule_delayed_work(&data->usecase_work, 0);
	}

	bms_usecase_work_unlock(bms_uc_data);

	__pm_relax(data->usecase_wake_lock);
}

static int max77779_otg_ilim_ma_to_code(u8 *code, int otg_ilim)
{
	if (otg_ilim == 0)
		*code = 0;
	else if (otg_ilim >= 500 && otg_ilim <= 1500)
		*code = 1 + (otg_ilim - 500) / 100;
	else
		return -EINVAL;

	return 0;
}

int max77779_otg_vbyp_mv_to_code(u8 *code, int vbyp)
{
	if (vbyp >= 12000)
		*code = 0x8c;
	else if (vbyp >= 5000)
		*code = (vbyp - 5000) / 50;
	else
		return -EINVAL;

	return 0;
}

#define GS201_OTG_ILIM_DEFAULT_MA	1500
#define GS201_OTG_VBYPASS_DEFAULT_MV	5100

/* lazy init on the switches */


static bool gs201_setup_usecases_done(struct max77779_usecase_data *uc_data)
{
	return (PTR_ERR(uc_data->wlc_en) != -EPROBE_DEFER) &&
	       (PTR_ERR(uc_data->bst_on) != -EPROBE_DEFER) &&
	       (PTR_ERR(uc_data->ext_bst_mode) != -EPROBE_DEFER) &&
	       (PTR_ERR(uc_data->ext_bst_ctl) != -EPROBE_DEFER) &&
	       (PTR_ERR(uc_data->rtx_ready) != -EPROBE_DEFER) &&
	       (PTR_ERR(uc_data->wlc_spoof_gpio) != -EPROBE_DEFER) &&
	       (PTR_ERR(uc_data->rtx_available) != -EPROBE_DEFER);

	/* TODO: handle platform specific differences.. */
}

static int gs201_usecase_hops(void *data, int from_uc, int to_uc)
{
	bool from_otg = false;
	bool need_stby = false;
	int hop = BMS_USECASE_NO_HOPS;
	struct max77779_usecase_data *uc_data = (struct max77779_usecase_data *)data;

	switch (from_uc) {
	case GSU_MODE_USB_CHG:
	case GSU_MODE_USB_CHG_CHARGE_ENABLED:
		if (to_uc == GSU_MODE_USB_OTG) {
			need_stby = uc_data->ext_bst_ctl >= 0;
			break;
		}

		need_stby = to_uc != GSU_MODE_USB_CHG &&
			    to_uc != GSU_MODE_USB_CHG_CHARGE_ENABLED &&
			    to_uc != GSU_MODE_DOCK &&
			    to_uc != GSU_MODE_USB_DC &&
			    to_uc != GSU_MODE_USB_OTG_FRS;
		break;
	case GSU_MODE_WLC_RX:
	case GSU_MODE_WLC_RX_CHARGE_ENABLED:
	case GSU_MODE_WLC_RX_SPOOFED:
		/* HPP supported by device handled by wlc driver */
		need_stby = to_uc != GSU_MODE_WLC_RX &&
			    to_uc != GSU_MODE_WLC_RX_CHARGE_ENABLED &&
			    to_uc != GSU_MODE_WLC_RX_SPOOFED &&
			    to_uc != GSU_MODE_USB_OTG_WLC_RX &&
			    to_uc != GSU_MODE_USB_OTG_WLC_RX_CHARGE_ENABLED &&
			    to_uc != GSU_MODE_WLC_DC &&
			    to_uc != GSU_MODE_USB_OTG_WLC_DC;
		break;
	case GSU_MODE_WLC_TX:
		need_stby = true;
		break;
	case GSU_MODE_USB_OTG:
		from_otg = true;
		if (to_uc == GSU_MODE_USB_OTG_WLC_RX ||
		    to_uc == GSU_MODE_USB_OTG_WLC_RX_CHARGE_ENABLED ||
		    to_uc == GSU_MODE_USB_OTG_WLC_DC)
			break;

		need_stby = true;
		break;
	case GSU_MODE_USB_OTG_FRS:
		from_otg = true;
		if (to_uc == GSU_MODE_USB_OTG_WLC_RX ||
		    to_uc == GSU_MODE_USB_OTG_WLC_RX_CHARGE_ENABLED ||
		    to_uc == GSU_MODE_USB_OTG_WLC_DC) {
			need_stby = uc_data->ext_bst_ctl >= 0;
			break;
		}

		need_stby = to_uc != GSU_MODE_USB_CHG &&
			    to_uc != GSU_MODE_USB_CHG_CHARGE_ENABLED;
		break;
	case GSU_MODE_USB_OTG_WLC_RX:
	case GSU_MODE_USB_OTG_WLC_RX_CHARGE_ENABLED:
		from_otg = true;
		if (to_uc == GSU_MODE_USB_OTG_FRS) {
			need_stby = uc_data->ext_bst_ctl >= 0;
			break;
		}

		need_stby = to_uc != GSU_MODE_WLC_RX &&
			    to_uc != GSU_MODE_WLC_RX_CHARGE_ENABLED &&
			    to_uc != GSU_MODE_WLC_RX_SPOOFED &&
			    to_uc != GSU_MODE_DOCK &&
			    to_uc != GSU_MODE_USB_OTG &&
			    to_uc != GSU_MODE_USB_OTG_WLC_RX_CHARGE_ENABLED &&
			    to_uc != GSU_MODE_USB_OTG_WLC_RX &&
			    to_uc != GSU_MODE_WLC_DC &&
			    to_uc != GSU_MODE_USB_OTG_WLC_DC;
		break;
	case GSU_MODE_USB_DC:
		need_stby = to_uc != GSU_MODE_USB_CHG &&
			    to_uc != GSU_MODE_USB_CHG_CHARGE_ENABLED;
		break;
	case GSU_MODE_USB_OTG_WLC_DC:
		from_otg = true;
		fallthrough;
	case GSU_MODE_WLC_DC:
		need_stby = to_uc != GSU_MODE_WLC_DC &&
			    to_uc != GSU_MODE_USB_OTG_WLC_DC &&
			    to_uc != GSU_MODE_USB_OTG &&
			    to_uc != GSU_MODE_USB_OTG_WLC_RX_CHARGE_ENABLED &&
			    to_uc != GSU_MODE_USB_OTG_WLC_RX &&
			    to_uc != GSU_MODE_WLC_RX &&
			    to_uc != GSU_MODE_WLC_RX_CHARGE_ENABLED &&
			    to_uc != GSU_MODE_WLC_RX_SPOOFED;
		break;
	case GSU_RAW_MODE:
	case GSU_MODE_FWUPDATE:
	case GSU_MODE_WLC_FWUPDATE:
		need_stby = true;
		break;
	case GSU_MODE_USB_OTG_POGO_VOUT:
		from_otg = true;
		need_stby = to_uc != GSU_MODE_POGO_VOUT &&
			    to_uc != GSU_MODE_USB_OTG;
		break;
	case GSU_MODE_POGO_VOUT:
		need_stby = to_uc != GSU_MODE_USB_CHG_POGO_VOUT &&
			    to_uc != GSU_MODE_USB_OTG_POGO_VOUT;
		break;
	case GSU_MODE_USB_CHG_POGO_VOUT:
		need_stby = to_uc != GSU_MODE_POGO_VOUT;
		break;
	case GSU_MODE_STANDBY:
	case GSU_MODE_STANDBY_BUCK_ON:
	default:
		break;
	}

	if (to_uc == GSU_MODE_STANDBY || to_uc == GSU_MODE_STANDBY_BUCK_ON)
		need_stby = false;
	else if ((to_uc == GSU_RAW_MODE || to_uc == GSU_MODE_USB_WLC_RX) &&
		 (from_uc != GSU_MODE_STANDBY && from_uc != GSU_MODE_STANDBY_BUCK_ON))
		need_stby = true;

	/*
	 * to_uc can not be GSU_MODE_STANDBY_BUCK_ON and need_stby be true, or UC state machine
	 * will loop
	 */
	if (need_stby && bms_usecase_is_uc_wired(to_uc))
		hop = GSU_MODE_STANDBY_BUCK_ON;

	dev_info(uc_data->dev, "%s: use_case=%s(%d)->%s(%d) from_otg=%d need_stby=%d hop:%s\n",
		 __func__,
		 bms_usecase_to_str(from_uc), from_uc,
		 bms_usecase_to_str(to_uc), to_uc,
		 from_otg, need_stby, bms_usecase_to_str(hop));

	if (hop != BMS_USECASE_NO_HOPS)
		return hop;

	return need_stby ? GSU_MODE_STANDBY : BMS_USECASE_NO_HOPS;
}

/*
 * Defines what to fill the cb_data to for intermediete usecases
 */
static void max77779_set_cb_data(struct max77779_foreach_cb_data *cb_data,
				struct max77779_usecase_data *uc_data,
				int to_uc)
{
	switch (to_uc) {
	case GSU_MODE_STANDBY:
		cb_data->stby_on = true;
		break;
	}
}

/*
 * function to fill out intermediate usecases cb_data to set the mode_callback
 * election result: gvotable_election_set_result
 */
static void gs201_populate_cb_data(void *prev_d, void *new_d,
				   void *uc_data, int to_uc, const char *reason)
{
	struct max77779_foreach_cb_data *prev_cb_data = (struct max77779_foreach_cb_data *)prev_d;
	struct max77779_foreach_cb_data *new_cb_data = (struct max77779_foreach_cb_data *)new_d;

	new_cb_data->el = prev_cb_data->el;
	new_cb_data->reason = reason;
	max77779_set_cb_data(new_cb_data, uc_data, to_uc);
	max77779_get_usecase(new_cb_data, uc_data);
}

static int gs201_setup_default_usecase(struct max77779_usecase_data *uc_data)
{
	int ret;
	struct bms_usecase_chg_data *chg_data;

	chg_data = devm_kzalloc(uc_data->dev, sizeof(struct bms_usecase_chg_data), GFP_KERNEL);
	if (!chg_data) {
		dev_err(uc_data->dev, "Error allocating bms_usecase_chg_data!!!\n");
		return -ENOMEM;
	}

	/* external boost */
	uc_data->bst_on = ERR_PTR(-EPROBE_DEFER);
	uc_data->ext_bst_ctl = ERR_PTR(-EPROBE_DEFER);
	uc_data->ext_bst_mode = ERR_PTR(-EPROBE_DEFER);
	uc_data->pogo_vout_en = ERR_PTR(-EPROBE_DEFER);

	uc_data->otg_enable = -EPROBE_DEFER;

	uc_data->wlc_en = ERR_PTR(-EPROBE_DEFER);
	uc_data->rtx_ready = ERR_PTR(-EPROBE_DEFER);
	uc_data->rtx_available = ERR_PTR(-EPROBE_DEFER);

	uc_data->wlc_spoof_gpio = ERR_PTR(-EPROBE_DEFER);

	uc_data->wlc_spoof_vbyp = 0;
	uc_data->init_done = false;
	uc_data->mode_cb_debounce = true;

	/* TODO: override in bootloader and remove */
	ret = max77779_otg_ilim_ma_to_code(&uc_data->otg_ilim,
					   GS201_OTG_ILIM_DEFAULT_MA);
	if (ret < 0)
		uc_data->otg_ilim = MAX77779_CHG_CNFG_05_OTG_ILIM_1500MA;
	ret = max77779_external_chg_reg_read(uc_data->dev, MAX77779_CHG_CNFG_05,
					     &uc_data->otg_orig);
	if (ret == 0) {
		uc_data->otg_orig &= MAX77779_CHG_CNFG_05_OTG_ILIM_MASK;
	} else {
		uc_data->otg_orig = uc_data->otg_ilim;
	}

	ret = max77779_otg_vbyp_mv_to_code(&uc_data->otg_vbyp,
					   GS201_OTG_VBYPASS_DEFAULT_MV);
	if (ret < 0)
		uc_data->otg_vbyp = MAX77779_CHG_CNFG_11_OTG_VBYP_5100MV;

	max77779_external_chg_reg_update(uc_data->dev, MAX77779_CHG_CNFG_12,
					MAX77779_CHG_CNFG_12_WCIN_REG_MASK,
					_max77779_chg_cnfg_12_wcin_reg_set(0, 0x0));

	if (chg_data) {
		uint8_t reg = 0;

		chg_data->dev = uc_data->dev;
		chg_data->uc_data = (void *)uc_data;
		chg_data->hop_func = gs201_usecase_hops;
		chg_data->cb_data_size = sizeof(struct max77779_foreach_cb_data);
		chg_data->populate_cb_data = gs201_populate_cb_data;
		bms_usecase_init(&uc_data->usecase_data, chg_data);

		ret = max77779_external_chg_reg_read(uc_data->dev, MAX77779_CHG_CNFG_00, &reg);
		if (ret < 0)
			dev_err(uc_data->dev, "Error reading mode reg ret:%d\n", ret);
		bms_usecase_set(&uc_data->usecase_data, GSU_MODE_STANDBY, reg);
	}

	return 0;
}

/*
 * Return usecase init status
 * 0: init not complete
 * 1: init complete
 * <0: error
 */
int gs201_setup_usecases(struct max77779_usecase_data *uc_data,
			  struct device_node *node)
{
	u32 data;
	int ret;

	if (!node)
		return gs201_setup_default_usecase(uc_data);

	/* control external boost if present */
	if (PTR_ERR(uc_data->bst_on) == -EPROBE_DEFER) {
		uc_data->bst_on = devm_gpiod_get_optional(uc_data->dev, "max77779,bst-on", 0);
		if (!IS_ERR_OR_NULL(uc_data->bst_on))
			gpiod_direction_output(uc_data->bst_on, 0);
	}
	if (PTR_ERR(uc_data->ext_bst_ctl) == -EPROBE_DEFER)
		uc_data->ext_bst_ctl = devm_gpiod_get_optional(uc_data->dev, "max77779,extbst-ctl", 0);
	if (PTR_ERR(uc_data->ext_bst_mode) == -EPROBE_DEFER) {
		uc_data->ext_bst_mode = devm_gpiod_get_optional(uc_data->dev,
								"max77779,extbst-mode", 0);
		if (!IS_ERR_OR_NULL(uc_data->ext_bst_mode))
			gpiod_set_value_cansleep(uc_data->ext_bst_mode, 0);
	}

	/*  wlc_rx: disable when chgin, CPOUT is safe */
	if (PTR_ERR(uc_data->wlc_en) == -EPROBE_DEFER)
		uc_data->wlc_en = devm_gpiod_get_optional(uc_data->dev, "max77779,wlc-en",
							  GPIOD_ASIS
							  | GPIOD_FLAGS_BIT_NONEXCLUSIVE);

	/*  wlc_rx thermal throttle -> spoof online */
	if (PTR_ERR(uc_data->wlc_spoof_gpio) == -EPROBE_DEFER)
		uc_data->wlc_spoof_gpio = devm_gpiod_get_optional(uc_data->dev,
								  "max77779,wlc-spoof",
								  GPIOD_ASIS);

	/* OPTIONAL: wlc-spoof-vol */
	ret = of_property_read_u32(node, "max77779,wlc-spoof-vbyp", &data);
	if (ret < 0)
		uc_data->wlc_spoof_vbyp = 0;
	else
		uc_data->wlc_spoof_vbyp = data;

	/* OPTIONAL: support wlc_rx -> wlc_rx+otg */
	uc_data->rx_otg_en = of_property_read_bool(node, "max77779,rx-to-rx-otg-en");

	uc_data->otg_wlc_dc_en = of_property_read_bool(node, "max77779,wlc-dc-otg-en");

	/* OPTIONAL: support external boost OTG only */
	uc_data->ext_otg_only = of_property_read_bool(node, "max77779,ext-otg-only");

	/* OPTIONAL: support chrg mode 0x1 during PPS */
	uc_data->chrg_byp_en = of_property_read_bool(node, "max77779,chrg-byp-en");

	/* OPTIONAL: set ILIM speed to slow for WLC */
	uc_data->slow_wlc_ilim = of_property_read_bool(node, "max77779,slow-wlc-ilim");

	/* OPTIONAL: support external boost for WLC_RX and OTG */
	uc_data->ext_rx_otg = of_property_read_bool(node, "max77779,ext-rx-otg");

	if (PTR_ERR(uc_data->rtx_ready) == -EPROBE_DEFER)
		uc_data->rtx_ready = devm_gpiod_get_optional(uc_data->dev, "max77779,rtx-ready", GPIOD_ASIS);

	if (PTR_ERR(uc_data->rtx_available) == -EPROBE_DEFER)
		uc_data->rtx_available = devm_gpiod_get_optional(uc_data->dev, "max77779,rtx-available", GPIOD_ASIS);

	if (PTR_ERR(uc_data->pogo_vout_en) == -EPROBE_DEFER) {
		uc_data->pogo_vout_en = devm_gpiod_get_optional(uc_data->dev,
								"max77779,pogo-vout-sw-en",
								GPIOD_ASIS);
		if (!IS_ERR_OR_NULL(uc_data->pogo_vout_en))
			gpiod_direction_output(uc_data->pogo_vout_en, 0);
	}

	return gs201_setup_usecases_done(uc_data);
}

void gs201_dump_usecasase_config(struct max77779_usecase_data *uc_data)
{
	pr_info("bst_on:%d, ext_bst_ctl: %d, ext_bst_mode:%d, pogo_vout_en:%d\n",
		(IS_ERR_OR_NULL(uc_data->bst_on)
		? (int)PTR_ERR(uc_data->bst_on)
		: desc_to_gpio(uc_data->bst_on)),
		(IS_ERR_OR_NULL(uc_data->ext_bst_ctl)
		? (int)PTR_ERR(uc_data->ext_bst_ctl)
		: desc_to_gpio(uc_data->ext_bst_ctl)),
		(IS_ERR_OR_NULL(uc_data->ext_bst_mode)
		? (int)PTR_ERR(uc_data->ext_bst_mode)
		: desc_to_gpio(uc_data->ext_bst_mode)),
		(IS_ERR_OR_NULL(uc_data->pogo_vout_en)
		? (int)PTR_ERR(uc_data->pogo_vout_en)
		: desc_to_gpio(uc_data->pogo_vout_en)));
	pr_info("wlc_en:%d, chrg_byp_en:%d rtx_ready:%d\n",
		(IS_ERR_OR_NULL(uc_data->wlc_en)
		? (int)PTR_ERR(uc_data->wlc_en)
		: desc_to_gpio(uc_data->wlc_en)),
		uc_data->chrg_byp_en,
		(IS_ERR_OR_NULL(uc_data->rtx_ready)
		? (int)PTR_ERR(uc_data->rtx_ready)
		: desc_to_gpio(uc_data->rtx_ready)));
	pr_info("rtx_available:%d, rx_to_rx_otg:%d ext_otg_only:%d wlc_spoof_gpio:%d\n",
		(IS_ERR_OR_NULL(uc_data->rtx_available)
		? (int)PTR_ERR(uc_data->rtx_available)
		: desc_to_gpio(uc_data->rtx_available)),
		uc_data->rx_otg_en, uc_data->ext_otg_only,
		(IS_ERR_OR_NULL(uc_data->wlc_spoof_gpio)
		? (int)PTR_ERR(uc_data->wlc_spoof_gpio)
		: desc_to_gpio(uc_data->wlc_spoof_gpio)));
}

