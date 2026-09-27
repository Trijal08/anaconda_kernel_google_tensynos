/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2021 Google, LLC
 *
 */

#ifndef MAX77779_USECASE_H_
#define MAX77779_USECASE_H_

#include "google_bms_usecase.h"

#define MAX77779_CHG_CNFG_05_WCSM_ILIM_1400_MA 0xA
#define MAX77779_CHG_TX_RETRIES 10

struct max77779_usecase_data {
	bool mode_cb_debounce;			/* debounce mode callback */
	struct gpio_desc *bst_on;		/* ext boost */
	struct gpio_desc *ext_bst_mode;		/* ext boost mode */
	int otg_enable;				/* enter/exit from OTG cases */
	struct gpio_desc *ext_bst_ctl;		/* SEQ VENDOR_EXTBST.EXT_BST_EN */
	bool rx_otg_en;				/* enable WLC_RX -> WLC_RX + OTG case */
	bool otg_wlc_dc_en;			/* enables OTG_WLC_DC usecase */
	bool ext_otg_only;			/* use external OTG only */
	int dc_sw_gpio;				/* WLC-DC switch enable */
	struct gpio_desc *pogo_vout_en;		/* pogo 5V vout */

	int vin_is_valid;			/* MAX20339 STATUS1.vinvalid */

	struct gpio_desc *wlc_en;		/* wlcrx/chgin coex */
	int wlc_vbus_en;			/* b/202526678 */
	bool chrg_byp_en;			/* charger mode 0x1 */
	bool slow_wlc_ilim;	/* ILIM SPEED to slow during WLC */
	struct gpio_desc *wlc_spoof_gpio;	/* wlcrx thermal throttle */
	u32 wlc_spoof_vbyp;			/* wlc spoof VBYP */
	bool ext_rx_otg;			/* ext boost for WLC_RX and OTG */
	bool ext_bst_on;			/* ext boost state */
	bool wlc_in_standby;			/* wlc_rx in input_suspend */
	bool wlc_standby_changed;		/* wlc_rx standby mode changed */

	u8 otg_ilim;				/* TODO: TCPM to control this? */
	u8 otg_vbyp;				/* TODO: TCPM to control this? */
	u8 otg_orig;				/* restore value */
	u8 otg_value;				/* CHG_CNFG_11:VBYPSET for USB OTG Voltage */
	int input_uv;

	struct device *dev;
	int init_done;

	struct gpio_desc *rtx_ready; /* rtx ready gpio from wireless */
	struct gpio_desc *rtx_available; /* rtx supported gpio from wlc, usecase set for UI */

	struct power_supply *psy;

	bool dcin_is_dock;

	struct gvotable_election *force_5v_votable;

	struct bms_usecase_data usecase_data;
};

enum wlc_state_t {
	WLC_DISABLED = 0,
	WLC_ENABLED = 1,
	WLC_SPOOFED = 2,
};

extern int gs201_wlc_en(struct max77779_usecase_data *uc_data, enum wlc_state_t state);
extern int gs201_to_usecase(struct max77779_usecase_data *uc_data, int use_case, int from_uc);
extern int gs201_finish_usecase(struct max77779_usecase_data *uc_data, int use_case, int from_uc);
extern int gs201_setup_usecases(struct max77779_usecase_data *uc_data,
				 struct device_node *node);
extern void gs201_dump_usecasase_config(struct max77779_usecase_data *uc_data);
extern int max77779_otg_vbyp_mv_to_code(u8 *code, int vbyp);
void max77779_usecase_work(struct work_struct *work);

#endif
