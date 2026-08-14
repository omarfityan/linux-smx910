/* SPDX-License-Identifier: GPL-2.0 */
/*
 * SM5714 battery aging table for the Galaxy Tab S9 Ultra (SM-X910, gts9uwifi).
 *
 * The vendor does not charge this pack to one fixed ceiling.  It de-rates the
 * ceiling -- and the full/recharge decision voltages with it -- as the pack
 * accumulates cycles, so an old pack is charged less hard than a new one.  The
 * table below is transcribed verbatim from THIS DEVICE'S OWN downstream device
 * tree, which is the authority for every hardware-specific value here:
 *
 *   gts9uwifi_eur_open_w00_r03.dts:216   battery,age_data
 *       = <0x00  0x1158 0x1112 0x1126 0x5d      cycles 0
 *          0x12c 0x1144 0x10fe 0x1112 0x5b      cycles 300
 *          0x258 0x1130 0x10ea 0x10fe 0x5a      cycles 600
 *          0x3e9 0x10ea 0x10a4 0x10b8 0x59>;    cycles 1001
 *
 * The rows are {cycle threshold, float voltage, recharge vcell, full vcell,
 * full soc}, and the vendor's supervisor selects a row by cycle count in
 * sec_bat_set_aging_info() (sec_battery.c:2230-2244), assigning exactly the
 * four values this header carries.
 *
 * Values are kept in millivolts, the same unit the device tree uses, so this
 * table can be diffed against the DTS by eye.  Callers that need the
 * power_supply microvolt convention convert at the point of use.
 *
 * WHY THIS IS A SHARED HEADER
 * Two drivers need different columns of the same row, and they must never
 * disagree about which row is in force: the charger programs the float voltage
 * into the SM5714's BATREG field, while the fuel gauge uses the full and
 * recharge voltages to decide when the pack is charged.  A single table read by
 * both is what keeps a de-rated ceiling and a non-de-rated full threshold from
 * drifting apart as the pack ages.
 */
#ifndef __LINUX_POWER_SM5714_BATTERY_AGE_H
#define __LINUX_POWER_SM5714_BATTERY_AGE_H

#include <linux/kernel.h>

struct sm5714_age_step {
	unsigned int cycles;		/* lower bound of this step, in cycles */
	unsigned int float_mv;		/* charge ceiling (BATREG)            */
	unsigned int recharge_mv;	/* resume charging at or below this   */
	unsigned int full_mv;		/* required to declare the pack full  */
	unsigned int full_soc;		/* vendor's full-condition SoC floor  */
};

static const struct sm5714_age_step sm5714_age_data[] = {
	{    0, 4440, 4370, 4390, 93 },
	{  300, 4420, 4350, 4370, 91 },
	{  600, 4400, 4330, 4350, 90 },
	{ 1001, 4330, 4260, 4280, 89 },
};

/**
 * sm5714_age_step_for - select the vendor's aging row for a cycle count
 * @cycles: pack cycle count, as reported by the fuel gauge
 *
 * Mirrors the vendor's selection: the highest row whose cycle threshold has
 * been reached.  A cycle count of zero -- including the value a driver falls
 * back to when the gauge cannot be read -- selects row 0.  That fallback is
 * deliberately the NEW-pack row rather than the most conservative one, because
 * it matches what the vendor's supervisor does with an unread cycle count
 * (pdata->age_step stays 0); a driver that wants to be cautious about an
 * unreadable gauge should decline to program anything rather than silently
 * substitute a different row.
 */
static inline const struct sm5714_age_step *sm5714_age_step_for(unsigned int cycles)
{
	const struct sm5714_age_step *step = &sm5714_age_data[0];
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(sm5714_age_data); i++)
		if (cycles >= sm5714_age_data[i].cycles)
			step = &sm5714_age_data[i];

	return step;
}

#endif /* __LINUX_POWER_SM5714_BATTERY_AGE_H */
