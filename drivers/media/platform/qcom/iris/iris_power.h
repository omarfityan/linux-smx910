/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2022-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef __IRIS_POWER_H__
#define __IRIS_POWER_H__

#include <linux/types.h>

struct iris_inst;

int iris_scale_power(struct iris_inst *inst);
void iris_power_data_reset(struct iris_inst *inst);
void iris_power_streamon(struct iris_inst *inst, bool src);
void iris_power_streamoff(struct iris_inst *inst, bool capture);
void iris_power_update_input_rate(struct iris_inst *inst);
void iris_power_capture_done(struct iris_inst *inst, u64 fw_timestamp);

#endif
