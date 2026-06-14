/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Empty sec_tsp_log compatibility shim for the ubuntu-tab stm32_pogo_v3 port.
 *
 * The downstream stm32_pogo_v3 bundle #includes "../sec_tsp_log.h" via its main
 * header but does not call any of its symbols (sec_debug_tsp_log* are only used
 * when CONFIG_SEC_DEBUG_TSP_LOG selects the tsp-log logging variant, which this
 * port does not enable). This stub satisfies the include with no symbols.
 */
#ifndef __SEC_TSP_LOG_SHIM_H__
#define __SEC_TSP_LOG_SHIM_H__

#endif /* __SEC_TSP_LOG_SHIM_H__ */
