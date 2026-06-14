#include "stm32_pogo_v3.h"

#if IS_ENABLED(CONFIG_QCOM_BUS_SCALING)
static void stm32_bus_voting_work(struct work_struct *work)
{
	struct stm32_dev *data = container_of((struct delayed_work *)work, struct stm32_dev, bus_voting_work);
	int ret = 0;

	if (data->stm32_bus_perf_client) {
		if (data->voting_flag == STM32_BUS_VOTING_UP) {
			ret = msm_bus_scale_client_update_request(data->stm32_bus_perf_client,
				(unsigned int)STM32_BUS_VOTING_NONE);
			if (ret)
				input_info(true, &data->client->dev, "%s: voting failed ret:%d\n", __func__, ret);
			else
				data->voting_flag = STM32_BUS_VOTING_NONE;
		}
	}
}
#elif IS_ENABLED(CONFIG_INTERCONNECT)
static void stm32_bus_voting_work(struct work_struct *work)
{
	struct stm32_dev *data = container_of((struct delayed_work *)work, struct stm32_dev, bus_voting_work);
	int ret = 0;

	if (data->stm32_register_ddr) {
		if (data->voting_flag == STM32_BUS_VOTING_UP) {
			ret = icc_set_bw(data->stm32_icc_data, STM32_AB_DATA, STM32_IB_MIN);
			if (ret)
				input_info(true, &data->client->dev, "%s: voting failed ret:%d\n", __func__, ret);
			else
				data->voting_flag = STM32_BUS_VOTING_NONE;
		}
	}
}
#endif

void stm32_voting_suspend(struct stm32_dev *stm32)
{
#if IS_ENABLED(CONFIG_QCOM_BUS_SCALING)
	if (stm32->stm32_bus_perf_client) {
		cancel_delayed_work(&stm32->bus_voting_work);
		stm32_bus_voting_work(&stm32->bus_voting_work.work);
	}
#elif IS_ENABLED(CONFIG_INTERCONNECT)
	if (stm32->stm32_register_ddr) {
		cancel_delayed_work_sync(&stm32->bus_voting_work);
		stm32_bus_voting_work(&stm32->bus_voting_work.work);
	}
#endif
}

void stm32_voting_remove(struct stm32_dev *stm32)
{
	stm32_voting_suspend(stm32);
#if IS_ENABLED(CONFIG_QCOM_BUS_SCALING)
	if (stm32->stm32_bus_perf_client)
		msm_bus_scale_unregister_client(stm32->stm32_bus_perf_client);
#elif IS_ENABLED(CONFIG_INTERCONNECT)
	if (stm32->stm32_register_ddr)
		icc_put(stm32->stm32_icc_data);
#endif
}

int stm32_init_voting(struct stm32_dev *stm32)
{
	if (unlikely(stm32 == NULL)) {
		pr_err("%s %s stm32 is NULL or Error\n", SECLOG, __func__);
		return SEC_ERROR;
	}
#if IS_ENABLED(CONFIG_QCOM_BUS_SCALING)
	stm32->stm32_bus_scale_table = msm_bus_cl_get_pdata_from_dev(&stm32->client->dev);
	if (stm32->stm32_bus_scale_table) {
		stm32->stm32_bus_perf_client = msm_bus_scale_register_client(stm32->stm32_bus_scale_table);
		input_info(true, &client->dev, "%s request success bus_scale:%d\n",
				__func__, stm32->stm32_bus_perf_client);
		stm32->voting_flag = STM32_BUS_VOTING_NONE;
	} else {
		input_info(true, &stm32->client->dev, "%s msm_bus_cl_get_pdata failed\n", __func__);
		return SEC_ERROR;
	}
	if (stm32->stm32_bus_perf_client)
		INIT_DELAYED_WORK(&stm32->bus_voting_work, stm32_bus_voting_work);
#elif IS_ENABLED(CONFIG_INTERCONNECT)
	/*
	 * ubuntu-tab port: DDR bus-perf voting left inert. The downstream form
	 * icc_get(dev, master_id, slave_id) was removed upstream; the mainline
	 * path (of_icc_get) needs an "interconnects" DT property the cover node
	 * does not carry. Voting is only a performance optimisation and is
	 * non-fatal, so disable it (stm32_register_ddr stays 0 → the icc_set_bw /
	 * icc_put paths below are never taken).
	 */
	stm32->stm32_icc_data = NULL;
	stm32->stm32_register_ddr = 0;
#endif
	return SEC_SUCCESS;
}

void stm32_destroy_voting(struct stm32_dev *stm32)
{
	if (unlikely(stm32 == NULL)) {
		pr_err("%s %s stm32 is NULL or Error\n", SECLOG, __func__);
		return;
	}
#if IS_ENABLED(CONFIG_QCOM_BUS_SCALING)
	if (stm32->stm32_bus_perf_client)
		msm_bus_scale_unregister_client(stm32->stm32_bus_perf_client);
#elif IS_ENABLED(CONFIG_INTERCONNECT)
	if (stm32->stm32_register_ddr)
		icc_put(stm32->stm32_icc_data);
#endif
}

MODULE_LICENSE("GPL");
