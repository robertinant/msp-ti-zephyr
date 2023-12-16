/* SPDX-License-Identifier: Apache-2.0 */

#define DT_DRV_COMPAT ti_mspm0g3xxx_adc

#include <errno.h>

#define LOG_LEVEL CONFIG_ADC_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(adc_mspm0g3xxx);

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/pinctrl.h>
#include <soc.h>

/* Driverlib includes */
#include <ti/driverlib/dl_adc12.h>
#include <ti/driverlib/dl_gpio.h>
#include <ti/driverlib/dl_vref.h>

#define ADC_CONTEXT_USES_KERNEL_TIMER
#include "adc_context.h"

/** Internal sample time unit conversion entry. */
struct adc_mspm0g3xxx_sample_time_entry {
	uint16_t time_us;
	uint8_t reg_value;
};

/** Maps standard unit sample times (us) to internal (raw hal_ti register) values */
/*static const struct adc_mspm0g3xxx_sample_time_entry adc_g3xxx_sample_times[] = {
	{ 2, 1 },
};
*/

struct adc_mspm0g3xxx_data {
	int8_t sampleTime0;
	int8_t sampleTime1;
	uint32_t channelMemCtl[ADC_SYS_NUM_ANALOG_CHAN];
	struct adc_context ctx;
	const struct device *dev;
	uint16_t *buffer;
	uint16_t *repeat_buffer;
	struct k_sem adc_busy_sem;
};

struct adc_mspm0g3xxx_cfg {
	uint32_t base;
	DL_ADC12_ClockConfig ADCClockConfig;
	const struct pinctrl_dev_config * pinctrl;
	void (*irq_cfg_func)(void);
};


static void adc_mspm0g3xxx_isr(const struct device *dev);


static void adc_context_start_sampling(struct adc_context *ctx)
{
	struct adc_mspm0g3xxx_data *data =
		CONTAINER_OF(ctx, struct adc_mspm0g3xxx_data, ctx);

	data->repeat_buffer = data->buffer;
}

static void adc_context_update_buffer_pointer(struct adc_context *ctx,
					      bool repeat)
{
	struct adc_mspm0g3xxx_data *data =
		CONTAINER_OF(ctx, struct adc_mspm0g3xxx_data, ctx);

	if (repeat) {
		data->buffer = data->repeat_buffer;
	} else {
		data->buffer++;
	}
}

static int adc_mspm0g3xxx_init(const struct device *dev)
{
	struct adc_mspm0g3xxx_data *data = dev->data;
	const struct adc_mspm0g3xxx_cfg *config = dev->config;
	int ret;

	LOG_DBG("Initializing %s", dev->name);

	data->dev = dev;

	/* Init GPIO */
	ret = pinctrl_apply_state(config->pinctrl, PINCTRL_STATE_DEFAULT);
	if(ret < 0){
		LOG_ERR("MSPM0 ADC pinctrl error (%d)", ret);
		return ret;
	}
	
	/* Init power */
	DL_ADC12_reset((ADC12_Regs *)config->base);
	DL_ADC12_enablePower((ADC12_Regs *)config->base);
	delay_cycles(POWER_STARTUP_DELAY);
	
	/* Configure clock */
	DL_ADC12_setClockConfig((ADC12_Regs *)config->base,
						    (DL_ADC12_ClockConfig *) &config->ADCClockConfig);
	
	DL_ADC12_setPowerDownMode((ADC12_Regs *)config->base, 
							  DL_ADC12_POWER_DOWN_MODE_MANUAL);

	/* Reset the sample time configuration */
	data->sampleTime0 = -1;
	data->sampleTime1 = -1;

#ifdef UNDO
	/* clear any previous events */
	AUXADCDisable();
	HWREG(AUX_EVCTL_BASE + AUX_EVCTL_O_EVTOMCUFLAGSCLR) =
		(AUX_EVCTL_EVTOMCUFLAGS_AUX_ADC_IRQ | AUX_EVCTL_EVTOMCUFLAGS_AUX_ADC_DONE);

	config->irq_cfg_func();
#endif
	adc_context_unlock_unconditionally(&data->ctx);
	return 0;
}

static int adc_mspm0g3xxx_validate_sampling_time(const struct device *dev, uint16_t acq_time)
{
	if (acq_time == ADC_ACQ_TIME_DEFAULT) {
		return 0;
	}

	/* Current implementation only supports configuration in ADC ticks.
	 *	The maximum value is limited by the hardware
	 */
	if ((ADC_ACQ_TIME_UNIT(acq_time) == ADC_ACQ_TIME_TICKS) &&
		(ADC_ACQ_TIME_VALUE(acq_time) <= ADC12_SCOMP0_VAL_MASK))
	{
		return ADC_ACQ_TIME_VALUE(acq_time);
	}

	LOG_ERR("Sampling time not supported.");
	return -EINVAL;
}

static const DL_VREF_ClockConfig gVREFClockConfig = {
    .clockSel = DL_VREF_CLOCK_BUSCLK,
    .divideRatio = DL_VREF_CLOCK_DIVIDE_1,
};
static DL_VREF_Config gVREFConfig = {
    .vrefEnable     = DL_VREF_ENABLE_ENABLE,
    .bufConfig      = DL_VREF_BUFCONFIG_OUTPUT_2_5V,
    .shModeEnable   = DL_VREF_SHMODE_DISABLE,
    .holdCycleCount = DL_VREF_HOLD_MIN,
    .shCycleCount   = DL_VREF_SH_MIN,
};

static int adc_mspm0g3xxx_config_Vref(int vref_source, 
									  	int vref_val, 
										uint32_t *memCtlConfig)
{
	int error = 0;
	bool initVREF = false;

	switch (vref_source)
	{
		case ADC_REF_VDD_1:
			*memCtlConfig |= DL_ADC12_REFERENCE_VOLTAGE_VDDA;
		break;
		case ADC_REF_EXTERNAL0:
			/* Initialize VREF if needed */
			if ( (VREF->GPRCM.PWREN & VREF_PWREN_ENABLE_MASK) == 
				 VREF_PWREN_ENABLE_DISABLE) 
			{
				gVREFConfig.vrefEnable = DL_VREF_ENABLE_DISABLE;
				initVREF = true;
				*memCtlConfig |= DL_ADC12_REFERENCE_VOLTAGE_EXTREF;	
			}
			else
			{
				/* If VREF was initialized, validate configuration */
				if (DL_VREF_isEnabled(VREF) == false)
				{
					/* VREF is already set to external*/
					*memCtlConfig |= DL_ADC12_REFERENCE_VOLTAGE_EXTREF;	
				}
				else
				{
					/* VREF is already configured to internal, 
					    using external for another channel is not valid */
					return -EINVAL;
				}
			}
		break;
		case ADC_REF_INTERNAL:
			/* Initialize VREF if needed */
			if ( (VREF->GPRCM.PWREN & VREF_PWREN_ENABLE_MASK) == 
				 VREF_PWREN_ENABLE_DISABLE) 
			{
				if (vref_val == 2500) 
				{
					gVREFConfig.bufConfig = DL_VREF_BUFCONFIG_OUTPUT_2_5V;
				}
				else if (vref_val == 1400)
				{
					gVREFConfig.bufConfig = DL_VREF_BUFCONFIG_OUTPUT_1_4V;
				}
				else
				{
					/* Invalid configuration */
					return -EINVAL;
				}
				initVREF = true;
				gVREFConfig.vrefEnable = DL_VREF_ENABLE_ENABLE;
				*memCtlConfig |= DL_ADC12_REFERENCE_VOLTAGE_INTREF;	
			}
			else
			{
				/* If VREF was initialized, validate configuration */
				if (DL_VREF_isEnabled(VREF) == true)
				{
					/* VREF is already set to internal*/
					if ( (vref_val == 2500) && 
						 ( (VREF->CTL0 & VREF_CTL0_BUFCONFIG_MASK) == 
							 VREF_CTL0_BUFCONFIG_OUTPUT2P5V) )
					{
						*memCtlConfig |= DL_ADC12_REFERENCE_VOLTAGE_INTREF;	
					}
					else if ( (vref_val == 1400) &&
							  ( (VREF->CTL0 & VREF_CTL0_BUFCONFIG_MASK) == 
							     VREF_CTL0_BUFCONFIG_OUTPUT1P4V) )
					{
						*memCtlConfig |= DL_ADC12_REFERENCE_VOLTAGE_INTREF;	
					}
					else
					{
						/* VREF is already configured but doesn't match
							requested configuration*/
						return -EINVAL;
					}
				}
				else
				{
					/* VREF is already configured to external, 
					    using internal for another channel is not valid */
					return -EINVAL;
				}
			}
		break;
		default:
			return -EINVAL;
	}

	if (initVREF == true)
	{
		DL_VREF_reset(VREF);
		DL_VREF_enablePower(VREF);
    	delay_cycles(POWER_STARTUP_DELAY);

		DL_VREF_setClockConfig(VREF,
        	(DL_VREF_ClockConfig *) &gVREFClockConfig);
    	DL_VREF_configReference(VREF,
        	(DL_VREF_Config *) &gVREFConfig);
	}
	return error;
}

static int adc_mspm0g3xxx_channel_setup(const struct device *dev,
				    const struct adc_channel_cfg *channel_cfg)
{
	struct adc_mspm0g3xxx_data *data = dev->data;
	const struct adc_mspm0g3xxx_cfg *config = dev->config;
	const uint8_t ch = channel_cfg->channel_id;
	const struct adc_driver_api *api = (struct adc_driver_api *)dev->api;
	int samplingTime;
	int vrefInit = 0;
	
	if (ch > ADC_SYS_NUM_ANALOG_CHAN) {
		LOG_ERR("Channel 0x%X is not supported, max 0x%X", ch, ADC_SYS_NUM_ANALOG_CHAN);
		return -EINVAL;
	}

	/* Initialize MEMCTL configuration to default */
	data->channelMemCtl[ch] = 0;

	samplingTime = adc_mspm0g3xxx_validate_sampling_time(dev, channel_cfg->acquisition_time);
	if (samplingTime < 0)
	{
		return samplingTime;
	}
	
	/* Select one of the sampling timer registers */
	LOG_DBG("Setup %d sampling time %d", ch, samplingTime);
	if (data->sampleTime0 == -1) {
		DL_ADC12_setSampleTime0((ADC12_Regs *)config->base, samplingTime);
		data->sampleTime0 = samplingTime;
		data->channelMemCtl[ch] |= DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0;
	}
	else if (data->sampleTime0 == samplingTime)
	{
		data->channelMemCtl[ch] |= DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0;
	}
	else if (data->sampleTime1 == -1) {
		DL_ADC12_setSampleTime1((ADC12_Regs *)config->base, samplingTime);
		data->sampleTime1 = samplingTime;
		data->channelMemCtl[ch] |= DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP1;
	}
	else if (data->sampleTime0 == samplingTime)
	{
		data->channelMemCtl[ch] |= DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP1;
	}
	else
	{
		LOG_ERR("Two sampling times are supported by this ADC");
		return -EINVAL;
	}

	if (channel_cfg->differential) {
		LOG_ERR("Differential channels are not supported");
		return -EINVAL;
	}

	if (channel_cfg->gain != ADC_GAIN_1) {
		LOG_ERR("Gain is not valid");
		return -EINVAL;
	}

	vrefInit = adc_mspm0g3xxx_config_Vref(channel_cfg->reference,
								   api->ref_internal,
								   &data->channelMemCtl[ch]);
	if (vrefInit < 0)
	{
		LOG_ERR("Error configuring VREF");
		return vrefInit;
	}

	if (channel_cfg->reference == ADC_REF_VDD_1) {
		data->channelMemCtl[ch] |= DL_ADC12_REFERENCE_VOLTAGE_VDDA;
	} else if (channel_cfg->reference == ADC_REF_INTERNAL) {
		if ((api->ref_internal == 2500) || (api->ref_internal == 1400))
		{
			LOG_DBG("Configure Internal VREF");
		}
		data->channelMemCtl[ch] |= DL_ADC12_REFERENCE_VOLTAGE_INTREF;
	} else if (channel_cfg->reference == ADC_REF_EXTERNAL0) {
		LOG_DBG("Configure External VREF");
		data->channelMemCtl[ch] |= DL_ADC12_REFERENCE_VOLTAGE_EXTREF;
	}
	else {
		LOG_ERR("ADC Voltage reference is not supported");
		return -EINVAL;
	}

	LOG_DBG("ADC Channel setup successful!");
	return 0;
}

static int mspm0g3xxx_read(const struct device *dev,
		       const struct adc_sequence *sequence,
		       bool asynchronous,
		       struct k_poll_signal *sig)
{
	struct adc_mspm0g3xxx_data *data = dev->data;
	int rv;
	size_t exp_size;

	if (sequence->resolution != 12) {
		LOG_ERR("Only 12 Resolution is supported, but %d got",
			sequence->resolution);
		return -EINVAL;
	}

	exp_size = sizeof(uint16_t);
	if (sequence->options) {
		exp_size *= (1 + sequence->options->extra_samplings);
	}

	if (sequence->buffer_size < exp_size) {
		LOG_ERR("Required buffer size is %u, but %u got",
			exp_size, sequence->buffer_size);
		return -ENOMEM;
	}

	data->buffer = sequence->buffer;

	adc_context_lock(&data->ctx, asynchronous, sig);
	adc_context_start_read(&data->ctx, sequence);
	rv = adc_context_wait_for_completion(&data->ctx);
	adc_context_release(&data->ctx, rv);
	return rv;
}

static int adc_mspm0g3xxx_read(const struct device *dev,
			   const struct adc_sequence *sequence)
{
	return mspm0g3xxx_read(dev, sequence, false, NULL);
}

#ifdef CONFIG_ADC_ASYNC
static int adc_mspm0g3xxx_read_async(const struct device *dev,
				 const struct adc_sequence *sequence,
				 struct k_poll_signal *async)
{
	return mspm0g3xxx_read(dev, sequence, true, async);
}
#endif

/**
 * AUX_ADC_IRQ handler, called for either of these events:
 * - conversion complete or DMA done (if used);
 * - FIFO underflow or overflow;
 */
static void adc_mspm0g3xxx_isr(const struct device *dev)
{
	struct adc_mspm0g3xxx_data *data = dev->data;
	/* get the statuses of ADC_DONE and ADC_IRQ events in order to clear them both */
 #ifdef UNDO	
 	uint32_t ev_status = (
		HWREG(AUX_EVCTL_BASE + AUX_EVCTL_O_EVTOMCUFLAGS) &
		(AUX_EVCTL_EVTOMCUFLAGS_AUX_ADC_IRQ | AUX_EVCTL_EVTOMCUFLAGS_AUX_ADC_DONE)
	);

	uint32_t fifo_status;
	uint32_t adc_value;

	/* clear the AUXADC-related event flags */
	HWREG(AUX_EVCTL_BASE + AUX_EVCTL_O_EVTOMCUFLAGSCLR) = ev_status;
	/* check the ADC FIFO's status */
	fifo_status = AUXADCGetFifoStatus();
	LOG_DBG("ISR flags 0x%08X fifo 0x%08X", ev_status, fifo_status);
	if ((fifo_status & (AUX_ANAIF_ADCFIFOSTAT_OVERFLOW | AUX_ANAIF_ADCFIFOSTAT_UNDERFLOW))) {
		AUXADCFlushFifo();
	}
	if ((fifo_status & AUX_ANAIF_ADCFIFOSTAT_EMPTY_M)) {
		/* no ADC values available */
		return;
	}
	adc_value = AUXADCPopFifo();
	LOG_DBG("ADC buf %04X val %d", (unsigned int)data->buffer, adc_value);
	*data->buffer = adc_value;
	AUXADCDisable();
#endif
	adc_context_on_sampling_done(&data->ctx, dev);
}

#define ADC_DT_CLOCK_SOURCE(x)	DT_INST_PROP(x, ti_clk_source)

#define ADC_CLOCK_DIV(x)	DT_INST_PROP(x, ti_clk_divider)
#define ADC_DT_CLOCK_DIV(x)	\
	_CONCAT(DL_ADC12_CLOCK_DIVIDE_, ADC_CLOCK_DIV(x))

#define ADC_DT_CLOCK_RANGE(x)	DT_INST_PROP(x, ti_clk_range)

#define MSPM0G3XXX_ADC_INIT(index)						 					\
																			\
PINCTRL_DT_INST_DEFINE(index);												\
																			\
static void adc_mspm0g3xxx_cfg_func_##index(void);			 				\
																			\
static const struct adc_mspm0g3xxx_cfg adc_mspm0g3xxx_cfg_##index = { 		\
	.base = DT_INST_REG_ADDR(index),				 						\
	.irq_cfg_func = adc_mspm0g3xxx_cfg_func_##index, 						\
	.pinctrl = PINCTRL_DT_INST_DEV_CONFIG_GET(index),						\
.ADCClockConfig = {															\
		.clockSel = ADC_DT_CLOCK_SOURCE(index),								\
		.freqRange = ADC_DT_CLOCK_RANGE(index), 							\
		.divideRatio = ADC_DT_CLOCK_DIV(index) 								\
	}		 																\
};									 										\
static const struct adc_driver_api mspm0g3xxx_driver_api##index = {			\
	.channel_setup = adc_mspm0g3xxx_channel_setup,							\
	.read = adc_mspm0g3xxx_read,											\
	.ref_internal = DT_INST_PROP(index, vref_mv),							\
	IF_ENABLED(CONFIG_ADC_ASYNC,											\
		(.read_async = adc_mspm0g3xxx_read_async,))							\
};																			\
static struct adc_mspm0g3xxx_data adc_mspm0g3xxx_data_##index; 				\
DEVICE_DT_INST_DEFINE(index,						 						\
	&adc_mspm0g3xxx_init, NULL,					 							\
	&adc_mspm0g3xxx_data_##index,				 							\
	&adc_mspm0g3xxx_cfg_##index, POST_KERNEL,			 					\
	CONFIG_ADC_INIT_PRIORITY,					 							\
	&mspm0g3xxx_driver_api##index);					 						\
																			\
static void adc_mspm0g3xxx_cfg_func_##index(void)			 				\
{									 										\
	IRQ_CONNECT(DT_INST_IRQN(index), DT_INST_IRQ(index, priority),	 		\
			adc_mspm0g3xxx_isr, DEVICE_DT_INST_GET(index), 0); 				\
	irq_enable(DT_INST_IRQN(index));				 						\
}

DT_INST_FOREACH_STATUS_OKAY(MSPM0G3XXX_ADC_INIT)
