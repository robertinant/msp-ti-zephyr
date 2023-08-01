/* SPDX-License-Identifier: Apache-2.0 */

#define DT_DRV_COMPAT ti_mspm0g3xxx_i2c

/* Zephyr includes */
#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <soc.h>

/* Logging includes */
#define LOG_LEVEL CONFIG_I2C_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(i2c_mspm0g3xxx);
#include "i2c-priv.h"

/* Driverlib includes */
#include <ti/driverlib/dl_i2c.h>
#include <ti/driverlib/dl_gpio.h>

/* Defines for I2C1 */
#define GPIO_I2C1_IOMUX_SDA      (IOMUX_PINCM16)
#define GPIO_I2C1_IOMUX_SDA_FUNC IOMUX_PINCM16_PF_I2C1_SDA
#define GPIO_I2C1_IOMUX_SCL      (IOMUX_PINCM15)
#define GPIO_I2C1_IOMUX_SCL_FUNC IOMUX_PINCM15_PF_I2C1_SCL

enum i2c_mspm0g3xxx_state {
	I2C_mspm0g3xxx_IDLE,
	I2C_mspm0g3xxx_TX_STARTED,
	I2C_mspm0g3xxx_TX_INPROGRESS,
	I2C_mspm0g3xxx_TX_COMPLETE,
	I2C_mspm0g3xxx_RX_STARTED,
	I2C_mspm0g3xxx_RX_INPROGRESS,
	I2C_mspm0g3xxx_RX_COMPLETE,
	I2C_mspm0g3xxx_ERROR
};

struct i2c_mspm0g3xxx_config {
	uint32_t base;
};

struct i2c_mspm0g3xxx_data {
	DL_I2C_ClockConfig gI2CClockConfig;       /* Clock config */
	volatile enum i2c_mspm0g3xxx_state state; /* Current state of I2C transmission */
	struct i2c_msg msg;                       /* Cache msg */
	uint16_t addr;                            /* Cache slave address */
	uint32_t count;                           /* Count for progress in I2C transmission */
};

static const struct i2c_mspm0g3xxx_config i2c_mspm0g3xxx_config = {.base = DT_INST_REG_ADDR(0)};

static struct i2c_mspm0g3xxx_data i2c_mspm0g3xxx_data = {
	.gI2CClockConfig = {.clockSel = DL_I2C_CLOCK_BUSCLK, .divideRatio = DL_I2C_CLOCK_DIVIDE_1}};

static int i2c_mspm0g3xxx_configure(const struct device *dev, uint32_t dev_config)
{
	const struct i2c_mspm0g3xxx_config *config = dev->config;
	uint32_t bitrate;

	/* Target mode not supported */
	if (!(dev_config & I2C_MODE_CONTROLLER)) {
		return -EINVAL;
	}

	/* 10-bit addressing not supported */
	if (dev_config & I2C_MSG_ADDR_10_BITS) {
		return -EINVAL;
	}

	/* Config I2C speed */
	switch (I2C_SPEED_GET(dev_config)) {
	case I2C_SPEED_STANDARD:
		bitrate = 7;
		break;
	case I2C_SPEED_FAST:
		bitrate = 31;
		break;
	default:
		return -EINVAL;
	}

	/* Set the I2C speed */
	DL_I2C_setTimerPeriod((I2C_Regs *)config->base, bitrate);

	return 0;
}

static int i2c_mspm0g3xxx_receive(const struct device *dev, struct i2c_msg msg, uint16_t addr)
{
	const struct i2c_mspm0g3xxx_config *config = dev->config;
	struct i2c_mspm0g3xxx_data *data = dev->data;

	/* Update cached msg and addr */
	data->msg = msg;
	data->addr = addr;

	/* Send a read request to Target */
	data->count = 0;
	data->state = I2C_mspm0g3xxx_RX_STARTED;
	DL_I2C_startControllerTransfer((I2C_Regs *)config->base, data->addr,
				       DL_I2C_CONTROLLER_DIRECTION_RX, data->msg.len);

	/* Wait for all bytes to be received in interrupt */
	while (data->state != I2C_mspm0g3xxx_RX_COMPLETE && (data->state != I2C_mspm0g3xxx_ERROR))
		;

	/* If error, return error */
	if (DL_I2C_getControllerStatus((I2C_Regs *)config->base) & DL_I2C_CONTROLLER_STATUS_ERROR) {
		return -EIO;
	}

	return 0;
}

static int i2c_mspm0g3xxx_transmit(const struct device *dev, struct i2c_msg msg, uint16_t addr)
{
	const struct i2c_mspm0g3xxx_config *config = dev->config;
	struct i2c_mspm0g3xxx_data *data = dev->data;

	/* Sending address without data is not supported */
	if (msg.len == 0) {
		return -EIO;
	}

	/* Update cached msg and addr */
	data->msg = msg;
	data->addr = addr;

	/* Update the state */
	data->state = I2C_mspm0g3xxx_IDLE;

	/*
	 * Fill the FIFO
	 *  The FIFO is 8-bytes deep, and this function will return number
	 *  of bytes written to FIFO */
	data->count =
		DL_I2C_fillControllerTXFIFO((I2C_Regs *)config->base, data->msg.buf, data->msg.len);

	/* Enable TXFIFO trigger interrupt if there are more bytes to send */
	if (data->count < data->msg.len) {
		DL_I2C_enableInterrupt((I2C_Regs *)config->base,
				       DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER);
	} else {
		DL_I2C_disableInterrupt((I2C_Regs *)config->base,
					DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER);
	}

	/*
	 * Send the packet to the controller.
	 * This function will send Start + Stop automatically */
	data->state = I2C_mspm0g3xxx_TX_STARTED;
	while (!(DL_I2C_getControllerStatus((I2C_Regs *)config->base) &
		 DL_I2C_CONTROLLER_STATUS_IDLE))
		;
	DL_I2C_startControllerTransfer((I2C_Regs *)config->base, data->addr,
				       DL_I2C_CONTROLLER_DIRECTION_TX, data->msg.len);

	/* Wait until the Controller sends all bytes */
	while ((data->state != I2C_mspm0g3xxx_TX_COMPLETE) && (data->state != I2C_mspm0g3xxx_ERROR))
		;

	/* If error, return error */
	if (DL_I2C_getControllerStatus((I2C_Regs *)config->base) & DL_I2C_CONTROLLER_STATUS_ERROR) {
		return -EIO;
	}

	return 0;
}

static int i2c_mspm0g3xxx_transfer(const struct device *dev, struct i2c_msg *msgs, uint8_t num_msgs,
				   uint16_t addr)
{
	int ret = 0;

	/* Sending address with no data not supported */
	if (num_msgs == 0) {
		return 0;
	}

	/* Transmit each message */
	for (int i = 0; i < num_msgs; i++) {
		if ((msgs[i].flags & I2C_MSG_RW_MASK) == I2C_MSG_WRITE) {
			ret = i2c_mspm0g3xxx_transmit(dev, msgs[i], addr);
		} else {
			ret = i2c_mspm0g3xxx_receive(dev, msgs[i], addr);
		}
	}

	return ret;
}

static void i2c_mspm0g3xxx_isr(const struct device *dev)
{
	const struct i2c_mspm0g3xxx_config *config = dev->config;
	struct i2c_mspm0g3xxx_data *data = dev->data;

	switch (DL_I2C_getPendingInterrupt((I2C_Regs *)config->base)) {
	case DL_I2C_IIDX_CONTROLLER_RX_DONE:
		data->state = I2C_mspm0g3xxx_RX_COMPLETE;
		break;
	case DL_I2C_IIDX_CONTROLLER_TX_DONE:
		DL_I2C_disableInterrupt((I2C_Regs *)config->base,
					DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER);
		data->state = I2C_mspm0g3xxx_TX_COMPLETE;
		break;
	case DL_I2C_IIDX_CONTROLLER_RXFIFO_TRIGGER:
		if (data->state != I2C_mspm0g3xxx_RX_COMPLETE) { // Fix for RX_DONE happening before
								 // the last RXFIFO_TRIGGER
			data->state = I2C_mspm0g3xxx_RX_INPROGRESS;
		}
		/* Receive all bytes from target */
		while (DL_I2C_isControllerRXFIFOEmpty((I2C_Regs *)config->base) != true) {
			if (data->count < data->msg.len) {
				data->msg.buf[data->count++] =
					DL_I2C_receiveControllerData((I2C_Regs *)config->base);
			} else {
				/* Ignore and remove from FIFO if the buffer is full */
				DL_I2C_receiveControllerData((I2C_Regs *)config->base);
			}
		}
		break;
	case DL_I2C_IIDX_CONTROLLER_TXFIFO_TRIGGER:
		data->state = I2C_mspm0g3xxx_TX_INPROGRESS;
		/* Fill TX FIFO with next bytes to send */
		if (data->count < data->msg.len) {
			data->count += DL_I2C_fillControllerTXFIFO((I2C_Regs *)config->base,
								   &data->msg.buf[data->count],
								   data->msg.len - data->count);
		}
		break;
	case DL_I2C_IIDX_CONTROLLER_ARBITRATION_LOST:
	case DL_I2C_IIDX_CONTROLLER_NACK:
		if ((data->state == I2C_mspm0g3xxx_RX_STARTED) ||
		    (data->state == I2C_mspm0g3xxx_TX_STARTED)) {
			/* NACK interrupt if I2C Target is disconnected */
			data->state = I2C_mspm0g3xxx_ERROR;
		}
	/* Not implemented */
	case DL_I2C_IIDX_CONTROLLER_RXFIFO_FULL:
	case DL_I2C_IIDX_CONTROLLER_TXFIFO_EMPTY:
	case DL_I2C_IIDX_CONTROLLER_START:
	case DL_I2C_IIDX_CONTROLLER_STOP:
	case DL_I2C_IIDX_CONTROLLER_EVENT1_DMA_DONE:
	case DL_I2C_IIDX_CONTROLLER_EVENT2_DMA_DONE:
	default:
		break;
	}
}

static int i2c_mspm0g3xxx_init(const struct device *dev)
{
	const struct i2c_mspm0g3xxx_config *config = dev->config;
	struct i2c_mspm0g3xxx_data *data = dev->data;

	/* Init power */
	DL_I2C_reset((I2C_Regs *)config->base);
	DL_I2C_enablePower((I2C_Regs *)config->base);
	delay_cycles(POWER_STARTUP_DELAY);

	/* Init GPIO */
	DL_GPIO_initPeripheralInputFunctionFeatures(
		GPIO_I2C1_IOMUX_SDA, GPIO_I2C1_IOMUX_SDA_FUNC, DL_GPIO_INVERSION_DISABLE,
		DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
	DL_GPIO_initPeripheralInputFunctionFeatures(
		GPIO_I2C1_IOMUX_SCL, GPIO_I2C1_IOMUX_SCL_FUNC, DL_GPIO_INVERSION_DISABLE,
		DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
	DL_GPIO_enableHiZ(GPIO_I2C1_IOMUX_SDA);
	DL_GPIO_enableHiZ(GPIO_I2C1_IOMUX_SCL);

	/* Config clocks and analog filter */
	DL_I2C_setClockConfig((I2C_Regs *)config->base,
			      (DL_I2C_ClockConfig *)&data->gI2CClockConfig);
	DL_I2C_disableAnalogGlitchFilter((I2C_Regs *)config->base);

	/* Configure Controller Mode */
	DL_I2C_resetControllerTransfer((I2C_Regs *)config->base);

	/* Set frequency */
	uint32_t speed_config =
		i2c_map_dt_bitrate(DT_INST_PROP(0, clock_frequency)) | I2C_MODE_CONTROLLER;
	i2c_mspm0g3xxx_configure(dev, speed_config);

	/* Config other settings */
	DL_I2C_setControllerTXFIFOThreshold((I2C_Regs *)config->base, DL_I2C_TX_FIFO_LEVEL_BYTES_1);
	DL_I2C_setControllerRXFIFOThreshold((I2C_Regs *)config->base, DL_I2C_RX_FIFO_LEVEL_BYTES_1);
	DL_I2C_enableControllerClockStretching((I2C_Regs *)config->base);

	/* Configure Interrupts */
	DL_I2C_enableInterrupt(
		(I2C_Regs *)config->base,
		DL_I2C_INTERRUPT_CONTROLLER_ARBITRATION_LOST | DL_I2C_INTERRUPT_CONTROLLER_NACK |
			DL_I2C_INTERRUPT_CONTROLLER_RXFIFO_TRIGGER |
			DL_I2C_INTERRUPT_CONTROLLER_RX_DONE | DL_I2C_INTERRUPT_CONTROLLER_TX_DONE);

	/* Enable module */
	DL_I2C_enableController((I2C_Regs *)config->base);

	IRQ_CONNECT(DT_INST_IRQN(0), DT_INST_IRQ(0, priority), i2c_mspm0g3xxx_isr,
		    DEVICE_DT_INST_GET(0), 0);
	irq_enable(DT_INST_IRQN(0));

	return 0;
}

static const struct i2c_driver_api i2c_mspm0g3xxx_driver_api = {
	.configure = i2c_mspm0g3xxx_configure,
	.transfer = i2c_mspm0g3xxx_transfer,
};

I2C_DEVICE_DT_INST_DEFINE(0, i2c_mspm0g3xxx_init, NULL, &i2c_mspm0g3xxx_data,
			  &i2c_mspm0g3xxx_config, POST_KERNEL, CONFIG_I2C_INIT_PRIORITY,
			  &i2c_mspm0g3xxx_driver_api);
