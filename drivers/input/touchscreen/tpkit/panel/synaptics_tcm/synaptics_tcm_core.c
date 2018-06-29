/*
 * Synaptics TCM touchscreen driver
 *
 * Copyright (C) 2017-2018 Synaptics Incorporated. All rights reserved.
 *
 * Copyright (C) 2017-2018 Scott Lin <scott.lin@tw.synaptics.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * INFORMATION CONTAINED IN THIS DOCUMENT IS PROVIDED "AS-IS," AND SYNAPTICS
 * EXPRESSLY DISCLAIMS ALL EXPRESS AND IMPLIED WARRANTIES, INCLUDING ANY
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE,
 * AND ANY WARRANTIES OF NON-INFRINGEMENT OF ANY INTELLECTUAL PROPERTY RIGHTS.
 * IN NO EVENT SHALL SYNAPTICS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, PUNITIVE, OR CONSEQUENTIAL DAMAGES ARISING OUT OF OR IN CONNECTION
 * WITH THE USE OF THE INFORMATION CONTAINED IN THIS DOCUMENT, HOWEVER CAUSED
 * AND BASED ON ANY THEORY OF LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, AND EVEN IF SYNAPTICS WAS ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE. IF A TRIBUNAL OF COMPETENT JURISDICTION DOES
 * NOT PERMIT THE DISCLAIMER OF DIRECT DAMAGES OR ANY OTHER DAMAGES, SYNAPTICS'
 * TOTAL CUMULATIVE LIABILITY TO ANY PARTY SHALL NOT EXCEED ONE HUNDRED U.S.
 * DOLLARS.
 */
 
#include <linux/of_gpio.h>
#include <linux/spi/spi.h>
#include <linux/gpio.h>
#include <linux/kthread.h>
#include <linux/interrupt.h>
#include <linux/regulator/consumer.h>
#include <linux/input/mt.h>

#include "synaptics_tcm_core.h"

#include "../../huawei_ts_kit_algo.h"
#include "../../tpkit_platform_adapter.h"
#include "../../huawei_ts_kit_api.h"

#if defined (CONFIG_HUAWEI_DSM)
#include <dsm/dsm_pub.h>
#endif
#if defined (CONFIG_TEE_TUI)
#include "tui.h"
#endif
#include "../../huawei_ts_kit.h"

#define SYNAPTICS_VENDER_NAME  "synaptics_tcm"
#define RESET_ON_RESUME
#define RESET_ON_RESUME_DELAY_MS 20
#define PREDICTIVE_READING
#define MIN_READ_LENGTH 9
#define KEEP_DRIVER_ON_ERROR
#define FORCE_RUN_APPLICATION_FIRMWARE
#define NOTIFIER_PRIORITY 2
#define NOTIFIER_TIMEOUT_MS 500
#define RESPONSE_TIMEOUT_MS 3000
#define APP_STATUS_POLL_TIMEOUT_MS 1000
#define APP_STATUS_POLL_MS 100
#define ENABLE_IRQ_DELAY_MS 20
#define FALL_BACK_ON_POLLING
#define POLLING_DELAY_MS 5
#define RUN_WATCHDOG true
#define WATCHDOG_TRIGGER_COUNT 2
#define WATCHDOG_DELAY_MS 1000
#define MODE_SWITCH_DELAY_MS 100
#define READ_RETRY_US_MIN 5000
#define READ_RETRY_US_MAX 10000
#define WRITE_DELAY_US_MIN 500
#define WRITE_DELAY_US_MAX 1000
#define HOST_DOWNLOAD_WAIT_MS 100
#define HOST_DOWNLOAD_TIMEOUT_MS 1000
#define DYNAMIC_CONFIG_SYSFS_DIR_NAME "dynamic_config"
#define PRODUCT_ID_FW_LEN 5
#define PROJECT_ID_FW_LEN 9

#define HW_BUS_USE_I2C

struct syna_tcm_hcd *tcm_hcd;
struct syna_tcm_board_data *bdata;

static unsigned char *buf;
static unsigned int buf_size;
static unsigned char buffer[FIXED_READ_LENGTH];
static DEFINE_MUTEX(ts_power_gpio_sem);
//static struct mutex wrong_touch_lock;
static int ts_power_gpio_ref = 1;
static u8 pre_finger_status = 0;
DECLARE_COMPLETION(response_complete);
//static struct syna_tcm_touch_settings *syna_tcm_setting_param_regs;

static int syna_tcm_comm_check(void);
static int syna_tcm_get_cal_data(struct ts_calibration_data_info *info,
				 struct ts_cmd_node *out_cmd);
static int syna_tcm_mmi_test(struct ts_rawdata_info *info,
				 struct ts_cmd_node *out_cmd);
static int syna_tcm_chip_get_info(struct ts_chip_info_param *info);
static int syna_tcm_fw_update_boot(char *file_name);
static int syna_tcm_fw_update_sd(void);
static int syna_tcm_before_suspend(void);
static int syna_tcm_suspend(void);
static int syna_tcm_resume(void);
static int syna_tcm_after_resume(void *feature_info);
static int syna_tcm_set_info_flag(struct ts_kit_platform_data *info);
static int syna_tcm_irq_bottom_half(struct ts_cmd_node *in_cmd,
				     struct ts_cmd_node *out_cmd);
static int syna_tcm_irq_top_half(struct ts_cmd_node *cmd);
static int syna_tcm_input_config(struct input_dev * input_dev);
static int syna_tcm_parse_dts(struct device_node *np, struct ts_kit_device_data *chip_data);
//static int syna_tcm_get_brightness_info(void);
static int syna_tcm_init_chip(void);
static int syna_tcm_chip_detect(struct ts_kit_platform_data* data);

struct ts_device_ops ts_kit_syna_tcm_ops = {
	.chip_detect = syna_tcm_chip_detect,
	.chip_init = syna_tcm_init_chip,
//	.chip_get_brightness_info = syna_tcm_get_brightness_info,
	.chip_parse_config = syna_tcm_parse_dts,
	.chip_input_config = syna_tcm_input_config,
	.chip_irq_top_half = syna_tcm_irq_top_half,
	.chip_irq_bottom_half = syna_tcm_irq_bottom_half,
	.chip_fw_update_boot = syna_tcm_fw_update_boot,  // host download on boot
	.chip_fw_update_sd = syna_tcm_fw_update_sd,   // host download by hand
//	.oem_info_switch = synaptics_oem_info_switch,
	.chip_get_info = syna_tcm_chip_get_info,
//	.chip_get_capacitance_test_type =
//	    synaptics_chip_get_capacitance_test_type,
	.chip_set_info_flag = syna_tcm_set_info_flag,
	.chip_before_suspend = syna_tcm_before_suspend,
	.chip_suspend = syna_tcm_suspend,
	.chip_resume = syna_tcm_resume,
	.chip_after_resume = syna_tcm_after_resume,
//	.chip_wakeup_gesture_enable_switch =
//	    synaptics_wakeup_gesture_enable_switch,
	.chip_get_rawdata = syna_tcm_mmi_test,
	.chip_get_calibration_data = syna_tcm_get_cal_data,
//	.chip_get_calibration_info = synaptics_get_calibration_info,
////	.chip_get_debug_data = synaptics_get_debug_data,
//	.chip_glove_switch = synaptics_glove_switch,
//	.chip_shutdown = synaptics_shutdown,
////	.chip_charger_switch = synaptics_charger_switch,
//	.chip_holster_switch = synaptics_holster_switch,
//	.chip_roi_switch = synaptics_roi_switch,
//	.chip_roi_rawdata = synaptics_roi_rawdata,
//	.chip_palm_switch = synaptics_palm_switch,
//	.chip_regs_operate = synaptics_regs_operate,
//	.chip_calibrate = synaptics_calibrate,
//	.chip_calibrate_wakeup_gesture = synaptics_calibrate_wakeup_gesture,
//	.chip_reset = synaptics_reset_device,
//#ifdef HUAWEI_TOUCHSCREEN_TEST
//	.chip_test = test_dbg_cmd_test,
//#endif
//	.chip_wrong_touch = synaptics_wrong_touch,
//	.chip_work_after_input = synaptics_work_after_input_kit,
//	.chip_ghost_detect = synaptics_ghost_detect,
//	.chip_check_status = synaptics_chip_check_status,
//	.chip_touch_switch = synaptics_chip_touch_switch,
};
#ifdef HW_BUS_USE_I2C
#define XFER_ATTEMPTS 10
#define TEMP_I2C_ADDR 0x2c

static int syna_tcm_i2c_alloc_mem(struct syna_tcm_hcd *tcm_hcd,
		unsigned int size)
{
	//static unsigned int buf_size;
	//struct i2c_client *i2c = tcm_hcd->syna_tcm_chip_data->ts_platform_data->client;

	if (size > buf_size) {
		if (buf_size)
			kfree(buf);
		buf = kmalloc(size, GFP_KERNEL);
		if (!buf) {
			TS_LOG_ERR(
					"Failed to allocate memory for buf\n");
			buf_size = 0;
			return -ENOMEM;
		}
		buf_size = size;
	}

	return 0;
}

static int syna_tcm_i2c_rmi_read(struct syna_tcm_hcd *tcm_hcd,
		unsigned short addr, unsigned char *data, unsigned int length)
{
	int retval;
	unsigned char address;
	unsigned int attempt;
	struct i2c_msg msg[2];
	struct i2c_client *i2c = tcm_hcd->syna_tcm_chip_data->ts_platform_data->client;
	//const struct syna_tcm_board_data *bdata = tcm_hcd->hw_if->bdata;



	address = (unsigned char)addr;

	//msg[0].addr = bdata->ubl_i2c_addr;
	msg[0].addr =TEMP_I2C_ADDR;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &address;

	msg[1].addr = TEMP_I2C_ADDR;
	msg[1].flags = I2C_M_RD;
	msg[1].len = length;
	msg[1].buf = data;

	for (attempt = 0; attempt < XFER_ATTEMPTS; attempt++) {
		if (i2c_transfer(i2c->adapter, msg, 2) == 2) {
			retval = length;
			goto exit;
		}
		TS_LOG_ERR(
				"Transfer attempt %d failed\n",
				attempt + 1);

		if (attempt + 1 == XFER_ATTEMPTS) {
			retval = -EIO;
			goto exit;
		}

		msleep(20);
	}

exit:

	return retval;
}

static int syna_tcm_i2c_rmi_write(struct syna_tcm_hcd *tcm_hcd,
		unsigned short addr, unsigned char *data, unsigned int length)
{
	int retval;
	unsigned int attempt;
	unsigned int byte_count;
	struct i2c_msg msg;
	struct i2c_client *i2c = tcm_hcd->syna_tcm_chip_data->ts_platform_data->client;
	//const struct syna_tcm_board_data *bdata = tcm_hcd->hw_if->bdata;



	byte_count = length + 1;

	retval = syna_tcm_i2c_alloc_mem(tcm_hcd, byte_count);
	if (retval < 0) {
		TS_LOG_ERR(
				"Failed to allocate memory\n");
		goto exit;
	}

	buf[0] = (unsigned char)addr;
	retval = secure_memcpy(&buf[1],
			length,
			data,
			length,
			length);
	if (retval < 0) {
		TS_LOG_ERR(
				"Failed to copy write data\n");
		goto exit;
	}

	//msg.addr = bdata->ubl_i2c_addr;
	msg.addr = TEMP_I2C_ADDR;
	msg.flags = 0;
	msg.len = byte_count;
	msg.buf = buf;

	for (attempt = 0; attempt < XFER_ATTEMPTS; attempt++) {
		if (i2c_transfer(i2c->adapter, &msg, 1) == 1) {
			retval = length;
			goto exit;
		}
		TS_LOG_ERR(
				"Transfer attempt %d failed\n",
				attempt + 1);

		if (attempt + 1 == XFER_ATTEMPTS) {
			retval = -EIO;
			goto exit;
		}

		msleep(20);
	}

exit:


	return retval;
}

static int syna_tcm_i2c_read(struct syna_tcm_hcd *tcm_hcd, unsigned char *data,
		unsigned int length)
{
	int retval;
	unsigned int attempt;
	struct i2c_msg msg;
	struct i2c_client *i2c = tcm_hcd->syna_tcm_chip_data->ts_platform_data->client;


	msg.addr = i2c->addr;
	msg.flags = I2C_M_RD;
	msg.len = length;
	msg.buf = data;

	for (attempt = 0; attempt < XFER_ATTEMPTS; attempt++) {
		if (i2c_transfer(i2c->adapter, &msg, 1) == 1) {
			retval = length;
			goto exit;
		}
		TS_LOG_ERR(
				"Transfer attempt %d failed\n",
				attempt + 1);

		if (attempt + 1 == XFER_ATTEMPTS) {
			retval = -EIO;
			goto exit;
		}

		msleep(20);
	}

exit:

	return retval;
}
void syna_log_data(unsigned char *data, int length)
{
	int i;
	printk("syna data begin\n");
	for (i = 0; i < length; i++) {
		printk("[%d]:%x, ", i,data[i]);
		if (i == 34) {
			printk("\n");
		}
	}
	printk("syna data end\n");
}
static int syna_tcm_i2c_write(struct syna_tcm_hcd *tcm_hcd, unsigned char *data,
		unsigned int length)
{
	int retval;
	unsigned int attempt;
	struct i2c_msg msg;
	struct i2c_client *i2c = tcm_hcd->syna_tcm_chip_data->ts_platform_data->client;

	msg.addr = i2c->addr;
	msg.flags = 0;
	msg.len = length;
	msg.buf = data;
/*
	TS_LOG_ERR("write i2c begin:\n");
	for (i = 0; i < length; i++) {
		TS_LOG_ERR("data[%d]:%x, \n", i, data[i]);
	}	
	TS_LOG_ERR("write i2c end\n");
*/
	syna_log_data(data, length);
	for (attempt = 0; attempt < XFER_ATTEMPTS; attempt++) {
		if (i2c_transfer(i2c->adapter, &msg, 1) == 1) {
			retval = length;
			goto exit;
		}
		TS_LOG_ERR(
				"Transfer attempt %d failed\n",
				attempt + 1);

		if (attempt + 1 == XFER_ATTEMPTS) {
			retval = -EIO;
			goto exit;
		}

		msleep(20);
	}

exit:
	return retval;
}
#else
static int syna_tcm_spi_alloc_mem(struct syna_tcm_hcd *tcm_hcd,
		unsigned int size)
{

	if (size > buf_size) {
		if (buf_size)
			kfree(buf);
		buf = kmalloc(size, GFP_KERNEL);
		if (!buf) {
			TS_LOG_ERR(
					"Failed to allocate memory for buf\n");
			buf_size = 0;
			return -ENOMEM;
		}
		buf_size = size;
	}

	return 0;
}

static int syna_tcm_spi_rmi_read_stransfer(struct syna_tcm_hcd *tcm_hcd,
		unsigned short addr, unsigned char *data, unsigned int length)
{
	int retval = NO_ERR;
	struct spi_device *spi = tcm_hcd->syna_tcm_chip_data->ts_platform_data->spi;
	struct spi_transfer xfer[] = {
			  {
					   .tx_buf = &buf[0],
					   .delay_usecs = bdata->ubl_byte_delay_us,
					   .len = 2,
					   .cs_change = 0,
					   .bits_per_word = 8,
			  },
			  {
			  		   .tx_buf = &buf[2],
					   .rx_buf = data,
					   .len = length,
			  },
	};

	memset(&buf[2], 0xff, length);
	buf[0] = (unsigned char)(addr >> 8) | 0x80;
	buf[1] = (unsigned char)addr;
	
	spi->max_speed_hz = 1000000;
	spi_setup(spi);

	retval = spi_sync_transfer(spi, xfer, ARRAY_SIZE(xfer));
	if (retval == 0) {
		retval = length;
	} else {
		TS_LOG_ERR(
				"Failed to complete SPI transfer, error = %d\n",
				retval);
	}

	return retval;
}

static int syna_tcm_spi_rmi_read(struct syna_tcm_hcd *tcm_hcd,
		unsigned short addr, unsigned char *data, unsigned int length)
{
	int retval = NO_ERR;
	
	retval = syna_tcm_spi_alloc_mem(tcm_hcd, length + 2);
	if (retval < 0) {
		TS_LOG_ERR(
				"Failed to allocate memory\n");
		goto exit;
	}

	retval = syna_tcm_spi_rmi_read_stransfer(tcm_hcd, addr, data, length);
	if (retval < 0) {
		TS_LOG_ERR(
				"Failed to spi rmi read transfer\n");
		goto exit;
	}
	
exit:
	return retval;
}
static int syna_tcm_spi_rmi_write_transfer(struct syna_tcm_hcd *tcm_hcd,
		unsigned short addr, unsigned char *data, unsigned int length)
{
	int retval = NO_ERR;
	struct spi_device *spi = tcm_hcd->syna_tcm_chip_data->ts_platform_data->spi;
	struct spi_transfer xfer[] = {
			  {
					   .tx_buf = buf,
					   .len = length,
					   .cs_change = 0,
					   .delay_usecs = bdata->ubl_byte_delay_us,
			  },
	};

	buf[0] = (unsigned char)(addr >> 8) & ~0x80;
	buf[1] = (unsigned char)addr;
	retval = secure_memcpy(&buf[2],
			buf_size - 2,
			data,
			length - 2,
			length - 2);
	if (retval < 0) {
		TS_LOG_ERR(
				"Failed to copy write data\n");
		goto exit;
	}

	spi->max_speed_hz = 1000000;
	retval = spi_setup(spi);
	if (retval) {
		TS_LOG_ERR("%s spi setup failed, retval = %d.\n", __func__, retval);
		goto exit;
	}
	
	retval = spi_sync_transfer(spi, xfer, ARRAY_SIZE(xfer));
	if (retval == 0) {
		retval = length;
	} else {
		TS_LOG_ERR(
				"Failed to complete SPI transfer, error = %d\n",
				retval);
	}

exit:

	return retval;
}

static int syna_tcm_spi_rmi_write(struct syna_tcm_hcd *tcm_hcd,
		unsigned short addr, unsigned char *data, unsigned int length)
{
	int retval = NO_ERR;

	retval = syna_tcm_spi_alloc_mem(tcm_hcd, (length + 2));
	if (retval < 0) {
		TS_LOG_ERR(
				"Failed to allocate memory\n");
		goto exit;
	}
	retval = syna_tcm_spi_rmi_write_transfer(tcm_hcd, addr, data, (length + 2));
	if (retval < 0) {
		TS_LOG_ERR(
				"Failed to spi rmi write transfer\n");
		goto exit;
	}	
exit:
	return retval;
}

static int syna_tcm_spi_read_transfer(struct syna_tcm_hcd *tcm_hcd, unsigned char *data,
		unsigned int length)
{
	int retval = NO_ERR;
	struct spi_device *spi = tcm_hcd->syna_tcm_chip_data->ts_platform_data->spi;
	struct spi_transfer xfer[] = {
			  {
			  		   .tx_buf = buf,
					   .rx_buf = data,
					   .delay_usecs = bdata->byte_delay_us,
					   .cs_change = 0,
					   .len = length,
			  },
	};

	memset(buf, 0xff, length);

	spi->max_speed_hz = 1000000;
	retval = spi_setup(spi);
	if (retval) {
		TS_LOG_ERR("%s spi setup failed, retval = %d.\n", __func__, retval);
		return retval;
	}
	
	retval = spi_sync_transfer(spi, xfer, ARRAY_SIZE(xfer));
	if (retval == 0) {
		retval = length;
	} else {
		TS_LOG_ERR(
				"Failed to complete SPI transfer, error = %d\n",
				retval);
	}

	return retval;
}

static int syna_tcm_spi_read(struct syna_tcm_hcd *tcm_hcd, unsigned char *data,
		unsigned int length)
{
	int retval = NO_ERR;

	retval = syna_tcm_spi_alloc_mem(tcm_hcd, length);
	if (retval < 0) {
		TS_LOG_ERR(
				"Failed to allocate memory\n");
		goto exit;
	}

	retval = syna_tcm_spi_read_transfer(tcm_hcd, data, length);
	if (retval < 0) {
		TS_LOG_ERR(
				"Failed to spi read transfer\n");
		goto exit;
	}

exit:
	return retval;
}


static int syna_tcm_spi_write_transfer(struct syna_tcm_hcd *tcm_hcd, unsigned char *data,
		unsigned int length)
{
	int retval = NO_ERR;
	struct spi_device *spi = tcm_hcd->syna_tcm_chip_data->ts_platform_data->spi;
	struct spi_transfer xfer[] = {
			  {
					   .tx_buf = data,
					   .delay_usecs = bdata->byte_delay_us,
					   .cs_change = 0,
					   .len = length,
			  },
	};

	spi->max_speed_hz = 1000000;
	retval = spi_setup(spi);
	if (retval) {
		TS_LOG_ERR("%s spi setup failed, retval = %d.\n", __func__, retval);
		goto exit;
	}
	
	retval = spi_sync_transfer(spi, xfer, ARRAY_SIZE(xfer));
	if (retval == 0) {
		retval = length;
	} else {
		TS_LOG_ERR(
				"Failed to complete SPI transfer, error = %d\n",
				retval);
	}

exit:
	return retval;
}


static int syna_tcm_spi_write(struct syna_tcm_hcd *tcm_hcd, unsigned char *data,
		unsigned int length)
{
	int retval = NO_ERR;

	retval = syna_tcm_spi_write_transfer(tcm_hcd, data, length);
	if (retval < 0) {
			TS_LOG_ERR(
				"Failed to spi write transfer\n");
	}

	return retval;
}

#endif

/**
 * syna_tcm_dispatch_report() - dispatch report received from device
 *
 * @tcm_hcd: handle of core module
 *
 * The report generated by the device is forwarded to the synchronous inbox of
 * each registered application module for further processing. In addition, the
 * report notifier thread is woken up for asynchronous notification of the
 * report occurrence.
 */
 
static void syna_tcm_dispatch_report(struct syna_tcm_hcd *tcm_hcd)
{
	LOCK_BUFFER(tcm_hcd->in);
	LOCK_BUFFER(tcm_hcd->report.buffer);

	tcm_hcd->report.buffer.buf = &tcm_hcd->in.buf[MESSAGE_HEADER_SIZE];

	tcm_hcd->report.buffer.buf_size = tcm_hcd->in.buf_size;
	tcm_hcd->report.buffer.buf_size -= MESSAGE_HEADER_SIZE;

	tcm_hcd->report.buffer.data_length = tcm_hcd->payload_length;

	tcm_hcd->report.id = tcm_hcd->status_report_code;
	UNLOCK_BUFFER(tcm_hcd->report.buffer);
	UNLOCK_BUFFER(tcm_hcd->in);

	return;
}

/**
 * syna_tcm_dispatch_response() - dispatch response received from device
 *
 * @tcm_hcd: handle of core module
 *
 * The response to a command is forwarded to the sender of the command.
 */
 
static void syna_tcm_dispatch_response(struct syna_tcm_hcd *tcm_hcd)
{
	int retval = NO_ERR;

	if (atomic_read(&tcm_hcd->command_status) != CMD_BUSY)
		return;

	tcm_hcd->response_code = tcm_hcd->status_report_code;

	if (tcm_hcd->payload_length == 0) {
		atomic_set(&tcm_hcd->command_status, CMD_IDLE);
		goto exit;
	}

	LOCK_BUFFER(tcm_hcd->resp);

	retval = syna_tcm_alloc_mem(tcm_hcd,
			&tcm_hcd->resp,
			tcm_hcd->payload_length);
	if (retval < 0) {
		TS_LOG_ERR(
				"Failed to allocate memory for tcm_hcd->resp.buf\n");
		UNLOCK_BUFFER(tcm_hcd->resp);
		atomic_set(&tcm_hcd->command_status, CMD_ERROR);
		goto exit;
	}

	LOCK_BUFFER(tcm_hcd->in);

	retval = secure_memcpy(tcm_hcd->resp.buf,
			tcm_hcd->resp.buf_size,
			&tcm_hcd->in.buf[MESSAGE_HEADER_SIZE],
			tcm_hcd->in.buf_size - MESSAGE_HEADER_SIZE,
			tcm_hcd->payload_length);
	if (retval < 0) {
		TS_LOG_ERR(
				"Failed to copy payload\n");
		UNLOCK_BUFFER(tcm_hcd->in);
		UNLOCK_BUFFER(tcm_hcd->resp);
		atomic_set(&tcm_hcd->command_status, CMD_ERROR);
		goto exit;
	}

	tcm_hcd->resp.data_length = tcm_hcd->payload_length;

	UNLOCK_BUFFER(tcm_hcd->in);
	UNLOCK_BUFFER(tcm_hcd->resp);

	atomic_set(&tcm_hcd->command_status, CMD_IDLE);

exit:
	complete(&response_complete);

	return;
}

/**
 * syna_tcm_dispatch_message() - dispatch message received from device
 *
 * @tcm_hcd: handle of core module
 *
 * The information received in the message read in from the device is dispatched
 * to the appropriate destination based on whether the information represents a
 * report or a response to a command.
 */
 
static void syna_tcm_dispatch_message(struct syna_tcm_hcd *tcm_hcd)
{
	int retval = NO_ERR;
	unsigned char *build_id = NULL;
	unsigned int payload_length = 0;
	unsigned int max_write_size = 0;

	if (tcm_hcd->status_report_code == REPORT_IDENTIFY) {
		payload_length = tcm_hcd->payload_length;

		LOCK_BUFFER(tcm_hcd->in);

		retval = secure_memcpy((unsigned char *)&tcm_hcd->id_info,
				sizeof(tcm_hcd->id_info),
				&tcm_hcd->in.buf[MESSAGE_HEADER_SIZE],
				tcm_hcd->in.buf_size - MESSAGE_HEADER_SIZE,
				MIN(sizeof(tcm_hcd->id_info), payload_length));
		if (retval < 0) {
			TS_LOG_ERR(
					"Failed to copy identification info\n");
			UNLOCK_BUFFER(tcm_hcd->in);
			return;
		}

		UNLOCK_BUFFER(tcm_hcd->in);

		build_id = tcm_hcd->id_info.build_id;
		tcm_hcd->packrat_number = le4_to_uint(build_id);

		max_write_size = le2_to_uint(tcm_hcd->id_info.max_write_size);
		tcm_hcd->wr_chunk_size = MIN(max_write_size, WR_CHUNK_SIZE);
		if (tcm_hcd->wr_chunk_size == 0)
			tcm_hcd->wr_chunk_size = max_write_size;

		TS_LOG_INFO(
				"Received identify report (firmware mode = 0x%02x)\n",
				tcm_hcd->id_info.mode);

		if (atomic_read(&tcm_hcd->command_status) == CMD_BUSY) {
			switch (tcm_hcd->command) {
			case CMD_RESET:
			case CMD_RUN_BOOTLOADER_FIRMWARE:
			case CMD_RUN_APPLICATION_FIRMWARE:
				tcm_hcd->response_code = STATUS_OK;
				atomic_set(&tcm_hcd->command_status, CMD_IDLE);
				complete(&response_complete);
				break;
			default:
				TS_LOG_INFO(
						"Device has been reset\n");
				atomic_set(&tcm_hcd->command_status, CMD_ERROR);
				complete(&response_complete);
				break;
			}
		}

		if (tcm_hcd->id_info.mode == MODE_HOST_DOWNLOAD) {
			tcm_hcd->host_download_mode = true;
			return;
		}

#ifdef FORCE_RUN_APPLICATION_FIRMWARE
		if (tcm_hcd->id_info.mode != MODE_APPLICATION) {
			if (atomic_read(&tcm_hcd->helper.task) == HELP_NONE) {
				atomic_set(&tcm_hcd->helper.task,
						HELP_RUN_APPLICATION_FIRMWARE);
				queue_work(tcm_hcd->helper.workqueue,
						&tcm_hcd->helper.work);
				return;
			}
		}
#endif
	}

	if (tcm_hcd->status_report_code >= REPORT_IDENTIFY)
		syna_tcm_dispatch_report(tcm_hcd);
	else
		syna_tcm_dispatch_response(tcm_hcd);

	return;
}

/**
 * syna_tcm_continued_read() - retrieve entire payload from device
 *
 * @tcm_hcd: handle of core module
 *
 * Read transactions are carried out until the entire payload is retrieved from
 * the device and stored in the handle of the core module.
 */
 
static int syna_tcm_continued_read(struct syna_tcm_hcd *tcm_hcd)
{
	int retval = NO_ERR;
	unsigned char marker = 0;
	unsigned char code = 0;
	unsigned int idx = 0;
	unsigned int offset = 0;
	unsigned int chunks = 0;
	unsigned int chunk_space = 0;
	unsigned int xfer_length = 0;
	unsigned int total_length = 0;
	unsigned int remaining_length = 0;

	total_length = MESSAGE_HEADER_SIZE + tcm_hcd->payload_length + 1;

	remaining_length = total_length - tcm_hcd->read_length;

	LOCK_BUFFER(tcm_hcd->in);

	retval = syna_tcm_realloc_mem(tcm_hcd,
			&tcm_hcd->in,
			total_length + 1);
	if (retval < 0) {
		TS_LOG_ERR(
				"Failed to reallocate memory for tcm_hcd->in.buf\n");
		UNLOCK_BUFFER(tcm_hcd->in);
		return retval;
	}

	// available chunk space for payload = total chunk size minus header
	// marker byte and header code byte 
	if (tcm_hcd->rd_chunk_size == 0)
		chunk_space = remaining_length;
	else
		chunk_space = tcm_hcd->rd_chunk_size - 2;

	chunks = ceil_div(remaining_length, chunk_space);

	chunks = chunks == 0 ? 1 : chunks;

	offset = tcm_hcd->read_length;

	LOCK_BUFFER(tcm_hcd->temp);

	for (idx = 0; idx < chunks; idx++) {
		if (remaining_length > chunk_space)
			xfer_length = chunk_space;
		else
			xfer_length = remaining_length;

		if (xfer_length == 1) {
			tcm_hcd->in.buf[offset] = MESSAGE_PADDING;
			offset += xfer_length;
			remaining_length -= xfer_length;
			continue;
		}

		retval = syna_tcm_alloc_mem(tcm_hcd,
				&tcm_hcd->temp,
				xfer_length + 2);
		if (retval < 0) {
			TS_LOG_ERR(
					"Failed to allocate memory for tcm_hcd->temp.buf\n");
			UNLOCK_BUFFER(tcm_hcd->temp);
			UNLOCK_BUFFER(tcm_hcd->in);
			return retval;
		}

		retval = syna_tcm_read(tcm_hcd,
				tcm_hcd->temp.buf,
				xfer_length + 2);
		if (retval < 0) {
			TS_LOG_ERR(
					"Failed to read from device\n");
			UNLOCK_BUFFER(tcm_hcd->temp);
			UNLOCK_BUFFER(tcm_hcd->in);
			return retval;
		}

		marker = tcm_hcd->temp.buf[0];
		code = tcm_hcd->temp.buf[1];

		if (marker != MESSAGE_MARKER) {
			TS_LOG_ERR(
					"Incorrect header marker (0x%02x)\n",
					marker);
			UNLOCK_BUFFER(tcm_hcd->temp);
			UNLOCK_BUFFER(tcm_hcd->in);
			return -EIO;
		}

		if (code != STATUS_CONTINUED_READ) {
			TS_LOG_ERR(
					"Incorrect header code (0x%02x)\n",
					code);
			UNLOCK_BUFFER(tcm_hcd->temp);
			UNLOCK_BUFFER(tcm_hcd->in);
			return -EIO;
		}

		retval = secure_memcpy(&tcm_hcd->in.buf[offset],
				tcm_hcd->in.buf_size - offset,
				&tcm_hcd->temp.buf[2],
				tcm_hcd->temp.buf_size - 2,
				xfer_length);
		if (retval < 0) {
			TS_LOG_ERR(
					"Failed to copy payload\n");
			UNLOCK_BUFFER(tcm_hcd->temp);
			UNLOCK_BUFFER(tcm_hcd->in);
			return retval;
		}

		offset += xfer_length;

		remaining_length -= xfer_length;
	}

	UNLOCK_BUFFER(tcm_hcd->temp);
	UNLOCK_BUFFER(tcm_hcd->in);

	return 0;
}

/**
 * syna_tcm_raw_read() - retrieve specific number of data bytes from device
 *
 * @tcm_hcd: handle of core module
 * @in_buf: buffer for storing data retrieved from device
 * @length: number of bytes to retrieve from device
 *
 * Read transactions are carried out until the specific number of data bytes are
 * retrieved from the device and stored in in_buf.
 */
 
int syna_tcm_raw_read(struct syna_tcm_hcd *tcm_hcd,
		unsigned char *in_buf, unsigned int length)
{
	int retval = NO_ERR;
	unsigned char code = 0;
	unsigned int idx = 0;
	unsigned int offset = 0;
	unsigned int chunks = 0;
	unsigned int chunk_space = 0;
	int xfer_length = 0;
	int remaining_length = 0;

	if (length < 2) {
		retval = syna_tcm_i2c_read(tcm_hcd,
				in_buf,
				length);
		if (retval < 0) {
			TS_LOG_ERR(
					"Failed to read from device\n");
			return retval;
		}
		TS_LOG_ERR(
				"small length information\n");
		return length;
	}

	// minus header marker byte and header code byte
	remaining_length = length - 2;

	// available chunk space for data = total chunk size minus header marker
	// byte and header code byte
	if (tcm_hcd->rd_chunk_size == 0)
		chunk_space = remaining_length;
	else
		chunk_space = tcm_hcd->rd_chunk_size - 2;

	chunks = ceil_div(remaining_length, chunk_space);

	chunks = chunks == 0 ? 1 : chunks;

	offset = 0;

	LOCK_BUFFER(tcm_hcd->temp);

	for (idx = 0; idx < chunks; idx++) {
		if (remaining_length <=0) {
			break;
		}
		if (remaining_length > chunk_space)
			xfer_length = chunk_space;
		else
			xfer_length = remaining_length;

		if (xfer_length <= 0) {
			break;
		}
		if (xfer_length == 1) {
			in_buf[offset] = MESSAGE_PADDING;
			offset += xfer_length;
			remaining_length -= xfer_length;
			continue;
		}

		retval = syna_tcm_alloc_mem(tcm_hcd,
				&tcm_hcd->temp,
				xfer_length + 2);
		if (retval < 0) {
			TS_LOG_ERR(
					"Failed to allocate memory for tcm_hcd->temp.buf\n");
			UNLOCK_BUFFER(tcm_hcd->temp);
			return retval;
		}

		retval = syna_tcm_i2c_read(tcm_hcd,
				tcm_hcd->temp.buf,
				xfer_length + 2);
		if (retval < 0) {
			TS_LOG_ERR(
					"Failed to read from device\n");
			UNLOCK_BUFFER(tcm_hcd->temp);
			return retval;
		}

		code = tcm_hcd->temp.buf[1];
		TS_LOG_ERR("temp buf:%x, %x, %x, %x, %x, %x\n", tcm_hcd->temp.buf[0], tcm_hcd->temp.buf[1],tcm_hcd->temp.buf[2], tcm_hcd->temp.buf[3],tcm_hcd->temp.buf[4], tcm_hcd->temp.buf[5]);
		if (idx == 0) {
			retval = secure_memcpy(&in_buf[0],
					length,
					&tcm_hcd->temp.buf[0],
					tcm_hcd->temp.buf_size,
					xfer_length + 2);
			//remaiming length cust
			TS_LOG_ERR("remaining length is  %d\n", remaining_length);
			if ((length > ((tcm_hcd->temp.buf[3] << 8 | tcm_hcd->temp.buf[2]) + 5)) && (code != STATUS_CONTINUED_READ))
				remaining_length = (tcm_hcd->temp.buf[3] << 8 | tcm_hcd->temp.buf[2]) + 5 - 2;
			TS_LOG_ERR("remaining length cust to %d\n", remaining_length);
		} else {
			if (code != STATUS_CONTINUED_READ) {
				TS_LOG_ERR(
						"Incorrect header code (0x%02x)\n",
						code);
				UNLOCK_BUFFER(tcm_hcd->temp);
				return -EIO;
			}

			retval = secure_memcpy(&in_buf[offset],
					length - offset,
					&tcm_hcd->temp.buf[2],
					tcm_hcd->temp.buf_size - 2,
					xfer_length);
		}
		if (retval < 0) {
			TS_LOG_ERR(
					"Failed to copy data\n");
			UNLOCK_BUFFER(tcm_hcd->temp);
			return retval;
		}

		if (idx == 0)
			offset += (xfer_length + 2);
		else
			offset += xfer_length;

		remaining_length -= xfer_length;
	}

	UNLOCK_BUFFER(tcm_hcd->temp);

	return 0;
}

/**
 * syna_tcm_raw_write() - write command/data to device without receiving
 * response
 *
 * @tcm_hcd: handle of core module
 * @command: command to send to device
 * @data: data to send to device
 * @length: length of data in bytes
 *
 * A command and its data, if any, are sent to the device.
 */
 
int syna_tcm_raw_write(struct syna_tcm_hcd *tcm_hcd,
		unsigned char command, unsigned char *data, unsigned int length)
{
	int retval = NO_ERR;
	unsigned int idx  = 0;
	unsigned int chunks = 0;
	unsigned int chunk_space = 0;
	unsigned int xfer_length = 0;
	unsigned int remaining_length = 0;

	remaining_length = length;

	//available chunk space for data = total chunk size minus command
	 //byte
	if (tcm_hcd->wr_chunk_size == 0)
		chunk_space = remaining_length;
	else
		chunk_space = tcm_hcd->wr_chunk_size - 1;

	chunks = ceil_div(remaining_length, chunk_space);

	chunks = chunks == 0 ? 1 : chunks;

	LOCK_BUFFER(tcm_hcd->out);

	for (idx = 0; idx < chunks; idx++) {
		if (remaining_length > chunk_space)
			xfer_length = chunk_space;
		else
			xfer_length = remaining_length;

		retval = syna_tcm_alloc_mem(tcm_hcd,
				&tcm_hcd->out,
				xfer_length + 1);
		if (retval < 0) {
			TS_LOG_ERR(
					"Failed to allocate memory for tcm_hcd->out.buf\n");
			UNLOCK_BUFFER(tcm_hcd->out);
			return retval;
		}

		if (idx == 0)
			tcm_hcd->out.buf[0] = command;
		else
			tcm_hcd->out.buf[0] = CMD_CONTINUE_WRITE;

		if (xfer_length) {
			retval = secure_memcpy(&tcm_hcd->out.buf[1],
					tcm_hcd->out.buf_size - 1,
					&data[idx * chunk_space],
					remaining_length,
					xfer_length);
			if (retval < 0) {
				TS_LOG_ERR(
						"Failed to copy data\n");
				UNLOCK_BUFFER(tcm_hcd->out);
				return retval;
			}
		}

		retval = syna_tcm_write(tcm_hcd,
				tcm_hcd->out.buf,
				xfer_length + 1);
		if (retval < 0) {
			TS_LOG_ERR(
					"Failed to write to device\n");
			UNLOCK_BUFFER(tcm_hcd->out);
			return retval;
		}

		remaining_length -= xfer_length;
	}

	UNLOCK_BUFFER(tcm_hcd->out);

	return 0;
}

/**
 * syna_tcm_read_message() - read message from device
 *
 * @tcm_hcd: handle of core module
 * @in_buf: buffer for storing data in raw read mode
 * @length: length of data in bytes in raw read mode
 *
 * If in_buf is not NULL, raw read mode is used and syna_tcm_raw_read() is
 * called. Otherwise, a message including its entire payload is retrieved from
 * the device and dispatched to the appropriate destination.
 */
 
static int syna_tcm_read_message(struct syna_tcm_hcd *tcm_hcd,
		unsigned char *in_buf, unsigned int length)
{
	int retval = NO_ERR;
	bool retry = 0;
	unsigned int total_length = 0;
	struct syna_tcm_message_header *header = NULL;

	if (in_buf != NULL) {
		retval = syna_tcm_raw_read(tcm_hcd, in_buf, length);
		goto exit;
	}

	retry = true;

retry:
	LOCK_BUFFER(tcm_hcd->in);

	retval = syna_tcm_read(tcm_hcd,
			tcm_hcd->in.buf,
			tcm_hcd->read_length);
	if (retval < 0) {
		TS_LOG_ERR(
				"Failed to read from device\n");
		UNLOCK_BUFFER(tcm_hcd->in);
		if (retry) {
			usleep_range(READ_RETRY_US_MIN, READ_RETRY_US_MAX);
			retry = false;
			goto retry;
		}
		goto exit;
	}

	header = (struct syna_tcm_message_header *)tcm_hcd->in.buf;

	if (header->marker != MESSAGE_MARKER) {
		TS_LOG_ERR(
				"Incorrect header marker (0x%02x)\n",
				header->marker);
		UNLOCK_BUFFER(tcm_hcd->in);
		retval = -ENXIO;
		if (retry) {
			usleep_range(READ_RETRY_US_MIN, READ_RETRY_US_MAX);
			retry = false;
			goto retry;
		}
		goto exit;
	}

	tcm_hcd->status_report_code = header->code;

	tcm_hcd->payload_length = le2_to_uint(header->length);

	if (tcm_hcd->status_report_code <= STATUS_ERROR ||
			tcm_hcd->status_report_code == STATUS_INVALID) {
		switch (tcm_hcd->status_report_code) {
		case STATUS_OK:
			break;
		case STATUS_CONTINUED_READ:
			TS_LOG_DEBUG(
					"Out-of-sync continued read\n");
		case STATUS_IDLE:
		case STATUS_BUSY:

			tcm_hcd->payload_length = 0;
			UNLOCK_BUFFER(tcm_hcd->in);
			retval = 0;
			goto exit;
		default:
			TS_LOG_ERR(
					"Incorrect header code (0x%02x)\n",
					tcm_hcd->status_report_code);
			if (tcm_hcd->status_report_code == STATUS_INVALID)
				tcm_hcd->payload_length = 0;
		}
	}

	total_length = MESSAGE_HEADER_SIZE + tcm_hcd->payload_length + 1;

#ifdef PREDICTIVE_READING
	if (total_length <= tcm_hcd->read_length) {
		goto check_padding;
	} else if (total_length - 1 == tcm_hcd->read_length) {
		tcm_hcd->in.buf[total_length - 1] = MESSAGE_PADDING;
		goto check_padding;
	}
#else
	if (tcm_hcd->payload_length == 0) {
		tcm_hcd->in.buf[total_length - 1] = MESSAGE_PADDING;
		goto check_padding;
	}
#endif

	UNLOCK_BUFFER(tcm_hcd->in);

	retval = syna_tcm_continued_read(tcm_hcd);
	if (retval < 0) {
		TS_LOG_ERR(
				"Failed to do continued read\n");
		goto exit;
	};

	LOCK_BUFFER(tcm_hcd->in);

	tcm_hcd->in.buf[0] = MESSAGE_MARKER;
	tcm_hcd->in.buf[1] = tcm_hcd->status_report_code;
	tcm_hcd->in.buf[2] = (unsigned char)tcm_hcd->payload_length;
	tcm_hcd->in.buf[3] = (unsigned char)(tcm_hcd->payload_length >> 8);

check_padding:
	if (tcm_hcd->in.buf[total_length - 1] != MESSAGE_PADDING) {
		TS_LOG_ERR(
				"Incorrect message padding byte (0x%02x)\n",
				tcm_hcd->in.buf[total_length - 1]);
		UNLOCK_BUFFER(tcm_hcd->in);
		retval = -EIO;
		goto exit;
	}

	UNLOCK_BUFFER(tcm_hcd->in);

#ifdef PREDICTIVE_READING
	total_length = MAX(total_length, MIN_READ_LENGTH);
	tcm_hcd->read_length = MIN(total_length, tcm_hcd->rd_chunk_size);
	if (tcm_hcd->rd_chunk_size == 0)
		tcm_hcd->read_length = total_length;
#endif

	syna_tcm_dispatch_message(tcm_hcd);

	retval = 0;

exit:
	if (retval < 0) {
		if (atomic_read(&tcm_hcd->command_status) == CMD_BUSY) {
			atomic_set(&tcm_hcd->command_status, CMD_ERROR);
			complete(&response_complete);
		}
	}

	return retval;
}

/**
 * syna_tcm_write_message() - write message to device and receive response
 *
 * @tcm_hcd: handle of core module
 * @command: command to send to device
 * @payload: payload of command
 * @length: length of payload in bytes
 * @resp_buf: buffer for storing command response
 * @resp_buf_size: size of response buffer in bytes
 * @resp_length: length of command response in bytes
 * @response_code: status code returned in command response
 * @polling_delay_ms: delay time after sending command before resuming polling
 *
 * If resp_buf is NULL, raw write mode is used and syna_tcm_raw_write() is
 * called. Otherwise, a command and its payload, if any, are sent to the device
 * and the response to the command generated by the device is read in.
 */
 
static int syna_tcm_write_message(struct syna_tcm_hcd *tcm_hcd,
		unsigned char command, unsigned char *payload,
		unsigned int length, unsigned char **resp_buf,
		unsigned int *resp_buf_size, unsigned int *resp_length,
		unsigned char *response_code, unsigned int polling_delay_ms)
{
	int retval = NO_ERR;
	unsigned int idx = 0;
	unsigned int chunks = 0;
	unsigned int chunk_space = 0;
	unsigned int xfer_length = 0;
	unsigned int remaining_length = 0;
	unsigned int command_status = 0;

	if (response_code != NULL)
		*response_code = STATUS_INVALID;

	if (!tcm_hcd->do_polling && current->pid == tcm_hcd->isr_pid) {
		TS_LOG_ERR(
				"Invalid execution context\n");
		return -EINVAL;
	}

//	mutex_lock(&tcm_hcd->command_mutex);

//	mutex_lock(&tcm_hcd->rw_ctrl_mutex);

	if (resp_buf == NULL) {
		retval = syna_tcm_raw_write(tcm_hcd, command, payload, length);
	//	mutex_unlock(&tcm_hcd->rw_ctrl_mutex);
		goto exit;
	}

	if (tcm_hcd->do_polling && polling_delay_ms) {
		cancel_delayed_work_sync(&tcm_hcd->polling_work);
		flush_workqueue(tcm_hcd->polling_workqueue);
	}

	atomic_set(&tcm_hcd->command_status, CMD_BUSY);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 13, 0))
	reinit_completion(&response_complete);
#else
	INIT_COMPLETION(response_complete);
#endif

	tcm_hcd->command = command;

	LOCK_BUFFER(tcm_hcd->resp);

	tcm_hcd->resp.buf = *resp_buf;
	tcm_hcd->resp.buf_size = *resp_buf_size;
	tcm_hcd->resp.data_length = 0;

	UNLOCK_BUFFER(tcm_hcd->resp);

	// adding two length bytes as part of payload
	remaining_length = length + 2;

	 //available chunk space for payload = total chunk size minus command
	//byte
	if (tcm_hcd->wr_chunk_size == 0)
		chunk_space = remaining_length;
	else
		chunk_space = tcm_hcd->wr_chunk_size - 1;

	chunks = ceil_div(remaining_length, chunk_space);

	chunks = chunks == 0 ? 1 : chunks;

	TS_LOG_DEBUG(
			"Command = 0x%02x\n",
			command);

	LOCK_BUFFER(tcm_hcd->out);

	for (idx = 0; idx < chunks; idx++) {
		if (remaining_length > chunk_space)
			xfer_length = chunk_space;
		else
			xfer_length = remaining_length;

		retval = syna_tcm_alloc_mem(tcm_hcd,
				&tcm_hcd->out,
				xfer_length + 1);
		if (retval < 0) {
			TS_LOG_ERR(
					"Failed to allocate memory for tcm_hcd->out.buf\n");
			UNLOCK_BUFFER(tcm_hcd->out);
			goto exit;
		}

		if (idx == 0) {
			tcm_hcd->out.buf[0] = command;
			tcm_hcd->out.buf[1] = (unsigned char)length;
			tcm_hcd->out.buf[2] = (unsigned char)(length >> 8);

			if (xfer_length > 2) {
				retval = secure_memcpy(&tcm_hcd->out.buf[3],
						tcm_hcd->out.buf_size - 3,
						payload,
						remaining_length - 2,
						xfer_length - 2);
				if (retval < 0) {
					TS_LOG_ERR(
							"Failed to copy payload\n");
					UNLOCK_BUFFER(tcm_hcd->out);
					goto exit;
				}
			}
		} else {
			tcm_hcd->out.buf[0] = CMD_CONTINUE_WRITE;

			retval = secure_memcpy(&tcm_hcd->out.buf[1],
					tcm_hcd->out.buf_size - 1,
					&payload[idx * chunk_space - 2],
					remaining_length,
					xfer_length);
			if (retval < 0) {
				TS_LOG_ERR(
						"Failed to copy payload\n");
				UNLOCK_BUFFER(tcm_hcd->out);
				goto exit;
			}
		}

		retval = syna_tcm_write(tcm_hcd,
				tcm_hcd->out.buf,
				xfer_length + 1);
		if (retval < 0) {
			TS_LOG_ERR(
					"Failed to write to device\n");
			UNLOCK_BUFFER(tcm_hcd->out);
			goto exit;
		}

		remaining_length -= xfer_length;

		if (chunks > 1)
			usleep_range(WRITE_DELAY_US_MIN, WRITE_DELAY_US_MAX);
	}

	UNLOCK_BUFFER(tcm_hcd->out);

	if (tcm_hcd->do_polling && polling_delay_ms) {
		queue_delayed_work(tcm_hcd->polling_workqueue,
				&tcm_hcd->polling_work,
				msecs_to_jiffies(polling_delay_ms));
	}

	retval = wait_for_completion_timeout(&response_complete,
			msecs_to_jiffies(RESPONSE_TIMEOUT_MS));
	if (retval == 0) {
		TS_LOG_ERR(
				"Timed out waiting for response (command 0x%02x)\n",
				tcm_hcd->command);
		retval = -EIO;
		goto exit;
	}

	command_status = atomic_read(&tcm_hcd->command_status);
	if (command_status != CMD_IDLE) {
		TS_LOG_ERR(
				"Failed to get valid response (command 0x%02x)\n",
				tcm_hcd->command);
		retval = -EIO;
		goto exit;
	}

	LOCK_BUFFER(tcm_hcd->resp);

	if (tcm_hcd->response_code != STATUS_OK) {
		if (tcm_hcd->resp.data_length) {
			TS_LOG_ERR(
					"Error code = 0x%02x (command 0x%02x)\n",
					tcm_hcd->resp.buf[0], tcm_hcd->command);
		}
		retval = -EIO;
	} else {
		retval = 0;
	}

	*resp_buf = tcm_hcd->resp.buf;
	*resp_buf_size = tcm_hcd->resp.buf_size;
	*resp_length = tcm_hcd->resp.data_length;

	if (response_code != NULL)
		*response_code = tcm_hcd->response_code;

	UNLOCK_BUFFER(tcm_hcd->resp);

exit:
	tcm_hcd->command = CMD_NONE;

	atomic_set(&tcm_hcd->command_status, CMD_IDLE);

	return retval;
}


static int syna_tcm_wait_hdl(struct syna_tcm_hcd *tcm_hcd)
{
	int retval = NO_ERR;

	msleep(HOST_DOWNLOAD_WAIT_MS);

	if (!atomic_read(&tcm_hcd->host_downloading))
		return 0;

	retval = wait_event_interruptible_timeout(tcm_hcd->hdl_wq,
			!atomic_read(&tcm_hcd->host_downloading),
			msecs_to_jiffies(HOST_DOWNLOAD_TIMEOUT_MS));
	if (retval == 0) {
		TS_LOG_ERR(
				"Timed out waiting for completion of host download\n");
		retval = -EIO;
	} else {
		retval = 0;
	}

	return retval;
}

static void syna_tcm_update_watchdog(struct syna_tcm_hcd *tcm_hcd, bool en)
{
	return;
	cancel_delayed_work_sync(&tcm_hcd->watchdog.work);
	flush_workqueue(tcm_hcd->watchdog.workqueue);

	if (!tcm_hcd->watchdog.run) {
		tcm_hcd->watchdog.count = 0;
		return;
	}

	if (en) {
		queue_delayed_work(tcm_hcd->watchdog.workqueue,
				&tcm_hcd->watchdog.work,
				msecs_to_jiffies(WATCHDOG_DELAY_MS));
	} else {
		tcm_hcd->watchdog.count = 0;
	}

	return;
}

static void ts_kit_power_gpio_disable(void)
{
	if (tcm_hcd == NULL) {
		TS_LOG_ERR("tcm_hcd == NULL\n");
		return;
	}

	if (tcm_hcd->syna_tcm_chip_data == NULL) {
		TS_LOG_ERR("syna_tcm_chip_data == NULL\n");
		return;
	}

	mutex_lock(&ts_power_gpio_sem);
	if (ts_power_gpio_ref == 1) {
		gpio_direction_output(tcm_hcd->syna_tcm_chip_data->vddio_gpio_ctrl, 0);
	}
	if (ts_power_gpio_ref > 0) {
		ts_power_gpio_ref--;
	}
	TS_LOG_INFO("ts_power_gpio_ref-- = %d\n", ts_power_gpio_ref);
	mutex_unlock(&ts_power_gpio_sem);
}


static int syna_tcm_pinctrl_select_lowpower(void)
{
	int retval = NO_ERR;
	if (tcm_hcd->syna_tcm_chip_data->ts_platform_data->fpga_flag == 1)
		return 0;

	retval = pinctrl_select_state(tcm_hcd->pctrl, tcm_hcd->pins_idle);
	if (retval < 0) {
		TS_LOG_ERR("set iomux lowpower error, %d\n", retval);
	}
	return retval;
}

static int syna_tcm_pinctrl_select_normal(void)
{
	int retval = NO_ERR;

	if (tcm_hcd->syna_tcm_chip_data->ts_platform_data->fpga_flag == 1)
		return 0;

	retval =
	    pinctrl_select_state(tcm_hcd->pctrl, tcm_hcd->pins_default);
	if (retval < 0) {
		TS_LOG_ERR("set iomux normal error, %d\n", retval);
	}
	return retval;
}

static void syna_tcm_power_on_gpio_set(void)
{
	syna_tcm_pinctrl_select_normal();
	gpio_direction_input(tcm_hcd->syna_tcm_chip_data->ts_platform_data->irq_gpio);
	gpio_direction_output(tcm_hcd->syna_tcm_chip_data->ts_platform_data->reset_gpio, 1);
}

static void ts_kit_power_gpio_enable(void)
{
	if (tcm_hcd == NULL) {
		TS_LOG_ERR("tcm_hcd == NULL\n");
		return;
	}

	if (tcm_hcd->syna_tcm_chip_data == NULL) {
		TS_LOG_ERR("syna_tcm_chip_data == NULL\n");
		return;
	}

	mutex_lock(&ts_power_gpio_sem);
	if (ts_power_gpio_ref == 0) {
		gpio_direction_output(tcm_hcd->syna_tcm_chip_data->vddio_gpio_ctrl, 1);
	}
	ts_power_gpio_ref++;
	TS_LOG_INFO("ts_power_gpio_ref++ = %d\n", ts_power_gpio_ref);
	mutex_unlock(&ts_power_gpio_sem);
}

static int syna_tcm_vci_enable(void)
{
	int retval = NO_ERR;
	int vol_vlaue = 0;

	if (tcm_hcd->syna_tcm_chip_data->ts_platform_data->fpga_flag == 1)
		return 0 ;

	if (IS_ERR(tcm_hcd->syna_tcm_tp_vci)) {
		TS_LOG_ERR("tp_vci is err\n");
		return -EINVAL;
	}

	vol_vlaue = tcm_hcd->syna_tcm_chip_data->regulator_ctr.vci_value;
	if (!IS_ERR(tcm_hcd->syna_tcm_tp_vci)) {
		if(g_tskit_ic_type == ONCELL)
		{
			TS_LOG_INFO("set vci voltage to %d\n", vol_vlaue);
			retval =
			    regulator_set_voltage(tcm_hcd->syna_tcm_tp_vci, vol_vlaue,
						  vol_vlaue);
			if (retval < 0) {
				TS_LOG_ERR
				    ("failed to set voltage regulator tp_vci error: %d\n",
				     retval);
				return -EINVAL;
			}
		}
		retval = regulator_enable(tcm_hcd->syna_tcm_tp_vci);
		if (retval < 0) {
			TS_LOG_ERR("failed to enable regulator tp_vci\n");
			return -EINVAL;
		}
	}
	return 0;
}

static int syna_tcm_vci_disable(void)
{
	int retval = NO_ERR;

	if (tcm_hcd->syna_tcm_chip_data->ts_platform_data->fpga_flag == 1)
		return 0;

	if (IS_ERR(tcm_hcd->syna_tcm_tp_vci)) {
		TS_LOG_ERR("tp_vci is err\n");
		return -EINVAL;
	}
	retval = regulator_disable(tcm_hcd->syna_tcm_tp_vci);
	if (retval < 0) {
		TS_LOG_ERR("failed to disable regulator tp_vci\n");
		return -EINVAL;
	}

	return 0;
}

static int syna_tcm_vddio_enable(void)
{
	int retval = NO_ERR;
	int vddio_value = 0;

	if (tcm_hcd->syna_tcm_chip_data->ts_platform_data->fpga_flag == 1)
		return 0 ;

	if (IS_ERR(tcm_hcd->syna_tcm_tp_vddio)) {
		TS_LOG_ERR("tp_vddio is err\n");
		return -EINVAL;
	}

	vddio_value = tcm_hcd->syna_tcm_chip_data->regulator_ctr.vddio_value;
	if (tcm_hcd->syna_tcm_chip_data->regulator_ctr.need_set_vddio_value) {
		TS_LOG_INFO("set tp_vddio voltage to %d\n", vddio_value);
		retval =
		    regulator_set_voltage(tcm_hcd->syna_tcm_tp_vddio, vddio_value,
					  vddio_value);
		if (retval < 0) {
			TS_LOG_ERR
			    ("failed to set voltage regulator tp_vddio error: %d\n",
			     retval);
			return -EINVAL;
		}
	}

	retval = regulator_enable(tcm_hcd->syna_tcm_tp_vddio);
	if (retval < 0) {
		TS_LOG_ERR("failed to enable regulator tp_vddio\n");
		return -EINVAL;
	}

	return 0;
}

static int syna_tcm_vddio_disable(void)
{
	int retval = NO_ERR;

	if (tcm_hcd->syna_tcm_chip_data->ts_platform_data->fpga_flag == 1)
		return 0;

	if (IS_ERR(tcm_hcd->syna_tcm_tp_vddio)) {
		TS_LOG_ERR("tp_vddio is err\n");
		return -EINVAL;
	}

	retval = regulator_disable(tcm_hcd->syna_tcm_tp_vddio);
	if (retval < 0) {
		TS_LOG_ERR("failed to disable regulator tp_vddio\n");
		return -EINVAL;
	}

	return 0;
}

static void syna_tcm_vci_on(void)
{
	TS_LOG_INFO("%s vci enable\n", __func__);
	if (1 == tcm_hcd->syna_tcm_chip_data->vci_regulator_type) {
		if (!IS_ERR(tcm_hcd->syna_tcm_tp_vci)) {
			TS_LOG_INFO("vci enable is called\n");
			syna_tcm_vci_enable();
		}
	}

	if (tcm_hcd->syna_tcm_chip_data->vci_gpio_type) {
		TS_LOG_INFO("%s vci switch gpio on\n", __func__);
		gpio_direction_output(tcm_hcd->syna_tcm_chip_data->vci_gpio_ctrl, 1);
	}
}

static void syna_tcm_vddio_on(void)
{
	TS_LOG_INFO("syna_tcm_vddio_on enable\n");
	if (1 == tcm_hcd->syna_tcm_chip_data->vddio_regulator_type) {
		if (!IS_ERR(tcm_hcd->syna_tcm_tp_vddio)) {
			TS_LOG_INFO("vddio enable is called\n");
			syna_tcm_vddio_enable();
		}
	}

	if (tcm_hcd->syna_tcm_chip_data->vddio_gpio_type) {
		TS_LOG_INFO("%s vddio switch gpio on\n", __func__);
		ts_kit_power_gpio_enable();
	}
}

static void syna_tcm_gpio_reset(void)
{
	int retval = 0;
	TS_LOG_INFO("synaptics_gpio_reset\n");
	if (!tcm_hcd->syna_tcm_chip_data->ts_platform_data->reset_gpio) {
		TS_LOG_INFO("reset_gpio is null, not supported reset\n");
		return;
	}

	gpio_direction_input(tcm_hcd->syna_tcm_chip_data->ts_platform_data->irq_gpio);
	TS_LOG_INFO("set gpio int input\n");

	gpio_direction_output(tcm_hcd->syna_tcm_chip_data->ts_platform_data->reset_gpio, 1);
	mdelay(1);
	gpio_direction_output(tcm_hcd->syna_tcm_chip_data->ts_platform_data->reset_gpio, 0);
	mdelay(300);
	gpio_direction_output(tcm_hcd->syna_tcm_chip_data->ts_platform_data->reset_gpio, 1);
	mdelay(bdata->reset_delay_ms);

	retval = gpio_request(bdata->display_reset_gpio, "display_reset_gpio");
	if (retval < 0) {
		TS_LOG_ERR("request display gpio fail\n");
		return;
	}
	gpio_direction_output(bdata->display_reset_gpio, 1);
	mdelay(1);
	gpio_direction_output(bdata->display_reset_gpio, 0);
	mdelay(300);
	gpio_direction_output(bdata->display_reset_gpio, 1);
	gpio_free(bdata->display_reset_gpio);
}

static void syna_tcm_power_off_gpio_set(void)
{
	TS_LOG_INFO("suspend RST out L\n");
	if (tcm_hcd->syna_tcm_chip_data->ts_platform_data->reset_gpio)
		gpio_direction_output(tcm_hcd->syna_tcm_chip_data->ts_platform_data->reset_gpio, 0);
	syna_tcm_pinctrl_select_lowpower();
	if (tcm_hcd->syna_tcm_chip_data->ts_platform_data->reset_gpio)
		gpio_direction_input(tcm_hcd->syna_tcm_chip_data->ts_platform_data->reset_gpio);
	mdelay(1);
}

static void syna_tcm_vddio_off(void)
{
	if (1 == tcm_hcd->syna_tcm_chip_data->vddio_regulator_type) {
		if (!IS_ERR(tcm_hcd->syna_tcm_tp_vddio)) {
			syna_tcm_vddio_disable();
		}
	}

	if (tcm_hcd->syna_tcm_chip_data->vddio_gpio_type) {
		TS_LOG_INFO("%s vddio switch gpio off\n", __func__);
		ts_kit_power_gpio_disable();
	}
}

static void syna_tcm_vci_off(void)
{
	if (1 == tcm_hcd->syna_tcm_chip_data->vci_regulator_type) {
		if (!IS_ERR(tcm_hcd->syna_tcm_tp_vci)) {
			syna_tcm_vci_disable();
		}
	}

	if (tcm_hcd->syna_tcm_chip_data->vci_gpio_type) {
		TS_LOG_INFO("%s vci switch gpio off\n", __func__);
		gpio_direction_output(tcm_hcd->syna_tcm_chip_data->vci_gpio_ctrl, 0);
	}
}

static void syna_tcm_regulator_put(void)
{
	if (tcm_hcd->syna_tcm_chip_data->ts_platform_data->fpga_flag == 1)
		return;

	if (1 == tcm_hcd->syna_tcm_chip_data->vci_regulator_type) {
		if (!IS_ERR(tcm_hcd->syna_tcm_tp_vci)) {
			regulator_put(tcm_hcd->syna_tcm_tp_vci);
		}
	}
	if (1 == tcm_hcd->syna_tcm_chip_data->vddio_regulator_type) {
		if (!IS_ERR(tcm_hcd->syna_tcm_tp_vddio)) {
			regulator_put(tcm_hcd->syna_tcm_tp_vddio);
		}
	}
}

static void syna_tcm_gpio_free(void)
{
	TS_LOG_INFO("syna_tcm_gpio_free called\n");

	// 0 is power supplied by gpio, 1 is power supplied by ldo 
	if (1 == tcm_hcd->syna_tcm_chip_data->vci_gpio_type) {
		if (tcm_hcd->syna_tcm_chip_data->vci_gpio_ctrl)
			gpio_free(tcm_hcd->syna_tcm_chip_data->vci_gpio_ctrl);
	}
	if (1 == tcm_hcd->syna_tcm_chip_data->vddio_gpio_type) {
		if (tcm_hcd->syna_tcm_chip_data->vddio_gpio_ctrl)
			gpio_free(tcm_hcd->syna_tcm_chip_data->vddio_gpio_ctrl);
	}
}

static void syna_tcm_power_off(void)
{
return;
	syna_tcm_power_off_gpio_set();
	if(g_tskit_ic_type < TDDI)
	{
		syna_tcm_vddio_off();
		mdelay(12);
		syna_tcm_vci_off();
		mdelay(30);
	}
}

static void syna_tcm_power_on(void)
{
	TS_LOG_INFO("syna_tcm_power_on called\n");
return;
	if(g_tskit_ic_type < TDDI)
	{
		syna_tcm_vci_on();
		mdelay(5);
		syna_tcm_vddio_on();
		mdelay(5);
	}
	syna_tcm_power_on_gpio_set();
}

static int syna_tcm_pinctrl_get_init(void)
{
	int ret = NO_ERR;

	if (tcm_hcd->syna_tcm_chip_data->ts_platform_data->fpga_flag == 1)
		return 0;

	tcm_hcd->pctrl = devm_pinctrl_get(&tcm_hcd->pdev->dev);
	if (IS_ERR(tcm_hcd->pctrl)) {
		TS_LOG_ERR("failed to devm pinctrl get\n");
		ret = -EINVAL;
		return ret;
	}

	tcm_hcd->pins_default = pinctrl_lookup_state(tcm_hcd->pctrl, "default");
	if (IS_ERR(tcm_hcd->pins_default)) {
		TS_LOG_ERR("failed to pinctrl lookup state default\n");
//		ret = -EINVAL;  //workaround
		goto err_pinctrl_put;
	}

	tcm_hcd->pins_idle = pinctrl_lookup_state(tcm_hcd->pctrl, "idle");
	if (IS_ERR(tcm_hcd->pins_idle)) {
		TS_LOG_ERR("failed to pinctrl lookup state idle\n");
//		ret = -EINVAL;  //workaround
		goto err_pinctrl_put;
	}

	return 0;

err_pinctrl_put:
	devm_pinctrl_put(tcm_hcd->pctrl);
	return ret;
}

static int syna_tcm_gpio_request(void)
{
	int retval = NO_ERR;
	TS_LOG_INFO("syna_tcm_gpio_request\n");

	if ((1 == tcm_hcd->syna_tcm_chip_data->vci_gpio_type)
	    && (1 == tcm_hcd->syna_tcm_chip_data->vddio_gpio_type)) {
		if (tcm_hcd->syna_tcm_chip_data->vci_gpio_ctrl ==
		   tcm_hcd->syna_tcm_chip_data->vddio_gpio_ctrl) {
			retval =
			    gpio_request(tcm_hcd->syna_tcm_chip_data->vci_gpio_ctrl, "ts_vci_gpio");
			if (retval) {
				TS_LOG_ERR
				    ("SFT:Ok;  ASIC: Real ERR----unable to request vci_gpio_ctrl firset:%d\n",
				     tcm_hcd->syna_tcm_chip_data->vci_gpio_ctrl);
				goto ts_vci_out;
			}
		} else {
			retval = gpio_request(tcm_hcd->syna_tcm_chip_data->vci_gpio_ctrl, "ts_vci_gpio");
			if (retval) {
				TS_LOG_ERR ("SFT:Ok;  ASIC: Real ERR----unable to request vci_gpio_ctrl2:%d\n",
				    tcm_hcd->syna_tcm_chip_data->vci_gpio_ctrl);
				goto ts_vci_out;
			}
			retval =
			    gpio_request(tcm_hcd->syna_tcm_chip_data->vddio_gpio_ctrl, "ts_vddio_gpio");
			if (retval) {
				TS_LOG_ERR
				    ("SFT:Ok;  ASIC: Real ERR----unable to request vddio_gpio_ctrl:%d\n",
				     tcm_hcd->syna_tcm_chip_data->vddio_gpio_ctrl);
				goto ts_vddio_out;
			}
		}
	} else {
		if (1 == tcm_hcd->syna_tcm_chip_data->vci_gpio_type) {
			retval =
			    gpio_request(tcm_hcd->syna_tcm_chip_data->vci_gpio_ctrl, "ts_vci_gpio");
			if (retval) {
				TS_LOG_ERR
				    ("SFT:Ok;  ASIC: Real ERR----unable to request vci_gpio_ctrl2:%d\n",
				    tcm_hcd->syna_tcm_chip_data->vci_gpio_ctrl);
				goto ts_vci_out;
			}
		}
		if (1 == tcm_hcd->syna_tcm_chip_data->vddio_gpio_type) {
			retval =
			    gpio_request(tcm_hcd->syna_tcm_chip_data->vddio_gpio_ctrl, "ts_vddio_gpio");
			if (retval) {
				TS_LOG_ERR
				    ("SFT:Ok;  ASIC: Real ERR----unable to request vddio_gpio_ctrl:%d\n",
				     tcm_hcd->syna_tcm_chip_data->vddio_gpio_ctrl);
				goto ts_vddio_out;
			}
		}
	}

	TS_LOG_INFO("reset:%d, irq:%d,\n",
		    tcm_hcd->syna_tcm_chip_data->ts_platform_data->reset_gpio,
		    tcm_hcd->syna_tcm_chip_data->ts_platform_data->irq_gpio);

	goto ts_reset_out;

ts_vddio_out:
	gpio_free(tcm_hcd->syna_tcm_chip_data->vci_gpio_ctrl);
ts_vci_out:
//	gpio_free(tcm_hcd->syna_tcm_chip_data->irq_gpio);
ts_reset_out:
	return retval;
}


static int syna_tcm_get_regulator(void)
{
	int retval;
	if (tcm_hcd->syna_tcm_chip_data->ts_platform_data->fpga_flag == 1)
		return 0;

	if (1 == tcm_hcd->syna_tcm_chip_data->vci_regulator_type) {
		tcm_hcd->syna_tcm_tp_vci =
		    regulator_get(&tcm_hcd->pdev->dev, bdata->bus_reg_name);
		if (IS_ERR(tcm_hcd->syna_tcm_tp_vci)) {
			TS_LOG_ERR("regulator tp vci not used\n");
			return -EINVAL;
		}
		retval = regulator_enable(tcm_hcd->syna_tcm_tp_vci);
		if (retval < 0) {
			TS_LOG_ERR("Failed to enable bus regulator\n");
			//goto exit;
		}
	}

	if (1 == tcm_hcd->syna_tcm_chip_data->vci_regulator_type) {
		tcm_hcd->syna_tcm_tp_vddio=
		    regulator_get(&tcm_hcd->pdev->dev, bdata->pwr_reg_name);
		if (IS_ERR(tcm_hcd->syna_tcm_tp_vddio)) {
			TS_LOG_ERR("regulator tp vddio not used\n");
			regulator_put(tcm_hcd->syna_tcm_tp_vddio);
			return -EINVAL;
		}
		retval = regulator_enable(tcm_hcd->syna_tcm_tp_vddio);
		if (retval < 0) {
			TS_LOG_ERR("Failed to enable pwr reg regulator\n");
			//goto exit;
		}
	}

	return 0;
}

static int syna_tcm_get_app_info(struct syna_tcm_hcd *tcm_hcd)
{
	int retval = NO_ERR;
	unsigned char *resp_buf = NULL;
	unsigned int resp_buf_size = 0;
	unsigned int resp_length = 0;
	unsigned int timeout = 0;

	timeout = APP_STATUS_POLL_TIMEOUT_MS;

	resp_buf = NULL;
	resp_buf_size = 0;

get_app_info:
	retval = tcm_hcd->write_message(tcm_hcd,
			CMD_GET_APPLICATION_INFO,
			NULL,
			0,
			&resp_buf,
			&resp_buf_size,
			&resp_length,
			NULL,
			0);
	if (retval < 0) {
		TS_LOG_ERR(
				"Failed to write command %s\n",
				STR(CMD_GET_APPLICATION_INFO));
		goto exit;
	}

	retval = secure_memcpy((unsigned char *)&tcm_hcd->app_info,
			sizeof(tcm_hcd->app_info),
			resp_buf,
			resp_buf_size,
			MIN(sizeof(tcm_hcd->app_info), resp_length));
	if (retval < 0) {
		TS_LOG_ERR(
				"Failed to copy application info\n");
		goto exit;
	}

	tcm_hcd->app_status = le2_to_uint(tcm_hcd->app_info.status);

	if (tcm_hcd->app_status == APP_STATUS_BOOTING ||
			tcm_hcd->app_status == APP_STATUS_UPDATING) {
		if (timeout > 0) {
			msleep(APP_STATUS_POLL_MS);
			timeout -= APP_STATUS_POLL_MS;
			goto get_app_info;
		}
	}

	retval = 0;

exit:
	kfree(resp_buf);

	return retval;
}

static int syna_tcm_get_boot_info(struct syna_tcm_hcd *tcm_hcd)
{
	int retval = NO_ERR;
	unsigned char *resp_buf = NULL;
	unsigned int resp_buf_size = 0;
	unsigned int resp_length = 0;

	resp_buf = NULL;
	resp_buf_size = 0;

	retval = tcm_hcd->write_message(tcm_hcd,
			CMD_GET_BOOT_INFO,
			NULL,
			0,
			&resp_buf,
			&resp_buf_size,
			&resp_length,
			NULL,
			0);
	if (retval < 0) {
		TS_LOG_ERR(
				"Failed to write command %s\n",
				STR(CMD_GET_BOOT_INFO));
		goto exit;
	}

	retval = secure_memcpy((unsigned char *)&tcm_hcd->boot_info,
			sizeof(tcm_hcd->boot_info),
			resp_buf,
			resp_buf_size,
			MIN(sizeof(tcm_hcd->boot_info), resp_length));
	if (retval < 0) {
		TS_LOG_ERR(
				"Failed to copy boot info\n");
		goto exit;
	}

	retval = 0;

exit:
	kfree(resp_buf);

	return retval;
}

static int syna_tcm_identify(struct syna_tcm_hcd *tcm_hcd, bool id)
{
	int retval = NO_ERR;
	unsigned char *resp_buf = NULL;
	unsigned int resp_buf_size = 0;
	unsigned int resp_length = 0;
	unsigned int max_write_size = 0;

	resp_buf = NULL;
	resp_buf_size = 0;

	if (!id)
		goto get_info;

	retval = tcm_hcd->write_message(tcm_hcd,
			CMD_IDENTIFY,
			NULL,
			0,
			&resp_buf,
			&resp_buf_size,
			&resp_length,
			NULL,
			0);
	if (retval < 0) {
		TS_LOG_ERR(
				"Failed to write command %s\n",
				STR(CMD_IDENTIFY));
		goto exit;
	}

	retval = secure_memcpy((unsigned char *)&tcm_hcd->id_info,
			sizeof(tcm_hcd->id_info),
			resp_buf,
			resp_buf_size,
			MIN(sizeof(tcm_hcd->id_info), resp_length));
	if (retval < 0) {
		TS_LOG_ERR(
				"Failed to copy identification info\n");
		goto exit;
	}

	tcm_hcd->packrat_number = le4_to_uint(tcm_hcd->id_info.build_id);

	max_write_size = le2_to_uint(tcm_hcd->id_info.max_write_size);
	tcm_hcd->wr_chunk_size = MIN(max_write_size, WR_CHUNK_SIZE);
	if (tcm_hcd->wr_chunk_size == 0)
		tcm_hcd->wr_chunk_size = max_write_size;

get_info:
	switch (tcm_hcd->id_info.mode) {
	case MODE_APPLICATION:
		retval = syna_tcm_get_app_info(tcm_hcd);
		if (retval < 0) {
			TS_LOG_ERR(
					"Failed to get application info\n");
			goto exit;
		}
		break;
	case MODE_BOOTLOADER:
	case MODE_TDDI_BOOTLOADER:
		retval = syna_tcm_get_boot_info(tcm_hcd);
		if (retval < 0) {
			TS_LOG_ERR(
					"Failed to get boot info\n");
			goto exit;
		}
		break;
	default:
		break;
	}

	retval = 0;

exit:

	kfree(resp_buf);

	return retval;
}

static int syna_tcm_run_application_firmware(struct syna_tcm_hcd *tcm_hcd)
{
	int retval = NO_ERR;
	bool retry = false;
	unsigned char *resp_buf = NULL;
	unsigned int resp_buf_size = 0;
	unsigned int resp_length = 0;

	retry = true;

	resp_buf = NULL;
	resp_buf_size = 0;

retry:
	retval = tcm_hcd->write_message(tcm_hcd,
			CMD_RUN_APPLICATION_FIRMWARE,
			NULL,
			0,
			&resp_buf,
			&resp_buf_size,
			&resp_length,
			NULL,
			MODE_SWITCH_DELAY_MS);
	if (retval < 0) {
		TS_LOG_ERR(
				"Failed to write command %s\n",
				STR(CMD_RUN_APPLICATION_FIRMWARE));
		goto exit;
	}

	retval = tcm_hcd->identify(tcm_hcd, false);
	if (retval < 0) {
		TS_LOG_ERR(
				"Failed to do identification\n");
		goto exit;
	}

	if (tcm_hcd->id_info.mode != MODE_APPLICATION) {
		TS_LOG_ERR(
				"Failed to run application firmware (boot status = 0x%02x)\n",
				tcm_hcd->boot_info.status);
		if (retry) {
			retry = false;
			goto retry;
		}
		retval = -EINVAL;
		goto exit;
	} else if (tcm_hcd->app_status != APP_STATUS_OK) {
		TS_LOG_ERR(
				"Application status = 0x%02x\n",
				tcm_hcd->app_status);
	}

	retval = 0;

exit:
	kfree(resp_buf);

	return retval;
}

static int syna_tcm_run_bootloader_firmware(struct syna_tcm_hcd *tcm_hcd)
{
	int retval = NO_ERR;
	unsigned char *resp_buf = NULL;
	unsigned int resp_buf_size = 0;
	unsigned int resp_length = 0;

	resp_buf = NULL;
	resp_buf_size = 0;

	retval = tcm_hcd->write_message(tcm_hcd,
			CMD_RUN_BOOTLOADER_FIRMWARE,
			NULL,
			0,
			&resp_buf,
			&resp_buf_size,
			&resp_length,
			NULL,
			MODE_SWITCH_DELAY_MS);
	if (retval < 0) {
		TS_LOG_ERR(
				"Failed to write command %s\n",
				STR(CMD_RUN_BOOTLOADER_FIRMWARE));
		goto exit;
	}

	retval = tcm_hcd->identify(tcm_hcd, false);
	if (retval < 0) {
		TS_LOG_ERR(
				"Failed to do identification\n");
		goto exit;
	}

	if (tcm_hcd->id_info.mode == MODE_APPLICATION) {
		TS_LOG_ERR(
				"Failed to enter bootloader mode\n");
		retval = -EINVAL;
		goto exit;
	}

	retval = 0;

exit:
	kfree(resp_buf);

	return retval;
}

static int syna_tcm_switch_mode(struct syna_tcm_hcd *tcm_hcd,
		enum firmware_mode mode)
{
	int retval = NO_ERR;

	tcm_hcd->update_watchdog(tcm_hcd, false);

	switch (mode) {
	case FW_MODE_BOOTLOADER:
		retval = syna_tcm_run_bootloader_firmware(tcm_hcd);
		if (retval < 0) {
			TS_LOG_ERR(
					"Failed to switch to bootloader mode\n");
			goto exit;
		}
		break;
	case FW_MODE_APPLICATION:
		retval = syna_tcm_run_application_firmware(tcm_hcd);
		if (retval < 0) {
			TS_LOG_ERR(
					"Failed to switch to application mode\n");
			goto exit;
		}
		break;
	default:
		TS_LOG_ERR(
				"Invalid firmware mode\n");
		retval = -EINVAL;
		goto exit;
	}

	retval = 0;

exit:
	tcm_hcd->update_watchdog(tcm_hcd, true);

	return retval;
}

static int syna_tcm_get_dynamic_config(struct syna_tcm_hcd *tcm_hcd,
		enum dynamic_config_id id, unsigned short *value)
{
	int retval = NO_ERR;
	unsigned char out_buf = 0;
	unsigned char *resp_buf = NULL;
	unsigned int resp_buf_size = 0;
	unsigned int resp_length = 0;

	resp_buf = NULL;
	resp_buf_size = 0;

	out_buf = (unsigned char)id;

	retval = tcm_hcd->write_message(tcm_hcd,
			CMD_GET_DYNAMIC_CONFIG,
			&out_buf,
			sizeof(out_buf),
			&resp_buf,
			&resp_buf_size,
			&resp_length,
			NULL,
			0);
	if (retval < 0) {
		TS_LOG_ERR(
				"Failed to write command %s\n",
				STR(CMD_GET_DYNAMIC_CONFIG));
		goto exit;
	}

	if (resp_length < 2) {
		TS_LOG_ERR(
				"Invalid data length\n");
		retval = -EINVAL;
		goto exit;
	}

	*value = (unsigned short)le2_to_uint(resp_buf);

	retval = 0;

exit:
	kfree(resp_buf);

	return retval;
}

static int syna_tcm_set_dynamic_config(struct syna_tcm_hcd *tcm_hcd,
		enum dynamic_config_id id, unsigned short value)
{
	int retval = NO_ERR;
	unsigned char out_buf[3] = {0};
	unsigned char *resp_buf = NULL;
	unsigned int resp_buf_size = 0;
	unsigned int resp_length = 0;

	resp_buf = NULL;
	resp_buf_size = 0;

	out_buf[0] = (unsigned char)id;
	out_buf[1] = (unsigned char)value;
	out_buf[2] = (unsigned char)(value >> 8);

	retval = tcm_hcd->write_message(tcm_hcd,
			CMD_SET_DYNAMIC_CONFIG,
			out_buf,
			sizeof(out_buf),
			&resp_buf,
			&resp_buf_size,
			&resp_length,
			NULL,
			0);
	if (retval < 0) {
		TS_LOG_ERR(
				"Failed to write command %s\n",
				STR(CMD_SET_DYNAMIC_CONFIG));
		goto exit;
	}

	retval = 0;

exit:
	kfree(resp_buf);

	return retval;
}

static int syna_tcm_get_data_location(struct syna_tcm_hcd *tcm_hcd,
		enum flash_area area, unsigned int *addr, unsigned int *length)
{
	int retval = NO_ERR;
	unsigned char out_buf = 0;
	unsigned char *resp_buf = NULL;
	unsigned int resp_buf_size = 0;
	unsigned int resp_length = 0;

	switch (area) {
	case CUSTOM_LCM:
		out_buf = LCM_DATA;
		break;
	case CUSTOM_OEM:
		out_buf = OEM_DATA;
		break;
	case PPDT:
		out_buf = PPDT_DATA;
		break;
	default:
		TS_LOG_ERR(
				"Invalid flash area\n");
		return -EINVAL;
	}

	resp_buf = NULL;
	resp_buf_size = 0;

	retval = tcm_hcd->write_message(tcm_hcd,
			CMD_GET_DATA_LOCATION,
			&out_buf,
			sizeof(out_buf),
			&resp_buf,
			&resp_buf_size,
			&resp_length,
			NULL,
			0);
	if (retval < 0) {
		TS_LOG_ERR(
				"Failed to write command %s\n",
				STR(CMD_GET_DATA_LOCATION));
		goto exit;
	}

	if (resp_length != 4) {
		TS_LOG_ERR(
				"Invalid data length\n");
		retval = -EINVAL;
		goto exit;
	}

	*addr = le2_to_uint(&resp_buf[0]);
	*length = le2_to_uint(&resp_buf[2]);

	retval = 0;

exit:
	kfree(resp_buf);

	return retval;
}

static int syna_tcm_sleep(struct syna_tcm_hcd *tcm_hcd, bool en)
{
	int retval = NO_ERR;
	unsigned char command = 0;
	unsigned char *resp_buf = NULL;
	unsigned int resp_buf_size = 0;
	unsigned int resp_length = 0;

	command = en ? CMD_ENTER_DEEP_SLEEP : CMD_EXIT_DEEP_SLEEP;

	resp_buf = NULL;
	resp_buf_size = 0;

	retval = tcm_hcd->write_message(tcm_hcd,
			command,
			NULL,
			0,
			&resp_buf,
			&resp_buf_size,
			&resp_length,
			NULL,
			0);
	if (retval < 0) {
		TS_LOG_ERR(
				"Failed to write command %s\n",
				en ?
				STR(CMD_ENTER_DEEP_SLEEP) :
				STR(CMD_EXIT_DEEP_SLEEP));
		goto exit;
	}

	retval = 0;

exit:
	kfree(resp_buf);

	return retval;
}

static int syna_tcm_reset(struct syna_tcm_hcd *tcm_hcd, bool hw, bool update_wd)
{
	int retval = NO_ERR;
	unsigned char *resp_buf = NULL;
	unsigned int resp_buf_size = 0;
	unsigned int resp_length = 0;

	resp_buf = NULL;
	resp_buf_size = 0;

	if (update_wd)
		tcm_hcd->update_watchdog(tcm_hcd, false);

	if (hw) {
		if (tcm_hcd->syna_tcm_chip_data->ts_platform_data->reset_gpio < 0) {
			TS_LOG_ERR(
					"Hardware reset unavailable\n");
			retval = -EINVAL;
			goto exit;
		}
		gpio_set_value(tcm_hcd->syna_tcm_chip_data->ts_platform_data->reset_gpio, bdata->reset_on_state);
		msleep(bdata->reset_active_ms);
		gpio_set_value(tcm_hcd->syna_tcm_chip_data->ts_platform_data->reset_gpio, !bdata->reset_on_state);
	} else {
		retval = tcm_hcd->write_message(tcm_hcd,
				CMD_RESET,
				NULL,
				0,
				&resp_buf,
				&resp_buf_size,
				&resp_length,
				NULL,
				bdata->reset_delay_ms);
		if (retval < 0 && !tcm_hcd->host_download_mode) {
			TS_LOG_ERR(
					"Failed to write command %s\n",
					STR(CMD_RESET));
			goto exit;
		}
	}

	if (tcm_hcd->host_download_mode) {
		kfree(resp_buf);
		retval = syna_tcm_wait_hdl(tcm_hcd);
		if (retval < 0) {
			TS_LOG_ERR(
					"Failed to wait for completion of host download\n");
			return retval;
		}
		if (update_wd)
			tcm_hcd->update_watchdog(tcm_hcd, true);
		return 0;
	}

	msleep(bdata->reset_delay_ms);

	retval = tcm_hcd->identify(tcm_hcd, false);
	if (retval < 0) {
		TS_LOG_ERR(
				"Failed to do identification\n");
		goto exit;
	}

	if (tcm_hcd->id_info.mode == MODE_APPLICATION)
		goto dispatch_reset;

	retval = tcm_hcd->write_message(tcm_hcd,
			CMD_RUN_APPLICATION_FIRMWARE,
			NULL,
			0,
			&resp_buf,
			&resp_buf_size,
			&resp_length,
			NULL,
			MODE_SWITCH_DELAY_MS);
	if (retval < 0) {
		TS_LOG_INFO(
				"Failed to write command %s\n",
				STR(CMD_RUN_APPLICATION_FIRMWARE));
	}

	retval = tcm_hcd->identify(tcm_hcd, false);
	if (retval < 0) {
		TS_LOG_ERR(
				"Failed to do identification\n");
		goto exit;
	}

dispatch_reset:
	TS_LOG_INFO(
			"Firmware mode = 0x%02x\n",
			tcm_hcd->id_info.mode);

	if (tcm_hcd->id_info.mode != MODE_APPLICATION) {
		TS_LOG_INFO(
				"Boot status = 0x%02x\n",
				tcm_hcd->boot_info.status);
	} else if (tcm_hcd->app_status != APP_STATUS_OK) {
		TS_LOG_INFO(
				"Application status = 0x%02x\n",
				tcm_hcd->app_status);
	}

	retval = 0;

exit:
	if (update_wd)
		tcm_hcd->update_watchdog(tcm_hcd, true);

	kfree(resp_buf);

	return retval;
}
/*
static int syna_tcm_rezero(struct syna_tcm_hcd *tcm_hcd)
{
	int retval = NO_ERR;
	unsigned char *resp_buf = NULL;
	unsigned int resp_buf_size = 0;
	unsigned int resp_length = 0;

	resp_buf = NULL;
	resp_buf_size = 0;

	retval = tcm_hcd->write_message(tcm_hcd,
			CMD_REZERO,
			NULL,
			0,
			&resp_buf,
			&resp_buf_size,
			&resp_length,
			NULL,
			0);
	if (retval < 0) {
		TS_LOG_ERR(
				"Failed to write command %s\n",
				STR(CMD_REZERO));
		goto exit;
	}

	retval = 0;

exit:
	kfree(resp_buf);

	return retval;
}
*/
static int syna_tcm_comm_check(void) 
{
#if 0
	int retval = NO_ERR;
	unsigned char uboot[6] = {0};
	
	retval = syna_tcm_rmi_read(tcm_hcd, PDT_START_ADDR, uboot, 6);
	TS_LOG_ERR("detect synaptics uboot[0] = %x.uboot[1] = %xuboot[2] = %x.uboot[3] = %xuboot[4] = %x.uboot[5] = %x\n",
		uboot[0],uboot[1], uboot[2], uboot[3], uboot[4],uboot[5]);

	if (uboot[5] == UBL_FN_NUMBER)
		return 0;
	else {
		TS_LOG_ERR("failed to detect F$35!");
		return -ENODEV;
	}
#else
	int retval = NO_ERR;
	unsigned char marker;
	retval = syna_tcm_read(tcm_hcd,
			&marker,
			1);
	TS_LOG_ERR("check result:%d\n", retval);
	
	if (retval < 0 || marker != MESSAGE_MARKER) {
		TS_LOG_ERR(
				"Failed to read from device\n");
		return RESULT_ERR;
	}
	return NO_ERR;
#endif
}

static int syna_tcm_get_cal_data(struct ts_calibration_data_info *info,
				 struct ts_cmd_node *out_cmd)
{
	struct syna_tcm_app_info *app_info;
	
	TS_LOG_INFO("syna_tcm_get_cal_data called\n");

	app_info = &tcm_hcd->app_info;
	info->tx_num = le2_to_uint(app_info->num_of_image_rows);
	info->rx_num = le2_to_uint(app_info->num_of_image_cols);

	return 0;
}

static int syna_tcm_mmi_test(struct ts_rawdata_info *info,
				 struct ts_cmd_node *out_cmd)
{
	int retval = 0;
//	const char *module_name = 0x01;

	//BEGIN PN: DTS2014101408241,Modified by l00216194, 2014/10/14 
	if (1) {        //(1 == tcm_hcd->syna_tcm_chip_data->unite_cap_test_interface) {
		// END   PN: DTS2014101408241,Modified by l00216194, 2014/10/14 
		TS_LOG_INFO("++++ syna_tcm_mmi_test in\n");
		retval = testing_init(tcm_hcd);
		if (retval < 0) {
			TS_LOG_ERR("Failed to init test_tcm\n");
			return retval;
		}

		retval = syna_tcm_testing(info);
		if (retval < 0) {
			TS_LOG_ERR("Failed to syna_tcm_testing\n");
			return retval;
		}
/*
		retval = testing_remove(tcm_hcd);
		if (retval < 0) {
			TS_LOG_ERR("Failed to remove test_tcm\n");
			return retval;
		}
*/		
		return NO_ERR;
	} else {
		TS_LOG_INFO("++++ syna_tcm_mmi_test err out\n");
		return -EINVAL;
	}
}

static int syna_tcm_read_one_package(struct ts_fingers *info)
{
	int retval = NO_ERR;
	bool retry = false;
	struct syna_tcm_message_header *header = NULL;

	retry = true;

retry:
	
	retval = syna_tcm_read(tcm_hcd,
			buffer,
			FIXED_READ_LENGTH);
	if (retval < 0) {
		TS_LOG_ERR(
				"Failed to read from device\n");
		if (retry) {
			usleep_range(READ_RETRY_US_MIN, READ_RETRY_US_MAX);
			retry = false;
			goto retry;
		}
		goto exit;
	}

	header = (struct syna_tcm_message_header *)buffer;

	if (header->marker != MESSAGE_MARKER) {
		TS_LOG_ERR(
				"Incorrect header marker (0x%02x)\n",
				header->marker);
		retval = -ENXIO;
		if (retry) {
			usleep_range(READ_RETRY_US_MIN, READ_RETRY_US_MAX);
			retry = false;
			goto retry;
		}
		goto exit;
	}

	tcm_hcd->status_report_code = header->code;

	tcm_hcd->payload_length = le2_to_uint(header->length);

	TS_LOG_INFO(
			"Header code = 0x%02x\n",
			tcm_hcd->status_report_code);

	TS_LOG_INFO(
			"Payload length = %d\n",
			tcm_hcd->payload_length);

	if (tcm_hcd->status_report_code <= STATUS_ERROR ||
			tcm_hcd->status_report_code == STATUS_INVALID) {
		switch (tcm_hcd->status_report_code) {
		case STATUS_OK:
			break;
		case STATUS_CONTINUED_READ:
			TS_LOG_INFO(
					"Out-of-sync continued read\n");
		case STATUS_IDLE:
		case STATUS_BUSY:
			tcm_hcd->payload_length = 0;
			retval = 0;
			goto exit;
		default:
			TS_LOG_ERR(
					"Incorrect header code (0x%02x)\n",
					tcm_hcd->status_report_code);
			if (tcm_hcd->status_report_code == STATUS_INVALID)
				tcm_hcd->payload_length = 0;
		}
	}

	if (tcm_hcd->status_report_code >= REPORT_IDENTIFY) {
		LOCK_BUFFER(tcm_hcd->report.buffer);
	
		tcm_hcd->report.buffer.buf = &buffer[MESSAGE_HEADER_SIZE];
	
		tcm_hcd->report.buffer.buf_size = tcm_hcd->payload_length;
	
		tcm_hcd->report.buffer.data_length = tcm_hcd->payload_length;
	
		tcm_hcd->report.id = tcm_hcd->status_report_code;
	
		touch_report(info);
	
		UNLOCK_BUFFER(tcm_hcd->report.buffer);
	}
	retval = 0;

exit:
 
	return retval;
	
}

static int syna_tcm_suspend(void)
{
	int retval = NO_ERR;

	return retval;
	TS_LOG_INFO("suspend +\n");
	
	if (tcm_hcd->in_suspend)
			return 0;
	
	gpio_direction_output(tcm_hcd->syna_tcm_chip_data->ts_platform_data->reset_gpio, 0);

	TS_LOG_INFO("suspend -\n");

	tcm_hcd->in_suspend = true;

	return retval;
}

 //   do not add time-costly function here.

static int syna_tcm_resume(void)
{
	int retval = NO_ERR;

	return retval;

	if (!tcm_hcd->in_suspend)
			return 0;	
	TS_LOG_INFO("resume +\n");

	gpio_direction_output(tcm_hcd->syna_tcm_chip_data->ts_platform_data->reset_gpio, 1);
	msleep(bdata->reset_delay_ms);
	
	pre_finger_status = 0;
	tcm_hcd->in_suspend = false;
	TS_LOG_INFO("resume -\n");
	return retval;
}

/*  do some things after power on. */
static int syna_tcm_after_resume(void *feature_info)
{
	int retval = NO_ERR;
	TS_LOG_INFO("after_resume +\n");

// waiting...

	TS_LOG_INFO("after_resume -\n");
	return retval;
}

static int syna_tcm_before_suspend(void)
{
	int retval = NO_ERR;

	TS_LOG_INFO("before_suspend +\n");
	TS_LOG_INFO("before_suspend -\n");
	return retval;
}

static int syna_tcm_set_info_flag(struct ts_kit_platform_data *info)
{
	tcm_hcd->syna_tcm_chip_data->ts_platform_data->get_info_flag = info->get_info_flag;
	return NO_ERR;
}

static int syna_tcm_irq_top_half(struct ts_cmd_node *cmd)
{
	cmd->command = TS_INT_PROCESS;
	return NO_ERR;
}

static int syna_tcm_fw_update_sd(void)
{
	int retval = NO_ERR;
	TS_LOG_INFO("%s is called\n", __func__);
	//reflash_do_reflash();
	return retval;
}

static int syna_tcm_chip_get_info(struct ts_chip_info_param *info)
{
	unsigned char buf[CHIP_INFO_LENGTH * 2] = { 0 };
	
	memcpy(&info->fw_vendor, tcm_hcd->app_info.customer_config_id, strlen(tcm_hcd->app_info.customer_config_id));
	memcpy(&buf, SYNA_TCM_CHIP_INFO, strlen(SYNA_TCM_CHIP_INFO));
	strncat(buf, tcm_hcd->app_info.customer_config_id, strlen(tcm_hcd->app_info.customer_config_id));  // no project id, 
	memcpy(&info->ic_vendor, buf, strlen(buf));

	return 0;
}

static int syna_tcm_fw_update_boot(char *file_name)
{
#if 0
	int retval = NO_ERR;
	int projectid_lenth = 0;

	if (tcm_hcd->syna_tcm_chip_data->projectid_len) {
		projectid_lenth = tcm_hcd->syna_tcm_chip_data->projectid_len;
	} else {
		projectid_lenth = PROJECT_ID_FW_LEN;
	}

	TS_LOG_INFO("syna_tcm_fw_update_boot called\n");

	retval = zeroflash_init(tcm_hcd);
	if (retval) {
		TS_LOG_ERR("zeroflash_init failed\n");
		goto data_release;
	}

	strncat(file_name, tcm_hcd->tcm_mod_info.project_id_string,
		projectid_lenth);
	TS_LOG_INFO("file_name name is :%s\n", file_name);

	retval = zeroflash_get_fw_image();
	if (retval) {
		retval = 0;
		TS_LOG_ERR("load fw data from bootimage error\n");
		goto data_release;
	}

	retval = zeroflash_download();
	if (retval) {
		TS_LOG_ERR("failed to download fw\n");
	} else {
		TS_LOG_INFO("downloaded firmware successfully\n");
		tcm_hcd->host_download_mode = true;
		retval = touch_init(tcm_hcd);
			if (retval) 
				TS_LOG_ERR("failed to touch_init\n");
	}
//	syna_tcm_mmi_test(info, out_cmd);
/*
	retval = testing_init(tcm_hcd);
	if (retval < 0) {
		TS_LOG_ERR("Failed to init test_tcm\n");
		return retval;
	}
*/
	return retval;

data_release:
	zeroflash_remove(tcm_hcd);
	return retval;
#else
	int retval = NO_ERR;
	TS_LOG_ERR("yuehao call firmware update\n");
	retval = touch_init(tcm_hcd);

	reflash_do_reflash();
	retval = touch_init(tcm_hcd);
	return retval;
#endif
}

static int syna_tcm_irq_bottom_half(struct ts_cmd_node *in_cmd,
				     struct ts_cmd_node *out_cmd)
{
	int retval = NO_ERR;
	struct ts_fingers *info =
	    &out_cmd->cmd_param.pub_params.algo_param.info;

	out_cmd->command = TS_INPUT_ALGO;
	out_cmd->cmd_param.pub_params.algo_param.algo_order =
	    tcm_hcd->syna_tcm_chip_data->algo_id;
	TS_LOG_DEBUG("order: %d\n",
		     out_cmd->cmd_param.pub_params.algo_param.algo_order);

	retval = syna_tcm_read_one_package(info);
	if (retval < 0) {
		TS_LOG_ERR("Failed to syna_tcm_read_one_package, try to read F$35\n");

		retval = syna_tcm_comm_check();
		if (retval < 0)
			goto exit;
		else { 
			if (tcm_hcd->host_download_mode)
				zeroflash_download();

		}

	}
	
exit:
	return retval;
}

static int syna_tcm_input_config(struct input_dev *input_dev)
{
	set_bit(EV_SYN, input_dev->evbit);
	set_bit(EV_KEY, input_dev->evbit);
	set_bit(EV_ABS, input_dev->evbit);
	set_bit(BTN_TOUCH, input_dev->keybit);
	set_bit(BTN_TOOL_FINGER, input_dev->keybit);

	set_bit(TS_DOUBLE_CLICK, input_dev->keybit);
	set_bit(TS_SLIDE_L2R, input_dev->keybit);
	set_bit(TS_SLIDE_R2L, input_dev->keybit);
	set_bit(TS_SLIDE_T2B, input_dev->keybit);
	set_bit(TS_SLIDE_B2T, input_dev->keybit);
	set_bit(TS_CIRCLE_SLIDE, input_dev->keybit);
	set_bit(TS_LETTER_c, input_dev->keybit);
	set_bit(TS_LETTER_e, input_dev->keybit);
	set_bit(TS_LETTER_m, input_dev->keybit);
	set_bit(TS_LETTER_w, input_dev->keybit);
	set_bit(TS_PALM_COVERED, input_dev->keybit);

	set_bit(TS_TOUCHPLUS_KEY0, input_dev->keybit);
	set_bit(TS_TOUCHPLUS_KEY1, input_dev->keybit);
	set_bit(TS_TOUCHPLUS_KEY2, input_dev->keybit);
	set_bit(TS_TOUCHPLUS_KEY3, input_dev->keybit);
	set_bit(TS_TOUCHPLUS_KEY4, input_dev->keybit);

#ifdef INPUT_PROP_DIRECT
	set_bit(INPUT_PROP_DIRECT, input_dev->propbit);
#endif

	input_set_abs_params(input_dev, ABS_X,
			     0, bdata->x_max, 0, 0);
	input_set_abs_params(input_dev, ABS_Y,
			     0, bdata->y_max, 0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0, 15, 0, 0);

	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0,
			     bdata->x_max_mt, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0,
			     bdata->y_max_mt, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, 255, 0, 0);
#if ANTI_FALSE_TOUCH_USE_PARAM_MAJOR_MINOR
	input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR, 0, 100, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_WIDTH_MINOR, 0, 100, 0, 0);
#else
	input_set_abs_params(input_dev, ABS_MT_DISTANCE, 0, 100, 0, 0);
#endif
#ifdef REPORT_2D_W
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0,
			     MAX_ABS_MT_TOUCH_MAJOR, 0, 0);
#endif

#ifdef TYPE_B_PROTOCOL
#ifdef KERNEL_ABOVE_3_7
	// input_mt_init_slots now has a "flags" parameter
	input_mt_init_slots(input_dev, bdata->max_finger_objects,
			    INPUT_MT_DIRECT);
#else
	input_mt_init_slots(input_dev, bdata->max_finger_objects);
#endif
#endif

	return NO_ERR;
}

static int syna_tcm_parse_dts(struct device_node *np, struct ts_kit_device_data *chip_data)
{
	int retval = NO_ERR;
	u32 value = 0;
	struct property *prop = NULL;
	const char *name = NULL;

	retval = of_property_read_u32(np, "synaptics,irq-on-state", &value);
	if (retval < 0)
		bdata->irq_on_state = 0;
	else
		bdata->irq_on_state = value;

	retval = of_property_read_string(np, "synaptics,pwr-reg-name", &name);
	if (retval < 0)
		bdata->pwr_reg_name = NULL;
	else
		bdata->pwr_reg_name = name;

	retval = of_property_read_string(np, "synaptics,bus-reg-name", &name);
	if (retval < 0)
		bdata->bus_reg_name = NULL;
	else
		bdata->bus_reg_name = name;

	prop = of_find_property(np, "synaptics,power-gpio", NULL);
	if (prop && prop->length) {
		bdata->power_gpio = of_get_named_gpio_flags(np,
				"synaptics,power-gpio", 0, NULL);
	} else {
		bdata->power_gpio = -1;
	}
/*
	prop = of_find_property(np, "synaptics,reset-gpio", NULL);
	if (prop && prop->length) {
		bdata->reset_gpio = of_get_named_gpio_flags(np,
				"synaptics,reset-gpio", 0, NULL);
	} else {
		bdata->reset_gpio = -1;
	}
*/
	prop = of_find_property(np, "synaptics,display-reset-gpio", NULL);
	if (prop && prop->length) {
		bdata->display_reset_gpio = of_get_named_gpio_flags(np,
				"synaptics,display-reset-gpio", 0, NULL);
	} else {
		bdata->display_reset_gpio = -1;
	}

	prop = of_find_property(np, "synaptics,power-on-state", NULL);
	if (prop && prop->length) {
		retval = of_property_read_u32(np, "synaptics,power-on-state",
				&value);
		if (retval < 0) {
			TS_LOG_ERR(
					"Failed to read synaptics,power-on-state property\n");
			return retval;
		} else {
			bdata->power_on_state = value;
		}
	} else {
		bdata->power_on_state = 0;
	}


	prop = of_find_property(np, "synaptics,power-delay-ms", NULL);
	if (prop && prop->length) {
		retval = of_property_read_u32(np, "synaptics,power-delay-ms",
				&value);
		if (retval < 0) {
			TS_LOG_ERR(
					"Failed to read synaptics,power-delay-ms property\n");
			return retval;
		} else {
			bdata->power_delay_ms = value;
		}
	} else {
		bdata->power_delay_ms = 0;
	}

	prop = of_find_property(np, "synaptics,reset-on-state", NULL);
	if (prop && prop->length) {
		retval = of_property_read_u32(np, "synaptics,reset-on-state",
				&value);
		if (retval < 0) {
			TS_LOG_ERR(
					"Failed to read synaptics,reset-on-state property\n");
			return retval;
		} else {
			bdata->reset_on_state = value;
		}
	} else {
		bdata->reset_on_state = 0;
	}

	prop = of_find_property(np, "synaptics,reset-active-ms", NULL);
	if (prop && prop->length) {
		retval = of_property_read_u32(np, "synaptics,reset-active-ms",
				&value);
		if (retval < 0) {
			TS_LOG_ERR(
					"Failed to read synaptics,reset-active-ms property\n");
			return retval;
		} else {
			bdata->reset_active_ms = value;
		}
	} else {
		bdata->reset_active_ms = 0;
	}

	prop = of_find_property(np, "synaptics,reset-delay-ms", NULL);
	if (prop && prop->length) {
		retval = of_property_read_u32(np, "synaptics,reset-delay-ms",
				&value);
		if (retval < 0) {
			TS_LOG_ERR(
					"Unable to read synaptics,reset-delay-ms property\n");
			return retval;
		} else {
			bdata->reset_delay_ms = value;
		}
	} else {
		bdata->reset_delay_ms = 0;
	}

	prop = of_find_property(np, "synaptics,x-flip", NULL);
	bdata->x_flip = prop > 0 ? true : false;

	prop = of_find_property(np, "synaptics,y-flip", NULL);
	bdata->y_flip = prop > 0 ? true : false;

	prop = of_find_property(np, "synaptics,swap-axes", NULL);
	bdata->swap_axes = prop > 0 ? true : false;

	prop = of_find_property(np, "synaptics,byte-delay-us", NULL);
	if (prop && prop->length) {
		retval = of_property_read_u32(np, "synaptics,byte-delay-us",
				&value);
		if (retval < 0) {
			TS_LOG_ERR(
					"Unable to read synaptics,byte-delay-us property\n");
			return retval;
		} else {
			bdata->byte_delay_us = value;
		}
	} else {
		bdata->byte_delay_us = 0;
	}

	prop = of_find_property(np, "synaptics,block-delay-us", NULL);
	if (prop && prop->length) {
		retval = of_property_read_u32(np, "synaptics,block-delay-us",
				&value);
		if (retval < 0) {
			TS_LOG_ERR(
					"Unable to read synaptics,block-delay-us property\n");
			return retval;
		} else {
			bdata->block_delay_us = value;
		}
	} else {
		bdata->block_delay_us = 0;
	}

	prop = of_find_property(np, "synaptics,spi-mode", NULL);
	if (prop && prop->length) {
		retval = of_property_read_u32(np, "synaptics,spi-mode",
				&value);
		if (retval < 0) {
			TS_LOG_ERR(
					"Unable to read synaptics,spi-mode property\n");
			return retval;
		} else {
			bdata->spi_mode = value;
		}
	} else {
		bdata->spi_mode = 0;
	}

	prop = of_find_property(np, "synaptics,ubl-max-freq", NULL);
	if (prop && prop->length) {
		retval = of_property_read_u32(np, "synaptics,ubl-max-freq",
				&value);
		if (retval < 0) {
			TS_LOG_ERR(
					"Unable to read synaptics,ubl-max-freq property\n");
			return retval;
		} else {
			bdata->ubl_max_freq = value;
		}
	} else {
		bdata->ubl_max_freq = 0;
	}

	prop = of_find_property(np, "synaptics,ubl-byte-delay-us", NULL);
	if (prop && prop->length) {
		retval = of_property_read_u32(np, "synaptics,ubl-byte-delay-us",
				&value);
		if (retval < 0) {
			TS_LOG_ERR(
					"Unable to read synaptics,ubl-byte-delay-us property\n");
			return retval;
		} else {
			bdata->ubl_byte_delay_us = value;
		}
	} else {
		bdata->ubl_byte_delay_us = 0;
	}

	retval =
	    of_property_read_u32(np, "irq_config",
				 &chip_data->irq_config);
	if (retval) {
		TS_LOG_ERR("get irq config failed\n");
	}

	retval =
	    of_property_read_u32(np, "ic_type",
				 &chip_data->ic_type);
	if (retval) {
		TS_LOG_ERR("get device ic_type failed\n");
	}

	retval =
	    of_property_read_u32(np, "vci_gpio_type",
				 &chip_data->vci_gpio_type);
	if (retval) {
		TS_LOG_ERR("get device SYNAPTICS_VCI_GPIO_TYPE failed\n");
	}
	retval =
	    of_property_read_u32(np, "vci_regulator_type",
				 &chip_data->vci_regulator_type);
	if (retval) {
		TS_LOG_ERR("get device SYNAPTICS_VCI_REGULATOR_TYPE failed\n");
	}
	
	retval =
	    of_property_read_u32(np, "vddio_gpio_type",
				 &chip_data->vddio_gpio_type);
	if (retval) {
		TS_LOG_ERR("get device SYNAPTICS_VDDIO_GPIO_TYPE failed\n");
	}
	
	retval =
	    of_property_read_u32(np, "vddio_regulator_type",
				 &chip_data->vddio_regulator_type);
	if (retval) {
		TS_LOG_ERR
		    ("get device SYNAPTICS_VDDIO_REGULATOR_TYPE failed\n");
	}

	retval =
	    of_property_read_u32(np, "vci_value",
				 &chip_data->regulator_ctr.vci_value);
	if (retval) {
		TS_LOG_INFO("Not define Vci value in Dts, use fault value\n");
		chip_data->regulator_ctr.vci_value = 3100000;
	}

	retval =
	    of_property_read_u32(np, "need_set_vddio_value",
				 &chip_data->regulator_ctr.need_set_vddio_value);
	if (retval) {
		TS_LOG_INFO
		    ("Not define need set Vddio value in Dts, use fault value\n");
		chip_data->regulator_ctr.need_set_vddio_value = 0;
	} else {
		retval =
		    of_property_read_u32(np, "vddio_value",
					 &chip_data->regulator_ctr.vddio_value);
		if (retval) {
			TS_LOG_INFO
			    ("Not define Vddio value in Dts, use fault value\n");
			chip_data->regulator_ctr.vddio_value = 1800000;
		}
	}

	retval =
	    of_property_read_u32(np, "x_max", &bdata->x_max);
	if (retval) {
		TS_LOG_ERR("get device x_max failed\n");
		bdata->x_max =0;
	}
	retval =
	    of_property_read_u32(np, "y_max", &bdata->y_max);
	if (retval) {
		TS_LOG_ERR("get device y_max failed\n");
		bdata->y_max =0;
	}
	bdata->y_max =2340;
	retval =
	    of_property_read_u32(np, "x_max_mt", &bdata->x_max_mt);
	if (retval) {
		TS_LOG_ERR("get device x_max failed\n");
		bdata->x_max_mt =0;
	}
	retval =
	    of_property_read_u32(np, "y_max_mt", &bdata->y_max_mt);
	if (retval) {
		TS_LOG_ERR("get device y_max failed\n");
		bdata->y_max_mt =0;
	}
	bdata->y_max_mt =2340;
	retval =
	    of_property_read_u32(np, "max_finger_objects", &bdata->max_finger_objects);
	if (retval) {
		TS_LOG_ERR("get device y_max failed\n");
		bdata->max_finger_objects = 0;
	}

	/*0 is power supplied by gpio, 1 is power supplied by ldo */
	if (1 == chip_data->vci_gpio_type) {
		chip_data->vci_gpio_ctrl =
		    of_get_named_gpio(np, "vci_ctrl_gpio", 0);
		if (!gpio_is_valid(chip_data->vci_gpio_ctrl)) {
			TS_LOG_ERR
			    ("SFT: ok; ASIC: Real err----power gpio is not valid\n");
		}
	}
	if (1 == chip_data->vddio_gpio_type) {
		chip_data->vddio_gpio_ctrl =
		    of_get_named_gpio(np, "vddio_ctrl_gpio", 0);
		if (!gpio_is_valid(chip_data->vddio_gpio_ctrl)) {
			TS_LOG_ERR
			    ("SFT: ok; ASIC: Real err----power gpio is not valid\n");
		}
	}

	return 0;
}

static int syna_tcm_init_chip(void)
{
	int retval = NO_ERR;

//	retval = touch_init(tcm_hcd);
	reflash_init(tcm_hcd);
	retval = debug_device_init(tcm_hcd);

	return retval;
}

static int syna_tcm_chip_detect(struct ts_kit_platform_data* data)
{
	int retval = NO_ERR;
#ifndef HW_BUS_USE_I2C	
	u16 tmp_spi_mode = SPI_MODE_0;
#endif
	TS_LOG_INFO(" syna_tcm_chip_detect called !\n");
	tcm_hcd->syna_tcm_chip_data->ts_platform_data = data;
	tcm_hcd->pdev = data->ts_dev;
	tcm_hcd->pdev->dev.of_node = tcm_hcd->syna_tcm_chip_data->cnode;
	tcm_hcd->reset = syna_tcm_reset;
	tcm_hcd->sleep = syna_tcm_sleep;
	tcm_hcd->identify = syna_tcm_identify;
	tcm_hcd->switch_mode = syna_tcm_switch_mode;
	tcm_hcd->read_message = syna_tcm_read_message;
	tcm_hcd->write_message = syna_tcm_write_message;
	tcm_hcd->get_dynamic_config = syna_tcm_get_dynamic_config;
	tcm_hcd->set_dynamic_config = syna_tcm_set_dynamic_config;
	tcm_hcd->get_data_location = syna_tcm_get_data_location;

	tcm_hcd->rd_chunk_size = RD_CHUNK_SIZE;
	tcm_hcd->wr_chunk_size = WR_CHUNK_SIZE;

#ifdef PREDICTIVE_READING
	tcm_hcd->read_length = MIN_READ_LENGTH;
#else
	tcm_hcd->read_length = MESSAGE_HEADER_SIZE;
#endif

	tcm_hcd->watchdog.run = RUN_WATCHDOG;
	tcm_hcd->update_watchdog = syna_tcm_update_watchdog;
 
	INIT_BUFFER(tcm_hcd->in, false);
	INIT_BUFFER(tcm_hcd->out, false);
	INIT_BUFFER(tcm_hcd->resp, true);
	INIT_BUFFER(tcm_hcd->temp, false);
	INIT_BUFFER(tcm_hcd->config, false);
	INIT_BUFFER(tcm_hcd->report.buffer, true);

	LOCK_BUFFER(tcm_hcd->in);

	retval = syna_tcm_alloc_mem(tcm_hcd,
			&tcm_hcd->in,
			tcm_hcd->read_length + 1);
	if (retval < 0) {
		TS_LOG_ERR(
				"Failed to allocate memory for tcm_hcd->in.buf\n");
		UNLOCK_BUFFER(tcm_hcd->in);
		goto err_alloc_mem;
	}

	UNLOCK_BUFFER(tcm_hcd->in);

	atomic_set(&tcm_hcd->command_status, CMD_IDLE);
	atomic_set(&tcm_hcd->helper.task, HELP_NONE);
	device_init_wakeup(&data->ts_dev->dev, 1);
	init_waitqueue_head(&tcm_hcd->hdl_wq);

	syna_tcm_parse_dts(tcm_hcd->pdev->dev.of_node, tcm_hcd->syna_tcm_chip_data);

	retval = syna_tcm_get_regulator();
	if (retval < 0) {
			TS_LOG_ERR("syna_tcm_get_regulator error %d \n",retval);
		goto err_alloc_mem;
	}

	retval = syna_tcm_gpio_request();
	if (retval < 0) {
            TS_LOG_ERR("syna_tcm_gpio_request error %d \n",retval);
	      goto gpio_err;
	}

	retval = syna_tcm_pinctrl_get_init();
	if (retval < 0) {
            TS_LOG_ERR("syna_tcm_pinctrl_get_init error %d \n",retval);
	     goto pinctrl_get_err;
	}

	/*power up the chip */
	syna_tcm_power_on();

	/*reset the chip */
	syna_tcm_gpio_reset();
	msleep(500);
#ifdef HW_BUS_USE_I2C

#else
	tmp_spi_mode = data->spi->mode;
	data->spi->mode = SPI_MODE_3;
	retval = spi_setup(data->spi);
	if (retval) {
		TS_LOG_ERR("%s spi_setup failed.\n", __func__);
		goto pinctrl_get_err;
	}
#endif	
	retval = syna_tcm_comm_check();
	if (retval < 0) {
		TS_LOG_ERR(
				"Failed to syna_tcm_comm_check\n");
		goto err_comm_check;
	}

	return 0;

err_comm_check:
	syna_tcm_power_off();
	
pinctrl_get_err:
#ifdef HW_BUS_USE_I2C

#else
	data->spi->mode = tmp_spi_mode;
	retval = spi_setup(data->spi);
	if (retval) {
		TS_LOG_ERR("%s spi_setup failed.\n", __func__);
	}
#endif
	syna_tcm_gpio_free();

gpio_err:
	syna_tcm_regulator_put();

err_alloc_mem:
	RELEASE_BUFFER(tcm_hcd->report.buffer);
	RELEASE_BUFFER(tcm_hcd->config);
	RELEASE_BUFFER(tcm_hcd->temp);
	RELEASE_BUFFER(tcm_hcd->resp);
	RELEASE_BUFFER(tcm_hcd->out);
	RELEASE_BUFFER(tcm_hcd->in);
	
//out:
	if(tcm_hcd->syna_tcm_chip_data) {
		kfree(tcm_hcd->syna_tcm_chip_data);
		tcm_hcd->syna_tcm_chip_data = NULL;
	}
	if (tcm_hcd) {
		kfree(tcm_hcd);
		tcm_hcd = NULL;
	}
	TS_LOG_ERR("detect synaptics error\n");
	return retval;
}

static int __init syna_tcm_module_init(void)
{
	int retval = NO_ERR;    
	bool found = false;
	struct device_node* child = NULL;
	struct device_node* root = NULL;

	TS_LOG_INFO(" syna_tcm_ts_module_init called here\n");
	
	root = of_find_compatible_node(NULL, NULL, "huawei,ts_kit");
	if (!root) {
		TS_LOG_ERR("huawei_ts, find_compatible_node huawei,ts_kit error\n");
		retval = -EINVAL;
		goto out;
	}

	for_each_child_of_node(root, child)
	{
		if (of_device_is_compatible(child, SYNAPTICS_VENDER_NAME)) {
			TS_LOG_INFO("found is true\n");
			found = true;
			break;
		} 
	}

	if (!found) {
		TS_LOG_ERR(" not found chip synaptics child node  !\n");
		retval = -EINVAL;
		goto out;
	}

	tcm_hcd = kzalloc(sizeof(*tcm_hcd), GFP_KERNEL);
	if (!tcm_hcd) {
		TS_LOG_ERR("Failed to allocate memory for tcm_hcd\n");
		return -ENOMEM;
	}

	tcm_hcd->syna_tcm_chip_data = kzalloc(sizeof(struct ts_kit_device_data), GFP_KERNEL);
	if (!tcm_hcd->syna_tcm_chip_data) {
		TS_LOG_ERR("Failed to allocate memory for tcm_hcd\n");
		return -ENOMEM;
	}

    tcm_hcd->syna_tcm_chip_data->cnode = child;
    tcm_hcd->syna_tcm_chip_data->ops = &ts_kit_syna_tcm_ops;

	bdata = kzalloc(sizeof(*bdata), GFP_KERNEL);
	if (!bdata) {
		TS_LOG_ERR(
				"Failed to allocate memory for board data\n");
		return -ENOMEM;
	}

	tcm_hcd->bdata = bdata;
#ifdef HW_BUS_USE_I2C
	tcm_hcd->read = syna_tcm_raw_read;
	tcm_hcd->write = syna_tcm_i2c_write;
	tcm_hcd->rmi_read = syna_tcm_i2c_rmi_read;
	tcm_hcd->rmi_write = syna_tcm_i2c_rmi_write;
#else
	tcm_hcd->read = syna_tcm_spi_read;
	tcm_hcd->write = syna_tcm_spi_write;
	tcm_hcd->rmi_read = syna_tcm_spi_rmi_read;
	tcm_hcd->rmi_write = syna_tcm_spi_rmi_write;
#endif
	tcm_hcd->host_download_mode = false;

	retval = huawei_ts_chip_register(tcm_hcd->syna_tcm_chip_data);
	if(retval)
	{
	  TS_LOG_ERR(" synaptics chip register fail !\n");
	  goto out;
	}

	return retval;
	
out:
	if (tcm_hcd->syna_tcm_chip_data)
		kfree(tcm_hcd->syna_tcm_chip_data);
	if (tcm_hcd)
		kfree(tcm_hcd);
	tcm_hcd = NULL;	
	return retval;
}

static void __exit syna_tcm_module_exit(void)
{

	return;
}

late_initcall(syna_tcm_module_init);
module_exit(syna_tcm_module_exit);

MODULE_AUTHOR("Synaptics, Inc.");
MODULE_DESCRIPTION("Synaptics TCM Touch Driver");
MODULE_LICENSE("GPL v2");
