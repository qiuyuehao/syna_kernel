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

#include <linux/gpio.h>
#include "synaptics_tcm_core.h"
#include "synaptics_tcm_testing.h"
#include <linux/interrupt.h>

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

#define SYSFS_DIR_NAME "testing"

#define SYNA_TCM_TEST_BUF_LEN 50
#define TRXNUM_DEFAULT 1024  // > Tx * Rx
#define RES_TIMEOUT_MS 20
#define REPORT_TIMEOUT_MS 1000

#define _TEST_PASS_ 1
#define _TEST_FAIL_ 0

static int rawdata_size;
static char syna_tcm_mmi_test_result[SYNA_TCM_TEST_BUF_LEN] = { 0 };

//extern char synaptics_raw_data_limit_flag;
extern struct ts_kit_platform_data g_ts_kit_platform_data;

enum test_code {
	TEST_TRX_TRX_SHORTS = 0,
	TEST_TRX_SENSOR_OPENS = 1,
	TEST_TRX_GROUND_SHORTS = 2,
	TEST_FULL_RAW_CAP = 5,
	TEST_DYNAMIC_RANGE = 7,
	TEST_OPEN_SHORT_DETECTOR = 8,
	TEST_NOISE = 10,
	TEST_PT11 = 11,
	TEST_PT12 = 12,
	TEST_PT13 = 13,
	TEST_PT18 = 18, //abs raw cap
	TEST_DISCRETE_PT196 = 196,       /* support discrete production test */
};

/* support discrete production test + */
#define MAX_PINS (64)
#define CFG_IMAGE_TXES_OFFSET (3184)
#define CFG_IMAGE_TXES_LENGTH (256)
#define CFG_IMAGE_RXES_OFFSET (2640)
#define CFG_IMAGE_RXES_LENGTH (544)
#define CFG_NUM_TXGUARD_OFFSET (3568)
#define CFG_NUM_TXGUARD_LENGTH (16)
#define CFG_NUM_RXGUARD_OFFSET (3552)
#define CFG_NUM_RXGUARD_LENGTH (16)
#define CFG_TX_GUARD_PINS_OFFSET (6640)
#define CFG_TX_GUARD_PINS_LENGTH (48)
#define CFG_RX_GUARD_PINS_OFFSET (6576)
#define CFG_RX_GUARD_PINS_LENGTH (64)

#define CFG_IMAGE_CBCS_OFFSET (2032)
#define CFG_IMAGE_CBCS_LENGTH (544)
#define CFG_REF_LO_TRANS_CAP_OFFSET (3664)
#define CFG_REF_LO_TRANS_CAP_LENGTH (16)
#define CFG_REF_LO_XMTR_PL_OFFSET (6560)
#define CFG_REF_LO_XMTR_PL_LENGTH (16)
#define CFG_REF_HI_TRANS_CAP_OFFSET (3648)
#define CFG_REF_HI_TRANS_CAP_LENGTH (16)
#define CFG_REF_HI_XMTR_PL_OFFSET (6528)
#define CFG_REF_HI_XMTR_PL_LENGTH (16)
#define CFG_REF_GAIN_CTRL_OFFSET (3632)
#define CFG_REF_GAIN_CTRL_LENGTH (16)

#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))
/* support discrete production test - */

struct testing_hcd {
	int *mmi_buf;
	bool result;
	unsigned char report_type;
	unsigned int report_index;
	unsigned int num_of_reports;
	struct kobject *sysfs_dir;
	struct syna_tcm_buffer out;
	struct syna_tcm_buffer resp;
	struct syna_tcm_buffer report;
	struct syna_tcm_buffer process;
	struct syna_tcm_buffer output;
	struct syna_tcm_hcd *tcm_hcd;

	/* support discrete production test + */
	unsigned char *satic_cfg_buf;
	short tx_pins[MAX_PINS];
	short tx_assigned;
	short rx_pins[MAX_PINS];
	short rx_assigned;
	short guard_pins[MAX_PINS];
	short guard_assigned;
	/* support discrete production test - */
};


static struct testing_hcd *testing_hcd;
/*
static ssize_t testing_sysfs_raw_data_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t testing_sysfs_testing_noise_show(struct device *dev,
		struct device_attribute *attr, char *buf);

SHOW_PROTOTYPE(testing, raw_data)
SHOW_PROTOTYPE(testing, testing_noise)
SHOW_PROTOTYPE(testing, testing_open_short)


static struct device_attribute *attrs[] = {
	ATTRIFY(raw_data),
	ATTRIFY(testing_noise),
	ATTRIFY(testing_open_short),
};
*/

extern int reflash_after_fw_update_do_reset(void);
static void testing_get_frame_size_words(unsigned int *size, bool image_only)
{
	unsigned int rows;
	unsigned int cols;
	unsigned int hybrid;
	unsigned int buttons;
	struct syna_tcm_app_info *app_info;
	struct syna_tcm_hcd *tcm_hcd = testing_hcd->tcm_hcd;

	app_info = &tcm_hcd->app_info;

	rows = le2_to_uint(app_info->num_of_image_rows);
	cols = le2_to_uint(app_info->num_of_image_cols);
	hybrid = le2_to_uint(app_info->has_hybrid_data);
	buttons = le2_to_uint(app_info->num_of_buttons);

	*size = rows * cols;

	if (!image_only) {
		if (hybrid)
			*size += rows + cols;
		*size += buttons;
	}

	return;
}

static int testing_open_short(void)
{
	int retval;
	unsigned int timeout;
	unsigned int data_length;
	signed short data;
	unsigned int idx;
	unsigned int row;
	unsigned int col;
	unsigned int rows;
	unsigned int cols;
	unsigned int limits_rows;
	unsigned int limits_cols;
	unsigned int frame_size_words;
	struct syna_tcm_app_info *app_info;
	struct syna_tcm_hcd *tcm_hcd = testing_hcd->tcm_hcd;
	unsigned char *temp_buf;

	if (tcm_hcd->id_info.mode != MODE_APPLICATION ||
			tcm_hcd->app_status != APP_STATUS_OK) {
		return -ENODEV;
	}

	app_info = &tcm_hcd->app_info;

	rows = le2_to_uint(app_info->num_of_image_rows);
	cols = le2_to_uint(app_info->num_of_image_cols);

	if (rows * cols > TRXNUM_DEFAULT) {
		kfree(testing_hcd->mmi_buf);
		testing_hcd->mmi_buf = kzalloc(rows * cols * sizeof(signed short), GFP_KERNEL);
		if (!testing_hcd->mmi_buf) {
			TS_LOG_ERR(" Failed to alloc mmi_buf\n");
			return -ENOMEM;
		}		
	}

	temp_buf = kzalloc(((rows * cols + MESSAGE_HEADER_SIZE) * 2), GFP_KERNEL);
	if (!temp_buf) {
		TS_LOG_ERR(
				"Failed to allocate memory for temp_buf\n");
		return -ENOMEM;
	}

	retval = syna_tcm_alloc_mem(tcm_hcd,
			&testing_hcd->out,
			1);
	if (retval < 0) {
		TS_LOG_ERR(
				"Failed to allocate memory for testing_hcd->out.buf\n");
		goto exit;
	}

	testing_hcd->out.buf[0] = TEST_PT11;

	retval = syna_tcm_write_hdl_message(tcm_hcd,
			CMD_PRODUCTION_TEST,
			testing_hcd->out.buf,
			1,
			&testing_hcd->resp.buf,
			&testing_hcd->resp.buf_size,
			&testing_hcd->resp.data_length,
			NULL,
			0);
	if (retval < 0) {
		TS_LOG_ERR(
				"Failed to write command %s\n",
				STR(CMD_PRODUCTION_TEST));
		goto exit;
	}

	
	timeout = REPORT_TIMEOUT_MS * 1.8;	

	msleep(timeout);
	
	retval = syna_tcm_read(tcm_hcd,
			temp_buf,
			((rows * cols + MESSAGE_HEADER_SIZE) * 2));
	if (retval < 0) {
		TS_LOG_ERR("Failed to read raw data\n");
		goto exit;
	}		

	if (temp_buf[0] != MESSAGE_MARKER) {
		TS_LOG_ERR("incorrect Header Marker!");
		return -EINVAL;

	}

	data_length = temp_buf[2] | temp_buf[3] << 8;

	testing_get_frame_size_words(&frame_size_words, false);

	if (frame_size_words != data_length / 2) {
		TS_LOG_ERR(
				"Frame size mismatch\n");
		goto exit;
	}


	testing_hcd->mmi_buf = (int *)(&temp_buf[MESSAGE_HEADER_SIZE]);
	/***  testing_hcd->mmi_buf is output ***/

	limits_rows = sizeof(pt11_hi_limits) / sizeof(pt11_hi_limits[0]);
	limits_cols = sizeof(pt11_hi_limits[0]) / sizeof(pt11_hi_limits[0][0]);

	if (rows > limits_rows || cols > limits_cols) {
		TS_LOG_ERR(
				"Mismatching limits data\n");
		goto exit;
	}

	idx =0;
	testing_hcd->result = true;
	
	for (row = 0; row < rows; row++) {
		for (col = 0; col < cols; col++) {
			data = (signed short)le2_to_uint(&temp_buf[idx * 2 + MESSAGE_HEADER_SIZE]);
			if (data > pt11_hi_limits[row][col] ||
					data < pt11_lo_limits[row][col]) {
				TS_LOG_INFO("overlow_data = %8u, row = %d, col = %d\n", data, row, col);
				testing_hcd->result = false;
				strncat(syna_tcm_mmi_test_result, "3F-", MAX_STR_LEN);
				goto exit;
			}
			idx++;
		}
	}

	strncat(syna_tcm_mmi_test_result, "3P-", MAX_STR_LEN);

exit:
	kfree(temp_buf);
	return retval;
}

static int testing_noise(void)
{
	int retval;
	unsigned int idx; 
	signed short data;
	unsigned int timeout;
	unsigned int data_length;
	unsigned int row;
	unsigned int col;
	unsigned int rows;
	unsigned int cols;
	unsigned int limits_rows;
	unsigned int limits_cols;
	unsigned int frame_size_words;
	struct syna_tcm_app_info *app_info;
	struct syna_tcm_hcd *tcm_hcd = testing_hcd->tcm_hcd;
	unsigned char *temp_buf;

	if (tcm_hcd->id_info.mode != MODE_APPLICATION ||
			tcm_hcd->app_status != APP_STATUS_OK) {
		return -ENODEV;
	}

	app_info = &tcm_hcd->app_info;
	rows = le2_to_uint(app_info->num_of_image_rows);
	cols = le2_to_uint(app_info->num_of_image_cols);

	temp_buf = kzalloc(((rows * cols + MESSAGE_HEADER_SIZE) * 2), GFP_KERNEL);
	if (!temp_buf) {
		TS_LOG_ERR(
				"Failed to allocate memory for temp_buf\n");
		return -ENOMEM;
	}

	retval = syna_tcm_alloc_mem(tcm_hcd,
			&testing_hcd->out,
			1);
	if (retval < 0) {
		TS_LOG_ERR(
				"Failed to allocate memory for testing_hcd->out.buf\n");
		goto exit;
	}

	testing_hcd->out.buf[0] = TEST_NOISE;

	retval = syna_tcm_write_hdl_message(tcm_hcd,
			CMD_PRODUCTION_TEST,
			testing_hcd->out.buf,
			1,
			&testing_hcd->resp.buf,
			&testing_hcd->resp.buf_size,
			&testing_hcd->resp.data_length,
			NULL,
			0);
	if (retval < 0) {
		TS_LOG_ERR(
				"Failed to write command %s\n",
				STR(CMD_PRODUCTION_TEST));
		goto exit;
	}
	
	//timeout = REPORT_TIMEOUT_MS;
	timeout = 100;

	msleep(timeout);
	retval = syna_tcm_read(tcm_hcd,
			temp_buf,
			((rows * cols + MESSAGE_HEADER_SIZE) * 2));
	if (retval < 0) {
		TS_LOG_ERR("Failed to read raw data\n");
		goto exit;
	}		

	if (temp_buf[0] != MESSAGE_MARKER) {
		TS_LOG_ERR("incorrect Header Marker!");
		retval = -EINVAL;
		goto exit;

	}

	data_length = temp_buf[2] | temp_buf[3] << 8;
	TS_LOG_INFO("%d\n", data_length);

	testing_get_frame_size_words(&frame_size_words, true);

	if (frame_size_words != data_length / 2) {
		TS_LOG_ERR(
				"Frame size mismatch\n");
		retval = -EINVAL;
		goto exit;
	}


	testing_hcd->mmi_buf = (int *)(&temp_buf[MESSAGE_HEADER_SIZE]);
	/***  testing_hcd->mmi_buf is output ***/

	limits_rows = sizeof(noise_limits) / sizeof(noise_limits[0]);
	limits_cols = sizeof(noise_limits[0]) / sizeof(noise_limits[0][0]);
if (0) {
	if (rows > limits_rows || cols > limits_cols) {
		TS_LOG_ERR(
				"Mismatching limits data\n");
		retval = -EINVAL;
		goto exit;
	}
}
	idx = 0;
	testing_hcd->result = true;

	//TS_LOG_ERR("noise data begin\n");
	for (row = 0; row < rows; row++) {
		for (col = 0; col < cols; col++) {
			data = (signed short)le2_to_uint(&temp_buf[idx * 2 + MESSAGE_HEADER_SIZE]);
			TS_LOG_ERR("noise data[%d]: %d,   %x , %x ", idx, data, temp_buf[idx * 2 + MESSAGE_HEADER_SIZE], temp_buf[idx * 2 + MESSAGE_HEADER_SIZE + 1]);
			/*if (data > noise_limits[row][col]) {
				TS_LOG_INFO("overlow_data = %8u, row = %d, col = %d\n", data, row, col);
				testing_hcd->result = false;                  
				strncat(syna_tcm_mmi_test_result, "2F-", MAX_STR_LEN);
				goto exit;
			}*/
			idx++;
		}
	}
	//TS_LOG_ERR("noise data end\n");
	strncat(syna_tcm_mmi_test_result, "2P-", MAX_STR_LEN);

exit:
	kfree(temp_buf);
	return retval;
}

static unsigned char temp_buf[2000];
static unsigned char temp_buf_2[2000];
static int testing_raw_data(struct ts_rawdata_info *info)
{
	int retval;
	unsigned int idx;
	signed short data;
	unsigned int data_length;
	unsigned int timeout;
	unsigned int row;
	unsigned int col;
	unsigned int rows;
	unsigned int cols;
	unsigned char res_buf[4];
	
	struct syna_tcm_app_info *app_info;
	struct syna_tcm_hcd *tcm_hcd = testing_hcd->tcm_hcd;

	TS_LOG_INFO("testing_raw_data called\n");

	testing_hcd->report_index = 0;
	testing_hcd->report_type = REPORT_RAW;
	testing_hcd->num_of_reports = 1;

	app_info = &tcm_hcd->app_info;
	rows = le2_to_uint(app_info->num_of_image_rows);
	cols = le2_to_uint(app_info->num_of_image_cols);

	TS_LOG_DEBUG("rows & cols = %d, %d\n", rows, cols);
	if (rows * cols > TRXNUM_DEFAULT) {
		kfree(testing_hcd->mmi_buf);
		testing_hcd->mmi_buf = (int *)kzalloc(rows * cols * sizeof(int), GFP_KERNEL);
		if (!testing_hcd->mmi_buf) {
			TS_LOG_ERR(" Failed to alloc mmi_buf\n");
			return -ENOMEM;
		}		
	}

	memset(temp_buf, 0x00, sizeof(temp_buf));
	
	retval = syna_tcm_alloc_mem(tcm_hcd,
			&testing_hcd->out,
			1);
	if (retval < 0) {
		TS_LOG_ERR(
				"Failed to allocate memory for testing_hcd->out.buf\n");
		goto exit;
	}

	testing_hcd->out.buf[0] = testing_hcd->report_type;

	retval = syna_tcm_write_hdl_message(tcm_hcd,
			CMD_ENABLE_REPORT,
			testing_hcd->out.buf,
			1,
			&testing_hcd->resp.buf,
			&testing_hcd->resp.buf_size,
			&testing_hcd->resp.data_length,
			NULL,
			0);
	if (retval < 0) {
		TS_LOG_ERR(
				"Failed to write command %s\n",
				STR(CMD_ENABLE_REPORT));
		goto exit;
	}

	timeout = RES_TIMEOUT_MS;
	
	msleep(timeout);

	/* read out the response*/
	retval = syna_tcm_read(tcm_hcd,
			res_buf,
			sizeof(res_buf));
	if (retval < 0) {
		TS_LOG_ERR("Failed to res_buf data\n");
		return retval;
	}	

	timeout = REPORT_TIMEOUT_MS * testing_hcd->num_of_reports;

	msleep(timeout);
	
	retval = syna_tcm_read(tcm_hcd,
			temp_buf,
			sizeof(temp_buf));
	if (retval < 0) {
		TS_LOG_ERR("Failed to read raw data\n");
		return retval;
	}		

	if (temp_buf[0] != MESSAGE_MARKER) {
		TS_LOG_ERR("incorrect Header Marker!");
		return -EINVAL;

	}

	data_length = temp_buf[2] | temp_buf[3] << 8;
	TS_LOG_INFO("%d\n", data_length);

	testing_hcd->out.buf[0] = testing_hcd->report_type;
	retval = syna_tcm_write_hdl_message(tcm_hcd,
			CMD_DISABLE_REPORT,
			testing_hcd->out.buf,
			1,
			&testing_hcd->resp.buf,
			&testing_hcd->resp.buf_size,
			&testing_hcd->resp.data_length,
			NULL,
			0);
	if (retval < 0) {
		TS_LOG_ERR(
				"Failed to write command %s\n",
				STR(CMD_DISABLE_REPORT));
		goto exit;
	}
	
	timeout = RES_TIMEOUT_MS;

	msleep(timeout);

	/* read out the response*/
	retval = syna_tcm_read(tcm_hcd,
			res_buf,
			sizeof(res_buf));
	if (retval < 0) {
		TS_LOG_ERR("Failed to res_buf data\n");
		return retval;
	}
	
	info->buff[0] = cols;
	info->buff[1] = rows;

	idx = 0;
	printk("\n");
	for (row = 0; row < rows; row++) {
		for (col = 0; col < cols; col++) {
			data = (signed short)le2_to_uint(&temp_buf[idx * 2 + MESSAGE_HEADER_SIZE]);
			info->buff[idx + 2] = data;
			printk("%d, ", data);
			idx++;
		}	
		printk("\n");
	}

	idx = 0;
	/*
	for (row = 0; row < rows; row++) {
		for (col = 0; col < cols; col++) {
			data = (signed short)le2_to_uint(&temp_buf[idx * 2 + MESSAGE_HEADER_SIZE]);
			if (data < raw_lo_limits[row][col] ||
				data > raw_hi_limits[row][col]) {
				TS_LOG_INFO("overlow_data = %8u, row = %d, col = %d\n", data, row, col);
				testing_hcd->result = false;
				strncat(syna_tcm_mmi_test_result, "1F-", MAX_STR_LEN);
				goto exit;
			}
			idx++;
		}
	}	
*/
	strncat(syna_tcm_mmi_test_result, "1P-", MAX_STR_LEN);

exit:
	testing_hcd->report_type = 0;

	return retval;
}

static int testing_full_raw_cap(void)
{
	int retval;
	unsigned int idx; 
	signed short data;
	unsigned int timeout;
	unsigned int data_length;
	unsigned int row;
	unsigned int col;
	unsigned int rows;
	unsigned int cols;
	unsigned int limits_rows;
	unsigned int limits_cols;
	unsigned int frame_size_words;
	struct syna_tcm_app_info *app_info;
	struct syna_tcm_hcd *tcm_hcd = testing_hcd->tcm_hcd;
	unsigned char *temp_buf;

	if (tcm_hcd->id_info.mode != MODE_APPLICATION ||
			tcm_hcd->app_status != APP_STATUS_OK) {
		return -ENODEV;
	}

	app_info = &tcm_hcd->app_info;
	rows = le2_to_uint(app_info->num_of_image_rows);
	cols = le2_to_uint(app_info->num_of_image_cols);

	temp_buf = kzalloc(((rows * cols + MESSAGE_HEADER_SIZE) * 2), GFP_KERNEL);
	if (!temp_buf) {
		TS_LOG_ERR(
				"Failed to allocate memory for temp_buf\n");
		return -ENOMEM;
	}

	retval = syna_tcm_alloc_mem(tcm_hcd,
			&testing_hcd->out,
			1);
	if (retval < 0) {
		TS_LOG_ERR(
				"Failed to allocate memory for testing_hcd->out.buf\n");
		goto exit;
	}

	testing_hcd->out.buf[0] = TEST_FULL_RAW_CAP;

	retval = syna_tcm_write_hdl_message(tcm_hcd,
			CMD_PRODUCTION_TEST,
			testing_hcd->out.buf,
			1,
			&testing_hcd->resp.buf,
			&testing_hcd->resp.buf_size,
			&testing_hcd->resp.data_length,
			NULL,
			0);
	if (retval < 0) {
		TS_LOG_ERR(
				"Failed to write command %s\n",
				STR(CMD_PRODUCTION_TEST));
		goto exit;
	}
	
	//timeout = REPORT_TIMEOUT_MS;
	timeout = 100;

	msleep(timeout);
	retval = syna_tcm_read(tcm_hcd,
			temp_buf,
			((rows * cols + MESSAGE_HEADER_SIZE) * 2));
	if (retval < 0) {
		TS_LOG_ERR("Failed to read raw data\n");
		goto exit;
	}		

	if (temp_buf[0] != MESSAGE_MARKER) {
		TS_LOG_ERR("incorrect Header Marker!");
		retval = -EINVAL;
		goto exit;

	}

	data_length = temp_buf[2] | temp_buf[3] << 8;
	TS_LOG_INFO("%d\n", data_length);

	testing_get_frame_size_words(&frame_size_words, true);

	if (frame_size_words != data_length / 2) {
		TS_LOG_ERR(
				"Frame size mismatch\n");
		retval = -EINVAL;
		goto exit;
	}


	testing_hcd->mmi_buf = (int *)(&temp_buf[MESSAGE_HEADER_SIZE]);
	/***  testing_hcd->mmi_buf is output ***/

	limits_rows = sizeof(discrete_full_raw_hi_limits) / sizeof(discrete_full_raw_hi_limits[0]);
	limits_cols = sizeof(discrete_full_raw_hi_limits[0]) / sizeof(discrete_full_raw_hi_limits[0][0]);

	if (rows > limits_rows || cols > limits_cols) {
		TS_LOG_ERR(
				"Mismatching limits data\n");
		retval = -EINVAL;
		goto exit;
	}

	idx = 0;
	testing_hcd->result = true;

	TS_LOG_ERR("full raw data begin\n");
	for (row = 0; row < rows; row++) {
		for (col = 0; col < cols; col++) {
			data = (signed short)le2_to_uint(&temp_buf[idx * 2 + MESSAGE_HEADER_SIZE]);
			TS_LOG_ERR("data[%d]: %d, ", idx, data);
			if ((data > discrete_full_raw_hi_limits[row][col]) || (data < discrete_full_raw_lo_limits[row][col])) {
				TS_LOG_ERR("overlow_data = %8u, row = %d, col = %d\n", data, row, col);
				testing_hcd->result = false;
				retval = -EINVAL;
				strncat(syna_tcm_mmi_test_result, "2F-", MAX_STR_LEN);
				goto exit;
			}
			idx++;
		}
	}
	TS_LOG_ERR("full raw data end\n");

	strncat(syna_tcm_mmi_test_result, "2P-", MAX_STR_LEN);

exit:
	kfree(temp_buf);
	return retval;
}



static int testing_abs_raw_cap(void)
{
	int retval;
	unsigned int idx; 
	unsigned int timeout;
	unsigned int data_length;
	unsigned int row;
	unsigned int col;
	unsigned int rows;
	unsigned int cols;
	unsigned int limits_rows;
	unsigned int limits_cols;
	unsigned int frame_size_words;
	struct syna_tcm_app_info *app_info;
	struct syna_tcm_hcd *tcm_hcd = testing_hcd->tcm_hcd;
	unsigned char *temp_buf;
	unsigned char *temp_data_buf;

	if (tcm_hcd->id_info.mode != MODE_APPLICATION ||
			tcm_hcd->app_status != APP_STATUS_OK) {
		return -ENODEV;
	}

	app_info = &tcm_hcd->app_info;
	rows = le2_to_uint(app_info->num_of_image_rows);
	cols = le2_to_uint(app_info->num_of_image_cols);

	temp_buf = kzalloc(((rows + cols) * 4 + MESSAGE_HEADER_SIZE + 1), GFP_KERNEL);
	if (!temp_buf) {
		TS_LOG_ERR(
				"Failed to allocate memory for temp_buf\n");
		return -ENOMEM;
	}

	temp_data_buf = kzalloc((rows + cols) * 4, GFP_KERNEL);
	if (!temp_data_buf) {
		TS_LOG_ERR(
				"Failed to allocate memory for temp_data_buf\n");
		return -ENOMEM;
	}
	
	retval = syna_tcm_alloc_mem(tcm_hcd,
			&testing_hcd->out,
			1);
	if (retval < 0) {
		TS_LOG_ERR(
				"Failed to allocate memory for testing_hcd->out.buf\n");
		goto exit;
	}

	testing_hcd->out.buf[0] = TEST_PT18;

	retval = syna_tcm_write_hdl_message(tcm_hcd,
			CMD_PRODUCTION_TEST,
			testing_hcd->out.buf,
			1,
			&testing_hcd->resp.buf,
			&testing_hcd->resp.buf_size,
			&testing_hcd->resp.data_length,
			NULL,
			0);
	if (retval < 0) {
		TS_LOG_ERR(
				"Failed to write command %s\n",
				STR(CMD_PRODUCTION_TEST));
		goto exit;
	}
	
	//timeout = REPORT_TIMEOUT_MS;
	timeout = 100;

	msleep(timeout);
	retval = syna_tcm_read(tcm_hcd,
			temp_buf,
			((rows + cols) * 4 + MESSAGE_HEADER_SIZE + 1));
	if (retval < 0) {
		TS_LOG_ERR("Failed to read raw data\n");
		goto exit;
	}		

	if (temp_buf[0] != MESSAGE_MARKER) {
		TS_LOG_ERR("incorrect Header Marker!");
		retval = -EINVAL;
		goto exit;

	}

	data_length = temp_buf[2] | temp_buf[3] << 8;
	TS_LOG_ERR("%d\n", data_length);

	if ((temp_buf[1] != STATUS_OK) || (data_length != ((rows + cols) * 4))) {
		TS_LOG_ERR("not correct status or data_length status:%x  length:%x\n", temp_buf[1], data_length);
		retval = -EINVAL;
		goto exit;
	}

if (0) {
	testing_get_frame_size_words(&frame_size_words, true);

	if (frame_size_words != data_length / 2) {
		TS_LOG_ERR(
				"Frame size mismatch\n");
		retval = -EINVAL;
		goto exit;
	}
}

	testing_hcd->mmi_buf = (int *)(&temp_buf[MESSAGE_HEADER_SIZE]);
	/***  testing_hcd->mmi_buf is output ***/

	limits_rows = sizeof(discrete_abs_row_limits) / sizeof(discrete_abs_row_limits[0]);
	limits_cols = sizeof(discrete_abs_col_limits) / sizeof(discrete_abs_col_limits[0]);

	if (rows > limits_rows || cols > limits_cols) {
		TS_LOG_ERR(
				"Mismatching limits data\n");
		retval = -EINVAL;
		goto exit;
	}

	idx = MESSAGE_HEADER_SIZE;
	testing_hcd->result = true;

	for (col = 0; col < cols; col++) {
			temp_data_buf[col] = (unsigned int)(temp_buf[idx] & 0xff) | (unsigned int)(temp_buf[idx+1] << 8) |
						(unsigned int)(temp_buf[idx+2] << 16) | (unsigned int)(temp_buf[idx+3] << 24);
						
			if (temp_data_buf[col] > discrete_abs_col_limits[col]) {
				testing_hcd->result = false;
				retval = -EINVAL;
				TS_LOG_ERR("fail at col=%-2d data = %d (0x%2x 0x%2x 0x%2x 0x%2x), limit = %d\n",
						col, temp_data_buf[col], temp_buf[idx], temp_buf[idx+1], temp_buf[idx+2], temp_buf[idx+3], discrete_abs_col_limits[col]);
			}
			idx+=4;
	}

	for (row = 0; row < rows; row++) {
			temp_data_buf[cols + row] = (unsigned int)(temp_buf[idx] & 0xff) | (unsigned int)(temp_buf[idx+1] << 8) |
						(unsigned int)(temp_buf[idx+2] << 16) | (unsigned int)(temp_buf[idx+3] << 24);
			
			if (temp_data_buf[cols + row] > discrete_abs_row_limits[row]) {
				testing_hcd->result = false;
				retval = -EINVAL;
				TS_LOG_ERR("fail at row=%-2d data = %d (0x%2x 0x%2x 0x%2x 0x%2x), limit = %d\n",
						row, temp_data_buf[cols + row], temp_buf[idx], temp_buf[idx+1], temp_buf[idx+2], temp_buf[idx+3], discrete_abs_row_limits[row]);
			}
			idx+=4;
	}




	strncat(syna_tcm_mmi_test_result, "2P-", MAX_STR_LEN);

exit:
	kfree(temp_buf);
	kfree(temp_data_buf);
	return retval;
}

static int testing_discrete_ex_trx_short(void);
int syna_tcm_testing(struct ts_rawdata_info *info)
{
	int retval = NO_ERR;
	unsigned int rows;
	unsigned int cols;
	struct syna_tcm_hcd *tcm_hcd = testing_hcd->tcm_hcd;
	struct syna_tcm_app_info *app_info;
	
	TS_LOG_INFO("syna_tcm_testing called\n");

	testing_hcd->tcm_hcd = tcm_hcd;
	app_info = &tcm_hcd->app_info;
	rows = le2_to_uint(app_info->num_of_image_rows);
	cols = le2_to_uint(app_info->num_of_image_cols);

	if (tcm_hcd->id_info.mode != MODE_APPLICATION ||
		tcm_hcd->app_status != APP_STATUS_OK) {
			memcpy(syna_tcm_mmi_test_result, "0F-1F-2F",
		       (strlen("0F-1F-2F-3F") + 1));
		return -ENODEV;
	} else {
			memcpy(syna_tcm_mmi_test_result, "0P-", (strlen("0P-") + 1));
	}
	if (0) {

		retval = testing_raw_data(info);
		if (retval < 0) {
			strncat(syna_tcm_mmi_test_result, "1F-", MAX_STR_LEN);
		}
	}
	retval = testing_full_raw_cap();
	if (retval < 0) {
		TS_LOG_ERR("fail to do full raw cap");
	}


	msleep(100);
	retval = testing_noise();
	if (retval < 0) {
		strncat(syna_tcm_mmi_test_result, "2F-", MAX_STR_LEN);
	}
	info->hybrid_buff[0] = rows;
	info->hybrid_buff[1] = cols;
	memcpy(&info->hybrid_buff[2], testing_hcd->mmi_buf, rows * cols * sizeof(int));

	memcpy(&info->buff[2 + rows * cols], testing_hcd->mmi_buf, rows * cols * sizeof(int));	


	retval = testing_discrete_ex_trx_short();
	if (retval < 0) {
		TS_LOG_ERR("fail to do testing_discrete_ex_trx_short");
	} else {
		TS_LOG_ERR("ok to do testing_discrete_ex_trx_short");
	}


	retval = testing_abs_raw_cap();
	if (retval < 0) {
		TS_LOG_ERR("fail to do abs raw cap");
	}
	info->buff[0] = rows;
	info->buff[1] = cols;
	//memcpy(&info->buff[2], testing_hcd->mmi_buf, rows * cols * sizeof(int));

if (0) {
	msleep(100);
	
	retval = testing_open_short();
	if (retval < 0) {
		strncat(syna_tcm_mmi_test_result, "3F-", MAX_STR_LEN);
	}
}
	memcpy(info->result, syna_tcm_mmi_test_result, strlen(syna_tcm_mmi_test_result));
	info->used_size = rows * cols + 2;
	info->used_size = rawdata_size;
	TS_LOG_INFO("info->used_size = %d\n", info->used_size);

	return 0;
}
/*
static ssize_t testing_sysfs_testing_open_short_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	disable_irq_nosync(g_ts_kit_platform_data.irq_id);
	TS_LOG_INFO("---testing_sysfs_testing_open_short_show---\n");
	testing_open_short();
	TS_LOG_INFO("+++testing_sysfs_testing_open_short_show+++\n");
	TS_LOG_INFO("%s\n", syna_tcm_mmi_test_result);
	enable_irq(g_ts_kit_platform_data.irq_id);
	return snprintf(buf, PAGE_SIZE, "_testing_open_short_%s\n", (testing_hcd->result == _TEST_PASS_) ? "PASS" : "FAIL");
}

static ssize_t testing_sysfs_testing_noise_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	disable_irq_nosync(g_ts_kit_platform_data.irq_id);
	TS_LOG_INFO("---testing_sysfs_testing_noise_show---\n");
	testing_noise();
	TS_LOG_INFO("+++testing_sysfs_testing_noise_show+++\n");
	TS_LOG_INFO("%s\n", syna_tcm_mmi_test_result);
	enable_irq(g_ts_kit_platform_data.irq_id);
	return snprintf(buf, PAGE_SIZE, "_testing_noise_%s\n", (testing_hcd->result == _TEST_PASS_) ? "PASS" : "FAIL");
}
static ssize_t testing_sysfs_raw_data_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	disable_irq_nosync(g_ts_kit_platform_data.irq_id);
	TS_LOG_INFO("---testing_sysfs_raw_data_show---\n");
	testing_raw_data();
	TS_LOG_INFO("+++testing_sysfs_raw_data_show+++\n");
	TS_LOG_INFO("%s\n", syna_tcm_mmi_test_result);
	enable_irq(g_ts_kit_platform_data.irq_id);

	return snprintf(buf, PAGE_SIZE, "_raw_data_%s\n", (testing_hcd->result == _TEST_PASS_) ? "PASS" : "FAIL");
}
*/

static bool testing_get_pins_assigned(unsigned short pin,
		short *tx_pins, short tx_assigned, short *rx_pins, short rx_assigned,
		short *guard_pins, short guard_assigned)
{
	int i;

	for (i = 0; i < tx_assigned; i++) {
		if (pin == tx_pins[i]) {
			return true;
		}
    }
	for (i = 0; i < rx_assigned; i++) {
		if (pin == rx_pins[i]) {
			return true;
		}
    }
	for (i = 0; i < guard_assigned; i++) {
		if (pin == guard_pins[i]) {
			return true;
		}
    }

    return false;
}

static int testing_pins_mapping(unsigned char *cfg_data, unsigned int cfg_data_len)
{
	int i, j;
	int idx;
	int offset_rx_pin = CFG_IMAGE_RXES_OFFSET/8;
	int length_rx_pin = CFG_IMAGE_RXES_LENGTH/8;
	int offset_tx_pin = CFG_IMAGE_TXES_OFFSET/8;
	int length_tx_pin = CFG_IMAGE_TXES_LENGTH/8;
	int num_rx_guard;
	int offset_num_rx_guard= CFG_NUM_RXGUARD_OFFSET/8;
	int length_num_rx_guard= CFG_NUM_RXGUARD_LENGTH/8;
	int offset_rx_guard= CFG_RX_GUARD_PINS_OFFSET/8;
	int length_rx_guard= CFG_RX_GUARD_PINS_LENGTH/8;
	int num_tx_guard;
	int offset_num_tx_guard= CFG_NUM_TXGUARD_OFFSET/8;
	int length_num_tx_guard= CFG_NUM_TXGUARD_LENGTH/8;
	int offset_tx_guard= CFG_TX_GUARD_PINS_OFFSET/8;
	int length_tx_guard= CFG_TX_GUARD_PINS_LENGTH/8;

	if (!cfg_data) {
		TS_LOG_ERR("invalid parameter\n");
		return -EINVAL;
	}

	testing_hcd->tx_assigned = 0;
	testing_hcd->rx_assigned = 0;
	testing_hcd->guard_assigned = 0;

	/* get tx pins mapping */
	if (cfg_data_len > offset_tx_pin + length_tx_pin) {

		testing_hcd->tx_assigned = (length_tx_pin/2);

		idx = 0;
		for (i = 0; i < (length_tx_pin/2); i++) {
			testing_hcd->tx_pins[i] = (short)cfg_data[offset_tx_pin + idx] |
									(short)(cfg_data[offset_tx_pin + idx + 1] << 8);
			idx += 2;

			TS_LOG_ERR("tx[%d] = %2d\n", i, testing_hcd->tx_pins[i]);
		}
	}

	/* get rx pins mapping */
	if (cfg_data_len > offset_rx_pin + length_rx_pin) {

		testing_hcd->rx_assigned = (length_rx_pin/2);

		idx = 0;
		for (i = 0; i < (length_rx_pin/2); i++) {
			testing_hcd->rx_pins[i] = (short)cfg_data[offset_rx_pin + idx] |
									(short)(cfg_data[offset_rx_pin + idx + 1] << 8);
			idx += 2;

			TS_LOG_ERR("rx[%d] = %2d\n", i, testing_hcd->rx_pins[i]);
		}
	}

	/* get number of tx guards */
	if (cfg_data_len > offset_num_tx_guard + length_num_tx_guard) {

		num_tx_guard = (short)cfg_data[offset_num_tx_guard] |
						(short)(cfg_data[offset_num_tx_guard + 1] << 8);

		testing_hcd->guard_assigned += num_tx_guard;
	}

	/* get number of rx guards */
	if (cfg_data_len > offset_num_rx_guard + length_num_rx_guard) {

		num_rx_guard = (short)cfg_data[offset_num_rx_guard] |
						(short)(cfg_data[offset_num_rx_guard + 1] << 8);

		testing_hcd->guard_assigned += num_rx_guard;
	}

	if (testing_hcd->guard_assigned > 0) {

		TS_LOG_ERR("num of guards = %2d (tx: %d, rx: %d)\n",
					testing_hcd->guard_assigned, num_tx_guard, num_rx_guard);

		j = 0;
	}

	/* get tx guards mapping */
	if ((num_tx_guard > 0) &&
		(cfg_data_len > offset_tx_guard + length_tx_guard)) {
		idx = 0;
		for (i = 0; i < num_tx_guard; i++) {
			testing_hcd->guard_pins[j] = (short)cfg_data[offset_tx_guard + idx] |
										(short)(cfg_data[offset_tx_guard + idx + 1] << 8);

			TS_LOG_ERR("guard_pins[%d] = %2d\n", i, testing_hcd->guard_pins[j]);
			idx += 2;
			j += 1;

		}
	}

	/* get rx guards mapping */
	if ((num_rx_guard > 0) &&
		(cfg_data_len > offset_rx_guard + length_rx_guard)) {
		for (i = 0; i < num_rx_guard; i++) {
			testing_hcd->guard_pins[j] = (short)cfg_data[offset_rx_guard + idx] |
										(short)(cfg_data[offset_rx_guard + idx + 1] << 8);

			TS_LOG_ERR("guard_pins[%d] = %2d\n", i, testing_hcd->guard_pins[j]);
			idx += 2;
			j += 1;
		}
	}

	return 0;
}

static int testing_get_static_config(unsigned char *buf, unsigned int buf_len)
{
	int retval;
	struct syna_tcm_hcd *tcm_hcd = testing_hcd->tcm_hcd;

	if (!buf) {
		TS_LOG_ERR("invalid parameter\n");
		return -EINVAL;
	}

	retval = syna_tcm_write_hdl_message(tcm_hcd,
			CMD_GET_STATIC_CONFIG,
			NULL,
			0,
			&testing_hcd->resp.buf,
			&testing_hcd->resp.buf_size,
			&testing_hcd->resp.data_length,
			NULL,
			0);
	if (retval < 0) {
		TS_LOG_ERR("Failed to write command %s\n",
				STR(CMD_GET_STATIC_CONFIG));
		goto exit;
	}
	
	msleep(50);
	retval = syna_tcm_read(tcm_hcd,
		temp_buf,
		sizeof(temp_buf));
	
	if (retval < 0) {
		TS_LOG_ERR("Failed to temp_buf data\n");
		return retval;
	}

	if ((temp_buf[0] != MESSAGE_MARKER) || (temp_buf[1] != STATUS_OK)) {
		TS_LOG_ERR("message error %x, %x %x %x\n", temp_buf[0], temp_buf[1], temp_buf[2], temp_buf[3]);
		retval = -EINVAL;
		goto exit;
	}

	TS_LOG_ERR("data length:%d\n", (temp_buf[2] | temp_buf[3] << 8));

	retval = secure_memcpy(buf,
				buf_len,
				&temp_buf[4],
				buf_len,
				buf_len);
	if (retval < 0) {
		TS_LOG_ERR("Failed to copy cfg data\n");
		goto exit;
	}

exit:
	return retval;
}

static int testing_set_static_config(unsigned char *buf, unsigned int buf_len)
{
	int retval;
	struct syna_tcm_hcd *tcm_hcd = testing_hcd->tcm_hcd;	
	if (!buf) {
		TS_LOG_ERR("invalid parameter\n");
		return -EINVAL;
	}



	retval = syna_tcm_alloc_mem(tcm_hcd,
			&testing_hcd->out,
			buf_len);
	if (retval < 0) {
		TS_LOG_ERR("Failed to allocate memory for testing_hcd->out.buf\n");
		return retval;
	}

	retval = secure_memcpy(testing_hcd->out.buf,
				testing_hcd->out.buf_size,
				buf,
				buf_len,
				buf_len);
	if (retval < 0) {
		TS_LOG_ERR("Failed to copy cfg data\n");
		return retval;
	}

	retval = syna_tcm_write_hdl_message(tcm_hcd,
			CMD_SET_STATIC_CONFIG,
			testing_hcd->out.buf,
			buf_len,
			&testing_hcd->resp.buf,
			&testing_hcd->resp.buf_size,
			&testing_hcd->resp.data_length,
			NULL,
			0);

	if (retval < 0) {
		TS_LOG_ERR("Failed to write command %s\n",
				STR(CMD_SET_STATIC_CONFIG));
		return retval;
	}

	msleep(200);

	retval = syna_tcm_read(tcm_hcd,
		temp_buf,
		5);
	
	if (retval < 0) {
		TS_LOG_ERR("Failed to temp_buf data\n");
		return retval;
	}

	if (temp_buf[0] != MESSAGE_MARKER) {
	if ((temp_buf[1] != STATUS_OK) && (temp_buf[1] != 0)) {
		TS_LOG_ERR("message error when set write config %x, %x %x %x\n", temp_buf[0], temp_buf[1], temp_buf[2], temp_buf[3]);
		retval = -EINVAL;
		goto exit;
	}
	}
	/*
	msleep(200);
	retval = syna_tcm_read(tcm_hcd,
		temp_buf,
		sizeof(temp_buf));
	
	if (retval < 0) {
		TS_LOG_ERR("Failed to read out the report mode data\n");
		return retval;
	}
	*/
exit:
	return retval;
}

static unsigned char trx_result[MAX_PINS] = {0};

static int testing_pt1(void)
{
	int retval;
	int i, j, ii;
 	int phy_pin;
	bool do_pin_test = false;
	bool is_rx = false;
	unsigned char *pt1_data = NULL;
	unsigned int pt1_data_size = 8;
	unsigned char *satic_cfg_buf = NULL;
	unsigned int satic_cfg_length;
	short *tx_pins = testing_hcd->tx_pins;
	short tx_assigned;
	short *rx_pins = testing_hcd->rx_pins;
	short rx_assigned;
	short *guard_pins = testing_hcd->guard_pins;
	short guard_assigned = 0;
	struct syna_tcm_app_info *app_info;
	struct syna_tcm_hcd *tcm_hcd = testing_hcd->tcm_hcd;
	int failure_cnt_pt1 = 0;
	int data_length;

	app_info = &tcm_hcd->app_info;

	memset(trx_result, 0, sizeof(trx_result));

	if (!testing_hcd->satic_cfg_buf) {
		satic_cfg_length = le2_to_uint(app_info->static_config_size);

		satic_cfg_buf = kzalloc(satic_cfg_length, GFP_KERNEL);
		if (!satic_cfg_buf) {
			TS_LOG_ERR("Failed on memory allocation for satic_cfg_buf\n");
			goto exit;
		}

		retval = testing_get_static_config(satic_cfg_buf,
										satic_cfg_length);
		if (retval < 0) {
			TS_LOG_ERR("Failed to get static config\n");
			goto exit;
		}

		testing_hcd->satic_cfg_buf = satic_cfg_buf;
	}

	// get pins mapping
	if ( (testing_hcd->tx_assigned <= 0) ||
		 (testing_hcd->rx_assigned <= 0) ||
		 (testing_hcd->guard_assigned <= 0) ){

		if (satic_cfg_buf) {
			retval = testing_pins_mapping(satic_cfg_buf, satic_cfg_length);
			if (retval < 0) {
				TS_LOG_ERR("Failed to get pins mapping\n");
				goto exit;
			}

			TS_LOG_ERR("tx_assigned = %d, rx_assigned = %d, guard_assigned = %d",
						testing_hcd->tx_assigned, testing_hcd->rx_assigned,
						testing_hcd->guard_assigned);
		}
	}

	tx_assigned = testing_hcd->tx_assigned;
	rx_assigned = testing_hcd->rx_assigned;
	guard_assigned = testing_hcd->guard_assigned;


	
	retval = syna_tcm_alloc_mem(tcm_hcd,
			&testing_hcd->out,
			1);
	if (retval < 0) {
		TS_LOG_ERR(
				"Failed to allocate memory for testing_hcd->out.buf\n");
		goto exit;
	}

	testing_hcd->out.buf[0] = TEST_TRX_SENSOR_OPENS;

	retval = syna_tcm_write_hdl_message(tcm_hcd,
			CMD_PRODUCTION_TEST,
			testing_hcd->out.buf,
			1,
			&testing_hcd->resp.buf,
			&testing_hcd->resp.buf_size,
			&testing_hcd->resp.data_length,
			NULL,
			0);
	if (retval < 0) {
		TS_LOG_ERR(
				"Failed to write command %s\n",
				STR(CMD_PRODUCTION_TEST));
		goto exit;
	}
	

	msleep(100);
	retval = syna_tcm_read(tcm_hcd,
			temp_buf,
			pt1_data_size + MESSAGE_HEADER_SIZE + 1);

	if (retval < 0) {
		TS_LOG_ERR("Failed to read raw data\n");
		goto exit;
	}		

	if (temp_buf[0] != MESSAGE_MARKER) {
		TS_LOG_ERR("incorrect Header Marker!");
		retval = -EINVAL;
		goto exit;

	}

	data_length = temp_buf[2] | temp_buf[3] << 8;
	TS_LOG_INFO("%d\n", data_length);
/*
	retval = testing_run_prod_test_item(TEST_TRX_SENSOR_OPENS);
	if (retval < 0) {
		TS_LOG_ERR("Failed to run test TEST_TRX_SENSOR_OPENS\n");
		return -EIO;
	}


	if (pt1_data_size != testing_hcd->resp.data_length) {
		TS_LOG_ERR("pt1 frame size mismatch\n");
		retval = -EINVAL;
		goto exit;
	}

	buf = testing_hcd->resp.buf;
	testing_hcd->result = true;

	pt1_data = kzalloc(sizeof(unsigned char)*pt1_data_size, GFP_KERNEL);
	if (!pt1_data) {
		TS_LOG_ERR("Failed to allocate mem to pt1_data\n");
		goto exit;
	}
	retval = secure_memcpy(pt1_data,
				pt1_data_size,
				testing_hcd->resp.buf,
				testing_hcd->resp.buf_size,
				pt1_data_size);
	if (retval < 0) {
		TS_LOG_ERR("Failed to copy pt1 data\n");
		goto exit;
	}
*/

	testing_hcd->result = true;

	pt1_data = kzalloc(sizeof(unsigned char)*pt1_data_size, GFP_KERNEL);
	if (!pt1_data) {
		TS_LOG_ERR("Failed to allocate mem to pt1_data\n");
		goto exit;
	}
	retval = secure_memcpy(pt1_data,
				pt1_data_size,
				&temp_buf[4],
				pt1_data_size,
				pt1_data_size);
	if (retval < 0) {
		TS_LOG_ERR("Failed to copy pt1 data\n");
		goto exit;
	}

	for (i = 0; i < pt1_data_size; i++) {

		TS_LOG_ERR("[%d]: %2d\n",i , pt1_data[i]);

		for (j = 0; j < 8; j++) {
			phy_pin = (i*8 + j);
			do_pin_test = testing_get_pins_assigned(phy_pin,
				 					tx_pins, tx_assigned,
				 					rx_pins, rx_assigned,
				 					guard_pins, guard_assigned);

            if (do_pin_test) {

				if (CHECK_BIT(pt1_data[i], j) == 0) {
					TS_LOG_ERR("pin-%2d : pass\n", phy_pin);
				}
				else {

					// check pin-0, 1, 32, 33
					if (( 0 == phy_pin) || ( 1 == phy_pin) ||
						(32 == phy_pin)|| (33 == phy_pin)) {

						for (ii = 0; ii < rx_assigned; ii++) {
							is_rx = false;
							if (phy_pin == rx_pins[ii]) {
								is_rx = true;
								break;
							}
						}

						if (is_rx) {
							TS_LOG_ERR("pin-%2d : n/a (is rx)\n", phy_pin);
						}
						else {
							TS_LOG_ERR("pin-%2d : fail (byte %d)\n", phy_pin, i);
							trx_result[phy_pin] = 1;
							failure_cnt_pt1 += 1;
						}
					}
					else {
						TS_LOG_ERR("pin-%2d : fail (byte %d)\n", phy_pin, i);
						trx_result[phy_pin] = 1;
						failure_cnt_pt1 += 1;
					}
				}

            }// end if(do_pin_test)

		}
	}


	testing_hcd->result = (failure_cnt_pt1 == 0);

	retval = failure_cnt_pt1;

exit:
	if (1) {

	if (pt1_data)
		kfree(pt1_data);
	if (satic_cfg_buf)
		kfree(satic_cfg_buf);
	}
	return retval;
}

static int testing_get_test_item_response_data(int item, unsigned char *data, int length)
{

	int retval = 0;
//	int timeout;
	int data_length;
	struct syna_tcm_hcd *tcm_hcd = testing_hcd->tcm_hcd;
	
	retval = syna_tcm_alloc_mem(tcm_hcd,
			&testing_hcd->out,
			1);
	if (retval < 0) {
		TS_LOG_ERR(
				"Failed to allocate memory for testing_hcd->out.buf\n");
		goto exit;
	}

	testing_hcd->out.buf[0] = item;

	retval = syna_tcm_write_hdl_message(tcm_hcd,
			CMD_PRODUCTION_TEST,
			testing_hcd->out.buf,
			1,
			&testing_hcd->resp.buf,
			&testing_hcd->resp.buf_size,
			&testing_hcd->resp.data_length,
			NULL,
			0);
	if (retval < 0) {
		TS_LOG_ERR(
				"Failed to write command %s\n",
				STR(CMD_PRODUCTION_TEST));
		goto exit;
	}
	
	//timeout = REPORT_TIMEOUT_MS;

	//msleep(timeout);
	msleep(100);
	retval = syna_tcm_read(tcm_hcd,
			temp_buf,
			sizeof(temp_buf));

	if (retval < 0) {
		TS_LOG_ERR("Failed to read raw data\n");
		goto exit;
	}		

	if (temp_buf[0] != MESSAGE_MARKER) {
		TS_LOG_ERR("incorrect Header Marker!");
		retval = -EINVAL;
		goto exit;

	}

	data_length = temp_buf[2] | temp_buf[3] << 8;
	TS_LOG_INFO("%d\n", data_length);

	retval = secure_memcpy(data,
				length,
				&temp_buf[4],
				length,
				length);
	if (retval < 0) {
		TS_LOG_ERR("Failed to copy cfg data\n");
		return retval;
	}	
exit:
	return retval;
}

static int testing_discrete_ex_trx_short(void)
{
	int retval;
	int i, j, k, pin;
	unsigned char *buf;
	unsigned int rows;
	unsigned int cols;
	unsigned char *satic_cfg_buf = NULL;
	unsigned int satic_cfg_length;
	struct syna_tcm_app_info *app_info;
	struct syna_tcm_hcd *tcm_hcd = testing_hcd->tcm_hcd;
	int result_pt1 = 0;
	int result_pt196 = 0;
	unsigned int pt196_size;
	unsigned short *pt196_base;
	unsigned short *pt196_delta;
	unsigned short *min_rx;
	unsigned short *max_rx;
	unsigned short tmp_data;
	unsigned short extend[4] = {0, 1, 32, 33};
	bool do_pin_test;
	unsigned short logical_pin;

	app_info = &tcm_hcd->app_info;

	rows = le2_to_uint(app_info->num_of_image_rows);
	cols = le2_to_uint(app_info->num_of_image_cols);

	pt196_size = rows * cols * 2;

	satic_cfg_length = le2_to_uint(app_info->static_config_size);
if (testing_hcd->satic_cfg_buf == NULL) {
	satic_cfg_buf = kzalloc(satic_cfg_length, GFP_KERNEL);
	if (!satic_cfg_buf) {
		TS_LOG_ERR("Failed on memory allocation for satic_cfg_buf\n");
		goto exit;
	}
	testing_hcd->satic_cfg_buf = satic_cfg_buf;
} else {
	satic_cfg_buf = testing_hcd->satic_cfg_buf;
}
	
	retval = testing_get_static_config(satic_cfg_buf,
									satic_cfg_length);
	if (retval < 0) {
		TS_LOG_ERR("Failed to get static config\n");
		goto exit;
	}

	

	// get pins mapping
	if ( (testing_hcd->tx_assigned <= 0) ||
		 (testing_hcd->rx_assigned <= 0) ||
		 (testing_hcd->guard_assigned <= 0) ){

		if (satic_cfg_buf) {
			retval = testing_pins_mapping(satic_cfg_buf, satic_cfg_length);
			if (retval < 0) {
				TS_LOG_ERR("Failed to get pins mapping\n");
				goto exit;
			}

			TS_LOG_ERR("tx_assigned = %d, rx_assigned = %d, guard_assigned = %d",
						testing_hcd->tx_assigned, testing_hcd->rx_assigned,
						testing_hcd->guard_assigned);
		}
	}

	// do pt1 testing
	retval = testing_pt1();
	if (retval < 0) {
		TS_LOG_ERR("Failed to run pt1 test\n");
		goto exit;
	}
	result_pt1 = retval;  // copy the result of pt1

	// do sw reset
	/*
	if (tcm_hcd->reset(tcm_hcd, false, true) < 0) {
		LOGE(tcm_hcd->pdev->dev.parent, "Failed to do reset\n");
	}
	*/
	reflash_after_fw_update_do_reset();
	// change analog settings
	for (i = 0; i < (CFG_IMAGE_CBCS_LENGTH/8); i++) {
		satic_cfg_buf[(CFG_IMAGE_CBCS_OFFSET/8) + i] = 0x00;
	}
	satic_cfg_buf[(CFG_REF_LO_TRANS_CAP_OFFSET/8)] = 0x06;
	satic_cfg_buf[(CFG_REF_LO_XMTR_PL_OFFSET/8)] = 0x01;
	satic_cfg_buf[(CFG_REF_HI_TRANS_CAP_OFFSET/8)] = 0x06;
	satic_cfg_buf[(CFG_REF_HI_XMTR_PL_OFFSET/8)] = 0x00;
	satic_cfg_buf[(CFG_REF_GAIN_CTRL_OFFSET/8)] = 0x00;

	retval = testing_set_static_config(satic_cfg_buf,
									satic_cfg_length);

	
	if (retval < 0) {
		TS_LOG_ERR("Failed to set static config\n");
		goto exit;
	}

	// get pt196 as baseline
/*	
	retval = testing_run_prod_test_item(TEST_DISCRETE_PT196);
	if (retval < 0) {
		TS_LOG_ERR("Failed to get pt196 base image\n");
		goto exit;
	}

	LOCK_BUFFER(testing_hcd->resp);

	if (pt196_size != testing_hcd->resp.data_length) {
		TS_LOG_ERR("pt196_size mismatch\n");
		UNLOCK_BUFFER(testing_hcd->resp);
		retval = -EINVAL;
		goto exit;
	}

	buf = testing_hcd->resp.buf;
*/
	pt196_base = kzalloc(sizeof(unsigned short)*rows*cols, GFP_KERNEL);
	if (!pt196_base) {
		TS_LOG_ERR("Failed on memory allocation for pt196_base\n");
		UNLOCK_BUFFER(testing_hcd->resp);
		retval = -EINVAL;
		goto exit;
	}

	retval = testing_get_test_item_response_data(TEST_DISCRETE_PT196, temp_buf_2, rows*cols*2);
	if (retval < 0) {
		TS_LOG_ERR("fail to do test item TEST_DISCRETE_PT196");
		retval = -EINVAL;
		goto exit;
	}
	buf = temp_buf_2;

	k = 0;
	for (i = 0; i < rows; i++) {
		for (j = 0; j < cols; j++) {
			pt196_base[i * cols + j] =
					(unsigned short)(buf[k] &0xff) | (unsigned short)(buf[k+1] << 8);
			k+=2;
		}
	}


	pt196_delta = kzalloc(sizeof(unsigned short)*rows*cols, GFP_KERNEL);
	if (!pt196_delta) {
		TS_LOG_ERR("Failed on memory allocation for pt196_delta\n");
		retval = -EINVAL;
		goto exit;
	}
	min_rx= kzalloc(sizeof(unsigned short)*(testing_hcd->rx_assigned), GFP_KERNEL);
	if (!min_rx) {
		TS_LOG_ERR("Failed on memory allocation for min_rx\n");
		retval = -EINVAL;
		goto exit;
	}
	max_rx = kzalloc(sizeof(unsigned short)*(testing_hcd->rx_assigned), GFP_KERNEL);
	if (!max_rx) {
		TS_LOG_ERR("Failed on memory allocation for max_rx\n");
		retval = -EINVAL;
		goto exit;
	}

	// walk through all extend pins
	for (pin = 0; pin < 4; pin++) {

		do_pin_test = testing_get_pins_assigned(extend[pin],
				 					testing_hcd->tx_pins, testing_hcd->tx_assigned,
				 					testing_hcd->rx_pins, testing_hcd->rx_assigned,
				 					testing_hcd->guard_pins, testing_hcd->guard_assigned);
		if (!do_pin_test)
			continue;  // skip if pin is not assigned

		for (i = 0; i < testing_hcd->rx_assigned; i++) {
			do_pin_test = false;
			if (extend[pin] == testing_hcd->rx_pins[i]) {
				do_pin_test = true;
				logical_pin = i;
				break;
			}
		}

		if (!do_pin_test)
			continue;  // skip if pin is not rx


		for (i = 0; i < testing_hcd->rx_assigned; i++) {
			min_rx[i] = 5000;
			max_rx[i] = 0;
		}

		TS_LOG_ERR("pin = %d, logical pin = %d\n", extend[pin], logical_pin);

		// adjust cbc for the target logical pin
		for (i = 0; i < (CFG_IMAGE_CBCS_LENGTH/8); i++) {
			if (i == logical_pin*2)
				satic_cfg_buf[(CFG_IMAGE_CBCS_OFFSET/8) + i] = 0x0f;
			else
				satic_cfg_buf[(CFG_IMAGE_CBCS_OFFSET/8) + i] = 0x00;
		}
		retval = testing_set_static_config(satic_cfg_buf,
									satic_cfg_length);
		if (retval < 0) {
			TS_LOG_ERR("Failed to set static config for logical pin %d\n",
					logical_pin);
			goto exit;
		}

		// get pt196 again
		// and do calculation to get the delta
		/*
		retval = testing_run_prod_test_item(TEST_DISCRETE_PT196);
		if (retval < 0) {
			TS_LOG_ERR("Failed to get pt196 base image\n");
			goto exit;
		}

		LOCK_BUFFER(testing_hcd->resp);

		if (pt196_size != testing_hcd->resp.data_length) {
			TS_LOG_ERR("pt196_size mismatch\n");
			UNLOCK_BUFFER(testing_hcd->resp);
			retval = -EINVAL;
			goto exit;
		}	
		buf = testing_hcd->resp.buf;
		*/
		retval = testing_get_test_item_response_data(TEST_DISCRETE_PT196, temp_buf_2, rows*cols*2);
		if (retval < 0) {
			TS_LOG_ERR("fail to do test item TEST_DISCRETE_PT196");
			retval = -EINVAL;
			goto exit;
		}
		buf = temp_buf_2;
		k = 0;
		for (i = 0; i < rows; i++) {
			for (j = 0; j < cols; j++) {

				tmp_data = (unsigned short)(buf[k] &0xff) | (unsigned short)(buf[k+1] << 8);
				pt196_delta[i * cols + j] = abs(tmp_data - pt196_base[i * cols + j]);

				if (testing_hcd->rx_assigned == cols) {
					min_rx[j] = MIN(min_rx[j], pt196_delta[i * cols + j]);
					max_rx[j] = MAX(max_rx[j], pt196_delta[i * cols + j]);
				}
				else if (testing_hcd->rx_assigned == rows) {
					min_rx[i] = MIN(min_rx[i], pt196_delta[i * cols + j]);
					max_rx[i] = MAX(max_rx[i], pt196_delta[i * cols + j]);
				}

				k+=2;
			}
		}

		// data verification
		for (i = 0; i < testing_hcd->rx_assigned; i++) {

			if (i == logical_pin) {
				// the delta should be higher than limit
				if (min_rx[i] < 2000) {
					TS_LOG_ERR("fail at pin %d (logical: %d), delta: (%d, %d)\n",
							extend[pin], logical_pin, min_rx[i], max_rx[i]);
					trx_result[extend[pin]] = 1;
					result_pt196 += 1;
				}
			}
			else {
				// if it is not the extended pin
				// the delta should be less than limit
				if (max_rx[i] >= 200) {
					TS_LOG_ERR("fail at logical pin %d, delta: (%d, %d)\n",
							i, min_rx[i], max_rx[i]);
					trx_result[extend[pin]] = 1;
					result_pt196 += 1;
				}
			}
		}

	}


	retval = 0;
	testing_hcd->result = ((result_pt1 == 0) && (result_pt196 == 0));


exit:
	TS_LOG_ERR("test result is %d  %d", result_pt1, result_pt196);
	for (i = 0; i < MAX_PINS; i++) {
		TS_LOG_ERR("trx_result[%d]:%d\n", i, trx_result[i]);
	}
	TS_LOG_ERR("at here, ready to call kfree");
	if (1) {
	//if (satic_cfg_buf)
		//kfree(satic_cfg_buf);
	if (pt196_base)
		kfree(pt196_base);
	if (pt196_delta)
		kfree(pt196_delta);
	if (min_rx)
		kfree(min_rx);
	if (max_rx)
		kfree(max_rx);
	}
/*
	if (tcm_hcd->reset(tcm_hcd, false, true) < 0) {
		TS_LOG_ERR("Failed to do reset\n");
	}
*/
	reflash_after_fw_update_do_reset();
	return retval;
}

int testing_init(struct syna_tcm_hcd *tcm_hcd)
{	
	unsigned int rows;
	unsigned int cols;

	struct syna_tcm_app_info *app_info;
	
	if (!testing_hcd) {
		testing_hcd = kzalloc(sizeof(*testing_hcd), GFP_KERNEL);
		if (!testing_hcd) {
			TS_LOG_ERR(
					"Failed to allocate memory for testing_hcd\n");
			return -ENOMEM;
		}
	}

	testing_hcd->tcm_hcd = tcm_hcd;
	app_info = &tcm_hcd->app_info;
	rows = le2_to_uint(app_info->num_of_image_rows);
	cols = le2_to_uint(app_info->num_of_image_cols);

	testing_hcd->mmi_buf = (int *)kzalloc(rows * cols * sizeof(int), GFP_KERNEL);
	if (!testing_hcd->mmi_buf) {
		TS_LOG_ERR(" Failed to alloc mmi_buf\n");
		return -ENOMEM;
	}		
	rawdata_size = rows * cols * 2 + 2;
	
	TS_LOG_ERR("testing init: rows:%d, cols:%d\n", rows, cols);

	memset(syna_tcm_mmi_test_result, 0, sizeof(syna_tcm_mmi_test_result));

	return 0;
}

int testing_remove(struct syna_tcm_hcd *tcm_hcd)
{
	if (testing_hcd) {
		kfree(testing_hcd);
		testing_hcd = NULL;
	}
	if (testing_hcd->satic_cfg_buf) {
		kfree(testing_hcd->satic_cfg_buf);
	}
	
	return 0;
}

MODULE_AUTHOR("Synaptics, Inc.");
MODULE_DESCRIPTION("Synaptics TCM Testing Module");
MODULE_LICENSE("GPL v2");
