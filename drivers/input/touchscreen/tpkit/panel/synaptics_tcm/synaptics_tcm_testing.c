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
	TEST_DYNAMIC_RANGE = 7,
	TEST_OPEN_SHORT_DETECTOR = 8,
	TEST_NOISE = 10,
	TEST_PT11 = 11,
	TEST_PT12 = 12,
	TEST_PT13 = 13,
};

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
	
	timeout = REPORT_TIMEOUT_MS;

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

	testing_get_frame_size_words(&frame_size_words, false);

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

	if (rows > limits_rows || cols > limits_cols) {
		TS_LOG_ERR(
				"Mismatching limits data\n");
		retval = -EINVAL;
		goto exit;
	}

	idx = 0;
	testing_hcd->result = true;

	for (row = 0; row < rows; row++) {
		for (col = 0; col < cols; col++) {
			data = (signed short)le2_to_uint(&temp_buf[idx * 2 + MESSAGE_HEADER_SIZE]);
			if (data > noise_limits[row][col]) {
				TS_LOG_INFO("overlow_data = %8u, row = %d, col = %d\n", data, row, col);
				testing_hcd->result = false;                  
				strncat(syna_tcm_mmi_test_result, "2F-", MAX_STR_LEN);
				goto exit;
			}
			idx++;
		}
	}

	strncat(syna_tcm_mmi_test_result, "2P-", MAX_STR_LEN);

exit:
	kfree(temp_buf);
	return retval;
}

static unsigned char temp_buf[2000];
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

	strncat(syna_tcm_mmi_test_result, "1P-", MAX_STR_LEN);

exit:
	testing_hcd->report_type = 0;

	return retval;
}

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
	
	retval = testing_raw_data(info);
	if (retval < 0) {
		strncat(syna_tcm_mmi_test_result, "1F-", MAX_STR_LEN);
	}
	
	info->buff[0] = rows;
	info->buff[1] = cols;
	//memcpy(&info->buff[2], testing_hcd->mmi_buf, rows * cols * sizeof(int));

	msleep(100);
	retval = testing_noise();
	if (retval < 0) {
		strncat(syna_tcm_mmi_test_result, "2F-", MAX_STR_LEN);
	}
	info->hybrid_buff[0] = rows;
	info->hybrid_buff[1] = cols;
	memcpy(&info->hybrid_buff[2], testing_hcd->mmi_buf, rows * cols * sizeof(int));
	
	msleep(100);
	
	retval = testing_open_short();
	if (retval < 0) {
		strncat(syna_tcm_mmi_test_result, "3F-", MAX_STR_LEN);
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
	
	return 0;
}

MODULE_AUTHOR("Synaptics, Inc.");
MODULE_DESCRIPTION("Synaptics TCM Testing Module");
MODULE_LICENSE("GPL v2");
