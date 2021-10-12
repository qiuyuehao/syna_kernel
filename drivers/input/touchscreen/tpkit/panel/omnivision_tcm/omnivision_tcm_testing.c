/*
 * Omnivision TCM touchscreen driver
 *
 * Copyright (C) 2017-2018 Omnivision Incorporated. All rights reserved.

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
 * INFORMATION CONTAINED IN THIS DOCUMENT IS PROVIDED "AS-IS," AND Omnivision
 * EXPRESSLY DISCLAIMS ALL EXPRESS AND IMPLIED WARRANTIES, INCLUDING ANY
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE,
 * AND ANY WARRANTIES OF NON-INFRINGEMENT OF ANY INTELLECTUAL PROPERTY RIGHTS.
 * IN NO EVENT SHALL Omnivision BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, PUNITIVE, OR CONSEQUENTIAL DAMAGES ARISING OUT OF OR IN CONNECTION
 * WITH THE USE OF THE INFORMATION CONTAINED IN THIS DOCUMENT, HOWEVER CAUSED
 * AND BASED ON ANY THEORY OF LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, AND EVEN IF Omnivision WAS ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE. IF A TRIBUNAL OF COMPETENT JURISDICTION DOES
 * NOT PERMIT THE DISCLAIMER OF DIRECT DAMAGES OR ANY OTHER DAMAGES, Omnivision'
 * TOTAL CUMULATIVE LIABILITY TO ANY PARTY SHALL NOT EXCEED ONE HUNDRED U.S.
 * DOLLARS.
 */

#include <linux/gpio.h>
#include "omnivision_tcm_core.h"
#include "omnivision_tcm_testing.h"

#define SYSFS_DIR_NAME "testing"

#define REPORT_TIMEOUT_MS 5000

#define testing_sysfs_show(t_name) \
static ssize_t testing_sysfs_##t_name##_show(struct device *dev, \
		struct device_attribute *attr, char *buf) \
{ \
	int retval; \
	struct ovt_tcm_hcd *tcm_hcd = testing_hcd->tcm_hcd; \
\
	mutex_lock(&tcm_hcd->extif_mutex); \
\
	retval = testing_##t_name(); \
	if (retval < 0) { \
		OVT_LOG_ERR( \
				"Failed to do "#t_name" test"); \
		goto exit; \
	} \
\
	retval = snprintf(buf, PAGE_SIZE, \
			"%s\n", \
			testing_hcd->result ? "Passed" : "Failed"); \
\
exit: \
	mutex_unlock(&tcm_hcd->extif_mutex); \
\
	return retval; \
}

#define CHECK_BIT(var, pos) ((var) & (1<<(pos)))

/* Sync with the Comm2 Release 21 */
/* Not all are implemented for every ASIC */
enum test_code {
	TEST_NOT_IMPLEMENTED = 0x00,
	TEST_PT1_TRX_TRX_SHORTS = 0x01,
	TEST_PT2_TRX_SENSOR_OPENS = 0x02,
	TEST_PT3_TRX_GROUND_SHORTS = 0x03,
	TEST_PT5_FULL_RAW_CAP = 0x05,
	TEST_PT6_EE_SHORT = 0x06,
	TEST_PT7_DYNAMIC_RANGE = 0x07,
	TEST_PT8_HIGH_RESISTANCE = 0x08,
	TEST_PT10_DELTA_NOISE = 0x0a,
	TEST_PT11_OPEN_DETECTION = 0x0b,
	TEST_PT12 = 0x0c,
	TEST_PT13 = 0x0d,
	TEST_PT14_DOZE_DYNAMIC_RANGE = 0x0e,
	TEST_PT15_DOZE_NOISE = 0x0f,
	TEST_PT16_SENSOR_SPEED = 0x10,
	TEST_PT17_ADC_RANGE = 0x11,
	TEST_PT18_HYBRID_ABS_RAW = 0x12,
	TEST_PT29_HYBRID_ABS_NOISE = 0x1D,
};

#define OVT_TCM_TEST_BUF_LEN 50
static char ovt_tcm_mmi_test_result[OVT_TCM_TEST_BUF_LEN] = { 0 };
struct testing_hcd {
	bool result;
	unsigned char report_type;
	unsigned int report_index;
	unsigned int num_of_reports;
	struct kobject *sysfs_dir;
	struct ovt_tcm_buffer out;
	struct ovt_tcm_buffer resp;
	struct ovt_tcm_buffer report;
	struct ovt_tcm_buffer process;
	struct ovt_tcm_buffer output;
	struct ovt_tcm_hcd *tcm_hcd;
	int (*collect_reports)(enum report_type report_type,
			unsigned int num_of_reports);

#ifdef PT1_GET_PIN_ASSIGNMENT
#define MAX_PINS (64)
	unsigned char *satic_cfg_buf;
	short tx_pins[MAX_PINS];
	short tx_assigned;
	short rx_pins[MAX_PINS];
	short rx_assigned;
	short guard_pins[MAX_PINS];
	short guard_assigned;
#endif

};

static struct testing_hcd *testing_hcd;
int ovt_testing_init(struct ovt_tcm_hcd *tcm_hcd)
{
	int retval = 0;
	int idx;

	testing_hcd = kzalloc(sizeof(*testing_hcd), GFP_KERNEL);
	if (!testing_hcd) {
		OVT_LOG_ERR(
				"Failed to allocate memory for testing_hcd");
		return -ENOMEM;
	}

	testing_hcd->tcm_hcd = tcm_hcd;


	INIT_BUFFER(testing_hcd->out, false);
	INIT_BUFFER(testing_hcd->resp, false);
	INIT_BUFFER(testing_hcd->report, false);
	INIT_BUFFER(testing_hcd->process, false);
	INIT_BUFFER(testing_hcd->output, false);
#if 0
	testing_hcd->sysfs_dir = kobject_create_and_add(SYSFS_DIR_NAME,
			tcm_hcd->sysfs_dir);
	if (!testing_hcd->sysfs_dir) {
		OVT_LOG_ERR(
				"Failed to create sysfs directory");
		retval = -EINVAL;
		goto err_sysfs_create_dir;
	}

	for (idx = 0; idx < ARRAY_SIZE(attrs); idx++) {
		retval = sysfs_create_file(testing_hcd->sysfs_dir,
				&(*attrs[idx]).attr);
		if (retval < 0) {
			OVT_LOG_ERR(
					"Failed to create sysfs file");
			goto err_sysfs_create_file;
		}
	}

	retval = sysfs_create_bin_file(testing_hcd->sysfs_dir, &bin_attr);
	if (retval < 0) {
		OVT_LOG_ERR(
				"Failed to create sysfs bin file");
		goto err_sysfs_create_bin_file;
	}

	return 0;

err_sysfs_create_bin_file:
err_sysfs_create_file:
	for (idx--; idx >= 0; idx--)
		sysfs_remove_file(testing_hcd->sysfs_dir, &(*attrs[idx]).attr);

	kobject_put(testing_hcd->sysfs_dir);

err_sysfs_create_dir:
	RELEASE_BUFFER(testing_hcd->output);
	RELEASE_BUFFER(testing_hcd->process);
	RELEASE_BUFFER(testing_hcd->report);
	RELEASE_BUFFER(testing_hcd->resp);
	RELEASE_BUFFER(testing_hcd->out);

	kfree(testing_hcd);
	testing_hcd = NULL;
#endif
	return retval;
}
static int ovt_testing_full_raw_cap(void)
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
	struct ovt_tcm_app_info *app_info;
	struct ovt_tcm_hcd *tcm_hcd = testing_hcd->tcm_hcd;


	// if (tcm_hcd->id_info.mode != MODE_APPLICATION ||
	// 		tcm_hcd->app_status != APP_STATUS_OK) {
	// 	return -ENODEV;
	// }

	app_info = &tcm_hcd->app_info;
	rows = le2_to_uint(app_info->num_of_image_rows);
	cols = le2_to_uint(app_info->num_of_image_cols);

	retval = ovt_tcm_alloc_mem(tcm_hcd,
			&testing_hcd->out,
			1);
	if (retval < 0) {
		OVT_LOG_ERR(
				"Failed to allocate memory for testing_hcd->out.buf");
		goto exit;
	}

	testing_hcd->out.buf[0] = TEST_PT5_FULL_RAW_CAP;

	retval = tcm_hcd->write_message(tcm_hcd,
			CMD_PRODUCTION_TEST,
			testing_hcd->out.buf,
			1,
			&testing_hcd->resp.buf,
			&testing_hcd->resp.buf_size,
			&testing_hcd->resp.data_length,
			NULL,
			0);
	if (retval < 0) {
		OVT_LOG_ERR(
				"Failed to write command %s %s\n",
				STR(CMD_PRODUCTION_TEST), STR(TEST_PT5_FULL_RAW_CAP));
		goto exit;
	}

	data_length = testing_hcd->resp.buf[2] | testing_hcd->resp.buf[3] << 8;
	OVT_LOG_INFO("%d\n", data_length);


	limits_rows = sizeof(pt5_hi_limits) / sizeof(pt5_hi_limits[0]);
	limits_cols = sizeof(pt5_hi_limits[0]) / sizeof(pt5_hi_limits[0][0]);

	if (rows > limits_rows || cols > limits_cols) {
		OVT_LOG_ERR(
				"Mismatching limits data");
		retval = -EINVAL;
		goto exit;
	}

	idx = 0;
	testing_hcd->result = true;

	OVT_LOG_ERR("full raw data begin");
	for (row = 0; row < rows; row++) {
		for (col = 0; col < cols; col++) {
			data = (signed short)le2_to_uint(&testing_hcd->resp.buf[idx * 2]);
			OVT_LOG_ERR("data[%d]: %d, ", idx, data);
			if ((data > pt5_hi_limits[row][col]) || (data < pt5_lo_limits[row][col])) {
				OVT_LOG_ERR("overlow_data = %8u, row = %d, col = %d\n", data, row, col);
				testing_hcd->result = false;
				retval = -EINVAL;
				strncat(ovt_tcm_mmi_test_result, "2F-", MAX_STR_LEN);
				goto exit;
			}
			idx++;
		}
	}
	OVT_LOG_ERR("full raw data end");

	strncat(ovt_tcm_mmi_test_result, "2P-", MAX_STR_LEN);

exit:
	return retval;
}

static int ovt_testing_noise(void)
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
	struct ovt_tcm_app_info *app_info;
	struct ovt_tcm_hcd *tcm_hcd = testing_hcd->tcm_hcd;

	// if (tcm_hcd->id_info.mode != MODE_APPLICATION ||
	// 		tcm_hcd->app_status != APP_STATUS_OK) {
	// 	return -ENODEV;
	// }
	retval = -1;

	app_info = &tcm_hcd->app_info;
	rows = le2_to_uint(app_info->num_of_image_rows);
	cols = le2_to_uint(app_info->num_of_image_cols);



	retval = ovt_tcm_alloc_mem(tcm_hcd,
			&testing_hcd->out,
			1);
	if (retval < 0) {
		OVT_LOG_ERR(
				"Failed to allocate memory for testing_hcd->out.buf");
		goto exit;
	}

	testing_hcd->out.buf[0] = TEST_PT10_DELTA_NOISE;

	retval = tcm_hcd->write_message(tcm_hcd,
			CMD_PRODUCTION_TEST,
			testing_hcd->out.buf,
			1,
			&testing_hcd->resp.buf,
			&testing_hcd->resp.buf_size,
			&testing_hcd->resp.data_length,
			NULL,
			0);
	if (retval < 0) {
		OVT_LOG_ERR(
				"Failed to write command %s  %s\n",
				STR(CMD_PRODUCTION_TEST), STR(TEST_PT10_DELTA_NOISE));
		goto exit;
	}

	data_length = testing_hcd->resp.data_length;
	OVT_LOG_INFO("%d\n", data_length);



	limits_rows = sizeof(pt10_limits) / sizeof(pt10_limits[0]);
	limits_cols = sizeof(pt10_limits[0]) / sizeof(pt10_limits[0][0]);

	idx = 0;
	testing_hcd->result = true;

	//OVT_LOG_ERR("noise data begin");
	for (row = 0; row < rows; row++) {
		for (col = 0; col < cols; col++) {
			data = (signed short)le2_to_uint(&testing_hcd->resp.buf[idx * 2]);
			OVT_LOG_ERR("noise data[%d]: %d,   %x , %x ", idx, data, testing_hcd->resp.buf[idx * 2], testing_hcd->resp.buf[idx * 2 + 1]);
			/*if (data > pt10_limits[row][col]) {
				OVT_LOG_INFO("overlow_data = %8u, row = %d, col = %d\n", data, row, col);
				testing_hcd->result = false;                  
				strncat(ovt_tcm_mmi_test_result, "2F-", MAX_STR_LEN);
				goto exit;
			}*/
			idx++;
		}
	}
	//OVT_LOG_ERR("noise data end");
	strncat(ovt_tcm_mmi_test_result, "2P-", MAX_STR_LEN);

exit:
	return retval;
}

int ovt_tcm_testing(struct ts_rawdata_info *info)
{
	int retval = NO_ERR;
	unsigned int rows;
	unsigned int cols;
	struct ovt_tcm_hcd *tcm_hcd = testing_hcd->tcm_hcd;
	struct ovt_tcm_app_info *app_info;
	
	OVT_LOG_INFO("ovt_tcm_testing called");

	testing_hcd->tcm_hcd = tcm_hcd;
	app_info = &tcm_hcd->app_info;

	rows = le2_to_uint(app_info->num_of_image_rows);
	cols = le2_to_uint(app_info->num_of_image_cols);

	// if (tcm_hcd->id_info.mode != MODE_APPLICATION ||
	// 	tcm_hcd->app_status != APP_STATUS_OK) {
	// 		memcpy(ovt_tcm_mmi_test_result, "0F-1F-2F-3F",
	// 	       (strlen("0F-1F-2F-3F") + 1));
	// 	return RESULT_ERR;
	// } else {
	memcpy(ovt_tcm_mmi_test_result, "0P-", (strlen("0P-") + 1));
	//}
	retval = ovt_testing_full_raw_cap();
	if (retval < 0) {
		OVT_LOG_ERR("fail to do full raw cap");
		strncat(ovt_tcm_mmi_test_result, "1F-", MAX_STR_LEN);
		goto exit;
	}

	retval = ovt_testing_noise();
	if (retval < 0) {
		OVT_LOG_ERR("fail to do full noise test");
		strncat(ovt_tcm_mmi_test_result, "2F-", MAX_STR_LEN);
		goto exit;
	}
	info->hybrid_buff[0] = rows;
	info->hybrid_buff[1] = cols;
	memcpy(&info->hybrid_buff[2], testing_hcd->resp.buf, rows * cols * sizeof(int));
	
	memcpy(info->result, ovt_tcm_mmi_test_result, strlen(ovt_tcm_mmi_test_result));
	info->used_size = rows * cols + 2;
	OVT_LOG_INFO("info->used_size = %d\n", info->used_size);
exit:

	return 0;
}



#if 0
DECLARE_COMPLETION(report_complete);

DECLARE_COMPLETION(testing_remove_complete);



/* testing implementation */
static int testing_device_id(void);

static int testing_config_id(void);

static int testing_reset_open(void);

static int testing_pt01_trx_trx_short(void);

static int testing_pt05_full_raw(void);

static int testing_pt07_dynamic_range(void);

static int testing_pt10_noise(void);

static int testing_pt11_open_detection(void);

SHOW_PROTOTYPE(testing, size)

/* nodes for testing */
SHOW_PROTOTYPE(testing, device_id)
SHOW_PROTOTYPE(testing, config_id)
SHOW_PROTOTYPE(testing, reset_open)
SHOW_PROTOTYPE(testing, pt01_trx_trx_short)
SHOW_PROTOTYPE(testing, pt05_full_raw)
SHOW_PROTOTYPE(testing, pt07_dynamic_range)
SHOW_PROTOTYPE(testing, pt10_noise)
SHOW_PROTOTYPE(testing, pt11_open_detection)


static struct device_attribute *attrs[] = {
	ATTRIFY(size),
	ATTRIFY(device_id),
	ATTRIFY(config_id),
	ATTRIFY(pt01_trx_trx_short),
	ATTRIFY(pt05_full_raw),
	ATTRIFY(pt07_dynamic_range),
	ATTRIFY(pt10_noise),
	ATTRIFY(pt11_open_detection),
	ATTRIFY(reset_open),
};

static ssize_t testing_sysfs_data_show(struct file *data_file,
		struct kobject *kobj, struct bin_attribute *attributes,
		char *buf, loff_t pos, size_t count);

static struct bin_attribute bin_attr = {
	.attr = {
		.name = "data",
		.mode = S_IRUGO,
	},
	.size = 0,
	.read = testing_sysfs_data_show,
};

testing_sysfs_show(device_id)

testing_sysfs_show(config_id)

testing_sysfs_show(pt01_trx_trx_short)

testing_sysfs_show(pt05_full_raw)

testing_sysfs_show(pt07_dynamic_range)

testing_sysfs_show(pt10_noise)

testing_sysfs_show(pt11_open_detection)

testing_sysfs_show(reset_open)


static ssize_t testing_sysfs_size_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int retval;
	struct ovt_tcm_hcd *tcm_hcd = testing_hcd->tcm_hcd;

	mutex_lock(&tcm_hcd->extif_mutex);

	LOCK_BUFFER(testing_hcd->output);

	retval = snprintf(buf, PAGE_SIZE,
			"%u\n",
			testing_hcd->output.data_length);

	UNLOCK_BUFFER(testing_hcd->output);

	mutex_unlock(&tcm_hcd->extif_mutex);

	return retval;
}

static ssize_t testing_sysfs_data_show(struct file *data_file,
		struct kobject *kobj, struct bin_attribute *attributes,
		char *buf, loff_t pos, size_t count)
{
	int retval;
	unsigned int readlen;
	struct ovt_tcm_hcd *tcm_hcd = testing_hcd->tcm_hcd;

	mutex_lock(&tcm_hcd->extif_mutex);

	LOCK_BUFFER(testing_hcd->output);

	readlen = MIN(count, testing_hcd->output.data_length - pos);

	retval = secure_memcpy(buf,
			count,
			&testing_hcd->output.buf[pos],
			testing_hcd->output.buf_size - pos,
			readlen);
	if (retval < 0) {
		OVT_LOG_ERR(
			"Failed to copy report data");
	} else {
		retval = readlen;
	}

	UNLOCK_BUFFER(testing_hcd->output);

	mutex_unlock(&tcm_hcd->extif_mutex);

	return retval;
}

static int testing_run_prod_test_item(enum test_code test_code)
{
	int retval;
	struct ovt_tcm_hcd *tcm_hcd = testing_hcd->tcm_hcd;

	if (tcm_hcd->features.dual_firmware &&
			tcm_hcd->id_info.mode != MODE_PRODUCTIONTEST_FIRMWARE) {
		retval = tcm_hcd->switch_mode(tcm_hcd, FW_MODE_PRODUCTION_TEST);
		if (retval < 0) {
			OVT_LOG_ERR(
					"Failed to run production test firmware");
			return retval;
		}
	} else if (IS_NOT_FW_MODE(tcm_hcd->id_info.mode) ||
			tcm_hcd->app_status != APP_STATUS_OK) {
		OVT_LOG_ERR(
				"Identifying mode = 0x%02x\n",
				tcm_hcd->id_info.mode);
		return -ENODEV;
	}

	LOCK_BUFFER(testing_hcd->out);

	retval = ovt_tcm_alloc_mem(tcm_hcd,
			&testing_hcd->out,
			1);
	if (retval < 0) {
		OVT_LOG_ERR(
				"Failed to allocate memory for testing_hcd->out.buf");
		UNLOCK_BUFFER(testing_hcd->out);
		return retval;
	}

	testing_hcd->out.buf[0] = test_code;

	LOCK_BUFFER(testing_hcd->resp);

	retval = tcm_hcd->write_message(tcm_hcd,
			CMD_PRODUCTION_TEST,
			testing_hcd->out.buf,
			1,
			&testing_hcd->resp.buf,
			&testing_hcd->resp.buf_size,
			&testing_hcd->resp.data_length,
			NULL,
			0);
	if (retval < 0) {
		OVT_LOG_ERR(
				"Failed to write command %s\n",
				STR(CMD_PRODUCTION_TEST));
		UNLOCK_BUFFER(testing_hcd->resp);
		UNLOCK_BUFFER(testing_hcd->out);
		return retval;
	}

	UNLOCK_BUFFER(testing_hcd->resp);
	UNLOCK_BUFFER(testing_hcd->out);

	return 0;
}

static int testing_collect_reports(enum report_type report_type,
		unsigned int num_of_reports)
{
	int retval;
	bool completed;
	unsigned int timeout;
	struct ovt_tcm_hcd *tcm_hcd = testing_hcd->tcm_hcd;

	testing_hcd->report_index = 0;
	testing_hcd->report_type = report_type;
	testing_hcd->num_of_reports = num_of_reports;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 13, 0))
	reinit_completion(&report_complete);
#else
	INIT_COMPLETION(report_complete);
#endif

	LOCK_BUFFER(testing_hcd->out);

	retval = ovt_tcm_alloc_mem(tcm_hcd,
			&testing_hcd->out,
			1);
	if (retval < 0) {
		OVT_LOG_ERR(
				"Failed to allocate memory for testing_hcd->out.buf");
		UNLOCK_BUFFER(testing_hcd->out);
		goto exit;
	}

	testing_hcd->out.buf[0] = testing_hcd->report_type;

	LOCK_BUFFER(testing_hcd->resp);

	retval = tcm_hcd->write_message(tcm_hcd,
			CMD_ENABLE_REPORT,
			testing_hcd->out.buf,
			1,
			&testing_hcd->resp.buf,
			&testing_hcd->resp.buf_size,
			&testing_hcd->resp.data_length,
			NULL,
			0);
	if (retval < 0) {
		OVT_LOG_ERR(
				"Failed to write command %s\n",
				STR(CMD_ENABLE_REPORT));
		UNLOCK_BUFFER(testing_hcd->resp);
		UNLOCK_BUFFER(testing_hcd->out);
		goto exit;
	}

	UNLOCK_BUFFER(testing_hcd->resp);
	UNLOCK_BUFFER(testing_hcd->out);

	completed = false;
	timeout = REPORT_TIMEOUT_MS * num_of_reports;

	retval = wait_for_completion_timeout(&report_complete,
			msecs_to_jiffies(timeout));
	if (retval == 0) {
		OVT_LOG_ERR(
				"Timed out waiting for report collection");
	} else {
		completed = true;
	}

	LOCK_BUFFER(testing_hcd->out);

	testing_hcd->out.buf[0] = testing_hcd->report_type;

	LOCK_BUFFER(testing_hcd->resp);

	retval = tcm_hcd->write_message(tcm_hcd,
			CMD_DISABLE_REPORT,
			testing_hcd->out.buf,
			1,
			&testing_hcd->resp.buf,
			&testing_hcd->resp.buf_size,
			&testing_hcd->resp.data_length,
			NULL,
			0);
	if (retval < 0) {
		OVT_LOG_ERR(
				"Failed to write command %s\n",
				STR(CMD_DISABLE_REPORT));
		UNLOCK_BUFFER(testing_hcd->resp);
		UNLOCK_BUFFER(testing_hcd->out);
		goto exit;
	}

	UNLOCK_BUFFER(testing_hcd->resp);
	UNLOCK_BUFFER(testing_hcd->out);

	if (completed)
		retval = 0;
	else
		retval = -EIO;

exit:
	testing_hcd->report_type = 0;

	return retval;
}

static void testing_get_frame_size_words(unsigned int *size, bool image_only)
{
	unsigned int rows;
	unsigned int cols;
	unsigned int hybrid;
	unsigned int buttons;
	struct ovt_tcm_app_info *app_info;
	struct ovt_tcm_hcd *tcm_hcd = testing_hcd->tcm_hcd;

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

static void testing_standard_frame_output(bool image_only)
{
	int retval;
	unsigned int data_size;
	unsigned int header_size;
	unsigned int output_size;
	struct ovt_tcm_app_info *app_info;
	struct ovt_tcm_hcd *tcm_hcd = testing_hcd->tcm_hcd;

	app_info = &tcm_hcd->app_info;

	testing_get_frame_size_words(&data_size, image_only);

	header_size = sizeof(app_info->num_of_buttons) +
			sizeof(app_info->num_of_image_rows) +
			sizeof(app_info->num_of_image_cols) +
			sizeof(app_info->has_hybrid_data);

	output_size = header_size + data_size * 2;

	LOCK_BUFFER(testing_hcd->output);

	retval = ovt_tcm_alloc_mem(tcm_hcd,
			&testing_hcd->output,
			output_size);
	if (retval < 0) {
		OVT_LOG_ERR(
				"Failed to allocate memory for testing_hcd->output.buf");
		UNLOCK_BUFFER(testing_hcd->output);
		return;
	}

	retval = secure_memcpy(testing_hcd->output.buf,
			testing_hcd->output.buf_size,
			&app_info->num_of_buttons[0],
			header_size,
			header_size);
	if (retval < 0) {
		OVT_LOG_ERR(
				"Failed to copy header data");
		UNLOCK_BUFFER(testing_hcd->output);
		return;
	}

	output_size = header_size;

	LOCK_BUFFER(testing_hcd->resp);

	retval = secure_memcpy(testing_hcd->output.buf + header_size,
			testing_hcd->output.buf_size - header_size,
			testing_hcd->resp.buf,
			testing_hcd->resp.buf_size,
			testing_hcd->resp.data_length);
	if (retval < 0) {
		OVT_LOG_ERR(
				"Failed to copy test data");
		UNLOCK_BUFFER(testing_hcd->resp);
		UNLOCK_BUFFER(testing_hcd->output);
		return;
	}

	output_size += testing_hcd->resp.data_length;

	UNLOCK_BUFFER(testing_hcd->resp);

	testing_hcd->output.data_length = output_size;

	UNLOCK_BUFFER(testing_hcd->output);

	return;
}

static int testing_device_id(void)
{
	struct ovt_tcm_hcd *tcm_hcd = testing_hcd->tcm_hcd;
	struct ovt_tcm_identification *id_info;
	char *strptr = NULL;

	OVT_LOG_INFO(
			"Start testing");
	testing_hcd->result = false;

	id_info = &tcm_hcd->id_info;

	strptr = strnstr(id_info->part_number,
					device_id_limit,
					sizeof(id_info->part_number));
	if (strptr != NULL)
		testing_hcd->result = true;
	else
		OVT_LOG_ERR(
				"Device ID is mismatching, FW: %s (%s)\n",
				id_info->part_number, device_id_limit);

	OVT_LOG_INFO(
			"Result = %s\n", (testing_hcd->result)?"pass":"fail");
	return 0;
}

static int testing_config_id(void)
{
	struct ovt_tcm_hcd *tcm_hcd = testing_hcd->tcm_hcd;
	struct ovt_tcm_app_info *app_info;
	int i;

	OVT_LOG_INFO(
			"Start testing");
	testing_hcd->result = false;

	app_info = &tcm_hcd->app_info;

	testing_hcd->result = true;
	for (i = 0; i < sizeof(config_id_limit); i++) {
		if (config_id_limit[i] !=
				tcm_hcd->app_info.customer_config_id[i]) {
			OVT_LOG_ERR(
					"Config ID is mismatching at byte %d\n",
					i);
			testing_hcd->result = false;
		}
	}

	OVT_LOG_INFO(
			"Result = %s\n", (testing_hcd->result)?"pass":"fail");
	return 0;
}

#ifdef PT1_GET_PIN_ASSIGNMENT

static int testing_get_static_config(unsigned char *buf, unsigned int buf_len)
{
	int retval;
	struct ovt_tcm_hcd *tcm_hcd = testing_hcd->tcm_hcd;

	if (!buf) {
		OVT_LOG_ERR(
				"invalid parameter");
		return -EINVAL;
	}

	LOCK_BUFFER(testing_hcd->out);
	LOCK_BUFFER(testing_hcd->resp);

	retval = tcm_hcd->write_message(tcm_hcd,
					CMD_GET_STATIC_CONFIG,
					NULL,
					0,
					&testing_hcd->resp.buf,
					&testing_hcd->resp.buf_size,
					&testing_hcd->resp.data_length,
					NULL,
					0);
	if (retval < 0) {
		OVT_LOG_ERR(
				"Failed to write command %s\n",
				STR(CMD_GET_STATIC_CONFIG));
		goto exit;
	}

	if (testing_hcd->resp.data_length != buf_len) {
		OVT_LOG_ERR(
				"Cfg size mismatch");
		retval = -EINVAL;
		goto exit;
	}

	retval = secure_memcpy(buf,
				buf_len,
				testing_hcd->resp.buf,
				testing_hcd->resp.buf_size,
				buf_len);
	if (retval < 0) {
		OVT_LOG_ERR(
				"Failed to copy cfg data");
		goto exit;
	}

exit:
	UNLOCK_BUFFER(testing_hcd->resp);
	UNLOCK_BUFFER(testing_hcd->out);

	return retval;
}

static bool testing_is_pins_assigned(int pin)
{
	int i;
	short *tx_pins = testing_hcd->tx_pins;
	short tx_assigned = testing_hcd->tx_assigned;
	short *rx_pins = testing_hcd->rx_pins;
	short rx_assigned = testing_hcd->rx_assigned;
	short *guard_pins = testing_hcd->guard_pins;
	short guard_assigned = testing_hcd->guard_assigned;

	for (i = 0; i < tx_assigned; i++) {
		if (pin == tx_pins[i])
			return true;
	}
	for (i = 0; i < rx_assigned; i++) {
		if (pin == rx_pins[i])
			return true;
	}
	for (i = 0; i < guard_assigned; i++) {
		if (pin == guard_pins[i])
			return true;
	}

	return false;
}

static int testing_get_pins_mapping(unsigned char *cfg_data, int length)
{
	struct ovt_tcm_hcd *tcm_hcd = testing_hcd->tcm_hcd;
	int i, j = 0;
	int idx;

	int offset_tx_pin = cfg_imagetxes.offset/8;
	int length_tx_pin = cfg_imagetxes.length/8;
	int offset_rx_pin = cfg_imagerxes.offset/8;
	int length_rx_pin = cfg_imagerxes.length/8;

	int num_tx_guard = 0;
	int offset_num_tx_guard = cfg_numtxguards.offset/8;
	int length_num_tx_guard = cfg_numtxguards.length/8;
	int offset_tx_guard = cfg_txguardpins.offset/8;
	int length_tx_guard = cfg_txguardpins.length/8;

	int num_rx_guard = 0;
	int offset_num_rx_guard = cfg_numrxguards.offset/8;
	int length_num_rx_guard = cfg_numrxguards.length/8;
	int offset_rx_guard = cfg_rxguardpins.offset/8;
	int length_rx_guard = cfg_rxguardpins.length/8;

	if (!cfg_data) {
		OVT_LOG_ERR(
				"invalid parameter");
		return -EINVAL;
	}

	testing_hcd->tx_assigned = 0;
	testing_hcd->rx_assigned = 0;
	testing_hcd->guard_assigned = 0;

	/* get tx pins mapping */
	if (0 == (offset_tx_pin + length_tx_pin))
		goto get_rx_pins;

	if (length > offset_tx_pin + length_tx_pin) {

		testing_hcd->tx_assigned = (length_tx_pin/2);

		idx = 0;
		for (i = 0; i < (length_tx_pin/2); i++) {
			testing_hcd->tx_pins[i] =
				(short)cfg_data[offset_tx_pin + idx] |
				(short)(cfg_data[offset_tx_pin + idx + 1] << 8);
			idx += 2;

			LOGD(tcm_hcd->pdev->dev.parent,
					"tx[%d] = %2d\n",
					i, testing_hcd->tx_pins[i]);
		}
	}

get_rx_pins:
	/* get rx pins mapping */
	if (0 == (offset_rx_pin + length_rx_pin))
		goto get_num_tx_guards;

	if (length > offset_rx_pin + length_rx_pin) {

		testing_hcd->rx_assigned = (length_rx_pin/2);

		idx = 0;
		for (i = 0; i < (length_rx_pin/2); i++) {
			testing_hcd->rx_pins[i] =
				(short)cfg_data[offset_rx_pin + idx] |
				(short)(cfg_data[offset_rx_pin + idx + 1] << 8);
			idx += 2;

			LOGD(tcm_hcd->pdev->dev.parent,
					"rx[%d] = %2d\n",
					i, testing_hcd->rx_pins[i]);
		}
	}

get_num_tx_guards:
	/* get number of tx guards */
	if (0 == (offset_num_tx_guard + length_num_tx_guard))
		goto get_num_rx_guards;

	if (length > offset_num_tx_guard + length_num_tx_guard) {

		num_tx_guard = (short)cfg_data[offset_num_tx_guard] |
				(short)(cfg_data[offset_num_tx_guard + 1] << 8);

		testing_hcd->guard_assigned += num_tx_guard;
	}

get_num_rx_guards:
	/* get number of rx guards */
	if (0 == (offset_num_rx_guard + length_num_rx_guard))
		goto get_guards;

	if (length > offset_num_rx_guard + length_num_rx_guard) {

		num_rx_guard = (short)cfg_data[offset_num_rx_guard] |
				(short)(cfg_data[offset_num_rx_guard + 1] << 8);

		testing_hcd->guard_assigned += num_rx_guard;
	}

get_guards:
	if (testing_hcd->guard_assigned > 0)
		LOGD(tcm_hcd->pdev->dev.parent,
				"num of guards = %2d\n",
				testing_hcd->guard_assigned);

	/* get tx guards mapping */
	if ((num_tx_guard > 0) &&
		(length > offset_tx_guard + length_tx_guard)) {
		idx = 0;
		for (i = 0; i < num_tx_guard; i++) {
			testing_hcd->guard_pins[j] =
			  (short)cfg_data[offset_tx_guard + idx] |
			  (short)(cfg_data[offset_tx_guard + idx + 1] << 8);

			LOGD(tcm_hcd->pdev->dev.parent,
					"guard_pins[%d] = %2d\n",
					i, testing_hcd->guard_pins[j]);
			idx += 2;
			j += 1;
		}
	}

	/* get rx guards mapping */
	if ((num_rx_guard > 0) &&
		(length > offset_rx_guard + length_rx_guard)) {
		idx = 0;
		for (i = 0; i < num_rx_guard; i++) {
			testing_hcd->guard_pins[j] =
			  (short)cfg_data[offset_rx_guard + idx] |
			  (short)(cfg_data[offset_rx_guard + idx + 1] << 8);

			LOGD(tcm_hcd->pdev->dev.parent,
					"guard_pins[%d] = %2d\n",
					i, testing_hcd->guard_pins[j]);
			idx += 2;
			j += 1;
		}
	}

	return 0;
}

#endif /* end of PT1_GET_PIN_ASSIGNMENT */

static int testing_pt01_trx_trx_short(void)
{
	int retval;
	int i, j;
	int phy_pin;
	bool do_pin_test = false;
	struct ovt_tcm_app_info *app_info;
	struct ovt_tcm_hcd *tcm_hcd = testing_hcd->tcm_hcd;
	unsigned int size;
	unsigned char limit;
	unsigned char *buf;
	unsigned char data;
#ifdef PT1_GET_PIN_ASSIGNMENT
	unsigned char *satic_cfg_buf = NULL;
	unsigned int satic_cfg_length;
#endif

	OVT_LOG_INFO(
			"Start testing");
	testing_hcd->result = false;

	app_info = &tcm_hcd->app_info;

#ifdef PT1_GET_PIN_ASSIGNMENT
	satic_cfg_length = le2_to_uint(app_info->static_config_size);

	if (!testing_hcd->satic_cfg_buf) {
		satic_cfg_buf = kzalloc(satic_cfg_length, GFP_KERNEL);
		if (!satic_cfg_buf) {
			OVT_LOG_ERR(
					"Failed to allocate memory for satic_cfg_buf");
			goto exit;
		}

		retval = testing_get_static_config(satic_cfg_buf,
						satic_cfg_length);
		if (retval < 0) {
			OVT_LOG_ERR(
					"Failed to get static config");
			goto exit;
		}

		testing_hcd->satic_cfg_buf = satic_cfg_buf;
	}

	if ((testing_hcd->tx_assigned <= 0) ||
		(testing_hcd->rx_assigned <= 0) ||
		(testing_hcd->guard_assigned <= 0)) {

		if (!testing_hcd->satic_cfg_buf) {
			OVT_LOG_ERR(
					"Failed to get proper satic_cfg");
			goto exit;
		}

		retval = testing_get_pins_mapping(testing_hcd->satic_cfg_buf,
							satic_cfg_length);
		if (retval < 0) {
			OVT_LOG_ERR(
					"Failed to get pins mapping");
			goto exit;
		}
	}
#endif

	retval = testing_run_prod_test_item(TEST_PT1_TRX_TRX_SHORTS);
	if (retval < 0) {
		OVT_LOG_ERR(
				"Failed to run test");
		return -EIO;
	}

	LOCK_BUFFER(testing_hcd->resp);

	size =
		sizeof(pt1_limits) / sizeof(pt1_limits[0]);

	if (size < testing_hcd->resp.data_length) {
		OVT_LOG_ERR(
				"Mismatching limits data");
		UNLOCK_BUFFER(testing_hcd->resp);
		retval = -EINVAL;
		goto exit;
	}

	buf = testing_hcd->resp.buf;
	testing_hcd->result = true;

	for (i = 0; i < testing_hcd->resp.data_length; i++) {

		data = buf[i];
		LOGD(tcm_hcd->pdev->dev.parent,
				"[%d]: 0x%02x\n", i, data);
		for (j = 0; j < 8; j++) {

			phy_pin = (i*8 + j);

#ifdef PT1_GET_PIN_ASSIGNMENT
			do_pin_test = testing_is_pins_assigned(phy_pin);
#else
			do_pin_test = true;
#endif
			limit = CHECK_BIT(pt1_limits[i], j);
			if (do_pin_test) {
				if (CHECK_BIT(data, j) != limit) {
					OVT_LOG_ERR(
							"pin-%2d : fail\n",
							phy_pin);
					testing_hcd->result = false;
				} else
					LOGD(tcm_hcd->pdev->dev.parent,
							"pin-%2d : pass\n",
							phy_pin);
			}
		}
	}

	UNLOCK_BUFFER(testing_hcd->resp);

exit:
#ifdef PT1_GET_PIN_ASSIGNMENT
	kfree(satic_cfg_buf);
#endif

	if (tcm_hcd->features.dual_firmware) {
		if (tcm_hcd->reset(tcm_hcd) < 0) {
			OVT_LOG_ERR(
					"Failed to do reset");
		}
	}

	OVT_LOG_INFO(
			"Result = %s\n", (testing_hcd->result)?"pass":"fail");
	return retval;
}

static int testing_pt05_full_raw(void)
{
	int retval;
	unsigned char *buf;
	unsigned int idx;
	unsigned int row;
	unsigned int col;
	unsigned int rows;
	unsigned int cols;
	unsigned int limits_rows;
	unsigned int limits_cols;
	unsigned int frame_size;
	struct ovt_tcm_app_info *app_info;
	struct ovt_tcm_hcd *tcm_hcd = testing_hcd->tcm_hcd;
	unsigned short data;

	OVT_LOG_INFO(
			"Start testing");
	testing_hcd->result = false;

	app_info = &tcm_hcd->app_info;

	rows = le2_to_uint(app_info->num_of_image_rows);
	cols = le2_to_uint(app_info->num_of_image_cols);

	frame_size = rows * cols * 2;

	retval = testing_run_prod_test_item(TEST_PT5_FULL_RAW_CAP);
	if (retval < 0) {
		OVT_LOG_ERR(
				"Failed to run test");
		goto exit;
	}

	LOCK_BUFFER(testing_hcd->resp);

	if (frame_size != testing_hcd->resp.data_length) {
		OVT_LOG_ERR(
				"Frame size mismatch");
		UNLOCK_BUFFER(testing_hcd->resp);
		retval = -EINVAL;
		goto exit;
	}

	limits_rows =
		sizeof(pt5_hi_limits) / sizeof(pt5_hi_limits[0]);
	limits_cols =
		sizeof(pt5_hi_limits[0]) / sizeof(pt5_hi_limits[0][0]);

	if (rows > limits_rows || cols > limits_cols) {
		OVT_LOG_ERR(
				"Mismatching limits data");
		UNLOCK_BUFFER(testing_hcd->resp);
		retval = -EINVAL;
		goto exit;
	}

	limits_rows =
		sizeof(pt5_lo_limits) / sizeof(pt5_lo_limits[0]);
	limits_cols =
		sizeof(pt5_lo_limits[0]) / sizeof(pt5_lo_limits[0][0]);

	if (rows > limits_rows || cols > limits_cols) {
		OVT_LOG_ERR(
				"Mismatching limits data");
		UNLOCK_BUFFER(testing_hcd->resp);
		retval = -EINVAL;
		goto exit;
	}

	buf = testing_hcd->resp.buf;
	testing_hcd->result = true;

	idx = 0;
	for (row = 0; row < rows; row++) {
		for (col = 0; col < cols; col++) {

			data = (unsigned short)(buf[idx] & 0xff) |
					(unsigned short)(buf[idx+1] << 8);

			if (data  > pt5_hi_limits[row][col] ||
					data  < pt5_lo_limits[row][col]) {

				OVT_LOG_ERR(
					"fail at (%2d, %2d) data = %5d, limit = (%4d, %4d)\n",
					row, col, data, pt5_lo_limits[row][col],
					pt5_hi_limits[row][col]);

				testing_hcd->result = false;
			}

			idx += 2;
		}
	}

	UNLOCK_BUFFER(testing_hcd->resp);

	retval = 0;

exit:
	if (tcm_hcd->features.dual_firmware) {
		if (tcm_hcd->reset(tcm_hcd) < 0) {
			OVT_LOG_ERR(
					"Failed to do reset");
		}
	}

	OVT_LOG_INFO(
			"Result = %s\n", (testing_hcd->result)?"pass":"fail");
	return retval;
}


static int testing_pt07_dynamic_range(void)
{
	int retval;
	unsigned char *buf;
	unsigned int idx;
	unsigned int row;
	unsigned int col;
	unsigned int data;
	unsigned int rows;
	unsigned int cols;
	unsigned int limits_rows;
	unsigned int limits_cols;
	unsigned int frame_size_words;
	struct ovt_tcm_app_info *app_info;
	struct ovt_tcm_hcd *tcm_hcd = testing_hcd->tcm_hcd;

	OVT_LOG_INFO(
			"Start testing");
	testing_hcd->result = false;

	app_info = &tcm_hcd->app_info;

	rows = le2_to_uint(app_info->num_of_image_rows);
	cols = le2_to_uint(app_info->num_of_image_cols);

	testing_get_frame_size_words(&frame_size_words, false);

	retval = testing_run_prod_test_item(TEST_PT7_DYNAMIC_RANGE);
	if (retval < 0) {
		OVT_LOG_ERR(
				"Failed to run test");
		goto exit;
	}

	LOCK_BUFFER(testing_hcd->resp);

	if (frame_size_words != testing_hcd->resp.data_length / 2) {
		OVT_LOG_ERR(
				"Frame size mismatch");
		UNLOCK_BUFFER(testing_hcd->resp);
		retval = -EINVAL;
		goto exit;
	}

	limits_rows =
		sizeof(pt7_hi_limits) / sizeof(pt7_hi_limits[0]);
	limits_cols =
		sizeof(pt7_hi_limits[0]) / sizeof(pt7_hi_limits[0][0]);

	if (rows > limits_rows || cols > limits_cols) {
		OVT_LOG_ERR(
				"Mismatching limits data");
		UNLOCK_BUFFER(testing_hcd->resp);
		retval = -EINVAL;
		goto exit;
	}

	limits_rows =
		sizeof(pt7_lo_limits) / sizeof(pt7_lo_limits[0]);
	limits_cols =
		sizeof(pt7_lo_limits[0]) / sizeof(pt7_lo_limits[0][0]);

	if (rows > limits_rows || cols > limits_cols) {
		OVT_LOG_ERR(
				"Mismatching limits data");
		UNLOCK_BUFFER(testing_hcd->resp);
		retval = -EINVAL;
		goto exit;
	}

	idx = 0;
	buf = testing_hcd->resp.buf;
	testing_hcd->result = true;

	for (row = 0; row < rows; row++) {
		for (col = 0; col < cols; col++) {
			data = le2_to_uint(&buf[idx * 2]);
			if (data > pt7_hi_limits[row][col] ||
					data < pt7_lo_limits[row][col]) {

				OVT_LOG_ERR(
					"fail at (%2d, %2d) data = %5d, limit = (%4d, %4d)\n",
					row, col, data, pt7_lo_limits[row][col],
					pt7_hi_limits[row][col]);

				testing_hcd->result = false;
			}
			idx++;
		}
	}

	UNLOCK_BUFFER(testing_hcd->resp);

	testing_standard_frame_output(false);

	retval = 0;

exit:
	if (tcm_hcd->features.dual_firmware) {
		if (tcm_hcd->reset(tcm_hcd) < 0) {
			OVT_LOG_ERR(
					"Failed to do reset");
		}
	}

	OVT_LOG_INFO(
			"Result = %s\n", (testing_hcd->result)?"pass":"fail");
	return retval;
}

static int testing_pt10_noise(void)
{
	int retval;
	short data;
	unsigned char *buf;
	unsigned int idx;
	unsigned int row;
	unsigned int col;
	unsigned int rows;
	unsigned int cols;
	unsigned int limits_rows;
	unsigned int limits_cols;
	unsigned int frame_size_words;
	struct ovt_tcm_app_info *app_info;
	struct ovt_tcm_hcd *tcm_hcd = testing_hcd->tcm_hcd;

	OVT_LOG_INFO(
			"Start testing");
	testing_hcd->result = false;

	app_info = &tcm_hcd->app_info;

	rows = le2_to_uint(app_info->num_of_image_rows);
	cols = le2_to_uint(app_info->num_of_image_cols);

	testing_get_frame_size_words(&frame_size_words, true);

	retval = testing_run_prod_test_item(TEST_PT10_DELTA_NOISE);
	if (retval < 0) {
		OVT_LOG_ERR(
				"Failed to run test");
		goto exit;
	}

	LOCK_BUFFER(testing_hcd->resp);

	if (frame_size_words != testing_hcd->resp.data_length / 2) {
		OVT_LOG_ERR(
				"Frame size mismatch");
		UNLOCK_BUFFER(testing_hcd->resp);
		retval = -EINVAL;
		goto exit;
	}

	limits_rows =
		sizeof(pt10_limits) / sizeof(pt10_limits[0]);
	limits_cols =
		sizeof(pt10_limits[0]) / sizeof(pt10_limits[0][0]);

	if (rows > limits_rows || cols > limits_cols) {
		OVT_LOG_ERR(
				"Mismatching limits data");
		UNLOCK_BUFFER(testing_hcd->resp);
		retval = -EINVAL;
		goto exit;
	}

	idx = 0;
	buf = testing_hcd->resp.buf;
	testing_hcd->result = true;

	for (row = 0; row < rows; row++) {
		for (col = 0; col < cols; col++) {
			data = (short)le2_to_uint(&buf[idx * 2]);
			if (data > pt10_limits[row][col]) {

				OVT_LOG_ERR(
					"fail at (%2d, %2d) data = %5d, limit = %4d\n",
					row, col, data, pt10_limits[row][col]);

				testing_hcd->result = false;
			}
			idx++;
		}
	}

	UNLOCK_BUFFER(testing_hcd->resp);

	testing_standard_frame_output(false);

	retval = 0;

exit:
	if (tcm_hcd->features.dual_firmware) {
		if (tcm_hcd->reset(tcm_hcd) < 0) {
			OVT_LOG_ERR(
					"Failed to do reset");
		}
	}

	OVT_LOG_INFO(
			"Result = %s\n", (testing_hcd->result)?"pass":"fail");
	return retval;
}


static int testing_pt11_open_detection(void)
{
	int retval;
	short data;
	unsigned char *buf;
	unsigned int idx;
	unsigned int row;
	unsigned int col;
	unsigned int rows;
	unsigned int cols;
	unsigned int limits_rows;
	unsigned int limits_cols;
	unsigned int image_size_words;
	struct ovt_tcm_app_info *app_info;
	struct ovt_tcm_hcd *tcm_hcd = testing_hcd->tcm_hcd;

	OVT_LOG_INFO(
			"Start testing");
	testing_hcd->result = false;

	app_info = &tcm_hcd->app_info;

	rows = le2_to_uint(app_info->num_of_image_rows);
	cols = le2_to_uint(app_info->num_of_image_cols);

	testing_get_frame_size_words(&image_size_words, true);

	retval = testing_run_prod_test_item(TEST_PT11_OPEN_DETECTION);
	if (retval < 0) {
		OVT_LOG_ERR(
				"Failed to run test");
		goto exit;
	}

	LOCK_BUFFER(testing_hcd->resp);

	if (image_size_words != testing_hcd->resp.data_length / 2) {
		OVT_LOG_ERR(
				"Image size mismatch");
		UNLOCK_BUFFER(testing_hcd->resp);
		retval = -EINVAL;
		goto exit;
	}

	limits_rows =
		sizeof(pt11_hi_limits) / sizeof(pt11_hi_limits[0]);
	limits_cols =
		sizeof(pt11_hi_limits[0]) / sizeof(pt11_hi_limits[0][0]);

	if (rows > limits_rows || cols > limits_cols) {
		OVT_LOG_ERR(
				"Mismatching limits data");
		UNLOCK_BUFFER(testing_hcd->resp);
		retval = -EINVAL;
		goto exit;
	}

	limits_rows =
		sizeof(pt11_lo_limits) / sizeof(pt11_lo_limits[0]);
	limits_cols =
		sizeof(pt11_lo_limits[0]) / sizeof(pt11_lo_limits[0][0]);

	if (rows > limits_rows || cols > limits_cols) {
		OVT_LOG_ERR(
				"Mismatching limits data");
		UNLOCK_BUFFER(testing_hcd->resp);
		retval = -EINVAL;
		goto exit;
	}

	idx = 0;
	buf = testing_hcd->resp.buf;
	testing_hcd->result = true;

	for (row = 0; row < rows; row++) {
		for (col = 0; col < cols; col++) {
			data = (short)le2_to_uint(&buf[idx * 2]);
			if (data > pt11_hi_limits[row][col] ||
					data < pt11_lo_limits[row][col]) {

				OVT_LOG_ERR(
					"fail at (%2d, %2d) data = %5d, limit = (%4d, %4d)\n",
					row, col, data,
					pt11_lo_limits[row][col],
					pt11_hi_limits[row][col]);

				testing_hcd->result = false;
			}
			idx++;
		}
	}

	UNLOCK_BUFFER(testing_hcd->resp);

	testing_standard_frame_output(true);

	retval = 0;

exit:
	if (tcm_hcd->features.dual_firmware) {
		if (tcm_hcd->reset(tcm_hcd) < 0) {
			OVT_LOG_ERR(
					"Failed to do reset");
		}
	}

	OVT_LOG_INFO(
			"Result = %s\n", (testing_hcd->result)?"pass":"fail");
	return retval;
}


static int testing_reset_open(void)
{
	int retval;
	struct ovt_tcm_hcd *tcm_hcd = testing_hcd->tcm_hcd;
	const struct ovt_tcm_board_data *bdata = tcm_hcd->hw_if->bdata;

	OVT_LOG_INFO(
			"Start testing");
	testing_hcd->result = false;

	if (bdata->reset_gpio < 0) {
		OVT_LOG_ERR(
				"Hardware reset unavailable");
		return -EINVAL;
	}

	mutex_lock(&tcm_hcd->reset_mutex);

#ifdef WATCHDOG_SW
	tcm_hcd->update_watchdog(tcm_hcd, false);
#endif

	gpio_set_value(bdata->reset_gpio, bdata->reset_on_state);
	msleep(bdata->reset_active_ms);
	gpio_set_value(bdata->reset_gpio, !bdata->reset_on_state);
	msleep(bdata->reset_delay_ms);

#ifdef WATCHDOG_SW
	tcm_hcd->update_watchdog(tcm_hcd, true);
#endif

	mutex_unlock(&tcm_hcd->reset_mutex);

	if (tcm_hcd->id_info.mode == MODE_APPLICATION_FIRMWARE) {
		retval = tcm_hcd->switch_mode(tcm_hcd, FW_MODE_BOOTLOADER);
		if (retval < 0) {
			OVT_LOG_ERR(
					"Failed to enter bootloader mode");
			return retval;
		}
	} else {
		retval = tcm_hcd->identify(tcm_hcd, false);
		if (retval < 0) {
			OVT_LOG_ERR(
					"Failed to do identification");
			goto run_app_firmware;
		}
	}

	if (tcm_hcd->boot_info.last_reset_reason == reset_open_limit)
		testing_hcd->result = true;
	else
		testing_hcd->result = false;

	retval = 0;

run_app_firmware:
	if (tcm_hcd->switch_mode(tcm_hcd, FW_MODE_APPLICATION) < 0) {
		OVT_LOG_ERR(
				"Failed to run application firmware");
	}

	OVT_LOG_INFO(
			"Result = %s\n", (testing_hcd->result)?"pass":"fail");
	return retval;
}


static void testing_report(void)
{
	int retval;
	unsigned int offset;
	unsigned int report_size;
	struct ovt_tcm_hcd *tcm_hcd = testing_hcd->tcm_hcd;

	report_size = tcm_hcd->report.buffer.data_length;

	LOCK_BUFFER(testing_hcd->report);

	if (testing_hcd->report_index == 0) {
		retval = ovt_tcm_alloc_mem(tcm_hcd,
				&testing_hcd->report,
				report_size * testing_hcd->num_of_reports);
		if (retval < 0) {
			OVT_LOG_ERR(
					"Failed to allocate memory for testing_hcd->report.buf");
			UNLOCK_BUFFER(testing_hcd->report);
			return;
		}
	}

	if (testing_hcd->report_index < testing_hcd->num_of_reports) {
		offset = report_size * testing_hcd->report_index;

		retval = secure_memcpy(testing_hcd->report.buf + offset,
				testing_hcd->report.buf_size - offset,
				tcm_hcd->report.buffer.buf,
				tcm_hcd->report.buffer.buf_size,
				tcm_hcd->report.buffer.data_length);
		if (retval < 0) {
			OVT_LOG_ERR(
					"Failed to copy report data");
			UNLOCK_BUFFER(testing_hcd->report);
			return;
		}

		testing_hcd->report_index++;
		testing_hcd->report.data_length += report_size;
	}

	UNLOCK_BUFFER(testing_hcd->report);

	if (testing_hcd->report_index == testing_hcd->num_of_reports)
		complete(&report_complete);

	return;
}
#endif


#if 0
static int testing_remove(struct ovt_tcm_hcd *tcm_hcd)
{
	int idx;

	if (!testing_hcd)
		goto exit;

	sysfs_remove_bin_file(testing_hcd->sysfs_dir, &bin_attr);

	for (idx = 0; idx < ARRAY_SIZE(attrs); idx++)
		sysfs_remove_file(testing_hcd->sysfs_dir, &(*attrs[idx]).attr);

	kobject_put(testing_hcd->sysfs_dir);

	RELEASE_BUFFER(testing_hcd->output);
	RELEASE_BUFFER(testing_hcd->process);
	RELEASE_BUFFER(testing_hcd->report);
	RELEASE_BUFFER(testing_hcd->resp);
	RELEASE_BUFFER(testing_hcd->out);

	kfree(testing_hcd);
	testing_hcd = NULL;

exit:
	complete(&testing_remove_complete);

	return 0;
}

static int testing_reinit(struct ovt_tcm_hcd *tcm_hcd)
{
	int retval;

	if (!testing_hcd) {
		retval = testing_init(tcm_hcd);
		return retval;
	}

	return 0;
}

static int testing_syncbox(struct ovt_tcm_hcd *tcm_hcd)
{
	if (!testing_hcd)
		return 0;

	if (tcm_hcd->report.id == testing_hcd->report_type)
		testing_report();

	return 0;
}

static struct ovt_tcm_module_cb testing_module = {
	.type = TCM_TESTING,
	.init = testing_init,
	.remove = testing_remove,
	.syncbox = testing_syncbox,
#ifdef REPORT_NOTIFIER
	.asyncbox = NULL,
#endif
	.reinit = testing_reinit,
	.suspend = NULL,
	.resume = NULL,
	.early_suspend = NULL,
};

static int __init testing_module_init(void)
{
	return ovt_tcm_add_module(&testing_module, true);
}

static void __exit testing_module_exit(void)
{
	ovt_tcm_add_module(&testing_module, false);

	wait_for_completion(&testing_remove_complete);

	return;
}

module_init(testing_module_init);
module_exit(testing_module_exit);

MODULE_AUTHOR("Omnivision, Inc.");
MODULE_DESCRIPTION("Omnivision TCM Testing Module");
MODULE_LICENSE("GPL v2");
#endif