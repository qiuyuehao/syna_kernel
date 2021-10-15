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
#include <linux/kthread.h>
#include <linux/interrupt.h>
#include <linux/regulator/consumer.h>
#include "omnivision_tcm_core.h"

/* #define RESET_ON_RESUME */

/* #define RESUME_EARLY_UNBLANK */

#define RESET_ON_RESUME_DELAY_MS 50

#define PREDICTIVE_READING

#define MIN_READ_LENGTH 9

/* #define FORCE_RUN_APPLICATION_FIRMWARE */

#define NOTIFIER_PRIORITY 2

#define RESPONSE_TIMEOUT_MS 3000

#define APP_STATUS_POLL_TIMEOUT_MS 1000

#define APP_STATUS_POLL_MS 100

#define ENABLE_IRQ_DELAY_MS 20

//#define FALL_BACK_ON_POLLING

#define POLLING_DELAY_MS 20

#define MODE_SWITCH_DELAY_MS 100

#define READ_RETRY_US_MIN 5000

#define READ_RETRY_US_MAX 10000

#define WRITE_DELAY_US_MIN 500

#define WRITE_DELAY_US_MAX 1000

#define DYNAMIC_CONFIG_SYSFS_DIR_NAME "dynamic_config"

#define ROMBOOT_DOWNLOAD_UNIT 16

#define PDT_END_ADDR 0x00ee

#define RMI_UBL_FN_NUMBER 0x35
struct ovt_tcm_hcd *g_tcm_hcd;
static struct ovt_tcm_hcd *tcm_hcd;
static int ovt_tcm_probe(struct platform_device *pdev);
static int ovt_tcm_raw_read(struct ovt_tcm_hcd *tcm_hcd,
		unsigned char *in_buf, unsigned int length);
static int ovt_tcm_sensor_detection(struct ovt_tcm_hcd *tcm_hcd);
static int ovt_tcm_read_message(struct ovt_tcm_hcd *tcm_hcd,
		unsigned char *in_buf, unsigned int length);

static int ovt_tcm_chip_detect(struct ts_kit_platform_data* data)
{
	//after module init register to huawei ts kit, will call this detect, if return NO_ERR, will wakeup ts init task
	int retval = 0;
	unsigned char header = 0x0;

	g_tcm_hcd->ovt_tcm_platform_data = data;
	g_tcm_hcd->ovt_tcm_chip_data->ts_platform_data = data;

	//g_tcm_hcd->ovt_tcm_chip_data->cnode is customer chip dtsi cnode, like synaptics, goodix or omnivision, add this for spi device probe
	data->spi->dev.of_node = g_tcm_hcd->ovt_tcm_chip_data->cnode;

	retval = ovt_tcm_spi_probe(data->spi);
	if (retval < 0) {
		OVT_LOG_ERR("ovt_tcm_chip_detect fail to do spi probe");
		return RESULT_ERR;
	}

	retval = ovt_tcm_probe(ovt_tcm_spi_platform_device);
	if (retval < 0) {
		OVT_LOG_ERR("ovt_tcm_chip_detect fail to do ovt_tcm_probe");
		return RESULT_ERR;		
	}

	gpio_set_value(tcm_hcd->ovt_tcm_platform_data->reset_gpio, 0);
	msleep(5);
	gpio_set_value(tcm_hcd->ovt_tcm_platform_data->reset_gpio, 1);        
	msleep(200);

	ovt_tcm_read_message(g_tcm_hcd, &header, sizeof(header));
	OVT_LOG_INFO("read one byte when sensor detect:%x\n", header);

	if (header == 0xa5) {
		g_tcm_hcd->sensor_type = TYPE_ROMBOOT;
		g_tcm_hcd->is_detected = true;
		return NO_ERR;
	} else {
		return RESULT_ERR;
	}

}
static int ovt_tcm_read_message(struct ovt_tcm_hcd *tcm_hcd,
		unsigned char *in_buf, unsigned int length);

static int ovt_tcm_init_chip(void)
{
	int retval = NO_ERR;
	unsigned char first_message[64];
	OVT_LOG_INFO("ovt_tcm_init_chip enter ");
	//irq not register here now, called by ts init task, after detect ok, before ovt_tcm_fw_update_boot().
	// uint8 r_byte = 0;
	ovt_zeroflash_init(g_tcm_hcd);
	ovt_tcm_device_init(g_tcm_hcd);
	ovt_testing_init(g_tcm_hcd);

	ovt_tcm_read_message(g_tcm_hcd, first_message, sizeof(first_message));
	OVT_LOG_INFO("read first message:%x  %x   %x  %x\n", first_message[0], first_message[1], first_message[2], first_message[3]);
	if (first_message[0] == 0xa5 && first_message[1] == 0x10) {
		secure_memcpy((unsigned char *)&g_tcm_hcd->id_info,
				sizeof(g_tcm_hcd->id_info),
				&first_message[MESSAGE_HEADER_SIZE],
				sizeof(first_message) - MESSAGE_HEADER_SIZE,
				sizeof(g_tcm_hcd->id_info));
	}
	if (tcm_hcd->id_info.mode == MODE_ROMBOOTLOADER) {
		g_tcm_hcd->sensor_type = TYPE_ROMBOOT;
		g_tcm_hcd->is_detected = true;
	}

	return NO_ERR;
}
static int ovt_tcm_fw_update_boot(char *file_name)
{
	//irq register now irq pin should be high now, after ovt_tcm_input_config, should set x, y max again
	int retval = NO_ERR;
	OVT_LOG_INFO("ovt_tcm_fw_update_boot enter ");
	retval = zeroflash_do_hostdownload(g_tcm_hcd);
	if (retval >= 0) {
		retval = g_tcm_hcd->identify(g_tcm_hcd, true);
		retval = ovt_touch_init(g_tcm_hcd);
		ovt_touch_config_input_dev(g_tcm_hcd->input_dev);
	}
	return NO_ERR;
}

static int ovt_tcm_input_config(struct input_dev *input_dev)
{
	//after ovt_tcm_init_chip, before ovt_tcm_fw_update_boot
	OVT_LOG_INFO("ovt_tcm_input_config enter ");
	g_tcm_hcd->input_dev = input_dev;

	ovt_touch_init(g_tcm_hcd);
	ovt_touch_config_input_dev(input_dev);

	return NO_ERR;
}
static int ovt_tcm_irq_bottom_half(struct ts_cmd_node *in_cmd,
				     struct ts_cmd_node *out_cmd);

static int ovt_tcm_resume(void);
static int ovt_tcm_suspend(void);

static int ovt_tcm_mmi_test(struct ts_rawdata_info *info,
				 struct ts_cmd_node *out_cmd)
{
	int retval = 0;
	OVT_LOG_INFO("++++ ovt_tcm_mmi_test in");
	retval = ovt_tcm_testing(info);
	if (retval < 0) {
		OVT_LOG_ERR("Failed to ovt_tcm_testing");
		return retval;
	}		
	return NO_ERR;
}

#define OMNIVISION_CHIP_INFO "omnivision"

static int ovt_tcm_chip_get_info(struct ts_chip_info_param *info)
{
	int retval = 0;
	char buf_fw_ver[16] = {0};
	int projectid_lenth = 0;
	int len = 0;
	struct ovt_tcm_hcd *tcm_hcd = g_tcm_hcd;
	
	if (!tcm_hcd)
		return -EINVAL;

	if (tcm_hcd->ovt_tcm_chip_data->projectid_len) {
		projectid_lenth = tcm_hcd->ovt_tcm_chip_data->projectid_len;
	} else {
		projectid_lenth = 16; // ?
	}
	memset(info->ic_vendor, 0, sizeof(info->ic_vendor));
	memset(info->mod_vendor, 0, sizeof(info->mod_vendor));
	memset(info->fw_vendor, 0, sizeof(info->fw_vendor));


	snprintf(buf_fw_ver, sizeof(buf_fw_ver), "PR%d", tcm_hcd->packrat_number);
	OVT_LOG_ERR("buf_fw_ver = %s", buf_fw_ver);
	strncpy(info->fw_vendor, buf_fw_ver, CHIP_INFO_LENGTH);

	if(!tcm_hcd->ovt_tcm_chip_data->ts_platform_data->hide_plain_id)  {
		len = (sizeof(info->ic_vendor) - 1) > sizeof(OMNIVISION_CHIP_INFO) ?
				sizeof(OMNIVISION_CHIP_INFO) : (sizeof(info->ic_vendor) - 1);
		memcpy(info->ic_vendor, OMNIVISION_CHIP_INFO, len);
	} else {
		if (tcm_hcd->hw_if->bdata->project_id) {
			len = (sizeof(info->ic_vendor) - 1) > strlen(tcm_hcd->hw_if->bdata->project_id) ?
				strlen(tcm_hcd->hw_if->bdata->project_id) : (sizeof(info->ic_vendor) - 1);
			memcpy(info->ic_vendor, tcm_hcd->hw_if->bdata->project_id, len);
		}
	}
	if (tcm_hcd->hw_if->bdata->project_id) {
		strncpy(info->mod_vendor, tcm_hcd->hw_if->bdata->project_id, CHIP_INFO_LENGTH - 1);
	} else {
		strncpy(info->mod_vendor, "NONE_LCD", CHIP_INFO_LENGTH - 1);
	}
	OVT_LOG_ERR("info->mod_vendor:%s, info->ic_vendor = %s, info->fw_vendor = %s", info->mod_vendor, info->ic_vendor,info->fw_vendor);
	return 0;
}
static int ovt_tcm_get_cal_data(struct ts_calibration_data_info *info,
				 struct ts_cmd_node *out_cmd)
{
	struct ovt_tcm_app_info *app_info;
	
	OVT_LOG_INFO("ovt_tcm_get_cal_data called");

	app_info = &g_tcm_hcd->app_info;
	info->tx_num = le2_to_uint(app_info->num_of_image_rows);
	info->rx_num = le2_to_uint(app_info->num_of_image_cols);

	return 0;
}
struct ts_device_ops ts_kit_ovt_tcm_ops = {
	.chip_detect = ovt_tcm_chip_detect,
	.chip_init = ovt_tcm_init_chip,
	//.chip_parse_config = ovt_tcm_parse_dts, did not called by ts kit?
	.chip_input_config = ovt_tcm_input_config,
	.chip_irq_top_half = NULL,
	.chip_irq_bottom_half = ovt_tcm_irq_bottom_half,
	.chip_fw_update_boot = ovt_tcm_fw_update_boot,  // host download on boot
	//.chip_fw_update_sd = ovt_tcm_fw_update_sd,   // host download by hand
//	.oem_info_switch = ovtptics_oem_info_switch,
	.chip_get_info = ovt_tcm_chip_get_info,
//	.chip_get_capacitance_test_type =
//	    ovtptics_chip_get_capacitance_test_type,
	//.chip_set_info_flag = ovt_tcm_set_info_flag,
	//.chip_before_suspend = ovt_tcm_before_suspend,
	.chip_suspend = ovt_tcm_suspend,
	.chip_resume = ovt_tcm_resume,
	//.chip_after_resume = ovt_tcm_after_resume,
//	.chip_wakeup_gesture_enable_switch =
//	    ovtptics_wakeup_gesture_enable_switch,
	.chip_get_rawdata = ovt_tcm_mmi_test,
	.chip_get_calibration_data = ovt_tcm_get_cal_data,
};


#define dynamic_config_sysfs(c_name, id) \
static ssize_t ovt_tcm_sysfs_##c_name##_show(struct device *dev, \
		struct device_attribute *attr, char *buf) \
{ \
	int retval; \
	unsigned short value; \
	struct ovt_tcm_hcd *tcm_hcd; \
\
	tcm_hcd = g_tcm_hcd; \
\
	mutex_lock(&tcm_hcd->extif_mutex); \
\
	retval = tcm_hcd->get_dynamic_config(tcm_hcd, id, &value); \
	if (retval < 0) { \
		OVT_LOG_ERR( \
				"Failed to get dynamic config"); \
		goto exit; \
	} \
\
	retval = snprintf(buf, PAGE_SIZE, "%u\n", value); \
\
exit: \
	mutex_unlock(&tcm_hcd->extif_mutex); \
\
	return retval; \
} \
\
static ssize_t ovt_tcm_sysfs_##c_name##_store(struct device *dev, \
		struct device_attribute *attr, const char *buf, size_t count) \
{ \
	int retval; \
	unsigned int input; \
	struct ovt_tcm_hcd *tcm_hcd; \
\
	tcm_hcd = g_tcm_hcd; \
\
	if (sscanf(buf, "%u", &input) != 1) \
		return -EINVAL; \
\
	mutex_lock(&tcm_hcd->extif_mutex); \
\
	retval = tcm_hcd->set_dynamic_config(tcm_hcd, id, input); \
	if (retval < 0) { \
		OVT_LOG_ERR( \
				"Failed to set dynamic config"); \
		goto exit; \
	} \
\
	retval = count; \
\
exit: \
	mutex_unlock(&tcm_hcd->extif_mutex); \
\
	return retval; \
}

DECLARE_COMPLETION(ovt_response_complete);
DECLARE_COMPLETION(helper_complete);
//static struct kobject *sysfs_dir;

static struct ovt_tcm_module_pool mod_pool;

SHOW_PROTOTYPE(ovt_tcm, info)
SHOW_PROTOTYPE(ovt_tcm, info_appfw)
STORE_PROTOTYPE(ovt_tcm, irq_en)
STORE_PROTOTYPE(ovt_tcm, reset)
#ifdef WATCHDOG_SW
STORE_PROTOTYPE(ovt_tcm, watchdog)
#endif
SHOW_STORE_PROTOTYPE(ovt_tcm, no_doze)
SHOW_STORE_PROTOTYPE(ovt_tcm, disable_noise_mitigation)
SHOW_STORE_PROTOTYPE(ovt_tcm, inhibit_frequency_shift)
SHOW_STORE_PROTOTYPE(ovt_tcm, requested_frequency)
SHOW_STORE_PROTOTYPE(ovt_tcm, disable_hsync)
SHOW_STORE_PROTOTYPE(ovt_tcm, rezero_on_exit_deep_sleep)
SHOW_STORE_PROTOTYPE(ovt_tcm, charger_connected)
SHOW_STORE_PROTOTYPE(ovt_tcm, no_baseline_relaxation)
SHOW_STORE_PROTOTYPE(ovt_tcm, in_wakeup_gesture_mode)
SHOW_STORE_PROTOTYPE(ovt_tcm, stimulus_fingers)
SHOW_STORE_PROTOTYPE(ovt_tcm, grip_suppression_enabled)
SHOW_STORE_PROTOTYPE(ovt_tcm, enable_thick_glove)
SHOW_STORE_PROTOTYPE(ovt_tcm, enable_glove)

static struct device_attribute *attrs[] = {
	ATTRIFY(info),
	ATTRIFY(info_appfw),
	ATTRIFY(irq_en),
	ATTRIFY(reset),
#ifdef WATCHDOG_SW
	ATTRIFY(watchdog),
#endif
};

static struct device_attribute *dynamic_config_attrs[] = {
	ATTRIFY(no_doze),
	ATTRIFY(disable_noise_mitigation),
	ATTRIFY(inhibit_frequency_shift),
	ATTRIFY(requested_frequency),
	ATTRIFY(disable_hsync),
	ATTRIFY(rezero_on_exit_deep_sleep),
	ATTRIFY(charger_connected),
	ATTRIFY(no_baseline_relaxation),
	ATTRIFY(in_wakeup_gesture_mode),
	ATTRIFY(stimulus_fingers),
	ATTRIFY(grip_suppression_enabled),
	ATTRIFY(enable_thick_glove),
	ATTRIFY(enable_glove),
};

static int ovt_tcm_get_app_info(struct ovt_tcm_hcd *tcm_hcd);
static int ovt_tcm_sensor_detection(struct ovt_tcm_hcd *tcm_hcd);
static void ovt_tcm_check_hdl(struct ovt_tcm_hcd *tcm_hcd,
							unsigned char id);

static ssize_t ovt_tcm_sysfs_info_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int retval;
	unsigned int count;
	struct ovt_tcm_hcd *tcm_hcd;

	tcm_hcd = g_tcm_hcd;

	mutex_lock(&tcm_hcd->extif_mutex);

	retval = tcm_hcd->identify(tcm_hcd, true);
	if (retval < 0) {
		OVT_LOG_ERR(
				"Failed to do identification");
		goto exit;
	}

	count = 0;

	retval = snprintf(buf, PAGE_SIZE - count,
			"TouchComm version:  %d\n",
			tcm_hcd->id_info.version);
	if (retval < 0)
		goto exit;

	buf += retval;
	count += retval;

	if (OMNIVISION_TCM_ID_SUBVERSION == 0) {
		retval = snprintf(buf, PAGE_SIZE - count,
				"Driver version:     %d.%d\n",
				(unsigned char)(OMNIVISION_TCM_ID_VERSION >> 8),
				(unsigned char)OMNIVISION_TCM_ID_VERSION);
	} else {
		retval = snprintf(buf, PAGE_SIZE - count,
				"Driver version:     %d.%d.%d\n",
				(unsigned char)(OMNIVISION_TCM_ID_VERSION >> 8),
				(unsigned char)OMNIVISION_TCM_ID_VERSION,
				OMNIVISION_TCM_ID_SUBVERSION);
	}
	if (retval < 0)
		goto exit;

	buf += retval;
	count += retval;

	switch (tcm_hcd->id_info.mode) {
	case MODE_APPLICATION_FIRMWARE:
		retval = snprintf(buf, PAGE_SIZE - count,
				"Firmware mode:      Application Firmware\n");
		if (retval < 0)
			goto exit;
		break;
	case MODE_HOSTDOWNLOAD_FIRMWARE:
		retval = snprintf(buf, PAGE_SIZE - count,
				"Firmware mode:      Host Download Firmware\n");
		if (retval < 0)
			goto exit;
		break;
	case MODE_BOOTLOADER:
		retval = snprintf(buf, PAGE_SIZE - count,
				"Firmware mode:      Bootloader\n");
		if (retval < 0)
			goto exit;
		break;
	case MODE_TDDI_BOOTLOADER:
		retval = snprintf(buf, PAGE_SIZE - count,
				"Firmware mode:      TDDI Bootloader\n");
		if (retval < 0)
			goto exit;
		break;
	case MODE_TDDI_HOSTDOWNLOAD_BOOTLOADER:
		retval = snprintf(buf, PAGE_SIZE - count,
				"Firmware mode:      TDDI Host Download Bootloader\n");
		if (retval < 0)
			goto exit;
		break;
	case MODE_ROMBOOTLOADER:
		retval = snprintf(buf, PAGE_SIZE - count,
				"Firmware mode:      Rom Bootloader\n");
		if (retval < 0)
			goto exit;
		break;
	default:
		retval = snprintf(buf, PAGE_SIZE - count,
				"Firmware mode:      Unknown (%d)\n",
				tcm_hcd->id_info.mode);
		if (retval < 0)
			goto exit;
		break;
	}
	buf += retval;
	count += retval;

	retval = snprintf(buf, PAGE_SIZE - count,
			"Part number:        ");
	if (retval < 0)
		goto exit;

	buf += retval;
	count += retval;

	retval = secure_memcpy(buf,
			PAGE_SIZE - count,
			tcm_hcd->id_info.part_number,
			sizeof(tcm_hcd->id_info.part_number),
			sizeof(tcm_hcd->id_info.part_number));
	if (retval < 0) {
		OVT_LOG_ERR(
				"Failed to copy part number string");
		goto exit;
	}
	buf += sizeof(tcm_hcd->id_info.part_number);
	count += sizeof(tcm_hcd->id_info.part_number);

	retval = snprintf(buf, PAGE_SIZE - count,
			"\n");
	if (retval < 0)
		goto exit;

	buf += retval;
	count += retval;

	retval = snprintf(buf, PAGE_SIZE - count,
			"Packrat number:     %d\n",
			tcm_hcd->packrat_number);
	if (retval < 0)
		goto exit;

	count += retval;

	retval = count;

exit:
	mutex_unlock(&tcm_hcd->extif_mutex);

	return retval;
}

static ssize_t ovt_tcm_sysfs_info_appfw_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int retval;
	unsigned int count;
	struct ovt_tcm_hcd *tcm_hcd;
	int i;

	tcm_hcd = g_tcm_hcd;

	mutex_lock(&tcm_hcd->extif_mutex);
	OVT_LOG_ERR(
				"ovt_tcm_sysfs_info_appfw_show test start,disable irq");
	tcm_hcd->enable_irq(tcm_hcd, false, false);
	{
		unsigned char *resp_buf = NULL;
		unsigned int resp_buf_size = 0;
		unsigned int resp_length = 0;
		unsigned char status_code = 0xf;
		retval = tcm_hcd->write_message(tcm_hcd, 0x20, NULL, 0, &resp_buf, &resp_buf_size, &resp_length,&status_code,10);
		if (retval >= 0) {
			OVT_LOG_ERR(
				"ovt_tcm_sysfs_info_appfw_show test result  resp_buf_size:%d, resp_length:%d, status_code:%d\n",
				resp_buf_size, resp_length, status_code);
		}
		kfree(resp_buf);
	}
	tcm_hcd->enable_irq(tcm_hcd,true, false);

	OVT_LOG_ERR(
				"ovt_tcm_sysfs_info_appfw_show test end,enable irq");
	retval = ovt_tcm_get_app_info(tcm_hcd);
	if (retval < 0) {
		OVT_LOG_ERR(
				"Failed to get app info");
		goto exit;
	}

	count = 0;

	retval = snprintf(buf, PAGE_SIZE - count,
		"app info version:  %d\n",
		le2_to_uint(tcm_hcd->app_info.version));
	if (retval < 0)
		goto exit;

	buf += retval;
	count += retval;

	retval = snprintf(buf, PAGE_SIZE - count,
		"app info status:  %d\n",
		le2_to_uint(tcm_hcd->app_info.status));
	if (retval < 0)
		goto exit;

	buf += retval;
	count += retval;

	retval = snprintf(buf, PAGE_SIZE - count,
		"static config size:  %d\n",
		le2_to_uint(tcm_hcd->app_info.static_config_size));
	if (retval < 0)
		goto exit;

	buf += retval;
	count += retval;

	retval = snprintf(buf, PAGE_SIZE - count,
		"dynamic config size:  %d\n",
		le2_to_uint(tcm_hcd->app_info.dynamic_config_size));
	if (retval < 0)
		goto exit;

	buf += retval;
	count += retval;

	retval = snprintf(buf, PAGE_SIZE - count,
		"app config block:  %d\n",
		le2_to_uint(tcm_hcd->app_info.app_config_start_write_block));
	if (retval < 0)
		goto exit;

	buf += retval;
	count += retval;

	retval = snprintf(buf, PAGE_SIZE - count,
		"app config size:  %d\n",
		le2_to_uint(tcm_hcd->app_info.app_config_size));
	if (retval < 0)
		goto exit;

	buf += retval;
	count += retval;

	retval = snprintf(buf, PAGE_SIZE - count,
		"touch report config max size:  %d\n",
		le2_to_uint(tcm_hcd->app_info.max_touch_report_config_size));
	if (retval < 0)
		goto exit;

	buf += retval;
	count += retval;

	retval = snprintf(buf, PAGE_SIZE - count,
		"touch report payload max size:  %d\n",
		le2_to_uint(tcm_hcd->app_info.max_touch_report_payload_size));
	if (retval < 0)
		goto exit;

	buf += retval;
	count += retval;

	retval = snprintf(buf, PAGE_SIZE - count, "config id:  ");
	if (retval < 0)
		goto exit;

	buf += retval;
	count += retval;

	for (i = 0; i < sizeof(tcm_hcd->app_info.customer_config_id); i++) {
		retval = snprintf(buf, PAGE_SIZE - count,
			"0x%2x ", tcm_hcd->app_info.customer_config_id[i]);
		buf += retval;
		count += retval;
	}

	retval = snprintf(buf, PAGE_SIZE - count, "\n");
	if (retval < 0)
		goto exit;

	buf += retval;
	count += retval;

	retval = snprintf(buf, PAGE_SIZE - count,
		"max x:  %d\n",
		le2_to_uint(tcm_hcd->app_info.max_x));
	if (retval < 0)
		goto exit;

	buf += retval;
	count += retval;

	retval = snprintf(buf, PAGE_SIZE - count,
		"max y:  %d\n",
		le2_to_uint(tcm_hcd->app_info.max_y));
	if (retval < 0)
		goto exit;

	buf += retval;
	count += retval;

	retval = snprintf(buf, PAGE_SIZE - count,
		"max objects:  %d\n",
		le2_to_uint(tcm_hcd->app_info.max_objects));
	if (retval < 0)
		goto exit;

	buf += retval;
	count += retval;

	retval = snprintf(buf, PAGE_SIZE - count,
		"num cols:  %d\n",
		le2_to_uint(tcm_hcd->app_info.num_of_image_cols));
	if (retval < 0)
		goto exit;

	buf += retval;
	count += retval;

	retval = snprintf(buf, PAGE_SIZE - count,
		"num rows:  %d\n",
		le2_to_uint(tcm_hcd->app_info.num_of_image_rows));
	if (retval < 0)
		goto exit;

	buf += retval;
	count += retval;

	retval = snprintf(buf, PAGE_SIZE - count,
		"num buttons:  %d\n",
		le2_to_uint(tcm_hcd->app_info.num_of_buttons));
	if (retval < 0)
		goto exit;

	buf += retval;
	count += retval;

	retval = snprintf(buf, PAGE_SIZE - count,
		"has profile:  %d\n",
		le2_to_uint(tcm_hcd->app_info.has_hybrid_data));
	if (retval < 0)
		goto exit;

	buf += retval;
	count += retval;

	retval = snprintf(buf, PAGE_SIZE - count,
		"num force electrodes:  %d\n",
		le2_to_uint(tcm_hcd->app_info.num_of_force_elecs));
	if (retval < 0)
		goto exit;

	count += retval;

	retval = count;

exit:
	mutex_unlock(&tcm_hcd->extif_mutex);

	return retval;
}

static ssize_t ovt_tcm_sysfs_irq_en_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned int input;
	struct ovt_tcm_hcd *tcm_hcd;

	tcm_hcd = g_tcm_hcd;

	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	mutex_lock(&tcm_hcd->extif_mutex);

	if (input == 0) {
		retval = tcm_hcd->enable_irq(tcm_hcd, false, true);
		if (retval < 0) {
			OVT_LOG_ERR(
					"Failed to disable interrupt");
			goto exit;
		}
	} else if (input == 1) {
		retval = tcm_hcd->enable_irq(tcm_hcd, true, NULL);
		if (retval < 0) {
			OVT_LOG_ERR(
					"Failed to enable interrupt");
			goto exit;
		}
	} else {
		retval = -EINVAL;
		goto exit;
	}

	retval = count;

exit:
	mutex_unlock(&tcm_hcd->extif_mutex);

	return retval;
}

static ssize_t ovt_tcm_sysfs_reset_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int retval;
	bool hw_reset;
	unsigned int input;
	struct ovt_tcm_hcd *tcm_hcd;

	tcm_hcd = g_tcm_hcd;

	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	if (input == 1)
		hw_reset = false;
	else if (input == 2)
		hw_reset = true;
	else
		return -EINVAL;

	mutex_lock(&tcm_hcd->extif_mutex);

	retval = tcm_hcd->reset_n_reinit(tcm_hcd, hw_reset, true);
	if (retval < 0) {
		OVT_LOG_ERR(
				"Failed to do reset and reinit");
		goto exit;
	}

	retval = count;

exit:
	mutex_unlock(&tcm_hcd->extif_mutex);

	return retval;
}

#ifdef WATCHDOG_SW
static ssize_t ovt_tcm_sysfs_watchdog_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int input;
	struct ovt_tcm_hcd *tcm_hcd;

	tcm_hcd = g_tcm_hcd;

	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	if (input != 0 && input != 1)
		return -EINVAL;

	mutex_lock(&tcm_hcd->extif_mutex);

	tcm_hcd->watchdog.run = input;
	tcm_hcd->update_watchdog(tcm_hcd, input);

	mutex_unlock(&tcm_hcd->extif_mutex);

	return count;
}
#endif

dynamic_config_sysfs(no_doze, DC_NO_DOZE)

dynamic_config_sysfs(disable_noise_mitigation, DC_DISABLE_NOISE_MITIGATION)

dynamic_config_sysfs(inhibit_frequency_shift, DC_INHIBIT_FREQUENCY_SHIFT)

dynamic_config_sysfs(requested_frequency, DC_REQUESTED_FREQUENCY)

dynamic_config_sysfs(disable_hsync, DC_DISABLE_HSYNC)

dynamic_config_sysfs(rezero_on_exit_deep_sleep, DC_REZERO_ON_EXIT_DEEP_SLEEP)

dynamic_config_sysfs(charger_connected, DC_CHARGER_CONNECTED)

dynamic_config_sysfs(no_baseline_relaxation, DC_NO_BASELINE_RELAXATION)

dynamic_config_sysfs(in_wakeup_gesture_mode, DC_IN_WAKEUP_GESTURE_MODE)

dynamic_config_sysfs(stimulus_fingers, DC_STIMULUS_FINGERS)

dynamic_config_sysfs(grip_suppression_enabled, DC_GRIP_SUPPRESSION_ENABLED)

dynamic_config_sysfs(enable_thick_glove, DC_ENABLE_THICK_GLOVE)

dynamic_config_sysfs(enable_glove, DC_ENABLE_GLOVE)


int ovt_tcm_add_module(struct ovt_tcm_module_cb *mod_cb, bool insert)
{
	struct ovt_tcm_module_handler *mod_handler;

	if (!mod_pool.initialized) {
		mutex_init(&mod_pool.mutex);
		INIT_LIST_HEAD(&mod_pool.list);
		mod_pool.initialized = true;
	}

	mutex_lock(&mod_pool.mutex);

	if (insert) {
		mod_handler = kzalloc(sizeof(*mod_handler), GFP_KERNEL);
		if (!mod_handler) {
			pr_err("%s: Failed to allocate memory for mod_handler\n",
					__func__);
			mutex_unlock(&mod_pool.mutex);
			return -ENOMEM;
		}
		mod_handler->mod_cb = mod_cb;
		mod_handler->insert = true;
		mod_handler->detach = false;
		list_add_tail(&mod_handler->link, &mod_pool.list);
	} else if (!list_empty(&mod_pool.list)) {
		list_for_each_entry(mod_handler, &mod_pool.list, link) {
			if (mod_handler->mod_cb->type == mod_cb->type) {
				mod_handler->insert = false;
				mod_handler->detach = true;
				goto exit;
			}
		}
	}

exit:
	mutex_unlock(&mod_pool.mutex);

	if (mod_pool.queue_work)
		queue_work(mod_pool.workqueue, &mod_pool.work);

	return 0;
}
EXPORT_SYMBOL(ovt_tcm_add_module);
#if 0
static void ovt_tcm_module_work(struct work_struct *work)
{
	struct ovt_tcm_module_handler *mod_handler;
	struct ovt_tcm_module_handler *tmp_handler;
	struct ovt_tcm_hcd *tcm_hcd = mod_pool.tcm_hcd;

	mutex_lock(&mod_pool.mutex);

	if (!list_empty(&mod_pool.list)) {
		list_for_each_entry_safe(mod_handler,
				tmp_handler,
				&mod_pool.list,
				link) {
			if (mod_handler->insert) {
				if (mod_handler->mod_cb->init)
					mod_handler->mod_cb->init(tcm_hcd);
				mod_handler->insert = false;
			}
			if (mod_handler->detach) {
				if (mod_handler->mod_cb->remove)
					mod_handler->mod_cb->remove(tcm_hcd);
				list_del(&mod_handler->link);
				kfree(mod_handler);
			}
		}
	}

	mutex_unlock(&mod_pool.mutex);

	return;
}
#endif

#ifdef REPORT_NOTIFIER
/**
 * ovt_tcm_report_notifier() - notify occurrence of report received from device
 *
 * @data: handle of core module
 *
 * The occurrence of the report generated by the device is forwarded to the
 * asynchronous inbox of each registered application module.
 */
static int ovt_tcm_report_notifier(void *data)
{
	struct sched_param param = { .sched_priority = NOTIFIER_PRIORITY };
	struct ovt_tcm_module_handler *mod_handler;
	struct ovt_tcm_hcd *tcm_hcd = data;

	sched_setscheduler(current, SCHED_RR, &param);

	set_current_state(TASK_INTERRUPTIBLE);

	while (!kthread_should_stop()) {
		schedule();

		if (kthread_should_stop())
			break;

		set_current_state(TASK_RUNNING);

		mutex_lock(&mod_pool.mutex);

		if (!list_empty(&mod_pool.list)) {
			list_for_each_entry(mod_handler, &mod_pool.list, link) {
				if (!mod_handler->insert &&
						!mod_handler->detach &&
						(mod_handler->mod_cb->asyncbox))
					mod_handler->mod_cb->asyncbox(tcm_hcd);
			}
		}

		mutex_unlock(&mod_pool.mutex);

		set_current_state(TASK_INTERRUPTIBLE);
	};

	return 0;
}
#endif

/**
 * ovt_tcm_dispatch_report() - dispatch report received from device
 *
 * @tcm_hcd: handle of core module
 *
 * The report generated by the device is forwarded to the synchronous inbox of
 * each registered application module for further processing. In addition, the
 * report notifier thread is woken up for asynchronous notification of the
 * report occurrence.
 */

#define FW_LOG_BUFFER_SIZE 2048
static unsigned char fw_log[FW_LOG_BUFFER_SIZE];

static void ovt_tcm_dispatch_report(struct ovt_tcm_hcd *tcm_hcd)
{
	struct ovt_tcm_module_handler *mod_handler;

	LOCK_BUFFER(tcm_hcd->in);
	LOCK_BUFFER(tcm_hcd->report.buffer);

	tcm_hcd->report.buffer.buf = &tcm_hcd->in.buf[MESSAGE_HEADER_SIZE];

	tcm_hcd->report.buffer.buf_size = tcm_hcd->in.buf_size;
	tcm_hcd->report.buffer.buf_size -= MESSAGE_HEADER_SIZE;

	tcm_hcd->report.buffer.data_length = tcm_hcd->payload_length;

	tcm_hcd->report.id = tcm_hcd->status_report_code;

	/* report directly if touch report is received */
	/* if (tcm_hcd->report.id == REPORT_TOUCH) {
		if (tcm_hcd->report_touch)
			tcm_hcd->report_touch();

	} else if (tcm_hcd->report.id == REPORT_FW_PRINTF) {
        int cpy_length;
		if (tcm_hcd->report.buffer.data_length >= FW_LOG_BUFFER_SIZE - 1) {
			cpy_length = FW_LOG_BUFFER_SIZE - 1;
		} else {
			cpy_length = tcm_hcd->report.buffer.data_length;
		}
		memset(fw_log, 0, sizeof(fw_log));
        secure_memcpy(fw_log, FW_LOG_BUFFER_SIZE - 1, tcm_hcd->report.buffer.buf, tcm_hcd->report.buffer.buf_size, cpy_length);
        OVT_LOG_ERR(
				"TouchFWLog: %s\n", fw_log);
    } else {

	} */

	UNLOCK_BUFFER(tcm_hcd->report.buffer);
	UNLOCK_BUFFER(tcm_hcd->in);

	return;
}

/**
 * ovt_tcm_dispatch_response() - dispatch response received from device
 *
 * @tcm_hcd: handle of core module
 *
 * The response to a command is forwarded to the sender of the command.
 */
static void ovt_tcm_dispatch_response(struct ovt_tcm_hcd *tcm_hcd)
{
	int retval;

	if (atomic_read(&tcm_hcd->command_status) != CMD_BUSY)
		return;

	tcm_hcd->response_code = tcm_hcd->status_report_code;

	if (tcm_hcd->payload_length == 0) {
		atomic_set(&tcm_hcd->command_status, CMD_IDLE);
		goto exit;
	}

	LOCK_BUFFER(tcm_hcd->resp);

	retval = ovt_tcm_alloc_mem(tcm_hcd,
			&tcm_hcd->resp,
			tcm_hcd->payload_length);
	if (retval < 0) {
		OVT_LOG_ERR(
				"Failed to allocate memory for tcm_hcd->resp.buf");
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
		OVT_LOG_ERR(
				"Failed to copy payload");
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
	complete(&ovt_response_complete);

	return;
}

/**
 * ovt_tcm_dispatch_message() - dispatch message received from device
 *
 * @tcm_hcd: handle of core module
 *
 * The information received in the message read in from the device is dispatched
 * to the appropriate destination based on whether the information represents a
 * report or a response to a command.
 */
static void ovt_tcm_dispatch_message(struct ovt_tcm_hcd *tcm_hcd)
{
	int retval;
	unsigned char *build_id;
	unsigned int payload_length;
	unsigned int max_write_size;

	if (tcm_hcd->status_report_code == REPORT_IDENTIFY) {
		payload_length = tcm_hcd->payload_length;

		LOCK_BUFFER(tcm_hcd->in);

		retval = secure_memcpy((unsigned char *)&tcm_hcd->id_info,
				sizeof(tcm_hcd->id_info),
				&tcm_hcd->in.buf[MESSAGE_HEADER_SIZE],
				tcm_hcd->in.buf_size - MESSAGE_HEADER_SIZE,
				MIN(sizeof(tcm_hcd->id_info), payload_length));
		if (retval < 0) {
			OVT_LOG_ERR(
					"Failed to copy identification info");
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

		OVT_LOG_INFO(
				"Received identify report (firmware mode = 0x%02x)\n",
				tcm_hcd->id_info.mode);

		if (atomic_read(&tcm_hcd->command_status) == CMD_BUSY) {
			switch (tcm_hcd->command) {
			case CMD_RESET:
			case CMD_RUN_BOOTLOADER_FIRMWARE:
			case CMD_RUN_APPLICATION_FIRMWARE:
			case CMD_ENTER_PRODUCTION_TEST_MODE:
			case CMD_ROMBOOT_RUN_BOOTLOADER_FIRMWARE:
				tcm_hcd->response_code = STATUS_OK;
				atomic_set(&tcm_hcd->command_status, CMD_IDLE);
				complete(&ovt_response_complete);
				break;
			default:
				OVT_LOG_INFO(
						"Device has been reset");
				atomic_set(&tcm_hcd->command_status, CMD_ERROR);
				complete(&ovt_response_complete);
				break;
			}
		} 

		if ((tcm_hcd->id_info.mode == MODE_ROMBOOTLOADER) &&
				tcm_hcd->in_hdl_mode) {

			retval = wait_for_completion_timeout(tcm_hcd->helper.helper_completion,
				msecs_to_jiffies(500));
			if (retval == 0) {
				OVT_LOG_ERR( "timeout to wait for helper completion");
				return;
			}
			if (atomic_read(&tcm_hcd->helper.task) ==
					HELP_NONE) {
				atomic_set(&tcm_hcd->helper.task,
						HELP_SEND_ROMBOOT_HDL);
				queue_work(tcm_hcd->helper.workqueue,
						&tcm_hcd->helper.work);
			} else {
				OVT_LOG_INFO(
						"Helper thread is busy");
			}
			return;
		}

#ifdef FORCE_RUN_APPLICATION_FIRMWARE
		if (IS_NOT_FW_MODE(tcm_hcd->id_info.mode) &&
				!mutex_is_locked(&tcm_hcd->reset_mutex)) {

			if (atomic_read(&tcm_hcd->helper.task) == HELP_NONE) {
				atomic_set(&tcm_hcd->helper.task,
						HELP_RUN_APPLICATION_FIRMWARE);
				queue_work(tcm_hcd->helper.workqueue,
						&tcm_hcd->helper.work);
				return;
			}
		}
#endif

		/* To avoid the identify report dispatching during the HDL. */
		if (atomic_read(&tcm_hcd->host_downloading)) {
			OVT_LOG_INFO(
					"Switched to TCM mode and going to download the configs");
			return;
		}
	}


	if (tcm_hcd->status_report_code >= REPORT_IDENTIFY)
		ovt_tcm_dispatch_report(tcm_hcd);
	else
		ovt_tcm_dispatch_response(tcm_hcd);

	return;
}

/**
 * ovt_tcm_continued_read() - retrieve entire payload from device
 *
 * @tcm_hcd: handle of core module
 *
 * Read transactions are carried out until the entire payload is retrieved from
 * the device and stored in the handle of the core module.
 */
static int ovt_tcm_continued_read(struct ovt_tcm_hcd *tcm_hcd)
{
	int retval;
	unsigned char marker;
	unsigned char code;
	unsigned int idx;
	unsigned int offset;
	unsigned int chunks;
	unsigned int chunk_space;
	unsigned int xfer_length;
	unsigned int total_length;
	unsigned int remaining_length;

	total_length = MESSAGE_HEADER_SIZE + tcm_hcd->payload_length + 1;

	remaining_length = total_length - tcm_hcd->read_length;

	LOCK_BUFFER(tcm_hcd->in);

	retval = ovt_tcm_realloc_mem(tcm_hcd,
			&tcm_hcd->in,
			total_length + 1);
	if (retval < 0) {
		OVT_LOG_ERR(
				"Failed to reallocate memory for tcm_hcd->in.buf");
		UNLOCK_BUFFER(tcm_hcd->in);
		return retval;
	}

	/* available chunk space for payload = total chunk size minus header
	 * marker byte and header code byte */
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

		retval = ovt_tcm_alloc_mem(tcm_hcd,
				&tcm_hcd->temp,
				xfer_length + 2);
		if (retval < 0) {
			OVT_LOG_ERR(
					"Failed to allocate memory for tcm_hcd->temp.buf");
			UNLOCK_BUFFER(tcm_hcd->temp);
			UNLOCK_BUFFER(tcm_hcd->in);
			return retval;
		}

		retval = ovt_tcm_read(tcm_hcd,
				tcm_hcd->temp.buf,
				xfer_length + 2);
		if (retval < 0) {
			OVT_LOG_ERR(
					"Failed to read from device");
			UNLOCK_BUFFER(tcm_hcd->temp);
			UNLOCK_BUFFER(tcm_hcd->in);
			return retval;
		}

		marker = tcm_hcd->temp.buf[0];
		code = tcm_hcd->temp.buf[1];

		if (marker != MESSAGE_MARKER) {
			OVT_LOG_ERR(
					"Incorrect header marker (0x%02x)\n",
					marker);
			UNLOCK_BUFFER(tcm_hcd->temp);
			UNLOCK_BUFFER(tcm_hcd->in);
			return -EIO;
		}

		if (code != STATUS_CONTINUED_READ) {
			OVT_LOG_ERR(
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
			OVT_LOG_ERR(
					"Failed to copy payload");
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
 * ovt_tcm_raw_read() - retrieve specific number of data bytes from device
 *
 * @tcm_hcd: handle of core module
 * @in_buf: buffer for storing data retrieved from device
 * @length: number of bytes to retrieve from device
 *
 * Read transactions are carried out until the specific number of data bytes are
 * retrieved from the device and stored in in_buf.
 */
static int ovt_tcm_raw_read(struct ovt_tcm_hcd *tcm_hcd,
		unsigned char *in_buf, unsigned int length)
{
	int retval;
	unsigned char code;
	unsigned int idx;
	unsigned int offset;
	unsigned int chunks;
	unsigned int chunk_space;
	unsigned int xfer_length;
	unsigned int remaining_length;

	if (length <= 2) {
		bool retry = true;
	retry_read:
		retval = ovt_tcm_read(tcm_hcd,
				in_buf,
				length);
		if (in_buf[0] != MESSAGE_MARKER && retry) {
			retry = false;
			goto retry_read;
		}
		return retval;
	}

	/* minus header marker byte and header code byte */
	remaining_length = length - 2;

	/* available chunk space for data = total chunk size minus header marker
	 * byte and header code byte */
	if (tcm_hcd->rd_chunk_size == 0)
		chunk_space = remaining_length;
	else
		chunk_space = tcm_hcd->rd_chunk_size - 2;

	chunks = ceil_div(remaining_length, chunk_space);

	chunks = chunks == 0 ? 1 : chunks;

	offset = 0;

	LOCK_BUFFER(tcm_hcd->temp);

	for (idx = 0; idx < chunks; idx++) {
		if (remaining_length > chunk_space)
			xfer_length = chunk_space;
		else
			xfer_length = remaining_length;

		if (xfer_length == 1) {
			in_buf[offset] = MESSAGE_PADDING;
			offset += xfer_length;
			remaining_length -= xfer_length;
			continue;
		}

		retval = ovt_tcm_alloc_mem(tcm_hcd,
				&tcm_hcd->temp,
				xfer_length + 2);
		if (retval < 0) {
			OVT_LOG_ERR(
					"Failed to allocate memory for tcm_hcd->temp.buf");
			UNLOCK_BUFFER(tcm_hcd->temp);
			return retval;
		}

		retval = ovt_tcm_read(tcm_hcd,
				tcm_hcd->temp.buf,
				xfer_length + 2);
		if (retval < 0) {
			OVT_LOG_ERR(
					"Failed to read from device");
			UNLOCK_BUFFER(tcm_hcd->temp);
			return retval;
		}

		code = tcm_hcd->temp.buf[1];

		if (idx == 0) {
			retval = secure_memcpy(&in_buf[0],
					length,
					&tcm_hcd->temp.buf[0],
					tcm_hcd->temp.buf_size,
					xfer_length + 2);
		} else {
			if (code != STATUS_CONTINUED_READ) {
				OVT_LOG_ERR(
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
			OVT_LOG_ERR(
					"Failed to copy data");
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
 * ovt_tcm_raw_write() - write command/data to device without receiving
 * response
 *
 * @tcm_hcd: handle of core module
 * @command: command to send to device
 * @data: data to send to device
 * @length: length of data in bytes
 *
 * A command and its data, if any, are sent to the device.
 */
static int ovt_tcm_raw_write(struct ovt_tcm_hcd *tcm_hcd,
		unsigned char command, unsigned char *data, unsigned int length)
{
	int retval;
	unsigned int idx;
	unsigned int chunks;
	unsigned int chunk_space;
	unsigned int xfer_length;
	unsigned int remaining_length;

	remaining_length = length;

	/* available chunk space for data = total chunk size minus command
	 * byte */
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

		retval = ovt_tcm_alloc_mem(tcm_hcd,
				&tcm_hcd->out,
				xfer_length + 1);
		if (retval < 0) {
			OVT_LOG_ERR(
					"Failed to allocate memory for tcm_hcd->out.buf");
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
				OVT_LOG_ERR(
						"Failed to copy data");
				UNLOCK_BUFFER(tcm_hcd->out);
				return retval;
			}
		}

		retval = ovt_tcm_write(tcm_hcd,
				tcm_hcd->out.buf,
				xfer_length + 1);
		if (retval < 0) {
			OVT_LOG_ERR(
					"Failed to write to device");
			UNLOCK_BUFFER(tcm_hcd->out);
			return retval;
		}

		remaining_length -= xfer_length;
	}

	UNLOCK_BUFFER(tcm_hcd->out);

	return 0;
}

/**
 * ovt_tcm_read_message() - read message from device
 *
 * @tcm_hcd: handle of core module
 * @in_buf: buffer for storing data in raw read mode
 * @length: length of data in bytes in raw read mode
 *
 * If in_buf is not NULL, raw read mode is used and ovt_tcm_raw_read() is
 * called. Otherwise, a message including its entire payload is retrieved from
 * the device and dispatched to the appropriate destination.
 */
static int ovt_tcm_read_message(struct ovt_tcm_hcd *tcm_hcd,
		unsigned char *in_buf, unsigned int length)
{
	int retval;
	bool retry;
	unsigned int total_length;
	struct ovt_tcm_message_header *header;

	mutex_lock(&tcm_hcd->rw_ctrl_mutex);

	if (in_buf != NULL) {
		retval = ovt_tcm_raw_read(tcm_hcd, in_buf, length);
		mutex_unlock(&tcm_hcd->rw_ctrl_mutex);
		goto exit;
	}

	retry = true;

retry:
	LOCK_BUFFER(tcm_hcd->in);

	retval = ovt_tcm_read(tcm_hcd,
			tcm_hcd->in.buf,
			tcm_hcd->read_length);
	if (retval < 0) {
		OVT_LOG_ERR(
				"Failed to read from device");
		UNLOCK_BUFFER(tcm_hcd->in);
		if (retry) {
			usleep_range(READ_RETRY_US_MIN, READ_RETRY_US_MAX);
			retry = false;
			goto retry;
		}
		mutex_unlock(&tcm_hcd->rw_ctrl_mutex);
		goto exit;
	}

	header = (struct ovt_tcm_message_header *)tcm_hcd->in.buf;


	if (header->marker != MESSAGE_MARKER) {
		OVT_LOG_ERR(
				"Incorrect header marker (0x%02x)\n",
				header->marker);
		UNLOCK_BUFFER(tcm_hcd->in);
		retval = -ENXIO;
		if (retry) {
			usleep_range(READ_RETRY_US_MIN, READ_RETRY_US_MAX);
			retry = false;
			goto retry;
		}
		mutex_unlock(&tcm_hcd->rw_ctrl_mutex);
		msleep(100);
		goto exit;
	}

	tcm_hcd->status_report_code = header->code;

	tcm_hcd->payload_length = le2_to_uint(header->length);

	OVT_LOG_INFO(
			"Status report code = 0x%02x\n",
			tcm_hcd->status_report_code);

	OVT_LOG_INFO(
			"Payload length = %d\n",
			tcm_hcd->payload_length);

	if (tcm_hcd->status_report_code <= STATUS_ERROR ||
			tcm_hcd->status_report_code == STATUS_INVALID) {
		switch (tcm_hcd->status_report_code) {
		case STATUS_OK:
			break;
		case STATUS_CONTINUED_READ:
			LOGD(tcm_hcd->pdev->dev.parent,
					"Out-of-sync continued read");
		case STATUS_IDLE:
		case STATUS_BUSY:
			tcm_hcd->payload_length = 0;
			UNLOCK_BUFFER(tcm_hcd->in);
			retval = 0;
			mutex_unlock(&tcm_hcd->rw_ctrl_mutex);
			goto exit;
		default:
			OVT_LOG_ERR(
					"Incorrect Status code (0x%02x)\n",
					tcm_hcd->status_report_code);
			if (tcm_hcd->status_report_code == STATUS_INVALID) {
				if (retry) {
					usleep_range(READ_RETRY_US_MIN,
							READ_RETRY_US_MAX);
					retry = false;
					goto retry;
				} else {
					tcm_hcd->payload_length = 0;
				}
			}
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

	retval = ovt_tcm_continued_read(tcm_hcd);
	if (retval < 0) {
		OVT_LOG_ERR(
				"Failed to do continued read");
		mutex_unlock(&tcm_hcd->rw_ctrl_mutex);
		goto exit;
	};

	LOCK_BUFFER(tcm_hcd->in);

	tcm_hcd->in.buf[0] = MESSAGE_MARKER;
	tcm_hcd->in.buf[1] = tcm_hcd->status_report_code;
	tcm_hcd->in.buf[2] = (unsigned char)tcm_hcd->payload_length;
	tcm_hcd->in.buf[3] = (unsigned char)(tcm_hcd->payload_length >> 8);

check_padding:
	if (tcm_hcd->in.buf[total_length - 1] != MESSAGE_PADDING) {
		OVT_LOG_ERR(
				"Incorrect message padding byte (0x%02x)\n",
				tcm_hcd->in.buf[total_length - 1]);
		UNLOCK_BUFFER(tcm_hcd->in);
		retval = -EIO;
		mutex_unlock(&tcm_hcd->rw_ctrl_mutex);
		goto exit;
	}

	UNLOCK_BUFFER(tcm_hcd->in);
	mutex_unlock(&tcm_hcd->rw_ctrl_mutex);
#ifdef PREDICTIVE_READING
	total_length = MAX(total_length, MIN_READ_LENGTH);
	tcm_hcd->read_length = MIN(total_length, tcm_hcd->rd_chunk_size);
	if (tcm_hcd->rd_chunk_size == 0)
		tcm_hcd->read_length = total_length;
#endif
	if (tcm_hcd->is_detected)
		ovt_tcm_dispatch_message(tcm_hcd);

	retval = 0;

exit:
	if (retval < 0) {
		if (atomic_read(&tcm_hcd->command_status) == CMD_BUSY) {
			atomic_set(&tcm_hcd->command_status, CMD_ERROR);
			complete(&ovt_response_complete);
		}
	}
	return retval;
}

/**
 * ovt_tcm_write_message() - write message to device and receive response
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
 * If resp_buf is NULL, raw write mode is used and ovt_tcm_raw_write() is
 * called. Otherwise, a command and its payload, if any, are sent to the device
 * and the response to the command generated by the device is read in.
 */
static int ovt_tcm_write_message(struct ovt_tcm_hcd *tcm_hcd,
		unsigned char command, unsigned char *payload,
		unsigned int length, unsigned char **resp_buf,
		unsigned int *resp_buf_size, unsigned int *resp_length,
		unsigned char *response_code, unsigned int polling_delay_ms)
{
	int retval;
	unsigned int idx;
	unsigned int chunks;
	unsigned int chunk_space;
	unsigned int xfer_length;
	unsigned int remaining_length;
	unsigned int command_status;
	bool is_romboot_hdl = (command == CMD_ROMBOOT_DOWNLOAD) ? true : false;
	bool is_hdl_reset = (command == CMD_RESET) && (tcm_hcd->in_hdl_mode);

	if (response_code != NULL)
		*response_code = STATUS_INVALID;

	if (!tcm_hcd->do_polling && current->pid == tcm_hcd->isr_pid) {
		OVT_LOG_ERR(
				"Invalid execution context");
		return -EINVAL;
	}

	mutex_lock(&tcm_hcd->command_mutex);

	mutex_lock(&tcm_hcd->rw_ctrl_mutex);

	if (resp_buf == NULL) {
		retval = ovt_tcm_raw_write(tcm_hcd, command, payload, length);
		mutex_unlock(&tcm_hcd->rw_ctrl_mutex);
		goto exit;
	}
#if 0
	if (tcm_hcd->do_polling && polling_delay_ms) {
		cancel_delayed_work_sync(&tcm_hcd->polling_work);
		flush_workqueue(tcm_hcd->polling_workqueue);
	}
#endif
	if (polling_delay_ms) {
		cancel_delayed_work_sync(&tcm_hcd->polling_work);
		flush_workqueue(tcm_hcd->polling_workqueue);
	}
	atomic_set(&tcm_hcd->command_status, CMD_BUSY);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 13, 0))
	reinit_completion(&ovt_response_complete);
#else
	INIT_COMPLETION(ovt_response_complete);
#endif

	tcm_hcd->command = command;

	LOCK_BUFFER(tcm_hcd->resp);

	tcm_hcd->resp.buf = *resp_buf;
	tcm_hcd->resp.buf_size = *resp_buf_size;
	tcm_hcd->resp.data_length = 0;

	UNLOCK_BUFFER(tcm_hcd->resp);

	/* adding two length bytes as part of payload */
	remaining_length = length + 2;

	/* available chunk space for payload = total chunk size minus command
	 * byte */
	if (tcm_hcd->wr_chunk_size == 0)
		chunk_space = remaining_length;
	else
		chunk_space = tcm_hcd->wr_chunk_size - 1;

	if (is_romboot_hdl) {
		if (WR_CHUNK_SIZE) {
			chunk_space = WR_CHUNK_SIZE - 1;
			chunk_space = chunk_space -
					(chunk_space % ROMBOOT_DOWNLOAD_UNIT);
		} else {
			chunk_space = remaining_length;
		}
	}

	chunks = ceil_div(remaining_length, chunk_space);

	chunks = chunks == 0 ? 1 : chunks;

	OVT_LOG_INFO(
			"Command = 0x%02x\n",
			command);

	LOCK_BUFFER(tcm_hcd->out);

	for (idx = 0; idx < chunks; idx++) {
		if (remaining_length > chunk_space)
			xfer_length = chunk_space;
		else
			xfer_length = remaining_length;

		retval = ovt_tcm_alloc_mem(tcm_hcd,
				&tcm_hcd->out,
				xfer_length + 1);
		if (retval < 0) {
			OVT_LOG_ERR(
					"Failed to allocate memory for tcm_hcd->out.buf");
			UNLOCK_BUFFER(tcm_hcd->out);
			mutex_unlock(&tcm_hcd->rw_ctrl_mutex);
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
					OVT_LOG_ERR(
							"Failed to copy payload");
					UNLOCK_BUFFER(tcm_hcd->out);
					mutex_unlock(&tcm_hcd->rw_ctrl_mutex);
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
				OVT_LOG_ERR(
						"Failed to copy payload");
				UNLOCK_BUFFER(tcm_hcd->out);
				mutex_unlock(&tcm_hcd->rw_ctrl_mutex);
				goto exit;
			}
		}

		retval = ovt_tcm_write(tcm_hcd,
				tcm_hcd->out.buf,
				xfer_length + 1);
		if (retval < 0) {
			OVT_LOG_ERR(
					"Failed to write to device");
			UNLOCK_BUFFER(tcm_hcd->out);
			mutex_unlock(&tcm_hcd->rw_ctrl_mutex);
			goto exit;
		}

		remaining_length -= xfer_length;

		if (chunks > 1)
			usleep_range(WRITE_DELAY_US_MIN, WRITE_DELAY_US_MAX);
	}

	UNLOCK_BUFFER(tcm_hcd->out);

	mutex_unlock(&tcm_hcd->rw_ctrl_mutex);

	if (is_hdl_reset)
		goto exit;
#if 0
	if (tcm_hcd->do_polling && polling_delay_ms) {
		queue_delayed_work(tcm_hcd->polling_workqueue,
				&tcm_hcd->polling_work,
				msecs_to_jiffies(polling_delay_ms));
	}
#endif
	if (polling_delay_ms) {
		msleep(polling_delay_ms);
		queue_delayed_work(tcm_hcd->polling_workqueue,
				&tcm_hcd->polling_work,
				msecs_to_jiffies(POLLING_DELAY_MS));
	}
	retval = wait_for_completion_timeout(&ovt_response_complete,
			msecs_to_jiffies(RESPONSE_TIMEOUT_MS));
	if (polling_delay_ms) {
		cancel_delayed_work_sync(&tcm_hcd->polling_work);
		flush_workqueue(tcm_hcd->polling_workqueue);		
	}
	if (retval == 0) {
		OVT_LOG_ERR(
				"Timed out waiting for response (command 0x%02x)\n",
				tcm_hcd->command);
		// if (tcm_hcd->command == CMD_GET_APPLICATION_INFO) {
		// 	//hardware reset TP here
		// 	if (atomic_read(&tcm_hcd->host_downloading) == 0) {
		// 		OVT_LOG_ERR(
		// 		"app info cmd timed out, hw reset TP");
		// 		gpio_set_value(tcm_hcd->ovt_tcm_platform_data->reset_gpio, 0);
		// 		msleep(5);
		// 		gpio_set_value(tcm_hcd->ovt_tcm_platform_data->reset_gpio, 1);        
		// 		msleep(5);
		// 	}			
		// }
		retval = -ETIME;
		goto exit;
	}

	command_status = atomic_read(&tcm_hcd->command_status);
	if (command_status != CMD_IDLE) {
		OVT_LOG_ERR(
				"Failed to get valid response (command 0x%02x)\n",
				tcm_hcd->command);
		retval = -EIO;
		goto exit;
	}

	LOCK_BUFFER(tcm_hcd->resp);

	if (tcm_hcd->response_code != STATUS_OK) {
		if (tcm_hcd->resp.data_length) {
			OVT_LOG_ERR(
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
	OVT_LOG_INFO("response code = 0x%02x (command 0x%02x)\n",tcm_hcd->response_code, tcm_hcd->command);
	UNLOCK_BUFFER(tcm_hcd->resp);

exit:
	tcm_hcd->command = CMD_NONE;

	atomic_set(&tcm_hcd->command_status, CMD_IDLE);

	mutex_unlock(&tcm_hcd->command_mutex);

	return retval;
}

static int ovt_tcm_wait_hdl(struct ovt_tcm_hcd *tcm_hcd)
{
	int retval;

	msleep(HOST_DOWNLOAD_WAIT_MS);

	if (!atomic_read(&tcm_hcd->host_downloading))
		return 0;

	retval = wait_event_interruptible_timeout(tcm_hcd->hdl_wq,
			!atomic_read(&tcm_hcd->host_downloading),
			msecs_to_jiffies(HOST_DOWNLOAD_TIMEOUT_MS));
	if (retval == 0) {
		OVT_LOG_ERR(
				"Timed out waiting for completion of host download");
		atomic_set(&tcm_hcd->host_downloading, 0);
		retval = -EIO;
	} else {
		retval = 0;
	}

	return retval;
}

static void ovt_tcm_check_hdl(struct ovt_tcm_hcd *tcm_hcd, unsigned char id)
{
	struct ovt_tcm_module_handler *mod_handler;

	LOCK_BUFFER(tcm_hcd->report.buffer);

	tcm_hcd->report.buffer.buf = NULL;
	tcm_hcd->report.buffer.buf_size = 0;
	tcm_hcd->report.buffer.data_length = 0;
	tcm_hcd->report.id = id;

	UNLOCK_BUFFER(tcm_hcd->report.buffer);

	mutex_lock(&mod_pool.mutex);

	if (!list_empty(&mod_pool.list)) {
		list_for_each_entry(mod_handler, &mod_pool.list, link) {
			if (!mod_handler->insert &&
					!mod_handler->detach &&
					(mod_handler->mod_cb->syncbox))
				mod_handler->mod_cb->syncbox(tcm_hcd);
		}
	}

	mutex_unlock(&mod_pool.mutex);

	return;
}

#ifdef WATCHDOG_SW
static void ovt_tcm_update_watchdog(struct ovt_tcm_hcd *tcm_hcd, bool en)
{
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

static void ovt_tcm_watchdog_work(struct work_struct *work)
{
	int retval;
	unsigned char marker;
	struct delayed_work *delayed_work =
			container_of(work, struct delayed_work, work);
	struct ovt_tcm_watchdog *watchdog =
			container_of(delayed_work, struct ovt_tcm_watchdog,
			work);
	struct ovt_tcm_hcd *tcm_hcd =
			container_of(watchdog, struct ovt_tcm_hcd, watchdog);

	if (mutex_is_locked(&tcm_hcd->rw_ctrl_mutex))
		goto exit;

	mutex_lock(&tcm_hcd->rw_ctrl_mutex);

	retval = ovt_tcm_read(tcm_hcd,
			&marker,
			1);

	mutex_unlock(&tcm_hcd->rw_ctrl_mutex);

	if (retval < 0 || marker != MESSAGE_MARKER) {
		OVT_LOG_ERR(
				"Failed to read from device");

		tcm_hcd->watchdog.count++;

		if (tcm_hcd->watchdog.count >= WATCHDOG_TRIGGER_COUNT) {
			retval = tcm_hcd->reset_n_reinit(tcm_hcd, true, false);
			if (retval < 0) {
				OVT_LOG_ERR(
						"Failed to do reset and reinit");
			}
			tcm_hcd->watchdog.count = 0;
		}
	}

exit:
	queue_delayed_work(tcm_hcd->watchdog.workqueue,
			&tcm_hcd->watchdog.work,
			msecs_to_jiffies(WATCHDOG_DELAY_MS));

	return;
}
#endif

static void ovt_tcm_polling_work(struct work_struct *work)
{
	int retval;
	struct delayed_work *delayed_work =
			container_of(work, struct delayed_work, work);
	struct ovt_tcm_hcd *tcm_hcd =
			container_of(delayed_work, struct ovt_tcm_hcd,
			polling_work);
#if 0
	if (!tcm_hcd->do_polling)
		return;
#endif
	OVT_LOG_INFO("polling to get message");
	retval = tcm_hcd->read_message(tcm_hcd,
			NULL,
			0);
	if (retval < 0) {
		OVT_LOG_ERR(
				"Failed to read message");
		// if (retval == -ENXIO && tcm_hcd->hw_if->bus_io->type == BUS_SPI)
		// 	ovt_tcm_check_hdl(tcm_hcd, REPORT_HDL_F35);
	}

	if (!(tcm_hcd->in_suspend && retval < 0)) {
		queue_delayed_work(tcm_hcd->polling_workqueue,
				&tcm_hcd->polling_work,
				msecs_to_jiffies(POLLING_DELAY_MS));
	}

	return;
}

static irqreturn_t ovt_tcm_isr(int irq, void *data)
{
	int retval;
	struct ovt_tcm_hcd *tcm_hcd = data;
	const struct ovt_tcm_board_data *bdata = tcm_hcd->hw_if->bdata;

	if (unlikely(gpio_get_value(bdata->irq_gpio) != bdata->irq_on_state))
		goto exit;

	tcm_hcd->isr_pid = current->pid;

	retval = tcm_hcd->read_message(tcm_hcd,
			NULL,
			0);
	if (retval < 0) {
        if (tcm_hcd->ovt_tcm_driver_removing) goto exit;

		if (tcm_hcd->sensor_type == TYPE_F35)
			ovt_tcm_check_hdl(tcm_hcd, REPORT_HDL_F35);
		else
			OVT_LOG_ERR(
				"Failed to read message");
	}

exit:
	return IRQ_HANDLED;
}

static int ovt_tcm_enable_irq(struct ovt_tcm_hcd *tcm_hcd, bool en, bool ns)
{
	int retval;
	const struct ovt_tcm_board_data *bdata = tcm_hcd->hw_if->bdata;
	static bool irq_freed = true;

	return 0; //for irq is register in tpkit side
	mutex_lock(&tcm_hcd->irq_en_mutex);

	if (en) {
		if (tcm_hcd->irq_enabled) {
			LOGD(tcm_hcd->pdev->dev.parent,
					"Interrupt already enabled");
			retval = 0;
			goto exit;
		}

		if (bdata->irq_gpio < 0) {
			OVT_LOG_ERR(
					"Invalid IRQ GPIO");
			retval = -EINVAL;
			goto queue_polling_work;
		}

		if (irq_freed) {
			retval = request_threaded_irq(tcm_hcd->irq, NULL,
					ovt_tcm_isr, bdata->irq_flags,
					PLATFORM_DRIVER_NAME, tcm_hcd);
			if (retval < 0) {
				OVT_LOG_ERR(
						"Failed to create interrupt thread");
			}
		} else {
			enable_irq(tcm_hcd->irq);
			retval = 0;
		}

queue_polling_work:
		if (retval < 0) {
#ifdef FALL_BACK_ON_POLLING
			queue_delayed_work(tcm_hcd->polling_workqueue,
					&tcm_hcd->polling_work,
					msecs_to_jiffies(POLLING_DELAY_MS));
			tcm_hcd->do_polling = true;
			retval = 0;
#endif
		}

		if (retval < 0)
			goto exit;
		else
			msleep(ENABLE_IRQ_DELAY_MS);
	} else {
		if (!tcm_hcd->irq_enabled) {
			LOGD(tcm_hcd->pdev->dev.parent,
					"Interrupt already disabled");
			retval = 0;
			goto exit;
		}

		if (bdata->irq_gpio >= 0) {
			if (ns) {
				disable_irq_nosync(tcm_hcd->irq);
			} else {
				disable_irq(tcm_hcd->irq);
				free_irq(tcm_hcd->irq, tcm_hcd);
			}
			irq_freed = !ns;
		}

		if (ns) {
			cancel_delayed_work(&tcm_hcd->polling_work);
		} else {
			cancel_delayed_work_sync(&tcm_hcd->polling_work);
			flush_workqueue(tcm_hcd->polling_workqueue);
		}

		tcm_hcd->do_polling = false;
	}

	retval = 0;

exit:
	if (retval == 0)
		tcm_hcd->irq_enabled = en;

	mutex_unlock(&tcm_hcd->irq_en_mutex);

	return retval;
}

static int ovt_tcm_set_gpio(struct ovt_tcm_hcd *tcm_hcd, int gpio,
		bool config, int dir, int state)
{
	int retval;
	char label[16];

	if (config) {
		retval = snprintf(label, 16, "tcm_gpio_%d\n", gpio);
		if (retval < 0) {
			OVT_LOG_ERR(
					"Failed to set GPIO label");
			return retval;
		}

		retval = gpio_request(gpio, label);
		if (retval < 0) {
			OVT_LOG_ERR(
					"Failed to request GPIO %d\n",
					gpio);
			return retval;
		}

		if (dir == 0)
			retval = gpio_direction_input(gpio);
		else
			retval = gpio_direction_output(gpio, state);
		if (retval < 0) {
			OVT_LOG_ERR(
					"Failed to set GPIO %d direction\n",
					gpio);
			return retval;
		}
	} else {
		gpio_free(gpio);
	}

	return 0;
}

static int ovt_tcm_config_gpio(struct ovt_tcm_hcd *tcm_hcd)
{
	int retval;
	const struct ovt_tcm_board_data *bdata = tcm_hcd->hw_if->bdata;
#if 0
	if (bdata->irq_gpio >= 0) {
		retval = ovt_tcm_set_gpio(tcm_hcd, bdata->irq_gpio,
				true, 0, 0);
		if (retval < 0) {
			OVT_LOG_ERR(
					"Failed to configure interrupt GPIO");
			goto err_set_gpio_irq;
		}
	}
#endif
	if (bdata->power_gpio >= 0) {
		retval = ovt_tcm_set_gpio(tcm_hcd, bdata->power_gpio,
				true, 1, !bdata->power_on_state);
		if (retval < 0) {
			OVT_LOG_ERR(
					"Failed to configure power GPIO");
			goto err_set_gpio_power;
		}
	}
#if 0
	if (bdata->reset_gpio >= 0) {
		retval = ovt_tcm_set_gpio(tcm_hcd, bdata->reset_gpio,
				true, 1, !bdata->reset_on_state);
		if (retval < 0) {
			OVT_LOG_ERR(
					"Failed to configure reset GPIO");
			goto err_set_gpio_reset;
		}
	}
#endif
	if (bdata->power_gpio >= 0) {
		gpio_set_value(bdata->power_gpio, bdata->power_on_state);
		msleep(bdata->power_delay_ms);
	}
#if 0
	if (bdata->reset_gpio >= 0) {
		gpio_set_value(bdata->reset_gpio, bdata->reset_on_state);
		msleep(bdata->reset_active_ms);
		gpio_set_value(bdata->reset_gpio, !bdata->reset_on_state);
		msleep(bdata->reset_delay_ms);
	}
#endif
	return 0;

err_set_gpio_reset:
	if (bdata->power_gpio >= 0)
		ovt_tcm_set_gpio(tcm_hcd, bdata->power_gpio, false, 0, 0);

err_set_gpio_power:
	if (bdata->irq_gpio >= 0)
		ovt_tcm_set_gpio(tcm_hcd, bdata->irq_gpio, false, 0, 0);

err_set_gpio_irq:
	return retval;
}

static int ovt_tcm_enable_regulator(struct ovt_tcm_hcd *tcm_hcd, bool en)
{
	int retval;
	const struct ovt_tcm_board_data *bdata = tcm_hcd->hw_if->bdata;

	if (!en) {
		retval = 0;
		goto disable_pwr_reg;
	}

	if (tcm_hcd->bus_reg) {
		retval = regulator_enable(tcm_hcd->bus_reg);
		if (retval < 0) {
			OVT_LOG_ERR(
					"Failed to enable bus regulator");
			goto exit;
		}
	}

	if (tcm_hcd->pwr_reg) {
		retval = regulator_enable(tcm_hcd->pwr_reg);
		if (retval < 0) {
			OVT_LOG_ERR(
					"Failed to enable power regulator");
			goto disable_bus_reg;
		}
		msleep(bdata->power_delay_ms);
	}

	return 0;

disable_pwr_reg:
	if (tcm_hcd->pwr_reg)
		regulator_disable(tcm_hcd->pwr_reg);

disable_bus_reg:
	if (tcm_hcd->bus_reg)
		regulator_disable(tcm_hcd->bus_reg);

exit:
	return retval;
}

static int ovt_tcm_get_regulator(struct ovt_tcm_hcd *tcm_hcd, bool get)
{
	int retval;
	const struct ovt_tcm_board_data *bdata = tcm_hcd->hw_if->bdata;

	if (!get) {
		retval = 0;
		goto regulator_put;
	}
	if (bdata->bus_reg_name != NULL && *bdata->bus_reg_name != 0) {
		tcm_hcd->bus_reg = regulator_get(tcm_hcd->pdev->dev.parent,
				bdata->bus_reg_name);
		if (IS_ERR(tcm_hcd->bus_reg)) {
			OVT_LOG_ERR(
					"Failed to get bus regulator");
			retval = PTR_ERR(tcm_hcd->bus_reg);
			goto regulator_put;
		}
	}

	if (bdata->pwr_reg_name != NULL && *bdata->pwr_reg_name != 0) {
		tcm_hcd->pwr_reg = regulator_get(tcm_hcd->pdev->dev.parent,
				bdata->pwr_reg_name);
		if (IS_ERR(tcm_hcd->pwr_reg)) {
			OVT_LOG_ERR(
					"Failed to get power regulator");
			retval = PTR_ERR(tcm_hcd->pwr_reg);
			goto regulator_put;
		}
	}

	return 0;

regulator_put:
	if (tcm_hcd->bus_reg) {
		regulator_put(tcm_hcd->bus_reg);
		tcm_hcd->bus_reg = NULL;
	}

	if (tcm_hcd->pwr_reg) {
		regulator_put(tcm_hcd->pwr_reg);
		tcm_hcd->pwr_reg = NULL;
	}

	return retval;
}

static int ovt_tcm_get_app_info(struct ovt_tcm_hcd *tcm_hcd)
{
	int retval;
	unsigned char *resp_buf;
	unsigned int resp_buf_size;
	unsigned int resp_length;
	unsigned int timeout;

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
			100);
	if (retval < 0) {
		OVT_LOG_ERR(
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
		OVT_LOG_ERR(
				"Failed to copy application info");
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

static int ovt_tcm_get_boot_info(struct ovt_tcm_hcd *tcm_hcd)
{
	int retval;
	unsigned char *resp_buf;
	unsigned int resp_buf_size;
	unsigned int resp_length;

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
		OVT_LOG_ERR(
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
		OVT_LOG_ERR(
				"Failed to copy boot info");
		goto exit;
	}

	retval = 0;

exit:
	kfree(resp_buf);

	return retval;
}

static int ovt_tcm_get_romboot_info(struct ovt_tcm_hcd *tcm_hcd)
{
	int retval;
	unsigned char *resp_buf;
	unsigned int resp_buf_size;
	unsigned int resp_length;

	resp_buf = NULL;
	resp_buf_size = 0;

	retval = tcm_hcd->write_message(tcm_hcd,
			CMD_GET_ROMBOOT_INFO,
			NULL,
			0,
			&resp_buf,
			&resp_buf_size,
			&resp_length,
			NULL,
			0);
	if (retval < 0) {
		OVT_LOG_ERR(
				"Failed to write command %s\n",
				STR(CMD_GET_ROMBOOT_INFO));
		goto exit;
	}

	retval = secure_memcpy((unsigned char *)&tcm_hcd->romboot_info,
			sizeof(tcm_hcd->romboot_info),
			resp_buf,
			resp_buf_size,
			MIN(sizeof(tcm_hcd->romboot_info), resp_length));
	if (retval < 0) {
		OVT_LOG_ERR(
				"Failed to copy boot info");
		goto exit;
	}

	LOGD(tcm_hcd->pdev->dev.parent,
			"version = %d\n", tcm_hcd->romboot_info.version);

	LOGD(tcm_hcd->pdev->dev.parent,
			"status = 0x%02x\n", tcm_hcd->romboot_info.status);

	LOGD(tcm_hcd->pdev->dev.parent,
			"version = 0x%02x 0x%02x\n",
			tcm_hcd->romboot_info.asic_id[0],
			tcm_hcd->romboot_info.asic_id[1]);

	LOGD(tcm_hcd->pdev->dev.parent,
			"write_block_size_words = %d\n",
			tcm_hcd->romboot_info.write_block_size_words);

	LOGD(tcm_hcd->pdev->dev.parent,
			"max_write_payload_size = %d\n",
			tcm_hcd->romboot_info.max_write_payload_size[0] |
			tcm_hcd->romboot_info.max_write_payload_size[1] << 8);

	LOGD(tcm_hcd->pdev->dev.parent,
			"last_reset_reason = 0x%02x\n",
			tcm_hcd->romboot_info.last_reset_reason);

	retval = 0;

exit:
	kfree(resp_buf);

	return retval;
}

static int ovt_tcm_identify(struct ovt_tcm_hcd *tcm_hcd, bool id)
{
	int retval;
	unsigned char *resp_buf;
	unsigned int resp_buf_size;
	unsigned int resp_length;
	unsigned int max_write_size;
	unsigned int retry_cnt = 3;

	resp_buf = NULL;
	resp_buf_size = 0;

	mutex_lock(&tcm_hcd->identify_mutex);

	if (!id)
		goto get_info;
id_info:
	retval = tcm_hcd->write_message(tcm_hcd,
			CMD_IDENTIFY,
			NULL,
			0,
			&resp_buf,
			&resp_buf_size,
			&resp_length,
			NULL,
			20);
	if (retval < 0) {
		OVT_LOG_ERR(
				"Failed to write command %s\n",
				STR(CMD_IDENTIFY));
		if (retry_cnt) {
			retry_cnt--;
			tcm_hcd->read_message(tcm_hcd,NULL,0);  //read out message
			msleep(100);
			goto id_info;
		} else {
			goto exit;
		}
	}

	retval = secure_memcpy((unsigned char *)&tcm_hcd->id_info,
			sizeof(tcm_hcd->id_info),
			resp_buf,
			resp_buf_size,
			MIN(sizeof(tcm_hcd->id_info), resp_length));
	if (retval < 0) {
		OVT_LOG_ERR(
				"Failed to copy identification info");
		goto exit;
	}

	tcm_hcd->packrat_number = le4_to_uint(tcm_hcd->id_info.build_id);

	max_write_size = le2_to_uint(tcm_hcd->id_info.max_write_size);
	tcm_hcd->wr_chunk_size = MIN(max_write_size, WR_CHUNK_SIZE);
	if (tcm_hcd->wr_chunk_size == 0)
		tcm_hcd->wr_chunk_size = max_write_size;

	OVT_LOG_INFO(
		"Firmware build id = %d\n", tcm_hcd->packrat_number);
	retry_cnt = 3;
get_info:
	switch (tcm_hcd->id_info.mode) {
	case MODE_APPLICATION_FIRMWARE:
	case MODE_HOSTDOWNLOAD_FIRMWARE:
		retval = ovt_tcm_get_app_info(tcm_hcd);
		if (retval < 0) {
			OVT_LOG_ERR(
					"Failed to get application info and retry");
			if (0) {
				retry_cnt--;
				tcm_hcd->read_message(tcm_hcd,NULL,0);  //read out message
				msleep(100);
				goto get_info;
			} else {
				goto exit;
			}
		}
		break;
	case MODE_BOOTLOADER:
	case MODE_TDDI_BOOTLOADER:

		LOGD(tcm_hcd->pdev->dev.parent,
			"In bootloader mode");

		retval = ovt_tcm_get_boot_info(tcm_hcd);
		if (retval < 0) {
			OVT_LOG_ERR(
					"Failed to get boot info");
			goto exit;
		}
		break;
	case MODE_ROMBOOTLOADER:

		LOGD(tcm_hcd->pdev->dev.parent,
			"In rombootloader mode");

		retval = ovt_tcm_get_romboot_info(tcm_hcd);
		if (retval < 0) {
			OVT_LOG_ERR(
					"Failed to get application info");
			goto exit;
		}
		break;
	default:
		break;
	}

	retval = 0;

exit:
	mutex_unlock(&tcm_hcd->identify_mutex);

	kfree(resp_buf);

	return retval;
}

static int ovt_tcm_run_production_test_firmware(struct ovt_tcm_hcd *tcm_hcd)
{
	int retval;
	bool retry;
	unsigned char *resp_buf;
	unsigned int resp_buf_size;
	unsigned int resp_length;

	retry = true;

	resp_buf = NULL;
	resp_buf_size = 0;

retry:
	retval = tcm_hcd->write_message(tcm_hcd,
			CMD_ENTER_PRODUCTION_TEST_MODE,
			NULL,
			0,
			&resp_buf,
			&resp_buf_size,
			&resp_length,
			NULL,
			MODE_SWITCH_DELAY_MS);
	if (retval < 0) {
		OVT_LOG_ERR(
				"Failed to write command %s\n",
				STR(CMD_ENTER_PRODUCTION_TEST_MODE));
		goto exit;
	}

	if (tcm_hcd->id_info.mode != MODE_PRODUCTIONTEST_FIRMWARE) {
		OVT_LOG_ERR(
				"Failed to run production test firmware");
		if (retry) {
			retry = false;
			goto retry;
		}
		retval = -EINVAL;
		goto exit;
	} else if (tcm_hcd->app_status != APP_STATUS_OK) {
		OVT_LOG_ERR(
				"Application status = 0x%02x\n",
				tcm_hcd->app_status);
	}

	retval = 0;

exit:
	kfree(resp_buf);

	return retval;
}

static int ovt_tcm_run_application_firmware(struct ovt_tcm_hcd *tcm_hcd)
{
	int retval;
	bool retry;
	unsigned char *resp_buf;
	unsigned int resp_buf_size;
	unsigned int resp_length;

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
		OVT_LOG_ERR(
				"Failed to write command %s\n",
				STR(CMD_RUN_APPLICATION_FIRMWARE));
		goto exit;
	}

	retval = tcm_hcd->identify(tcm_hcd, false);
	if (retval < 0) {
		OVT_LOG_ERR(
				"Failed to do identification");
		goto exit;
	}

	if (IS_NOT_FW_MODE(tcm_hcd->id_info.mode)) {
		OVT_LOG_ERR(
				"Failed to run application firmware (boot status = 0x%02x)\n",
				tcm_hcd->boot_info.status);
		if (retry) {
			retry = false;
			goto retry;
		}
		retval = -EINVAL;
		goto exit;
	} else if (tcm_hcd->app_status != APP_STATUS_OK) {
		OVT_LOG_ERR(
				"Application status = 0x%02x\n",
				tcm_hcd->app_status);
	}

	retval = 0;

exit:
	kfree(resp_buf);

	return retval;
}

static int ovt_tcm_run_bootloader_firmware(struct ovt_tcm_hcd *tcm_hcd)
{
	int retval;
	unsigned char *resp_buf;
	unsigned int resp_buf_size;
	unsigned int resp_length;
	unsigned char command;

	resp_buf = NULL;
	resp_buf_size = 0;
	command = (tcm_hcd->id_info.mode == MODE_ROMBOOTLOADER) ?
			CMD_ROMBOOT_RUN_BOOTLOADER_FIRMWARE :
			CMD_RUN_BOOTLOADER_FIRMWARE;

	retval = tcm_hcd->write_message(tcm_hcd,
			command,
			NULL,
			0,
			&resp_buf,
			&resp_buf_size,
			&resp_length,
			NULL,
			MODE_SWITCH_DELAY_MS);
	if (retval < 0) {
		if (tcm_hcd->id_info.mode == MODE_ROMBOOTLOADER) {
			OVT_LOG_ERR(
				"Failed to write command %s\n",
				STR(CMD_ROMBOOT_RUN_BOOTLOADER_FIRMWARE));
		} else {
			OVT_LOG_ERR(
				"Failed to write command %s\n",
				STR(CMD_RUN_BOOTLOADER_FIRMWARE));
		}
		goto exit;
	}

	if (command != CMD_ROMBOOT_RUN_BOOTLOADER_FIRMWARE) {
		retval = tcm_hcd->identify(tcm_hcd, false);
		if (retval < 0) {
			OVT_LOG_ERR(
				"Failed to do identification");
		goto exit;
		}

		if (IS_FW_MODE(tcm_hcd->id_info.mode)) {
			OVT_LOG_ERR(
					"Failed to enter bootloader mode");
			retval = -EINVAL;
			goto exit;
		}
	}

	retval = 0;

exit:
	kfree(resp_buf);

	return retval;
}

static int ovt_tcm_switch_mode(struct ovt_tcm_hcd *tcm_hcd,
		enum firmware_mode mode)
{
	int retval;

	mutex_lock(&tcm_hcd->reset_mutex);

#ifdef WATCHDOG_SW
	tcm_hcd->update_watchdog(tcm_hcd, false);
#endif

	switch (mode) {
	case FW_MODE_BOOTLOADER:
		retval = ovt_tcm_run_bootloader_firmware(tcm_hcd);
		if (retval < 0) {
			OVT_LOG_ERR(
					"Failed to switch to bootloader mode");
			goto exit;
		}
		break;
	case FW_MODE_APPLICATION:
		retval = ovt_tcm_run_application_firmware(tcm_hcd);
		if (retval < 0) {
			OVT_LOG_ERR(
					"Failed to switch to application mode");
			goto exit;
		}
		break;
	case FW_MODE_PRODUCTION_TEST:
		retval = ovt_tcm_run_production_test_firmware(tcm_hcd);
		if (retval < 0) {
			OVT_LOG_ERR(
					"Failed to switch to production test mode");
			goto exit;
		}
		break;
	default:
		OVT_LOG_ERR(
				"Invalid firmware mode");
		retval = -EINVAL;
		goto exit;
	}

	retval = 0;

exit:
#ifdef WATCHDOG_SW
	tcm_hcd->update_watchdog(tcm_hcd, true);
#endif

	mutex_unlock(&tcm_hcd->reset_mutex);

	return retval;
}

static int ovt_tcm_get_dynamic_config(struct ovt_tcm_hcd *tcm_hcd,
		enum dynamic_config_id id, unsigned short *value)
{
	int retval;
	unsigned char out_buf;
	unsigned char *resp_buf;
	unsigned int resp_buf_size;
	unsigned int resp_length;

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
		OVT_LOG_ERR(
				"Failed to write command %s\n",
				STR(CMD_GET_DYNAMIC_CONFIG));
		goto exit;
	}

	if (resp_length < 2) {
		OVT_LOG_ERR(
				"Invalid data length");
		retval = -EINVAL;
		goto exit;
	}

	*value = (unsigned short)le2_to_uint(resp_buf);

	retval = 0;

exit:
	kfree(resp_buf);

	return retval;
}

static int ovt_tcm_set_dynamic_config(struct ovt_tcm_hcd *tcm_hcd,
		enum dynamic_config_id id, unsigned short value)
{
	int retval;
	unsigned char out_buf[3];
	unsigned char *resp_buf;
	unsigned int resp_buf_size;
	unsigned int resp_length;

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
		OVT_LOG_ERR(
				"Failed to write command %s\n",
				STR(CMD_SET_DYNAMIC_CONFIG));
		goto exit;
	}

	retval = 0;

exit:
	kfree(resp_buf);

	return retval;
}

static int ovt_tcm_get_data_location(struct ovt_tcm_hcd *tcm_hcd,
		enum flash_area area, unsigned int *addr, unsigned int *length)
{
	int retval;
	unsigned char out_buf;
	unsigned char *resp_buf;
	unsigned int resp_buf_size;
	unsigned int resp_length;

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
		OVT_LOG_ERR(
				"Invalid flash area");
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
		OVT_LOG_ERR(
				"Failed to write command %s\n",
				STR(CMD_GET_DATA_LOCATION));
		goto exit;
	}

	if (resp_length != 4) {
		OVT_LOG_ERR(
				"Invalid data length");
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

static int ovt_tcm_sleep(struct ovt_tcm_hcd *tcm_hcd, bool en)
{
	int retval;
	unsigned char command;
	unsigned char *resp_buf;
	unsigned int resp_buf_size;
	unsigned int resp_length;

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
		OVT_LOG_ERR(
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

static int ovt_tcm_reset(struct ovt_tcm_hcd *tcm_hcd)
{
	int retval;
	unsigned char *resp_buf;
	unsigned int resp_buf_size;
	unsigned int resp_length;
	const struct ovt_tcm_board_data *bdata = tcm_hcd->hw_if->bdata;

	retval = tcm_hcd->write_message(tcm_hcd,
				CMD_RESET,
				NULL,
				0,
				&resp_buf,
				&resp_buf_size,
				&resp_length,
				NULL,
				bdata->reset_delay_ms);
	if (retval < 0) {
		OVT_LOG_ERR(
				"Failed to write command %s\n",
				STR(CMD_RESET));
	}

	return retval;
}

static int ovt_tcm_reset_and_reinit(struct ovt_tcm_hcd *tcm_hcd,
		bool hw, bool update_wd)
{
	int retval;
	unsigned char *resp_buf;
	unsigned int resp_buf_size;
	unsigned int resp_length;
	struct ovt_tcm_module_handler *mod_handler;
	const struct ovt_tcm_board_data *bdata = tcm_hcd->hw_if->bdata;

	resp_buf = NULL;
	resp_buf_size = 0;

	mutex_lock(&tcm_hcd->reset_mutex);

#ifdef WATCHDOG_SW
	if (update_wd)
		tcm_hcd->update_watchdog(tcm_hcd, false);
#endif

	if (hw) {
		if (bdata->reset_gpio < 0) {
			OVT_LOG_ERR(
					"Hardware reset unavailable");
			retval = -EINVAL;
			goto exit;
		}
		gpio_set_value(bdata->reset_gpio, bdata->reset_on_state);
		msleep(bdata->reset_active_ms);
		gpio_set_value(bdata->reset_gpio, !bdata->reset_on_state);
	} else {
		retval = ovt_tcm_reset(tcm_hcd);
		if (retval < 0) {
			OVT_LOG_ERR(
					"Failed to do reset");
			goto exit;
		}
	}

	/* for hdl, the remaining re-init process will be done */
	/* in the helper thread, so wait for the completion here */
	if (tcm_hcd->in_hdl_mode) {
		mutex_unlock(&tcm_hcd->reset_mutex);
		kfree(resp_buf);

		retval = ovt_tcm_wait_hdl(tcm_hcd);
		if (retval < 0) {
			OVT_LOG_ERR(
					"Failed to wait for completion of host download");
			return retval;
		}

#ifdef WATCHDOG_SW
		if (update_wd)
			tcm_hcd->update_watchdog(tcm_hcd, true);
#endif
		return 0;
	}

	msleep(bdata->reset_delay_ms);

	retval = tcm_hcd->identify(tcm_hcd, false);
	if (retval < 0) {
		OVT_LOG_ERR(
				"Failed to do identification");
		goto exit;
	}

	if (IS_FW_MODE(tcm_hcd->id_info.mode))
		goto get_features;

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
		OVT_LOG_INFO(
				"Failed to write command %s\n",
				STR(CMD_RUN_APPLICATION_FIRMWARE));
	}

	retval = tcm_hcd->identify(tcm_hcd, false);
	if (retval < 0) {
		OVT_LOG_ERR(
				"Failed to do identification");
		goto exit;
	}

get_features:
	OVT_LOG_INFO(
			"Firmware mode = 0x%02x\n",
			tcm_hcd->id_info.mode);

	if (IS_NOT_FW_MODE(tcm_hcd->id_info.mode)) {
		OVT_LOG_INFO(
				"Boot status = 0x%02x\n",
				tcm_hcd->boot_info.status);
	} else if (tcm_hcd->app_status != APP_STATUS_OK) {
		OVT_LOG_INFO(
				"Application status = 0x%02x\n",
				tcm_hcd->app_status);
	}

	if (IS_NOT_FW_MODE(tcm_hcd->id_info.mode))
		goto dispatch_reinit;

	retval = tcm_hcd->write_message(tcm_hcd,
			CMD_GET_FEATURES,
			NULL,
			0,
			&resp_buf,
			&resp_buf_size,
			&resp_length,
			NULL,
			0);
	if (retval < 0) {
		OVT_LOG_INFO(
				"Failed to write command %s\n",
				STR(CMD_GET_FEATURES));
	} else {
		retval = secure_memcpy((unsigned char *)&tcm_hcd->features,
				sizeof(tcm_hcd->features),
				resp_buf,
				resp_buf_size,
				MIN(sizeof(tcm_hcd->features), resp_length));
		if (retval < 0) {
			OVT_LOG_ERR(
					"Failed to copy feature description");
		}
	}

dispatch_reinit:
	mutex_lock(&mod_pool.mutex);

	if (!list_empty(&mod_pool.list)) {
		list_for_each_entry(mod_handler, &mod_pool.list, link) {
			if (!mod_handler->insert &&
					!mod_handler->detach &&
					(mod_handler->mod_cb->reinit))
				mod_handler->mod_cb->reinit(tcm_hcd);
		}
	}

	mutex_unlock(&mod_pool.mutex);

	retval = 0;

exit:
#ifdef WATCHDOG_SW
	if (update_wd)
		tcm_hcd->update_watchdog(tcm_hcd, true);
#endif

	mutex_unlock(&tcm_hcd->reset_mutex);

	kfree(resp_buf);

	return retval;
}

static int ovt_tcm_rezero(struct ovt_tcm_hcd *tcm_hcd)
{
	int retval;
	unsigned char *resp_buf;
	unsigned int resp_buf_size;
	unsigned int resp_length;

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
		OVT_LOG_ERR(
				"Failed to write command %s\n",
				STR(CMD_REZERO));
		goto exit;
	}

	retval = 0;

exit:
	kfree(resp_buf);

	return retval;
}

static void ovt_tcm_helper_work(struct work_struct *work)
{
	int retval;
	unsigned char task;
	struct ovt_tcm_helper *helper =
			container_of(work, struct ovt_tcm_helper, work);
	struct ovt_tcm_hcd *tcm_hcd = g_tcm_hcd;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 13, 0))
	reinit_completion(helper->helper_completion);
#else
	INIT_COMPLETION(*(helper->helper_completion));
#endif

	task = atomic_read(&helper->task);
    if (tcm_hcd->ovt_tcm_driver_removing) return;

	switch (task) {
		case HELP_SEND_ROMBOOT_HDL:
			tcm_hcd->sensor_type = TYPE_ROMBOOT;
			retval = zeroflash_do_hostdownload(tcm_hcd);
			if (retval >= 0) {
				retval = tcm_hcd->identify(tcm_hcd, true);
				retval = ovt_touch_init(tcm_hcd);
				ovt_touch_config_input_dev(tcm_hcd->input_dev);
			}
			break;
		default:
			break;
	}
	atomic_set(&helper->task, HELP_NONE);
	complete(helper->helper_completion);
	return;
}


static int ovt_tcm_resume(void)
{
#if 0
	int retval;
	struct ovt_tcm_module_handler *mod_handler;
	struct ovt_tcm_hcd *tcm_hcd = dev_get_drvdata(dev);

	if (!tcm_hcd->in_suspend)
		return 0;

	if (tcm_hcd->in_hdl_mode) {
		if (!tcm_hcd->wakeup_gesture_enabled) {
			tcm_hcd->enable_irq(tcm_hcd, true, NULL);
			retval = ovt_tcm_wait_hdl(tcm_hcd);
			if (retval < 0) {
				OVT_LOG_ERR(
						"Failed to wait for completion of host download");
				goto exit;
			}
			goto mod_resume;
		}
	} else {
		if (!tcm_hcd->wakeup_gesture_enabled)
			tcm_hcd->enable_irq(tcm_hcd, true, NULL);

#ifdef RESET_ON_RESUME
		msleep(RESET_ON_RESUME_DELAY_MS);
		goto do_reset;
#endif
	}

	if (IS_NOT_FW_MODE(tcm_hcd->id_info.mode) ||
			tcm_hcd->app_status != APP_STATUS_OK) {
		OVT_LOG_INFO(
				"Identifying mode = 0x%02x\n",
				tcm_hcd->id_info.mode);
		goto do_reset;
	}

	retval = tcm_hcd->sleep(tcm_hcd, false);
	if (retval < 0) {
		OVT_LOG_ERR(
				"Failed to exit deep sleep");
		goto exit;
	}

	retval = ovt_tcm_rezero(tcm_hcd);
	if (retval < 0) {
		OVT_LOG_ERR(
				"Failed to rezero");
		goto exit;
	}

	goto mod_resume;

do_reset:
	retval = tcm_hcd->reset_n_reinit(tcm_hcd, false, true);
	if (retval < 0) {
		OVT_LOG_ERR(
				"Failed to do reset and reinit");
		goto exit;
	}

	if (IS_NOT_FW_MODE(tcm_hcd->id_info.mode) ||
			tcm_hcd->app_status != APP_STATUS_OK) {
		OVT_LOG_INFO(
				"Identifying mode = 0x%02x\n",
				tcm_hcd->id_info.mode);
		retval = 0;
		goto exit;
	}

mod_resume:
	touch_resume(tcm_hcd);

#ifdef WATCHDOG_SW
	tcm_hcd->update_watchdog(tcm_hcd, true);
#endif

	mutex_lock(&mod_pool.mutex);

	if (!list_empty(&mod_pool.list)) {
		list_for_each_entry(mod_handler, &mod_pool.list, link) {
			if (!mod_handler->insert &&
					!mod_handler->detach &&
					(mod_handler->mod_cb->resume))
				mod_handler->mod_cb->resume(tcm_hcd);
		}
	}

	mutex_unlock(&mod_pool.mutex);

	retval = 0;

exit:
	tcm_hcd->in_suspend = false;

	return retval;
#endif
	int retval = 0;
	struct ovt_tcm_hcd *tcm_hcd = g_tcm_hcd;

	if (!tcm_hcd->in_suspend)
		return 0;
	//gpio_direction_output(tcm_hcd->ovt_tcm_platform_data->reset_gpio, 1);
	tcm_hcd->in_suspend = false;

	return retval;
}

static int ovt_tcm_suspend(void)
{
#if 0
	struct ovt_tcm_module_handler *mod_handler;
	struct ovt_tcm_hcd *tcm_hcd = dev_get_drvdata(dev);

	if (tcm_hcd->in_suspend)
		return 0;


	touch_suspend(tcm_hcd);

	mutex_lock(&mod_pool.mutex);

	if (!list_empty(&mod_pool.list)) {
		list_for_each_entry(mod_handler, &mod_pool.list, link) {
			if (!mod_handler->insert &&
					!mod_handler->detach &&
					(mod_handler->mod_cb->suspend))
				mod_handler->mod_cb->suspend(tcm_hcd);
		}
	}

	mutex_unlock(&mod_pool.mutex);

	if (!tcm_hcd->wakeup_gesture_enabled)
		tcm_hcd->enable_irq(tcm_hcd, false, true);

	tcm_hcd->in_suspend = true;

	return 0;
#endif
	int retval = 0;
	struct ovt_tcm_hcd *tcm_hcd = g_tcm_hcd;

	if (tcm_hcd->in_suspend)
		return 0;
	//gpio_direction_output(tcm_hcd->ovt_tcm_platform_data->reset_gpio, 0);


	tcm_hcd->in_suspend = true;
	return retval;
}

#if 0
static int ovt_tcm_early_suspend(struct device *dev)
{
	int retval;
	struct ovt_tcm_module_handler *mod_handler;
	struct ovt_tcm_hcd *tcm_hcd = dev_get_drvdata(dev);

	if (tcm_hcd->in_suspend)
		return 0;

#ifdef WATCHDOG_SW
	tcm_hcd->update_watchdog(tcm_hcd, false);
#endif

	if (IS_NOT_FW_MODE(tcm_hcd->id_info.mode) ||
			tcm_hcd->app_status != APP_STATUS_OK) {
		OVT_LOG_INFO(
				"Identifying mode = 0x%02x\n",
				tcm_hcd->id_info.mode);
		return 0;
	}

	if (!tcm_hcd->wakeup_gesture_enabled) {
		retval = tcm_hcd->sleep(tcm_hcd, true);
		if (retval < 0) {
			OVT_LOG_ERR(
					"Failed to enter deep sleep");
			return retval;
		}
	}

	touch_early_suspend(tcm_hcd);

	mutex_lock(&mod_pool.mutex);

	if (!list_empty(&mod_pool.list)) {
		list_for_each_entry(mod_handler, &mod_pool.list, link) {
			if (!mod_handler->insert &&
					!mod_handler->detach &&
					(mod_handler->mod_cb->early_suspend))
				mod_handler->mod_cb->early_suspend(tcm_hcd);
		}
	}

	mutex_unlock(&mod_pool.mutex);

	return 0;
}

static int ovt_tcm_fb_notifier_cb(struct notifier_block *nb,
		unsigned long action, void *data)
{
	int retval;
	int *transition;
	struct fb_event *evdata = data;
	struct ovt_tcm_hcd *tcm_hcd =
			container_of(nb, struct ovt_tcm_hcd, fb_notifier);

	retval = 0;
	if (evdata && evdata->data && tcm_hcd) {
		transition = evdata->data;

		if (atomic_read(&tcm_hcd->firmware_flashing) &&
				*transition == FB_BLANK_POWERDOWN) {

			retval = wait_event_interruptible_timeout(
				tcm_hcd->reflash_wq,
				!atomic_read(&tcm_hcd->firmware_flashing),
				msecs_to_jiffies(RESPONSE_TIMEOUT_MS)
				);
			if (retval == 0) {
				OVT_LOG_ERR(
						"Timed out waiting for completion of flashing firmware");
				atomic_set(&tcm_hcd->firmware_flashing, 0);
				return -EIO;
			} else {
				retval = 0;
			}
		}

		if (action == FB_EARLY_EVENT_BLANK &&
				*transition == FB_BLANK_POWERDOWN)
			retval = ovt_tcm_early_suspend(&tcm_hcd->pdev->dev);
		else if (action == FB_EVENT_BLANK) {
			if (*transition == FB_BLANK_POWERDOWN) {
				retval = ovt_tcm_suspend(&tcm_hcd->pdev->dev);
				tcm_hcd->fb_ready = 0;
			} else if (*transition == FB_BLANK_UNBLANK) {
#ifndef RESUME_EARLY_UNBLANK
				retval = ovt_tcm_resume(&tcm_hcd->pdev->dev);
				tcm_hcd->fb_ready++;
#endif
			}
		} else if (action == FB_EARLY_EVENT_BLANK &&
				*transition == FB_BLANK_UNBLANK) {
#ifdef RESUME_EARLY_UNBLANK
				retval = ovt_tcm_resume(&tcm_hcd->pdev->dev);
				tcm_hcd->fb_ready++;
#endif
		}
	}

	return 0;
}
#endif
void ovt_tcm_simple_hw_reset(struct ovt_tcm_hcd *tcm_hcd)
{
	const struct ovt_tcm_board_data *bdata = tcm_hcd->hw_if->bdata;        
	gpio_set_value(bdata->reset_gpio, 0);
	msleep(5);
	gpio_set_value(bdata->reset_gpio, 1);        
	msleep(5);
}
static int ovt_tcm_check_f35(struct ovt_tcm_hcd *tcm_hcd)
{
	int retval;
	unsigned char fn_number;
	int retry = 0;
	const int retry_max = 10;

f35_boot_recheck:
			retval = ovt_tcm_rmi_read(tcm_hcd,
						PDT_END_ADDR,
						&fn_number,
						sizeof(fn_number));
			if (retval < 0) {
				OVT_LOG_ERR(
				"Failed to read F35 function number");
				tcm_hcd->is_detected = false;
				return -ENODEV;
			}

			OVT_LOG_ERR(
					"Found F$%02x\n",
					fn_number);

			if (fn_number != RMI_UBL_FN_NUMBER) {
					OVT_LOG_ERR(
							"Failed to find F$35, try_times = %d\n",
							retry);
				if (retry < retry_max) {
					ovt_tcm_simple_hw_reset(tcm_hcd);
					retry++;
					goto f35_boot_recheck;
				}
				tcm_hcd->is_detected = false;
				return -ENODEV;
			}
	return 0;
}

static int ovt_tcm_sensor_detection(struct ovt_tcm_hcd *tcm_hcd)
{
	int retval;
	unsigned char *build_id;
	unsigned int payload_length;
	unsigned int max_write_size;

	tcm_hcd->in_hdl_mode = false;
	tcm_hcd->sensor_type = TYPE_UNKNOWN;

	/* read sensor info for identification */
	retval = tcm_hcd->read_message(tcm_hcd,
			NULL,
			0);

	/* once the tcm communication interface is not ready, */
	/* check whether the device is in F35 mode        */
	if (retval < 0) {
		if (retval == -ENXIO &&
				tcm_hcd->hw_if->bus_io->type == BUS_SPI) {

			retval = ovt_tcm_check_f35(tcm_hcd);
			if (retval < 0) {
				OVT_LOG_ERR(
					"Failed to read TCM message, default to F35");
				retval = 0; // force to F35 here, because on dragon board, when run ovt_tcm_sensor_detection, the display still off, maybe dragon doesn't have little kernel code.
				//return retval;
			}
			tcm_hcd->in_hdl_mode = true;
			tcm_hcd->sensor_type = TYPE_F35;
			tcm_hcd->is_detected = true;
			tcm_hcd->rd_chunk_size = HDL_RD_CHUNK_SIZE;
			tcm_hcd->wr_chunk_size = HDL_WR_CHUNK_SIZE;
			OVT_LOG_INFO(
					"F35 mode");

			return retval;
		} else {
			OVT_LOG_ERR(
				"Failed to read TCM message");

			return retval;
		}
	}

	/* expect to get an identify report after powering on */

	if (tcm_hcd->status_report_code != REPORT_IDENTIFY) {
		OVT_LOG_ERR(
				"Unexpected report code (0x%02x)\n",
				tcm_hcd->status_report_code);

		return -ENODEV;
	}

	tcm_hcd->is_detected = true;
	payload_length = tcm_hcd->payload_length;

	LOCK_BUFFER(tcm_hcd->in);

	retval = secure_memcpy((unsigned char *)&tcm_hcd->id_info,
				sizeof(tcm_hcd->id_info),
				&tcm_hcd->in.buf[MESSAGE_HEADER_SIZE],
				tcm_hcd->in.buf_size - MESSAGE_HEADER_SIZE,
				MIN(sizeof(tcm_hcd->id_info), payload_length));
	if (retval < 0) {
		OVT_LOG_ERR(
				"Failed to copy identification info");
		UNLOCK_BUFFER(tcm_hcd->in);
		return retval;
	}

	UNLOCK_BUFFER(tcm_hcd->in);

	build_id = tcm_hcd->id_info.build_id;
	tcm_hcd->packrat_number = le4_to_uint(build_id);

	max_write_size = le2_to_uint(tcm_hcd->id_info.max_write_size);
	tcm_hcd->wr_chunk_size = MIN(max_write_size, WR_CHUNK_SIZE);
	if (tcm_hcd->wr_chunk_size == 0)
		tcm_hcd->wr_chunk_size = max_write_size;

	if (tcm_hcd->id_info.mode == MODE_ROMBOOTLOADER || tcm_hcd->id_info.mode == MODE_HOSTDOWNLOAD_FIRMWARE) {
		tcm_hcd->in_hdl_mode = true;
		tcm_hcd->sensor_type = TYPE_ROMBOOT;
		tcm_hcd->rd_chunk_size = HDL_RD_CHUNK_SIZE;
		tcm_hcd->wr_chunk_size = HDL_WR_CHUNK_SIZE;
		OVT_LOG_INFO(
					"RomBoot mode");
	} else if (tcm_hcd->id_info.mode == MODE_APPLICATION_FIRMWARE) {
		tcm_hcd->sensor_type = TYPE_FLASH;
		tcm_hcd->rd_chunk_size = RD_CHUNK_SIZE;
		tcm_hcd->wr_chunk_size = WR_CHUNK_SIZE;
		OVT_LOG_INFO(
				"Application mode (build id = %d)\n",
				tcm_hcd->packrat_number);
	} else {
		LOGW(tcm_hcd->pdev->dev.parent,
				"TCM is detected, but mode is 0x%02x\n",
			tcm_hcd->id_info.mode);
	}

	return 0;
}

static int ovt_tcm_probe(struct platform_device *pdev)
{
	int retval;
	struct ovt_tcm_hcd *tcm_hcd;
	const struct ovt_tcm_board_data *bdata;
	const struct ovt_tcm_hw_interface *hw_if;

	tcm_hcd = g_tcm_hcd;

	hw_if = pdev->dev.platform_data;
	if (!hw_if) {
		LOGE(&pdev->dev,
				"Hardware interface not found");
		return -ENODEV;
	}

	bdata = hw_if->bdata;
	if (!bdata) {
		LOGE(&pdev->dev,
				"Board data not found");
		return -ENODEV;
	}

	if (!tcm_hcd) {
		LOGE(&pdev->dev,
				"Failed to allocate memory for tcm_hcd");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, tcm_hcd);

	tcm_hcd->pdev = pdev;
	tcm_hcd->hw_if = hw_if;
	tcm_hcd->reset = ovt_tcm_reset;
	tcm_hcd->reset_n_reinit = ovt_tcm_reset_and_reinit;
	tcm_hcd->sleep = ovt_tcm_sleep;
	tcm_hcd->identify = ovt_tcm_identify;
	tcm_hcd->enable_irq = ovt_tcm_enable_irq;
	tcm_hcd->switch_mode = ovt_tcm_switch_mode;
	tcm_hcd->read_message = ovt_tcm_read_message;
	tcm_hcd->write_message = ovt_tcm_write_message;
	tcm_hcd->get_dynamic_config = ovt_tcm_get_dynamic_config;
	tcm_hcd->set_dynamic_config = ovt_tcm_set_dynamic_config;
	tcm_hcd->get_data_location = ovt_tcm_get_data_location;

	// tcm_hcd->rd_chunk_size = RD_CHUNK_SIZE;
	// tcm_hcd->wr_chunk_size = WR_CHUNK_SIZE;
	tcm_hcd->rd_chunk_size = HDL_RD_CHUNK_SIZE;
	tcm_hcd->wr_chunk_size = HDL_WR_CHUNK_SIZE;
	tcm_hcd->is_detected = false;
	tcm_hcd->wakeup_gesture_enabled = WAKEUP_GESTURE;

#ifdef PREDICTIVE_READING
	tcm_hcd->read_length = MIN_READ_LENGTH;
#else
	tcm_hcd->read_length = MESSAGE_HEADER_SIZE;
#endif

#ifdef WATCHDOG_SW
	tcm_hcd->watchdog.run = RUN_WATCHDOG;
	tcm_hcd->update_watchdog = ovt_tcm_update_watchdog;
#endif

#if 0
	if (bdata->irq_gpio >= 0)
		tcm_hcd->irq = gpio_to_irq(bdata->irq_gpio);
	else
		tcm_hcd->irq = bdata->irq_gpio;
#endif

	mutex_init(&tcm_hcd->extif_mutex);
	mutex_init(&tcm_hcd->reset_mutex);
	mutex_init(&tcm_hcd->irq_en_mutex);
	mutex_init(&tcm_hcd->io_ctrl_mutex);
	mutex_init(&tcm_hcd->rw_ctrl_mutex);
	mutex_init(&tcm_hcd->command_mutex);
	mutex_init(&tcm_hcd->identify_mutex);

	INIT_BUFFER(tcm_hcd->in, false);
	INIT_BUFFER(tcm_hcd->out, false);
	INIT_BUFFER(tcm_hcd->resp, true);
	INIT_BUFFER(tcm_hcd->temp, false);
	INIT_BUFFER(tcm_hcd->config, false);
	INIT_BUFFER(tcm_hcd->report.buffer, true);

	LOCK_BUFFER(tcm_hcd->in);

	retval = ovt_tcm_alloc_mem(tcm_hcd,
			&tcm_hcd->in,
			tcm_hcd->read_length + 1);
	if (retval < 0) {
		LOGE(&pdev->dev,
				"Failed to allocate memory for tcm_hcd->in.buf");
		UNLOCK_BUFFER(tcm_hcd->in);
	}

	UNLOCK_BUFFER(tcm_hcd->in);

	atomic_set(&tcm_hcd->command_status, CMD_IDLE);

	atomic_set(&tcm_hcd->helper.task, HELP_NONE);

	tcm_hcd->helper.helper_completion = &helper_complete;
	complete(tcm_hcd->helper.helper_completion);

	device_init_wakeup(&pdev->dev, 1);

	init_waitqueue_head(&tcm_hcd->hdl_wq);

	init_waitqueue_head(&tcm_hcd->reflash_wq);
	atomic_set(&tcm_hcd->firmware_flashing, 0);

#if 1
	retval = ovt_tcm_get_regulator(tcm_hcd, true);
	if (retval < 0) {
		OVT_LOG_ERR(
				"Failed to get regulators");
	}

	retval = ovt_tcm_enable_regulator(tcm_hcd, true);
	if (retval < 0) {
		OVT_LOG_ERR(
				"Failed to enable regulators");
	}

	retval = ovt_tcm_config_gpio(tcm_hcd);
	if (retval < 0) {
		OVT_LOG_ERR(
				"Failed to configure GPIO's");
	}

#endif
	tcm_hcd->polling_workqueue =
			create_singlethread_workqueue("ovt_tcm_polling");
	INIT_DELAYED_WORK(&tcm_hcd->polling_work, ovt_tcm_polling_work);

	tcm_hcd->helper.workqueue =
			create_singlethread_workqueue("ovt_tcm_helper");
	INIT_WORK(&tcm_hcd->helper.work, ovt_tcm_helper_work);

	return 0;
}

static int ovt_tcm_remove(struct platform_device *pdev)
{
	int idx;
	struct ovt_tcm_module_handler *mod_handler;
	struct ovt_tcm_module_handler *tmp_handler;
	struct ovt_tcm_hcd *tcm_hcd = platform_get_drvdata(pdev);
	const struct ovt_tcm_board_data *bdata = tcm_hcd->hw_if->bdata;

    tcm_hcd->ovt_tcm_driver_removing = 1;

	cancel_work_sync(&tcm_hcd->helper.work);
	flush_workqueue(tcm_hcd->helper.workqueue);
	destroy_workqueue(tcm_hcd->helper.workqueue);


	mutex_lock(&mod_pool.mutex);

	if (!list_empty(&mod_pool.list)) {
		list_for_each_entry_safe(mod_handler,
				tmp_handler,
				&mod_pool.list,
				link) {
			if (mod_handler->mod_cb->remove)
				mod_handler->mod_cb->remove(tcm_hcd);
			list_del(&mod_handler->link);
			kfree(mod_handler);
		}
	}

	mod_pool.queue_work = false;
	cancel_work_sync(&mod_pool.work);
	flush_workqueue(mod_pool.workqueue);
	destroy_workqueue(mod_pool.workqueue);

	mutex_unlock(&mod_pool.mutex);

	touch_remove(tcm_hcd);

	if (tcm_hcd->irq_enabled && bdata->irq_gpio >= 0) {
		disable_irq(tcm_hcd->irq);
		free_irq(tcm_hcd->irq, tcm_hcd);
	}

	cancel_delayed_work_sync(&tcm_hcd->polling_work);
	flush_workqueue(tcm_hcd->polling_workqueue);
	destroy_workqueue(tcm_hcd->polling_workqueue);

#ifdef WATCHDOG_SW
	cancel_delayed_work_sync(&tcm_hcd->watchdog.work);
	flush_workqueue(tcm_hcd->watchdog.workqueue);
	destroy_workqueue(tcm_hcd->watchdog.workqueue);
#endif

#ifdef REPORT_NOTIFIER
	kthread_stop(tcm_hcd->notifier_thread);
#endif

#ifdef CONFIG_FB
	fb_unregister_client(&tcm_hcd->fb_notifier);
#endif

	for (idx = 0; idx < ARRAY_SIZE(dynamic_config_attrs); idx++) {
		sysfs_remove_file(tcm_hcd->dynamnic_config_sysfs_dir,
				&(*dynamic_config_attrs[idx]).attr);
	}

	kobject_put(tcm_hcd->dynamnic_config_sysfs_dir);

	for (idx = 0; idx < ARRAY_SIZE(attrs); idx++)
		sysfs_remove_file(tcm_hcd->sysfs_dir, &(*attrs[idx]).attr);

	kobject_put(tcm_hcd->sysfs_dir);

	if (bdata->irq_gpio >= 0)
		ovt_tcm_set_gpio(tcm_hcd, bdata->irq_gpio, false, 0, 0);

	if (bdata->power_gpio >= 0)
		ovt_tcm_set_gpio(tcm_hcd, bdata->power_gpio, false, 0, 0);

	if (bdata->reset_gpio >= 0)
		ovt_tcm_set_gpio(tcm_hcd, bdata->reset_gpio, false, 0, 0);

	ovt_tcm_enable_regulator(tcm_hcd, false);

	ovt_tcm_get_regulator(tcm_hcd, false);

	device_init_wakeup(&pdev->dev, 0);

	RELEASE_BUFFER(tcm_hcd->report.buffer);
	RELEASE_BUFFER(tcm_hcd->config);
	RELEASE_BUFFER(tcm_hcd->temp);
	RELEASE_BUFFER(tcm_hcd->resp);
	RELEASE_BUFFER(tcm_hcd->out);
	RELEASE_BUFFER(tcm_hcd->in);

	kfree(tcm_hcd);

	return 0;
}
static int ovt_tcm_irq_bottom_half(struct ts_cmd_node *in_cmd,
				     struct ts_cmd_node *out_cmd)
{
	int retval = NO_ERR;
	struct ts_fingers *info =
	    &out_cmd->cmd_param.pub_params.algo_param.info;

	out_cmd->command = TS_INVAILD_CMD;  //default to invalid cmd
	out_cmd->cmd_param.pub_params.algo_param.algo_order =
	    tcm_hcd->ovt_tcm_chip_data->algo_id;

	OVT_LOG_INFO("ovt_tcm_irq_bottom_half");

	retval = g_tcm_hcd->read_message(g_tcm_hcd, NULL, 0);
	if (retval < 0) {
		msleep(500);
		retval = zeroflash_do_hostdownload(g_tcm_hcd);
		if (retval >= 0) { //error
			retval = g_tcm_hcd->identify(g_tcm_hcd, true);
			retval = ovt_touch_init(g_tcm_hcd);
		}
		goto exit;
	}
	if (g_tcm_hcd->in.buf[1] == REPORT_TOUCH) {
		fill_touch_info_data(info);
		out_cmd->command = TS_INPUT_ALGO;
		return NO_ERR;
	} else if (g_tcm_hcd->in.buf[1] == REPORT_IDENTIFY) {
		if (g_tcm_hcd->in.buf[5] == MODE_ROMBOOTLOADER) {
			retval = zeroflash_do_hostdownload(g_tcm_hcd);
			if (retval >= 0) { //error
				retval = g_tcm_hcd->identify(g_tcm_hcd, true);
				retval = ovt_touch_init(g_tcm_hcd);
				g_tcm_hcd->init_ok_flag = true;
				ovt_touch_config_input_dev(g_tcm_hcd->input_dev);
			}
			goto exit;
		}
	}
exit:
	return retval;
}
static void ovt_tcm_shutdown(struct platform_device *pdev)
{
	int retval;

	retval = ovt_tcm_remove(pdev);
}

#ifdef CONFIG_PM
static const struct dev_pm_ops ovt_tcm_dev_pm_ops = {
#ifndef CONFIG_FB
	.suspend = ovt_tcm_suspend,
	.resume = ovt_tcm_resume,
#endif
};
#endif

static struct platform_driver ovt_tcm_driver = {
	.driver = {
		.name = PLATFORM_DRIVER_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_PM
		.pm = &ovt_tcm_dev_pm_ops,
#endif
	},
	.probe = ovt_tcm_probe,
	.remove = ovt_tcm_remove,
	.shutdown = ovt_tcm_shutdown,
};

#define OMNIVISION_VENDER_NAME "omnivision_tcm"
static int __init ovt_tcm_module_init(void)
{
	int retval = 0;    
	bool found = false;
	struct device_node* child = NULL;
	struct device_node* root = NULL;

	OVT_LOG_INFO("ovt_tcm_module_init called");
	
	root = of_find_compatible_node(NULL, NULL, "huawei,ts_kit");
	if (!root) {
		OVT_LOG_ERR("can not find_compatible_node huawei,ts_kit error");
		retval = -EINVAL;
		goto out;
	}

	for_each_child_of_node(root, child)
	{
		if (of_device_is_compatible(child, OMNIVISION_VENDER_NAME)) {
			OVT_LOG_INFO("%s dtsi node in huawei,ts_kit found\n", OMNIVISION_VENDER_NAME);
			found = true;
			break;
		} 
	}

	if (!found) {
		OVT_LOG_ERR(" not found chip omnivision child node in huawei ts kit  !");
		retval = -EINVAL;
		goto out;
	}

	tcm_hcd = kzalloc(sizeof(*tcm_hcd), GFP_KERNEL);
	g_tcm_hcd = tcm_hcd;

	if (!tcm_hcd) {
		OVT_LOG_ERR("Failed to allocate memory for tcm_hcd");
		return -ENOMEM;
	}

	tcm_hcd->ovt_tcm_chip_data = kzalloc(sizeof(struct ts_kit_device_data), GFP_KERNEL);
	if (!tcm_hcd->ovt_tcm_chip_data) {
		OVT_LOG_ERR("Failed to allocate memory for tcm_hcd chip data");
		return -ENOMEM;
	}

    tcm_hcd->ovt_tcm_chip_data->cnode = child;
    tcm_hcd->ovt_tcm_chip_data->ops = &ts_kit_ovt_tcm_ops;

	retval = huawei_ts_chip_register(tcm_hcd->ovt_tcm_chip_data);
	if(retval)
	{
	  OVT_LOG_ERR(" ovtptics chip register fail !");
	  goto out;
	}

	return retval;
	
out:
	if (tcm_hcd) {
		if (tcm_hcd->ovt_tcm_chip_data)
			kfree(tcm_hcd->ovt_tcm_chip_data);
	}

	if (tcm_hcd)
		kfree(tcm_hcd);
	tcm_hcd = NULL;	
	return retval;
}

static void __exit ovt_tcm_module_exit(void)
{
	platform_driver_unregister(&ovt_tcm_driver);

	//ovt_tcm_bus_exit();

	return;
}

late_initcall(ovt_tcm_module_init);
module_exit(ovt_tcm_module_exit);

MODULE_AUTHOR("Omnivision, Inc.");
MODULE_DESCRIPTION("Omnivision TCM Touch Driver");
MODULE_LICENSE("GPL v2");
