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
#include "synaptics_tcm_core.h"
#include <linux/input/mt.h>

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

#include "synaptics_tcm_core.h"
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

#define RUN_WATCHDOG false

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

#define dynamic_config_sysfs(c_name, id) \
static ssize_t syna_tcm_sysfs_##c_name##_show(struct device *dev, \
		struct device_attribute *attr, char *buf) \
{ \
	int retval; \
	unsigned short value; \
	struct device *p_dev; \
	struct kobject *p_kobj; \
	struct syna_tcm_hcd *tcm_hcd; \
\
	p_kobj = sysfs_dir->parent; \
	p_dev = container_of(p_kobj, struct device, kobj); \
	tcm_hcd = dev_get_drvdata(p_dev); \
\
	mutex_lock(&tcm_hcd->extif_mutex); \
\
	retval = tcm_hcd->get_dynamic_config(tcm_hcd, id, &value); \
	if (retval < 0) { \
		LOGE(tcm_hcd->pdev->dev.parent, \
				"Failed to get dynamic config\n"); \
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
static ssize_t syna_tcm_sysfs_##c_name##_store(struct device *dev, \
		struct device_attribute *attr, const char *buf, size_t count) \
{ \
	int retval; \
	unsigned int input; \
	struct device *p_dev; \
	struct kobject *p_kobj; \
	struct syna_tcm_hcd *tcm_hcd; \
\
	p_kobj = sysfs_dir->parent; \
	p_dev = container_of(p_kobj, struct device, kobj); \
	tcm_hcd = dev_get_drvdata(p_dev); \
\
	if (sscanf(buf, "%u", &input) != 1) \
		return -EINVAL; \
\
	mutex_lock(&tcm_hcd->extif_mutex); \
\
	retval = tcm_hcd->set_dynamic_config(tcm_hcd, id, input); \
	if (retval < 0) { \
		LOGE(tcm_hcd->pdev->dev.parent, \
				"Failed to set dynamic config\n"); \
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

DECLARE_COMPLETION(response_complete);

static struct kobject *sysfs_dir;

static struct syna_tcm_module_pool mod_pool;

SHOW_PROTOTYPE(syna_tcm, info)
STORE_PROTOTYPE(syna_tcm, irq_en)
STORE_PROTOTYPE(syna_tcm, reset)
STORE_PROTOTYPE(syna_tcm, watchdog)
SHOW_STORE_PROTOTYPE(syna_tcm, no_doze)
SHOW_STORE_PROTOTYPE(syna_tcm, disable_noise_mitigation)
SHOW_STORE_PROTOTYPE(syna_tcm, inhibit_frequency_shift)
SHOW_STORE_PROTOTYPE(syna_tcm, requested_frequency)
SHOW_STORE_PROTOTYPE(syna_tcm, disable_hsync)
SHOW_STORE_PROTOTYPE(syna_tcm, rezero_on_exit_deep_sleep)
SHOW_STORE_PROTOTYPE(syna_tcm, charger_connected)
SHOW_STORE_PROTOTYPE(syna_tcm, no_baseline_relaxation)
SHOW_STORE_PROTOTYPE(syna_tcm, in_wakeup_gesture_mode)
SHOW_STORE_PROTOTYPE(syna_tcm, stimulus_fingers)
SHOW_STORE_PROTOTYPE(syna_tcm, grip_suppression_enabled)
SHOW_STORE_PROTOTYPE(syna_tcm, enable_thick_glove)
SHOW_STORE_PROTOTYPE(syna_tcm, enable_glove)

static struct device_attribute *attrs[] = {
	ATTRIFY(info),
	ATTRIFY(irq_en),
	ATTRIFY(reset),
	ATTRIFY(watchdog),
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

static ssize_t syna_tcm_sysfs_info_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int retval;
	unsigned int count;
	struct device *p_dev;
	struct kobject *p_kobj;
	struct syna_tcm_hcd *tcm_hcd;

	p_kobj = sysfs_dir->parent;
	p_dev = container_of(p_kobj, struct device, kobj);
	tcm_hcd = dev_get_drvdata(p_dev);

	mutex_lock(&tcm_hcd->extif_mutex);

	retval = tcm_hcd->identify(tcm_hcd, true);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to do identification\n");
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

	retval = snprintf(buf, PAGE_SIZE - count,
			"Driver version:     %d.%d\n",
			(unsigned char)(SYNAPTICS_TCM_ID_VERSION >> 8),
			(unsigned char)SYNAPTICS_TCM_ID_VERSION);
	if (retval < 0)
		goto exit;

	buf += retval;
	count += retval;

	switch (tcm_hcd->id_info.mode) {
	case MODE_APPLICATION:
		retval = snprintf(buf, PAGE_SIZE - count,
				"Firmware mode:      Application\n");
		if (retval < 0)
			goto exit;
		break;
	case MODE_HOST_DOWNLOAD:
		retval = snprintf(buf, PAGE_SIZE - count,
				"Firmware mode:      Host Download\n");
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
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to copy part number string\n");
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

static ssize_t syna_tcm_sysfs_irq_en_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned int input;
	struct device *p_dev;
	struct kobject *p_kobj;
	struct syna_tcm_hcd *tcm_hcd;

	p_kobj = sysfs_dir->parent;
	p_dev = container_of(p_kobj, struct device, kobj);
	tcm_hcd = dev_get_drvdata(p_dev);

	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	mutex_lock(&tcm_hcd->extif_mutex);

	if (input == 0) {
		retval = tcm_hcd->enable_irq(tcm_hcd, false, true);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to disable interrupt\n");
			goto exit;
		}
	} else if (input == 1) {
		retval = tcm_hcd->enable_irq(tcm_hcd, true, NULL);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to enable interrupt\n");
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

static ssize_t syna_tcm_sysfs_reset_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int retval;
	bool hw_reset;
	unsigned int input;
	struct device *p_dev;
	struct kobject *p_kobj;
	struct syna_tcm_hcd *tcm_hcd;

	p_kobj = sysfs_dir->parent;
	p_dev = container_of(p_kobj, struct device, kobj);
	tcm_hcd = dev_get_drvdata(p_dev);

	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	if (input == 1)
		hw_reset = false;
	else if (input == 2)
		hw_reset = true;
	else
		return -EINVAL;

	mutex_lock(&tcm_hcd->extif_mutex);

	retval = tcm_hcd->reset(tcm_hcd, hw_reset, true);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to do reset\n");
		goto exit;
	}

	retval = count;

exit:
	mutex_unlock(&tcm_hcd->extif_mutex);

	return retval;
}

static ssize_t syna_tcm_sysfs_watchdog_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int input;
	struct device *p_dev;
	struct kobject *p_kobj;
	struct syna_tcm_hcd *tcm_hcd;

	p_kobj = sysfs_dir->parent;
	p_dev = container_of(p_kobj, struct device, kobj);
	tcm_hcd = dev_get_drvdata(p_dev);

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

int syna_tcm_add_module(struct syna_tcm_module_cb *mod_cb, bool insert)
{
	struct syna_tcm_module_handler *mod_handler;

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
EXPORT_SYMBOL(syna_tcm_add_module);

static void syna_tcm_module_work(struct work_struct *work)
{
	struct syna_tcm_module_handler *mod_handler;
	struct syna_tcm_module_handler *tmp_handler;
	struct syna_tcm_hcd *tcm_hcd = mod_pool.tcm_hcd;

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

/**
 * syna_tcm_report_notifier() - notify occurrence of report received from device
 *
 * @data: handle of core module
 *
 * The occurrence of the report generated by the device is forwarded to the
 * asynchronous inbox of each registered application module.
 */
static int syna_tcm_report_notifier(void *data)
{
	int retval;
	struct sched_param param = { .sched_priority = NOTIFIER_PRIORITY };
	struct syna_tcm_module_handler *mod_handler;
	struct syna_tcm_hcd *tcm_hcd = data;

	sched_setscheduler(current, SCHED_RR, &param);

	while (!kthread_should_stop()) {
		set_current_state(TASK_INTERRUPTIBLE);

		retval = wait_event_interruptible_timeout(tcm_hcd->report_wq,
				tcm_hcd->dispatch_report == true,
				msecs_to_jiffies(NOTIFIER_TIMEOUT_MS));
		if (retval == 0) {
			if (kthread_should_stop())
				break;
			else
				continue;
		} else if (retval < 0) {
			continue;
		}

		tcm_hcd->dispatch_report = false;

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
	};

	return 0;
}

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
	struct syna_tcm_module_handler *mod_handler;

	LOCK_BUFFER(tcm_hcd->in);
	LOCK_BUFFER(tcm_hcd->report.buffer);

	tcm_hcd->report.buffer.buf = &tcm_hcd->in.buf[MESSAGE_HEADER_SIZE];

	tcm_hcd->report.buffer.buf_size = tcm_hcd->in.buf_size;
	tcm_hcd->report.buffer.buf_size -= MESSAGE_HEADER_SIZE;

	tcm_hcd->report.buffer.data_length = tcm_hcd->payload_length;

	tcm_hcd->report.id = tcm_hcd->status_report_code;

	mutex_lock(&mod_pool.mutex);

	if (!list_empty(&mod_pool.list)) {
		list_for_each_entry(mod_handler, &mod_pool.list, link) {
			if (!mod_handler->insert &&
					!mod_handler->detach &&
					(mod_handler->mod_cb->syncbox))
				mod_handler->mod_cb->syncbox(tcm_hcd);
		}
	}


	tcm_hcd->async_report_id = tcm_hcd->status_report_code;

	mutex_unlock(&mod_pool.mutex);

	UNLOCK_BUFFER(tcm_hcd->report.buffer);
	UNLOCK_BUFFER(tcm_hcd->in);

	tcm_hcd->dispatch_report = true;

	wake_up_interruptible(&tcm_hcd->report_wq);

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
	int retval;

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
		LOGE(tcm_hcd->pdev->dev.parent,
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
		LOGE(tcm_hcd->pdev->dev.parent,
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
			LOGE(tcm_hcd->pdev->dev.parent,
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

		LOGN(tcm_hcd->pdev->dev.parent,
				"Received identify report (firmware mode = 0x%02x)\n",
				tcm_hcd->id_info.mode);

		if (atomic_read(&tcm_hcd->command_status) == CMD_BUSY) {
			switch (tcm_hcd->command) {
			case CMD_RESET:
			case CMD_RUN_BOOTLOADER_FIRMWARE:
			case CMD_RUN_APPLICATION_FIRMWARE:
				atomic_set(&tcm_hcd->command_status, CMD_IDLE);
				complete(&response_complete);
				break;
			default:
				LOGN(tcm_hcd->pdev->dev.parent,
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
		if (tcm_hcd->id_info.mode != MODE_APPLICATION &&
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

	retval = syna_tcm_realloc_mem(tcm_hcd,
			&tcm_hcd->in,
			total_length);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to reallocate memory for tcm_hcd->in.buf\n");
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

		retval = syna_tcm_alloc_mem(tcm_hcd,
				&tcm_hcd->temp,
				xfer_length + 2);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to allocate memory for tcm_hcd->temp.buf\n");
			UNLOCK_BUFFER(tcm_hcd->temp);
			UNLOCK_BUFFER(tcm_hcd->in);
			return retval;
		}

		retval = syna_tcm_read(tcm_hcd,
				tcm_hcd->temp.buf,
				xfer_length + 2);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to read from device\n");
			UNLOCK_BUFFER(tcm_hcd->temp);
			UNLOCK_BUFFER(tcm_hcd->in);
			return retval;
		}

		marker = tcm_hcd->temp.buf[0];
		code = tcm_hcd->temp.buf[1];

		if (marker != MESSAGE_MARKER) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Incorrect header marker (0x%02x)\n",
					marker);
			UNLOCK_BUFFER(tcm_hcd->temp);
			UNLOCK_BUFFER(tcm_hcd->in);
			return -EIO;
		}

		if (code != STATUS_CONTINUED_READ) {
			LOGE(tcm_hcd->pdev->dev.parent,
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
			LOGE(tcm_hcd->pdev->dev.parent,
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
static int syna_tcm_raw_read(struct syna_tcm_hcd *tcm_hcd,
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

	if (length < 2) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Invalid length information\n");
		return -EINVAL;
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

		retval = syna_tcm_alloc_mem(tcm_hcd,
				&tcm_hcd->temp,
				xfer_length + 2);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to allocate memory for tcm_hcd->temp.buf\n");
			UNLOCK_BUFFER(tcm_hcd->temp);
			return retval;
		}

		retval = syna_tcm_read(tcm_hcd,
				tcm_hcd->temp.buf,
				xfer_length + 2);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to read from device\n");
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
				LOGE(tcm_hcd->pdev->dev.parent,
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
			LOGE(tcm_hcd->pdev->dev.parent,
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
static int syna_tcm_raw_write(struct syna_tcm_hcd *tcm_hcd,
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

		retval = syna_tcm_alloc_mem(tcm_hcd,
				&tcm_hcd->out,
				xfer_length + 1);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
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
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to copy data\n");
				UNLOCK_BUFFER(tcm_hcd->out);
				return retval;
			}
		}

		retval = syna_tcm_write(tcm_hcd,
				tcm_hcd->out.buf,
				xfer_length + 1);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
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
	int retval;
	bool retry;
	unsigned int total_length;
	struct syna_tcm_message_header *header;

	mutex_lock(&tcm_hcd->rw_ctrl_mutex);

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
		LOGE(tcm_hcd->pdev->dev.parent,
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
		LOGE(tcm_hcd->pdev->dev.parent,
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

	LOGD(tcm_hcd->pdev->dev.parent,
			"Header code = 0x%02x\n",
			tcm_hcd->status_report_code);

	LOGD(tcm_hcd->pdev->dev.parent,
			"Payload length = %d\n",
			tcm_hcd->payload_length);

	if (tcm_hcd->status_report_code <= STATUS_ERROR ||
			tcm_hcd->status_report_code == STATUS_INVALID) {
		switch (tcm_hcd->status_report_code) {
		case STATUS_OK:
			break;
		case STATUS_CONTINUED_READ:
			LOGD(tcm_hcd->pdev->dev.parent,
					"Out-of-sync continued read\n");
		case STATUS_IDLE:
		case STATUS_BUSY:
			tcm_hcd->payload_length = 0;
			UNLOCK_BUFFER(tcm_hcd->in);
			retval = 0;
			goto exit;
		default:
			LOGE(tcm_hcd->pdev->dev.parent,
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
		LOGE(tcm_hcd->pdev->dev.parent,
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
		LOGE(tcm_hcd->pdev->dev.parent,
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

	mutex_unlock(&tcm_hcd->rw_ctrl_mutex);

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
	int retval;
	unsigned int idx;
	unsigned int chunks;
	unsigned int chunk_space;
	unsigned int xfer_length;
	unsigned int remaining_length;
	unsigned int command_status;

	if (response_code != NULL)
		*response_code = STATUS_INVALID;

	if (!tcm_hcd->do_polling && current->pid == tcm_hcd->isr_pid) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Invalid execution context\n");
		return -EINVAL;
	}

	mutex_lock(&tcm_hcd->command_mutex);

	mutex_lock(&tcm_hcd->rw_ctrl_mutex);

	if (resp_buf == NULL) {
		retval = syna_tcm_raw_write(tcm_hcd, command, payload, length);
		mutex_unlock(&tcm_hcd->rw_ctrl_mutex);
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

	/* adding two length bytes as part of payload */
	remaining_length = length + 2;

	/* available chunk space for payload = total chunk size minus command
	 * byte */
	if (tcm_hcd->wr_chunk_size == 0)
		chunk_space = remaining_length;
	else
		chunk_space = tcm_hcd->wr_chunk_size - 1;

	chunks = ceil_div(remaining_length, chunk_space);

	chunks = chunks == 0 ? 1 : chunks;

	LOGD(tcm_hcd->pdev->dev.parent,
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
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to allocate memory for tcm_hcd->out.buf\n");
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
					LOGE(tcm_hcd->pdev->dev.parent,
							"Failed to copy payload\n");
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
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to copy payload\n");
				UNLOCK_BUFFER(tcm_hcd->out);
				mutex_unlock(&tcm_hcd->rw_ctrl_mutex);
				goto exit;
			}
		}

		retval = syna_tcm_write(tcm_hcd,
				tcm_hcd->out.buf,
				xfer_length + 1);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to write to device\n");
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
/*
	if (tcm_hcd->do_polling && polling_delay_ms) {
		queue_delayed_work(tcm_hcd->polling_workqueue,
				&tcm_hcd->polling_work,
				msecs_to_jiffies(polling_delay_ms));
	}
*/
	if (tcm_hcd->do_polling) {
		if (polling_delay_ms == 0) {
			queue_delayed_work(tcm_hcd->polling_workqueue,
				&tcm_hcd->polling_work,
				msecs_to_jiffies(10));
		} else {
			queue_delayed_work(tcm_hcd->polling_workqueue,
				&tcm_hcd->polling_work,
				msecs_to_jiffies(polling_delay_ms));
		}	
	}

	if (tcm_hcd->command == CMD_ERASE_FLASH) {
		retval = wait_for_completion_timeout(&response_complete,
			msecs_to_jiffies(10000));
	} else {
		retval = wait_for_completion_timeout(&response_complete,
			msecs_to_jiffies(RESPONSE_TIMEOUT_MS));
	}

	if (retval == 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Timed out waiting for response (command 0x%02x)\n",
				tcm_hcd->command);
		retval = -EIO;
		goto exit;
	}

	command_status = atomic_read(&tcm_hcd->command_status);
	if (command_status != CMD_IDLE) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to get valid response (command 0x%02x)\n",
				tcm_hcd->command);
		retval = -EIO;
		goto exit;
	}

	LOCK_BUFFER(tcm_hcd->resp);

	if (tcm_hcd->response_code != STATUS_OK) {
		if (tcm_hcd->resp.data_length) {
			LOGE(tcm_hcd->pdev->dev.parent,
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

	mutex_unlock(&tcm_hcd->command_mutex);

	return retval;
}

static int syna_tcm_wait_hdl(struct syna_tcm_hcd *tcm_hcd)
{
	int retval;

	msleep(HOST_DOWNLOAD_WAIT_MS);

	if (!atomic_read(&tcm_hcd->host_downloading))
		return 0;

	retval = wait_event_interruptible_timeout(tcm_hcd->hdl_wq,
			!atomic_read(&tcm_hcd->host_downloading),
			msecs_to_jiffies(HOST_DOWNLOAD_TIMEOUT_MS));
	if (retval == 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Timed out waiting for completion of host download\n");
		retval = -EIO;
	} else {
		retval = 0;
	}

	return retval;
}

static void syna_tcm_check_hdl(struct syna_tcm_hcd *tcm_hcd)
{
	struct syna_tcm_module_handler *mod_handler;

	LOCK_BUFFER(tcm_hcd->report.buffer);

	tcm_hcd->report.buffer.buf = NULL;
	tcm_hcd->report.buffer.buf_size = 0;
	tcm_hcd->report.buffer.data_length = 0;
	tcm_hcd->report.id = REPORT_HDL;

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

static void syna_tcm_update_watchdog(struct syna_tcm_hcd *tcm_hcd, bool en)
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

static void syna_tcm_watchdog_work(struct work_struct *work)
{
	int retval;
	unsigned char marker;
	struct delayed_work *delayed_work =
			container_of(work, struct delayed_work, work);
	struct syna_tcm_watchdog *watchdog =
			container_of(delayed_work, struct syna_tcm_watchdog,
			work);
	struct syna_tcm_hcd *tcm_hcd =
			container_of(watchdog, struct syna_tcm_hcd, watchdog);

	if (mutex_is_locked(&tcm_hcd->rw_ctrl_mutex))
		goto exit;

	mutex_lock(&tcm_hcd->rw_ctrl_mutex);

	retval = syna_tcm_read(tcm_hcd,
			&marker,
			1);

	mutex_unlock(&tcm_hcd->rw_ctrl_mutex);

	if (retval < 0 || marker != MESSAGE_MARKER) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to read from device\n");

		tcm_hcd->watchdog.count++;

		if (tcm_hcd->watchdog.count >= WATCHDOG_TRIGGER_COUNT) {
			retval = tcm_hcd->reset(tcm_hcd, true, false);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to do reset\n");
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

static void syna_tcm_polling_work(struct work_struct *work)
{
	int retval;
	struct delayed_work *delayed_work =
			container_of(work, struct delayed_work, work);
	struct syna_tcm_hcd *tcm_hcd =
			container_of(delayed_work, struct syna_tcm_hcd,
			polling_work);

	if (!tcm_hcd->do_polling)
		return;
	TS_LOG_ERR("syna polling work running\n");

	retval = tcm_hcd->read_message(tcm_hcd,
			NULL,
			0);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to read message\n");
		if (retval == -ENXIO && tcm_hcd->hw_if->bus_io->type == BUS_SPI)
			syna_tcm_check_hdl(tcm_hcd);
	}

	if (!(tcm_hcd->in_suspend && retval < 0)) {
		queue_delayed_work(tcm_hcd->polling_workqueue,
				&tcm_hcd->polling_work,
				msecs_to_jiffies(POLLING_DELAY_MS));
	}

	return;
}

static irqreturn_t syna_tcm_isr(int irq, void *data)
{
	int retval;
	struct syna_tcm_hcd *tcm_hcd = data;
	const struct syna_tcm_board_data *bdata = tcm_hcd->hw_if->bdata;

	if (unlikely(gpio_get_value(bdata->irq_gpio) != bdata->irq_on_state))
		goto exit;

	tcm_hcd->isr_pid = current->pid;

	retval = tcm_hcd->read_message(tcm_hcd,
			NULL,
			0);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to read message\n");
		if (retval == -ENXIO && tcm_hcd->hw_if->bus_io->type == BUS_SPI)
			syna_tcm_check_hdl(tcm_hcd);
	}

exit:
	return IRQ_HANDLED;
}

static int syna_tcm_enable_irq(struct syna_tcm_hcd *tcm_hcd, bool en, bool ns)
{
	int retval;
	const struct syna_tcm_board_data *bdata = tcm_hcd->hw_if->bdata;
	static bool irq_freed = true;

	//control irq enable/disable in huawei ts kit common driver
	return retval = 0;
	mutex_lock(&tcm_hcd->irq_en_mutex);

	if (en) {
		if (tcm_hcd->irq_enabled) {
			LOGD(tcm_hcd->pdev->dev.parent,
					"Interrupt already enabled\n");
			retval = 0;
			goto exit;
		}

		if (bdata->irq_gpio < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Invalid IRQ GPIO\n");
			retval = -EINVAL;
			goto queue_polling_work;
		}

		if (irq_freed) {
			retval = request_threaded_irq(tcm_hcd->irq, NULL,
					syna_tcm_isr, bdata->irq_flags,
					PLATFORM_DRIVER_NAME, tcm_hcd);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to create interrupt thread\n");
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
					"Interrupt already disabled\n");
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

static int syna_tcm_set_gpio(struct syna_tcm_hcd *tcm_hcd, int gpio,
		bool config, int dir, int state)
{
	int retval;
	char label[16];

	if (config) {
		retval = snprintf(label, 16, "tcm_gpio_%d\n", gpio);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to set GPIO label\n");
			return retval;
		}

		retval = gpio_request(gpio, label);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to request GPIO %d\n",
					gpio);
			return retval;
		}

		if (dir == 0)
			retval = gpio_direction_input(gpio);
		else
			retval = gpio_direction_output(gpio, state);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to set GPIO %d direction\n",
					gpio);
			return retval;
		}
	} else {
		gpio_free(gpio);
	}

	return 0;
}

static int syna_tcm_config_gpio(struct syna_tcm_hcd *tcm_hcd)
{
	int retval;
	const struct syna_tcm_board_data *bdata = tcm_hcd->hw_if->bdata;

	if (bdata->irq_gpio >= 0) {
		retval = syna_tcm_set_gpio(tcm_hcd, bdata->irq_gpio,
				true, 0, 0);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to configure interrupt GPIO\n");
			goto err_set_gpio_irq;
		}
	}

	if (bdata->power_gpio >= 0) {
		retval = syna_tcm_set_gpio(tcm_hcd, bdata->power_gpio,
				true, 1, !bdata->power_on_state);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to configure power GPIO\n");
			goto err_set_gpio_power;
		}
	}

	if (bdata->reset_gpio >= 0) {
		retval = syna_tcm_set_gpio(tcm_hcd, bdata->reset_gpio,
				true, 1, !bdata->reset_on_state);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to configure reset GPIO\n");
			goto err_set_gpio_reset;
		}
	}

	if (bdata->power_gpio >= 0) {
		gpio_set_value(bdata->power_gpio, bdata->power_on_state);
		msleep(bdata->power_delay_ms);
	}

	if (bdata->reset_gpio >= 0) {
		gpio_set_value(bdata->reset_gpio, bdata->reset_on_state);
		msleep(bdata->reset_active_ms);
		gpio_set_value(bdata->reset_gpio, !bdata->reset_on_state);
		msleep(bdata->reset_delay_ms);
	}

	return 0;

err_set_gpio_reset:
	if (bdata->power_gpio >= 0)
		syna_tcm_set_gpio(tcm_hcd, bdata->power_gpio, false, 0, 0);

err_set_gpio_power:
	if (bdata->irq_gpio >= 0)
		syna_tcm_set_gpio(tcm_hcd, bdata->irq_gpio, false, 0, 0);

err_set_gpio_irq:
	return retval;
}

static int syna_tcm_enable_regulator(struct syna_tcm_hcd *tcm_hcd, bool en)
{
	int retval;
	const struct syna_tcm_board_data *bdata = tcm_hcd->hw_if->bdata;

	if (!en) {
		retval = 0;
		goto disable_pwr_reg;
	}

	if (tcm_hcd->bus_reg) {
		retval = regulator_enable(tcm_hcd->bus_reg);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to enable bus regulator\n");
			goto exit;
		}
	}

	if (tcm_hcd->pwr_reg) {
		retval = regulator_enable(tcm_hcd->pwr_reg);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to enable power regulator\n");
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

static int syna_tcm_get_regulator(struct syna_tcm_hcd *tcm_hcd, bool get)
{
	int retval;
	const struct syna_tcm_board_data *bdata = tcm_hcd->hw_if->bdata;

	if (!get) {
		retval = 0;
		goto regulator_put;
	}

	if (bdata->bus_reg_name != NULL && *bdata->bus_reg_name != 0) {
		tcm_hcd->bus_reg = regulator_get(tcm_hcd->pdev->dev.parent,
				bdata->bus_reg_name);
		if (IS_ERR(tcm_hcd->bus_reg)) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to get bus regulator\n");
			retval = PTR_ERR(tcm_hcd->bus_reg);
			goto regulator_put;
		}
	}

	if (bdata->pwr_reg_name != NULL && *bdata->pwr_reg_name != 0) {
		tcm_hcd->pwr_reg = regulator_get(tcm_hcd->pdev->dev.parent,
				bdata->pwr_reg_name);
		if (IS_ERR(tcm_hcd->pwr_reg)) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to get power regulator\n");
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

static int syna_tcm_get_app_info(struct syna_tcm_hcd *tcm_hcd)
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
			0);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
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
		LOGE(tcm_hcd->pdev->dev.parent,
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
		LOGE(tcm_hcd->pdev->dev.parent,
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
		LOGE(tcm_hcd->pdev->dev.parent,
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
	int retval;
	unsigned char *resp_buf;
	unsigned int resp_buf_size;
	unsigned int resp_length;
	unsigned int max_write_size;

	resp_buf = NULL;
	resp_buf_size = 0;

	mutex_lock(&tcm_hcd->identify_mutex);

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
		LOGE(tcm_hcd->pdev->dev.parent,
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
		LOGE(tcm_hcd->pdev->dev.parent,
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
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to get application info\n");
			goto exit;
		}
		break;
	case MODE_BOOTLOADER:
	case MODE_TDDI_BOOTLOADER:
		retval = syna_tcm_get_boot_info(tcm_hcd);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to get boot info\n");
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

static int syna_tcm_run_application_firmware(struct syna_tcm_hcd *tcm_hcd)
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
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to write command %s\n",
				STR(CMD_RUN_APPLICATION_FIRMWARE));
		goto exit;
	}

	retval = tcm_hcd->identify(tcm_hcd, false);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to do identification\n");
		goto exit;
	}

	if (tcm_hcd->id_info.mode != MODE_APPLICATION) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to run application firmware (boot status = 0x%02x)\n",
				tcm_hcd->boot_info.status);
		if (retry) {
			retry = false;
			goto retry;
		}
		retval = -EINVAL;
		goto exit;
	} else if (tcm_hcd->app_status != APP_STATUS_OK) {
		LOGE(tcm_hcd->pdev->dev.parent,
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
	int retval;
	unsigned char *resp_buf;
	unsigned int resp_buf_size;
	unsigned int resp_length;

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
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to write command %s\n",
				STR(CMD_RUN_BOOTLOADER_FIRMWARE));
		goto exit;
	}

	retval = tcm_hcd->identify(tcm_hcd, false);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to do identification\n");
		goto exit;
	}

	if (tcm_hcd->id_info.mode == MODE_APPLICATION) {
		LOGE(tcm_hcd->pdev->dev.parent,
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
	int retval;

	mutex_lock(&tcm_hcd->reset_mutex);

	tcm_hcd->update_watchdog(tcm_hcd, false);

	switch (mode) {
	case FW_MODE_BOOTLOADER:
		retval = syna_tcm_run_bootloader_firmware(tcm_hcd);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to switch to bootloader mode\n");
			goto exit;
		}
		break;
	case FW_MODE_APPLICATION:
		retval = syna_tcm_run_application_firmware(tcm_hcd);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to switch to application mode\n");
			goto exit;
		}
		break;
	default:
		LOGE(tcm_hcd->pdev->dev.parent,
				"Invalid firmware mode\n");
		retval = -EINVAL;
		goto exit;
	}

	retval = 0;

exit:
	tcm_hcd->update_watchdog(tcm_hcd, true);

	mutex_unlock(&tcm_hcd->reset_mutex);

	return retval;
}

static int syna_tcm_get_dynamic_config(struct syna_tcm_hcd *tcm_hcd,
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
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to write command %s\n",
				STR(CMD_GET_DYNAMIC_CONFIG));
		goto exit;
	}

	if (resp_length < 2) {
		LOGE(tcm_hcd->pdev->dev.parent,
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
		LOGE(tcm_hcd->pdev->dev.parent,
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
		LOGE(tcm_hcd->pdev->dev.parent,
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
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to write command %s\n",
				STR(CMD_GET_DATA_LOCATION));
		goto exit;
	}

	if (resp_length != 4) {
		LOGE(tcm_hcd->pdev->dev.parent,
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
		LOGE(tcm_hcd->pdev->dev.parent,
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
	int retval;
	unsigned char *resp_buf;
	unsigned int resp_buf_size;
	unsigned int resp_length;
	struct syna_tcm_module_handler *mod_handler;
	const struct syna_tcm_board_data *bdata = tcm_hcd->hw_if->bdata;

	resp_buf = NULL;
	resp_buf_size = 0;

	mutex_lock(&tcm_hcd->reset_mutex);

	if (update_wd)
		tcm_hcd->update_watchdog(tcm_hcd, false);

	if (hw) {
		if (bdata->reset_gpio < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Hardware reset unavailable\n");
			retval = -EINVAL;
			goto exit;
		}
		gpio_set_value(bdata->reset_gpio, bdata->reset_on_state);
		msleep(bdata->reset_active_ms);
		gpio_set_value(bdata->reset_gpio, !bdata->reset_on_state);
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
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to write command %s\n",
					STR(CMD_RESET));
			goto exit;
		}
	}

	if (tcm_hcd->host_download_mode) {
		mutex_unlock(&tcm_hcd->reset_mutex);
		kfree(resp_buf);
		retval = syna_tcm_wait_hdl(tcm_hcd);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
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
		LOGE(tcm_hcd->pdev->dev.parent,
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
		LOGN(tcm_hcd->pdev->dev.parent,
				"Failed to write command %s\n",
				STR(CMD_RUN_APPLICATION_FIRMWARE));
	}

	retval = tcm_hcd->identify(tcm_hcd, false);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to do identification\n");
		goto exit;
	}

dispatch_reset:
	LOGN(tcm_hcd->pdev->dev.parent,
			"Firmware mode = 0x%02x\n",
			tcm_hcd->id_info.mode);

	if (tcm_hcd->id_info.mode != MODE_APPLICATION) {
		LOGN(tcm_hcd->pdev->dev.parent,
				"Boot status = 0x%02x\n",
				tcm_hcd->boot_info.status);
	} else if (tcm_hcd->app_status != APP_STATUS_OK) {
		LOGN(tcm_hcd->pdev->dev.parent,
				"Application status = 0x%02x\n",
				tcm_hcd->app_status);
	}

	mutex_lock(&mod_pool.mutex);

	if (!list_empty(&mod_pool.list)) {
		list_for_each_entry(mod_handler, &mod_pool.list, link) {
			if (!mod_handler->insert &&
					!mod_handler->detach &&
					(mod_handler->mod_cb->reset))
				mod_handler->mod_cb->reset(tcm_hcd);
		}
	}

	mutex_unlock(&mod_pool.mutex);

	retval = 0;

exit:
	if (update_wd)
		tcm_hcd->update_watchdog(tcm_hcd, true);

	mutex_unlock(&tcm_hcd->reset_mutex);

	kfree(resp_buf);

	return retval;
}

static int syna_tcm_rezero(struct syna_tcm_hcd *tcm_hcd)
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
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to write command %s\n",
				STR(CMD_REZERO));
		goto exit;
	}

	retval = 0;

exit:
	kfree(resp_buf);

	return retval;
}

static void syna_tcm_helper_work(struct work_struct *work)
{
	int retval;
	unsigned char task;
	struct syna_tcm_module_handler *mod_handler;
	struct syna_tcm_helper *helper =
			container_of(work, struct syna_tcm_helper, work);
	struct syna_tcm_hcd *tcm_hcd =
			container_of(helper, struct syna_tcm_hcd, helper);

	task = atomic_read(&helper->task);

	switch (task) {
	case HELP_RUN_APPLICATION_FIRMWARE:
		mutex_lock(&tcm_hcd->reset_mutex);
		tcm_hcd->update_watchdog(tcm_hcd, false);
		retval = syna_tcm_run_application_firmware(tcm_hcd);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to switch to application mode\n");
		}
		tcm_hcd->update_watchdog(tcm_hcd, true);
		mutex_unlock(&tcm_hcd->reset_mutex);
		break;
	case HELP_SEND_RESET_NOTIFICATION:
		mutex_lock(&tcm_hcd->reset_mutex);
		retval = tcm_hcd->identify(tcm_hcd, true);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to do identification\n");
			mutex_unlock(&tcm_hcd->reset_mutex);
			break;
		}
		mutex_lock(&mod_pool.mutex);
		if (!list_empty(&mod_pool.list)) {
			list_for_each_entry(mod_handler, &mod_pool.list, link) {
				if (!mod_handler->insert &&
						!mod_handler->detach &&
						(mod_handler->mod_cb->reset))
					mod_handler->mod_cb->reset(tcm_hcd);
			}
		}
		mutex_unlock(&mod_pool.mutex);
		mutex_unlock(&tcm_hcd->reset_mutex);
		break;
	default:
		break;
	}

	atomic_set(&helper->task, HELP_NONE);

	return;
}

static int syna_tcm_resume(struct device *dev)
{
	int retval;
	struct syna_tcm_module_handler *mod_handler;
	struct syna_tcm_hcd *tcm_hcd = dev_get_drvdata(dev);

	if (!tcm_hcd->in_suspend)
		return 0;

	tcm_hcd->enable_irq(tcm_hcd, true, NULL);

	if (tcm_hcd->host_download_mode) {
		retval = syna_tcm_wait_hdl(tcm_hcd);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to wait for completion of host download\n");
			goto exit;
		}
	}

	tcm_hcd->update_watchdog(tcm_hcd, true);

#ifdef RESET_ON_RESUME
	msleep(RESET_ON_RESUME_DELAY_MS);

	goto do_reset;
#endif

	if (tcm_hcd->id_info.mode != MODE_APPLICATION ||
			tcm_hcd->app_status != APP_STATUS_OK) {
		LOGN(tcm_hcd->pdev->dev.parent,
				"Application firmware not running\n");
		goto do_reset;
	}

	retval = tcm_hcd->sleep(tcm_hcd, false);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to exit deep sleep\n");
		goto exit;
	}

	retval = syna_tcm_rezero(tcm_hcd);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to rezero\n");
		goto exit;
	}

	goto mod_resume;

do_reset:
	retval = tcm_hcd->reset(tcm_hcd, false, true);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to do reset\n");
		goto exit;
	}

	if (tcm_hcd->id_info.mode != MODE_APPLICATION ||
			tcm_hcd->app_status != APP_STATUS_OK) {
		LOGN(tcm_hcd->pdev->dev.parent,
				"Application firmware not running\n");
		retval = 0;
		goto exit;
	}

mod_resume:
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
}

static int syna_tcm_suspend(struct device *dev)
{
	int retval;

	struct syna_tcm_module_handler *mod_handler;
	struct syna_tcm_hcd *tcm_hcd = dev_get_drvdata(dev);

	if (tcm_hcd->in_suspend)
		return 0;

	tcm_hcd->update_watchdog(tcm_hcd, false);

	if (tcm_hcd->id_info.mode != MODE_APPLICATION ||
			tcm_hcd->app_status != APP_STATUS_OK) {
		LOGN(tcm_hcd->pdev->dev.parent,
				"Application firmware not running\n");
		retval = 0;
		goto exit;
	}

#ifndef WAKEUP_GESTURE
	retval = tcm_hcd->sleep(tcm_hcd, true);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to enter deep sleep\n");
		goto exit;
	}
#endif

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

	retval = 0;

exit:
#ifndef WAKEUP_GESTURE
	tcm_hcd->enable_irq(tcm_hcd, false, true);
#endif

	tcm_hcd->in_suspend = true;

	return retval;
}

#ifdef CONFIG_FB
static int syna_tcm_fb_notifier_cb(struct notifier_block *nb,
		unsigned long action, void *data)
{
	int retval;
	int *transition;
	struct fb_event *evdata = data;
	struct syna_tcm_hcd *tcm_hcd =
			container_of(nb, struct syna_tcm_hcd, fb_notifier);

	retval = 0;

	if (evdata && evdata->data && tcm_hcd) {
		if (action == FB_EVENT_BLANK) {
			transition = evdata->data;
			if (*transition == FB_BLANK_POWERDOWN) {
				retval = syna_tcm_suspend(&tcm_hcd->pdev->dev);
				tcm_hcd->fb_ready = 0;
			} else if (*transition == FB_BLANK_UNBLANK) {
				retval = syna_tcm_resume(&tcm_hcd->pdev->dev);
				tcm_hcd->fb_ready++;
			}
		}
	}

	return 0;
}
#endif

static int syna_tcm_probe(struct platform_device *pdev)
{
	int retval;
	int idx;
	struct syna_tcm_hcd *tcm_hcd;
	const struct syna_tcm_board_data *bdata;
	const struct syna_tcm_hw_interface *hw_if;

	hw_if = pdev->dev.platform_data;
	if (!hw_if) {
		LOGE(&pdev->dev,
				"Hardware interface not found\n");
		return -ENODEV;
	}

	bdata = hw_if->bdata;
	if (!bdata) {
		LOGE(&pdev->dev,
				"Board data not found\n");
		return -ENODEV;
	}

	tcm_hcd = kzalloc(sizeof(*tcm_hcd), GFP_KERNEL);
	if (!tcm_hcd) {
		LOGE(&pdev->dev,
				"Failed to allocate memory for tcm_hcd\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, tcm_hcd);

	tcm_hcd->pdev = pdev;
	tcm_hcd->hw_if = hw_if;
	tcm_hcd->reset = syna_tcm_reset;
	tcm_hcd->sleep = syna_tcm_sleep;
	tcm_hcd->identify = syna_tcm_identify;
	tcm_hcd->enable_irq = syna_tcm_enable_irq;
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

	if (bdata->irq_gpio >= 0)
		tcm_hcd->irq = gpio_to_irq(bdata->irq_gpio);
	else
		tcm_hcd->irq = bdata->irq_gpio;

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

	retval = syna_tcm_alloc_mem(tcm_hcd,
			&tcm_hcd->in,
			MESSAGE_HEADER_SIZE + 2);
	if (retval < 0) {
		LOGE(&pdev->dev,
				"Failed to allocate memory for tcm_hcd->in.buf\n");
		UNLOCK_BUFFER(tcm_hcd->in);
		goto err_alloc_mem;
	}

	UNLOCK_BUFFER(tcm_hcd->in);

	atomic_set(&tcm_hcd->command_status, CMD_IDLE);

	atomic_set(&tcm_hcd->helper.task, HELP_NONE);

	device_init_wakeup(&pdev->dev, 1);

	init_waitqueue_head(&tcm_hcd->report_wq);

	init_waitqueue_head(&tcm_hcd->hdl_wq);

	if (!mod_pool.initialized) {
		mutex_init(&mod_pool.mutex);
		INIT_LIST_HEAD(&mod_pool.list);
		mod_pool.initialized = true;
	}

	retval = syna_tcm_get_regulator(tcm_hcd, true);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to get regulators\n");
		goto err_get_regulator;
	}

	retval = syna_tcm_enable_regulator(tcm_hcd, true);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to enable regulators\n");
		goto err_enable_regulator;
	}

	retval = syna_tcm_config_gpio(tcm_hcd);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to configure GPIO's\n");
		goto err_config_gpio;
	}

	sysfs_dir = kobject_create_and_add(PLATFORM_DRIVER_NAME,
			&pdev->dev.kobj);
	if (!sysfs_dir) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to create sysfs directory\n");
		retval = -EINVAL;
		goto err_sysfs_create_dir;
	}

	tcm_hcd->sysfs_dir = sysfs_dir;

	for (idx = 0; idx < ARRAY_SIZE(attrs); idx++) {
		retval = sysfs_create_file(tcm_hcd->sysfs_dir,
				&(*attrs[idx]).attr);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to create sysfs file\n");
			goto err_sysfs_create_file;
		}
	}

	tcm_hcd->dynamnic_config_sysfs_dir =
			kobject_create_and_add(DYNAMIC_CONFIG_SYSFS_DIR_NAME,
			tcm_hcd->sysfs_dir);
	if (!tcm_hcd->dynamnic_config_sysfs_dir) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to create dynamic config sysfs directory\n");
		retval = -EINVAL;
		goto err_sysfs_create_dynamic_config_dir;
	}

	for (idx = 0; idx < ARRAY_SIZE(dynamic_config_attrs); idx++) {
		retval = sysfs_create_file(tcm_hcd->dynamnic_config_sysfs_dir,
				&(*dynamic_config_attrs[idx]).attr);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to create dynamic config sysfs file\n");
			goto err_sysfs_create_dynamic_config_file;
		}
	}

#ifdef CONFIG_FB
	tcm_hcd->fb_notifier.notifier_call = syna_tcm_fb_notifier_cb;
	retval = fb_register_client(&tcm_hcd->fb_notifier);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to register FB notifier client\n");
	}
#endif

	tcm_hcd->notifier_thread = kthread_run(syna_tcm_report_notifier,
			tcm_hcd, "syna_tcm_report_notifier");
	if (IS_ERR(tcm_hcd->notifier_thread)) {
		retval = PTR_ERR(tcm_hcd->notifier_thread);
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to create and run tcm_hcd->notifier_thread\n");
		goto err_create_run_kthread;
	}

	tcm_hcd->helper.workqueue =
			create_singlethread_workqueue("syna_tcm_helper");
	INIT_WORK(&tcm_hcd->helper.work, syna_tcm_helper_work);

	tcm_hcd->watchdog.workqueue =
			create_singlethread_workqueue("syna_tcm_watchdog");
	INIT_DELAYED_WORK(&tcm_hcd->watchdog.work, syna_tcm_watchdog_work);

	tcm_hcd->polling_workqueue =
			create_singlethread_workqueue("syna_tcm_polling");
	INIT_DELAYED_WORK(&tcm_hcd->polling_work, syna_tcm_polling_work);

	retval = tcm_hcd->enable_irq(tcm_hcd, true, NULL);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to enable interrupt\n");
		goto err_enable_irq;
	}

	retval = tcm_hcd->reset(tcm_hcd, false, false);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to do reset\n");
		tcm_hcd->init_okay = false;
		tcm_hcd->watchdog.run = false;
		tcm_hcd->update_watchdog(tcm_hcd, false);
		tcm_hcd->enable_irq(tcm_hcd, false, false);
#ifndef KEEP_DRIVER_ON_ERROR
		goto err_reset;
#endif
	} else {
		tcm_hcd->init_okay = true;
		tcm_hcd->update_watchdog(tcm_hcd, true);
	}

	mod_pool.workqueue =
			create_singlethread_workqueue("syna_tcm_module");
	INIT_WORK(&mod_pool.work, syna_tcm_module_work);
	mod_pool.tcm_hcd = tcm_hcd;
	mod_pool.queue_work = true;
	queue_work(mod_pool.workqueue, &mod_pool.work);

	return 0;

#ifndef KEEP_DRIVER_ON_ERROR
err_reset:
#endif
err_enable_irq:
	cancel_delayed_work_sync(&tcm_hcd->polling_work);
	flush_workqueue(tcm_hcd->polling_workqueue);
	destroy_workqueue(tcm_hcd->polling_workqueue);

	cancel_delayed_work_sync(&tcm_hcd->watchdog.work);
	flush_workqueue(tcm_hcd->watchdog.workqueue);
	destroy_workqueue(tcm_hcd->watchdog.workqueue);

	kthread_stop(tcm_hcd->notifier_thread);

err_create_run_kthread:
#ifdef CONFIG_FB
	fb_unregister_client(&tcm_hcd->fb_notifier);
#endif

err_sysfs_create_dynamic_config_file:
	for (idx--; idx >= 0; idx--) {
		sysfs_remove_file(tcm_hcd->dynamnic_config_sysfs_dir,
				&(*dynamic_config_attrs[idx]).attr);
	}

	kobject_put(tcm_hcd->dynamnic_config_sysfs_dir);

	idx = ARRAY_SIZE(attrs);

err_sysfs_create_dynamic_config_dir:
err_sysfs_create_file:
	for (idx--; idx >= 0; idx--)
		sysfs_remove_file(tcm_hcd->sysfs_dir, &(*attrs[idx]).attr);

	kobject_put(tcm_hcd->sysfs_dir);

err_sysfs_create_dir:
	if (bdata->irq_gpio >= 0)
		syna_tcm_set_gpio(tcm_hcd, bdata->irq_gpio, false, 0, 0);

	if (bdata->power_gpio >= 0)
		syna_tcm_set_gpio(tcm_hcd, bdata->power_gpio, false, 0, 0);

	if (bdata->reset_gpio >= 0)
		syna_tcm_set_gpio(tcm_hcd, bdata->reset_gpio, false, 0, 0);

err_config_gpio:
	syna_tcm_enable_regulator(tcm_hcd, false);

err_enable_regulator:
	syna_tcm_get_regulator(tcm_hcd, false);

err_get_regulator:
	device_init_wakeup(&pdev->dev, 0);

err_alloc_mem:
	RELEASE_BUFFER(tcm_hcd->report.buffer);
	RELEASE_BUFFER(tcm_hcd->config);
	RELEASE_BUFFER(tcm_hcd->temp);
	RELEASE_BUFFER(tcm_hcd->resp);
	RELEASE_BUFFER(tcm_hcd->out);
	RELEASE_BUFFER(tcm_hcd->in);

	kfree(tcm_hcd);

	return retval;
}

static int syna_tcm_remove(struct platform_device *pdev)
{
	int idx;
	struct syna_tcm_module_handler *mod_handler;
	struct syna_tcm_hcd *tcm_hcd = platform_get_drvdata(pdev);
	const struct syna_tcm_board_data *bdata = tcm_hcd->hw_if->bdata;

	mutex_lock(&mod_pool.mutex);

	if (!list_empty(&mod_pool.list)) {
		list_for_each_entry(mod_handler, &mod_pool.list, link) {
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

	if (tcm_hcd->irq_enabled && bdata->irq_gpio >= 0) {
		disable_irq(tcm_hcd->irq);
		free_irq(tcm_hcd->irq, tcm_hcd);
	}

	cancel_delayed_work_sync(&tcm_hcd->polling_work);
	flush_workqueue(tcm_hcd->polling_workqueue);
	destroy_workqueue(tcm_hcd->polling_workqueue);

	cancel_delayed_work_sync(&tcm_hcd->watchdog.work);
	flush_workqueue(tcm_hcd->watchdog.workqueue);
	destroy_workqueue(tcm_hcd->watchdog.workqueue);

	kthread_stop(tcm_hcd->notifier_thread);

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
		syna_tcm_set_gpio(tcm_hcd, bdata->irq_gpio, false, 0, 0);

	if (bdata->power_gpio >= 0)
		syna_tcm_set_gpio(tcm_hcd, bdata->power_gpio, false, 0, 0);

	if (bdata->reset_gpio >= 0)
		syna_tcm_set_gpio(tcm_hcd, bdata->reset_gpio, false, 0, 0);

	syna_tcm_enable_regulator(tcm_hcd, false);

	syna_tcm_get_regulator(tcm_hcd, false);

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

#ifdef CONFIG_PM
static const struct dev_pm_ops syna_tcm_dev_pm_ops = {
#ifndef CONFIG_FB
	.suspend = syna_tcm_suspend,
	.resume = syna_tcm_resume,
#endif
};
#endif

static struct platform_driver syna_tcm_driver = {
	.driver = {
		.name = PLATFORM_DRIVER_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_PM
		.pm = &syna_tcm_dev_pm_ops,
#endif
	},
	.probe = syna_tcm_probe,
	.remove = syna_tcm_remove,
};


struct syna_tcm_hcd *tcm_hcd;
struct syna_tcm_board_data *bdata;
static int syna_tcm_fw_update_boot(char *file_name)
{
	int retval = NO_ERR;
	retval = tcm_hcd->update_firmware(tcm_hcd, file_name);
	return NO_ERR;
}


static int syna_tcm_init_chip(void)
{
	int retval = NO_ERR;

//	retval = debug_device_init(tcm_hcd);
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
static int syna_tcm_irq_top_half(struct ts_cmd_node *cmd)
{
	//cmd->command = TS_INT_PROCESS;
	TS_LOG_ERR("top half call\n");
	return NO_ERR;
}

static int syna_tcm_irq_bottom_half(struct ts_cmd_node *in_cmd,
				     struct ts_cmd_node *out_cmd)
{
	int retval = NO_ERR;
	tcm_hcd->in_cmd = in_cmd;
	tcm_hcd->out_cmd = out_cmd;

	out_cmd->command = TS_INPUT_ALGO;
	//out_cmd->command = TS_REPORT_INPUT;
	out_cmd->cmd_param.pub_params.algo_param.algo_order =
	    tcm_hcd->syna_tcm_chip_data->algo_id;
	TS_LOG_DEBUG("order: %d\n",
		     out_cmd->cmd_param.pub_params.algo_param.algo_order);

	retval = tcm_hcd->read_message(tcm_hcd,
			NULL,
			0);

	if (retval < 0) {
		TS_LOG_ERR(
				"Failed to read message\n");	
	}
	return retval;
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
/*
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
*/
static int syna_tcm_get_regulator_new(void)
{
	if (tcm_hcd->syna_tcm_chip_data->ts_platform_data->fpga_flag == 1)
		return 0;

	if (1 == tcm_hcd->syna_tcm_chip_data->vci_regulator_type) {
		tcm_hcd->syna_tcm_tp_vci =
		    regulator_get(&tcm_hcd->pdev->dev, bdata->bus_reg_name);
		if (IS_ERR(tcm_hcd->syna_tcm_tp_vci)) {
			TS_LOG_ERR("regulator tp vci not used\n");
			return -EINVAL;
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
	}

	return 0;
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
static void syna_tcm_gpio_reset(void)
{
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
	udelay(300);
	gpio_direction_output(tcm_hcd->syna_tcm_chip_data->ts_platform_data->reset_gpio, 1);
	mdelay(bdata->reset_delay_ms);
}
static int syna_tcm_comm_check(void) 
{
	//do not use write message here, will timeout, because irq not enable yet
	int retval = NO_ERR;
	unsigned char marker;
	retval = syna_tcm_read(tcm_hcd,
			&marker,
			1);
	TS_LOG_ERR("check result:%d\n", retval);
	
	if (retval < 0 || marker != MESSAGE_MARKER) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to read from device\n");
		return RESULT_ERR;
	}
	return NO_ERR;
}
static int syna_tcm_chip_detect(struct ts_kit_platform_data* data)
{
	int retval = NO_ERR;
	int idx = 0;
	//u16 tmp_spi_mode = SPI_MODE_0;
	
	TS_LOG_ERR(" syna_tcm_chip_detect called !\n");
	tcm_hcd->syna_tcm_chip_data->ts_platform_data = data;
	tcm_hcd->pdev = data->ts_dev;
	//tcm_hcd->pdev->dev.parent = data->client;
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


	//tcm_hcd->enable_irq = syna_tcm_enable_irq;

	tcm_hcd->rd_chunk_size = RD_CHUNK_SIZE;
	tcm_hcd->wr_chunk_size = WR_CHUNK_SIZE;
	TS_LOG_ERR(" syna_tcm_chip_detect called 1!\n");
#ifdef PREDICTIVE_READING
	tcm_hcd->read_length = MIN_READ_LENGTH;
#else
	tcm_hcd->read_length = MESSAGE_HEADER_SIZE;
#endif

	tcm_hcd->watchdog.run = RUN_WATCHDOG;
	tcm_hcd->update_watchdog = syna_tcm_update_watchdog;


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

	retval = syna_tcm_alloc_mem(tcm_hcd,
			&tcm_hcd->in,
			MESSAGE_HEADER_SIZE + 2);
	if (retval < 0) {
		TS_LOG_ERR(
				"Failed to allocate memory for tcm_hcd->in.buf\n");
		UNLOCK_BUFFER(tcm_hcd->in);
		//goto err_alloc_mem;
	}

	UNLOCK_BUFFER(tcm_hcd->in);

	atomic_set(&tcm_hcd->command_status, CMD_IDLE);
	atomic_set(&tcm_hcd->helper.task, HELP_NONE);
	//device_init_wakeup(&data->ts_dev->dev, 1);
	//init_waitqueue_head(&tcm_hcd->hdl_wq);

	wake_lock_init(&tcm_hcd->wakelock, WAKE_LOCK_SUSPEND,
			PLATFORM_DRIVER_NAME);

	init_waitqueue_head(&tcm_hcd->report_wq);

	init_waitqueue_head(&tcm_hcd->hdl_wq);


	if (!mod_pool.initialized) {
		mutex_init(&mod_pool.mutex);
		INIT_LIST_HEAD(&mod_pool.list);
		mod_pool.initialized = true;
	}

	syna_tcm_parse_dts(tcm_hcd->pdev->dev.of_node, tcm_hcd->syna_tcm_chip_data);

	retval = syna_tcm_get_regulator_new();
	if (retval < 0) {
			TS_LOG_ERR("syna_tcm_get_regulator error %d \n",retval);
		//goto err_alloc_mem;
	}

	retval = syna_tcm_gpio_request();
	if (retval < 0) {
            TS_LOG_ERR("syna_tcm_gpio_request error %d \n",retval);
	      //goto gpio_err;
	}
/*
	retval = syna_tcm_pinctrl_get_init();
	if (retval < 0) {
            TS_LOG_ERR("syna_tcm_pinctrl_get_init error %d \n",retval);
	     //goto pinctrl_get_err;
	}
*/
	/*power up the chip */
	//syna_tcm_power_on();

	/*reset the chip */
	syna_tcm_gpio_reset();
/*
	tmp_spi_mode = data->spi->mode;
	data->spi->mode = SPI_MODE_3;
	retval = spi_setup(data->spi);
	if (retval) {
		TS_LOG_ERR("%s spi_setup failed.\n", __func__);
		goto pinctrl_get_err;
	}
*/
	msleep(300);
	retval = syna_tcm_comm_check();

	if (retval < 0) {
		TS_LOG_ERR(
				"Failed to syna_tcm_comm_check\n");
		//goto err_comm_check;
	}

	tcm_hcd->notifier_thread = kthread_run(syna_tcm_report_notifier,
			tcm_hcd, "syna_tcm_report_notifier");
	if (IS_ERR(tcm_hcd->notifier_thread)) {
		retval = PTR_ERR(tcm_hcd->notifier_thread);
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to create and run tcm_hcd->notifier_thread\n");
		//goto err_create_run_kthread;
	}

	tcm_hcd->helper.workqueue =
			create_singlethread_workqueue("syna_tcm_helper");
	INIT_WORK(&tcm_hcd->helper.work, syna_tcm_helper_work);

	tcm_hcd->watchdog.workqueue =
			create_singlethread_workqueue("syna_tcm_watchdog");
	INIT_DELAYED_WORK(&tcm_hcd->watchdog.work, syna_tcm_watchdog_work);

	tcm_hcd->polling_workqueue =
			create_singlethread_workqueue("syna_tcm_polling");
	INIT_DELAYED_WORK(&tcm_hcd->polling_work, syna_tcm_polling_work);
/*	retval = tcm_hcd->reset(tcm_hcd, false);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to do reset\n");
		tcm_hcd->init_okay = false;
		tcm_hcd->watchdog.run = false;
		tcm_hcd->update_watchdog(tcm_hcd, false);
		tcm_hcd->enable_irq(tcm_hcd, false);
#ifndef KEEP_DRIVER_ON_ERROR
		goto err_reset;
#endif
	} else {
		tcm_hcd->init_okay = true;
		tcm_hcd->update_watchdog(tcm_hcd, true);
	}
*/
//to create sys file 
	platform_set_drvdata(tcm_hcd->pdev, tcm_hcd);

	sysfs_dir = kobject_create_and_add(PLATFORM_DRIVER_NAME,
			&tcm_hcd->pdev->dev.kobj);
	if (!sysfs_dir) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to create sysfs directory\n");
		//goto err_sysfs_create_dir;
	}

	tcm_hcd->sysfs_dir = sysfs_dir;

	for (idx = 0; idx < ARRAY_SIZE(attrs); idx++) {
		retval = sysfs_create_file(tcm_hcd->sysfs_dir,
				&(*attrs[idx]).attr);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to create sysfs file\n");
			//goto err_sysfs_create_file;
		}
	}

	tcm_hcd->dynamnic_config_sysfs_dir =
			kobject_create_and_add(DYNAMIC_CONFIG_SYSFS_DIR_NAME,
			tcm_hcd->sysfs_dir);
	if (!tcm_hcd->dynamnic_config_sysfs_dir) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to create dynamic config sysfs directory\n");
		//goto err_sysfs_create_dynamic_config_dir;
	}

	for (idx = 0; idx < ARRAY_SIZE(dynamic_config_attrs); idx++) {
		retval = sysfs_create_file(tcm_hcd->dynamnic_config_sysfs_dir,
				&(*dynamic_config_attrs[idx]).attr);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to create dynamic config sysfs file\n");
			//goto err_sysfs_create_dynamic_config_file;
		}
	}

//create sys file end


	mod_pool.workqueue =
			create_singlethread_workqueue("syna_tcm_module");
	INIT_WORK(&mod_pool.work, syna_tcm_module_work);
	mod_pool.tcm_hcd = tcm_hcd;
	mod_pool.queue_work = true;
	queue_work(mod_pool.workqueue, &mod_pool.work);
	return 0;
/*
err_comm_check:
	syna_tcm_power_off();
	
pinctrl_get_err:
if (0) {
	data->spi->mode = tmp_spi_mode;
	retval = spi_setup(data->spi);
	if (retval) {
		TS_LOG_ERR("%s spi_setup failed.\n", __func__);
	}
}
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
	TS_LOG_ERR("detect synaptics error\n");*/
	return retval;
}
static int syna_tcm_chip_get_info(struct ts_chip_info_param *info)
{
	unsigned char buf[CHIP_INFO_LENGTH * 2] = { 0 };

#define SYNA_TCM_CHIP_INFO "syna_tcm_"
	memcpy(&info->fw_vendor, tcm_hcd->app_info.customer_config_id, strlen(tcm_hcd->app_info.customer_config_id));
	memcpy(&buf, SYNA_TCM_CHIP_INFO, strlen(SYNA_TCM_CHIP_INFO));
	strncat(buf, tcm_hcd->app_info.customer_config_id, strlen(tcm_hcd->app_info.customer_config_id));  // no project id, 
	memcpy(&info->ic_vendor, buf, strlen(buf));

	return 0;
}

static int syna_tcm_set_info_flag(struct ts_kit_platform_data *info)
{
	tcm_hcd->syna_tcm_chip_data->ts_platform_data->get_info_flag = info->get_info_flag;
	return NO_ERR;
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

static int syna_tcm_suspend_ts_kit(void)
{
	int retval = NO_ERR;

	TS_LOG_INFO("suspend +\n");
	
	if (tcm_hcd->in_suspend)
			return 0;
	
	gpio_direction_output(tcm_hcd->syna_tcm_chip_data->ts_platform_data->reset_gpio, 0);

	TS_LOG_INFO("suspend -\n");

	tcm_hcd->in_suspend = true;

	return retval;
}

 //   do not add time-costly function here.

static int syna_tcm_resume_ts_kit(void)
{
	int retval = NO_ERR;

	if (!tcm_hcd->in_suspend)
			return 0;	
	TS_LOG_INFO("resume +\n");

	gpio_direction_output(tcm_hcd->syna_tcm_chip_data->ts_platform_data->reset_gpio, 1);
	msleep(bdata->reset_delay_ms);
	
	//pre_finger_status = 0;
	tcm_hcd->in_suspend = false;
	TS_LOG_INFO("resume -\n");
	return retval;
}

static int syna_wakeup_gesture_enable_switch(struct ts_wakeup_gesture_enable_info* info)
{
	int retval = NO_ERR;
	
	retval = tcm_hcd->set_dynamic_config(tcm_hcd,
			DC_IN_WAKEUP_GESTURE_MODE,
			info->switch_value ? 1 : 0);
	if (retval < 0) {
				TS_LOG_ERR("Failed to set wakeup gesture mode\n");
		return retval;
	}
	return retval;
}

struct ts_device_ops ts_kit_syna_tcm_ops = {
	.chip_detect = syna_tcm_chip_detect,
	.chip_init = syna_tcm_init_chip,
	.chip_parse_config = syna_tcm_parse_dts,
	.chip_input_config = syna_tcm_input_config,
	.chip_irq_top_half = syna_tcm_irq_top_half,
	.chip_irq_bottom_half = syna_tcm_irq_bottom_half,
	.chip_fw_update_boot = syna_tcm_fw_update_boot,  // update firmware img
	//.chip_fw_update_sd = syna_tcm_fw_update_sd,   // host download by hand
	.chip_get_info = syna_tcm_chip_get_info,
	.chip_set_info_flag = syna_tcm_set_info_flag,
	.chip_suspend = syna_tcm_suspend_ts_kit,
	.chip_resume = syna_tcm_resume_ts_kit,
	.chip_before_suspend = syna_tcm_before_suspend,
	.chip_after_resume = syna_tcm_after_resume,
	.chip_get_rawdata = syna_tcm_get_raw_data,
	//.chip_get_calibration_data = syna_tcm_get_cal_data,
	.chip_wakeup_gesture_enable_switch = syna_wakeup_gesture_enable_switch,
};
static int __init syna_tcm_module_init(void)
{
	if (0) {
		int retval = 0;
		//retval = syna_tcm_bus_init();
		if (retval < 0)
			return retval;

		return platform_driver_register(&syna_tcm_driver);
	} else {
		//struct syna_tcm_hcd *tcm_hcd;
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

		syna_tcm_i2c_init(tcm_hcd, bdata);
		//hw_if.bdata = bdata;
		//tcm_hcd->hw_if->bdata = bdata;

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
}
/*
static int __init syna_tcm_module_init(void)
{
	int retval;

	retval = syna_tcm_bus_init();
	if (retval < 0)
		return retval;

	return platform_driver_register(&syna_tcm_driver);
}
*/
static void __exit syna_tcm_module_exit(void)
{
	platform_driver_unregister(&syna_tcm_driver);

	syna_tcm_bus_exit();

	return;
}

module_init(syna_tcm_module_init);
module_exit(syna_tcm_module_exit);

MODULE_AUTHOR("Synaptics, Inc.");
MODULE_DESCRIPTION("Synaptics TCM Touch Driver");
MODULE_LICENSE("GPL v2");
