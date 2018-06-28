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

#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include "synaptics_tcm_core.h"

#define TYPE_B_PROTOCOL

#define USE_DEFAULT_TOUCH_REPORT_CONFIG

#define TOUCH_REPORT_CONFIG_SIZE 128

#define APP_STATUS_POLL_TIMEOUT_MS 1000

#define APP_STATUS_POLL_MS 100

enum touch_status {
	LIFT = 0,
	FINGER = 1,
	GLOVED_FINGER = 2,
	NOP = -1,
};

enum touch_report_code {
	TOUCH_END = 0,
	TOUCH_FOREACH_ACTIVE_OBJECT,
	TOUCH_FOREACH_OBJECT,
	TOUCH_FOREACH_END,
	TOUCH_PAD_TO_NEXT_BYTE,
	TOUCH_TIMESTAMP,
	TOUCH_OBJECT_N_INDEX,
	TOUCH_OBJECT_N_CLASSIFICATION,
	TOUCH_OBJECT_N_X_POSITION,
	TOUCH_OBJECT_N_Y_POSITION,
	TOUCH_OBJECT_N_Z,
	TOUCH_OBJECT_N_X_WIDTH,
	TOUCH_OBJECT_N_Y_WIDTH,
	TOUCH_OBJECT_N_TX_POSITION_TIXELS,
	TOUCH_OBJECT_N_RX_POSITION_TIXELS,
	TOUCH_0D_BUTTONS_STATE,
	TOUCH_GESTURE_DOUBLE_TAP,
	TOUCH_FRAME_RATE,
	TOUCH_POWER_IM,
	TOUCH_CID_IM,
	TOUCH_RAIL_IM,
	TOUCH_CID_VARIANCE_IM,
	TOUCH_NSM_FREQUENCY,
	TOUCH_NSM_STATE,
	TOUCH_NUM_OF_ACTIVE_OBJECTS,
	TOUCH_NUM_OF_CPU_CYCLES_USED_SINCE_LAST_FRAME,
	TOUCH_TUNING_GAUSSIAN_WIDTHS = 0x80,
	TOUCH_TUNING_SMALL_OBJECT_PARAMS,
	TOUCH_TUNING_0D_BUTTONS_VARIANCE,
};

struct object_data {
	unsigned char status;
	unsigned int x_pos;
	unsigned int y_pos;
	unsigned int x_width;
	unsigned int y_width;
	unsigned int z;
	unsigned int tx_pos;
	unsigned int rx_pos;
};

struct input_params {
	unsigned int max_x;
	unsigned int max_y;
	unsigned int max_objects;
};

struct touch_data {
	struct object_data *object_data;
	unsigned int timestamp;
	unsigned int buttons_state;
	unsigned int gesture_double_tap;
	unsigned int frame_rate;
	unsigned int power_im;
	unsigned int cid_im;
	unsigned int rail_im;
	unsigned int cid_variance_im;
	unsigned int nsm_frequency;
	unsigned int nsm_state;
	unsigned int num_of_active_objects;
	unsigned int num_of_cpu_cycles;
};

struct touch_hcd {
	bool irq_wake;
	bool report_touch;
	unsigned char *prev_status;
	unsigned int max_x;
	unsigned int max_y;
	unsigned int max_objects;
	struct mutex report_mutex;
	struct input_dev *input_dev;
	struct touch_data touch_data;
	struct input_params input_params;
	struct syna_tcm_buffer out;
	struct syna_tcm_buffer resp;
	struct syna_tcm_hcd *tcm_hcd;
};

DECLARE_COMPLETION(touch_remove_complete);

static struct touch_hcd *touch_hcd;

static int touch_get_report_data(unsigned int offset,
		unsigned int bits, unsigned int *data)
{
	unsigned char mask = 0;
	unsigned char byte_data = 0;
	unsigned int output_data = 0;
	unsigned int bit_offset = 0;
	unsigned int byte_offset = 0;
	unsigned int data_bits = 0;
	unsigned int available_bits = 0;
	unsigned int remaining_bits = 0;
	unsigned char *touch_report = 0;
	struct syna_tcm_hcd *tcm_hcd = touch_hcd->tcm_hcd;

	if (bits == 0 || bits > 32) {
		TS_LOG_ERR(
				"Invalid number of bits\n");
		return -EINVAL;
	}

	if (offset + bits > tcm_hcd->report.buffer.data_length * 8) {
		*data = 0;
		return 0;
	}

	touch_report = tcm_hcd->report.buffer.buf;

	output_data = 0;
	remaining_bits = bits;

	bit_offset = offset % 8;
	byte_offset = offset / 8;

	while (remaining_bits) {
		byte_data = touch_report[byte_offset];
		byte_data >>= bit_offset;

		available_bits = 8 - bit_offset;
		data_bits = MIN(available_bits, remaining_bits);
		mask = 0xff >> (8 - data_bits);

		byte_data &= mask;

		output_data |= byte_data << (bits - remaining_bits);

		bit_offset = 0;
		byte_offset += 1;
		remaining_bits -= data_bits;
	}

	*data = output_data;

	return 0;
}

/**
 * touch_parse_report() - Parse touch report
 *
 * Traverse through the touch report configuration and parse the touch report
 * generated by the device accordingly to retrieve the touch data.
 */
static int touch_parse_report(void)
{
	int retval = NO_ERR;
	bool active_only = false;
	bool num_of_active_objects = false;
	unsigned char code = 0;
	unsigned int size = 0;
	unsigned int idx = 0;
	unsigned int obj = 0;
	unsigned int next = 0;
	unsigned int data = 0;
	unsigned int bits = 0;
	unsigned int offset = 0;
	unsigned int objects = 0;
	unsigned int active_objects = 0;
	unsigned int report_size = 0;
	unsigned int config_size = 0;
	unsigned char *config_data = NULL;
	struct touch_data *touch_data = NULL;
	struct object_data *object_data = NULL;
	struct syna_tcm_hcd *tcm_hcd = touch_hcd->tcm_hcd;
	static unsigned int end_of_foreach = 0;

	touch_data = &touch_hcd->touch_data;
	object_data = touch_hcd->touch_data.object_data;

	config_data = tcm_hcd->config.buf;
	config_size = tcm_hcd->config.data_length;

	report_size = tcm_hcd->report.buffer.data_length;

	size = sizeof(*object_data) * touch_hcd->max_objects;
	memset(touch_hcd->touch_data.object_data, 0x00, size);
	
	num_of_active_objects = false;

	idx = 0;
	offset = 0;
	objects = 0;
	while (idx < config_size) {
		code = config_data[idx++];
		switch (code) {
		case TOUCH_END:
			goto exit;
		case TOUCH_FOREACH_ACTIVE_OBJECT:
			obj = 0;
			next = idx;
			active_only = true;
			break;
		case TOUCH_FOREACH_OBJECT:
			obj = 0;
			next = idx;
			active_only = false;
			break;
		case TOUCH_FOREACH_END:
			end_of_foreach = idx;
			if (active_only) {
				if (num_of_active_objects) {
					objects++;
					if (objects < active_objects)
						idx = next;
				} else if (offset < report_size * 8) {
					idx = next;
				}
			} else {
				obj++;
				if (obj < touch_hcd->max_objects)
					idx = next;
			}
			break;
		case TOUCH_PAD_TO_NEXT_BYTE:
			offset = ceil_div(offset, 8) * 8;
			break;
		case TOUCH_TIMESTAMP:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				TS_LOG_ERR(
						"Failed to get timestamp\n");
				return retval;
			}
			touch_data->timestamp = data;
			offset += bits;
			break;
		case TOUCH_OBJECT_N_INDEX:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &obj);
			if (retval < 0) {
				TS_LOG_ERR(
						"Failed to get object index\n");
				return retval;
			}
			offset += bits;
			break;
		case TOUCH_OBJECT_N_CLASSIFICATION:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				TS_LOG_ERR(
						"Failed to get object classification\n");
				return retval;
			}
			object_data[obj].status = data;
			offset += bits;
			break;
		case TOUCH_OBJECT_N_X_POSITION:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				TS_LOG_ERR(
						"Failed to get object x position\n");
				return retval;
			}
			object_data[obj].x_pos = data;
			offset += bits;
			break;
		case TOUCH_OBJECT_N_Y_POSITION:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				TS_LOG_ERR(
						"Failed to get object y position\n");
				return retval;
			}
			object_data[obj].y_pos = data;
			offset += bits;
			break;
		case TOUCH_OBJECT_N_Z:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				TS_LOG_ERR(
						"Failed to get object z\n");
				return retval;
			}
			object_data[obj].z = data;
			offset += bits;
			break;
		case TOUCH_OBJECT_N_X_WIDTH:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				TS_LOG_ERR(
						"Failed to get object x width\n");
				return retval;
			}
			object_data[obj].x_width = data;
			offset += bits;
			break;
		case TOUCH_OBJECT_N_Y_WIDTH:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				TS_LOG_ERR(
						"Failed to get object y width\n");
				return retval;
			}
			object_data[obj].y_width = data;
			offset += bits;
			break;
		case TOUCH_OBJECT_N_TX_POSITION_TIXELS:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				TS_LOG_ERR(
						"Failed to get object tx position\n");
				return retval;
			}
			object_data[obj].tx_pos = data;
			offset += bits;
			break;
		case TOUCH_OBJECT_N_RX_POSITION_TIXELS:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				TS_LOG_ERR(
						"Failed to get object rx position\n");
				return retval;
			}
			object_data[obj].rx_pos = data;
			offset += bits;
			break;
		case TOUCH_0D_BUTTONS_STATE:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				TS_LOG_ERR(
						"Failed to get 0D buttons state\n");
				return retval;
			}
			touch_data->buttons_state = data;
			offset += bits;
			break;
		case TOUCH_GESTURE_DOUBLE_TAP:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				TS_LOG_ERR(
						"Failed to get gesture double tap\n");
				return retval;
			}
			touch_data->gesture_double_tap = data;
			offset += bits;
			break;
		case TOUCH_FRAME_RATE:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				TS_LOG_ERR(
						"Failed to get frame rate\n");
				return retval;
			}
			touch_data->frame_rate = data;
			offset += bits;
			break;
		case TOUCH_POWER_IM:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				TS_LOG_ERR(
						"Failed to get power IM\n");
				return retval;
			}
			touch_data->power_im = data;
			offset += bits;
			break;
		case TOUCH_CID_IM:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				TS_LOG_ERR(
						"Failed to get CID IM\n");
				return retval;
			}
			touch_data->cid_im = data;
			offset += bits;
			break;
		case TOUCH_RAIL_IM:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				TS_LOG_ERR(
						"Failed to get rail IM\n");
				return retval;
			}
			touch_data->rail_im = data;
			offset += bits;
			break;
		case TOUCH_CID_VARIANCE_IM:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				TS_LOG_ERR(
						"Failed to get CID variance IM\n");
				return retval;
			}
			touch_data->cid_variance_im = data;
			offset += bits;
			break;
		case TOUCH_NSM_FREQUENCY:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				TS_LOG_ERR(
						"Failed to get NSM frequency\n");
				return retval;
			}
			touch_data->nsm_frequency = data;
			offset += bits;
			break;
		case TOUCH_NSM_STATE:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				TS_LOG_ERR(
						"Failed to get NSM state\n");
				return retval;
			}
			touch_data->nsm_state = data;
			offset += bits;
			break;
		case TOUCH_NUM_OF_ACTIVE_OBJECTS:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				TS_LOG_ERR(
						"Failed to get number of active objects\n");
				return retval;
			}
			active_objects = data;
			num_of_active_objects = true;
			touch_data->num_of_active_objects = data;
			offset += bits;
			if (touch_data->num_of_active_objects == 0)
				idx = end_of_foreach;
			break;
		case TOUCH_NUM_OF_CPU_CYCLES_USED_SINCE_LAST_FRAME:
			bits = config_data[idx++];
			retval = touch_get_report_data(offset, bits, &data);
			if (retval < 0) {
				TS_LOG_ERR(
						"Failed to get number of CPU cycles used since last frame\n");
				return retval;
			}
			touch_data->num_of_cpu_cycles = data;
			offset += bits;
			break;
		case TOUCH_TUNING_GAUSSIAN_WIDTHS:
			bits = config_data[idx++];
			offset += bits;
			break;
		case TOUCH_TUNING_SMALL_OBJECT_PARAMS:
			bits = config_data[idx++];
			offset += bits;
			break;
		case TOUCH_TUNING_0D_BUTTONS_VARIANCE:
			bits = config_data[idx++];
			offset += bits;
			break;
		}
	}

exit:
	return 0;
}

/**
 * touch_report() - Report touch events
 *
 * Retrieve data from the touch report generated by the device and report touch
 * events to the input subsystem.
 */
void touch_report(struct ts_fingers *info)
{
	int retval = NO_ERR;
	unsigned int idx = 0;
	unsigned int x = 0;
	unsigned int y = 0;
	unsigned int z = 0;
	unsigned int wx = 0;
	unsigned int wy = 0;
	unsigned int temp = 0;
	unsigned int status = 0;
	unsigned int touch_count = 0;
	struct touch_data *touch_data = NULL;
	struct object_data *object_data = NULL;
	struct syna_tcm_hcd *tcm_hcd = touch_hcd->tcm_hcd;
	const struct syna_tcm_board_data *bdata = tcm_hcd->bdata;

	TS_LOG_DEBUG(
			"touch_report called\n");

	if (tcm_hcd->in_suspend)
		goto exit;

	TS_LOG_DEBUG("not in suspend\n");

	if (!tcm_hcd->report_touch)
		goto exit;
	TS_LOG_DEBUG("report touch is true\n");
	
	retval = touch_parse_report();
	if (retval < 0) {
		TS_LOG_ERR(
				"Failed to parse touch report\n");
		goto exit;
	}

	touch_data = &touch_hcd->touch_data;
	object_data = touch_hcd->touch_data.object_data;

	touch_count = 0;
	for (idx = 0; idx < touch_hcd->max_objects; idx++) {
		if (touch_hcd->prev_status[idx] == LIFT &&
				object_data[idx].status == LIFT)
			status = NOP;
		else
			status = object_data[idx].status;
		TS_LOG_DEBUG("status = %d\n", status);
		switch (status) {
		case LIFT:
			break;
		case FINGER:
		case GLOVED_FINGER:
			x = object_data[idx].x_pos;
			y = object_data[idx].y_pos;
			wx = object_data[idx].x_width;
			wy = object_data[idx].y_width;
			z = object_data[idx].z;
			if (bdata->swap_axes) {
				temp = x;
				x = y;
				y = temp;
			}

			y = 2244-y;
			if (bdata->x_flip)
				x = touch_hcd->input_params.max_x - x;
			if (bdata->y_flip)
				y = touch_hcd->input_params.max_y - y;
			if (status == 1)
				info->fingers[idx].status = status << 6;
			info->fingers[idx].x = x;
			info->fingers[idx].y = y;
			info->fingers[idx].major = wx;
			info->fingers[idx].minor = wy;
			info->fingers[idx].sg = 0;
			info->fingers[idx].pressure = z;
			TS_LOG_INFO(
					"Finger %d: x = %d, y = %d\n",
					idx, x, y);
			touch_count++;
			break;	
		default:
			break;
		}
		touch_hcd->prev_status[idx] = object_data[idx].status;
	}

	info->cur_finger_number = touch_count;

exit:

	return;
}

/**
 * touch_get_input_params() - Get input parameters
 *
 * Retrieve the input parameters to register with the input subsystem for
 * the input device from the application information packet. In addition,
 * the touch report configuration is retrieved and stored.
 */

static int touch_get_input_params(void)
{
	int retval = NO_ERR;
	unsigned int temp = 0;
	struct syna_tcm_app_info *app_info = NULL;
	struct syna_tcm_hcd *tcm_hcd = touch_hcd->tcm_hcd;
	const struct syna_tcm_board_data *bdata = tcm_hcd->bdata;
	unsigned char *report_config;

	app_info = &tcm_hcd->app_info;
	touch_hcd->max_x = le2_to_uint(app_info->max_x);
	touch_hcd->max_y = le2_to_uint(app_info->max_y);
	touch_hcd->max_objects = le2_to_uint(app_info->max_objects);

	if (bdata->swap_axes) {
		temp = touch_hcd->max_x;
		touch_hcd->max_x = touch_hcd->max_y;
		touch_hcd->max_y = temp;
	}

	report_config = kzalloc(TOUCH_REPORT_CONFIG_SIZE + 4, GFP_KERNEL);

	LOCK_BUFFER(tcm_hcd->config);

	retval = syna_tcm_alloc_mem(tcm_hcd,
			&tcm_hcd->config,
			TOUCH_REPORT_CONFIG_SIZE);
	if (retval < 0) {
		TS_LOG_ERR("failed to allocate memory for tcm_hcd->config\n");
		UNLOCK_BUFFER(tcm_hcd->config);
		return retval;
	}
	
	retval = syna_tcm_write_hdl_message(tcm_hcd,
			CMD_GET_TOUCH_REPORT_CONFIG,
			NULL,
			0,
			&tcm_hcd->config.buf,
			&tcm_hcd->config.buf_size,
			&tcm_hcd->config.data_length,
			NULL,
			0);
	if (retval < 0) {
		TS_LOG_ERR(
				"Failed to write command %s\n",
				STR(CMD_GET_TOUCH_REPORT_CONFIG));
		UNLOCK_BUFFER(tcm_hcd->config);
		kfree(report_config);
		return retval;
	}
	udelay(1000);
	retval = syna_tcm_read(tcm_hcd,
			report_config,
			TOUCH_REPORT_CONFIG_SIZE + 4);
	if (retval < 0) {
		TS_LOG_ERR("Failed to read id report\n");
		return retval;
	}
	TS_LOG_INFO("report_config = 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n", 
			report_config[0], report_config[1], report_config[2],
			report_config[3], report_config[4], report_config[5]);

	if (report_config[0] != MESSAGE_MARKER || report_config[1] != STATUS_OK)
		return -EINVAL;

	tcm_hcd->config.data_length = report_config[2] | (report_config[3] << 8);
	memcpy((unsigned char *)tcm_hcd->config.buf, &report_config[4], 
			TOUCH_REPORT_CONFIG_SIZE);

	UNLOCK_BUFFER(tcm_hcd->config);
	kfree(report_config);
	return 0;
}

/**
 * touch_check_input_params() - Check input parameters
 *
 * Check if any of the input parameters registered with the input subsystem for
 * the input device has changed.
 */

static int touch_check_input_params(void)
{
	unsigned int size = 0;

	TS_LOG_INFO("touch_hcd->input_params.max_objects = %d\n", touch_hcd->input_params.max_objects);
	TS_LOG_INFO("touch_hcd->max_objects = %d\n", touch_hcd->max_objects);	
//	touch_hcd->max_objects = touch_hcd->tcm_hcd->bdata->max_finger_objects;
	if (touch_hcd->input_params.max_objects != touch_hcd->max_objects) {
		kfree(touch_hcd->touch_data.object_data);
		size = sizeof(*touch_hcd->touch_data.object_data);
		size *= touch_hcd->max_objects;
		touch_hcd->touch_data.object_data = kzalloc(size, GFP_KERNEL);
		if (!touch_hcd->touch_data.object_data) {
			TS_LOG_ERR(
					"Failed to allocate memory for touch_hcd->touch_data.object_data\n");
			return -ENOMEM;
		}
		return 1;
	}

	if (touch_hcd->input_params.max_x != touch_hcd->max_x)
		return 1;

	if (touch_hcd->input_params.max_y != touch_hcd->max_y)
		return 1;

	return 0;
}

static int syna_tcm_get_app_info(struct syna_tcm_hcd *tcm_hcd)
{
	int retval = NO_ERR;
	unsigned char *resp_buf = NULL;
	unsigned int resp_buf_size = 0;
	unsigned int resp_length = 0;
//	unsigned int timeout;
	unsigned char app_info[200] = {0};
	unsigned int retry = 0;

//	timeout = APP_STATUS_POLL_TIMEOUT_MS;

	int rows, cols;
	resp_buf = NULL;
	resp_buf_size = 0;


get_app_info:
	msleep(APP_STATUS_POLL_MS);
	syna_tcm_write_hdl_message(tcm_hcd,
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

	msleep(50);
	retval = syna_tcm_read(tcm_hcd,
			app_info,
			sizeof(app_info));
	if (retval < 0) {
		TS_LOG_ERR("Failed to read app_info\n");
		return retval;
	}
	TS_LOG_ERR("app_info = 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n", 
			app_info[0], app_info[1], app_info[2],
			app_info[3], app_info[4], app_info[5]);

	if (app_info[0] != MESSAGE_MARKER || app_info[1] != STATUS_OK) {
		msleep(500);
		retry ++;
		if (retry == 3) {
			retval = -1;
			goto exit;
		}
		goto get_app_info;
	}

	memcpy((unsigned char *)&tcm_hcd->app_info, &app_info[4], 
			sizeof(tcm_hcd->app_info));

	tcm_hcd->app_status = le2_to_uint(tcm_hcd->app_info.status);


	rows = le2_to_uint(tcm_hcd->app_info.num_of_image_rows);
	cols = le2_to_uint(tcm_hcd->app_info.num_of_image_cols);

	
	TS_LOG_ERR("testing init: rows:%d, cols:%d\n", rows, cols);
	TS_LOG_INFO("%s\n", tcm_hcd->app_info.customer_config_id);

/*
	if (tcm_hcd->app_status == APP_STATUS_BOOTING ||
			tcm_hcd->app_status == APP_STATUS_UPDATING) {
		if (timeout > 0) {
			msleep(APP_STATUS_POLL_MS);
			timeout -= APP_STATUS_POLL_MS;
			goto get_app_info;
		}
	}
*/
	retval = 0;

exit:
	kfree(resp_buf);

	return retval;
}

static int syna_tcm_get_id_info(struct syna_tcm_hcd *tcm_hcd)
{
	int retval = NO_ERR;
	unsigned char *resp_buf = NULL;
	unsigned int resp_buf_size = 0;
	unsigned int resp_length = 0;
	unsigned char id_report[30] = {0};

	resp_buf = NULL;
	resp_buf_size = 0;

//	mutex_lock(&tcm_hcd->identify_mutex);

	retval = syna_tcm_write_hdl_message(tcm_hcd,
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

	msleep(50);
	retval = syna_tcm_read(tcm_hcd,
			id_report,
			sizeof(id_report));
	if (retval < 0) {
		TS_LOG_ERR("Failed to read id report\n");
		return retval;
	}		

	if (id_report[1] != REPORT_IDENTIFY)
		TS_LOG_ERR("yuehao after identify cmd is CMD_IDENTIFY!!!!!!!!!!!\n");
	else {
		TS_LOG_ERR("yuehao identify cmd seems ok:%x\n", id_report[1]);
	}
	if (id_report[0] != MESSAGE_MARKER || (id_report[1] != STATUS_OK && id_report[1] != REPORT_IDENTIFY)) {
		TS_LOG_ERR("id_report:%x  %x %x %x %x\n", id_report[0],  id_report[1], id_report[2], id_report[3], id_report[4]);
		return -EINVAL;
	}
	
	memcpy((unsigned char *)&tcm_hcd->id_info, &id_report[4], 
			sizeof(tcm_hcd->id_info));

	TS_LOG_ERR("yuehao info mode:%x\n", tcm_hcd->id_info.mode);

	if (tcm_hcd->id_info.mode == 0x0b) {
		retval = syna_tcm_write_hdl_message(tcm_hcd,
				CMD_RUN_APPLICATION_FIRMWARE,
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
					STR(CMD_RUN_APPLICATION_FIRMWAR));
			goto exit;
		}

		//udelay(1000);
		msleep(50);
		retval = syna_tcm_read(tcm_hcd,
				id_report,
				sizeof(id_report));
		if (retval < 0) {
			TS_LOG_ERR("Failed to read id 2 report\n");
			return retval;
		}
		if (id_report[0] != MESSAGE_MARKER || (id_report[1] != STATUS_OK && id_report[1] != REPORT_IDENTIFY)) {
			TS_LOG_ERR("id_report:%x  %x %x %x %x\n", id_report[0],  id_report[1], id_report[2], id_report[3], id_report[4]);
		return -EINVAL;
		}
	}

	memcpy((unsigned char *)&tcm_hcd->id_info, &id_report[4], 
			sizeof(tcm_hcd->id_info));
	tcm_hcd->packrat_number = le4_to_uint(tcm_hcd->id_info.build_id);

	tcm_hcd->wr_chunk_size = le2_to_uint(tcm_hcd->id_info.max_write_size);
	TS_LOG_ERR("mode = 0x%02x\n", tcm_hcd->id_info.mode);

	switch (tcm_hcd->id_info.mode) {
	case MODE_APPLICATION:
		msleep(300);
		retval = syna_tcm_get_app_info(tcm_hcd);
		if (retval < 0) {
			TS_LOG_ERR(
					"Failed to get application info\n");
			goto exit;
		}
		break;

	default:
		break;
	}

	retval = 0;

exit:
	//mutex_unlock(&tcm_hcd->identify_mutex);

	kfree(resp_buf);

	return retval;
}

/**
 * touch_set_input_reporting() - Configure touch report and set up new input
 * device if necessary
 *
 * After a device reset event, configure the touch report and set up a new input
 * device if any of the input parameters has changed after the device reset.
 */

static int touch_set_input_reporting(void)
{
	int retval = NO_ERR;
	struct syna_tcm_hcd *tcm_hcd = touch_hcd->tcm_hcd;

	touch_hcd->report_touch = false;

	retval = syna_tcm_get_id_info(tcm_hcd);	
	if (retval < 0) {
		TS_LOG_ERR("%s: failed to write msg\n", __func__);
		return retval;
	}

	if (tcm_hcd->id_info.mode != MODE_APPLICATION ||
			tcm_hcd->app_status != APP_STATUS_OK) {
		TS_LOG_ERR(
				"Application firmware not running\n");
		return -EIO;
	}

	//mutex_lock(&touch_hcd->report_mutex);

	retval = touch_get_input_params();
	if (retval < 0) {
		TS_LOG_ERR(
				"Failed to get input parameters\n");
		goto exit;
	}

	retval = touch_check_input_params();
	if (retval < 0) {
		TS_LOG_ERR(
				"Failed to check input parameters\n");
		goto exit;
	} else if (retval == 0) {
		TS_LOG_INFO(
				"Input parameters unchanged\n");
		goto exit;
	}

exit:
	//mutex_unlock(&touch_hcd->report_mutex);

	touch_hcd->report_touch = retval < 0 ? false : true;

	return retval;
}

int touch_init(struct syna_tcm_hcd *tcm_hcd)
{
	int retval = NO_ERR;

	touch_hcd = kzalloc(sizeof(*touch_hcd), GFP_KERNEL);
	if (!touch_hcd) {
		TS_LOG_ERR(
				"Failed to allocate memory for touch_hcd\n");
		return -ENOMEM;
	}

	touch_hcd->tcm_hcd = tcm_hcd;

	mutex_init(&touch_hcd->report_mutex);

	INIT_BUFFER(touch_hcd->out, false);
	INIT_BUFFER(touch_hcd->resp, false);

	retval = touch_set_input_reporting();
	if (retval < 0) {
		TS_LOG_ERR(
				"Failed to set up input reporting\n");
		goto err_set_input_reporting;
	}

	tcm_hcd->report_touch = touch_report;

	kfree(touch_hcd->prev_status);
	touch_hcd->prev_status = kzalloc(touch_hcd->max_objects, GFP_KERNEL);
	if (!touch_hcd->prev_status) {
		TS_LOG_ERR(
				"Failed to allocate memory for touch_hcd->prev_status\n");
		return -ENOMEM;
	}

	return 0;

err_set_input_reporting:
	kfree(touch_hcd->touch_data.object_data);
	kfree(touch_hcd->prev_status);

	RELEASE_BUFFER(touch_hcd->resp);
	RELEASE_BUFFER(touch_hcd->out);

	kfree(touch_hcd);
	touch_hcd = NULL;

	return retval;
}

MODULE_AUTHOR("Synaptics, Inc.");
MODULE_DESCRIPTION("Synaptics TCM Touch Module");
MODULE_LICENSE("GPL v2");
