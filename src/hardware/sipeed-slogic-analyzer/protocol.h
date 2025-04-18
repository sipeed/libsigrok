/*
 * This file is part of the libsigrok project.
 *
 * Copyright (C) 2023-2025 Shenzhen Sipeed Technology Co., Ltd. (深圳市矽速科技有限公司) <support@sipeed.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef LIBSIGROK_HARDWARE_SIPEED_SLOGIC_ANALYZER_PROTOCOL_H
#define LIBSIGROK_HARDWARE_SIPEED_SLOGIC_ANALYZER_PROTOCOL_H

#include <stdint.h>
#include <glib.h>
#include <libusb.h>
#include <libsigrok/libsigrok.h>
#include "libsigrok-internal.h"

#define LOG_PREFIX "sipeed-slogic-analyzer"

#define USB_VID_SIPEED UINT16_C(0x359f)
#define NUM_MAX_TRANSFERS 64
#define TRANSFERS_DURATION_TOLERANCE 0.05f

enum {
	PATTERN_MODE_NOMAL,
	PATTERN_MODE_TEST_MAX_SPEED,
};

struct slogic_model {
	char *name;
	uint16_t pid;
	uint8_t ep_in;
	uint64_t max_samplerate; // limit by hardware
	uint64_t max_samplechannel; // limit by hardware
	uint64_t max_bandwidth; // limit by hardware
	struct {
		int (*remote_run)(const struct sr_dev_inst *sdi);
		int (*remote_stop)(const struct sr_dev_inst *sdi);
	} operation;
	void (*submit_raw_data)(void *data, size_t len, const struct sr_dev_inst *sdi);
};

struct dev_context {
	struct slogic_model *model;

	struct sr_channel_group *digital_group;

	struct {
		uint64_t limit_samplerate;
		uint64_t limit_samplechannel;
	};

	struct {
		uint64_t cur_limit_samples;
		uint64_t cur_samplerate;
		uint64_t cur_samplechannel;
		int64_t cur_pattern_mode_idx;
	}; // configuration

	struct {
		enum libusb_speed speed;

		uint64_t samples_need_nbytes;
		uint64_t samples_got_nbytes;

		uint64_t per_transfer_duration; /* unit: ms */
		uint64_t per_transfer_nbytes;

		size_t num_transfers_completed;
		size_t num_transfers_used;
		struct libusb_transfer *transfers[NUM_MAX_TRANSFERS];

		uint64_t transfers_reached_nbytes; /* real received bytes in all */
		uint64_t transfers_reached_nbytes_latest; /* real received bytes this transfer */
		int64_t transfers_reached_time_start;
		int64_t transfers_reached_time_latest;

		GThread *raw_data_handle_thread;
		GAsyncQueue *raw_data_queue;
		uint64_t timeout_count;
	}; // usb

	int acq_aborted;

	/* Triggers */
	uint64_t capture_ratio;
	gboolean trigger_fired;
	struct soft_trigger_logic *stl;

	double voltage_threshold[2];
};

SR_PRIV int sipeed_slogic_acquisition_start(const struct sr_dev_inst *sdi);
SR_PRIV int sipeed_slogic_acquisition_stop(struct sr_dev_inst *sdi);

static inline void clear_ep(const struct sr_dev_inst *sdi) {
	struct dev_context *devc = sdi->priv;
	struct sr_usb_dev_inst *usb = sdi->conn;
	uint8_t ep = devc->model->ep_in;

	size_t tmp_size = 4 * 1024 * 1024;
	uint8_t *tmp = malloc(tmp_size);
	int actual_length = 0;
	do {
		libusb_bulk_transfer(usb->devhdl, ep,
				tmp, tmp_size, &actual_length, 100);
	} while (actual_length);
	free(tmp);
	sr_dbg("Cleared EP: 0x%02x", ep);
}

#endif
