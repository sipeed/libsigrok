/*
 * This file is part of the libsigrok project.
 *
 * Copyright (C) 2023 taorye <taorye@outlook.com>
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

#define NUM_MAX_TRANSFERS 64
#define TRANSFERS_DURATION_TOLERANCE 0.05f

struct slogic_model {
	char *name;
	uint8_t ep_in;
	uint64_t max_samplerate; // limit by hardware
	uint64_t max_samplechannel; // limit by hardware
	uint64_t max_bandwidth; // limit by hardware
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
	}; // usb

	int acq_aborted;

	/* Triggers */
	uint64_t capture_ratio;
	gboolean trigger_fired;
	struct soft_trigger_logic *stl;

	double voltage_threshold[2];
};

#pragma pack(push, 1)
struct cmd_start_acquisition {
  union {
    struct {
      uint8_t sample_rate_l;
      uint8_t sample_rate_h;
    };
    uint16_t sample_rate;
  };
	uint8_t sample_channel;
};
#pragma pack(pop)

/* Protocol commands */
#define CMD_START	0xb1
#define CMD_STOP	0xb3

SR_PRIV int sipeed_slogic_acquisition_start(const struct sr_dev_inst *sdi);
SR_PRIV int sipeed_slogic_acquisition_stop(struct sr_dev_inst *sdi);

static inline void clear_ep(uint8_t ep, libusb_device_handle *usbh) {
	sr_dbg("Clearring EP: %u", ep);
	uint8_t tmp[512];
	int actual_length = 0;
	do {
		libusb_bulk_transfer(usbh, ep | LIBUSB_ENDPOINT_IN,
				tmp, sizeof(tmp), &actual_length, 100);
	} while (actual_length);
	sr_dbg("Cleared EP: %u", ep);
}

#define CONTROL_IN_REQ_REG_READ 	0x00
#define CONTROL_OUT_REQ_REG_WRITE 	0x01

static inline int slogic_basic_16_u3_reg_write(const struct sr_dev_inst *sdi, uint16_t value, uint8_t *data, size_t len)
{
	int ret;
	struct dev_context *devc;
	struct sr_usb_dev_inst *usb;

	devc = sdi->priv;
	usb  = sdi->conn;

	sr_spew("%s %u %p:%u.", __func__, value, data, len);
	if (!data || !len) {
		sr_err("%s failed(Nothing to write)!", __func__);
		return SR_ERR_ARG;
	}

	if ((ret = libusb_control_transfer(
		usb->devhdl, LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_OUT,
		CONTROL_OUT_REQ_REG_WRITE,
		value, 0x0000,
		(unsigned char *)data, len,
		500
	)) < 0) {
		sr_err("%s failed(libusb: %s)!", __func__, libusb_error_name(ret));
		return SR_ERR_NA;
	}
	return ret;
}


static inline int slogic_basic_16_u3_reg_read(const struct sr_dev_inst *sdi, uint16_t value, uint8_t *data, size_t len)
{
	int ret;
	struct dev_context *devc;
	struct sr_usb_dev_inst *usb;

	devc = sdi->priv;
	usb  = sdi->conn;

	sr_spew("%s %u %p:%u.", __func__, value, data, len);
	if (!data || !len) {
		sr_err("%s failed(Nothing to read)!", __func__);
		return SR_ERR_ARG;
	}

	if ((ret = libusb_control_transfer(
		usb->devhdl, LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_IN,
		CONTROL_IN_REQ_REG_READ,
		value, 0x0000,
		(unsigned char *)data, len,
		500
	)) < 0) {
		sr_err("%s failed(libusb: %s)!", __func__, libusb_error_name(ret));
		return SR_ERR_NA;
	}
	return ret;
}

#endif
