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

#define EP_IN 0x02
#define SIZE_MAX_EP_HS 512
#define NUM_MAX_TRANSFERS 64

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
	}; // usb

	uint64_t num_transfers;
	struct libusb_transfer *transfers[NUM_MAX_TRANSFERS];

	gboolean acq_aborted;

	uint64_t timeout;

	size_t transfers_buffer_size;

	size_t bytes_need_transfer;
	size_t bytes_transferring;
	size_t bytes_transfered;
	size_t transfers_used;

	gboolean trigger_fired;
	struct soft_trigger_logic *stl;

	uint64_t num_frames;
	uint64_t sent_samples;
	int submitted_transfers;
	int empty_transfer_count;


	double voltage_threshold[2];
	/* Triggers */
	uint64_t capture_ratio;
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

static inline size_t to_bytes_per_ms(struct dev_context *devc)
{
	return (devc->cur_samplerate * devc->cur_samplechannel) / 8 / 1000;
}

static inline size_t get_buffer_size(struct dev_context *devc)
{
	/**
	 * The buffer should be large enough to hold 10ms of data and
	 * a multiple of 210 * 512.
	 */
	// const size_t pack_size = SIZE_MAX_EP_HS;
	// size_t s = 10 * to_bytes_per_ms(devc);
	// size_t rem = s % (210 * pack_size);
	// if (rem) s += 210 * pack_size - rem;
	// return s;
	return 210 * SIZE_MAX_EP_HS;
}

static inline size_t get_number_of_transfers(struct dev_context *devc)
{
	/* Total buffer size should be able to hold about 500ms of data. */
	// size_t s = 500 * to_bytes_per_ms(devc);
	// size_t n = s / get_buffer_size(devc);
	// size_t rem = s % get_buffer_size(devc);
	// if (rem) n += 1;
	// if (n > NUM_MAX_TRANSFERS)
	// 	return NUM_MAX_TRANSFERS;
	// return n;
	return 1;
}

static inline size_t get_timeout(struct dev_context *devc)
{
	size_t total_size = get_buffer_size(devc) *
			get_number_of_transfers(devc);
	size_t timeout = total_size / to_bytes_per_ms(devc);
	return timeout * 5 / 4; /* Leave a headroom of 25% percent. */
}

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
