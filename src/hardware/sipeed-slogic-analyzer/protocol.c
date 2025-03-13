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

#include <config.h>
#include "protocol.h"

static int handle_events(int fd, int revents, void *cb_data);

static void submit_data(void *data, size_t len, struct sr_dev_inst *sdi) {
	struct sr_datafeed_logic logic = {
		.length = len,
		.unitsize = 1,
		.data = data,
	};

	struct sr_datafeed_packet packet = {
		.type = SR_DF_LOGIC,
		.payload = &logic
	};

	sr_session_send(sdi, &packet);
}

static void LIBUSB_CALL receive_transfer(struct libusb_transfer *transfer) {

	int ret;
	const struct sr_dev_inst *sdi;
	struct dev_context *devc;
	struct sr_usb_dev_inst *usb;

	sdi  = transfer->user_data;
	if (!sdi)
		return;
	devc = sdi->priv;
	usb  = sdi->conn;

	sr_spew("[%p]usb status: %d", transfer, transfer->status);
	switch (transfer->status) {
		case LIBUSB_TRANSFER_COMPLETED: 
		case LIBUSB_TRANSFER_TIMED_OUT: { /* may have received some data */
			sr_dbg("\ttransfing: %u - %u, done: %u + %u, remain: %u/%u",
				devc->bytes_transferring, transfer->length,
				devc->bytes_transfered, transfer->actual_length,
				devc->bytes_need_transfer + (transfer->length - transfer->actual_length) - (devc->bytes_transfered + devc->bytes_transferring),
				devc->bytes_need_transfer
			);

			devc->bytes_transfered += transfer->actual_length;
			devc->bytes_transferring -= transfer->length;

			if (transfer->actual_length == 0) {
				devc->transfers_used -= 1;
				break;
			}

			{
				// sr_dbg("samplechannel: %d, actual_length: %u", devc->cur_samplechannel, transfer->actual_length);
				// devc->cur_samplechannel == 16
				uint8_t * d = transfer->buffer;
				size_t len = transfer->actual_length;
				// sr_dbg("HEAD: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
				// 	d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7], d[8], d[9], d[10], d[11], d[12], d[13], d[14], d[15]);

				uint8_t *ptr = g_malloc(len);

				for(size_t i=0; i<len; i+=16) {
					for(size_t j=0; j< 8; j++) {
						#define B(n) (((d[i+(n)] >> 7-j) & 0x1) << ((n)%8))
						ptr[i+j*2+0] =
							B(0)|B(1)|B(2)|B(3)|B(4)|B(5)|B(6)|B(7);
						ptr[i+j*2+1] =
							B(8)|B(9)|B(10)|B(11)|B(12)|B(13)|B(14)|B(15);
						#undef B

						// ptr[i+j*2+0] |= 0x1;
						// ptr[i+j*2+1] |= 0x80;
					}
				}

				struct sr_datafeed_logic logic = {
					.length = len,
					.unitsize = (devc->cur_samplechannel + 7)/8,
					.data = ptr,
				};
			
				struct sr_datafeed_packet packet = {
					.type = SR_DF_LOGIC,
					.payload = &logic
				};
			
				sr_session_send(sdi, &packet);
				g_free(ptr);
			}

			size_t bytes_to_transfer = devc->bytes_need_transfer -
				(devc->bytes_transfered + devc->bytes_transferring);
			if (bytes_to_transfer > devc->transfers_buffer_size) {
				bytes_to_transfer = devc->transfers_buffer_size;
			}
			if (bytes_to_transfer) {
				transfer->length = bytes_to_transfer;
				transfer->actual_length = 0;
				transfer->timeout = 1000 + devc->timeout * devc->transfers_used;
				ret = libusb_submit_transfer(transfer);
				if (ret) {
					sr_warn("Failed to submit transfer: %s", libusb_error_name(ret));
					devc->transfers_used -= 1;
				}
				sr_dbg("submit transfer: %p", transfer);

				devc->bytes_transferring += bytes_to_transfer;
			} else {
				devc->transfers_used -= 1;
			}
		} break;

		case LIBUSB_TRANSFER_NO_DEVICE: {
			devc->transfers_used = 0;
		} break;

		default: {
			devc->transfers_used -= 1;
		} break;
	}

	if (devc->transfers_used == 0) {
		sr_dbg("free all transfers");

		sr_dbg("Bulk in %u/%u bytes", devc->bytes_transfered,
				devc->bytes_need_transfer);
		
		sr_dev_acquisition_stop(sdi);
	}
};

static int handle_events(int fd, int revents, void *cb_data)
{
	struct sr_dev_inst *sdi;
	struct sr_dev_driver *di;
	struct dev_context *devc;
	struct drv_context *drvc;
	struct timeval tv;

	(void)fd;
	(void)revents;

	sdi = cb_data;
	devc = sdi->priv;
	di = sdi->driver;
	drvc = di->context;

	sr_spew("handle_events enter");

	if (devc->acq_aborted == TRUE) {
		for (size_t i = 0; i < NUM_MAX_TRANSFERS; ++i) {
			struct libusb_transfer *transfer = devc->transfers[i];
			if (transfer) {
				libusb_cancel_transfer(transfer);
				g_free(transfer->buffer);
				libusb_free_transfer(transfer);
			}
			devc->transfers[i] = NULL;
		}

		// usb_source_remove(sdi->session, drvc->sr_ctx);
		sr_session_source_remove(sdi->session, -1 * (size_t)drvc->sr_ctx->libusb_ctx);
		std_session_send_df_end(sdi);
	}

	memset(&tv, 0, sizeof(struct timeval));
	libusb_handle_events_timeout_completed(drvc->sr_ctx->libusb_ctx, &tv,
			&devc->acq_aborted);

	return TRUE;
}

SR_PRIV int sipeed_slogic_acquisition_start(const struct sr_dev_inst *sdi)
{
	// sr_dbg("Enter func %s", __func__);
	/* TODO: configure hardware, reset acquisition state, set up
	 * callbacks and send header packet. */

	struct sr_dev_driver *di;
	struct dev_context *devc;
	struct drv_context *drvc;
	struct sr_usb_dev_inst *usb;

	int ret;
	size_t size_transfer_buf;

	devc = sdi->priv;
	di = sdi->driver;
	drvc = di->context;
	usb  = sdi->conn;

	uint8_t cmd_rst[] = {0x02, 0x00, 0x00, 0x00};
	ret = slogic_basic_16_u3_reg_write(sdi, 0x0004, ARRAY_AND_SIZE(cmd_rst));
	if (ret < 0) {
		sr_err("Unhandled `CMD_RST`");
		return ret;
	}
	// clear_ep(EP_IN, usb->devhdl);

	sr_info("Need %ux %uch@%uMHz in %ums.", 
			devc->cur_limit_samples,
			devc->cur_samplechannel, 
			devc->cur_samplerate / SR_MHZ(1),
			1000 * devc->cur_limit_samples / devc->cur_samplerate
	);

	devc->per_transfer_duration = 500;
	devc->per_transfer_nbytes = devc->cur_samplerate / SR_KHZ(1) * devc->cur_samplechannel / 8 * devc->per_transfer_duration /* ms */;
	devc->per_transfer_nbytes = (devc->per_transfer_nbytes + 1023) & ~1023;

	do {
		struct libusb_transfer *transfer = libusb_alloc_transfer(0);
		if (!transfer) {
			sr_err("Failed to allocate libusb transfer!");
			return SR_ERR_IO;
		}
		while (devc->per_transfer_nbytes > 128*1024) { // 128kiB > 500ms * 1MHZ * 2ch
			devc->per_transfer_duration = devc->per_transfer_nbytes / (devc->cur_samplerate / SR_KHZ(1) * devc->cur_samplechannel / 8);
			sr_dbg("Plan to receive %u bytes per %ums...", devc->per_transfer_nbytes, devc->per_transfer_duration);
			
			uint8_t *dev_buf = g_malloc(devc->per_transfer_nbytes);
			if (!dev_buf) {
				sr_warn("Failed to allocate memory: %u bytes!", devc->per_transfer_nbytes);
				devc->per_transfer_nbytes >>= 1;
				continue;
			}

			libusb_fill_bulk_transfer(transfer, usb->devhdl, devc->model->ep_in,
										dev_buf, devc->per_transfer_nbytes, receive_transfer,
										NULL, devc->per_transfer_duration);

			ret = libusb_submit_transfer(transfer);
			libusb_handle_events_timeout(drvc->sr_ctx->libusb_ctx, &(struct timeval){0, 0});
			if (ret) {
				sr_warn("Failed to submit transfer: %s!", libusb_error_name(ret));
				g_free(transfer->buffer);
				if (ret == LIBUSB_ERROR_NO_MEM)
					devc->per_transfer_nbytes >>= 1;
				else
					return SR_ERR_IO;
				continue;
			} else {
				ret = libusb_cancel_transfer(transfer);
				libusb_handle_events_timeout(drvc->sr_ctx->libusb_ctx, &(struct timeval){0, 0});
				g_free(transfer->buffer);
				break;
			}
		}
		libusb_free_transfer(transfer);
		if (devc->per_transfer_nbytes < 128*1024) {
			return SR_ERR_MALLOC;
		}
		sr_dbg("Nice plan! :)");
	} while (0);


	devc->acq_aborted = FALSE;
  	devc->bytes_need_transfer = 0;
	devc->bytes_transferring = 0;
	devc->bytes_transfered = 0;
	devc->transfers_used = 0;
	memset(devc->transfers, 0, sizeof(devc->transfers));

	devc->transfers_buffer_size = get_buffer_size(devc);
	sr_dbg("transfers_buffer_size: %u", devc->transfers_buffer_size);

	devc->timeout = get_timeout(devc);
	sr_dbg("timeout: %ums", devc->timeout);
	// usb_source_add(sdi->session, drvc->sr_ctx, 10,
	// 		handle_events, (void *)sdi);

	sr_session_source_add(sdi->session, -1 * (size_t)drvc->sr_ctx->libusb_ctx, 0, devc->timeout, handle_events, (void *)sdi);

	/* compute needed bytes */
	uint64_t samples_in_bytes = devc->cur_limit_samples * devc->cur_samplechannel / 8;
	devc->bytes_need_transfer = samples_in_bytes / devc->transfers_buffer_size;
	devc->bytes_need_transfer += !!(samples_in_bytes % devc->transfers_buffer_size);
	devc->bytes_need_transfer *= devc->transfers_buffer_size;

	while (devc->transfers_used < NUM_MAX_TRANSFERS && devc->bytes_transfered
			+ devc->bytes_transferring < devc->bytes_need_transfer)
	{
		uint8_t *dev_buf = g_malloc(devc->transfers_buffer_size);
		if (!dev_buf) {
			sr_warn("Failed to allocate memory");
			break;
		}

		struct libusb_transfer *transfer = libusb_alloc_transfer(0);
		if (!transfer) {
			g_free(dev_buf);
			sr_warn("Failed to allocate transfer");
			break;
		}

		size_t bytes_to_transfer = devc->bytes_need_transfer -
			devc->bytes_transfered - devc->bytes_transferring;
		if (bytes_to_transfer > devc->transfers_buffer_size) {
			bytes_to_transfer = devc->transfers_buffer_size;
		}

		libusb_fill_bulk_transfer(transfer, usb->devhdl, EP_IN | LIBUSB_ENDPOINT_IN,
									dev_buf, bytes_to_transfer, receive_transfer,
									sdi, 3500 + devc->timeout * (devc->transfers_used + 1));
		transfer->actual_length = 0;

		ret = libusb_submit_transfer(transfer);
		if (ret) {
			sr_warn("Failed to submit transfer[%d]: %s.", devc->transfers_used, libusb_error_name(ret));
			g_free(transfer->buffer);
			libusb_free_transfer(transfer);
			break;
		}
		devc->transfers[devc->transfers_used] = transfer;

		devc->bytes_transferring += bytes_to_transfer;
		devc->transfers_used += 1;
	}
	sr_dbg("Submited %u transfers", devc->transfers_used);

	std_session_send_df_header(sdi);
	std_session_send_df_frame_begin(sdi);

	uint8_t cmd_run[] = {0x01, 0x00, 0x00, 0x00};
	ret = slogic_basic_16_u3_reg_write(sdi, 0x0004, ARRAY_AND_SIZE(cmd_run));
	if (ret < 0) {
		sr_err("Unhandled `CMD_RUN`");
		return ret;
	}

	return SR_OK;
}

SR_PRIV int sipeed_slogic_acquisition_stop(struct sr_dev_inst *sdi)
{

	struct dev_context *devc;

	devc = sdi->priv;

	devc->acq_aborted = TRUE;

	return SR_OK;
}
