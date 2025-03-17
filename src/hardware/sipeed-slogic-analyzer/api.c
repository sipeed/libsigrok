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

static const uint32_t scanopts[] = {
	SR_CONF_CONN,
};

static const uint32_t drvopts[] = {
	SR_CONF_LOGIC_ANALYZER,
};

static const uint32_t devopts[] = {
	SR_CONF_CONTINUOUS,
	SR_CONF_LIMIT_SAMPLES | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_SAMPLERATE    | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_BUFFERSIZE | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_PATTERN_MODE | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_TRIGGER_MATCH | SR_CONF_GET | SR_CONF_LIST,
};


static int slogic_lite_8_remote_run(const struct sr_dev_inst *sdi);
static int slogic_lite_8_remote_stop(const struct sr_dev_inst *sdi);
static void slogic_lite_8_submit_raw_data(void *data, size_t len, const struct sr_dev_inst *sdi);
static int slogic_basic_16_remote_run(const struct sr_dev_inst *sdi);
static int slogic_basic_16_remote_stop(const struct sr_dev_inst *sdi);
static void slogic_basic_16_submit_raw_data(void *data, size_t len, const struct sr_dev_inst *sdi);

static const struct slogic_model support_models[] = {
	{
		.name = "Slogic Lite 8",
		.pid = 0x0300,
		.ep_in = 0x01 | LIBUSB_ENDPOINT_IN,
		.max_samplerate = SR_MHZ(160),
		.max_samplechannel = 8,
		.max_bandwidth = SR_MHZ(320),
		.operation = {
			.remote_run = slogic_lite_8_remote_run,
			.remote_stop = slogic_lite_8_remote_stop,
		},
		.submit_raw_data = slogic_lite_8_submit_raw_data,
	},
	{
		.name = "Slogic Basic 16 U3",
		.pid = 0x3031,
		.ep_in = 0x02 | LIBUSB_ENDPOINT_IN,
		.max_samplerate = SR_MHZ(1600),
		.max_samplechannel = 16,
		.max_bandwidth = SR_MHZ(3200),
		.operation = {
			.remote_run = slogic_basic_16_remote_run,
			.remote_stop = slogic_basic_16_remote_stop,
		},
		.submit_raw_data = slogic_basic_16_submit_raw_data,
	},
	{
		.name = NULL,
		.pid = 0x0000,
	}
};

static const uint64_t samplerates[] = {
	/* 160M = 2^5*5M */
	/* 1600M = 2^6*5^2M */
	SR_MHZ(1),
	SR_MHZ(2),
	SR_MHZ(4),
	SR_MHZ(5),
	SR_MHZ(8),
	SR_MHZ(10),
	SR_MHZ(16),
	SR_MHZ(20),
	SR_MHZ(32),

	/* Slogic Lite 8 */
	/* x 8ch */
	SR_MHZ(40),
	/* x 4ch */
	SR_MHZ(80),
	/* x 2ch */
	SR_MHZ(160),

	/* Slogic Basic 16 U3 */
	/* x 16ch */
	SR_MHZ(200),
	/* x 8ch */
	SR_MHZ(400),
	/* x 4ch */
	SR_MHZ(800),
	/* x 2ch */
	SR_MHZ(1600),
};

static const uint64_t buffersizes[] = {
	2, 4, 8, 16
};

static const char *patterns[] = {
#define GEN_PATTERN(P) [P] = #P
	GEN_PATTERN(PATTERN_MODE_NOMAL),
	GEN_PATTERN(PATTERN_MODE_TEST_MAX_SPEED),
#undef GEN_PATTERN
};

static const int32_t trigger_matches[] = {
	SR_TRIGGER_ZERO,
	SR_TRIGGER_ONE,
	SR_TRIGGER_RISING,
	SR_TRIGGER_FALLING,
	SR_TRIGGER_EDGE,
};

static struct sr_dev_driver sipeed_slogic_analyzer_driver_info;

static GSList *scan(struct sr_dev_driver *di, GSList *options)
{
	int ret;
	struct sr_dev_inst *sdi;
	struct sr_usb_dev_inst *usb;
	struct drv_context *drvc;
	struct dev_context *devc;

	struct slogic_model *model;
	struct sr_config *option;
	struct libusb_device_descriptor des;
	GSList *devices;
	GSList *l, *conn_devices;
	const char *conn;
	char cbuf[128];
	char *iManufacturer, *iProduct, *iSerialNumber, *iPortPath;

	struct sr_channel *ch;
	unsigned int i;
	gchar *channel_name;

	(void)options;

	conn = NULL;

	devices = NULL;
	drvc = di->context;
	// drvc->instances = NULL;
	
	/* scan for devices, either based on a SR_CONF_CONN option
	 * or on a USB scan. */
	for (l = options; l; l = l->next) {
		option = l->data;
		switch (option->key) {
		case SR_CONF_CONN:
			conn = g_variant_get_string(option->data, NULL);
			sr_info("Use conn: %s", conn);
			sr_err("Not supported now!");
			return NULL;
			break;
		default:
			sr_warn("Unhandled option key: %u", option->key);
		}
	}

	for (model = &support_models[0]; model->name; model++) {
		conn = g_strdup_printf("%04x.%04x", USB_VID_SIPEED, model->pid);
		/* Find all slogic compatible devices. */
		conn_devices = sr_usb_find(drvc->sr_ctx->libusb_ctx, conn);
		for(l = conn_devices; l; l = l->next) {
			usb = l->data;
			ret = sr_usb_open(drvc->sr_ctx->libusb_ctx, usb);
			if (SR_OK != ret) continue;
			libusb_get_device_descriptor(
				libusb_get_device(usb->devhdl), &des);
			libusb_get_string_descriptor_ascii(usb->devhdl,
					des.iManufacturer, cbuf, sizeof(cbuf));
			iManufacturer = g_strdup(cbuf);
			libusb_get_string_descriptor_ascii(usb->devhdl,
					des.iProduct, cbuf, sizeof(cbuf));
			iProduct = g_strdup(cbuf);
			libusb_get_string_descriptor_ascii(usb->devhdl,
					des.iSerialNumber, cbuf, sizeof(cbuf));
			iSerialNumber = g_strdup(cbuf);
			usb_get_port_path(libusb_get_device(usb->devhdl),
					cbuf, sizeof(cbuf));
			iPortPath = g_strdup(cbuf);

			sdi = sr_dev_inst_user_new(iManufacturer, iProduct, NULL);
			sdi->serial_num = iSerialNumber;
			sdi->connection_id = iPortPath;
			sdi->status = SR_ST_INACTIVE;
			sdi->conn = usb;
			sdi->inst_type = SR_INST_USB;

			devc = g_malloc0(sizeof(struct dev_context));
			sdi->priv = devc;

			{
				devc->model = model;

				devc->limit_samplechannel = devc->model->max_samplechannel;
				devc->limit_samplerate = devc->model->max_bandwidth / devc->model->max_samplechannel;

				devc->cur_samplechannel = devc->limit_samplechannel;
				devc->cur_samplerate = devc->limit_samplerate;
				devc->cur_pattern_mode_idx = PATTERN_MODE_NOMAL;

				devc->digital_group = sr_channel_group_new(sdi, "LA", NULL);
				for (i = 0; i < devc->model->max_samplechannel; i++) {
					channel_name = g_strdup_printf("D%u", i);
					ch = sr_channel_new(sdi, i, SR_CHANNEL_LOGIC, TRUE, channel_name);
					g_free(channel_name);
					devc->digital_group->channels = g_slist_append(
						devc->digital_group->channels, ch);
				}

				devc->speed = libusb_get_device_speed(libusb_get_device(usb->devhdl));
			}

			sr_usb_close(usb);
			devices = g_slist_append(devices, sdi);
		}
		// g_slist_free_full(conn_devices, (GDestroyNotify)sr_usb_dev_inst_free);
		g_free(conn);
	}

	return std_scan_complete(di, devices);
}

static int dev_open(struct sr_dev_inst *sdi)
{
	int ret;
	struct sr_usb_dev_inst *usb;
	struct dev_context *devc;
	struct sr_dev_driver *di;
	struct drv_context *drvc;

	usb  = sdi->conn;
	devc = sdi->priv;
	di	 = sdi->driver;
	drvc = di->context;

	ret = sr_usb_open(drvc->sr_ctx->libusb_ctx, usb);
	if (SR_OK != ret) return ret;

	ret = libusb_claim_interface(usb->devhdl, 0);
	if (ret != LIBUSB_SUCCESS) {
		switch (ret) {
		case LIBUSB_ERROR_BUSY:
			sr_err("Unable to claim USB interface. Another "
			       "program or driver has already claimed it.");
			break;
		case LIBUSB_ERROR_NO_DEVICE:
			sr_err("Device has been disconnected.");
			break;
		default:
			sr_err("Unable to claim interface: %s.",
			       libusb_error_name(ret));
			break;
		}
		return SR_ERR;
	}

	return std_dummy_dev_open(sdi);
}

static int dev_close(struct sr_dev_inst *sdi)
{
	int ret;
	struct sr_usb_dev_inst *usb;
	struct dev_context *devc;
	struct sr_dev_driver *di;
	struct drv_context *drvc;

	usb  = sdi->conn;
	devc = sdi->priv;
	di	 = sdi->driver;
	drvc = di->context;

	ret = libusb_release_interface(usb->devhdl, 0);
	if (ret != LIBUSB_SUCCESS) {
		switch (ret) {
		case LIBUSB_ERROR_NO_DEVICE:
			sr_err("Device has been disconnected.");
			// return SR_ERR_DEV_CLOSED;
			break;
		default:
			sr_err("Unable to release Interface for %s.",
					libusb_error_name(ret));
			break;
		}
	}

	sr_usb_close(usb);

	return std_dummy_dev_close(sdi);
}

static int config_get(uint32_t key, GVariant **data,
	const struct sr_dev_inst *sdi, const struct sr_channel_group *cg)
{
	int ret;
	struct dev_context *devc;

	(void)cg;

	devc = sdi->priv;

	ret = SR_OK;
	switch (key) {
	case SR_CONF_SAMPLERATE:
		*data = g_variant_new_uint64(devc->cur_samplerate);
		break;
	case SR_CONF_BUFFERSIZE:
		*data = g_variant_new_uint64(devc->cur_samplechannel);
		break;
	case SR_CONF_PATTERN_MODE:
		*data = g_variant_new_string(patterns[devc->cur_pattern_mode_idx]);
		break;
	case SR_CONF_LIMIT_SAMPLES:
		*data = g_variant_new_uint64(devc->cur_limit_samples);
		break;
	default:
		return SR_ERR_NA;
	}

	return ret;
}

static int config_set(uint32_t key, GVariant *data,
	const struct sr_dev_inst *sdi, const struct sr_channel_group *cg)
{
	int ret;
	struct dev_context *devc;

	(void)cg;

	devc = sdi->priv;

	ret = SR_OK;
	switch (key) {
	case SR_CONF_SAMPLERATE:
		if (g_variant_get_uint64(data) > devc->limit_samplerate || std_u64_idx(data, ARRAY_AND_SIZE(samplerates)) < 0) {
			devc->cur_samplerate = devc->limit_samplerate;
			sr_warn("Reach limit or not supported, wrap to %uMHz.", devc->limit_samplerate/SR_MHZ(1));
		} else {
			devc->cur_samplerate = g_variant_get_uint64(data);
		}
		devc->limit_samplechannel = devc->model->max_bandwidth / devc->cur_samplerate;
		if (devc->limit_samplechannel > devc->model->max_samplechannel)
			devc->limit_samplechannel = devc->model->max_samplechannel;
		break;
	case SR_CONF_BUFFERSIZE:
		if (g_variant_get_uint64(data) > devc->limit_samplechannel || std_u64_idx(data, ARRAY_AND_SIZE(buffersizes)) < 0) {
			devc->cur_samplechannel = devc->limit_samplechannel;
			sr_warn("Reach limit or not supported, wrap to %uch.", devc->limit_samplechannel);
		} else {
			devc->cur_samplechannel = g_variant_get_uint64(data);
		}
		devc->limit_samplerate = devc->model->max_bandwidth / devc->cur_samplechannel;
		if (devc->limit_samplerate > devc->model->max_samplerate)
			devc->limit_samplerate = devc->model->max_samplerate;

		// [en|dis]able channels and dbg
		{
			for (GSList *l = devc->digital_group->channels; l; l = l->next) {
				struct sr_channel *ch = l->data;
				if (ch->type == SR_CHANNEL_LOGIC) { /* Might as well do this now, these are static. */
					sr_dev_channel_enable(ch, (ch->index >= devc->cur_samplechannel) ? FALSE : TRUE);
				} else {
					sr_warn("devc->digital_group->channels[%u] is not Logic?", ch->index);
				}
				sr_dbg("\tch[%2u] %-3s:%d %sabled priv:%p.", ch->index, ch->name, ch->type, ch->enabled?"en":"dis", ch->priv);
			}
		}
		break;
	case SR_CONF_PATTERN_MODE:
		devc->cur_pattern_mode_idx = std_str_idx(data, ARRAY_AND_SIZE(patterns));
		if (devc->cur_pattern_mode_idx < 0)
			devc->cur_pattern_mode_idx = 0;
		break;
	case SR_CONF_LIMIT_SAMPLES:
		devc->cur_limit_samples = g_variant_get_uint64(data);
		break;
	default:
		ret = SR_ERR_NA;
	}

	return ret;
}

static int config_list(uint32_t key, GVariant **data,
	const struct sr_dev_inst *sdi, const struct sr_channel_group *cg)
{
	int ret;
	struct dev_context *devc;

	(void)cg;

	devc = sdi ? (sdi->priv) : NULL;

	ret = SR_OK;
	switch (key) {
	case SR_CONF_SCAN_OPTIONS:
	case SR_CONF_DEVICE_OPTIONS:
		ret = STD_CONFIG_LIST(key, data, sdi, cg, scanopts, drvopts, devopts);
		break;
	case SR_CONF_SAMPLERATE:
		*data = std_gvar_samplerates(samplerates, 1+std_u64_idx(g_variant_new_uint64(devc->limit_samplerate), ARRAY_AND_SIZE(samplerates)));
		break;
	case SR_CONF_BUFFERSIZE:
		*data = std_gvar_array_u64(buffersizes, 1+std_u64_idx(g_variant_new_uint64(devc->limit_samplechannel), ARRAY_AND_SIZE(buffersizes)));
		break;
	case SR_CONF_PATTERN_MODE:
		*data = g_variant_new_strv(ARRAY_AND_SIZE(patterns));
		break;
	case SR_CONF_TRIGGER_MATCH:
		*data = std_gvar_array_i32(ARRAY_AND_SIZE(trigger_matches));
		break;
	default:
		ret = SR_ERR_NA;
	}

	return ret;
}

static struct sr_dev_driver sipeed_slogic_analyzer_driver_info = {
	.name = "sipeed-slogic-analyzer",
	.longname = "Sipeed Slogic Analyzer",
	.api_version = 1,
	.init = std_init,
	.cleanup = std_cleanup,
	.scan = scan,
	.dev_list = std_dev_list,
	.dev_clear = std_dev_clear,
	.config_get = config_get,
	.config_set = config_set,
	.config_list = config_list,
	.dev_open = dev_open,
	.dev_close = dev_close,
	.dev_acquisition_start = sipeed_slogic_acquisition_start,
	.dev_acquisition_stop = sipeed_slogic_acquisition_stop,
	.context = NULL,
};
SR_REGISTER_DEV_DRIVER(sipeed_slogic_analyzer_driver_info);


static int slogic_usb_control_write(const struct sr_dev_inst *sdi, uint8_t request, uint16_t value, uint16_t index, uint8_t *data, size_t len, int timeout);
static int slogic_usb_control_read(const struct sr_dev_inst *sdi, uint8_t request, uint16_t value, uint16_t index, uint8_t *data, size_t len, int timeout);

/* Slogic Lite 8 start */
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

#define CMD_START	0xb1
#define CMD_STOP	0xb3

static int slogic_lite_8_remote_run(const struct sr_dev_inst *sdi) {
	struct dev_context *devc = sdi->priv;
	const struct cmd_start_acquisition cmd_run = {
		.sample_rate = devc->cur_samplerate / SR_MHZ(1),
		.sample_channel = devc->cur_samplechannel,
	};
	return slogic_usb_control_write(sdi, CMD_START, 0x0000, 0x0000, (uint8_t *)&cmd_run, sizeof(cmd_run), 500);
}

static int slogic_lite_8_remote_stop(const struct sr_dev_inst *sdi) {
	struct dev_context *devc = sdi->priv;
	struct sr_usb_dev_inst *usb = sdi->conn;
	clear_ep(sdi);
	return SR_OK;
	/* not stable, but can be ignored */
	// int ret = slogic_usb_control_write(sdi, CMD_STOP, 0x0000, 0x0000, NULL, 0, 500);
	// clear_ep(sdi);
	// return ret;
}

static void slogic_lite_8_submit_raw_data(void *data, size_t len, const struct sr_dev_inst *sdi) {
	struct dev_context *devc = sdi->priv;

	size_t length = len * (8/devc->cur_samplechannel);
	uint8_t *ptr = g_malloc(length);

	for(size_t i=0; i<len; i+=devc->cur_samplechannel) {
		for(size_t j=0; j<8; j++) {
			ptr[i*(8/devc->cur_samplechannel)+j] = (((uint8_t *)data)[i+j/(8/devc->cur_samplechannel)] >> (j%(8/devc->cur_samplechannel) * devc->cur_samplechannel)) & ((1<<devc->cur_samplechannel)-1);
			// switch (devc->cur_samplechannel) {
			// case 8:
			// 	ptr[i*1+j] = (((uint8_t *)data)[i+j/1] >> (j%1 * 8)) & 0xff;
			// break;
			// case 4:
			// 	ptr[i*2+j] = (((uint8_t *)data)[i+j/2] >> (j%2 * 4)) & 0xf;
			// break;
			// case 2:
			// 	ptr[i*4+j] = (((uint8_t *)data)[i+j/4] >> (j%4 * 2)) & 0x3;
			// break;
			// }
		}
	}

	sr_session_send(sdi, &(struct sr_datafeed_packet) {
		.type = SR_DF_LOGIC,
		.payload = &(struct sr_datafeed_logic) {
			.length = length,
			.unitsize = 1,
			.data = ptr,
		}
	});

	g_free(ptr);
}
/* Slogic Lite 8 end */



/* Slogic Basic 16 start */
#define SLOGIC_BASIC_16_CONTROL_IN_REQ_REG_READ 	0x00
#define SLOGIC_BASIC_16_CONTROL_OUT_REQ_REG_WRITE 	0x01

#define SLOGIC_BASIC_16_R32_CTRL 	0x0004
#define SLOGIC_BASIC_16_R32_FLAG 	0x0008
#define SLOGIC_BASIC_16_R32_AUX 	0x000c

static int slogic_basic_16_remote_run(const struct sr_dev_inst *sdi) {
	struct dev_context *devc = sdi->priv;
	const uint8_t cmd_derst[] = {0x00, 0x00, 0x00, 0x00};
	const uint8_t cmd_run[] = {0x01, 0x00, 0x00, 0x00};
	uint8_t cmd_aux_channel[8] = {0x01, 0x00, 0x00, 0x00}; // configure channel
	uint8_t cmd_aux_samplerate[12] = {0x02, 0x00, 0x00, 0x00}; // configure samplerate

	slogic_usb_control_write(sdi, SLOGIC_BASIC_16_CONTROL_OUT_REQ_REG_WRITE, SLOGIC_BASIC_16_R32_CTRL, 0x0000, ARRAY_AND_SIZE(cmd_derst), 500);

	{
		uint8_t *cmd_aux = cmd_aux_channel;
		slogic_usb_control_write(sdi, SLOGIC_BASIC_16_CONTROL_OUT_REQ_REG_WRITE, SLOGIC_BASIC_16_R32_AUX, 0x0000, cmd_aux, 4, 500);
		do {
			slogic_usb_control_read(sdi, SLOGIC_BASIC_16_CONTROL_IN_REQ_REG_READ, SLOGIC_BASIC_16_R32_AUX, 0x0000, cmd_aux, 4, 500);
			sr_dbg("read aux channel.");
		} while (!(cmd_aux[2] & 0x01));
		sr_dbg("channel length: %u.", (*(uint16_t*)cmd_aux)>>8);
		slogic_usb_control_read(sdi, SLOGIC_BASIC_16_CONTROL_IN_REQ_REG_READ, SLOGIC_BASIC_16_R32_AUX, 0x0000, cmd_aux, 4 + (*(uint16_t*)cmd_aux)>>8, 500);

		// sr_dbg("aux: %u %u %u %u %08x.", cmd_aux[0], cmd_aux[1], cmd_aux[2], cmd_aux[3], *(uint32_t*)(cmd_aux+4));

		*(uint32_t*)(cmd_aux+4) = (1 << devc->cur_samplechannel) - 1;

		// sr_dbg("aux: %u %u %u %u %08x.", cmd_aux[0], cmd_aux[1], cmd_aux[2], cmd_aux[3], *(uint32_t*)(cmd_aux+4));
		slogic_usb_control_write(sdi, SLOGIC_BASIC_16_CONTROL_OUT_REQ_REG_WRITE, SLOGIC_BASIC_16_R32_AUX, 0x0000, cmd_aux, 4 + (*(uint16_t*)cmd_aux)>>8, 500);

		slogic_usb_control_read(sdi, SLOGIC_BASIC_16_CONTROL_IN_REQ_REG_READ, SLOGIC_BASIC_16_R32_AUX, 0x0000, cmd_aux, 4 + (*(uint16_t*)cmd_aux)>>8, 500);
		sr_dbg("aux: %u %u %u %u %08x.", cmd_aux[0], cmd_aux[1], cmd_aux[2], cmd_aux[3], *(uint32_t*)(cmd_aux+4));

		if ((1 << devc->cur_samplechannel) - 1 != *(uint32_t*)(cmd_aux+4)) {
			sr_dbg("Failed to configure sample channel.");
		} else {
			sr_dbg("Succeed to configure sample channel.");
		}
	}


	{
		uint8_t *cmd_aux = cmd_aux_samplerate;
		slogic_usb_control_write(sdi, SLOGIC_BASIC_16_CONTROL_OUT_REQ_REG_WRITE, SLOGIC_BASIC_16_R32_AUX, 0x0000, cmd_aux, 4, 500);
		do {
			slogic_usb_control_read(sdi, SLOGIC_BASIC_16_CONTROL_IN_REQ_REG_READ, SLOGIC_BASIC_16_R32_AUX, 0x0000, cmd_aux, 4, 500);
			sr_dbg("read aux samplerate.");
		} while (!(cmd_aux[2] & 0x01));
		sr_dbg("samplerate length: %u.", (*(uint16_t*)cmd_aux)>>8);
		slogic_usb_control_read(sdi, SLOGIC_BASIC_16_CONTROL_IN_REQ_REG_READ, SLOGIC_BASIC_16_R32_AUX, 0x0000, cmd_aux, 4 + (*(uint16_t*)cmd_aux)>>8, 500);

		// sr_dbg("aux: %u %u %u %u %u.", cmd_aux[0], cmd_aux[1], cmd_aux[2], cmd_aux[3], *(uint64_t*)(cmd_aux+4));

		*(uint64_t*)(cmd_aux+4) = devc->cur_samplerate;

		// sr_dbg("aux: %u %u %u %u %u.", cmd_aux[0], cmd_aux[1], cmd_aux[2], cmd_aux[3], *(uint64_t*)(cmd_aux+4));
		slogic_usb_control_write(sdi, SLOGIC_BASIC_16_CONTROL_OUT_REQ_REG_WRITE, SLOGIC_BASIC_16_R32_AUX, 0x0000, cmd_aux, 4 + (*(uint16_t*)cmd_aux)>>8, 500);

		slogic_usb_control_read(sdi, SLOGIC_BASIC_16_CONTROL_IN_REQ_REG_READ, SLOGIC_BASIC_16_R32_AUX, 0x0000, cmd_aux, 4 + (*(uint16_t*)cmd_aux)>>8, 500);
		sr_dbg("aux: %u %u %u %u %u.", cmd_aux[0], cmd_aux[1], cmd_aux[2], cmd_aux[3], *(uint64_t*)(cmd_aux+4));

		if (devc->cur_samplerate != *(uint64_t*)(cmd_aux+4)) {
			sr_dbg("Failed to configure samplerate.");
		} else {
			sr_dbg("Succeed to configure samplerate.");
		}
	}

	return slogic_usb_control_write(sdi, SLOGIC_BASIC_16_CONTROL_OUT_REQ_REG_WRITE, SLOGIC_BASIC_16_R32_CTRL, 0x0000, ARRAY_AND_SIZE(cmd_run), 500);
}

static int slogic_basic_16_remote_stop(const struct sr_dev_inst *sdi) {
	const uint8_t cmd_rst[] = {0x02, 0x00, 0x00, 0x00};
	return slogic_usb_control_write(sdi, SLOGIC_BASIC_16_CONTROL_OUT_REQ_REG_WRITE, SLOGIC_BASIC_16_R32_CTRL, 0x0000, ARRAY_AND_SIZE(cmd_rst), 500);
}
// reverse bits（MSB <-> LSB）
static uint8_t reverse_bits(uint8_t x) {
    x = ((x & 0x55) << 1) | ((x >> 1) & 0x55); // 交换相邻的1位
    x = ((x & 0x33) << 2) | ((x >> 2) & 0x33); // 交换相邻的2位
    x = ((x & 0x0F) << 4) | ((x >> 4) & 0x0F); // 交换相邻的4位
    return x;
}

static void transpose_8x8_swar(uint8_t A[8], int n, uint8_t B[8]) {
    // 将 8x8 矩阵的每一行加载到 64 位寄存器中
    uint64_t src = 0;
    for (int i = 0; i < n; i++) {
        src |= (uint64_t)reverse_bits(A[i]) << (8 * i);
    }

    // SWAR 分阶段转置
    src = (src & 0xAA55AA55AA55AA55) | ((src & 0x00AA00AA00AA00AA) << 7) | ((src >> 7) & 0x00AA00AA00AA00AA);
    src = (src & 0xCCCC3333CCCC3333) | ((src & 0x0000CCCC0000CCCC) << 14) | ((src >> 14) & 0x0000CCCC0000CCCC);
    src = (src & 0xF0F0F0F00F0F0F0F) | ((src & 0x00000000F0F0F0F0) << 28) | ((src >> 28) & 0x00000000F0F0F0F0);

    // 将转置后的结果存储到 B 数组中
    for (int i = 0; i < 8; i++) {
        B[i] = (src >> (8 * i)) & 0xFF;
    }
}

static void slogic_basic_16_submit_raw_data(void *data, size_t len, const struct sr_dev_inst *sdi) {
	struct dev_context *devc = sdi->priv;

	size_t length = len * ((devc->cur_samplechannel>=8)?:(8/devc->cur_samplechannel));
	uint8_t *ptr = g_malloc(length);

	for(size_t i=0; i<len; i+=devc->cur_samplechannel) {
		if (devc->cur_samplechannel <= 8) {
			transpose_8x8_swar(data + i, devc->cur_samplechannel, ptr + i / devc->cur_samplechannel * 8);
			continue;
		}
		for(size_t j=0; j<8; j++) {
			#define B(n) (((((uint8_t *)data)[i+(n)] >> 7-j) & 0x1) << ((n)%8))
			switch (devc->cur_samplechannel) {
			case 16:
				ptr[i+j*2+0] =
					B(0)|B(1)|B(2)|B(3)|B(4)|B(5)|B(6)|B(7);
				ptr[i+j*2+1] =
					B(8)|B(9)|B(10)|B(11)|B(12)|B(13)|B(14)|B(15);
			break;
			case 8:
				ptr[i+j] =
					B(0)|B(1)|B(2)|B(3)|B(4)|B(5)|B(6)|B(7);
			break;
			case 4:
				ptr[i*2+j] =
					B(0)|B(1)|B(2)|B(3);
			break;
			case 2:
				ptr[i*4+j] =
					B(0)|B(1);
			break;
			}
			#undef B
		}
	}

	sr_session_send(sdi, &(struct sr_datafeed_packet) {
		.type = SR_DF_LOGIC,
		.payload = &(struct sr_datafeed_logic) {
			.length = length,
			.unitsize = (devc->cur_samplechannel + 7)/8,
			.data = ptr,
		}
	});

	g_free(ptr);
}
/* Slogic Basic 16 end */

static int slogic_usb_control_write(const struct sr_dev_inst *sdi, uint8_t request, uint16_t value, uint16_t index, uint8_t *data, size_t len, int timeout)
{
	int ret;
	struct dev_context *devc;
	struct sr_usb_dev_inst *usb;

	devc = sdi->priv;
	usb  = sdi->conn;

	sr_spew("%s req:%u value:%u index:%u %p:%u in %dms.", __func__, request, value, index, data, len, timeout);
	if (!data && len) {
		sr_warn("%s Nothing to write although len(%u)>0!", __func__, len);
		len = 0;
	}

	if ((ret = libusb_control_transfer(
		usb->devhdl, LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_OUT,
		request,
		value, index,
		(unsigned char *)data, len,
		timeout
	)) < 0) {
		sr_err("%s failed(libusb: %s)!", __func__, libusb_error_name(ret));
		return SR_ERR_NA;
	}
	return ret;
}


static int slogic_usb_control_read(const struct sr_dev_inst *sdi, uint8_t request, uint16_t value, uint16_t index, uint8_t *data, size_t len, int timeout)
{
	int ret;
	struct dev_context *devc;
	struct sr_usb_dev_inst *usb;

	devc = sdi->priv;
	usb  = sdi->conn;

	sr_spew("%s req:%u value:%u index:%u %p:%u in %dms.", __func__, request, value, index, data, len, timeout);
	if (!data && len) {
		sr_err("%s Can't read to NULL while len(%u)>0!", __func__, len);
		return SR_ERR_ARG;
	}

	if ((ret = libusb_control_transfer(
		usb->devhdl, LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_IN,
		request,
		value, index,
		(unsigned char *)data, len,
		timeout
	)) < 0) {
		sr_err("%s failed(libusb: %s)!", __func__, libusb_error_name(ret));
		return SR_ERR_NA;
	}
	return ret;
}