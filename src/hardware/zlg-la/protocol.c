#include <config.h>
#include "protocol.h"

#include <fcntl.h>

static int zlg_la_cmd_write(const struct sr_dev_inst *sdi, uint8_t cmd_id, uint8_t *payload, size_t len) {
	struct sr_usb_dev_inst *usb = sdi->conn;
	uint8_t cmd[16] = {0};
	int transferred, ret;

	cmd[0] = cmd_id;
	cmd[1] = (uint8_t)~cmd_id;

	if (payload && len > 0)
		memcpy(&cmd[2], payload, (len > 14) ? 14 : len);

	sr_dbg("%s:%d: cmd %02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x", __func__, __LINE__,
		cmd[0], cmd[1], cmd[2], cmd[3], cmd[4], cmd[5], cmd[6], cmd[7],
		cmd[8], cmd[9], cmd[10],cmd[11],cmd[12],cmd[13],cmd[14],cmd[15]
	);

	ret = libusb_interrupt_transfer(usb->devhdl, 0x01, cmd, 16, &transferred, 100);
	if (ret < 0 || transferred != 16) {
		sr_err("Cmd 0x%02x write failed: %s", cmd_id, libusb_error_name(ret));
		return SR_ERR;
	}

	return SR_OK;
}

static int zlg_la_cmd_read(const struct sr_dev_inst *sdi, uint8_t *rsp, int* len)
{
	struct sr_usb_dev_inst *usb = sdi->conn;
	int ret;

	ret = libusb_interrupt_transfer(usb->devhdl, 0x81, rsp, 16, len, 100);
	if (ret < 0 || *len == 0) {
		sr_err("Response read failed: %s, transfered:%d", libusb_error_name(ret), *len);
		return SR_ERR;
	}
	return SR_OK;
}

static int zlg_la_cmd(const struct sr_dev_inst *sdi, uint8_t cmd_id, uint8_t *payload, size_t len, uint8_t *rsp, int *rsp_len) {
	int ret;
	ret = zlg_la_cmd_write(sdi, cmd_id, payload, len);
	if (ret != SR_OK) {
		return ret;
	}
	if (rsp && rsp_len) {
		ret = zlg_la_cmd_read(sdi, rsp, rsp_len);
		sr_dbg("%s:%d: cmd %02x rsp %02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x", __func__, __LINE__, cmd_id,
			rsp[0], rsp[1], rsp[2], rsp[3], rsp[4], rsp[5], rsp[6], rsp[7],
			rsp[8], rsp[9], rsp[10],rsp[11],rsp[12],rsp[13],rsp[14],rsp[15]
		);
	}

	return ret;
}

/* Replicates sub_100031C0: 100MHz Divider Logic */
SR_PRIV int zlg_la_set_samplerate(const struct sr_dev_inst *sdi, uint64_t samplerate)
{
	uint8_t payload[14] = {0};
	uint32_t divider;

	/* Formula: Divider = (100MHz / Rate) - 1 */
	divider = (uint32_t)(100000000 / samplerate) - 1;

	/* Replicating pcap: 05 fa 02 83 04 09 ... divider at offset 10 */
	payload[0] = 0x02;
	payload[1] = 0x83;
	payload[2] = 0x04;
	payload[3] = 0x09;
	payload[8] = divider & 0xFF;
	payload[9] = (divider >> 8) & 0xFF;
	payload[10] = (divider >> 16) & 0xFF;
	payload[11] = (divider >> 24) & 0xFF;

	sr_info("Setting samplerate to %" PRIu64 " Hz (Divider: %u)", samplerate, divider);
	
	if (zlg_la_cmd(sdi, CMD_SET_FREQ, payload, 14, NULL, NULL) != SR_OK) {
		return SR_ERR;
	}

	return SR_OK;
}

/**
 * Configures the FPGA trigger engine.
 * Replicates sub_100028E0.
 */
SR_PRIV int zlg_la_set_trigger(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc = sdi->priv;
	uint8_t payload[14] = {0};
	uint8_t pattern[32] = {0};
	int transferred;

	sr_dbg("Configuring trigger...");

	/* 
	 * TODO: Use the trigger_mask/value/edge from the session to fill
	 * the 32-byte pattern. For now, we use a simple "Always Trigger" 
	 * or "Manual Trigger" pattern seen in your pcap.
	 */
	pattern[0] = 0x01;
	pattern[1] = 0x01;
	pattern[2] = 0x10;
	pattern[3] = 0x1a; 
	/* ... fill remaining based on your pcap BULK_OUT 01011010... */

	/* 1. Tell device to expect 32 bytes of trigger data */
	payload[0] = 0x02;
	payload[1] = 0x00;
	payload[2] = 0x06;
	payload[3] = 0x40;
	payload[10] = 32; /* Length of the following bulk out */
	
	if (zlg_la_cmd(sdi, CMD_SET_TRIG, payload, 14, NULL, NULL) != SR_OK)
		return SR_ERR;

	/* 2. Send the 32-byte pattern over Pipe 2 (Endpoint 0x02 OUT) */
	struct sr_usb_dev_inst *usb = sdi->conn;
	libusb_bulk_transfer(usb->devhdl, 0x02, pattern, 32, &transferred, 100);

	return SR_OK;
}

static int zlg_la_start_capture(struct sr_dev_inst *sdi) {
	int ret = SR_OK;
	uint8_t payload[16] = {0};
	uint8_t rsp[16] = {0};
	int rsp_len = 0;

	// set trigger
	ret = zlg_la_set_trigger(sdi);
	if (ret != SR_OK) {
		return ret;
	}

	// check device state
	ret = zlg_la_cmd(sdi, CMD_GET_STATUS, NULL, 0, rsp, &rsp_len);
	if (ret != SR_OK) {
		return ret;
	}
	if (rsp[4] == 0xff || rsp[5] == 0xff) {
		sr_err("device not ready");
		return SR_ERR;
	}

	// set freq
	ret = zlg_la_set_samplerate(sdi, 100);
	if (ret != SR_OK) {
		return ret;
	}

	// commit
	payload[0] = 0xff; payload[1] = 0xff; payload[2] = 0xff; payload[3] = 0xff;
	ret = zlg_la_cmd(sdi, CMD_COMMIT, payload, 4, rsp, &rsp_len);
	if (ret != SR_OK) {
		return ret;
	}
	if (rsp[0] != 0x03 || rsp[1] == 0xfc) {
		sr_err("commit failed");
		return SR_ERR;
	}

	// start capture
	//028000010001, 028000010000, 028000010002
	payload[0] = 0x02; payload[1] = 0x80; payload[2] = 0x00; payload[3] = 0x01; payload[4] = 0x00;
	for (int i = 0; i < 3; i++) {
		switch (i) {
			case 0:
			payload[5] = 0x01;
			break;
			case 1:
			payload[5] = 0x00;
			break;
			case 2:
			payload[5] = 0x02;
			break;
		}
		ret = zlg_la_cmd(sdi, CMD_SET_FREQ, payload, 6, NULL, NULL);
		if (ret != SR_OK) {
			return ret;
		}
	}

	return SR_OK;
}

/* Process the raw 98312 bytes logic data */
SR_PRIV int zlg_la_receive_data(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc = sdi->priv;
	struct sr_usb_dev_inst *usb = sdi->conn;
	struct sr_datafeed_packet packet;
	struct sr_datafeed_logic logic;
	int ret = SR_OK;
	int transferred = 0;
	uint8_t payload[16] = {0};
	uint8_t rsp[16] = {0};
	int rsp_len = 0;
	uint8_t *in_buffer;

	// stop capture 05fa 02800001
	payload[0] = 0x02; payload[1] = 0x80; payload[2] = 0x00; payload[3] = 0x01;
	ret = zlg_la_cmd(sdi, CMD_SET_FREQ, payload, 4, NULL, NULL);
	if (ret != SR_OK) {
		return ret;
	}

	// get data size 0df2 0609010101
	payload[0] = 0x06; payload[1] = 0x09; payload[2] = 0x01; payload[3] = 0x01; payload[4] = 0x01;
	ret = zlg_la_cmd(sdi, CMD_SET_FREQ, payload, 5, rsp, &rsp_len);
	if (ret != SR_OK) {
		return ret;
	}
	uint32_t data_len = RL32(&rsp[2]);

	sr_dbg("Receiving %u bytes of data...", data_len);

	in_buffer = g_malloc(data_len);
	
	/* Blocking read for now (TODO: Change to async later) */
	ret = libusb_bulk_transfer(usb->devhdl, 0x82, in_buffer, 
				data_len, &transferred, 5000);

	// cleanup
	payload[0] = 0x02; payload[1] = 0x80; payload[2] = 0x00; payload[3] = 0x01;
	ret = zlg_la_cmd(sdi, CMD_TELEMETRY, payload, 4, rsp, &rsp_len);
	if (ret != SR_OK) {
		return TRUE;
	}
	payload[0] = 0x02; payload[1] = 0x88; payload[2] = 0x04; payload[3] = 0x0c;
	ret = zlg_la_cmd(sdi, CMD_TELEMETRY, payload, 4, rsp, &rsp_len);
	if (ret != SR_OK) {
		return TRUE;
	}
	ret = zlg_la_cmd(sdi, CMD_GET_STATUS, NULL, 0, rsp, &rsp_len);
	if (ret != SR_OK) {
		return ret;
	}

	if (ret == 0 && transferred > 0) {
		packet.type = SR_DF_LOGIC;
		packet.payload = &logic;
		logic.length = transferred;
		logic.unitsize = 2; /* 16 channels */
		logic.data = in_buffer;
		sr_session_send(sdi, &packet);
	}

	g_free(in_buffer);
	
	std_session_send_df_end(sdi);

	return TRUE;
}

// return G_SOURCE_CONTINUE to continue the loop
static uint8_t sink_buffer[0xff];
SR_PRIV gboolean zlg_la_work_loop(gpointer user_data) {
// SR_PRIV int zlg_la_work_loop(int fd, int revents, void *cb_data) {
    struct sr_dev_inst *sdi = user_data;
    struct dev_context *devc = sdi->priv;
	struct sr_usb_dev_inst *usb = sdi->conn;
	int ret = SR_OK;
	uint8_t payload[16] = {0};
	uint8_t rsp[16] = {0};
	uint8_t telemetry = 0;
	int rsp_len = 0;

	if (devc->state == STATE_CAPTURE) {
		if (zlg_la_start_capture(sdi) != SR_OK) {
			devc->state = STATE_IDLE;
		} else {
			devc->state = STATE_WAITING;
		}
		return G_SOURCE_CONTINUE;
	}

	/* 1. Send F906 (Telemetry) 02800402 - check if triggered */
	payload[0] = 0x02; payload[1] = 0x80; payload[2] = 0x04; payload[3] = 0x02;
	ret = zlg_la_cmd(sdi, CMD_TELEMETRY, payload, 4, rsp, &rsp_len);
	if (ret != SR_OK) {
		return G_SOURCE_CONTINUE;
	}
	telemetry = rsp[2];
	ret = zlg_la_cmd(sdi, CMD_QUERY_BUF, NULL, 0, rsp, &rsp_len);
	if (ret != SR_OK) {
		return G_SOURCE_CONTINUE;
	}
	if (rsp[2] != 0) {
		// drain the data
		ret = libusb_bulk_transfer(usb->devhdl, 0x82, sink_buffer, 0xff, &rsp_len, 100);
		sr_spew("cmd 0cf3 len:%x data: %x%x%x%x", rsp[3], sink_buffer[0],sink_buffer[1],sink_buffer[2],sink_buffer[3]);
		if (ret != SR_OK) {
			return G_SOURCE_CONTINUE;
		}
	}

	if (devc->state == STATE_WAITING && telemetry == 0x28) {
		// trigger armed
		ret = zlg_la_receive_data(sdi);
		devc->state == STATE_IDLE;
		return G_SOURCE_CONTINUE;
	}

	if (telemetry == 0x26 && devc->state == STATE_IDLE) {
		// waiting data and user request stop
		ret = zlg_la_receive_data(sdi);
	}

    return G_SOURCE_CONTINUE;
}

SR_PRIV int zlg_la_fw_upload(const struct sr_dev_inst *sdi, const char *name)
{
	struct drv_context *drvc = sdi->driver->context;
	struct sr_usb_dev_inst *usb = sdi->conn;
	struct sr_resource bitstream;
	uint8_t payload[4];
	uint8_t rsp[16] = {0};
	int rsp_len;
	uint8_t buffer[512];
	int transferred, ret;
	size_t size, offset;

	// check device state
	ret = zlg_la_cmd(sdi, CMD_GET_STATUS, NULL, 0, rsp, &rsp_len);
	if (ret != SR_OK) {
		return ret;
	}
	if (rsp[4] != 0xff && rsp[5] != 0xff) {
		sr_err("skip fw uploading");
		return SR_OK;
	}

	sr_info("Uploading firmware '%s'...", name);

	/* 1. Open the firmware file using sigrok's resource loader */
	ret = sr_resource_open(drvc->sr_ctx, &bitstream, SR_RESOURCE_FIRMWARE, name);
	if (ret != SR_OK) {
		sr_err("Could not find firmware %s. Place it in ~/.local/share/sigrok-firmware/", name);
		return SR_ERR;
	}

	size = bitstream.size;

	/* 2. Send CMD_FW_UPLOAD (0x01 / FE01) with the size as payload */
	/* Payload is 4 bytes, Little Endian */
	WL32(payload, size);
	if (zlg_la_cmd(sdi, CMD_FW_UPLOAD, payload, 4, rsp, &rsp_len) != SR_OK) {
		sr_resource_close(drvc->sr_ctx, &bitstream);
		return SR_ERR;
	}
	if (RL16(rsp) != 0xFE01) {
		sr_err("Hardware rejected firmware upload command, 0x%x 0x%x", rsp[0], rsp[1]);
		sr_resource_close(drvc->sr_ctx, &bitstream);
		return SR_ERR;
	}

	/* 3. Read 16-byte acknowledgement (matches sub_100023F0 check for 65025) */
	// uint8_t rsp[16];
	// zlg_la_read_response(sdi, rsp);
	// if (RL16(rsp) != 0xFE01) {
	// 	sr_err("Hardware rejected firmware upload command, 0x%x 0x%x", rsp[0], rsp[1]);
	// 	sr_resource_close(drvc->sr_ctx, &bitstream);
	// 	return SR_ERR;
	// }

	/* 4. Stream the data in 512-byte blocks to Endpoint 0x02 */
	offset = 0;
	while (offset < size) {
		size_t chunk_size = MIN(size - offset, 512);
		sr_resource_read(drvc->sr_ctx, &bitstream, buffer, chunk_size);

		ret = libusb_bulk_transfer(usb->devhdl, 0x02, buffer, chunk_size, &transferred, 1000);
		if (ret < 0) {
			sr_err("Firmware transfer failed at offset %zu: %s", offset, libusb_error_name(ret));
			break;
		}
		offset += transferred;
		sr_spew("Uploaded %zu/%zu bytes", offset, size);
	}

	sr_resource_close(drvc->sr_ctx, &bitstream);

	if (offset < size) return SR_ERR;

	sr_info("Firmware upload complete.");
	return SR_OK;
}