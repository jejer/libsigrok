#include <config.h>
#include "protocol.h"

#include <fcntl.h>

/* Replicates sub_10001130: Packs the 16-byte command wrapper */
SR_PRIV int zlg_la_transmit(const struct sr_dev_inst *sdi, uint8_t cmd_id, uint8_t *payload, size_t len)
{
	struct sr_usb_dev_inst *usb = sdi->conn;
	uint8_t cmd[16] = {0};
	int transferred, ret;

	cmd[0] = cmd_id;
	cmd[1] = (uint8_t)~cmd_id;

	if (payload && len > 0)
		memcpy(&cmd[2], payload, (len > 14) ? 14 : len);

	ret = libusb_bulk_transfer(usb->devhdl, 0x01, cmd, 16, &transferred, 100);
	if (ret < 0 || transferred != 16) {
		sr_err("Cmd 0x%02x write failed: %s", cmd_id, libusb_error_name(ret));
		return SR_ERR;
	}

	return SR_OK;
}

/* Helper to read the response from EP 0x81, size not fixed */
static int zlg_la_read_response(const struct sr_dev_inst *sdi, uint8_t *rsp)
{
	struct sr_usb_dev_inst *usb = sdi->conn;
	int transferred, ret;

	ret = libusb_bulk_transfer(usb->devhdl, 0x81, rsp, 16, &transferred, 100);
	if (ret < 0 || transferred == 0) {
		sr_err("Response read failed: %s, transfered:%d", libusb_error_name(ret), transferred);
		return SR_ERR;
	}
	return SR_OK;
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
	
	if (zlg_la_transmit(sdi, CMD_SET_FREQ, payload, 14) != SR_OK)
		return SR_ERR;

	/* FC03 Commit Ping */
	memset(payload, 0xff, 4);
	return zlg_la_transmit(sdi, CMD_COMMIT, payload, 4);
}

/* Replicates sub_10002F60: Start Capture and get Buffer Size */
SR_PRIV int zlg_la_setup_acquisition(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc = sdi->priv;
	uint8_t payload[14] = {0};
	uint8_t rsp[16];

	/* Build F20D payload from pcap: 06 09 01 01 01 */
	payload[0] = 0x06;
	payload[1] = 0x09;
	payload[2] = 0x01;
	payload[3] = 0x01;
	payload[4] = 0x01;

	if (zlg_la_transmit(sdi, CMD_START_CAP, payload, 5) != SR_OK)
		return SR_ERR;

	/* Read back the 16-byte header to find data size */
	if (zlg_la_read_response(sdi, rsp) != SR_OK)
		return SR_ERR;

	/* pcap analysis: Byte 2-5 of response is the length (e.g., 0x18008) */
	devc->expected_bytes = RL32(&rsp[2]);
	sr_info("Hardware reports data size: %u bytes", devc->expected_bytes);

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
	
	if (zlg_la_transmit(sdi, CMD_SET_TRIG, payload, 14) != SR_OK)
		return SR_ERR;

	/* 2. Send the 32-byte pattern over Pipe 2 (Endpoint 0x02 OUT) */
	struct sr_usb_dev_inst *usb = sdi->conn;
	libusb_bulk_transfer(usb->devhdl, 0x02, pattern, 32, &transferred, 100);

	return SR_OK;
}

/**
 * The repeating Activity Poll (F906 + F30C).
 * Replicates the live data flow from your pcap.
 */
SR_PRIV int zlg_la_poll_activity(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc = sdi->priv;
	struct sr_usb_dev_inst *usb = sdi->conn;
	uint8_t rsp[16];
	uint8_t activity_data[255];
	int transferred;

	if (!g_mutex_trylock(&devc->usb_mutex))
		return SR_OK;

	/* 1. Send F906 (Telemetry) - check if triggered */
	zlg_la_transmit(sdi, CMD_TELEMETRY, NULL, 0);
	if (zlg_la_read_response(sdi, rsp) == SR_OK) {
		/* If Byte 0 is 0x28, hardware is armed. If 0x20, it's idle. */
		if (rsp[0] == 0x28) {
			/* Hardware is waiting for trigger. We might want to 
			 * stop the F30C poll here to give FPGA more bandwidth. */
		}
	}

	/* 2. Send F30C (Activity check) */
	zlg_la_transmit(sdi, CMD_QUERY_BUF, NULL, 0);
	if (zlg_la_read_response(sdi, rsp) == SR_OK) {
		/* Byte 2 of response is the length of activity data available */
		uint8_t len = rsp[2];
		if (len > 0) {
			/* 3. Read activity logic from Pipe 2 (EP 0x82) */
			libusb_bulk_transfer(usb->devhdl, 0x82, activity_data, len, &transferred, 50);
			/* TODO: Pass this to PulseView to blink the "Live" indicators */
		}
	}

	g_mutex_unlock(&devc->usb_mutex);
	return SR_OK;
}

/* Process the raw 98312 bytes logic data */
SR_PRIV int zlg_la_receive_data(int fd, int revents, void *cb_data)
{
	struct sr_dev_inst *sdi;
	struct dev_context *devc;
	struct sr_usb_dev_inst *usb;
	struct sr_datafeed_packet packet;
	struct sr_datafeed_logic logic;
	int transferred, ret;
	uint8_t *in_buffer;

	/* Suppress unused parameter warnings */
	(void)fd;
	(void)revents;

	if (!(sdi = cb_data))
		return TRUE;

	devc = sdi->priv;
	usb = sdi->conn;

	sr_dbg("Receiving %u bytes of data...", devc->expected_bytes);

	in_buffer = g_malloc(devc->expected_bytes);
	
	/* Blocking read for now (TODO: Change to async later) */
	ret = libusb_bulk_transfer(usb->devhdl, 0x82, in_buffer, 
				devc->expected_bytes, &transferred, 5000);

	if (ret == 0 && transferred > 0) {
		packet.type = SR_DF_LOGIC;
		packet.payload = &logic;
		logic.length = transferred;
		logic.unitsize = 2; /* 16 channels */
		logic.data = in_buffer;
		sr_session_send(sdi, &packet);
	}

	g_free(in_buffer);
	
	sr_dev_acquisition_stop(sdi);

	return TRUE;
}

SR_PRIV int zlg_la_fw_upload(const struct sr_dev_inst *sdi, const char *name)
{
	struct drv_context *drvc = sdi->driver->context;
	struct sr_usb_dev_inst *usb = sdi->conn;
	struct sr_resource bitstream;
	uint8_t payload[4];
	uint8_t buffer[512];
	int transferred, ret;
	size_t size, offset;

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
	if (zlg_la_transmit(sdi, CMD_FW_UPLOAD, payload, 4) != SR_OK) {
		sr_resource_close(drvc->sr_ctx, &bitstream);
		return SR_ERR;
	}

	/* 3. Read 16-byte acknowledgement (matches sub_100023F0 check for 65025) */
	uint8_t rsp[16];
	zlg_la_read_response(sdi, rsp);
	if (RL16(rsp) != 0xFE01) {
		sr_err("Hardware rejected firmware upload command, 0x%x 0x%x", rsp[0], rsp[1]);
		sr_resource_close(drvc->sr_ctx, &bitstream);
		return SR_ERR;
	}

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