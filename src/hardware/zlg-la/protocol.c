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
static int zlg_la_set_samplerate_ratio(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc = sdi->priv;
	uint8_t payload[14] = {0};
	uint32_t divider = 0;

	uint32_t ratio = devc->product->max_sample_depth * 1024 * devc->capture_ratio / 100;

	/* Replicating pcap: 05 fa 02 83 04 09 ... divider ratio */
	payload[0] = 0x02;
	payload[1] = 0x83;
	payload[2] = 0x04;
	payload[3] = 0x09;
	WL32(&(payload[10]), ratio);
	if (devc->cur_samplerate != SR_MHZ(devc->product->max_samplerate)) {
		/* Formula: Divider = (100MHz / Rate) - 1 */
		divider = (uint32_t)(100000000 / devc->cur_samplerate) - 1;
		payload[5] = 0x1;	// enable divider
		WL32(&(payload[6]), divider);
	}

	sr_info("Setting samplerate to %" PRIu64 " Hz (Divider: %u)", devc->cur_samplerate, divider);
	
	if (zlg_la_cmd(sdi, CMD_SET_EXEC, payload, 14, NULL, NULL) != SR_OK) {
		return SR_ERR;
	}

	return SR_OK;
}

/**
 * Configures the FPGA trigger engine.
 * Replicates sub_100028E0.
 */
static int zlg_la_build_and_send_stages(const struct sr_dev_inst *sdi, 
    uint32_t m_lvl, uint32_t v_lvl, uint32_t m_edge, uint32_t v_e_start, uint32_t v_e_end, gboolean has_edge)
{
    uint8_t buf[96] = {0};
    int num_stages = has_edge ? 3 : 2;

    /* STAGE 1: Static levels AND the starting state of the edge */
    buf[0] = 0x01;
    WL32(&buf[4], m_lvl | m_edge);
    WL32(&buf[8], v_lvl | v_e_start);
    buf[12] = 0x03; // Match enabled
    buf[17] = 0x18; // Pre-trigger ratio

    if (has_edge) {
        /* STAGE 2: Static levels AND the final state of the edge */
        buf[32] = 0x02;
        buf[32+2] = 0x01; // Transition Mode
        WL32(&buf[32+4], m_lvl | m_edge);
        WL32(&buf[32+8], v_lvl | v_e_end);
        buf[32+12] = 0x03;
        buf[32+17] = 0x18;

        /* STAGE 3: Level finalized */
        buf[64] = 0x02;
        buf[64+2] = 0x10; // Level Mode
        buf[64+17] = 0x18;
    } else {
        /* Level-only capture just needs a sustain stage */
        buf[32] = 0x01; // Immediate/Level
        buf[32+2] = 0x10;
        buf[32+17] = 0x10;
    }
}

static int zlg_la_send_trigger_blocks(const struct sr_dev_inst *sdi, uint8_t *data, int len) {
	struct sr_usb_dev_inst *usb = sdi->conn;
	uint8_t payload[14] = {0};
	int ret, transferred;

	sr_dbg("Configuring trigger...");

	/* 1. Tell device to expect 32 bytes of trigger data */
	payload[0] = 0x02;
	payload[1] = 0x00;
	payload[2] = 0x06;
	payload[3] = len;
	
	if (zlg_la_cmd(sdi, CMD_SET_TRIG, payload, 14, NULL, NULL) != SR_OK)
		return SR_ERR;

	GString *hex_str = sr_hexdump_new(data, len);
	sr_dbg("trigger: %s", hex_str->str);
	ret = libusb_bulk_transfer(usb->devhdl, 0x02, data, len, &transferred, 100);
	if (ret < 0 || transferred != len) {
		sr_err("send trigger failed, ret = %s, transferred = %d/%d", libusb_error_name(ret), transferred, len);
		return SR_ERR;
	}
	return SR_OK;
}
static int zlg_la_set_trigger(const struct sr_dev_inst *sdi)
{
    struct sr_trigger *trigger;
    struct sr_trigger_stage *stage;
    struct sr_trigger_match *match;
    GSList *l, *m;
	uint8_t buf[96] = {0};

    uint32_t mask_lvl = 0, value_lvl = 0;
    uint32_t mask_edge = 0, value_edge_start = 0, value_edge_end = 0;
    gboolean has_edge = FALSE;
	
    if (!(trigger = sr_session_trigger_get(sdi->session))) {
        /* --- CASE 1: IMMEDIATE TRIGGER (Baseline Dump) --- */
        /* Block 1 */
        buf[0] = 0x01; buf[1] = 0x01; buf[2] = 0x10; buf[3] = 0x10;
        buf[12] = 0x00; buf[17] = 0x18;
        /* Block 2 */
        buf[32] = 0x01; buf[32+1] = 0x01; buf[32+2] = 0x10; buf[32+3] = 0x10;
        buf[32+12] = 0x00; buf[32+17] = 0x10;
        
        return zlg_la_send_trigger_blocks(sdi, buf, 64);
    }

    /* We only support the first sigrok trigger stage */
    stage = trigger->stages->data; 

    for (m = stage->matches; m; m = m->next) {
        match = m->data;
        if (!match->channel->enabled) continue;

        uint32_t bit = (1 << match->channel->index);

        if (match->match == SR_TRIGGER_ONE) {
            mask_lvl |= bit;
            value_lvl |= bit;
        } else if (match->match == SR_TRIGGER_ZERO) {
            mask_lvl |= bit;
        } else if (match->match == SR_TRIGGER_RISING) {
            has_edge = TRUE;
            mask_edge |= bit;
            value_edge_end |= bit;   /* Ends High */
        } else if (match->match == SR_TRIGGER_FALLING) {
            has_edge = TRUE;
            mask_edge |= bit;
            value_edge_start |= bit; /* Starts High */
        }
    }

    if (has_edge) {
        /* --- CASE 2: EDGE TRIGGER (3 Stages) --- */
        /* STAGE 1: Look for the "Before" condition */
        buf[0] = 0x01;
        buf[1] = 0x00; buf[2] = 0x00; buf[3] = 0x00; // Edge Prep mode
        WL32(&buf[4], mask_lvl | mask_edge);
        WL32(&buf[8], value_lvl | value_edge_start);
        buf[12] = 0x03; buf[17] = 0x18;

        /* STAGE 2: Transition Condition */
        buf[32] = 0x02;
        buf[32+1] = 0x01; buf[32+2] = 0x10; buf[32+3] = 0x00; // Edge Transition mode
        WL32(&buf[32+4], mask_lvl | mask_edge);
        WL32(&buf[32+8], value_lvl | value_edge_end);
        buf[32+12] = 0x03; buf[32+17] = 0x18;

        /* STAGE 3: Finalizer (Level Match) */
        buf[64] = 0x02;
        buf[64+1] = 0x02; buf[64+2] = 0x10; buf[64+3] = 0x10; // Finalize mode
        // WL32(&buf[64+4], mask_lvl | mask_edge);
        // WL32(&buf[64+8], value_lvl | value_edge_end);
        buf[64+12] = 0x03; buf[64+17] = 0x18;

        return zlg_la_send_trigger_blocks(sdi, buf, 96);

    } else {
        /* --- CASE 3: LEVEL/BUS TRIGGER (2 Stages) --- */
        /* STAGE 1: Compare against pattern */
        buf[0] = 0x01;
        buf[1] = 0x00; buf[2] = 0x10; buf[3] = 0x00; // Level Match mode
        WL32(&buf[4], mask_lvl);
        WL32(&buf[8], value_lvl);
        buf[12] = 0x03; buf[17] = 0x18;

        /* STAGE 2: Terminal State */
        buf[32] = 0x01;
        buf[32+1] = 0x01; buf[32+2] = 0x10; buf[32+3] = 0x10; // Sustain mode
        buf[32+12] = 0x00; buf[32+17] = 0x18;

        return zlg_la_send_trigger_blocks(sdi, buf, 64);
    }
}

// SR_PRIV int zlg_la_set_trigger(const struct sr_dev_inst *sdi)
// {
	// struct dev_context *devc = sdi->priv;
	// uint8_t payload[14] = {0};
	// uint8_t pattern[96] = {0};	// 64 for one shot; 96 for pin trigger;
	// int transferred;
// 
	// sr_dbg("Configuring trigger...");
// 
	// /* 
	//  * TODO: Use the trigger_mask/value/edge from the session to fill
	//  * the 32-byte pattern. For now, we use a simple "Always Trigger" 
	//  * or "Manual Trigger" pattern seen in your pcap.
	//  */
	// pattern[0] = 0x01;
	// pattern[1] = 0x01;
	// pattern[2] = 0x10;
	// pattern[3] = 0x10;
	// pattern[17] = 0x18;
	// pattern[32] = 0x01;
	// pattern[33] = 0x01;
	// pattern[34] = 0x10;
	// pattern[35] = 0x10;
	// pattern[49] = 0x10;
	// /* ... fill remaining based on your pcap BULK_OUT 01011010... */
// 
	// /* 1. Tell device to expect 32 bytes of trigger data */
	// payload[0] = 0x02;
	// payload[1] = 0x00;
	// payload[2] = 0x06;
	// payload[3] = 0x20;	// 0x40->one shot; 0x66 trigger
	// 
	// if (zlg_la_cmd(sdi, CMD_SET_TRIG, payload, 14, NULL, NULL) != SR_OK)
		// return SR_ERR;
// 
	// /* 2. Send the 32-byte pattern over Pipe 2 (Endpoint 0x02 OUT) */
	// struct sr_usb_dev_inst *usb = sdi->conn;
	// libusb_bulk_transfer(usb->devhdl, 0x02, pattern, 64, &transferred, 100);
// 
	// return SR_OK;
// }

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
	ret = zlg_la_cmd(sdi, CMD_GET_DEVICE_INFO, NULL, 0, rsp, &rsp_len);
	if (ret != SR_OK) {
		return ret;
	}
	if (rsp[4] == 0xff || rsp[5] == 0xff) {
		sr_err("device not ready");
		return SR_ERR;
	}

	// set freq
	ret = zlg_la_set_samplerate_ratio(sdi);
	if (ret != SR_OK) {
		return ret;
	}

	// commit
	payload[0] = 0xff; payload[1] = 0xff; payload[2] = 0xff; payload[3] = 0xff;
	ret = zlg_la_cmd(sdi, CMD_COMMIT, payload, 4, rsp, &rsp_len);
	if (ret != SR_OK) {
		return ret;
	}
	if (rsp[0] != 0x03 || rsp[1] != 0xfc) {
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
		ret = zlg_la_cmd(sdi, CMD_SET_EXEC, payload, 6, NULL, NULL);
		if (ret != SR_OK) {
			return ret;
		}
	}

	return SR_OK;
}

/* Process the raw 98312 bytes logic data */
static int zlg_la_receive_data(const struct sr_dev_inst *sdi, gboolean drain)
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
	uint8_t *out_buffer;

	int pin_data_len = 32 * 1024 * 16 / 8;

	// stop capture 05fa 02800001
	payload[0] = 0x02; payload[1] = 0x80; payload[2] = 0x00; payload[3] = 0x01;
	ret = zlg_la_cmd(sdi, CMD_SET_EXEC, payload, 4, NULL, NULL);
	if (ret != SR_OK) {
		return ret;
	}

	// get data size 0df2 0609010101
	payload[0] = 0x06; payload[1] = 0x09; payload[2] = 0x01; payload[3] = 0x01; payload[4] = 0x01;
	ret = zlg_la_cmd(sdi, CMD_GET_SIZE, payload, 5, rsp, &rsp_len);
	if (ret != SR_OK) {
		return ret;
	}
	uint32_t data_len = RL32(&rsp[2]);

	sr_dbg("Receiving %u bytes of data...", data_len);

	in_buffer = g_malloc(data_len);
	
	/* Blocking read for now (TODO: Change to async later) */
	// we must read all the data from device
	ret = libusb_bulk_transfer(usb->devhdl, 0x82, in_buffer, data_len, &transferred, 2000);
	if (ret < 0) {
		sr_err("Receive failed: %s, transferred: %d", libusb_error_name(ret), transferred);
	}

	// cleanup
	payload[0] = 0x02; payload[1] = 0x80; payload[2] = 0x00; payload[3] = 0x01;
	ret = zlg_la_cmd(sdi, CMD_GET_STATE, payload, 4, rsp, &rsp_len);
	if (ret != SR_OK) {
		return ret;
	}
	payload[0] = 0x02; payload[1] = 0x88; payload[2] = 0x04; payload[3] = 0x0c;
	ret = zlg_la_cmd(sdi, CMD_GET_STATE, payload, 4, rsp, &rsp_len);
	if (ret != SR_OK) {
		return ret;
	}
	ret = zlg_la_cmd(sdi, CMD_GET_DEVICE_INFO, NULL, 0, rsp, &rsp_len);
	if (ret != SR_OK) {
		return ret;
	}

	sr_dbg("Received %u bytes of data", transferred);
	if (drain) {
		g_free(in_buffer);
		return SR_OK;
	}
	/*
	  la1016 data depth: 32k bits per channel, 16 channels.
      32 * 1024 / 8 32k bytes per channel  x 16 channels = 65536 bytes (0xFFFF)
      also, 2 bytes per sample x 32768 samples = 65536 bytes

      usb raw data: 3 bytes per sample x 32768 samples = 98304 bytes + 8 extra bytes (3 samples) = 98312
	*/
	// remote the extra byte
	out_buffer = g_malloc(pin_data_len);
	int out_buffer_idx = 0;
	for (int i = 0; i < 3 * devc->product->max_sample_depth * 1024; i++) {
		if ((i + 1) % 3 == 0) {
			continue;
		}
		out_buffer[out_buffer_idx] = in_buffer[i];
		out_buffer_idx += 1;
		if (out_buffer_idx > (devc->limit_samples * 2)) {
			break;
		}
	}
	if (transferred > 0) {
		packet.type = SR_DF_LOGIC;
		packet.payload = &logic;
		logic.length = devc->limit_samples * 2;
		logic.unitsize = 2; /* 16 channels */
		logic.data = out_buffer;
		sr_session_send(sdi, &packet);
	}

	g_free(in_buffer);
	g_free(out_buffer);
	
	std_session_send_df_end(sdi);

	sr_session_source_remove(sdi->session, -1);

	return SR_OK;
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
	uint8_t status = 0;
	int rsp_len = 0;

	if (devc->state == STATE_CAPTURE) {
		if (zlg_la_start_capture(sdi) != SR_OK) {
			devc->state = STATE_IDLE;
		} else {
			devc->state = STATE_WAITING;
		}
		return G_SOURCE_CONTINUE;
	}

	/* 1. Send F906 (status) 02800402 - check if triggered */
	payload[0] = 0x02; payload[1] = 0x80; payload[2] = 0x04; payload[3] = 0x02;
	ret = zlg_la_cmd(sdi, CMD_GET_STATE, payload, 4, rsp, &rsp_len);
	if (ret != SR_OK) {
		return G_SOURCE_CONTINUE;
	}
	status = rsp[0];
	ret = zlg_la_cmd(sdi, CMD_GET_BULKIN, NULL, 0, rsp, &rsp_len);
	if (ret != SR_OK) {
		return G_SOURCE_CONTINUE;
	}
	if (rsp[2] != 0) {
		// drain the data
		ret = libusb_bulk_transfer(usb->devhdl, 0x82, sink_buffer, 0xff, &rsp_len, 100);
		sr_spew("cmd 0cf3 len:%x data: %02x%02x%02x%02x", rsp[2], sink_buffer[0],sink_buffer[1],sink_buffer[2],sink_buffer[3]);
		if (ret != SR_OK) {
			return G_SOURCE_CONTINUE;
		}
	}

	if (devc->state == STATE_WAITING && !(status & 0x02) && (status & 0x08)  ) {
		// trigger armed
		ret = zlg_la_receive_data(sdi, FALSE);
		devc->state = STATE_IDLE;
		return G_SOURCE_CONTINUE;
	}

	if ((status & 0x02) && devc->state == STATE_IDLE) {
		// waiting data and user request stop
		ret = zlg_la_receive_data(sdi, FALSE);
	}

    return G_SOURCE_CONTINUE;
}

SR_PRIV int test_work(int fd, int revents, void *cb_data) {
	const struct sr_dev_inst *sdi = cb_data;
    struct dev_context *devc = sdi->priv;
	struct sr_usb_dev_inst *usb = sdi->conn;
	int ret = SR_OK;
	uint8_t payload[16] = {0};
	uint8_t rsp[16] = {0};
	uint8_t status = 0;
	int rsp_len = 0;

	// check device
	ret = zlg_la_cmd(sdi, CMD_GET_DEVICE_INFO, NULL, 0, rsp, &rsp_len);
	if (ret != SR_OK || rsp[4] == 0xff || rsp[5] == 0xff) {
		sr_err("Device fw not working");
		return FALSE;
	}

	// check state
	payload[0] = 0x02; payload[1] = 0x80; payload[2] = 0x04; payload[3] = 0x02;
	ret = zlg_la_cmd(sdi, CMD_GET_STATE, payload, 4, rsp, &rsp_len);
	if (ret != SR_OK || rsp[0] == 0x2a) {
		sr_err("Device not idle");
		return FALSE;
	}

	// drain residual data
	zlg_la_receive_data(sdi, TRUE);

	// start
	zlg_la_start_capture(sdi);

	while(1) {
		payload[0] = 0x02; payload[1] = 0x80; payload[2] = 0x04; payload[3] = 0x02;
		ret = zlg_la_cmd(sdi, CMD_GET_STATE, payload, 4, rsp, &rsp_len);
		if (ret != SR_OK) {
			return FALSE;
		}
		status = rsp[0];
		ret = zlg_la_cmd(sdi, CMD_GET_BULKIN, NULL, 0, rsp, &rsp_len);
		if (ret != SR_OK) {
			return FALSE;
		}
		if (rsp[2] != 0) {
			// drain the data
			ret = libusb_bulk_transfer(usb->devhdl, 0x82, sink_buffer, 0xff, &rsp_len, 100);
			sr_spew("cmd 0cf3 len:%x data: %02x%02x%02x%02x", rsp[2], sink_buffer[0],sink_buffer[1],sink_buffer[2],sink_buffer[3]);
			if (ret != SR_OK) {
				return FALSE;
			}
		}

		if (!(status & 0x02) && (status & 0x08)) {
			// trigger armed
			ret = zlg_la_receive_data(sdi, FALSE);
			devc->state = STATE_IDLE;
			return FALSE;
		}

		g_usleep(200*1000);	//1MHz 32k samples ~ 32ms
	}
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
	ret = zlg_la_cmd(sdi, CMD_GET_DEVICE_INFO, NULL, 0, rsp, &rsp_len);
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
		ret = sr_resource_read(drvc->sr_ctx, &bitstream, buffer, chunk_size);
		if (ret != SR_OK) {
			sr_err("sr_resource_read failed %d", ret);
			break;
		}

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