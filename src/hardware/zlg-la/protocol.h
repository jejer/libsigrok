#ifndef LIBSIGROK_HARDWARE_ZLG_LA_PROTOCOL_H
#define LIBSIGROK_HARDWARE_ZLG_LA_PROTOCOL_H

#include <config.h>
#include <stdint.h>
#include <glib.h>
#include <libusb.h>
#include <libsigrok/libsigrok.h>
#include "libsigrok-internal.h"

#include "protocol.h"

#define LOG_PREFIX "zlg-la"

#define ZLG_VID 0x04cc
#define ZLG_PID 0x120e

#define CMD_FW_UPLOAD  0x01 /* FE01: Firmware/FPGA bitstream start */
#define CMD_GET_STATUS 0x02 /* FD02: Periodic status (every 3ms) */
#define CMD_COMMIT     0x03 /* FC03: Settings commit/ping */
#define CMD_SET_FREQ   0x05 /* FA05: Sample rate / divider */
#define CMD_TELEMETRY  0x06 /* F906: Battery, power, and signal flags */
#define CMD_SET_TRIG   0x07 /* F807: Trigger pattern/mask upload */
#define CMD_QUERY_BUF  0x0c /* F30C: Activity check (Live LEDs) */
#define CMD_START_CAP  0x0d /* F20D: Start logic acquisition */

#define ZLG_FW_NAME "zlg-la1016.bitstream"

struct dev_context {
	struct sr_sw_limits limits;
	uint64_t cur_samplerate;
	gboolean poll_running;
	GMutex usb_mutex;

	uint16_t trigger_mask;    /* Which channels are involved in the trigger */
    uint16_t trigger_value;   /* High or Low level */
    uint16_t trigger_edge;    /* Rising or Falling */

	uint32_t expected_bytes;
};

SR_PRIV int zlg_la_fw_upload(const struct sr_dev_inst *sdi, const char *name);
SR_PRIV int zlg_la_transmit(const struct sr_dev_inst *sdi, uint8_t cmd_id, uint8_t *payload, size_t len);
SR_PRIV int zlg_la_poll_activity(const struct sr_dev_inst *sdi);
SR_PRIV int zlg_la_setup_acquisition(const struct sr_dev_inst *sdi);
SR_PRIV int zlg_la_receive_data(int fd, int revents, void *cb_data);
SR_PRIV int zlg_la_set_samplerate(const struct sr_dev_inst *sdi, uint64_t samplerate);
SR_PRIV int zlg_la_set_trigger(const struct sr_dev_inst *sdi);

#endif