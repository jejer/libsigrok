#include <config.h>
#include <libusb.h>
#include "protocol.h"

static const uint32_t scanopts[] = {
	SR_CONF_CONN,
};

static const uint32_t drvopts[] = {
	SR_CONF_LOGIC_ANALYZER,
};

static const uint32_t devopts[] = {
	SR_CONF_CONN,
	SR_CONF_SAMPLERATE,
	SR_CONF_LIMIT_SAMPLES,
};

static const uint64_t samplerates[] = {
	SR_HZ(1), SR_MHZ(100), SR_HZ(1),
};

static GSList *scan(struct sr_dev_driver *di, GSList *options)
{
	// printf("%s: %d %s\n", __FUNCTION__, __LINE__, di->name);
	struct drv_context *drvc = di->context;
	GSList *devices = NULL;
	struct sr_dev_inst *sdi;
	struct dev_context *devc;
	libusb_device **devlist;
	struct libusb_device_descriptor desc;
	int i, ret;

	(void)options;

	/* Get the list of all USB devices on the system */
	libusb_get_device_list(drvc->sr_ctx->libusb_ctx, &devlist);

	for (i = 0; devlist[i]; i++) {
		ret = libusb_get_device_descriptor(devlist[i], &desc);
		if (ret != 0)
			continue;

		/* Check if VID and PID match 04cc:120e */
		if (desc.idVendor != ZLG_VID || desc.idProduct != ZLG_PID)
			continue;

		sdi = g_malloc0(sizeof(struct sr_dev_inst));
		sdi->driver = di;
		sdi->status = SR_ST_INACTIVE;
		sdi->vendor = g_strdup("ZLG");
		sdi->model = g_strdup("LA1016");
		sdi->inst_type = SR_INST_USB;
		
		/* Initialize the USB connection instance with Bus and Address */
		sdi->conn = sr_usb_dev_inst_new(libusb_get_bus_number(devlist[i]),
						libusb_get_device_address(devlist[i]), NULL);

		devc = g_malloc0(sizeof(struct dev_context));
		sdi->priv = devc;
		
		g_mutex_init(&devc->usb_mutex);
		devc->cur_samplerate = SR_MHZ(1);

		for (int j = 0; j < 16; j++)
			sr_channel_new(sdi, j, SR_CHANNEL_LOGIC, TRUE, g_strdup_printf("CH%d", j));

		devices = g_slist_append(devices, sdi);
	}

	libusb_free_device_list(devlist, 1);
	return devices;
}

static int config_get(uint32_t key, GVariant **data, 
	const struct sr_dev_inst *sdi, const struct sr_channel_group *cg)
{
	struct dev_context *devc;
	(void)cg;

	if (!sdi) return SR_ERR_ARG;
	devc = sdi->priv;

	switch (key) {
	case SR_CONF_SAMPLERATE:
		*data = g_variant_new_uint64(devc->cur_samplerate);
		break;
	default: return SR_ERR_NA;
	}
	return SR_OK;
}

static int config_set(uint32_t key, GVariant *data, 
	const struct sr_dev_inst *sdi, const struct sr_channel_group *cg)
{
	struct dev_context *devc;
	(void)cg;

	if (!sdi) return SR_ERR_ARG;
	devc = sdi->priv;

	switch (key) {
	case SR_CONF_SAMPLERATE:
		devc->cur_samplerate = g_variant_get_uint64(data);
		return zlg_la_set_samplerate(sdi, devc->cur_samplerate);
	case SR_CONF_LIMIT_SAMPLES:
		return sr_sw_limits_config_set(&devc->limits, key, data);
	default: return SR_ERR_NA;
	}
	return SR_OK;
}

static int config_list(uint32_t key, GVariant **data, 
	const struct sr_dev_inst *sdi, const struct sr_channel_group *cg)
{
	switch (key) {
	case SR_CONF_SCAN_OPTIONS:
	case SR_CONF_DEVICE_OPTIONS:
		return std_opts_config_list(key, data, sdi, cg, 
			scanopts, G_N_ELEMENTS(scanopts),
			drvopts, G_N_ELEMENTS(drvopts),
			devopts, G_N_ELEMENTS(devopts));
	case SR_CONF_SAMPLERATE:
		*data = std_gvar_samplerates_steps(samplerates, G_N_ELEMENTS(samplerates));
		break;
	default: return SR_ERR_NA;
	}
	return SR_OK;
}

static int dev_open(struct sr_dev_inst *sdi)
{
	struct sr_dev_driver *di = sdi->driver;
	struct drv_context *drvc = di->context;
	struct sr_usb_dev_inst *usb = sdi->conn;
	struct dev_context *devc = sdi->priv;
	int ret;

	/* 
	 * Instead of std_usb_dev_open, we use sr_usb_open.
	 * This is more reliable across different sigrok versions.
	 */
	ret = sr_usb_open(drvc->sr_ctx->libusb_ctx, usb);
	if (ret != SR_OK)
		return ret;

	/*
	 * TODO: Check if FW is already loaded.
	 * We can send 0xFD02 and check the response.
	 * For now, we upload every time for testing.
	 */
	ret = zlg_la_fw_upload(sdi, ZLG_FW_NAME);
	if (ret != SR_OK)
		return ret;

	devc->state = STATE_IDLE;
	devc->timer_id = g_timeout_add(100, zlg_la_work_loop, (void *)sdi);

	return SR_OK;
}

static int dev_close(struct sr_dev_inst *sdi)
{
	struct sr_usb_dev_inst *usb = sdi->conn;
	struct dev_context *devc = sdi->priv;

	if (!usb->devhdl)
		return SR_OK;

	sr_info("Closing device on %d.%d interface 0", usb->bus, usb->address);
	devc->state = STATE_DESTROY;
	g_source_remove(devc->timer_id);

	
	/* Close the libusb handle directly */
	libusb_release_interface(usb->devhdl, 0);
	libusb_close(usb->devhdl);
	usb->devhdl = NULL;

	return SR_OK;
}

static int dev_acquisition_start(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc = sdi->priv;

	std_session_send_df_header(sdi);

	devc->state = STATE_CAPTURE;

	return SR_OK;
}

static int dev_acquisition_stop(struct sr_dev_inst *sdi)
{
	struct dev_context *devc = sdi->priv;
	devc->state = STATE_IDLE;
	return SR_OK;
}

static struct sr_dev_driver zlg_la_driver_info = {
	.name = "zlg-la",
	.longname = "ZLG LA Series",
	.api_version = 1,
	.init = std_init,
	.cleanup = std_cleanup,
	.scan = scan,
	.dev_list = std_dev_list,
	.dev_clear = std_dev_clear,
	.dev_open = dev_open,
	.dev_close = dev_close,
	.config_get = config_get,
	.config_set = config_set,
	.config_list = config_list,
	.dev_acquisition_start = dev_acquisition_start,
	.dev_acquisition_stop = dev_acquisition_stop,
};
SR_REGISTER_DEV_DRIVER(zlg_la_driver_info);