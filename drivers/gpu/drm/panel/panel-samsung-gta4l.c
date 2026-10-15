// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung Galaxy Tab A7 (gta4l) DSI panels.
 *
 * The tablet was built with many different 1200x2000 panels. The bootloader
 * detects the fitted one and names it on the kernel command line
 * (msm_drm.dsi_display0=qcom,mdss_dsi_<name>:). This driver picks the
 * matching entry and replays the vendor's own command tables, which are
 * generated from the stock device tree (see gta4l-panels.h).
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_of.h>
#include <drm/drm_panel.h>

struct gta4l_panel {
	const char *name;
	const u8 *on, *off, *rst;
	unsigned int on_len, off_len, rst_len;
	u16 hfp, hsw, hbp, vfp, vsw, vbp;
	u8 fps, width_mm, height_mm, lanes;
	u8 bl_be, eot, hs_clk, on_lp, off_lp;
	u16 bl_min, bl_max;
};

#include "gta4l-panels.h"

static char *panel_name;
module_param_named(panel, panel_name, charp, 0444);
MODULE_PARM_DESC(panel, "Panel name, overriding the one named by the bootloader");

struct gta4l {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	const struct gta4l_panel *p;
	struct regulator *vddio;
	struct gpio_desc *reset;
	enum drm_panel_orientation orientation;
};

static inline struct gta4l *to_gta4l(struct drm_panel *panel)
{
	return container_of(panel, struct gta4l, panel);
}

static const struct gta4l_panel *gta4l_find(const char *name, size_t len)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(gta4l_panels); i++)
		if (strlen(gta4l_panels[i].name) == len &&
		    !strncmp(gta4l_panels[i].name, name, len))
			return &gta4l_panels[i];
	return NULL;
}

static const struct gta4l_panel *gta4l_detect(struct device *dev)
{
	static const char key[] = "dsi_display0=qcom,mdss_dsi_";
	const struct gta4l_panel *p;
	const char *args, *s;

	if (panel_name) {
		p = gta4l_find(panel_name, strlen(panel_name));
		if (p)
			return p;
		dev_warn(dev, "unknown panel '%s' requested\n", panel_name);
	}

	if (of_chosen && !of_property_read_string(of_chosen, "bootargs", &args)) {
		s = strstr(args, key);
		if (s) {
			s += sizeof(key) - 1;
			p = gta4l_find(s, strcspn(s, ": "));
			if (p)
				return p;
			dev_warn(dev, "bootloader names a panel this driver does not know: %.40s\n", s);
		} else {
			dev_warn(dev, "bootloader did not name a panel\n");
		}
	}

	/*
	 * Driving a panel with another panel's commands can leave it in a bad
	 * state, so do not guess: leave the display off. The serial console
	 * still works, and panel= can name the panel by hand.
	 */
	dev_err(dev, "no known panel, leaving the display off (use panel=<name>)\n");
	return NULL;
}

/*
 * Replay a vendor command table. Each entry is: type, last, channel, ack,
 * wait in ms, 16-bit payload length, payload.
 */
static int gta4l_send(struct gta4l *ctx, const u8 *b, unsigned int len, bool lp)
{
	struct mipi_dsi_device *dsi = ctx->dsi;
	const struct mipi_dsi_host_ops *ops = dsi->host->ops;
	unsigned int o = 0;
	int ret;

	while (o + 7 <= len) {
		unsigned int n = (b[o + 5] << 8) | b[o + 6];
		struct mipi_dsi_msg msg = {
			.channel = dsi->channel,
			.type = b[o],
			.flags = lp ? MIPI_DSI_MSG_USE_LPM : 0,
			.tx_len = n,
			.tx_buf = &b[o + 7],
		};

		if (o + 7 + n > len)
			return -EINVAL;

		ret = ops->transfer(dsi->host, &msg);
		if (ret < 0) {
			dev_err(&dsi->dev, "command at offset %u failed: %d\n", o, ret);
			return ret;
		}
		if (b[o + 4])
			msleep(b[o + 4]);
		o += 7 + n;
	}

	return 0;
}

static void gta4l_reset(struct gta4l *ctx)
{
	const struct gta4l_panel *p = ctx->p;
	int i;

	/* Pairs of line level and time to hold it; the GPIO is active low */
	for (i = 0; i + 1 < p->rst_len; i += 2) {
		gpiod_set_value_cansleep(ctx->reset, !p->rst[i]);
		if (p->rst[i + 1])
			usleep_range(p->rst[i + 1] * 1000, p->rst[i + 1] * 1000 + 1000);
	}
}

static int gta4l_prepare(struct drm_panel *panel)
{
	struct gta4l *ctx = to_gta4l(panel);
	int ret;

	ret = regulator_enable(ctx->vddio);
	if (ret) {
		dev_err(panel->dev, "failed to enable vddio: %d\n", ret);
		return ret;
	}

	gta4l_reset(ctx);

	ret = gta4l_send(ctx, ctx->p->on, ctx->p->on_len, ctx->p->on_lp);
	if (ret < 0) {
		gpiod_set_value_cansleep(ctx->reset, 1);
		regulator_disable(ctx->vddio);
		return ret;
	}

	return 0;
}

static int gta4l_disable(struct drm_panel *panel)
{
	struct gta4l *ctx = to_gta4l(panel);

	gta4l_send(ctx, ctx->p->off, ctx->p->off_len, ctx->p->off_lp);

	return 0;
}

static int gta4l_unprepare(struct drm_panel *panel)
{
	struct gta4l *ctx = to_gta4l(panel);

	gpiod_set_value_cansleep(ctx->reset, 1);
	regulator_disable(ctx->vddio);

	return 0;
}

static int gta4l_get_modes(struct drm_panel *panel, struct drm_connector *connector)
{
	const struct gta4l_panel *p = to_gta4l(panel)->p;
	struct drm_display_mode m = {
		.hdisplay = 1200,
		.hsync_start = 1200 + p->hfp,
		.hsync_end = 1200 + p->hfp + p->hsw,
		.htotal = 1200 + p->hfp + p->hsw + p->hbp,
		.vdisplay = 2000,
		.vsync_start = 2000 + p->vfp,
		.vsync_end = 2000 + p->vfp + p->vsw,
		.vtotal = 2000 + p->vfp + p->vsw + p->vbp,
		.width_mm = p->width_mm,
		.height_mm = p->height_mm,
		.type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED,
	};
	struct drm_display_mode *mode;

	m.clock = m.htotal * m.vtotal * p->fps / 1000;

	mode = drm_mode_duplicate(connector->dev, &m);
	if (!mode)
		return -ENOMEM;

	drm_mode_set_name(mode);
	drm_mode_probed_add(connector, mode);

	connector->display_info.width_mm = p->width_mm;
	connector->display_info.height_mm = p->height_mm;
	connector->display_info.bpc = 8;

	return 1;
}

static enum drm_panel_orientation gta4l_get_orientation(struct drm_panel *panel)
{
	return to_gta4l(panel)->orientation;
}

static const struct drm_panel_funcs gta4l_funcs = {
	.prepare = gta4l_prepare,
	.disable = gta4l_disable,
	.unprepare = gta4l_unprepare,
	.get_modes = gta4l_get_modes,
	.get_orientation = gta4l_get_orientation,
};

static int gta4l_bl_update_status(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	struct gta4l *ctx = mipi_dsi_get_drvdata(dsi);
	u16 v = backlight_get_brightness(bl);
	int ret;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;
	if (ctx->p->bl_be)
		ret = mipi_dsi_dcs_set_display_brightness_large(dsi, v);
	else
		ret = mipi_dsi_dcs_set_display_brightness(dsi, v);
	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	return ret < 0 ? ret : 0;
}

static const struct backlight_ops gta4l_bl_ops = {
	.update_status = gta4l_bl_update_status,
};

static int gta4l_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct backlight_properties props = {
		.type = BACKLIGHT_RAW,
		.brightness = 512,
		.scale = BACKLIGHT_SCALE_NON_LINEAR,
	};
	struct gta4l *ctx;
	int ret;

	ctx = devm_drm_panel_alloc(dev, struct gta4l, panel, &gta4l_funcs,
				   DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	ctx->p = gta4l_detect(dev);
	if (!ctx->p)
		return -ENODEV;
	dev_info(dev, "panel: %s\n", ctx->p->name);

	ctx->vddio = devm_regulator_get(dev, "vddio");
	if (IS_ERR(ctx->vddio))
		return dev_err_probe(dev, PTR_ERR(ctx->vddio), "failed to get vddio\n");

	ctx->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset))
		return dev_err_probe(dev, PTR_ERR(ctx->reset), "failed to get reset gpio\n");

	ctx->dsi = dsi;
	mipi_dsi_set_drvdata(dsi, ctx);

	ret = drm_of_get_panel_orientation(dev->of_node, &ctx->orientation);
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to get orientation\n");

	ctx->panel.prepare_prev_first = true;

	props.max_brightness = ctx->p->bl_max;
	ctx->panel.backlight = devm_backlight_device_register(dev, dev_name(dev), dev, dsi,
							      &gta4l_bl_ops, &props);
	if (IS_ERR(ctx->panel.backlight))
		return dev_err_probe(dev, PTR_ERR(ctx->panel.backlight),
				     "failed to create backlight\n");

	drm_panel_add(&ctx->panel);

	dsi->lanes = ctx->p->lanes;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_LPM;
	if (!ctx->p->hs_clk)
		dsi->mode_flags |= MIPI_DSI_CLOCK_NON_CONTINUOUS;
	if (!ctx->p->eot)
		dsi->mode_flags |= MIPI_DSI_MODE_NO_EOT_PACKET;

	ret = devm_mipi_dsi_attach(dev, dsi);
	if (ret < 0) {
		drm_panel_remove(&ctx->panel);
		return dev_err_probe(dev, ret, "failed to attach to DSI host\n");
	}

	return 0;
}

static void gta4l_remove(struct mipi_dsi_device *dsi)
{
	struct gta4l *ctx = mipi_dsi_get_drvdata(dsi);

	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id gta4l_of_match[] = {
	{ .compatible = "samsung,gta4l-panel" },
	{ }
};
MODULE_DEVICE_TABLE(of, gta4l_of_match);

static struct mipi_dsi_driver gta4l_driver = {
	.probe = gta4l_probe,
	.remove = gta4l_remove,
	.driver = {
		.name = "panel-samsung-gta4l",
		.of_match_table = gta4l_of_match,
	},
};
module_mipi_dsi_driver(gta4l_driver);

MODULE_DESCRIPTION("Samsung Galaxy Tab A7 (gta4l) DSI panels");
MODULE_LICENSE("GPL");
