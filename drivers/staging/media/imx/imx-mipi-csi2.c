/*
 * MIPI CSI-2 Receiver Subdev for Freescale i.MX5/6 SOC.
 *
 * Copyright (c) 2012-2014 Mentor Graphics Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#include <linux/module.h>
#include <linux/export.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/list.h>
#include <linux/irq.h>
#include <linux/of_device.h>
#include <media/v4l2-device.h>
#include <media/v4l2-of.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-async.h>
#include <asm/mach/irq.h>
#include <video/imx-ipu-v3.h>
#include "imx-media.h"

/*
 * there must be 5 pads: 1 input pad from sensor, and
 * the 4 virtual channel output pads
 */
#define CSI2_NUM_SINK_PADS  1
#define CSI2_NUM_SRC_PADS   4
#define CSI2_NUM_PADS       5

struct imxcsi2_dev {
	struct device          *dev;
	struct imx_media_dev   *md;
	struct v4l2_subdev      sd;
	struct media_pad       pad[CSI2_NUM_PADS];
	struct v4l2_mbus_framefmt format_mbus;
	struct v4l2_subdev     *src_sd;
	struct v4l2_subdev     *sink_sd[CSI2_NUM_SRC_PADS];
	int                    input_pad;
	struct clk             *dphy_clk;
	struct clk             *cfg_clk;
	struct clk             *pix_clk; /* what is this? */
	void __iomem           *base;
	int                     intr1;
	int                     intr2;
	struct v4l2_of_bus_mipi_csi2 bus;
	bool                    on;
	bool                    stream_on;
};

#define DEVICE_NAME "imx-mipi-csi2"

/* Register offsets */
#define CSI2_VERSION            0x000
#define CSI2_N_LANES            0x004
#define CSI2_PHY_SHUTDOWNZ      0x008
#define CSI2_DPHY_RSTZ          0x00c
#define CSI2_RESETN             0x010
#define CSI2_PHY_STATE          0x014
#define CSI2_DATA_IDS_1         0x018
#define CSI2_DATA_IDS_2         0x01c
#define CSI2_ERR1               0x020
#define CSI2_ERR2               0x024
#define CSI2_MSK1               0x028
#define CSI2_MSK2               0x02c
#define CSI2_PHY_TST_CTRL0      0x030
#define CSI2_PHY_TST_CTRL1      0x034
#define CSI2_SFT_RESET          0xf00

static inline struct imxcsi2_dev *sd_to_dev(struct v4l2_subdev *sdev)
{
	return container_of(sdev, struct imxcsi2_dev, sd);
}

static inline u32 imxcsi2_read(struct imxcsi2_dev *csi2, unsigned int regoff)
{
	return readl(csi2->base + regoff);
}

static inline void imxcsi2_write(struct imxcsi2_dev *csi2, u32 val,
				 unsigned int regoff)
{
	writel(val, csi2->base + regoff);
}

static void imxcsi2_set_lanes(struct imxcsi2_dev *csi2)
{
	int lanes = csi2->bus.num_data_lanes;

	imxcsi2_write(csi2, lanes - 1, CSI2_N_LANES);
}

static void imxcsi2_enable(struct imxcsi2_dev *csi2, bool enable)
{
	if (enable) {
		imxcsi2_write(csi2, 0xffffffff, CSI2_PHY_SHUTDOWNZ);
		imxcsi2_write(csi2, 0xffffffff, CSI2_DPHY_RSTZ);
		imxcsi2_write(csi2, 0xffffffff, CSI2_RESETN);
	} else {
		imxcsi2_write(csi2, 0x0, CSI2_PHY_SHUTDOWNZ);
		imxcsi2_write(csi2, 0x0, CSI2_DPHY_RSTZ);
		imxcsi2_write(csi2, 0x0, CSI2_RESETN);
	}
}

static void imxcsi2_reset(struct imxcsi2_dev *csi2)
{
	imxcsi2_enable(csi2, false);

	imxcsi2_write(csi2, 0x00000001, CSI2_PHY_TST_CTRL0);
	imxcsi2_write(csi2, 0x00000000, CSI2_PHY_TST_CTRL1);
	imxcsi2_write(csi2, 0x00000000, CSI2_PHY_TST_CTRL0);
	imxcsi2_write(csi2, 0x00000002, CSI2_PHY_TST_CTRL0);
	imxcsi2_write(csi2, 0x00010044, CSI2_PHY_TST_CTRL1);
	imxcsi2_write(csi2, 0x00000000, CSI2_PHY_TST_CTRL0);
	imxcsi2_write(csi2, 0x00000014, CSI2_PHY_TST_CTRL1);
	imxcsi2_write(csi2, 0x00000002, CSI2_PHY_TST_CTRL0);
	imxcsi2_write(csi2, 0x00000000, CSI2_PHY_TST_CTRL0);

	imxcsi2_enable(csi2, true);
}

static int imxcsi2_dphy_wait(struct imxcsi2_dev *csi2)
{
	u32 reg;
	int i;

	/* wait for mipi sensor ready */
	for (i = 0; i < 50; i++) {
		reg = imxcsi2_read(csi2, CSI2_PHY_STATE);
		if (reg != 0x200)
			break;
		usleep_range(10000, 20000);
	}

	if (i >= 50) {
		v4l2_err(&csi2->sd,
			 "wait for clock lane timeout, phy_state = 0x%08x\n",
			 reg);
		return -ETIME;
	}

	/* wait for mipi stable */
	for (i = 0; i < 50; i++) {
		reg = imxcsi2_read(csi2, CSI2_ERR1);
		if (reg == 0x0)
			break;
		usleep_range(10000, 20000);
	}

	if (i >= 50) {
		v4l2_err(&csi2->sd,
			 "wait for controller timeout, err1 = 0x%08x\n",
			 reg);
		return -ETIME;
	}

	/* finally let's wait for active clock on the clock lane */
	for (i = 0; i < 50; i++) {
		reg = imxcsi2_read(csi2, CSI2_PHY_STATE);
		if (reg & (1 << 8))
			break;
		usleep_range(10000, 20000);
	}

	if (i >= 50) {
		v4l2_err(&csi2->sd,
			 "wait for active clock timeout, phy_state = 0x%08x\n",
			 reg);
		return -ETIME;
	}

	v4l2_info(&csi2->sd, "ready, dphy version 0x%x\n",
		  imxcsi2_read(csi2, CSI2_VERSION));

	return 0;
}

/*
 * V4L2 subdev operations
 */

static int imxcsi2_link_setup(struct media_entity *entity,
			      const struct media_pad *local,
			      const struct media_pad *remote, u32 flags)
{
	struct v4l2_subdev *sd = media_entity_to_v4l2_subdev(entity);
	struct imxcsi2_dev *csi2 = sd_to_dev(sd);
	struct v4l2_subdev *remote_sd;

	dev_dbg(csi2->dev, "link setup %s -> %s", remote->entity->name,
		local->entity->name);

	remote_sd = media_entity_to_v4l2_subdev(remote->entity);

	if (local->flags & MEDIA_PAD_FL_SOURCE) {
		if (flags & MEDIA_LNK_FL_ENABLED) {
			if (csi2->sink_sd[local->index])
				return -EBUSY;
			csi2->sink_sd[local->index] = remote_sd;
		} else {
			csi2->sink_sd[local->index] = NULL;
		}
	} else {
		if (flags & MEDIA_LNK_FL_ENABLED) {
			if (csi2->src_sd)
				return -EBUSY;
			csi2->src_sd = remote_sd;
		} else {
			csi2->src_sd = NULL;
		}
	}

	return 0;
}

static int imxcsi2_s_power(struct v4l2_subdev *sd, int on)
{
	struct imxcsi2_dev *csi2 = sd_to_dev(sd);

	if (on && !csi2->on) {
		v4l2_info(&csi2->sd, "power on\n");
		clk_prepare_enable(csi2->cfg_clk);
		clk_prepare_enable(csi2->dphy_clk);
		imxcsi2_set_lanes(csi2);
		imxcsi2_reset(csi2);
	} else if (!on && csi2->on) {
		v4l2_info(&csi2->sd, "power off\n");
		imxcsi2_enable(csi2, false);
		clk_disable_unprepare(csi2->dphy_clk);
		clk_disable_unprepare(csi2->cfg_clk);
	}

	csi2->on = on;
	return 0;
}

static int imxcsi2_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct imxcsi2_dev *csi2 = sd_to_dev(sd);
	int ret = 0;

	v4l2_info(sd, "stream %s\n", enable ? "ON" : "OFF");

	if (enable && !csi2->stream_on) {
		clk_prepare_enable(csi2->pix_clk);
		ret = imxcsi2_dphy_wait(csi2);
		if (ret)
			clk_disable_unprepare(csi2->pix_clk);
	} else if (!enable && csi2->stream_on) {
		clk_disable_unprepare(csi2->pix_clk);
	}

	if (!ret)
		csi2->stream_on = enable;
	return ret;
}

static int imxcsi2_get_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *sdformat)
{
	struct imxcsi2_dev *csi2 = sd_to_dev(sd);

	sdformat->format = csi2->format_mbus;

	return 0;
}

static int imxcsi2_set_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *sdformat)
{
	struct imxcsi2_dev *csi2 = sd_to_dev(sd);

	if (sdformat->pad >= CSI2_NUM_PADS)
		return -EINVAL;

	if (csi2->stream_on)
		return -EBUSY;

	/* Output pads mirror active input pad, no limits on input pads */
	if (sdformat->pad != csi2->input_pad)
		sdformat->format = csi2->format_mbus;

	if (sdformat->which == V4L2_SUBDEV_FORMAT_TRY)
		cfg->try_fmt = sdformat->format;
	else
		csi2->format_mbus = sdformat->format;

	return 0;
}

/*
 * retrieve our pads parsed from the OF graph by the media device
 */
static int imxcsi2_registered(struct v4l2_subdev *sd)
{
	struct imxcsi2_dev *csi2 = sd_to_dev(sd);
	struct imx_media_subdev *imxsd;
	struct imx_media_pad *pad;
	int i;

	/* get media device */
	csi2->md = dev_get_drvdata(sd->v4l2_dev->dev);

	imxsd = imx_media_find_subdev_by_sd(csi2->md, sd);
	if (IS_ERR(imxsd))
		return PTR_ERR(imxsd);

	if (imxsd->num_sink_pads != 1 || imxsd->num_src_pads != 4)
		return -EINVAL;

	for (i = 0; i < CSI2_NUM_PADS; i++) {
		pad = &imxsd->pad[i];
		csi2->pad[i] = pad->pad;
		if (csi2->pad[i].flags & MEDIA_PAD_FL_SINK)
			csi2->input_pad = i;
	}

	return media_entity_pads_init(&sd->entity, CSI2_NUM_PADS, csi2->pad);
}

static struct media_entity_operations imxcsi2_entity_ops = {
	.link_setup = imxcsi2_link_setup,
	.link_validate = v4l2_subdev_link_validate,
};

static struct v4l2_subdev_core_ops imxcsi2_core_ops = {
	.s_power = imxcsi2_s_power,
};

static struct v4l2_subdev_video_ops imxcsi2_video_ops = {
	.s_stream = imxcsi2_s_stream,
};

static struct v4l2_subdev_pad_ops imxcsi2_pad_ops = {
	.get_fmt = imxcsi2_get_fmt,
	.set_fmt = imxcsi2_set_fmt,
};

static struct v4l2_subdev_ops imxcsi2_subdev_ops = {
	.core = &imxcsi2_core_ops,
	.video = &imxcsi2_video_ops,
	.pad = &imxcsi2_pad_ops,
};

static struct v4l2_subdev_internal_ops imxcsi2_internal_ops = {
	.registered = imxcsi2_registered,
};

static int imxcsi2_parse_endpoints(struct imxcsi2_dev *csi2)
{
	struct device_node *node = csi2->dev->of_node;
	struct device_node *epnode;
	struct v4l2_of_endpoint ep;
	int ret = 0;

	epnode = of_graph_get_next_endpoint(node, NULL);
	if (!epnode) {
		v4l2_err(&csi2->sd, "failed to get endpoint node\n");
		return -EINVAL;
	}

	v4l2_of_parse_endpoint(epnode, &ep);
	if (ep.bus_type != V4L2_MBUS_CSI2) {
		v4l2_err(&csi2->sd, "invalid bus type, must be MIPI CSI2\n");
		ret = -EINVAL;
		goto out;
	}

	csi2->bus = ep.bus.mipi_csi2;

	v4l2_info(&csi2->sd, "data lanes: %d\n", csi2->bus.num_data_lanes);
	v4l2_info(&csi2->sd, "flags: 0x%08x\n", csi2->bus.flags);
out:
	of_node_put(epnode);
	return ret;
}

static int imxcsi2_probe(struct platform_device *pdev)
{
	struct imxcsi2_dev *csi2;
	struct resource *res;
	int ret;

	csi2 = devm_kzalloc(&pdev->dev, sizeof(*csi2), GFP_KERNEL);
	if (!csi2)
		return -ENOMEM;

	csi2->dev = &pdev->dev;

	v4l2_subdev_init(&csi2->sd, &imxcsi2_subdev_ops);
	v4l2_set_subdevdata(&csi2->sd, &pdev->dev);
	csi2->sd.internal_ops = &imxcsi2_internal_ops;
	csi2->sd.entity.ops = &imxcsi2_entity_ops;
	csi2->sd.dev = &pdev->dev;
	csi2->sd.owner = THIS_MODULE;
	csi2->sd.flags = V4L2_SUBDEV_FL_HAS_DEVNODE;
	strcpy(csi2->sd.name, DEVICE_NAME);
	/* FIXME: this is a mipi-csi2 receiver, function isn't right */
	csi2->sd.entity.function = MEDIA_ENT_F_ATV_DECODER;
	csi2->sd.grp_id = IMX_MEDIA_GRP_ID_CSI2;

	ret = imxcsi2_parse_endpoints(csi2);
	if (ret)
		return ret;

	csi2->cfg_clk = devm_clk_get(&pdev->dev, "cfg_clk");
	if (IS_ERR(csi2->cfg_clk)) {
		v4l2_err(&csi2->sd, "failed to get cfg clock\n");
		ret = PTR_ERR(csi2->cfg_clk);
		return ret;
	}

	csi2->dphy_clk = devm_clk_get(&pdev->dev, "dphy_clk");
	if (IS_ERR(csi2->dphy_clk)) {
		v4l2_err(&csi2->sd, "failed to get dphy clock\n");
		ret = PTR_ERR(csi2->dphy_clk);
		return ret;
	}

	csi2->pix_clk = devm_clk_get(&pdev->dev, "pix_clk");
	if (IS_ERR(csi2->pix_clk)) {
		v4l2_err(&csi2->sd, "failed to get pixel clock\n");
		ret = PTR_ERR(csi2->pix_clk);
		return ret;
	}

	csi2->intr1 = platform_get_irq(pdev, 0);
	csi2->intr2 = platform_get_irq(pdev, 1);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	if (!res || csi2->intr1 < 0 || csi2->intr2 < 0) {
		v4l2_err(&csi2->sd, "failed to get platform resources\n");
		return -ENODEV;
	}

	csi2->base = devm_ioremap(&pdev->dev, res->start, PAGE_SIZE);
	if (!csi2->base) {
		v4l2_err(&csi2->sd, "failed to map CSI-2 registers\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, &csi2->sd);

	return v4l2_async_register_subdev(&csi2->sd);
}

static int imxcsi2_remove(struct platform_device *pdev)
{
	struct v4l2_subdev *sd = platform_get_drvdata(pdev);
	struct imxcsi2_dev *csi2 = sd_to_dev(sd);

	imxcsi2_s_power(sd, 0);

	v4l2_async_unregister_subdev(&csi2->sd);
	media_entity_cleanup(&csi2->sd.entity);
	v4l2_device_unregister_subdev(sd);

	return 0;
}

static const struct of_device_id imxcsi2_dt_ids[] = {
	{ .compatible = "fsl,imx-mipi-csi2", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imxcsi2_dt_ids);

static struct platform_driver imxcsi2_driver = {
	.driver = {
		.name = DEVICE_NAME,
		.owner = THIS_MODULE,
		.of_match_table = imxcsi2_dt_ids,
	},
	.probe = imxcsi2_probe,
	.remove = imxcsi2_remove,
};

module_platform_driver(imxcsi2_driver);

MODULE_DESCRIPTION("i.MX5/6 MIPI CSI-2 Receiver driver");
MODULE_AUTHOR("Mentor Graphics Inc.");
MODULE_LICENSE("GPL");

