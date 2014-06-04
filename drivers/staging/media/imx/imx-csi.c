/*
 * V4L2 Capture CSI Subdev for Freescale i.MX5/6 SOC
 *
 * Copyright (c) 2014-2016 Mentor Graphics Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#include <linux/module.h>
#include <linux/platform_device.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <media/videobuf2-dma-contig.h>
#include <media/v4l2-of.h>
#include <media/v4l2-ctrls.h>
#include <video/imx-ipu-v3.h>
#include "imx-media.h"

#define CSI_NUM_PADS 2

struct csi_priv {
	struct device *dev;
	struct ipu_soc *ipu;
	struct imx_media_dev *md;
	struct v4l2_subdev sd;
	struct media_pad pad[CSI_NUM_PADS];
	struct v4l2_mbus_framefmt format_mbus[CSI_NUM_PADS];
	struct v4l2_mbus_config sensor_mbus_cfg;
	struct v4l2_rect crop;
	struct ipu_csi *csi;
	int csi_id;
	int input_pad;
	int output_pad;
	bool stream_on;

	/* the sink for the captured frames */
	struct v4l2_subdev *sink_sd;
	enum ipu_csi_dest dest;
	struct v4l2_subdev *src_sd;
};

static inline struct csi_priv *sd_to_dev(struct v4l2_subdev *sdev)
{
	return container_of(sdev, struct csi_priv, sd);
}

/* Update the CSI whole sensor and active windows */
static int csi_setup(struct csi_priv *priv)
{
	struct v4l2_mbus_framefmt infmt;

	ipu_csi_set_window(priv->csi, &priv->crop);

	/*
	 * the ipu-csi doesn't understand ALTERNATE, but it only
	 * needs to know whether the stream is interlaced, so set
	 * to INTERLACED if infmt field is ALTERNATE.
	 */
	infmt = priv->format_mbus[priv->input_pad];
	if (infmt.field == V4L2_FIELD_ALTERNATE)
		infmt.field = V4L2_FIELD_INTERLACED;

	ipu_csi_init_interface(priv->csi, &priv->sensor_mbus_cfg, &infmt);

	ipu_csi_set_dest(priv->csi, priv->dest);

	ipu_csi_dump(priv->csi);

	return 0;
}

static int csi_start(struct csi_priv *priv)
{
	int ret;

	ret = csi_setup(priv);
	if (ret)
		return ret;

	ret = ipu_csi_enable(priv->csi);
	if (ret) {
		v4l2_err(&priv->sd, "CSI enable error: %d\n", ret);
		return ret;
	}

	return 0;
}

static int csi_stop(struct csi_priv *priv)
{
	ipu_csi_disable(priv->csi);

	return 0;
}

static int csi_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct csi_priv *priv = v4l2_get_subdevdata(sd);
	int ret = 0;

	v4l2_info(sd, "stream %s\n", enable ? "ON" : "OFF");

	if (enable && !priv->stream_on)
		ret = csi_start(priv);
	else if (!enable && priv->stream_on)
		ret = csi_stop(priv);

	if (!ret)
		priv->stream_on = enable;
	return ret;
}

static int csi_link_setup(struct media_entity *entity,
			  const struct media_pad *local,
			  const struct media_pad *remote, u32 flags)
{
	struct v4l2_subdev *sd = media_entity_to_v4l2_subdev(entity);
	struct csi_priv *priv = v4l2_get_subdevdata(sd);
	struct v4l2_subdev *remote_sd;

	dev_dbg(priv->dev, "link setup %s -> %s", remote->entity->name,
		local->entity->name);

	remote_sd = media_entity_to_v4l2_subdev(remote->entity);

	if (local->flags & MEDIA_PAD_FL_SINK) {
		if (flags & MEDIA_LNK_FL_ENABLED) {
			if (priv->src_sd)
				return -EBUSY;
			priv->src_sd = remote_sd;
		} else {
			priv->src_sd = NULL;
			return 0;
		}

		return 0;
	}

	if (flags & MEDIA_LNK_FL_ENABLED) {
		if (priv->sink_sd)
			return -EBUSY;
		priv->sink_sd = remote_sd;
	} else {
		priv->sink_sd = NULL;
		return 0;
	}

	/* set CSI destination */
	switch (remote_sd->grp_id) {
	case IMX_MEDIA_GRP_ID_SMFC0:
	case IMX_MEDIA_GRP_ID_SMFC1:
	case IMX_MEDIA_GRP_ID_SMFC2:
	case IMX_MEDIA_GRP_ID_SMFC3:
		priv->dest = IPU_CSI_DEST_IDMAC;
		break;
	case IMX_MEDIA_GRP_ID_IC_PRPVF:
		priv->dest = IPU_CSI_DEST_VDIC;
		break;
	case IMX_MEDIA_GRP_ID_IC_PRPENC:
		priv->dest = IPU_CSI_DEST_IC;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int csi_link_validate(struct v4l2_subdev *sd,
			     struct media_link *link,
			     struct v4l2_subdev_format *source_fmt,
			     struct v4l2_subdev_format *sink_fmt)
{
	struct csi_priv *priv = v4l2_get_subdevdata(sd);
	struct imx_media_subdev *sensor;
	bool is_csi2;
	int ret;

	ret = v4l2_subdev_link_validate_default(sd, link, source_fmt, sink_fmt);
	if (ret)
		return ret;

	sensor = priv->md->sensor;
	if (!sensor) {
		v4l2_err(&priv->sd, "no sensor attached\n");
		return -EINVAL;
	}

	ret = v4l2_subdev_call(sensor->sd, video, g_mbus_config,
			       &priv->sensor_mbus_cfg);
	if (ret)
		return ret;

	is_csi2 = (priv->sensor_mbus_cfg.type == V4L2_MBUS_CSI2);

	if (is_csi2) {
		int vc_num = 0;
		/*
		 * NOTE! It seems the virtual channels from the mipi csi-2
		 * receiver are used only for routing by the video mux's,
		 * or for hard-wired routing to the CSI's. Once the stream
		 * enters the CSI's however, they are treated internally
		 * in the IPU as virtual channel 0.
		 */
#if 0
		vc_num = imx_media_find_mipi_csi2_channel(priv->md,
							  &priv->sd.entity);
		if (vc_num < 0)
			return vc_num;
#endif
		ipu_csi_set_mipi_datatype(priv->csi, vc_num,
					  &priv->format_mbus[priv->input_pad]);
	}

	/* select either parallel or MIPI-CSI2 as input to CSI */
	ipu_set_csi_src_mux(priv->ipu, priv->csi_id, is_csi2);

	return 0;
}


static int csi_try_crop(struct csi_priv *priv, struct v4l2_rect *crop)
{
	struct v4l2_mbus_framefmt *infmt;
	struct imx_media_subdev *sensor;
	v4l2_std_id std;
	int ret;

	sensor = priv->md->sensor;
	if (!sensor) {
		v4l2_err(&priv->sd, "no sensor attached\n");
		return -EINVAL;
	}

	ret = v4l2_subdev_call(sensor->sd, video, g_mbus_config,
			       &priv->sensor_mbus_cfg);
	if (ret)
		return ret;

	infmt = &priv->format_mbus[priv->input_pad];

	crop->width = min_t(__u32, infmt->width, crop->width);
	if (crop->left + crop->width > infmt->width)
		crop->left = infmt->width - crop->width;
	/* adjust crop left/width to h/w alignment restrictions */
	crop->left &= ~0x3;
	crop->width &= ~0x7;

	/*
	 * FIXME: not sure why yet, but on interlaced bt.656,
	 * changing the vertical cropping causes loss of vertical
	 * sync, so fix it to NTSC/PAL active lines. NTSC contains
	 * 2 extra lines of active video that need to be cropped.
	 */
	if (priv->sensor_mbus_cfg.type == V4L2_MBUS_BT656) {
		ret = v4l2_subdev_call(sensor->sd, video, g_std, &std);
		if (ret)
			return ret;
		if (std & V4L2_STD_525_60) {
			crop->top = 2;
			crop->height = 480;
		} else {
			crop->top = 0;
			crop->height = 576;
		}
	} else {
		crop->height = min_t(__u32, infmt->height, crop->height);
		if (crop->top + crop->height > infmt->height)
			crop->top = infmt->height - crop->height;
	}

	return 0;
}

static int csi_get_fmt(struct v4l2_subdev *sd,
		       struct v4l2_subdev_pad_config *cfg,
		       struct v4l2_subdev_format *sdformat)
{
	struct csi_priv *priv = v4l2_get_subdevdata(sd);

	if (sdformat->pad >= CSI_NUM_PADS)
		return -EINVAL;

	sdformat->format = priv->format_mbus[sdformat->pad];

	return 0;
}

static int csi_set_fmt(struct v4l2_subdev *sd,
		       struct v4l2_subdev_pad_config *cfg,
		       struct v4l2_subdev_format *sdformat)
{
	struct csi_priv *priv = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *infmt, *outfmt;
	struct v4l2_rect crop;
	int ret;

	if (sdformat->pad >= CSI_NUM_PADS)
		return -EINVAL;

	if (priv->stream_on)
		return -EBUSY;

	infmt = &priv->format_mbus[priv->input_pad];
	outfmt = &priv->format_mbus[priv->output_pad];

	if (sdformat->pad == priv->output_pad) {
		sdformat->format.code = infmt->code;
		sdformat->format.field = infmt->field;
		crop.left = priv->crop.left;
		crop.top = priv->crop.top;
		crop.width = sdformat->format.width;
		crop.height = sdformat->format.height;
		ret = csi_try_crop(priv, &crop);
		if (ret)
			return ret;
		sdformat->format.width = crop.width;
		sdformat->format.height = crop.height;
	}

	if (sdformat->which == V4L2_SUBDEV_FORMAT_TRY) {
		cfg->try_fmt = sdformat->format;
	} else {
		priv->format_mbus[sdformat->pad] = sdformat->format;
		/* Update the crop window if this is output pad  */
		if (sdformat->pad == priv->output_pad)
			priv->crop = crop;
	}

	return 0;
}

static int csi_get_selection(struct v4l2_subdev *sd,
			     struct v4l2_subdev_pad_config *cfg,
			     struct v4l2_subdev_selection *sel)
{
	struct csi_priv *priv = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *infmt;

	if (sel->pad != priv->output_pad)
		return -EINVAL;

	infmt = &priv->format_mbus[priv->input_pad];

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = infmt->width;
		sel->r.height = infmt->height;
		break;
	case V4L2_SEL_TGT_CROP:
		sel->r = priv->crop;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int csi_set_selection(struct v4l2_subdev *sd,
			     struct v4l2_subdev_pad_config *cfg,
			     struct v4l2_subdev_selection *sel)
{
	struct csi_priv *priv = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *outfmt;
	int ret;

	if (sel->pad != priv->output_pad ||
	    sel->target != V4L2_SEL_TGT_CROP)
		return -EINVAL;

	if (priv->stream_on)
		return -EBUSY;

	/*
	 * Modifying the crop rectangle always changes the format on the source
	 * pad. If the KEEP_CONFIG flag is set, just return the current crop
	 * rectangle.
	 */
	if (sel->flags & V4L2_SEL_FLAG_KEEP_CONFIG) {
		sel->r = priv->crop;
		if (sel->which == V4L2_SUBDEV_FORMAT_TRY)
			cfg->try_crop = sel->r;
		return 0;
	}

	outfmt = &priv->format_mbus[priv->output_pad];

	ret = csi_try_crop(priv, &sel->r);
	if (ret)
		return ret;

	if (sel->which == V4L2_SUBDEV_FORMAT_TRY) {
		cfg->try_crop = sel->r;
	} else {
		priv->crop = sel->r;
		/* Update the source format */
		outfmt->width = sel->r.width;
		outfmt->height = sel->r.height;
	}

	return 0;
}

/*
 * retrieve our pads parsed from the OF graph by the media device
 */
static int csi_registered(struct v4l2_subdev *sd)
{
	struct csi_priv *priv = v4l2_get_subdevdata(sd);
	struct imx_media_subdev *imxsd;
	struct imx_media_pad *pad;
	int i;

	/* get media device */
	priv->md = dev_get_drvdata(sd->v4l2_dev->dev);

	/* get handle to IPU CSI */
	priv->csi = ipu_csi_get(priv->ipu, priv->csi_id);
	if (IS_ERR(priv->csi)) {
		v4l2_err(&priv->sd, "failed to get CSI %d\n", priv->csi_id);
		return PTR_ERR(priv->csi);
	}

	imxsd = imx_media_find_subdev_by_sd(priv->md, sd);
	if (IS_ERR(imxsd))
		return PTR_ERR(imxsd);

	if (imxsd->num_sink_pads != 1 || imxsd->num_src_pads != 1)
		return -EINVAL;

	for (i = 0; i < CSI_NUM_PADS; i++) {
		pad = &imxsd->pad[i];
		priv->pad[i] = pad->pad;
		if (priv->pad[i].flags & MEDIA_PAD_FL_SINK)
			priv->input_pad = i;
		else
			priv->output_pad = i;
	}

	return media_entity_pads_init(&sd->entity, CSI_NUM_PADS, priv->pad);
}

static struct media_entity_operations csi_entity_ops = {
	.link_setup = csi_link_setup,
	.link_validate = v4l2_subdev_link_validate,
};

static struct v4l2_subdev_video_ops csi_video_ops = {
	.s_stream = csi_s_stream,
};

static struct v4l2_subdev_pad_ops csi_pad_ops = {
	.get_fmt = csi_get_fmt,
	.set_fmt = csi_set_fmt,
	.get_selection = csi_get_selection,
	.set_selection = csi_set_selection,
	.link_validate = csi_link_validate,
};

static struct v4l2_subdev_ops csi_subdev_ops = {
	.video = &csi_video_ops,
	.pad = &csi_pad_ops,
};

static struct v4l2_subdev_internal_ops csi_internal_ops = {
	.registered = csi_registered,
};

static int imx_csi_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct ipu_client_platformdata *pdata;
	struct csi_priv *priv;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	platform_set_drvdata(pdev, &priv->sd);
	priv->dev = &pdev->dev;

	/* get parent IPU */
	priv->ipu = dev_get_drvdata(priv->dev->parent);

	/* get our CSI id */
	pdata = priv->dev->platform_data;
	priv->csi_id = pdata->csi;

	v4l2_subdev_init(&priv->sd, &csi_subdev_ops);
	v4l2_set_subdevdata(&priv->sd, priv);
	priv->sd.internal_ops = &csi_internal_ops;
	priv->sd.entity.ops = &csi_entity_ops;
	/* FIXME: this the right function? */
	priv->sd.entity.function = MEDIA_ENT_F_PROC_VIDEO_PIXEL_FORMATTER;
	priv->sd.grp_id = priv->csi_id ?
		IMX_MEDIA_GRP_ID_CSI1 : IMX_MEDIA_GRP_ID_CSI0;
	priv->sd.dev = &pdev->dev;
	priv->sd.owner = THIS_MODULE;
	priv->sd.flags = V4L2_SUBDEV_FL_HAS_DEVNODE;
	snprintf(priv->sd.name, sizeof(priv->sd.name), "%s%d",
		 np->name, priv->csi_id);

	return v4l2_async_register_subdev(&priv->sd);
}

static int imx_csi_remove(struct platform_device *pdev)
{
	struct v4l2_subdev *sd = platform_get_drvdata(pdev);
	struct csi_priv *priv = sd_to_dev(sd);

	v4l2_async_unregister_subdev(&priv->sd);
	media_entity_cleanup(&priv->sd.entity);
	v4l2_device_unregister_subdev(sd);

	if (!IS_ERR_OR_NULL(priv->csi))
		ipu_csi_put(priv->csi);

	return 0;
}

static const struct platform_device_id imx_csi_ids[] = {
	{ .name = "imx-ipuv3-csi" },
	{ },
};
MODULE_DEVICE_TABLE(platform, imx_csi_ids);

static struct platform_driver imx_csi_driver = {
	.probe = imx_csi_probe,
	.remove = imx_csi_remove,
	.id_table = imx_csi_ids,
	.driver = {
		.name = "imx-ipuv3-csi",
		.owner = THIS_MODULE,
	},
};
module_platform_driver(imx_csi_driver);

MODULE_AUTHOR("Mentor Graphics Inc.");
MODULE_DESCRIPTION("i.MX CSI subdev driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:imx-ipuv3-csi");
