/*
 * V4L2 IC Post-Processor Subdev for Freescale i.MX5/6 SOC
 *
 * Copyright (c) 2014-2016 Mentor Graphics Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/timer.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/consumer.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-dma-contig.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-of.h>
#include <media/v4l2-ctrls.h>
#include <media/imx.h>
#include <video/imx-ipu-image-convert.h>
#include "imx-media.h"
#include "imx-ic.h"

#define PP_NUM_PADS 2

struct pp_priv {
	struct imx_media_dev *md;
	struct imx_ic_priv *ic_priv;
	int pp_id;

	struct ipu_image_convert_ctx *ic_ctx;

	struct media_pad pad[PP_NUM_PADS];
	int input_pad;
	int output_pad;

	/* our dma buffer sink ring */
	struct imx_media_dma_buf_ring *sink_ring;
	/* the dma buffer ring we send to sink */
	struct imx_media_dma_buf_ring *src_ring;
	struct ipu_image_convert_run *src_run;

	bool stream_on;    /* streaming is on */
	bool stop;         /* streaming is stopping */
	spinlock_t irqlock;

	struct v4l2_subdev *src_sd;
	struct v4l2_subdev *sink_sd;

	struct v4l2_mbus_framefmt format_mbus[PP_NUM_PADS];
	const struct imx_media_pixfmt *cc[PP_NUM_PADS];

	/* motion select control */
	struct v4l2_ctrl_handler ctrl_hdlr;
	int  rotation; /* degrees */
	bool hflip;
        bool vflip;

	/* derived from rotation, hflip, vflip controls */
	enum ipu_rotate_mode rot_mode;
};

static inline struct pp_priv *sd_to_priv(struct v4l2_subdev *sd)
{
	struct imx_ic_priv *ic_priv = v4l2_get_subdevdata(sd);

	return ic_priv->task_priv;
}

static void pp_convert_complete(struct ipu_image_convert_run *run,
				void *data)
{
	struct pp_priv *priv = data;

	/* push the completed buffer to the sink */
	imx_media_dma_buf_return(priv->src_ring,
				 run->status ?
				 IMX_MEDIA_BUF_STATUS_ERROR :
				 IMX_MEDIA_BUF_STATUS_DONE);
}

static void pp_queue_conversion(struct pp_priv *priv,
				struct imx_media_dma_buf *srcbuf)
{
	struct ipu_image_convert_run *run;
	struct imx_media_dma_buf *sinkbuf;

	sinkbuf = imx_media_dma_buf_next_in(priv->src_ring);

	run = &priv->src_run[sinkbuf->index];
	run->ctx = priv->ic_ctx;
	run->in_phys = srcbuf->phys;
	run->out_phys = sinkbuf->phys;
	ipu_image_convert_queue(run);
}

static long pp_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct pp_priv *priv = sd_to_priv(sd);
	struct imx_media_dma_buf_ring **ring;
	struct imx_media_dma_buf *buf;
	unsigned long flags;

	switch (cmd) {
	case IMX_MEDIA_REQ_DMA_BUF_RING:
		/* src asks for a buffer ring */
		if (!priv->sink_ring)
			return -EINVAL;
		ring = (struct imx_media_dma_buf_ring **)arg;
		*ring = priv->sink_ring;
		break;
	case IMX_MEDIA_NEW_DMA_BUF:
		/* src hands us a new buffer */
		spin_lock_irqsave(&priv->irqlock, flags);
		if (!priv->stop) {
			buf = imx_media_dma_buf_next_out(priv->sink_ring);
			pp_queue_conversion(priv, buf);
		}
		spin_unlock_irqrestore(&priv->irqlock, flags);
		break;
	case IMX_MEDIA_REL_DMA_BUF_RING:
		/* src indicates buffer ring can be freed */
		if (!priv->sink_ring)
			return 0;
		v4l2_info(sd, "%s: freeing sink ring\n", __func__);
		imx_media_free_dma_buf_ring(priv->sink_ring);
		priv->sink_ring = NULL;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int pp_start(struct pp_priv *priv)
{
	struct imx_ic_priv *ic_priv = priv->ic_priv;
	struct ipu_image image_in, image_out;
	const struct imx_media_pixfmt *incc;
	struct v4l2_mbus_framefmt *infmt;
	int in_size, ret;

	/* ask the sink for the buffer ring */
	ret = v4l2_subdev_call(priv->sink_sd, core, ioctl,
			       IMX_MEDIA_REQ_DMA_BUF_RING,
			       &priv->src_ring);
	if (ret)
		return ret;

	imx_media_mbus_fmt_to_ipu_image(&image_in,
					&priv->format_mbus[priv->input_pad]);
	imx_media_mbus_fmt_to_ipu_image(&image_out,
					&priv->format_mbus[priv->output_pad]);

	priv->ic_ctx = ipu_image_convert_prepare(ic_priv->ipu,
						 IC_TASK_POST_PROCESSOR,
						 &image_in, &image_out,
						 priv->rot_mode,
						 pp_convert_complete, priv);
	if (IS_ERR(priv->ic_ctx))
		return PTR_ERR(priv->ic_ctx);

	infmt = &priv->format_mbus[priv->input_pad];
	incc = priv->cc[priv->input_pad];
	in_size = (infmt->width * incc->bpp * infmt->height) >> 3;

	if (priv->sink_ring) {
		v4l2_warn(&ic_priv->sd, "%s: dma-buf ring was not freed\n",
			  __func__);
		imx_media_free_dma_buf_ring(priv->sink_ring);
	}

	priv->sink_ring =
		imx_media_alloc_dma_buf_ring(priv->md,
					     priv->src_sd, &ic_priv->sd,
					     in_size, 2, true);
	if (IS_ERR(priv->sink_ring)) {
		v4l2_err(&ic_priv->sd, "failed to alloc dma-buf ring\n");
		ret = PTR_ERR(priv->sink_ring);
		priv->sink_ring = NULL;
		goto out_unprep;
	}

	priv->src_run = kzalloc(IMX_MEDIA_MAX_RING_BUFS *
				 sizeof(*priv->src_run), GFP_KERNEL);
	if (!priv->src_run) {
		v4l2_err(&ic_priv->sd, "failed to alloc src ring runs\n");
		ret = -ENOMEM;
		goto out_free_ring;
	}

	priv->stop = false;

	return 0;

out_free_ring:
	imx_media_free_dma_buf_ring(priv->sink_ring);
	priv->sink_ring = NULL;
out_unprep:
	ipu_image_convert_unprepare(priv->ic_ctx);
	return ret;
}

static int pp_stop(struct pp_priv *priv)
{
	unsigned long flags;

	spin_lock_irqsave(&priv->irqlock, flags);
	priv->stop = true;
	spin_unlock_irqrestore(&priv->irqlock, flags);

	ipu_image_convert_unprepare(priv->ic_ctx);
	kfree(priv->src_run);

	priv->src_ring = NULL;

	/* inform sink that the buffer ring can now be freed */
	return v4l2_subdev_call(priv->sink_sd, core, ioctl,
				IMX_MEDIA_REL_DMA_BUF_RING, 0);
}

static int pp_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct pp_priv *priv = sd_to_priv(sd);
	int ret = 0;

	v4l2_info(sd, "stream %s\n", enable ? "ON" : "OFF");

	if (enable && !priv->stream_on)
		ret = pp_start(priv);
	else if (!enable && priv->stream_on)
		ret = pp_stop(priv);

	if (!ret)
		priv->stream_on = enable;
	return ret;
}

static int pp_enum_mbus_code(struct v4l2_subdev *sd,
				struct v4l2_subdev_pad_config *cfg,
				struct v4l2_subdev_mbus_code_enum *code)
{
	const struct imx_media_pixfmt *cc;
	u32 fourcc;
	int ret;

	if (code->pad >= PP_NUM_PADS)
		return -EINVAL;

	ret = ipu_image_convert_enum_format(code->index, &fourcc);
	if (ret)
		return ret;

	/* convert returned fourcc to mbus code */
	cc = imx_media_find_format(fourcc, 0, true, true);
	if (WARN_ON(!cc))
		return -EINVAL;

	code->code = cc->codes[0];
	return 0;
}

static int pp_get_fmt(struct v4l2_subdev *sd,
			 struct v4l2_subdev_pad_config *cfg,
			 struct v4l2_subdev_format *sdformat)
{
	struct pp_priv *priv = sd_to_priv(sd);

	if (sdformat->pad >= PP_NUM_PADS)
		return -EINVAL;

	sdformat->format = priv->format_mbus[sdformat->pad];

	return 0;
}

static int pp_set_fmt(struct v4l2_subdev *sd,
		      struct v4l2_subdev_pad_config *cfg,
		      struct v4l2_subdev_format *sdformat)
{
	struct pp_priv *priv = sd_to_priv(sd);
	struct v4l2_mbus_framefmt *infmt, *outfmt;
	const struct imx_media_pixfmt *cc;
	struct ipu_image test_in, test_out;
	u32 code;

	if (sdformat->pad >= PP_NUM_PADS)
		return -EINVAL;

	if (priv->stream_on)
		return -EBUSY;

	infmt = &priv->format_mbus[priv->input_pad];
	outfmt = &priv->format_mbus[priv->output_pad];

	cc = imx_media_find_format(0, sdformat->format.code, true, true);
	if (!cc) {
		imx_media_enum_format(&code, 0, true, true);
		cc = imx_media_find_format(0, code, true, true);
		sdformat->format.code = cc->codes[0];
	}

	if (sdformat->pad == priv->output_pad) {
		imx_media_mbus_fmt_to_ipu_image(&test_out, &sdformat->format);
		imx_media_mbus_fmt_to_ipu_image(&test_in, infmt);
		ipu_image_convert_adjust(&test_in, &test_out, priv->rot_mode);
		imx_media_ipu_image_to_mbus_fmt(&sdformat->format, &test_out);
	} else {
		imx_media_mbus_fmt_to_ipu_image(&test_in, &sdformat->format);
		imx_media_mbus_fmt_to_ipu_image(&test_out, outfmt);
		ipu_image_convert_adjust(&test_in, &test_out, priv->rot_mode);
		imx_media_ipu_image_to_mbus_fmt(&sdformat->format, &test_in);
	}

	if (sdformat->which == V4L2_SUBDEV_FORMAT_TRY) {
		cfg->try_fmt = sdformat->format;
	} else { 
		if (sdformat->pad == priv->output_pad) {
			*outfmt = sdformat->format;
			imx_media_ipu_image_to_mbus_fmt(infmt, &test_in);
		} else {
			*infmt = sdformat->format;
			imx_media_ipu_image_to_mbus_fmt(outfmt, &test_out);
		}
		priv->cc[sdformat->pad] = cc;
	}

	return 0;
}

static int pp_link_setup(struct media_entity *entity,
			 const struct media_pad *local,
			 const struct media_pad *remote, u32 flags)
{
	struct v4l2_subdev *sd = media_entity_to_v4l2_subdev(entity);
	struct imx_ic_priv *ic_priv = v4l2_get_subdevdata(sd);
	struct pp_priv *priv = ic_priv->task_priv;
	struct v4l2_subdev *remote_sd;

	dev_dbg(ic_priv->dev, "link setup %s -> %s", remote->entity->name,
		local->entity->name);

	remote_sd = media_entity_to_v4l2_subdev(remote->entity);

	if (local->flags & MEDIA_PAD_FL_SOURCE) {
		if (flags & MEDIA_LNK_FL_ENABLED) {
			if (priv->sink_sd)
				return -EBUSY;
			priv->sink_sd = remote_sd;
		} else {
			priv->sink_sd = NULL;
		}
	} else {
		if (flags & MEDIA_LNK_FL_ENABLED) {
			if (priv->src_sd)
				return -EBUSY;
			priv->src_sd = remote_sd;
		} else {
			priv->src_sd = NULL;
		}
	}

	return 0;
}

static int pp_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct pp_priv *priv = container_of(ctrl->handler,
					       struct pp_priv, ctrl_hdlr);
	struct imx_ic_priv *ic_priv = priv->ic_priv;
	enum ipu_rotate_mode rot_mode;
	bool hflip, vflip;
	int rotation, ret;

	rotation = priv->rotation;
	hflip = priv->hflip;
	vflip = priv->vflip;

	switch (ctrl->id) {
	case V4L2_CID_HFLIP:
		hflip = (ctrl->val == 1);
		break;
	case V4L2_CID_VFLIP:
		vflip = (ctrl->val == 1);
		break;
	case V4L2_CID_ROTATE:
		rotation = ctrl->val;
		break;
	default:
		v4l2_err(&ic_priv->sd, "Invalid control\n");
		return -EINVAL;
	}

	ret = ipu_degrees_to_rot_mode(&rot_mode, rotation, hflip, vflip);
	if (ret)
		return ret;

	if (rot_mode != priv->rot_mode) {
		struct v4l2_mbus_framefmt *infmt, *outfmt;
		struct ipu_image test_in, test_out;

		/* can't change rotation mid-streaming */
		if (priv->stream_on)
			return -EBUSY;

		/*
		 * make sure this rotation will work with current input/output
		 * formats before setting
		 */
		infmt = &priv->format_mbus[priv->input_pad];
		outfmt = &priv->format_mbus[priv->output_pad];
		imx_media_mbus_fmt_to_ipu_image(&test_in, infmt);
		imx_media_mbus_fmt_to_ipu_image(&test_out, outfmt);

		ret = ipu_image_convert_verify(&test_in, &test_out, rot_mode);
		if (ret)
			return ret;

		priv->rot_mode = rot_mode;
		priv->rotation = rotation;
		priv->hflip = hflip;
		priv->vflip = vflip;
	}

	return 0;
}

static const struct v4l2_ctrl_ops pp_ctrl_ops = {
	.s_ctrl = pp_s_ctrl,
};

static const struct v4l2_ctrl_config pp_std_ctrl[] = {
	{
		.id = V4L2_CID_HFLIP,
		.name = "Horizontal Flip",
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.def =  0,
		.min =  0,
		.max =  1,
		.step = 1,
	}, {
		.id = V4L2_CID_VFLIP,
		.name = "Vertical Flip",
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.def =  0,
		.min =  0,
		.max =  1,
		.step = 1,
	}, {
		.id = V4L2_CID_ROTATE,
		.name = "Rotation",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.def =   0,
		.min =   0,
		.max = 270,
		.step = 90,
	},
};

#define PP_NUM_CONTROLS ARRAY_SIZE(pp_std_ctrl)

static int pp_init_controls(struct pp_priv *priv)
{
	struct imx_ic_priv *ic_priv = priv->ic_priv;
	struct v4l2_ctrl_handler *hdlr = &priv->ctrl_hdlr;
	const struct v4l2_ctrl_config *c;
	int i, ret;

	v4l2_ctrl_handler_init(hdlr, PP_NUM_CONTROLS);

	for (i = 0; i < PP_NUM_CONTROLS; i++) {
		c = &pp_std_ctrl[i];
		v4l2_ctrl_new_std(hdlr, &pp_ctrl_ops,
				  c->id, c->min, c->max, c->step, c->def);
	}

	ic_priv->sd.ctrl_handler = hdlr;

	if (hdlr->error) {
		ret = hdlr->error;
		v4l2_ctrl_handler_free(hdlr);
		return ret;
	}

	v4l2_ctrl_handler_setup(hdlr);

	return 0;
}

/*
 * retrieve our pads parsed from the OF graph by the media device
 */
static int pp_registered(struct v4l2_subdev *sd)
{
	struct pp_priv *priv = sd_to_priv(sd);
	struct imx_media_subdev *imxsd;
	struct imx_media_pad *pad;
	int i;

	/* get media device */
	priv->md = dev_get_drvdata(sd->v4l2_dev->dev);

	imxsd = imx_media_find_subdev_by_sd(priv->md, sd);
	if (IS_ERR(imxsd))
		return PTR_ERR(imxsd);

	if (imxsd->num_sink_pads != 1 || imxsd->num_src_pads != 1)
		return -EINVAL;

	for (i = 0; i < PP_NUM_PADS; i++) {
		pad = &imxsd->pad[i];
		priv->pad[i] = pad->pad;
		if (priv->pad[i].flags & MEDIA_PAD_FL_SINK)
			priv->input_pad = i;
		else
			priv->output_pad = i;
	}

	return media_entity_pads_init(&sd->entity, PP_NUM_PADS,
				      priv->pad);
}

static struct v4l2_subdev_pad_ops pp_pad_ops = {
	.enum_mbus_code = pp_enum_mbus_code,
	.get_fmt = pp_get_fmt,
	.set_fmt = pp_set_fmt,
};

static struct v4l2_subdev_video_ops pp_video_ops = {
	.s_stream = pp_s_stream,
};

static struct v4l2_subdev_core_ops pp_core_ops = {
	.ioctl = pp_ioctl,
};

static struct media_entity_operations pp_entity_ops = {
	.link_setup = pp_link_setup,
	.link_validate = v4l2_subdev_link_validate,
};

static struct v4l2_subdev_ops pp_subdev_ops = {
	.video = &pp_video_ops,
	.pad = &pp_pad_ops,
	.core = &pp_core_ops,
};

static struct v4l2_subdev_internal_ops pp_internal_ops = {
	.registered = pp_registered,
};

static int pp_init(struct imx_ic_priv *ic_priv)
{
	struct device_node *np = ic_priv->dev->of_node;
	struct pp_priv *priv;

	priv = devm_kzalloc(ic_priv->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	ic_priv->task_priv = priv;
	priv->ic_priv = ic_priv;
	spin_lock_init(&priv->irqlock);

	/* get our PP id */
	of_property_read_u32(np, "reg", &priv->pp_id);

	snprintf(ic_priv->sd.name, sizeof(ic_priv->sd.name), "%s%d",
		 np->name, priv->pp_id);

	return pp_init_controls(priv);
}

static void pp_remove(struct imx_ic_priv *ic_priv)
{
}

struct imx_ic_ops imx_ic_pp_ops = {
	.subdev_ops = &pp_subdev_ops,
	.internal_ops = &pp_internal_ops,
	.entity_ops = &pp_entity_ops,
	.init = pp_init,
	.remove = pp_remove,
};
