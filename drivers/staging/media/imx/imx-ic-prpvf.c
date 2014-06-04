/*
 * V4L2 IC Deinterlacer Subdev for Freescale i.MX5/6 SOC
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
#include "imx-media.h"
#include "imx-ic.h"

/*
 * This subdev implements two different video pipelines:
 *
 * CSI -> VDIC -> IC -> CH21 -> MEM
 *
 * In this pipeline, the CSI sends a single interlaced field F(n-1)
 * directly to the VDIC (and optionally the following field F(n)
 * can be sent to memory via IDMAC channel 13). So only two fields
 * can be processed by the VDIC. This pipeline only works in VDIC's
 * high motion mode, which only requires a single field for processing.
 * The other motion modes (low and medium) require three fields, so this
 * pipeline does not work in those modes. Also, it is not clear how this
 * pipeline can deal with the various field orders (sequential BT/TB,
 * interlaced BT/TB).
 *
 * CSI -> CH[0-3] -> MEM -> CH8,9,10 -> VDIC -> IC -> CH21 -> MEM
 *
 * In this pipeline, the CSI sends raw and full frames to memory buffers
 * via the SMFC channels 0-3. Fields from these frames are then
 * transferred to the VDIC via IDMAC channels 8,9,10. The VDIC requires
 * three fields: previous field F(n-1), current field F(n), and next
 * field F(n+1), so we need three raw frames in memory: two completed frames
 * to send F(n-1), F(n), F(n+1) to the VDIC, and a third frame for active
 * CSI capture while the completed fields are sent through the VDIC->IC for
 * processing.
 *
 * The "direct" CSI->VDIC pipeline requires less memory bus bandwidth
 * (just 1 channel vs. 5 channels for indirect pipeline), but it can
 * only be used in high motion mode, and it only processes a single
 * field (so half the original image resolution is lost).
 */

struct prpvf_priv;

struct prpvf_pipeline_ops {
	int (*setup)(struct prpvf_priv *priv);
	void (*start)(struct prpvf_priv *priv);
	void (*stop)(struct prpvf_priv *priv);
	void (*disable)(struct prpvf_priv *priv);
};

struct prpvf_field_addr {
	dma_addr_t prev; /* F(n-1) */
	dma_addr_t curr; /* F(n) */
	dma_addr_t next; /* F(n+1) */
};

#define PRPVF_NUM_PADS 2

#define MAX_W_IC   1024
#define MAX_H_IC   1024
#define MAX_W_VDIC  968
#define MAX_H_VDIC 2048

struct prpvf_priv {
	struct imx_media_dev *md;
	struct imx_ic_priv *ic_priv;

	/* IPU units we require */
	struct ipu_ic *ic_vf;
	struct ipu_vdi *vdi;

	struct media_pad pad[PRPVF_NUM_PADS];
	int input_pad;
	int output_pad;

	struct ipuv3_channel *vdi_in_ch_p; /* F(n-1) transfer channel */
	struct ipuv3_channel *vdi_in_ch;   /* F(n) transfer channel */
	struct ipuv3_channel *vdi_in_ch_n; /* F(n+1) transfer channel */
	struct ipuv3_channel *prpvf_out_ch;/* final progressive frame channel */

	/* pipeline operations */
	struct prpvf_pipeline_ops *ops;

	/* our dma buffer sink ring */
	struct imx_media_dma_buf_ring *sink_ring;
	/* the dma buffer ring to send to sink */
	struct imx_media_dma_buf_ring *src_ring;
	int ipu_buf_num;

	struct v4l2_subdev *src_sd;
	/* the sink that will receive the progressive out buffers */
	struct v4l2_subdev *sink_sd;

	struct prpvf_field_addr field[3];

	/* the video standard from sensor at time of streamon */
	v4l2_std_id std;

	struct v4l2_mbus_framefmt format_mbus[PRPVF_NUM_PADS];
	const struct imx_media_pixfmt *cc[PRPVF_NUM_PADS];

	bool csi_direct;  /* using direct CSI->VDIC->IC pipeline */

	/* motion select control */
	struct v4l2_ctrl_handler ctrl_hdlr;
	enum ipu_motion_sel motion;

	bool stream_on;    /* streaming is on */
	bool stop;         /* streaming is stopping */
	spinlock_t irqlock;

	struct timer_list eof_timeout_timer;

	int nfb4eof_irq; /* CSI or PRPVF channel NFB4EOF IRQ */
	int out_eof_irq; /* PRPVF channel EOF IRQ */

	bool last_eof;  /* waiting for last EOF at stream off */
	struct completion last_eof_comp;
};

static inline struct prpvf_priv *sd_to_priv(struct v4l2_subdev *sd)
{
	struct imx_ic_priv *ic_priv = v4l2_get_subdevdata(sd);

	return ic_priv->task_priv;
}

static void prpvf_put_ipu_resources(struct prpvf_priv *priv)
{
	if (!IS_ERR_OR_NULL(priv->ic_vf))
		ipu_ic_put(priv->ic_vf);
	priv->ic_vf = NULL;

	if (!IS_ERR_OR_NULL(priv->vdi_in_ch_p))
		ipu_idmac_put(priv->vdi_in_ch_p);
	priv->vdi_in_ch_p = NULL;

	if (!IS_ERR_OR_NULL(priv->vdi_in_ch))
		ipu_idmac_put(priv->vdi_in_ch);
	priv->vdi_in_ch = NULL;

	if (!IS_ERR_OR_NULL(priv->vdi_in_ch_n))
		ipu_idmac_put(priv->vdi_in_ch_n);
	priv->vdi_in_ch_n = NULL;

	if (!IS_ERR_OR_NULL(priv->prpvf_out_ch))
		ipu_idmac_put(priv->prpvf_out_ch);
	priv->prpvf_out_ch = NULL;

	if (!IS_ERR_OR_NULL(priv->vdi))
		ipu_vdi_put(priv->vdi);
	priv->vdi = NULL;
}

static int prpvf_get_ipu_resources(struct prpvf_priv *priv)
{
	struct imx_ic_priv *ic_priv = priv->ic_priv;
	int ret, err_chan;

	priv->ic_vf = ipu_ic_get(ic_priv->ipu, IC_TASK_VIEWFINDER);
	if (IS_ERR(priv->ic_vf)) {
		v4l2_err(&ic_priv->sd, "failed to get IC VF\n");
		ret = PTR_ERR(priv->ic_vf);
		goto out;
	}

	priv->vdi = ipu_vdi_get(ic_priv->ipu);
	if (IS_ERR(priv->vdi)) {
		v4l2_err(&ic_priv->sd, "failed to get VDIC\n");
		ret = PTR_ERR(priv->vdi);
		goto out;
	}

	priv->prpvf_out_ch = ipu_idmac_get(ic_priv->ipu,
					   IPUV3_CHANNEL_IC_PRP_VF_MEM);
	if (IS_ERR(priv->prpvf_out_ch)) {
		err_chan = IPUV3_CHANNEL_IC_PRP_VF_MEM;
		ret = PTR_ERR(priv->prpvf_out_ch);
		goto out_err_chan;
	}

	if (!priv->csi_direct) {
		priv->vdi_in_ch_p = ipu_idmac_get(ic_priv->ipu,
						  IPUV3_CHANNEL_MEM_VDI_PREV);
		if (IS_ERR(priv->vdi_in_ch_p)) {
			err_chan = IPUV3_CHANNEL_MEM_VDI_PREV;
			ret = PTR_ERR(priv->vdi_in_ch_p);
			goto out_err_chan;
		}

		priv->vdi_in_ch = ipu_idmac_get(ic_priv->ipu,
						IPUV3_CHANNEL_MEM_VDI_CUR);
		if (IS_ERR(priv->vdi_in_ch)) {
			err_chan = IPUV3_CHANNEL_MEM_VDI_CUR;
			ret = PTR_ERR(priv->vdi_in_ch);
			goto out_err_chan;
		}

		priv->vdi_in_ch_n = ipu_idmac_get(ic_priv->ipu,
						  IPUV3_CHANNEL_MEM_VDI_NEXT);
		if (IS_ERR(priv->vdi_in_ch_n)) {
			err_chan = IPUV3_CHANNEL_MEM_VDI_NEXT;
			ret = PTR_ERR(priv->vdi_in_ch_n);
			goto out_err_chan;
		}
	}

	return 0;

out_err_chan:
	v4l2_err(&ic_priv->sd, "could not get IDMAC channel %u\n", err_chan);
out:
	prpvf_put_ipu_resources(priv);
	return ret;
}

static void prepare_vdi_in_buffers(struct prpvf_priv *priv,
				   struct imx_media_dma_buf *buf)
{
	int last_frame, curr_frame;

	curr_frame = buf->index;
	last_frame = curr_frame - 1;
	if (last_frame < 0)
		last_frame = 2;

	ipu_cpmem_set_buffer(priv->vdi_in_ch_p, 0,
			     priv->field[last_frame].prev);
	ipu_cpmem_set_buffer(priv->vdi_in_ch,   0,
			     priv->field[curr_frame].curr);
	ipu_cpmem_set_buffer(priv->vdi_in_ch_n, 0,
			     priv->field[curr_frame].next);

	ipu_idmac_select_buffer(priv->vdi_in_ch_p, 0);
	ipu_idmac_select_buffer(priv->vdi_in_ch, 0);
	ipu_idmac_select_buffer(priv->vdi_in_ch_n, 0);
}

static void prepare_prpvf_out_buffer(struct prpvf_priv *priv)
{
	struct imx_media_dma_buf *buf;

	/* get next buffer to prepare */
	buf = imx_media_dma_buf_next_in(priv->src_ring);

	ipu_cpmem_set_buffer(priv->prpvf_out_ch, priv->ipu_buf_num, buf->phys);
	ipu_idmac_select_buffer(priv->prpvf_out_ch, priv->ipu_buf_num);
}

/* prpvf_out_ch EOF interrupt (progressive frame ready) */
static irqreturn_t prpvf_out_eof_interrupt(int irq, void *dev_id)
{
	struct prpvf_priv *priv = dev_id;
	unsigned long flags;

	spin_lock_irqsave(&priv->irqlock, flags);

	if (priv->last_eof) {
		complete(&priv->last_eof_comp);
		priv->last_eof = false;
		goto unlock;
	}

	/* push the completed buffer to the sink */
	imx_media_dma_buf_return(priv->src_ring, IMX_MEDIA_BUF_STATUS_DONE);

	/* bump the EOF timeout timer */
	mod_timer(&priv->eof_timeout_timer,
		  jiffies + msecs_to_jiffies(IMX_MEDIA_EOF_TIMEOUT));

	if (priv->csi_direct)
		prepare_prpvf_out_buffer(priv);

	/* toggle IPU double-buffer index */
	priv->ipu_buf_num ^= 1;

	/* TODO!
	if (dev->fim.eof && dev->fim.eof(dev, &cur_ts))
		v4l2_subdev_notify(&ic_priv->sd, IMX_MEDIA_FRAME_INTERVAL_NOTIFY,
				   NULL);
	*/

unlock:
	spin_unlock_irqrestore(&priv->irqlock, flags);
	return IRQ_HANDLED;
}

static long prpvf_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct prpvf_priv *priv = sd_to_priv(sd);
	struct imx_media_dma_buf_ring **ring;
	struct imx_media_dma_buf *buf;
	unsigned long flags;

	switch (cmd) {
	case IMX_MEDIA_REQ_DMA_BUF_RING:
		if (!priv->sink_ring)
			return -EINVAL;
		ring = (struct imx_media_dma_buf_ring **)arg;
		*ring = priv->sink_ring;
		break;
	case IMX_MEDIA_NEW_DMA_BUF:
		spin_lock_irqsave(&priv->irqlock, flags);
		if (!priv->stop) {
			buf = imx_media_dma_buf_next_out(priv->sink_ring);
			prepare_vdi_in_buffers(priv, buf);
			prepare_prpvf_out_buffer(priv);
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

static irqreturn_t nfb4eof_interrupt(int irq, void *dev_id)
{
	struct prpvf_priv *priv = dev_id;
	struct imx_ic_priv *ic_priv = priv->ic_priv;
	static const struct v4l2_event ev = {
		.type = V4L2_EVENT_IMX_NFB4EOF,
	};

	v4l2_err(&ic_priv->sd, "NFB4EOF\n");

	v4l2_subdev_notify_event(&ic_priv->sd, &ev);

	return IRQ_HANDLED;
}

/*
 * EOF timeout timer function.
 */
static void prpvf_eof_timeout(unsigned long data)
{
	struct prpvf_priv *priv = (struct prpvf_priv *)data;
	struct imx_ic_priv *ic_priv = priv->ic_priv;
	static const struct v4l2_event ev = {
		.type = V4L2_EVENT_IMX_EOF_TIMEOUT,
	};

	v4l2_err(&ic_priv->sd, "EOF timeout\n");

	v4l2_subdev_notify_event(&ic_priv->sd, &ev);
}

static void setup_vdi_channel(struct prpvf_priv *priv,
			      struct ipuv3_channel *channel,
			      dma_addr_t phys0, dma_addr_t phys1,
			      bool out_chan)
{
	struct v4l2_mbus_framefmt *infmt, *outfmt;
	unsigned int burst_size;
	struct ipu_image image;

	infmt = &priv->format_mbus[priv->input_pad];
	outfmt = &priv->format_mbus[priv->output_pad];

	if (out_chan) {
		imx_media_mbus_fmt_to_ipu_image(&image, outfmt);
	} else {
		/* one field to VDIC channels */
		infmt->height /= 2;
		imx_media_mbus_fmt_to_ipu_image(&image, infmt);
		infmt->height *= 2;
	}
	image.phys0 = phys0;
	image.phys1 = phys1;

	ipu_cpmem_zero(channel);
	ipu_cpmem_set_image(channel, &image);

	if (out_chan) {
		burst_size = (outfmt->width & 0xf) ? 8 : 16;
		ipu_cpmem_set_burstsize(channel, burst_size);
		ipu_ic_task_idma_init(priv->ic_vf, channel,
				      outfmt->width, outfmt->height,
				      burst_size, IPU_ROTATE_NONE);
	} else {
		burst_size = (infmt->width & 0xf) ? 8 : 16;
		ipu_cpmem_set_burstsize(channel, burst_size);
	}

	ipu_cpmem_set_axi_id(channel, 1);

	ipu_idmac_set_double_buffer(channel, out_chan);
}

static int prpvf_setup_direct(struct prpvf_priv *priv)
{
	struct imx_media_dma_buf *buf0, *buf1;

	priv->ipu_buf_num = 0;

	buf0 = imx_media_dma_buf_next_in(priv->src_ring);
	buf1 = imx_media_dma_buf_next_in(priv->src_ring);

	/* init the prpvf out channel */
	setup_vdi_channel(priv, priv->prpvf_out_ch,
			  buf0->phys, buf1->phys, true);

	return 0;
}

static void prpvf_start_direct(struct prpvf_priv *priv)
{
	/* set buffers ready */
	ipu_idmac_select_buffer(priv->prpvf_out_ch, 0);
	ipu_idmac_select_buffer(priv->prpvf_out_ch, 1);

	/* enable the channels */
	ipu_idmac_enable_channel(priv->prpvf_out_ch);
}

static void prpvf_stop_direct(struct prpvf_priv *priv)
{
	ipu_idmac_disable_channel(priv->prpvf_out_ch);
}

static void prpvf_disable_direct(struct prpvf_priv *priv)
{
	/* nothing to do */
}

static int prpvf_setup_indirect(struct prpvf_priv *priv)
{
	struct imx_ic_priv *ic_priv = priv->ic_priv;
	struct v4l2_mbus_framefmt *infmt;
	const struct imx_media_pixfmt *incc;
	struct prpvf_field_addr *field;
	struct imx_media_dma_buf *frame;
	u32 in_stride, field_size, fieldtype;
	int in_size, i, ret;

	infmt = &priv->format_mbus[priv->input_pad];
	incc = priv->cc[priv->input_pad];

	in_size = (infmt->width * incc->bpp * infmt->height) >> 3;

	/* 1/2 full image size */
	field_size = in_size / 2;
	in_stride = incc->planar ?
		infmt->width : (infmt->width * incc->bpp) >> 3;

	priv->ipu_buf_num = 0;

	if (priv->sink_ring) {
		v4l2_warn(&ic_priv->sd, "%s: dma-buf ring was not freed\n",
			  __func__);
		imx_media_free_dma_buf_ring(priv->sink_ring);
	}

	priv->sink_ring =
		imx_media_alloc_dma_buf_ring(priv->md,
					     priv->src_sd, &ic_priv->sd,
					     in_size, 3, true);
	if (IS_ERR(priv->sink_ring)) {
		v4l2_err(&ic_priv->sd, "failed to alloc dma-buf ring\n");
		ret = PTR_ERR(priv->sink_ring);
		priv->sink_ring = NULL;
		return ret;
	}

	/* translate V4L2_FIELD_ALTERNATE to SEQ_TB or SEQ_BT */
	fieldtype = infmt->field;
	if (infmt->field == V4L2_FIELD_ALTERNATE)
		fieldtype = (priv->std & V4L2_STD_525_60) ?
			V4L2_FIELD_SEQ_TB : V4L2_FIELD_SEQ_BT;

	for (i = 0; i < 3; i++) {
		frame = imx_media_dma_buf_peek(priv->sink_ring, i);
		/* precalculate the field addresses for this frame */
		field = &priv->field[i];
		switch (fieldtype) {
		case V4L2_FIELD_SEQ_TB:
			field->prev = frame->phys;
			field->curr = frame->phys + field_size;
			field->next = frame->phys;
			break;
		case V4L2_FIELD_SEQ_BT:
			field->prev = frame->phys + field_size;
			field->curr = frame->phys;
			field->next = frame->phys + field_size;
			break;
		case V4L2_FIELD_INTERLACED_BT:
			field->prev = frame->phys + in_stride;
			field->curr = frame->phys;
			field->next = frame->phys + in_stride;
			break;
		default:
			/* assume V4L2_FIELD_INTERLACED_TB */
			field->prev = frame->phys;
			field->curr = frame->phys + in_stride;
			field->next = frame->phys;
			break;
		}
	}

	/* init the vdi-in channels */
	setup_vdi_channel(priv, priv->vdi_in_ch_p, 0, 0, false);
	setup_vdi_channel(priv, priv->vdi_in_ch, 0, 0, false);
	setup_vdi_channel(priv, priv->vdi_in_ch_n, 0, 0, false);

	/* init the prpvf out channel */
	setup_vdi_channel(priv, priv->prpvf_out_ch, 0, 0, true);

	return 0;
}

static void prpvf_start_indirect(struct prpvf_priv *priv)
{
	/* enable the channels */
	ipu_idmac_enable_channel(priv->prpvf_out_ch);
	ipu_idmac_enable_channel(priv->vdi_in_ch_p);
	ipu_idmac_enable_channel(priv->vdi_in_ch);
	ipu_idmac_enable_channel(priv->vdi_in_ch_n);
}

static void prpvf_stop_indirect(struct prpvf_priv *priv)
{
	/* disable channels */
	ipu_idmac_disable_channel(priv->prpvf_out_ch);
	ipu_idmac_disable_channel(priv->vdi_in_ch_p);
	ipu_idmac_disable_channel(priv->vdi_in_ch);
	ipu_idmac_disable_channel(priv->vdi_in_ch_n);
}

static void prpvf_disable_indirect(struct prpvf_priv *priv)
{
}

static struct prpvf_pipeline_ops direct_ops = {
	.setup = prpvf_setup_direct,
	.start = prpvf_start_direct,
	.stop = prpvf_stop_direct,
	.disable = prpvf_disable_direct,
};

static struct prpvf_pipeline_ops indirect_ops = {
	.setup = prpvf_setup_indirect,
	.start = prpvf_start_indirect,
	.stop = prpvf_stop_indirect,
	.disable = prpvf_disable_indirect,
};

static int prpvf_start(struct prpvf_priv *priv)
{
	struct imx_ic_priv *ic_priv = priv->ic_priv;
	struct v4l2_mbus_framefmt *infmt, *outfmt;
	const struct imx_media_pixfmt *outcc, *incc;
	struct imx_media_subdev *sensor;
	int ret;

	infmt = &priv->format_mbus[priv->input_pad];
	outfmt = &priv->format_mbus[priv->output_pad];
	incc = priv->cc[priv->input_pad];
	outcc = priv->cc[priv->output_pad];

	priv->ops = priv->csi_direct ? &direct_ops : &indirect_ops;

	ret = prpvf_get_ipu_resources(priv);
	if (ret)
		return ret;

	/* ask the sink for the buffer ring */
	ret = v4l2_subdev_call(priv->sink_sd, core, ioctl,
			       IMX_MEDIA_REQ_DMA_BUF_RING,
			       &priv->src_ring);
	if (ret)
		goto out_put_ipu;

	/* init EOF completion waitq */
	init_completion(&priv->last_eof_comp);
	priv->last_eof = false;

	/* request EOF irq for prpvf out channel */
	priv->out_eof_irq = ipu_idmac_channel_irq(ic_priv->ipu,
						  priv->prpvf_out_ch,
						  IPU_IRQ_EOF);
	ret = devm_request_irq(ic_priv->dev, priv->out_eof_irq,
			       prpvf_out_eof_interrupt, 0,
			       "imx-ic-prpvf-out-eof", priv);
	if (ret) {
		v4l2_err(&ic_priv->sd,
			 "Error registering out eof irq: %d\n", ret);
		goto out_put_ipu;
	}

	/* request NFB4EOF irq */
	priv->nfb4eof_irq = ipu_idmac_channel_irq(ic_priv->ipu,
						  priv->prpvf_out_ch,
						  IPU_IRQ_NFB4EOF);
	ret = devm_request_irq(ic_priv->dev, priv->nfb4eof_irq,
			       nfb4eof_interrupt, 0,
			       "imx-ic-prpvf-nfb4eof", priv);
	if (ret) {
		v4l2_err(&ic_priv->sd,
			 "Error registering NFB4EOF irq: %d\n", ret);
		goto out_free_eof_irq;
	}

	sensor = priv->md->sensor;
	if (!sensor) {
		v4l2_err(&ic_priv->sd, "no sensor attached\n");
		ret = -EINVAL;
		goto out_free_nfb4eof_irq;
	}

	ret = v4l2_subdev_call(sensor->sd, video, g_std, &priv->std);
	if (ret)
		goto out_free_nfb4eof_irq;

	/* init the VDIC */
	ipu_vdi_setup(priv->vdi, infmt->code,
		      infmt->width, infmt->height);
	ipu_vdi_set_field_order(priv->vdi, priv->std, infmt->field);
	ipu_vdi_set_motion(priv->vdi, priv->motion);

	ret = ipu_ic_task_init(priv->ic_vf,
			       infmt->width, infmt->height,
			       outfmt->width, outfmt->height,
			       incc->cs, outcc->cs);
	if (ret) {
		v4l2_err(&ic_priv->sd, "ipu_ic_task_init failed, %d\n", ret);
		goto out_free_nfb4eof_irq;
	}

	ret = priv->ops->setup(priv);
	if (ret)
		goto out_free_nfb4eof_irq;

	ipu_vdi_enable(priv->vdi);
	ipu_ic_enable(priv->ic_vf);

	priv->ops->start(priv);

	/* enable the IC VF task */
	ipu_ic_task_enable(priv->ic_vf);

	/* start the EOF timeout timer */
	mod_timer(&priv->eof_timeout_timer,
		  jiffies + msecs_to_jiffies(IMX_MEDIA_EOF_TIMEOUT));

	priv->stop = false;

	return 0;

out_free_nfb4eof_irq:
	devm_free_irq(ic_priv->dev, priv->nfb4eof_irq, priv);
out_free_eof_irq:
	devm_free_irq(ic_priv->dev, priv->out_eof_irq, priv);
out_put_ipu:
	prpvf_put_ipu_resources(priv);
	return ret;
}

static int prpvf_stop(struct prpvf_priv *priv)
{
	struct imx_ic_priv *ic_priv = priv->ic_priv;
	unsigned long flags;
	int ret;

	/* mark next EOF interrupt as the last before stream off */
	spin_lock_irqsave(&priv->irqlock, flags);
	priv->last_eof = true;
	priv->stop = true;
	spin_unlock_irqrestore(&priv->irqlock, flags);

	/*
	 * and then wait for interrupt handler to mark completion.
	 */
	ret = wait_for_completion_timeout(
		&priv->last_eof_comp, msecs_to_jiffies(IMX_MEDIA_EOF_TIMEOUT));
	if (ret == 0)
		v4l2_warn(&ic_priv->sd, "wait last EOF timeout\n");

	ipu_ic_task_disable(priv->ic_vf);
	priv->ops->stop(priv);
	ipu_ic_disable(priv->ic_vf);
	ipu_vdi_disable(priv->vdi);
	priv->ops->disable(priv);
	if (priv->csi_direct)
		ipu_fsu_unlink(ic_priv->ipu, IPUV3_CHANNEL_CSI_DIRECT,
			       IPUV3_CHANNEL_CSI_VDI_PREV);
	devm_free_irq(ic_priv->dev, priv->nfb4eof_irq, priv);
	devm_free_irq(ic_priv->dev, priv->out_eof_irq, priv);
	prpvf_put_ipu_resources(priv);

	/* cancel the EOF timeout timer */
	del_timer_sync(&priv->eof_timeout_timer);

	priv->src_ring = NULL;

	/* inform sink that the buffer ring can now be freed */
	return v4l2_subdev_call(priv->sink_sd, core, ioctl,
				IMX_MEDIA_REL_DMA_BUF_RING, 0);
}

static int prpvf_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct prpvf_priv *priv = container_of(ctrl->handler,
					       struct prpvf_priv, ctrl_hdlr);
	struct imx_ic_priv *ic_priv = priv->ic_priv;
	enum ipu_motion_sel motion;

	switch (ctrl->id) {
	case V4L2_CID_IMX_MOTION:
		motion = ctrl->val;
		if (motion != priv->motion) {
			/* can't change motion control mid-streaming */
			if (priv->stream_on)
				return -EBUSY;
			priv->motion = motion;
		}
		break;
	default:
		v4l2_err(&ic_priv->sd, "Invalid control\n");
		return -EINVAL;
	}

	return 0;
}

static const struct v4l2_ctrl_ops prpvf_ctrl_ops = {
	.s_ctrl = prpvf_s_ctrl,
};

static const struct v4l2_ctrl_config prpvf_custom_ctrl[] = {
	{
		.ops = &prpvf_ctrl_ops,
		.id = V4L2_CID_IMX_MOTION,
		.name = "Motion Compensation",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.def = MOTION_NONE,
		.min = MOTION_NONE,
		.max = HIGH_MOTION,
		.step = 1,
	},
};

#define PRPVF_NUM_CONTROLS ARRAY_SIZE(prpvf_custom_ctrl)

static int prpvf_init_controls(struct prpvf_priv *priv)
{
	struct imx_ic_priv *ic_priv = priv->ic_priv;
	struct v4l2_ctrl_handler *hdlr = &priv->ctrl_hdlr;
	const struct v4l2_ctrl_config *c;
	int i, ret;

	v4l2_ctrl_handler_init(hdlr, PRPVF_NUM_CONTROLS);

	for (i = 0; i < PRPVF_NUM_CONTROLS; i++) {
		c = &prpvf_custom_ctrl[i];
		v4l2_ctrl_new_custom(hdlr, c, NULL);
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

static int prpvf_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct prpvf_priv *priv = sd_to_priv(sd);
	int ret = 0;

	v4l2_info(sd, "stream %s\n", enable ? "ON" : "OFF");

	if (enable && !priv->stream_on)
		ret = prpvf_start(priv);
	else if (!enable && priv->stream_on)
		ret = prpvf_stop(priv);

	if (!ret)
		priv->stream_on = enable;
	return ret;
}

static int prpvf_enum_mbus_code(struct v4l2_subdev *sd,
				struct v4l2_subdev_pad_config *cfg,
				struct v4l2_subdev_mbus_code_enum *code)
{
	struct prpvf_priv *priv = sd_to_priv(sd);
	bool allow_planar, allow_rgb;

	if (code->pad >= PRPVF_NUM_PADS)
		return -EINVAL;

	allow_planar = (code->pad == priv->output_pad);
	allow_rgb = allow_planar;

	return imx_media_enum_format(&code->code, code->index,
				     allow_rgb, allow_planar);
}

static int prpvf_get_fmt(struct v4l2_subdev *sd,
			 struct v4l2_subdev_pad_config *cfg,
			 struct v4l2_subdev_format *sdformat)
{
	struct prpvf_priv *priv = sd_to_priv(sd);

	if (sdformat->pad >= PRPVF_NUM_PADS)
		return -EINVAL;

	sdformat->format = priv->format_mbus[sdformat->pad];

	return 0;
}

static int prpvf_set_fmt(struct v4l2_subdev *sd,
			 struct v4l2_subdev_pad_config *cfg,
			 struct v4l2_subdev_format *sdformat)
{
	struct prpvf_priv *priv = sd_to_priv(sd);
	struct v4l2_mbus_framefmt *infmt, *outfmt;
	const struct imx_media_pixfmt *cc;
	bool allow_planar, allow_rgb;
	u32 code;

	if (sdformat->pad >= PRPVF_NUM_PADS)
		return -EINVAL;

	if (priv->stream_on)
		return -EBUSY;

	infmt = &priv->format_mbus[priv->input_pad];
	outfmt = &priv->format_mbus[priv->output_pad];
	allow_planar = (sdformat->pad == priv->output_pad);
	allow_rgb = allow_planar;

	cc = imx_media_find_format(0, sdformat->format.code,
				   allow_rgb, allow_planar);
	if (!cc) {
		imx_media_enum_format(&code, 0, false, false);
		cc = imx_media_find_format(0, code, false, false);
		sdformat->format.code = cc->codes[0];
	}

	if (sdformat->pad == priv->output_pad) {
		sdformat->format.width = min_t(__u32,
					       sdformat->format.width,
					       MAX_W_IC);
		sdformat->format.height = min_t(__u32,
						sdformat->format.height,
						MAX_H_IC);
		/* IC resizer cannot downsize more than 4:1 */
		sdformat->format.width = max_t(__u32, sdformat->format.width,
					       infmt->width / 4);
                sdformat->format.height = max_t(__u32, sdformat->format.height,
						infmt->height / 4);

		/* output is always progressive! */
		sdformat->format.field = V4L2_FIELD_NONE;
	} else {
		sdformat->format.width = min_t(__u32,
					       sdformat->format.width,
					       MAX_W_VDIC);
		sdformat->format.height = min_t(__u32,
						sdformat->format.height,
						MAX_H_VDIC);

		/* input must be interlaced! Choose alternate if not */
		if (!V4L2_FIELD_HAS_BOTH(sdformat->format.field))
			sdformat->format.field = V4L2_FIELD_ALTERNATE;
	}

	if (sdformat->which == V4L2_SUBDEV_FORMAT_TRY) {
		cfg->try_fmt = sdformat->format;
	} else { 
		priv->format_mbus[sdformat->pad] = sdformat->format;
		priv->cc[sdformat->pad] = cc;
	}

	return 0;
}

static int prpvf_link_setup(struct media_entity *entity,
			    const struct media_pad *local,
			    const struct media_pad *remote, u32 flags)
{
	struct v4l2_subdev *sd = media_entity_to_v4l2_subdev(entity);
	struct imx_ic_priv *ic_priv = v4l2_get_subdevdata(sd);
	struct prpvf_priv *priv = ic_priv->task_priv;
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

		return 0;
	}

	/* this is sink pad */
	if (flags & MEDIA_LNK_FL_ENABLED) {
		if (priv->src_sd)
			return -EBUSY;
		priv->src_sd = remote_sd;
	} else {
		priv->src_sd = NULL;
		return 0;
	}

	/* set IC to receive from VDIC */
	ipu_set_ic_src_mux(ic_priv->ipu, 0, true);

	return 0;
}

static int prpvf_link_validate(struct v4l2_subdev *sd,
			       struct media_link *link,
			       struct v4l2_subdev_format *source_fmt,
			       struct v4l2_subdev_format *sink_fmt)
{
	struct imx_ic_priv *ic_priv = v4l2_get_subdevdata(sd);
	struct prpvf_priv *priv = ic_priv->task_priv;
	struct v4l2_mbus_config sensor_mbus_cfg;
	struct v4l2_of_endpoint *sensor_ep;
	struct imx_media_subdev *sensor;
	int ret;

	ret = v4l2_subdev_link_validate_default(sd, link, source_fmt, sink_fmt);
	if (ret)
		return ret;

	priv->csi_direct = ((priv->src_sd->grp_id & IMX_MEDIA_GRP_ID_CSI) != 0);

	if (!priv->csi_direct)
		return 0;

	sensor = priv->md->sensor;
	if (!sensor) {
		v4l2_err(&ic_priv->sd, "no sensor attached\n");
		return -EINVAL;
	}

	if (priv->motion != HIGH_MOTION) {
		v4l2_err(&ic_priv->sd,
			 "direct CSI pipeline requires HIGH_MOTION\n");
		return -EINVAL;
	}
		
	sensor_ep = &sensor->pad[0].link[0].local_ep;

	ret = v4l2_subdev_call(sensor->sd, video, g_mbus_config,
			       &sensor_mbus_cfg);
	if (ret)
		return ret;

	if (sensor_mbus_cfg.type == V4L2_MBUS_CSI2) {
		int vc_num = 0;
		/* see NOTE in imx-csi.c */
#if 0
		vc_num = imx_media_find_mipi_csi2_channel(
			priv->md, &ic_priv->sd.entity);
		if (vc_num < 0)
			return vc_num;
#endif
		/* only virtual channel 0 can be sent to IC */
		if (vc_num != 0)
			return -EINVAL;
	} else {
		/* only 8-bit pixels can be sent to IC for parallel busses */
		if (sensor_ep->bus.parallel.bus_width >= 16)
			return -EINVAL;
	}

	/* set VDIC to receive from CSI for direct path */
	ipu_fsu_link(ic_priv->ipu, IPUV3_CHANNEL_CSI_DIRECT,
		     IPUV3_CHANNEL_CSI_VDI_PREV);

	return 0;
}

/*
 * retrieve our pads parsed from the OF graph by the media device
 */
static int prpvf_registered(struct v4l2_subdev *sd)
{
	struct prpvf_priv *priv = sd_to_priv(sd);
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

	for (i = 0; i < PRPVF_NUM_PADS; i++) {
		pad = &imxsd->pad[i];
		priv->pad[i] = pad->pad;
		if (priv->pad[i].flags & MEDIA_PAD_FL_SINK)
			priv->input_pad = i;
		else
			priv->output_pad = i;
	}

	return media_entity_pads_init(&sd->entity, PRPVF_NUM_PADS,
				      priv->pad);
}

static struct v4l2_subdev_pad_ops prpvf_pad_ops = {
	.enum_mbus_code = prpvf_enum_mbus_code,
	.get_fmt = prpvf_get_fmt,
	.set_fmt = prpvf_set_fmt,
	.link_validate = prpvf_link_validate,
};

static struct v4l2_subdev_video_ops prpvf_video_ops = {
	.s_stream = prpvf_s_stream,
};

static struct v4l2_subdev_core_ops prpvf_core_ops = {
	.ioctl = prpvf_ioctl,
};

static struct media_entity_operations prpvf_entity_ops = {
	.link_setup = prpvf_link_setup,
	.link_validate = v4l2_subdev_link_validate,
};

static struct v4l2_subdev_ops prpvf_subdev_ops = {
	.video = &prpvf_video_ops,
	.pad = &prpvf_pad_ops,
	.core = &prpvf_core_ops,
};

static struct v4l2_subdev_internal_ops prpvf_internal_ops = {
	.registered = prpvf_registered,
};

static int prpvf_init(struct imx_ic_priv *ic_priv)
{
	struct device_node *np = ic_priv->dev->of_node;
	struct prpvf_priv *priv;

	priv = devm_kzalloc(ic_priv->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	ic_priv->task_priv = priv;
	priv->ic_priv = ic_priv;

	spin_lock_init(&priv->irqlock);
	init_timer(&priv->eof_timeout_timer);
	priv->eof_timeout_timer.data = (unsigned long)priv;
	priv->eof_timeout_timer.function = prpvf_eof_timeout;
	snprintf(ic_priv->sd.name, sizeof(ic_priv->sd.name), "%s", np->name);

	return prpvf_init_controls(priv);
}

static void prpvf_remove(struct imx_ic_priv *ic_priv)
{
}

struct imx_ic_ops imx_ic_prpvf_ops = {
	.subdev_ops = &prpvf_subdev_ops,
	.internal_ops = &prpvf_internal_ops,
	.entity_ops = &prpvf_entity_ops,
	.init = prpvf_init,
	.remove = prpvf_remove,
};
