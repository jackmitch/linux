/*
 * V4L2 Capture IC Encoder Subdev for Freescale i.MX5/6 SOC
 *
 * This subdevice handles capture of video frames from the CSI, which
 * are routed directly to the Image Converter preprocess encode task,
 * for resizing, colorspace conversion, and rotation.
 *
 * Copyright (c) 2012-2016 Mentor Graphics Inc.
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
#include <linux/spinlock.h>
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

#define PRPENC_NUM_PADS 2

#define MAX_W_IC   1024
#define MAX_H_IC   1024
#define MAX_W_SINK 4096
#define MAX_H_SINK 4096

struct prpenc_priv {
	struct imx_media_dev *md;
	struct imx_ic_priv *ic_priv;

	/* IPU units we require */
	struct ipu_ic *ic_enc;

	struct media_pad pad[PRPENC_NUM_PADS];
	int input_pad;
	int output_pad;

	struct ipuv3_channel *enc_ch;
	struct ipuv3_channel *enc_rot_in_ch;
	struct ipuv3_channel *enc_rot_out_ch;

	/* the dma buffer ring to send to sink */
	struct imx_media_dma_buf_ring *src_ring;

	int ipu_buf_num;  /* ipu double buffer index: 0-1 */

	struct v4l2_subdev *src_sd;
	struct v4l2_subdev *sink_sd;

	struct v4l2_mbus_framefmt format_mbus[PRPENC_NUM_PADS];
	const struct imx_media_pixfmt *cc[PRPENC_NUM_PADS];

	struct imx_media_dma_buf rot_buf[2];

	/* controls */
	struct v4l2_ctrl_handler ctrl_hdlr;
	int  rotation; /* degrees */
	bool hflip;
        bool vflip;

	/* derived from rotation, hflip, vflip controls */
	enum ipu_rotate_mode rot_mode;

	spinlock_t irqlock;

	struct timer_list eof_timeout_timer;
	int eof_irq;
	int nfb4eof_irq;

	bool stream_on; /* streaming is on */
	bool last_eof;  /* waiting for last EOF at stream off */
	struct completion last_eof_comp;
};

static inline struct prpenc_priv *sd_to_priv(struct v4l2_subdev *sd)
{
	struct imx_ic_priv *ic_priv = v4l2_get_subdevdata(sd);

	return ic_priv->task_priv;
}

static void prpenc_put_ipu_resources(struct prpenc_priv *priv)
{
	if (!IS_ERR_OR_NULL(priv->ic_enc))
		ipu_ic_put(priv->ic_enc);
	priv->ic_enc = NULL;

	if (!IS_ERR_OR_NULL(priv->enc_ch))
		ipu_idmac_put(priv->enc_ch);
	priv->enc_ch = NULL;

	if (!IS_ERR_OR_NULL(priv->enc_rot_in_ch))
		ipu_idmac_put(priv->enc_rot_in_ch);
	priv->enc_rot_in_ch = NULL;

	if (!IS_ERR_OR_NULL(priv->enc_rot_out_ch))
		ipu_idmac_put(priv->enc_rot_out_ch);
	priv->enc_rot_out_ch = NULL;
}

static int prpenc_get_ipu_resources(struct prpenc_priv *priv)
{
	struct imx_ic_priv *ic_priv = priv->ic_priv;
	int ret;

	priv->ic_enc = ipu_ic_get(ic_priv->ipu, IC_TASK_ENCODER);
	if (IS_ERR(priv->ic_enc)) {
		v4l2_err(&ic_priv->sd, "failed to get IC ENC\n");
		ret = PTR_ERR(priv->ic_enc);
		goto out;
	}

	priv->enc_ch = ipu_idmac_get(ic_priv->ipu,
				     IPUV3_CHANNEL_IC_PRP_ENC_MEM);
	if (IS_ERR(priv->enc_ch)) {
		v4l2_err(&ic_priv->sd, "could not get IDMAC channel %u\n",
			 IPUV3_CHANNEL_IC_PRP_ENC_MEM);
		ret = PTR_ERR(priv->enc_ch);
		goto out;
	}

	priv->enc_rot_in_ch = ipu_idmac_get(ic_priv->ipu,
					    IPUV3_CHANNEL_MEM_ROT_ENC);
	if (IS_ERR(priv->enc_rot_in_ch)) {
		v4l2_err(&ic_priv->sd, "could not get IDMAC channel %u\n",
			 IPUV3_CHANNEL_MEM_ROT_ENC);
		ret = PTR_ERR(priv->enc_rot_in_ch);
		goto out;
	}

	priv->enc_rot_out_ch = ipu_idmac_get(ic_priv->ipu,
					     IPUV3_CHANNEL_ROT_ENC_MEM);
	if (IS_ERR(priv->enc_rot_out_ch)) {
		v4l2_err(&ic_priv->sd, "could not get IDMAC channel %u\n",
			 IPUV3_CHANNEL_ROT_ENC_MEM);
		ret = PTR_ERR(priv->enc_rot_out_ch);
		goto out;
	}

	return 0;
out:
	prpenc_put_ipu_resources(priv);
	return ret;
}

static irqreturn_t prpenc_eof_interrupt(int irq, void *dev_id)
{
	struct prpenc_priv *priv = dev_id;
	struct imx_media_dma_buf *buf;
	struct ipuv3_channel *channel;
	unsigned long flags;

	spin_lock_irqsave(&priv->irqlock, flags);

	if (priv->last_eof) {
		complete(&priv->last_eof_comp);
		priv->last_eof = false;
		goto unlock;
	}

	/* push the completed buffer to the sink */
	imx_media_dma_buf_return(priv->src_ring, IMX_MEDIA_BUF_STATUS_DONE);

	/* TODO!
	if (dev->fim.eof && dev->fim.eof(dev, &cur_ts))
		v4l2_subdev_notify(&ic_priv->sd, IMX_MEDIA_FRAME_INTERVAL_NOTIFY,
				   NULL);
	*/

	/* bump the EOF timeout timer */
	mod_timer(&priv->eof_timeout_timer,
		  jiffies + msecs_to_jiffies(IMX_MEDIA_EOF_TIMEOUT));

	channel = (ipu_rot_mode_is_irt(priv->rot_mode)) ?
		priv->enc_rot_out_ch : priv->enc_ch;

	/* get next buffer to prepare for capture */
	buf = imx_media_dma_buf_next_in(priv->src_ring);

	if (ipu_idmac_buffer_is_ready(channel, priv->ipu_buf_num))
		ipu_idmac_clear_buffer(channel, priv->ipu_buf_num);

	ipu_cpmem_set_buffer(channel, priv->ipu_buf_num, buf->phys);
	ipu_idmac_select_buffer(channel, priv->ipu_buf_num);

	/* toggle IPU double-buffer index */
	priv->ipu_buf_num ^= 1;

unlock:
	spin_unlock_irqrestore(&priv->irqlock, flags);
	return IRQ_HANDLED;
}

static irqreturn_t prpenc_nfb4eof_interrupt(int irq, void *dev_id)
{
	struct prpenc_priv *priv = dev_id;
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
static void prpenc_eof_timeout(unsigned long data)
{
	struct prpenc_priv *priv = (struct prpenc_priv *)data;
	struct imx_ic_priv *ic_priv = priv->ic_priv;
	static const struct v4l2_event ev = {
		.type = V4L2_EVENT_IMX_EOF_TIMEOUT,
	};

	v4l2_err(&ic_priv->sd, "EOF timeout\n");

	v4l2_subdev_notify_event(&ic_priv->sd, &ev);
}

static void prpenc_setup_channel(struct prpenc_priv *priv,
				 struct ipuv3_channel *channel,
				 enum ipu_rotate_mode rot_mode,
				 dma_addr_t addr0, dma_addr_t addr1,
				 bool rot_swap_width_height)
{
	struct v4l2_mbus_framefmt *infmt, *outfmt;
	unsigned int burst_size;
	struct ipu_image image;

	infmt = &priv->format_mbus[priv->input_pad];
	outfmt = &priv->format_mbus[priv->output_pad];

	if (rot_swap_width_height)
		swap(outfmt->width, outfmt->height);

	ipu_cpmem_zero(channel);

	imx_media_mbus_fmt_to_ipu_image(&image, outfmt);

	image.phys0 = addr0;
	image.phys1 = addr1;
	ipu_cpmem_set_image(channel, &image);

	if (channel == priv->enc_rot_in_ch ||
	    channel == priv->enc_rot_out_ch) {
		burst_size = 8;
		ipu_cpmem_set_block_mode(channel);
	} else {
		burst_size = (outfmt->width & 0xf) ? 8 : 16;
	}

	ipu_cpmem_set_burstsize(channel, burst_size);

	if (rot_mode)
		ipu_cpmem_set_rotation(channel, rot_mode);

	if (outfmt->field == V4L2_FIELD_NONE &&
	    (V4L2_FIELD_HAS_BOTH(infmt->field) ||
	     infmt->field == V4L2_FIELD_ALTERNATE) &&
	    channel == priv->enc_ch)
		ipu_cpmem_interlaced_scan(channel, image.pix.bytesperline);

	ipu_ic_task_idma_init(priv->ic_enc, channel,
			      outfmt->width, outfmt->height,
			      burst_size, rot_mode);
	ipu_cpmem_set_axi_id(channel, 1);

	ipu_idmac_set_double_buffer(channel, true);

	if (rot_swap_width_height)
		swap(outfmt->width, outfmt->height);
}

static int prpenc_setup_rotation(struct prpenc_priv *priv)
{
	struct imx_ic_priv *ic_priv = priv->ic_priv;
	struct v4l2_mbus_framefmt *infmt, *outfmt;
	const struct imx_media_pixfmt *outcc, *incc;
	struct imx_media_dma_buf *buf0, *buf1;
	int out_size, ret;

	infmt = &priv->format_mbus[priv->input_pad];
	outfmt = &priv->format_mbus[priv->output_pad];
	incc = priv->cc[priv->input_pad];
	outcc = priv->cc[priv->output_pad];

	out_size = (outfmt->width * outcc->bpp * outfmt->height) >> 3;

	ret = imx_media_alloc_dma_buf(priv->md, &priv->rot_buf[0], out_size);
	if (ret) {
		v4l2_err(&ic_priv->sd, "failed to alloc rot_buf[0], %d\n", ret);
		return ret;
	}
	ret = imx_media_alloc_dma_buf(priv->md, &priv->rot_buf[1], out_size);
	if (ret) {
		v4l2_err(&ic_priv->sd, "failed to alloc rot_buf[1], %d\n", ret);
		goto free_rot0;
	}

	ret = ipu_ic_task_init(priv->ic_enc,
			       infmt->width, infmt->height,
			       outfmt->height, outfmt->width,
			       incc->cs, outcc->cs);
	if (ret) {
		v4l2_err(&ic_priv->sd, "ipu_ic_task_init failed, %d\n", ret);
		goto free_rot1;
	}

	/* init the IC ENC-->MEM IDMAC channel */
	prpenc_setup_channel(priv, priv->enc_ch,
			     IPU_ROTATE_NONE,
			     priv->rot_buf[0].phys,
			     priv->rot_buf[1].phys,
			     true);

	/* init the MEM-->IC ENC ROT IDMAC channel */
	prpenc_setup_channel(priv, priv->enc_rot_in_ch,
			     priv->rot_mode,
			     priv->rot_buf[0].phys,
			     priv->rot_buf[1].phys,
			     true);

	buf0 = imx_media_dma_buf_next_in(priv->src_ring);
	buf1 = imx_media_dma_buf_next_in(priv->src_ring);

	/* init the destination IC ENC ROT-->MEM IDMAC channel */
	prpenc_setup_channel(priv, priv->enc_rot_out_ch,
			     IPU_ROTATE_NONE,
			     buf0->phys, buf1->phys,
			     false);

	/* now link IC ENC-->MEM to MEM-->IC ENC ROT */
	ipu_idmac_link(priv->enc_ch, priv->enc_rot_in_ch);

	/* enable the IC */
	ipu_ic_enable(priv->ic_enc);

	/* set buffers ready */
	ipu_idmac_select_buffer(priv->enc_ch, 0);
	ipu_idmac_select_buffer(priv->enc_ch, 1);
	ipu_idmac_select_buffer(priv->enc_rot_out_ch, 0);
	ipu_idmac_select_buffer(priv->enc_rot_out_ch, 1);

	/* enable the channels */
	ipu_idmac_enable_channel(priv->enc_ch);
	ipu_idmac_enable_channel(priv->enc_rot_in_ch);
	ipu_idmac_enable_channel(priv->enc_rot_out_ch);

	/* and finally enable the IC PRPENC task */
	ipu_ic_task_enable(priv->ic_enc);

	return 0;

free_rot1:
	imx_media_free_dma_buf(priv->md, &priv->rot_buf[1]);
free_rot0:
	imx_media_free_dma_buf(priv->md, &priv->rot_buf[0]);
	return ret;
}

static void prpenc_unsetup_rotation(struct prpenc_priv *priv)
{
	ipu_ic_task_disable(priv->ic_enc);

	ipu_idmac_disable_channel(priv->enc_ch);
	ipu_idmac_disable_channel(priv->enc_rot_in_ch);
	ipu_idmac_disable_channel(priv->enc_rot_out_ch);

	ipu_idmac_unlink(priv->enc_ch, priv->enc_rot_in_ch);

	ipu_ic_disable(priv->ic_enc);

	imx_media_free_dma_buf(priv->md, &priv->rot_buf[0]);
	imx_media_free_dma_buf(priv->md, &priv->rot_buf[1]);
}

static int prpenc_setup_norotation(struct prpenc_priv *priv)
{
	struct imx_ic_priv *ic_priv = priv->ic_priv;
	struct v4l2_mbus_framefmt *infmt, *outfmt;
	const struct imx_media_pixfmt *outcc, *incc;
	struct imx_media_dma_buf *buf0, *buf1;
	int ret;

	infmt = &priv->format_mbus[priv->input_pad];
	outfmt = &priv->format_mbus[priv->output_pad];
	incc = priv->cc[priv->input_pad];
	outcc = priv->cc[priv->output_pad];

	ret = ipu_ic_task_init(priv->ic_enc,
			       infmt->width, infmt->height,
			       outfmt->width, outfmt->height,
			       incc->cs, outcc->cs);
	if (ret) {
		v4l2_err(&ic_priv->sd, "ipu_ic_task_init failed, %d\n", ret);
		return ret;
	}

	buf0 = imx_media_dma_buf_next_in(priv->src_ring);
	buf1 = imx_media_dma_buf_next_in(priv->src_ring);

	/* init the IC PRP-->MEM IDMAC channel */
	prpenc_setup_channel(priv, priv->enc_ch, priv->rot_mode,
			     buf0->phys, buf1->phys,
			     false);

	ipu_cpmem_dump(priv->enc_ch);
	ipu_ic_dump(priv->ic_enc);
	ipu_dump(ic_priv->ipu);

	ipu_ic_enable(priv->ic_enc);

	/* set buffers ready */
	ipu_idmac_select_buffer(priv->enc_ch, 0);
	ipu_idmac_select_buffer(priv->enc_ch, 1);

	/* enable the channels */
	ipu_idmac_enable_channel(priv->enc_ch);

	/* enable the IC ENCODE task */
	ipu_ic_task_enable(priv->ic_enc);

	return 0;
}

static void prpenc_unsetup_norotation(struct prpenc_priv *priv)
{
	ipu_ic_task_disable(priv->ic_enc);
	ipu_idmac_disable_channel(priv->enc_ch);
	ipu_ic_disable(priv->ic_enc);
}

static int prpenc_start(struct prpenc_priv *priv)
{
	struct imx_ic_priv *ic_priv = priv->ic_priv;
	int ret;

	ret = prpenc_get_ipu_resources(priv);
	if (ret)
		return ret;

	/* ask the sink for the buffer ring */
	ret = v4l2_subdev_call(priv->sink_sd, core, ioctl,
			       IMX_MEDIA_REQ_DMA_BUF_RING,
			       &priv->src_ring);
	if (ret)
		goto out_put_ipu;

	priv->ipu_buf_num = 0;

	/* init EOF completion waitq */
	init_completion(&priv->last_eof_comp);
	priv->last_eof = false;

	if (ipu_rot_mode_is_irt(priv->rot_mode))
		ret = prpenc_setup_rotation(priv);
	else
		ret = prpenc_setup_norotation(priv);
	if (ret)
		goto out_put_ipu;

	priv->nfb4eof_irq = ipu_idmac_channel_irq(ic_priv->ipu,
						  priv->enc_ch,
						  IPU_IRQ_NFB4EOF);
	ret = devm_request_irq(ic_priv->dev, priv->nfb4eof_irq,
			       prpenc_nfb4eof_interrupt, 0,
			       "imx-ic-prpenc-nfb4eof", priv);
	if (ret) {
		v4l2_err(&ic_priv->sd,
			 "Error registering NFB4EOF irq: %d\n", ret);
		goto out_unsetup;
	}

	if (ipu_rot_mode_is_irt(priv->rot_mode))
		priv->eof_irq = ipu_idmac_channel_irq(
			ic_priv->ipu, priv->enc_rot_out_ch, IPU_IRQ_EOF);
	else
		priv->eof_irq = ipu_idmac_channel_irq(
			ic_priv->ipu, priv->enc_ch, IPU_IRQ_EOF);

	ret = devm_request_irq(ic_priv->dev, priv->eof_irq,
			       prpenc_eof_interrupt, 0,
			       "imx-ic-prpenc-eof", priv);
	if (ret) {
		v4l2_err(&ic_priv->sd,
			 "Error registering eof irq: %d\n", ret);
		goto out_free_nfb4eof_irq;
	}

	/* start the EOF timeout timer */
	mod_timer(&priv->eof_timeout_timer,
		  jiffies + msecs_to_jiffies(IMX_MEDIA_EOF_TIMEOUT));

	return 0;

out_free_nfb4eof_irq:
	devm_free_irq(ic_priv->dev, priv->nfb4eof_irq, priv);
out_unsetup:
	if (ipu_rot_mode_is_irt(priv->rot_mode))
		prpenc_unsetup_rotation(priv);
	else
		prpenc_unsetup_norotation(priv);
out_put_ipu:
	prpenc_put_ipu_resources(priv);
	return ret;
}

static int prpenc_stop(struct prpenc_priv *priv)
{
	struct imx_ic_priv *ic_priv = priv->ic_priv;
	unsigned long flags;
	int ret;

	/* mark next EOF interrupt as the last before stream off */
	spin_lock_irqsave(&priv->irqlock, flags);
	priv->last_eof = true;
	spin_unlock_irqrestore(&priv->irqlock, flags);

	/*
	 * and then wait for interrupt handler to mark completion.
	 */
	ret = wait_for_completion_timeout(
		&priv->last_eof_comp,
		msecs_to_jiffies(IMX_MEDIA_EOF_TIMEOUT));
	if (ret == 0)
		v4l2_warn(&ic_priv->sd, "wait last EOF timeout\n");

	devm_free_irq(ic_priv->dev, priv->eof_irq, priv);
	devm_free_irq(ic_priv->dev, priv->nfb4eof_irq, priv);

	if (ipu_rot_mode_is_irt(priv->rot_mode))
		prpenc_unsetup_rotation(priv);
	else
		prpenc_unsetup_norotation(priv);

	prpenc_put_ipu_resources(priv);

	/* cancel the EOF timeout timer */
	del_timer_sync(&priv->eof_timeout_timer);

	priv->src_ring = NULL;

	/* inform sink that the buffer ring can now be freed */
	return v4l2_subdev_call(priv->sink_sd, core, ioctl,
				IMX_MEDIA_REL_DMA_BUF_RING, 0);
}

static int prpenc_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct prpenc_priv *priv = sd_to_priv(sd);
	bool allow_planar;

	if (code->pad >= PRPENC_NUM_PADS)
		return -EINVAL;

	allow_planar = (code->pad == priv->output_pad);

	return imx_media_enum_format(&code->code, code->index,
				     true, allow_planar);
}

static int prpenc_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *sdformat)
{
	struct prpenc_priv *priv = sd_to_priv(sd);

	if (sdformat->pad >= PRPENC_NUM_PADS)
		return -EINVAL;

	sdformat->format = priv->format_mbus[sdformat->pad];

	return 0;
}

static int prpenc_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *sdformat)
{
	struct prpenc_priv *priv = sd_to_priv(sd);
	struct v4l2_mbus_framefmt *infmt, *outfmt;
	const struct imx_media_pixfmt *cc;
	bool allow_planar;
	u32 code;

	if (sdformat->pad >= PRPENC_NUM_PADS)
		return -EINVAL;

	if (priv->stream_on)
		return -EBUSY;

	infmt = &priv->format_mbus[priv->input_pad];
	outfmt = &priv->format_mbus[priv->output_pad];
	allow_planar = (sdformat->pad == priv->output_pad);

	cc = imx_media_find_format(0, sdformat->format.code,
				   true, allow_planar);
	if (!cc) {
		imx_media_enum_format(&code, 0, true, false);
		cc = imx_media_find_format(0, code, true, false);
		sdformat->format.code = cc->codes[0];
	}

	if (sdformat->pad == priv->output_pad) {
		sdformat->format.width = min_t(__u32,
					       sdformat->format.width,
					       MAX_W_IC);
		sdformat->format.height = min_t(__u32,
						sdformat->format.height,
						MAX_H_IC);

		if (sdformat->format.field != V4L2_FIELD_NONE)
			sdformat->format.field = infmt->field;

		/* IC resizer cannot downsize more than 4:1 */
		if (ipu_rot_mode_is_irt(priv->rot_mode)) {
			sdformat->format.width = max_t(__u32,
						       sdformat->format.width,
						       infmt->height / 4);
			sdformat->format.height = max_t(__u32,
							sdformat->format.height,
							infmt->width / 4);
		} else {
			sdformat->format.width = max_t(__u32,
						       sdformat->format.width,
						       infmt->width / 4);
			sdformat->format.height = max_t(__u32,
							sdformat->format.height,
							infmt->height / 4);
		}
	} else {
		sdformat->format.width = min_t(__u32,
					       sdformat->format.width,
					       MAX_W_SINK);
		sdformat->format.height = min_t(__u32,
						sdformat->format.height,
						MAX_H_SINK);
	}

	if (sdformat->which == V4L2_SUBDEV_FORMAT_TRY) {
		cfg->try_fmt = sdformat->format;
	} else { 
		priv->format_mbus[sdformat->pad] = sdformat->format;
		priv->cc[sdformat->pad] = cc;
	}

	return 0;
}

static int prpenc_link_setup(struct media_entity *entity,
			    const struct media_pad *local,
			    const struct media_pad *remote, u32 flags)
{
	struct v4l2_subdev *sd = media_entity_to_v4l2_subdev(entity);
	struct imx_ic_priv *ic_priv = v4l2_get_subdevdata(sd);
	struct prpenc_priv *priv = ic_priv->task_priv;
	struct v4l2_subdev *remote_sd;
	int csi_id;

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

	switch (remote_sd->grp_id) {
	case IMX_MEDIA_GRP_ID_CSI0:
		csi_id = 0;
		break;
	case IMX_MEDIA_GRP_ID_CSI1:
		csi_id = 1;
		break;
	default:
		return -EINVAL;
	}

	/* set IC to receive from CSI */
	ipu_set_ic_src_mux(ic_priv->ipu, csi_id, false);

	return 0;
}

static int prpenc_link_validate(struct v4l2_subdev *sd,
				struct media_link *link,
				struct v4l2_subdev_format *source_fmt,
				struct v4l2_subdev_format *sink_fmt)
{
	struct imx_ic_priv *ic_priv = v4l2_get_subdevdata(sd);
	struct prpenc_priv *priv = ic_priv->task_priv;
	struct v4l2_mbus_config sensor_mbus_cfg;
	struct v4l2_of_endpoint *sensor_ep;
	struct imx_media_subdev *sensor;
	int ret;

	ret = v4l2_subdev_link_validate_default(sd, link, source_fmt, sink_fmt);
	if (ret)
		return ret;

	sensor = priv->md->sensor;
	if (!sensor) {
		v4l2_err(&ic_priv->sd, "no sensor attached\n");
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
		/*
		 * only 8-bit pixels can be sent to IC for parallel
		 * busses
		 */
		if (sensor_ep->bus.parallel.bus_width >= 16)
			return -EINVAL;
	}

	return 0;
}

static int prpenc_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct prpenc_priv *priv = container_of(ctrl->handler,
					       struct prpenc_priv, ctrl_hdlr);
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
		/* can't change rotation mid-streaming */
		if (priv->stream_on)
			return -EBUSY;

		priv->rot_mode = rot_mode;
		priv->rotation = rotation;
		priv->hflip = hflip;
		priv->vflip = vflip;
	}

	return 0;
}

static const struct v4l2_ctrl_ops prpenc_ctrl_ops = {
	.s_ctrl = prpenc_s_ctrl,
};

static const struct v4l2_ctrl_config prpenc_std_ctrl[] = {
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

#define PRPENC_NUM_CONTROLS ARRAY_SIZE(prpenc_std_ctrl)

static int prpenc_init_controls(struct prpenc_priv *priv)
{
	struct imx_ic_priv *ic_priv = priv->ic_priv;
	struct v4l2_ctrl_handler *hdlr = &priv->ctrl_hdlr;
	const struct v4l2_ctrl_config *c;
	int i, ret;

	v4l2_ctrl_handler_init(hdlr, PRPENC_NUM_CONTROLS);

	for (i = 0; i < PRPENC_NUM_CONTROLS; i++) {
		c = &prpenc_std_ctrl[i];
		v4l2_ctrl_new_std(hdlr, &prpenc_ctrl_ops,
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

static int prpenc_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct prpenc_priv *priv = sd_to_priv(sd);
	int ret = 0;

	v4l2_info(sd, "stream %s\n", enable ? "ON" : "OFF");

	if (enable && !priv->stream_on)
		ret = prpenc_start(priv);
	else if (!enable && priv->stream_on)
		ret = prpenc_stop(priv);

	if (!ret)
		priv->stream_on = enable;
	return ret;
}

/*
 * retrieve our pads parsed from the OF graph by the media device
 */
static int prpenc_registered(struct v4l2_subdev *sd)
{
	struct prpenc_priv *priv = sd_to_priv(sd);
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

	for (i = 0; i < PRPENC_NUM_PADS; i++) {
		pad = &imxsd->pad[i];
		priv->pad[i] = pad->pad;
		if (priv->pad[i].flags & MEDIA_PAD_FL_SINK)
			priv->input_pad = i;
		else
			priv->output_pad = i;
	}

	return media_entity_pads_init(&sd->entity, PRPENC_NUM_PADS,
				      priv->pad);
}

static struct v4l2_subdev_pad_ops prpenc_pad_ops = {
	.enum_mbus_code = prpenc_enum_mbus_code,
	.get_fmt = prpenc_get_fmt,
	.set_fmt = prpenc_set_fmt,
	.link_validate = prpenc_link_validate,
};

static struct v4l2_subdev_video_ops prpenc_video_ops = {
	.s_stream = prpenc_s_stream,
};

static struct media_entity_operations prpenc_entity_ops = {
	.link_setup = prpenc_link_setup,
	.link_validate = v4l2_subdev_link_validate,
};

static struct v4l2_subdev_ops prpenc_subdev_ops = {
	.video = &prpenc_video_ops,
	.pad = &prpenc_pad_ops,
};

static struct v4l2_subdev_internal_ops prpenc_internal_ops = {
	.registered = prpenc_registered,
};

static int prpenc_init(struct imx_ic_priv *ic_priv)
{
	struct device_node *np = ic_priv->dev->of_node;
	struct prpenc_priv *priv;

	priv = devm_kzalloc(ic_priv->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	ic_priv->task_priv = priv;
	priv->ic_priv = ic_priv;

	spin_lock_init(&priv->irqlock);
	init_timer(&priv->eof_timeout_timer);
	priv->eof_timeout_timer.data = (unsigned long)priv;
	priv->eof_timeout_timer.function = prpenc_eof_timeout;
	snprintf(ic_priv->sd.name, sizeof(ic_priv->sd.name), "%s", np->name);

	return prpenc_init_controls(priv);
}

static void prpenc_remove(struct imx_ic_priv *ic_priv)
{
}

struct imx_ic_ops imx_ic_prpenc_ops = {
	.subdev_ops = &prpenc_subdev_ops,
	.internal_ops = &prpenc_internal_ops,
	.entity_ops = &prpenc_entity_ops,
	.init = prpenc_init,
	.remove = prpenc_remove,
};
