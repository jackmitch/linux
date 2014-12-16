/*
 * Video Mem2mem driver for Freescale i.MX5/6 SOC
 *
 * Copyright (c) 2012-2016 Mentor Graphics Inc.
 * Steve Longerbeam <steve_longerbeam@mentor.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version
 */
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/timer.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/log2.h>

#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <media/v4l2-mem2mem.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-ctrls.h>
#include <media/videobuf2-dma-contig.h>
#include <video/imx-ipu-v3.h>
#include <media/imx.h>
#include "imx-media.h"

#define DEVICE_NAME "imx-media-m2m"

#define M2MX_NUM_SINK_PADS     2 /* a devnode pad and a subdev pad */
#define M2MX_NUM_SRC_PADS      2 /* a devnode pad and a subdev pad */
#define M2MX_NUM_SUBDEV_PADS   2
#define M2MX_NUM_DEVNODE_PADS  2
#define M2MX_NUM_PADS          4

static int instrument;
module_param(instrument, int, 0);
MODULE_PARM_DESC(instrument, "1 = enable conversion time measurement");

/* Per queue */
#define MEM2MEM_DEF_NUM_BUFS	VIDEO_MAX_FRAME
/* In bytes, per queue */
#define MEM2MEM_VID_MEM_LIMIT	SZ_256M

struct m2mx_mbus_fmt {
	struct v4l2_mbus_framefmt fmt;
	const struct imx_media_pixfmt *cc;
};

struct m2mx_priv {
	struct device         *dev;
	struct video_device    vfd;
	struct media_pipeline  mp;
	struct imx_media_dev  *md;
	struct v4l2_subdev     sd;
	struct media_pad       pad[M2MX_NUM_PADS];
	struct media_pad       vd_pad[M2MX_NUM_DEVNODE_PADS];
	int id;
	int input_pad;
	int output_pad;
	int devnode_input_pad;
	int devnode_output_pad;

	/* our dma buffer sink ring */
	struct imx_media_dma_buf_ring *sink_ring;
	/* the dma buffer ring we send to sink */
	struct imx_media_dma_buf_ring *src_ring;

	/* Source and destination image data */
	struct m2mx_mbus_fmt format_mbus[M2MX_NUM_PADS];

	struct v4l2_subdev     *src_sd;
	struct v4l2_subdev     *sink_sd;

	/* the device mutex */
	struct mutex		dev_mutex;
	spinlock_t		q_lock;

	struct v4l2_m2m_dev	*m2m_dev;

	struct v4l2_fh     fh;

	bool		aborting;  /* abort requested by m2m */
	bool		stop;      /* streaming is stopping */

	/* for instrumenting */
	struct timespec   start;
};

#define fh_to_priv(p) container_of(p, struct m2mx_priv, fh)

static struct m2mx_mbus_fmt *get_mbus_fmt(struct m2mx_priv *priv,
					  enum v4l2_buf_type type)
{
	switch (type) {
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
		return &priv->format_mbus[priv->output_pad];
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		return &priv->format_mbus[priv->input_pad];
	default:
		break;
	}
	return NULL;
}

/*
 * mem2mem callbacks
 */

static void m2mx_job_abort(void *ctx_priv)
{
	struct m2mx_priv *priv = ctx_priv;

	priv->aborting = true;
}

static void m2mx_lock(void *ctx_priv)
{
	struct m2mx_priv *priv = ctx_priv;

	mutex_lock(&priv->dev_mutex);
}

static void m2mx_unlock(void *ctx_priv)
{
	struct m2mx_priv *priv = ctx_priv;

	mutex_unlock(&priv->dev_mutex);
}

static void m2mx_device_run(void *ctx_priv)
{
	struct m2mx_priv *priv = ctx_priv;
	struct vb2_v4l2_buffer *src_buf, *dst_buf;
	struct imx_media_dma_buf *sinkbuf;

	src_buf = v4l2_m2m_next_src_buf(priv->fh.m2m_ctx);
	dst_buf = v4l2_m2m_next_dst_buf(priv->fh.m2m_ctx);

	sinkbuf = imx_media_dma_buf_next_in(priv->src_ring);
	imx_media_dma_buf_return(priv->src_ring, IMX_MEDIA_BUF_STATUS_DONE);

	if (instrument)
		ktime_get_ts(&priv->start);
}

/*
 * video ioctls
 */
static int m2mx_querycap(struct file *file, void *fh,
			 struct v4l2_capability *cap)
{
	strncpy(cap->driver, DEVICE_NAME, sizeof(cap->driver) - 1);
	strncpy(cap->card, DEVICE_NAME, sizeof(cap->card) - 1);
	cap->bus_info[0] = 0;
	cap->device_caps = V4L2_CAP_VIDEO_M2M | V4L2_CAP_STREAMING;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;

	return 0;
}

static int enum_fmt(struct v4l2_fmtdesc *f)
{
	const struct imx_media_pixfmt *cc;
	u32 code;
	int ret;

	ret = imx_media_enum_format(&code, f->index, true, true);
	if (ret)
		return ret;
	cc = imx_media_find_format(0, code, true, true);
	if (!cc)
		return -EINVAL;

	f->pixelformat = cc->fourcc;

	return 0;
}

static int m2mx_enum_fmt_vid_cap(struct file *file, void *fh,
				 struct v4l2_fmtdesc *f)
{
	return enum_fmt(f);
}

static int m2mx_enum_fmt_vid_out(struct file *file, void *fh,
				 struct v4l2_fmtdesc *f)
{
	return enum_fmt(f);
}

static int m2mx_g_fmt(struct m2mx_priv *priv, struct v4l2_format *f)
{
	struct m2mx_mbus_fmt *fmt;

	/* user formats are same as formats from the input/output pads */
	fmt = get_mbus_fmt(priv, f->type);
	if (!fmt)
		return -EINVAL;

	return imx_media_mbus_fmt_to_pix_fmt(&f->fmt.pix, &fmt->fmt);
}

static int m2mx_g_fmt_vid_out(struct file *file, void *fh,
			      struct v4l2_format *f)
{
	struct m2mx_priv *priv = fh_to_priv(fh);

	return m2mx_g_fmt(priv, f);
}

static int m2mx_g_fmt_vid_cap(struct file *file, void *fh,
			      struct v4l2_format *f)
{
	struct m2mx_priv *priv = fh_to_priv(fh);

	return m2mx_g_fmt(priv, f);
}

static int m2mx_try_fmt_vid_cap(struct file *file, void *fh,
				struct v4l2_format *f)
{
	struct m2mx_priv *priv = fh_to_priv(fh);

	return m2mx_g_fmt(priv, f);
}

static int m2mx_try_fmt_vid_out(struct file *file, void *fh,
				struct v4l2_format *f)
{
	struct m2mx_priv *priv = fh_to_priv(fh);

	return m2mx_g_fmt(priv, f);
}

static int m2mx_s_fmt(struct m2mx_priv *priv, struct v4l2_format *f)
{
	struct vb2_queue *vq;

	vq = v4l2_m2m_get_vq(priv->fh.m2m_ctx, f->type);
	if (!vq)
		return -EINVAL;

	if (vb2_is_busy(vq)) {
		v4l2_err(&priv->sd, "%s: queue busy\n", __func__);
		return -EBUSY;
	}

	return m2mx_g_fmt(priv, f);
}

static int m2mx_s_fmt_vid_cap(struct file *file, void *fh,
			      struct v4l2_format *f)
{
	struct m2mx_priv *priv = fh_to_priv(fh);

	return m2mx_s_fmt(priv, f);
}

static int m2mx_s_fmt_vid_out(struct file *file, void *fh,
			      struct v4l2_format *f)
{
	struct m2mx_priv *priv = fh_to_priv(fh);

	return m2mx_s_fmt(priv, f);
}

static int m2mx_reqbufs(struct file *file, void *fh,
			struct v4l2_requestbuffers *reqbufs)
{
	struct m2mx_priv *priv = fh_to_priv(fh);

	return v4l2_m2m_reqbufs(file, priv->fh.m2m_ctx, reqbufs);
}

static int m2mx_querybuf(struct file *file, void *fh,
			 struct v4l2_buffer *buf)
{
	struct m2mx_priv *priv = fh_to_priv(fh);

	return v4l2_m2m_querybuf(file, priv->fh.m2m_ctx, buf);
}

static int m2mx_qbuf(struct file *file, void *fh, struct v4l2_buffer *buf)
{
	struct m2mx_priv *priv = fh_to_priv(fh);

	return v4l2_m2m_qbuf(file, priv->fh.m2m_ctx, buf);
}

static int m2mx_dqbuf(struct file *file, void *fh, struct v4l2_buffer *buf)
{
	struct m2mx_priv *priv = fh_to_priv(fh);

	return v4l2_m2m_dqbuf(file, priv->fh.m2m_ctx, buf);
}

static int m2mx_expbuf(struct file *file, void *fh,
		       struct v4l2_exportbuffer *eb)
{
	struct m2mx_priv *priv = fh_to_priv(fh);

	return v4l2_m2m_expbuf(file, priv->fh.m2m_ctx, eb);
}

static int m2mx_streamon(struct file *file, void *fh,
			 enum v4l2_buf_type type)
{
	struct m2mx_priv *priv = fh_to_priv(fh);

	return v4l2_m2m_streamon(file, priv->fh.m2m_ctx, type);
}

static int m2mx_streamoff(struct file *file, void *fh,
			  enum v4l2_buf_type type)
{
	struct m2mx_priv *priv = fh_to_priv(fh);

	return v4l2_m2m_streamoff(file, priv->fh.m2m_ctx, type);
}

static const struct v4l2_ioctl_ops m2mx_ioctl_ops = {
	.vidioc_querycap	= m2mx_querycap,

	.vidioc_enum_fmt_vid_cap = m2mx_enum_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap	= m2mx_g_fmt_vid_cap,
	.vidioc_try_fmt_vid_cap	= m2mx_try_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap	= m2mx_s_fmt_vid_cap,

	.vidioc_enum_fmt_vid_out = m2mx_enum_fmt_vid_out,
	.vidioc_g_fmt_vid_out	= m2mx_g_fmt_vid_out,
	.vidioc_try_fmt_vid_out	= m2mx_try_fmt_vid_out,
	.vidioc_s_fmt_vid_out	= m2mx_s_fmt_vid_out,

	.vidioc_reqbufs		= m2mx_reqbufs,
	.vidioc_querybuf	= m2mx_querybuf,

	.vidioc_qbuf		= m2mx_qbuf,
	.vidioc_dqbuf		= m2mx_dqbuf,
	.vidioc_expbuf		= m2mx_expbuf,

	.vidioc_streamon	= m2mx_streamon,
	.vidioc_streamoff	= m2mx_streamoff,
};

/*
 * Queue operations
 */

static u32 m2mx_get_sizeimage(struct m2mx_priv *priv, enum v4l2_buf_type type)
{
	struct m2mx_mbus_fmt *fmt;

	fmt = get_mbus_fmt(priv, type);
	if (!fmt)
		return 0;

	return (fmt->fmt.width * fmt->fmt.height * fmt->cc->bpp) >> 3;
}

static int m2mx_queue_setup(struct vb2_queue *vq,
			    unsigned int *nbuffers, unsigned int *nplanes,
			    unsigned int sizes[], struct device *alloc_devs[])
{
	struct m2mx_priv *priv = vb2_get_drv_priv(vq);
	unsigned int count = *nbuffers;
	u32 sizeimage;

	sizeimage = m2mx_get_sizeimage(priv, vq->type);

	while (sizeimage * count > MEM2MEM_VID_MEM_LIMIT)
		count--;

	*nplanes = 1;
	*nbuffers = count;
	sizes[0] = sizeimage;

	return 0;
}

static int m2mx_buf_prepare(struct vb2_buffer *vb)
{
	struct vb2_queue *vq = vb->vb2_queue;
	struct m2mx_priv *priv = vb2_get_drv_priv(vq);
	u32 sizeimage;

	sizeimage = m2mx_get_sizeimage(priv, vq->type);

	if (vb2_plane_size(vb, 0) < sizeimage) {
		v4l2_err(&priv->sd,
			 "%s: data will not fit into plane (%lu < %lu)\n",
			 __func__, vb2_plane_size(vb, 0), (long)sizeimage);
		return -EINVAL;
	}

	vb2_set_plane_payload(vb, 0, sizeimage);

	return 0;
}

static void m2mx_buf_queue(struct vb2_buffer *vb)
{
	struct m2mx_priv *priv = vb2_get_drv_priv(vb->vb2_queue);

	v4l2_m2m_buf_queue(priv->fh.m2m_ctx, to_vb2_v4l2_buffer(vb));
}

static void m2mx_wait_prepare(struct vb2_queue *vq)
{
	struct m2mx_priv *priv = vb2_get_drv_priv(vq);

	m2mx_unlock(priv);
}

static void m2mx_wait_finish(struct vb2_queue *vq)
{
	struct m2mx_priv *priv = vb2_get_drv_priv(vq);

	m2mx_lock(priv);
}

static int m2mx_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct m2mx_priv *priv = vb2_get_drv_priv(vq);
	struct vb2_v4l2_buffer *buf;
	unsigned long flags;
	u32 sizeimage;
	int i, ret;

	if (vb2_is_streaming(vq) || vq->type == V4L2_BUF_TYPE_VIDEO_OUTPUT)
		return 0;

	sizeimage = m2mx_get_sizeimage(priv, vq->type);

	if (priv->sink_ring) {
		v4l2_warn(&priv->sd, "%s: dma-buf ring was not freed\n",
			  __func__);
		imx_media_free_dma_buf_ring(priv->sink_ring);
	}

	priv->sink_ring =
		imx_media_alloc_dma_buf_ring(priv->md, priv->src_sd, &priv->sd,
					     sizeimage, vq->num_buffers, false);
	if (IS_ERR(priv->sink_ring)) {
		v4l2_err(&priv->sd, "failed to alloc dma-buf ring\n");
		ret = PTR_ERR(priv->sink_ring);
		priv->sink_ring = NULL;
		goto return_bufs;
	}

	spin_lock_irqsave(&priv->q_lock, flags);
	for (i = 0; i < vq->num_buffers; i++) {
		ret = imx_media_dma_buf_init_from_vb(priv->sink_ring,
						     vq->bufs[i]);
		if (ret) {
			spin_unlock_irqrestore(&priv->q_lock, flags);
			goto free_ring;
		}
	}
	spin_unlock_irqrestore(&priv->q_lock, flags);

	ret = imx_media_pipeline_set_stream(priv->md, &priv->sd.entity,
					    &priv->mp, true);
	if (ret) {
		v4l2_err(&priv->sd, "pipeline_set_stream failed with %d\n",
			 ret);
		goto free_ring;
	}

	/* ask the sink for the buffer ring */
	ret = v4l2_subdev_call(priv->sink_sd, core, ioctl,
			       IMX_MEDIA_REQ_DMA_BUF_RING,
			       &priv->src_ring);
	if (ret)
		goto pipeline_off;
		
	priv->stop = false;

	return 0;

pipeline_off:
	imx_media_pipeline_set_stream(priv->md, &priv->sd.entity,
				      &priv->mp, false);
free_ring:
	imx_media_free_dma_buf_ring(priv->sink_ring);
	priv->sink_ring = NULL;
return_bufs:
	spin_lock_irqsave(&priv->q_lock, flags);
	while ((buf = v4l2_m2m_dst_buf_remove(priv->fh.m2m_ctx)))
		v4l2_m2m_buf_done(buf, VB2_BUF_STATE_QUEUED);
	spin_unlock_irqrestore(&priv->q_lock, flags);
	return ret;
}

static void m2mx_stop_streaming(struct vb2_queue *vq)
{
	struct m2mx_priv *priv = vb2_get_drv_priv(vq);
	struct vb2_v4l2_buffer *buf;
	unsigned long flags;
	int ret;

	if (!vb2_is_streaming(vq))
		return;

	if (vq->type == V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		spin_lock_irqsave(&priv->q_lock, flags);
		priv->stop = true;
		spin_unlock_irqrestore(&priv->q_lock, flags);

		ret = imx_media_pipeline_set_stream(priv->md, &priv->sd.entity,
						    &priv->mp, false);
		if (ret)
			v4l2_warn(&priv->sd,
				  "pipeline_set_stream failed with %d\n", ret);

		priv->src_ring = NULL;
		/* inform sink that the buffer ring can now be freed */
		v4l2_subdev_call(priv->sink_sd, core, ioctl,
				 IMX_MEDIA_REL_DMA_BUF_RING, 0);
	}

	/* release all active buffers */
	spin_lock_irqsave(&priv->q_lock, flags);
	if (vq->type == V4L2_BUF_TYPE_VIDEO_OUTPUT) {
		while ((buf = v4l2_m2m_src_buf_remove(priv->fh.m2m_ctx)))
			v4l2_m2m_buf_done(buf, VB2_BUF_STATE_QUEUED);
	} else {
		while ((buf = v4l2_m2m_dst_buf_remove(priv->fh.m2m_ctx)))
			v4l2_m2m_buf_done(buf, VB2_BUF_STATE_QUEUED);
	}
	spin_unlock_irqrestore(&priv->q_lock, flags);
}

static struct vb2_ops m2mx_qops = {
	.queue_setup	 = m2mx_queue_setup,
	.buf_prepare	 = m2mx_buf_prepare,
	.buf_queue	 = m2mx_buf_queue,
	.wait_prepare	 = m2mx_wait_prepare,
	.wait_finish	 = m2mx_wait_finish,
	.start_streaming = m2mx_start_streaming,
	.stop_streaming  = m2mx_stop_streaming,
};

static int queue_init(void *ctx_priv, struct vb2_queue *src_vq,
		      struct vb2_queue *dst_vq)
{
	struct m2mx_priv *priv = ctx_priv;
	int ret;

	memset(src_vq, 0, sizeof(*src_vq));
	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	src_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	src_vq->drv_priv = priv;
	src_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	src_vq->ops = &m2mx_qops;
	src_vq->mem_ops = &vb2_dma_contig_memops;
	src_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src_vq->dev = priv->dev;
	ret = vb2_queue_init(src_vq);
	if (ret)
		return ret;

	memset(dst_vq, 0, sizeof(*dst_vq));
	dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	dst_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	dst_vq->drv_priv = priv;
	dst_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	dst_vq->ops = &m2mx_qops;
	dst_vq->mem_ops = &vb2_dma_contig_memops;
	dst_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst_vq->dev = priv->dev;

	return vb2_queue_init(dst_vq);
}

/*
 * File operations
 */
static int m2mx_open(struct file *file)
{
	struct m2mx_priv *priv = video_drvdata(file);
	int ret = 0;

	if (mutex_lock_interruptible(&priv->dev_mutex))
		return -ERESTARTSYS;

	v4l2_fh_init(&priv->fh, &priv->vfd);
	file->private_data = &priv->fh;
	v4l2_fh_add(&priv->fh);

	priv->fh.m2m_ctx = v4l2_m2m_ctx_init(priv->m2m_dev, priv, &queue_init);
	if (IS_ERR(priv->fh.m2m_ctx)) {
		ret = PTR_ERR(priv->fh.m2m_ctx);
		goto error_fh;
	}

	mutex_unlock(&priv->dev_mutex);
	return 0;

error_fh:
	v4l2_fh_del(&priv->fh);
	v4l2_fh_exit(&priv->fh);
	mutex_unlock(&priv->dev_mutex);
	return ret;
}

static int m2mx_release(struct file *file)
{
	struct m2mx_priv *priv = video_drvdata(file);

	mutex_lock(&priv->dev_mutex);

	v4l2_m2m_ctx_release(priv->fh.m2m_ctx);
	v4l2_fh_del(&priv->fh);
	v4l2_fh_exit(&priv->fh);

	mutex_unlock(&priv->dev_mutex);
	return 0;
}

static unsigned int m2mx_poll(struct file *file,
			      struct poll_table_struct *wait)
{
	struct m2mx_priv *priv = video_drvdata(file);
	int ret;

	if (mutex_lock_interruptible(&priv->dev_mutex))
		return -ERESTARTSYS;

	ret = v4l2_m2m_poll(file, priv->fh.m2m_ctx, wait);

	mutex_unlock(&priv->dev_mutex);
	return ret;
}

static int m2mx_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct m2mx_priv *priv = video_drvdata(file);
	int ret;

	if (mutex_lock_interruptible(&priv->dev_mutex))
		return -ERESTARTSYS;

	ret = v4l2_m2m_mmap(file, priv->fh.m2m_ctx, vma);

	mutex_unlock(&priv->dev_mutex);
	return ret;
}

static const struct v4l2_file_operations m2mx_fops = {
	.owner		= THIS_MODULE,
	.open		= m2mx_open,
	.release	= m2mx_release,
	.poll		= m2mx_poll,
	.unlocked_ioctl	= video_ioctl2,
	.mmap		= m2mx_mmap,
};

static struct video_device m2mx_videodev = {
	.name		= DEVICE_NAME,
	.fops		= &m2mx_fops,
	.ioctl_ops	= &m2mx_ioctl_ops,
	.minor		= -1,
	.release	= video_device_release,
	.vfl_dir	= VFL_DIR_M2M,
};

static struct v4l2_m2m_ops m2m_ops = {
	.device_run	= m2mx_device_run,
	.job_abort	= m2mx_job_abort,
	.lock		= m2mx_lock,
	.unlock		= m2mx_unlock,
};

/*
 * Subdev and media entity operations
 */

static void m2mx_new_dma_buf(struct m2mx_priv *priv)
{
	struct vb2_v4l2_buffer *src_vb, *dst_vb;
	struct imx_media_dma_buf *dmabuf;
	enum vb2_buffer_state state;
	struct timespec ts, diff;
	unsigned long interval;
	struct vb2_buffer *vb;
	unsigned long flags;

	spin_lock_irqsave(&priv->q_lock, flags);

	if (priv->stop)
		goto unlock;

	dmabuf = imx_media_dma_buf_next_out(priv->sink_ring);
	vb = dmabuf->vb;

	dev_dbg(priv->dev, "%s: new dmabuf %d\n", __func__, vb->index);

	if (!priv->aborting)
		goto unlock;

	state = dmabuf->status == IMX_MEDIA_BUF_STATUS_DONE ?
		VB2_BUF_STATE_DONE : VB2_BUF_STATE_ERROR;

	src_vb = v4l2_m2m_src_buf_remove(priv->fh.m2m_ctx);
	dst_vb = v4l2_m2m_dst_buf_remove(priv->fh.m2m_ctx);

	if (src_vb) {
		v4l2_m2m_buf_done(src_vb, state);
		dev_dbg(priv->dev, "%s: new src_vb %d\n", __func__,
			src_vb->vb2_buf.index);
	}
	if (dst_vb) {
		v4l2_m2m_buf_done(dst_vb, state);
		dev_dbg(priv->dev, "%s: new dst_vb %d\n", __func__,
			dst_vb->vb2_buf.index);
	}

	if (src_vb && dst_vb) {
		v4l2_m2m_job_finish(priv->m2m_dev, priv->fh.m2m_ctx);

		if (instrument) {
			ktime_get_ts(&ts);
			diff = timespec_sub(ts, priv->start);
			interval = diff.tv_sec * 1000 * 1000 +
				diff.tv_nsec / 1000;
			v4l2_info(&priv->sd, "buf%d completed in %lu usec\n",
				  dst_vb->vb2_buf.index, interval);
		}
	}

unlock:
	spin_unlock_irqrestore(&priv->q_lock, flags);
}

static long m2mx_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct m2mx_priv *priv = v4l2_get_subdevdata(sd);
	struct imx_media_dma_buf_ring **ring;

	switch (cmd) {
	case IMX_MEDIA_REQ_DMA_BUF_RING:
		if (!priv->sink_ring)
			return -EINVAL;
		ring = (struct imx_media_dma_buf_ring **)arg;
		*ring = priv->sink_ring;
		break;
	case IMX_MEDIA_NEW_DMA_BUF:
		m2mx_new_dma_buf(priv);
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

static int m2mx_link_setup(struct media_entity *entity,
			   const struct media_pad *local,
			   const struct media_pad *remote, u32 flags)
{
	struct v4l2_subdev *sd = media_entity_to_v4l2_subdev(entity);
	struct m2mx_priv *priv = v4l2_get_subdevdata(sd);
	struct v4l2_subdev *remote_sd;

	dev_dbg(priv->dev, "link setup %s -> %s", remote->entity->name,
		local->entity->name);

	if (is_media_entity_v4l2_video_device(remote->entity))
		return 0;

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

static int m2mx_enum_mbus_code(struct v4l2_subdev *sd,
			       struct v4l2_subdev_pad_config *cfg,
			       struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->pad >= M2MX_NUM_PADS)
		return -EINVAL;

	return imx_media_enum_format(&code->code, code->index, true, true);
}

static int m2mx_get_fmt(struct v4l2_subdev *sd,
			struct v4l2_subdev_pad_config *cfg,
			struct v4l2_subdev_format *sdformat)
{
	struct m2mx_priv *priv = v4l2_get_subdevdata(sd);

	if (sdformat->pad >= M2MX_NUM_PADS)
		return -EINVAL;

	sdformat->format = priv->format_mbus[sdformat->pad].fmt;

	return 0;
}

static int m2mx_set_fmt(struct v4l2_subdev *sd,
			 struct v4l2_subdev_pad_config *cfg,
			 struct v4l2_subdev_format *sdformat)
{
	struct m2mx_priv *priv = v4l2_get_subdevdata(sd);
	const struct imx_media_pixfmt *cc;
	u32 code;

	if (sdformat->pad >= M2MX_NUM_PADS)
		return -EINVAL;

	cc = imx_media_find_format(0, sdformat->format.code, true, true);
	if (!cc) {
		imx_media_enum_format(&code, 0, true, true);
		cc = imx_media_find_format(0, code, true, true);
		sdformat->format.code = cc->codes[0];
	}

	/* device-node pads mirror subdev pads */
	if (sdformat->pad == priv->devnode_input_pad) {
		priv->format_mbus[priv->output_pad].fmt = sdformat->format;
		priv->format_mbus[priv->output_pad].cc = cc;
	} else if (sdformat->pad == priv->devnode_output_pad) {
		priv->format_mbus[priv->input_pad].fmt = sdformat->format;
		priv->format_mbus[priv->input_pad].cc = cc;
	} else if (sdformat->pad == priv->input_pad) {
		priv->format_mbus[priv->devnode_output_pad].fmt =
			sdformat->format;
		priv->format_mbus[priv->devnode_output_pad].cc = cc;
	} else {
		/* output_pad */
		priv->format_mbus[priv->devnode_input_pad].fmt =
			sdformat->format;
		priv->format_mbus[priv->devnode_input_pad].cc = cc;
	}

	if (sdformat->which == V4L2_SUBDEV_FORMAT_TRY) {
		cfg->try_fmt = sdformat->format;
	} else {
		priv->format_mbus[sdformat->pad].fmt = sdformat->format;
		priv->format_mbus[sdformat->pad].cc = cc;
	}

	return 0;
}

static int m2mx_init_pads(struct m2mx_priv *priv)
{
	struct video_device *vfd = &priv->vfd;
	struct imx_media_subdev *imxsd;
	struct imx_media_pad *pad;
	int i, ret;

	imxsd = imx_media_find_subdev_by_sd(priv->md, &priv->sd);
	if (IS_ERR(imxsd))
		return PTR_ERR(imxsd);

	if (imxsd->num_sink_pads != M2MX_NUM_SINK_PADS ||
	    imxsd->num_src_pads != M2MX_NUM_SRC_PADS) {
		v4l2_err(&priv->sd, "invalid num pads %d/%d\n",
			 imxsd->num_sink_pads, imxsd->num_src_pads);
		return -EINVAL;
	}

	priv->vd_pad[0].flags = MEDIA_PAD_FL_SINK;
	priv->vd_pad[1].flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&vfd->entity, 2, priv->vd_pad);
	if (ret) {
		v4l2_err(&priv->sd, "failed to init device node pads\n");
		return ret;
	}

	for (i = 0; i < M2MX_NUM_PADS; i++) {
		pad = &imxsd->pad[i];
		priv->pad[i] = pad->pad;
		if (priv->pad[i].flags & MEDIA_PAD_FL_SINK) {
			if (pad->num_links)
				priv->input_pad = i;
			else
				priv->devnode_input_pad = i;
		} else {
			if (pad->num_links)
				priv->output_pad = i;
			else
				priv->devnode_output_pad = i;
		}
	}

	return media_entity_pads_init(&priv->sd.entity, M2MX_NUM_PADS,
				      priv->pad);
}

static int m2mx_registered(struct v4l2_subdev *sd)
{
	struct m2mx_priv *priv = v4l2_get_subdevdata(sd);
	struct video_device *vfd = &priv->vfd;
	struct m2mx_mbus_fmt *capfmt, *outfmt;
	int ret;

	/* get media device */
	priv->md = dev_get_drvdata(sd->v4l2_dev->dev);

	vfd->v4l2_dev = sd->v4l2_dev;

	ret = video_register_device(vfd, VFL_TYPE_GRABBER, 0);
	if (ret) {
		v4l2_err(sd, "Failed to register video device\n");
		return ret;
	}

	ret = m2mx_init_pads(priv);
	if (ret) {
		v4l2_err(sd, "m2mx_init_pads failed\n");
		goto unreg;
	}

	/*
	 * create the links to our device nodes (imx-media-dev creates
	 * the rest)
	 */
	ret = media_create_pad_link(&sd->entity, priv->devnode_output_pad,
				    &vfd->entity, 0,
				    MEDIA_LNK_FL_IMMUTABLE |
				    MEDIA_LNK_FL_ENABLED);
	if (ret) {
		v4l2_err(sd, "failed to create link to devnode sink pad\n");
		goto unreg;
	}
	ret = media_create_pad_link(&vfd->entity, 1,
				    &sd->entity, priv->devnode_input_pad,
				    MEDIA_LNK_FL_IMMUTABLE |
				    MEDIA_LNK_FL_ENABLED);
	if (ret) {
		v4l2_err(sd, "failed to create link to devnode source pad\n");
		goto unreg;
	}

	priv->m2m_dev = v4l2_m2m_init(&m2m_ops);
	if (IS_ERR(priv->m2m_dev)) {
		v4l2_err(&priv->sd, "Failed to init mem2mem device\n");
		ret = PTR_ERR(priv->m2m_dev);
		goto unreg;
	}

	/*
	 * set some defaults for output and capture image formats.
	 */
	outfmt = get_mbus_fmt(priv, V4L2_BUF_TYPE_VIDEO_OUTPUT);
	capfmt = get_mbus_fmt(priv, V4L2_BUF_TYPE_VIDEO_CAPTURE);

	outfmt->fmt.width = 640;
	outfmt->fmt.height = 480;
	imx_media_enum_format(&outfmt->fmt.code, 0, true, true);
	outfmt->cc = imx_media_find_format(0, outfmt->fmt.code, true, true);
	*capfmt = *outfmt;

	v4l2_info(sd, "Registered %s as /dev/%s\n", vfd->name,
		  video_device_node_name(vfd));

	return 0;
unreg:
	video_unregister_device(vfd);
	return ret;
}

static void m2mx_unregistered(struct v4l2_subdev *sd)
{
	struct m2mx_priv *priv = v4l2_get_subdevdata(sd);
	struct video_device *vfd = &priv->vfd;

	mutex_lock(&priv->dev_mutex);

	if (video_is_registered(vfd)) {
		v4l2_m2m_release(priv->m2m_dev);
		video_unregister_device(vfd);
		media_entity_cleanup(&vfd->entity);
	}

	mutex_unlock(&priv->dev_mutex);
}

static const struct v4l2_subdev_internal_ops m2mx_internal_ops = {
	.registered = m2mx_registered,
	.unregistered = m2mx_unregistered,
};

static struct v4l2_subdev_core_ops m2mx_core_ops = {
	.ioctl = m2mx_ioctl,
};

static struct media_entity_operations m2mx_entity_ops = {
	.link_setup = m2mx_link_setup,
	.link_validate = v4l2_subdev_link_validate,
};

static struct v4l2_subdev_pad_ops m2mx_pad_ops = {
	.enum_mbus_code = m2mx_enum_mbus_code,
	.get_fmt = m2mx_get_fmt,
	.set_fmt = m2mx_set_fmt,
};

static struct v4l2_subdev_ops m2mx_subdev_ops = {
	.pad = &m2mx_pad_ops,
	.core = &m2mx_core_ops,
};

static int m2mx_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct m2mx_priv *priv;
	struct video_device *vfd;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	platform_set_drvdata(pdev, priv);
	priv->dev = &pdev->dev;

	/* get our id */
	of_property_read_u32(np, "reg", &priv->id);

	mutex_init(&priv->dev_mutex);
	spin_lock_init(&priv->q_lock);

	v4l2_subdev_init(&priv->sd, &m2mx_subdev_ops);
	v4l2_set_subdevdata(&priv->sd, priv);
	priv->sd.internal_ops = &m2mx_internal_ops;
	priv->sd.entity.ops = &m2mx_entity_ops;
	priv->sd.entity.function = MEDIA_ENT_F_PROC_VIDEO_PIXEL_FORMATTER;
	priv->sd.grp_id = IMX_MEDIA_GRP_ID_M2M;
	priv->sd.dev = &pdev->dev;
	priv->sd.owner = THIS_MODULE;
	snprintf(m2mx_videodev.name, sizeof(m2mx_videodev.name),
		 "%s%d devnode", np->name, priv->id);
	snprintf(priv->sd.name, sizeof(priv->sd.name), "%s%d",
		 np->name, priv->id);

	priv->sd.flags = V4L2_SUBDEV_FL_HAS_DEVNODE;

	vfd = &priv->vfd;
	*vfd = m2mx_videodev;
	vfd->lock = &priv->dev_mutex;

	video_set_drvdata(vfd, priv);

	return v4l2_async_register_subdev(&priv->sd);
}

static int m2mx_remove(struct platform_device *pdev)
{
	struct m2mx_priv *priv =
		(struct m2mx_priv *)platform_get_drvdata(pdev);

	v4l2_async_unregister_subdev(&priv->sd);
	media_entity_cleanup(&priv->sd.entity);
	v4l2_device_unregister_subdev(&priv->sd);

	return 0;
}

static const struct of_device_id m2mx_dt_ids[] = {
	{ .compatible = "fsl,imx-media-mem2mem" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, m2mx_dt_ids);

static struct platform_driver m2mx_pdrv = {
	.probe		= m2mx_probe,
	.remove		= m2mx_remove,
	.driver		= {
		.name	= DEVICE_NAME,
		.owner	= THIS_MODULE,
		.of_match_table	= m2mx_dt_ids,
	},
};

module_platform_driver(m2mx_pdrv);

MODULE_DESCRIPTION("i.MX mem2mem subdev driver");
MODULE_AUTHOR("Steve Longerbeam <steve_longerbeam@mentor.com>");
MODULE_LICENSE("GPL");
