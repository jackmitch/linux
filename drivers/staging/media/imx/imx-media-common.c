/*
 * V4L2 Media Controller Driver for Freescale i.MX5/6 SOC
 *
 * Copyright (c) 2016 Mentor Graphics Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#include <linux/module.h>
#include "imx-media.h"

/*
 * List of pixel formats for the subdevs. TODO: This must be a super-set of
 * the formats supported by the ipu image converter.
 */
static const struct imx_media_pixfmt imx_media_formats[] = {
	{
		.fourcc	= V4L2_PIX_FMT_UYVY,
		.codes  = {MEDIA_BUS_FMT_UYVY8_2X8, MEDIA_BUS_FMT_UYVY8_1X16},
		.cs     = IPUV3_COLORSPACE_YUV,
		.bpp    = 16,
	}, {
		.fourcc	= V4L2_PIX_FMT_YUYV,
		.codes  = {MEDIA_BUS_FMT_YUYV8_2X8, MEDIA_BUS_FMT_YUYV8_1X16},
		.cs     = IPUV3_COLORSPACE_YUV,
		.bpp    = 16,
	}, {
		.fourcc	= V4L2_PIX_FMT_RGB565,
		.codes  = {MEDIA_BUS_FMT_RGB565_2X8_LE},
		.cs     = IPUV3_COLORSPACE_RGB,
		.bpp    = 16,
	}, {
		.fourcc	= V4L2_PIX_FMT_RGB24,
		.codes  = {MEDIA_BUS_FMT_RGB888_1X24,
			   MEDIA_BUS_FMT_RGB888_2X12_LE},
		.cs     = IPUV3_COLORSPACE_RGB,
		.bpp    = 24,
	}, {
		.fourcc	= V4L2_PIX_FMT_BGR24,
		.cs     = IPUV3_COLORSPACE_RGB,
		.bpp    = 24,
	}, {
		.fourcc	= V4L2_PIX_FMT_RGB32,
		.codes  = {MEDIA_BUS_FMT_ARGB8888_1X32},
		.cs     = IPUV3_COLORSPACE_RGB,
		.bpp    = 32,
	}, {
		.fourcc	= V4L2_PIX_FMT_BGR32,
		.cs     = IPUV3_COLORSPACE_RGB,
		.bpp    = 32,
	}, {
		.fourcc	= V4L2_PIX_FMT_YUV420,
		.cs     = IPUV3_COLORSPACE_YUV,
		.bpp    = 12,
		.planar = true,
	}, {
		.fourcc = V4L2_PIX_FMT_YVU420,
		.cs     = IPUV3_COLORSPACE_YUV,
		.bpp    = 12,
		.planar = true,
	}, {
		.fourcc = V4L2_PIX_FMT_YUV422P,
		.cs     = IPUV3_COLORSPACE_YUV,
		.bpp    = 16,
		.planar = true,
	}, {
		.fourcc = V4L2_PIX_FMT_NV12,
		.cs     = IPUV3_COLORSPACE_YUV,
		.bpp    = 12,
		.planar = true,
	}, {
		.fourcc = V4L2_PIX_FMT_NV16,
		.cs     = IPUV3_COLORSPACE_YUV,
		.bpp    = 16,
		.planar = true,
	}, {
               .fourcc = V4L2_PIX_FMT_GREY,
               .cs     = IPUV3_COLORSPACE_UNKNOWN,
               .bpp    = 8,
               .codes  = {MEDIA_BUS_FMT_Y8_1X8},
	},
};

const struct imx_media_pixfmt *imx_media_find_format(u32 fourcc, u32 code,
						     bool allow_rgb,
						     bool allow_planar)
{
	const struct imx_media_pixfmt *fmt, *ret = NULL;
	int i, j;

	for (i = 0; i < ARRAY_SIZE(imx_media_formats); i++) {
		fmt = &imx_media_formats[i];

		if (fourcc && fmt->fourcc == fourcc &&
		    (fmt->cs != IPUV3_COLORSPACE_RGB || allow_rgb) &&
		    (!fmt->planar || (allow_planar && fmt->codes[0]))) {
			ret = fmt;
			goto out;
		}

		for (j = 0; fmt->codes[j]; j++) {
			if (fmt->codes[j] == code &&
			    (fmt->cs != IPUV3_COLORSPACE_RGB || allow_rgb) &&
			    (!fmt->planar || allow_planar)) {
				ret = fmt;
				goto out;
			}
		}
	}
out:
	return ret;
}
EXPORT_SYMBOL_GPL(imx_media_find_format);

int imx_media_enum_format(u32 *code, u32 index, bool allow_rgb,
			  bool allow_planar)
{
	const struct imx_media_pixfmt *fmt;

	if (index >= ARRAY_SIZE(imx_media_formats))
		return -EINVAL;

	fmt = &imx_media_formats[index];
	if ((fmt->cs == IPUV3_COLORSPACE_RGB && !allow_rgb) ||
	    (fmt->planar && (!allow_planar || !fmt->codes[0])))
		return -EINVAL;

	*code = fmt->codes[0];
	return 0;
}
EXPORT_SYMBOL_GPL(imx_media_enum_format);

int imx_media_mbus_fmt_to_pix_fmt(struct v4l2_pix_format *pix,
				  struct v4l2_mbus_framefmt *mbus)
{
	const struct imx_media_pixfmt *fmt;
	u32 stride;

	fmt = imx_media_find_format(0, mbus->code, true, true);
	if (!fmt)
		return -EINVAL;

	stride = fmt->planar ? mbus->width : (mbus->width * fmt->bpp) >> 3;

	pix->width = mbus->width;
	pix->height = mbus->height;
	pix->pixelformat = fmt->fourcc;
	pix->field = mbus->field;
	pix->bytesperline = stride;
	pix->sizeimage = (pix->width * pix->height * fmt->bpp) >> 3;

	return 0;
}
EXPORT_SYMBOL_GPL(imx_media_mbus_fmt_to_pix_fmt);

int imx_media_mbus_fmt_to_ipu_image(struct ipu_image *image,
				    struct v4l2_mbus_framefmt *mbus)
{
	int ret;

	memset(image, 0, sizeof(*image));

	ret = imx_media_mbus_fmt_to_pix_fmt(&image->pix, mbus);
	if (ret)
		return ret;

	image->rect.width = mbus->width;
	image->rect.height = mbus->height;

	return 0;
}
EXPORT_SYMBOL_GPL(imx_media_mbus_fmt_to_ipu_image);

int imx_media_ipu_image_to_mbus_fmt(struct v4l2_mbus_framefmt *mbus,
				    struct ipu_image *image)
{
	const struct imx_media_pixfmt *fmt;

	fmt = imx_media_find_format(image->pix.pixelformat, 0, true, true);
	if (!fmt)
		return -EINVAL;

	memset(mbus, 0, sizeof(*mbus));
	mbus->width = image->pix.width;
	mbus->height = image->pix.height;
	mbus->code = fmt->codes[0];
	mbus->field = image->pix.field;

	return 0;
}
EXPORT_SYMBOL_GPL(imx_media_ipu_image_to_mbus_fmt);

struct imx_media_dma_buf_ring {
	struct imx_media_dev *imxmd;
	/* the ring */
	struct imx_media_dma_buf buf[IMX_MEDIA_MAX_RING_BUFS];
	/* buffer generator */
	struct v4l2_subdev *src_sd;
	/* buffer receiver */
	struct v4l2_subdev *sink_sd;
	spinlock_t lock;
	int num_bufs;
	int next_in;
	int next_out;
	int count;
};

void imx_media_free_dma_buf(struct imx_media_dev *imxmd,
			    struct imx_media_dma_buf *buf)
{
	if (buf->virt && !buf->vb)
		dma_free_coherent(imxmd->dev, buf->len, buf->virt, buf->phys);

	buf->virt = NULL;
	buf->phys = 0;
}
EXPORT_SYMBOL_GPL(imx_media_free_dma_buf);

int imx_media_alloc_dma_buf(struct imx_media_dev *imxmd,
			    struct imx_media_dma_buf *buf,
			    int size)
{
	imx_media_free_dma_buf(imxmd, buf);

	buf->vb = NULL;
	buf->len = PAGE_ALIGN(size);
	buf->virt = dma_alloc_coherent(imxmd->dev, buf->len, &buf->phys,
				       GFP_DMA | GFP_KERNEL);
	if (!buf->virt) {
		dev_err(imxmd->dev, "failed to alloc dma buffer\n");
		return -ENOMEM;
	}

	buf->status = IMX_MEDIA_BUF_STATUS_READY;
	return 0;
}
EXPORT_SYMBOL_GPL(imx_media_alloc_dma_buf);

void imx_media_free_dma_buf_ring(struct imx_media_dma_buf_ring *ring)
{
	int i;

	if (!ring)
		return;

	for (i = 0; i < ring->num_bufs; i++)
		imx_media_free_dma_buf(ring->imxmd, &ring->buf[i]);
	kfree(ring);
}
EXPORT_SYMBOL_GPL(imx_media_free_dma_buf_ring);

struct imx_media_dma_buf_ring *
imx_media_alloc_dma_buf_ring(struct imx_media_dev *imxmd,
			     struct v4l2_subdev *src_sd,
			     struct v4l2_subdev *sink_sd,
			     int size, int num_bufs,
			     bool alloc_bufs)
{
	struct imx_media_dma_buf_ring *ring;
	int i, ret;

	if (num_bufs < IMX_MEDIA_MIN_RING_BUFS ||
	    num_bufs > IMX_MEDIA_MAX_RING_BUFS)
		return ERR_PTR(-EINVAL);

	ring = kzalloc(sizeof(*ring), GFP_KERNEL);
	if (!ring)
		return ERR_PTR(-ENOMEM);

	spin_lock_init(&ring->lock);
	ring->imxmd = imxmd;
	ring->src_sd = src_sd;
	ring->sink_sd = sink_sd;
	ring->num_bufs = num_bufs;

	for (i = 0; i < num_bufs; i++) {
		if (alloc_bufs) {
			ret = imx_media_alloc_dma_buf(imxmd, &ring->buf[i],
						      size);
			if (ret) {
				ring->num_bufs = i;
				imx_media_free_dma_buf_ring(ring);
				kfree(ring);
				return ERR_PTR(ret);
			}
		}
		ring->buf[i].index = i;
	}

	return ring;
}
EXPORT_SYMBOL_GPL(imx_media_alloc_dma_buf_ring);

int imx_media_dma_buf_init_from_vb(struct imx_media_dma_buf_ring *ring,
				   struct vb2_buffer *vb)
{
	struct imx_media_dma_buf *buf;

	if (vb->index >= ring->num_bufs)
		return -EINVAL;

	buf = &ring->buf[vb->index];

	buf->virt = vb2_plane_vaddr(vb, 0);
	buf->phys = vb2_dma_contig_plane_dma_addr(vb, 0);
	buf->status = IMX_MEDIA_BUF_STATUS_READY;
	buf->vb = vb;

	return 0;
}
EXPORT_SYMBOL_GPL(imx_media_dma_buf_init_from_vb);

struct imx_media_dma_buf *
imx_media_dma_buf_peek(struct imx_media_dma_buf_ring *ring, int index)
{
	if (index > ring->num_bufs - 1)
		return ERR_PTR(-EINVAL);
	return &ring->buf[index];
}
EXPORT_SYMBOL_GPL(imx_media_dma_buf_peek);

struct imx_media_dma_buf *
imx_media_dma_buf_next_in(struct imx_media_dma_buf_ring *ring)
{
	struct imx_media_dma_buf *buf;
	unsigned long flags;

	spin_lock_irqsave(&ring->lock, flags);

	buf = &ring->buf[ring->next_in];
	if (ring->count < ring->num_bufs) {
		ring->next_in = (ring->next_in + 1) % ring->num_bufs;
		ring->count++;
	}
	buf->status = IMX_MEDIA_BUF_STATUS_INPROGRESS;

	spin_unlock_irqrestore(&ring->lock, flags);

	dev_dbg(ring->imxmd->dev, "buf%d inprogress by %s\n",
		buf->index, ring->src_sd->name);
	return buf;
}
EXPORT_SYMBOL_GPL(imx_media_dma_buf_next_in);

struct imx_media_dma_buf *
imx_media_dma_buf_next_out(struct imx_media_dma_buf_ring *ring)
{
	struct imx_media_dma_buf *buf = NULL;
	unsigned long flags;

	spin_lock_irqsave(&ring->lock, flags);
	if (ring->count) {
		buf = &ring->buf[ring->next_out];
		ring->next_out = (ring->next_out + 1) % ring->num_bufs;
		ring->count--;
	}
	spin_unlock_irqrestore(&ring->lock, flags);

	if (buf)
		dev_dbg(ring->imxmd->dev, "buf%d pulled by %s\n",
			buf->index, ring->sink_sd->name);

	return buf;
}
EXPORT_SYMBOL_GPL(imx_media_dma_buf_next_out);

int imx_media_dma_buf_return(struct imx_media_dma_buf_ring *ring,
			     enum imx_media_dma_buf_status status)
{
	struct imx_media_dma_buf *buf;
	unsigned long flags;

	spin_lock_irqsave(&ring->lock, flags);
	buf = &ring->buf[ring->next_out];
	buf->status = status;
	spin_unlock_irqrestore(&ring->lock, flags);

	dev_dbg(ring->imxmd->dev, "buf%d pushed %s -> %s\n",
		buf->index, ring->src_sd->name, ring->sink_sd->name);

	/* inform the sink that a new buffer has arrived */
	return v4l2_subdev_call(ring->sink_sd, core, ioctl,
				IMX_MEDIA_NEW_DMA_BUF, buf);
}
EXPORT_SYMBOL_GPL(imx_media_dma_buf_return);

struct imx_media_subdev *
imx_media_find_subdev_by_sd(struct imx_media_dev *imxmd,
			    struct v4l2_subdev *sd)
{
	struct imx_media_subdev *imxsd;
	int i, ret = -ENODEV;

	for (i = 0; i < imxmd->num_subdevs; i++) {
		imxsd = &imxmd->subdev[i];
		if (sd == imxsd->sd) {
			ret = 0;
			break;
		}
	}

	return ret ? ERR_PTR(ret) : imxsd;
}
EXPORT_SYMBOL_GPL(imx_media_find_subdev_by_sd);

struct imx_media_subdev *
imx_media_find_subdev_by_id(struct imx_media_dev *imxmd, u32 grp_id)
{
	struct imx_media_subdev *imxsd;
	int i, ret = -ENODEV;

	for (i = 0; i < imxmd->num_subdevs; i++) {
		imxsd = &imxmd->subdev[i];
		if (imxsd->sd && imxsd->sd->grp_id == grp_id) {
			ret = 0;
			break;
		}
	}

	return ret ? ERR_PTR(ret) : imxsd;
}
EXPORT_SYMBOL_GPL(imx_media_find_subdev_by_id);

static void imx_media_set_sensor(struct imx_media_dev *imxmd,
				 struct v4l2_subdev *sd, bool on)
{
	if (!on) {
		v4l2_info(&imxmd->v4l2_dev, "detached sensor\n");
		imxmd->sensor = NULL;
	} else {
		imxmd->sensor = imx_media_find_subdev_by_sd(imxmd, sd);
		if (IS_ERR(imxmd->sensor)) {
			v4l2_err(&imxmd->v4l2_dev,
				 "failed to find sensor %s\n", sd->name);
			imxmd->sensor = NULL;
		}
		v4l2_info(&imxmd->v4l2_dev, "attached sensor %s\n",
			  imxmd->sensor->sd->name);
	}
}

/*
 * Search for an entity in the current pipeline with given grp_id.
 * Called with mdev->graph_mutex held.
 */
static struct media_entity *
find_pipeline_entity(struct imx_media_dev *imxmd,
		     struct media_entity_graph *graph,
		     struct media_entity *start_entity,
		     u32 grp_id)
{
	struct media_entity *entity;
	struct v4l2_subdev *sd;

	media_entity_graph_walk_start(graph, start_entity);

	while ((entity = media_entity_graph_walk_next(graph))) {
		if (is_media_entity_v4l2_video_device(entity))
			continue;

		sd = media_entity_to_v4l2_subdev(entity);
		if (sd->grp_id & grp_id)
			return entity;
	}

	return NULL;
}

/*
 * Search for an entity in the current pipeline with given grp_id,
 * then locate the remote enabled source pad from that entity.
 * Called with mdev->graph_mutex held.
 */
static struct media_pad *
find_pipeline_remote_source_pad(struct imx_media_dev *imxmd,
				struct media_entity_graph *graph,
				struct media_entity *start_entity,
				u32 grp_id)
{
	struct media_pad *pad = NULL;
	struct media_entity *me;
	int i;

	me = find_pipeline_entity(imxmd, graph, start_entity, grp_id);
	if (!me)
		return NULL;

	/* Find remote source pad */
	for (i = 0; i < me->num_pads; i++) {
		struct media_pad *spad = &me->pads[i];
		if (!(spad->flags & MEDIA_PAD_FL_SINK))
			continue;
		pad = media_entity_remote_pad(spad);
		if (pad)
			return pad;
	}

	return NULL;
}

/*
 * Find the mipi-csi2 virtual channel reached from the given
 * start entity in the current pipeline.
 * Must be called with mdev->graph_mutex held.
 */
int imx_media_find_mipi_csi2_channel(struct imx_media_dev *imxmd,
				     struct media_entity *start_entity)
{
	struct media_entity_graph *graph = &imxmd->link_setup_graph;
	struct v4l2_subdev *sd;
	struct media_pad *pad;
	int ret;

	ret = media_entity_graph_walk_init(graph, &imxmd->md);
	if (ret)
		return ret;

	/* first try to locate the mipi-csi2 from the video mux */
	pad = find_pipeline_remote_source_pad(imxmd, graph, start_entity,
					      IMX_MEDIA_GRP_ID_VIDMUX);
	/* if couldn't reach it from there, try from a CSI */
	if (!pad)
		pad = find_pipeline_remote_source_pad(imxmd, graph, start_entity,
						      IMX_MEDIA_GRP_ID_CSI);
	if (pad) {
		sd = media_entity_to_v4l2_subdev(pad->entity);
		if (sd->grp_id & IMX_MEDIA_GRP_ID_CSI2) {
			ret = pad->index - 1; /* found it! */
			v4l2_info(&imxmd->v4l2_dev, "found vc %d from %s\n",
				  ret, start_entity->name);
			goto cleanup;
		}
	}

	ret = -EPIPE;

cleanup:
	media_entity_graph_walk_cleanup(graph);
	return ret;
}
EXPORT_SYMBOL_GPL(imx_media_find_mipi_csi2_channel);

/*
 * The subdevs have to be powered on/off, and streaming
 * enabled/disabled, in a specific sequence.
 */
static const u32 stream_on_seq[] = {
	IMX_MEDIA_GRP_ID_IC_PP,
	IMX_MEDIA_GRP_ID_IC_PRPVF,
	IMX_MEDIA_GRP_ID_IC_PRPENC,
	IMX_MEDIA_GRP_ID_SMFC,
	IMX_MEDIA_GRP_ID_SENSOR,
	IMX_MEDIA_GRP_ID_CSI2,
	IMX_MEDIA_GRP_ID_VIDMUX,
	IMX_MEDIA_GRP_ID_CSI,
};
static const u32 stream_off_seq[] = {
	IMX_MEDIA_GRP_ID_IC_PP,
	IMX_MEDIA_GRP_ID_IC_PRPVF,
	IMX_MEDIA_GRP_ID_IC_PRPENC,
	IMX_MEDIA_GRP_ID_SMFC,
	IMX_MEDIA_GRP_ID_CSI,
	IMX_MEDIA_GRP_ID_VIDMUX,
	IMX_MEDIA_GRP_ID_CSI2,
	IMX_MEDIA_GRP_ID_SENSOR,
};
#define NUM_STREAM_ENTITIES ARRAY_SIZE(stream_on_seq)

static const u32 power_on_seq[] = {
	IMX_MEDIA_GRP_ID_CSI2,
	IMX_MEDIA_GRP_ID_SENSOR,
	IMX_MEDIA_GRP_ID_VIDMUX,
	IMX_MEDIA_GRP_ID_CSI,
	IMX_MEDIA_GRP_ID_SMFC,
	IMX_MEDIA_GRP_ID_IC_PRPENC,
	IMX_MEDIA_GRP_ID_IC_PRPVF,
	IMX_MEDIA_GRP_ID_IC_PP,
};
static const u32 power_off_seq[] = {
	IMX_MEDIA_GRP_ID_IC_PP,
	IMX_MEDIA_GRP_ID_IC_PRPVF,
	IMX_MEDIA_GRP_ID_IC_PRPENC,
	IMX_MEDIA_GRP_ID_SMFC,
	IMX_MEDIA_GRP_ID_CSI,
	IMX_MEDIA_GRP_ID_VIDMUX,
	IMX_MEDIA_GRP_ID_SENSOR,
	IMX_MEDIA_GRP_ID_CSI2,
};
#define NUM_POWER_ENTITIES ARRAY_SIZE(power_on_seq)

static int imx_media_set_stream(struct imx_media_dev *imxmd,
				struct media_entity *start_entity,
				bool on)
{
	struct media_entity_graph *graph = &imxmd->link_setup_graph;
	struct media_entity *entity;
	struct v4l2_subdev *sd;
	int i, ret;
	u32 id;

	mutex_lock(&imxmd->md.graph_mutex);

	ret = media_entity_graph_walk_init(graph, &imxmd->md);
	if (ret)
		goto unlock;

	for (i = 0; i < NUM_STREAM_ENTITIES; i++) {
		id = on ? stream_on_seq[i] : stream_off_seq[i];
		entity = find_pipeline_entity(imxmd, graph, start_entity, id);
		if (!entity)
			continue;

		sd = media_entity_to_v4l2_subdev(entity);
		ret = v4l2_subdev_call(sd, video, s_stream, on);
		if (ret && ret != -ENOIOCTLCMD)
			break;
	}

	media_entity_graph_walk_cleanup(graph);
unlock:
	mutex_unlock(&imxmd->md.graph_mutex);

	return (ret && ret != -ENOIOCTLCMD) ? ret : 0;
}

/*
 * Turn current pipeline streaming on/off starting from entity.
 */
int imx_media_pipeline_set_stream(struct imx_media_dev *imxmd,
				  struct media_entity *entity,
				  struct media_pipeline *pipe,
				  bool on)
{
	int ret = 0;

	if (on) {
		ret = media_entity_pipeline_start(entity, pipe);
		if (ret)
			return ret;
		ret = imx_media_set_stream(imxmd, entity, true);
		if (!ret)
			return 0;
		/* fall through */
	}

	imx_media_set_stream(imxmd, entity, false);
	media_entity_pipeline_stop(entity);

	return ret;
}
EXPORT_SYMBOL_GPL(imx_media_pipeline_set_stream);

/*
 * Turn current pipeline power on/off starting from start_entity.
 * Must be called with mdev->graph_mutex held.
 */
int imx_media_pipeline_set_power(struct imx_media_dev *imxmd,
				 struct media_entity *start_entity, bool on)
{
	struct media_entity_graph *graph = &imxmd->link_setup_graph;
	struct media_entity *entity;
	struct v4l2_subdev *sd;
	int i, ret;
	u32 id;

	for (i = 0; i < NUM_POWER_ENTITIES; i++) {
		id = on ? power_on_seq[i] : power_off_seq[i];
		entity = find_pipeline_entity(imxmd, graph, start_entity, id);
		if (!entity)
			continue;

		sd = media_entity_to_v4l2_subdev(entity);

		if (sd->grp_id == IMX_MEDIA_GRP_ID_SENSOR)
			imx_media_set_sensor(imxmd, sd, on);

		ret = v4l2_subdev_call(sd, core, s_power, on);
		if (ret && ret != -ENOIOCTLCMD)
			break;
	}

	return (ret && ret != -ENOIOCTLCMD) ? ret : 0;
}
EXPORT_SYMBOL_GPL(imx_media_pipeline_set_power);

MODULE_DESCRIPTION("i.MX5/6 v4l2 media controller driver");
MODULE_AUTHOR("Mentor Graphics Inc.");
MODULE_LICENSE("GPL");
