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
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/timer.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_platform.h>
#include <linux/mxc_icap.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-event.h>
#include <media/v4l2-mc.h>
#include <video/imx-ipu-v3.h>
#include <media/imx.h>
#include "imx-media.h"
#include "imx-media-of.h"

#define DEVICE_NAME "imx-media"

static inline struct imx_media_dev *notifier2dev(struct v4l2_async_notifier *n)
{
	return container_of(n, struct imx_media_dev, subdev_notifier);
}

/* async subdev bound notifier */
static int imx_media_subdev_bound(struct v4l2_async_notifier *notifier,
				  struct v4l2_subdev *sd,
				  struct v4l2_async_subdev *asd)
{
	struct imx_media_dev *imxmd = notifier2dev(notifier);
	struct imx_media_subdev *imxsd;
	int i, ret = -EINVAL;

	for (i = 0; i < imxmd->num_subdevs; i++) {
		imxsd = &imxmd->subdev[i];
		if (sd->dev->of_node == imxsd->asd.match.of.node) {
			imxsd->sd = sd;
			ret = 0;
			break;
		}
 	}
	if (ret)
		goto out;

	/* is this a sensor? */
	if (imxsd->num_sink_pads == 0 &&
	    ((sd->entity.flags & (MEDIA_ENT_F_CAM_SENSOR |
				  MEDIA_ENT_F_ATV_DECODER)) ||
	     (sd->entity.function & (MEDIA_ENT_F_CAM_SENSOR |
				     MEDIA_ENT_F_ATV_DECODER)))) {
		sd->grp_id = IMX_MEDIA_GRP_ID_SENSOR;

		/* set sensor input names if needed */
		for (i = 0; i < imxsd->input.num; i++) {
			if (strlen(imxsd->input.name[i]))
				continue;
			snprintf(imxsd->input.name[i],
				 sizeof(imxsd->input.name[i]),
				 "%s-%d", sd->name, i);
		}
	}

out:
	if (ret)
		v4l2_warn(&imxmd->v4l2_dev, "Received unknown subdev %s\n",
			  sd->name);
	else
		v4l2_info(&imxmd->v4l2_dev, "Registered subdev %s\n", sd->name);

	return ret;
}

static int imx_media_create_links(struct imx_media_dev *imxmd)
{
	struct imx_media_subdev *local_sd;
	struct imx_media_subdev *remote_sd;
	struct v4l2_subdev *source, *sink;
	struct imx_media_of_link *link;
	struct imx_media_pad *pad;
	u16 source_pad, sink_pad;
	int num_pads, i, j, k;
	int ret = 0;

	for (i = 0; i < imxmd->num_subdevs; i++) {
		local_sd = &imxmd->subdev[i];
		num_pads = local_sd->num_sink_pads + local_sd->num_src_pads;

		for (j = 0; j < num_pads; j++) {
			pad = &local_sd->pad[j];
			for (k = 0; k < pad->num_links; k++) {
				link = &pad->link[k];

				remote_sd = imx_media_of_find_subdev_by_node(
					imxmd, link->remote_sd_node);
				if (!remote_sd) {
					v4l2_warn(&imxmd->v4l2_dev,
						  "%s: no remote for %s:%d\n",
						  __func__, local_sd->sd->name,
						  link->local_ep.base.port);
					continue;
				}

				/* only create the source->sink links */
				if (pad->pad.flags & MEDIA_PAD_FL_SINK)
					continue;

				source = local_sd->sd;
				sink = remote_sd->sd;
				source_pad = link->local_ep.base.port;
				sink_pad = link->remote_ep.base.port;

				v4l2_info(&imxmd->v4l2_dev,
					  "%s: %s:%d -> %s:%d\n", __func__,
					  source->name, source_pad,
					  sink->name, sink_pad);

				ret = media_create_pad_link(&source->entity,
							    source_pad,
							    &sink->entity,
							    sink_pad,
							    0);
				if (ret) {
					v4l2_err(&imxmd->v4l2_dev,
						 "create_pad_link failed: %d\n",
						 ret);
					goto out;
				}
			}
		}
	}

out:
	return ret;
}

/* async subdev complete notifier */
static int imx_media_probe_complete(struct v4l2_async_notifier *notifier)
{
	struct imx_media_dev *imxmd = notifier2dev(notifier);
	int ret;

	mutex_lock(&imxmd->md.graph_mutex);

	ret = imx_media_create_links(imxmd);
	if (ret)
		goto unlock;

	ret = v4l2_device_register_subdev_nodes(&imxmd->v4l2_dev);
unlock:
	mutex_unlock(&imxmd->md.graph_mutex);
	if (ret)
		return ret;

	return media_device_register(&imxmd->md);
}

static int imx_media_link_notify(struct media_link *link, unsigned int flags,
				 unsigned int notification)
{
	struct media_entity *sink = link->sink->entity;
	struct v4l2_subdev *sink_sd;
	struct imx_media_dev *imxmd;
	struct media_entity_graph *graph;
	int ret = 0;

	if (is_media_entity_v4l2_video_device(sink))
		return 0;
	sink_sd = media_entity_to_v4l2_subdev(sink);
	imxmd = dev_get_drvdata(sink_sd->v4l2_dev->dev);
	graph = &imxmd->link_setup_graph;

	if (notification == MEDIA_DEV_NOTIFY_PRE_LINK_CH) {
		ret = media_entity_graph_walk_init(graph, &imxmd->md);
		if (ret)
			return ret;

		if (!(flags & MEDIA_LNK_FL_ENABLED)) {
			/* Before link disconnection */
			ret = imx_media_pipeline_set_power(imxmd, sink, false);
		}
	} else if (notification == MEDIA_DEV_NOTIFY_POST_LINK_CH) {
		if (link->flags & MEDIA_LNK_FL_ENABLED) {
			/* After link activation */
			ret = imx_media_pipeline_set_power(imxmd, sink, true);
		}

		media_entity_graph_walk_cleanup(graph);
	}

        return ret ? -EPIPE : 0;
}

static const struct media_device_ops imx_media_md_ops = {
	.link_notify = imx_media_link_notify,
};

static int imx_media_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	struct imx_media_dev *imxmd;
	int ret;

	imxmd = devm_kzalloc(dev, sizeof(*imxmd), GFP_KERNEL);
	if (!imxmd)
		return -ENOMEM;

	imxmd->dev = dev;
	dev_set_drvdata(dev, imxmd);

	strlcpy(imxmd->md.model, DEVICE_NAME, sizeof(imxmd->md.model));
	imxmd->md.ops = &imx_media_md_ops;
	imxmd->md.dev = dev;

	imxmd->v4l2_dev.mdev = &imxmd->md;
	strlcpy(imxmd->v4l2_dev.name, DEVICE_NAME,
		sizeof(imxmd->v4l2_dev.name));

	media_device_init(&imxmd->md);

	ret = v4l2_device_register(dev, &imxmd->v4l2_dev);
	if (ret < 0) {
		v4l2_err(&imxmd->v4l2_dev,
			 "Failed to register v4l2_device: %d\n", ret);
		return ret;
	}

	/* FIXME: will this get overriden? */
	dev_set_drvdata(imxmd->v4l2_dev.dev, imxmd);

	ret = imx_media_of_parse(imxmd, node);
	if (ret) {
		v4l2_err(&imxmd->v4l2_dev,
			 "imx_media_of_parse failed with %d\n", ret);
		goto unreg_dev;
	}

	/* no subdevs? just bail for this media device */
	imxmd->num_subdevs = imxmd->subdev_notifier.num_subdevs;
	if (imxmd->num_subdevs == 0) {
		ret = -ENODEV;
		goto unreg_dev;
	}

	/* prepare the async subdev notifier and register it */
	imxmd->subdev_notifier.subdevs = imxmd->async_ptrs;
	imxmd->subdev_notifier.bound = imx_media_subdev_bound;
	imxmd->subdev_notifier.complete = imx_media_probe_complete;
	ret = v4l2_async_notifier_register(&imxmd->v4l2_dev,
					   &imxmd->subdev_notifier);
	if (ret) {
		v4l2_err(&imxmd->v4l2_dev,
			 "v4l2_async_notifier_register failed with %d\n", ret);
		goto unreg_dev;
	}

	return 0;

unreg_dev:
	v4l2_device_unregister(&imxmd->v4l2_dev);
	return ret;
}

static int imx_media_remove(struct platform_device *pdev)
{
	struct imx_media_dev *imxmd =
		(struct imx_media_dev *)platform_get_drvdata(pdev);

	v4l2_info(&imxmd->v4l2_dev, "Removing " DEVICE_NAME "\n");

	v4l2_async_notifier_unregister(&imxmd->subdev_notifier);
	v4l2_device_unregister(&imxmd->v4l2_dev);
	media_device_unregister(&imxmd->md);
	media_device_cleanup(&imxmd->md);

	return 0;
}

static const struct of_device_id imx_media_dt_ids[] = {
	{ .compatible = "fsl,imx-media" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_media_dt_ids);

static struct platform_driver imx_media_pdrv = {
	.probe		= imx_media_probe,
	.remove		= imx_media_remove,
	.driver		= {
		.name	= DEVICE_NAME,
		.owner	= THIS_MODULE,
		.of_match_table	= imx_media_dt_ids,
	},
};

module_platform_driver(imx_media_pdrv);

MODULE_DESCRIPTION("i.MX5/6 v4l2 media controller driver");
MODULE_AUTHOR("Mentor Graphics Inc.");
MODULE_LICENSE("GPL");
