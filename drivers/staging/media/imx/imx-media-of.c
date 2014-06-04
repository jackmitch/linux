/*
 * Media driver for Freescale i.MX5/6 SOC
 *
 * Open Firmware parsing.
 *
 * Copyright (c) 2016 Mentor Graphics Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#include <linux/of_platform.h>
#include <media/v4l2-device.h>
#include <media/videobuf2-dma-contig.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-of.h>
#include <media/v4l2-ctrls.h>
#include <video/imx-ipu-v3.h>
#include "imx-media.h"

struct imx_media_subdev *
imx_media_of_find_subdev_by_node(struct imx_media_dev *imxmd,
				 struct device_node *np)
{
	struct imx_media_subdev *imxsd, *ret = NULL;
	int i;

	for (i = 0; np && i < imxmd->subdev_notifier.num_subdevs; i++) {
		imxsd = &imxmd->subdev[i];
		if (imxsd->asd.match.of.node == np) {
			ret = imxsd;
			break;
		}
	}

	return ret;
}

static struct imx_media_subdev *add_async_subdev(struct imx_media_dev *imxmd,
						 struct device_node *np)
{
	struct imx_media_subdev *imxsd;
	struct v4l2_async_subdev *asd;
	int sd_idx;

	/* return NULL if this subdev already added */
	if (imx_media_of_find_subdev_by_node(imxmd, np)) {
		dev_dbg(imxmd->dev, "%s: already added %s\n",
			__func__, np->name);
		return NULL;
	}

	sd_idx = imxmd->subdev_notifier.num_subdevs;
	if (sd_idx >= IMX_MEDIA_MAX_SUBDEVS) {
		dev_err(imxmd->dev, "%s: too many subdevs! can't add %s\n",
			__func__, np->name);
		return ERR_PTR(-ENOSPC);
	}

	imxsd = &imxmd->subdev[sd_idx];

	asd = &imxsd->asd;
	asd->match_type = V4L2_ASYNC_MATCH_OF;
	asd->match.of.node = np;

	imxmd->async_ptrs[sd_idx] = asd;
	imxmd->subdev_notifier.num_subdevs++;

	dev_dbg(imxmd->dev, "%s: added %s, num_subdevs=%d\n",
		__func__, np->name, imxmd->subdev_notifier.num_subdevs);

	return imxsd;
}

static int add_pad_link(struct imx_media_dev *imxmd,
			struct imx_media_pad *pad,
			struct device_node *local_sd_node,
			struct device_node *remote_sd_node,
			struct device_node *local_epnode,
			struct device_node *remote_epnode)
{
	struct imx_media_of_link *link;
	int link_idx;

	link_idx = pad->num_links;
	if (link_idx >= IMX_MEDIA_MAX_LINKS) {
		dev_err(imxmd->dev, "%s: too many links!\n", __func__);
		return -ENOSPC;
	}

	link = &pad->link[link_idx];

	link->local_sd_node = local_sd_node;
	link->remote_sd_node = remote_sd_node;

	v4l2_of_parse_endpoint(local_epnode, &link->local_ep);
	if (remote_epnode)
		v4l2_of_parse_endpoint(remote_epnode, &link->remote_ep);

	pad->num_links++;

	dev_dbg(imxmd->dev, "%s: added %s:%d -> %s:%d\n", __func__,
		local_sd_node->name, link->local_ep.base.port,
		remote_sd_node ? remote_sd_node->name : NULL,
		link->remote_ep.base.port);

	return 0;
}

/* parse inputs property from a sensor node */
static void of_parse_sensor_inputs(struct imx_media_dev *imxmd,
				   struct imx_media_subdev *sensor,
				   struct device_node *sensor_np)
{
	struct imx_media_sensor_input *sinput = &sensor->input;
	int ret, i;

	for (i = 0; i < IMX_MEDIA_MAX_SENSOR_INPUTS; i++) {
		const char *input_name;
		u32 val;

		ret = of_property_read_u32_index(sensor_np, "inputs", i, &val);
		if (ret)
			break;

		sinput->value[i] = val;

		ret = of_property_read_string_index(sensor_np, "input-names",
						    i, &input_name);
		/*
		 * if input-names not provided, they will be set using
		 * the subdev name once the sensor is known during
		 * async bind
		 */
		if (!ret)
			strncpy(sinput->name[i], input_name,
				sizeof(sinput->name[i]));
	}

	sinput->num = i;

	/* if no inputs provided just assume a single input */
	if (sinput->num == 0)
		sinput->num = 1;
}

static int of_get_port_count(const struct device_node *np)
{
	struct device_node *child;
	int num = 0;

	for_each_child_of_node(np, child) {
		if (of_node_cmp(child->name, "port") == 0)
			num++;
	}
	return num;
}

static int of_parse_subdev(struct imx_media_dev *imxmd,
			   struct device_node *sd_np)
{
	struct device_node *epnode, *remote_epnode, *rpp, *port;
	struct imx_media_subdev *imxsd;
	struct imx_media_pad *pad;
	int i, val, num_pads, ret;

	if (!of_device_is_available(sd_np)) {
		dev_dbg(imxmd->dev, "%s: %s not enabled\n", __func__,
			sd_np->name);
		return 0;
	}

	/* register this subdev with async notifier */
	imxsd = add_async_subdev(imxmd, sd_np);
	if (!imxsd)
		return 0;
	if (IS_ERR(imxsd))
		return PTR_ERR(imxsd);

	ret = of_property_read_u32(sd_np, "sink-ports", &val);
	if (ret) {
		ret = 0;
		val = 0;
	}

	imxsd->num_sink_pads = val;
	num_pads = of_get_port_count(sd_np);
	if (imxsd->num_sink_pads >= num_pads)
		return -EINVAL;

	imxsd->num_src_pads = num_pads - imxsd->num_sink_pads;

	dev_dbg(imxmd->dev, "%s: %s has %d pads (%d sink, %d src)\n",
		__func__, sd_np->name, num_pads, imxsd->num_sink_pads,
		imxsd->num_src_pads);

	if (imxsd->num_sink_pads == 0) {
		/* this might be a sensor, look for input property */
		of_parse_sensor_inputs(imxmd, imxsd, sd_np);
	}

	for (i = 0; i < num_pads; i++) {
		/* init this pad */
		pad = &imxsd->pad[i];
		pad->pad.flags = (i < imxsd->num_sink_pads) ?
			MEDIA_PAD_FL_SINK : MEDIA_PAD_FL_SOURCE;

		port = of_graph_get_port_by_id(sd_np, i);
		if (!port) {
			v4l2_err(&imxmd->v4l2_dev, "missing ports on %s\n",
				 sd_np->name);
			ret = -EINVAL;
			break;
		}

		epnode = NULL;
		while ((epnode = of_get_next_child(port, epnode))) {
			remote_epnode = of_parse_phandle(epnode,
							 "remote-endpoint", 0);
			rpp = of_graph_get_remote_port_parent(epnode);

			if (rpp && of_device_is_available(rpp)) {
				ret = add_pad_link(imxmd, pad, sd_np, rpp,
						   epnode, remote_epnode);
				if (ret)
					break;

				if (i < imxsd->num_sink_pads) {
					/* follow sink endpoints upstream */
					ret = of_parse_subdev(imxmd, rpp);
					if (ret)
						break;
				}
			}

			of_node_put(rpp);
			of_node_put(epnode);
			of_node_put(remote_epnode);

		}

		of_node_put(port);
		if (ret) {
			of_node_put(rpp);
			of_node_put(epnode);
			of_node_put(remote_epnode);
			break;
		}
	}

	return ret;
}

int imx_media_of_parse(struct imx_media_dev *imxmd, struct device_node *np)
{
	struct device_node *child;
	int ret;

	for_each_child_of_node(np, child) {
		ret = of_parse_subdev(imxmd, child);
		if (ret)
			break;

		of_node_put(child);
	}

	if (ret)
		of_node_put(child);

	return ret;
}
