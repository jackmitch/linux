/*
 * V4L2 Capture IC Subdev for Freescale i.MX5/6 SOC
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
#include "imx-media.h"
#include "imx-ic.h"

static struct imx_ic_ops *ic_ops[IC_NUM_TASKS] = {
	[IC_TASK_ENCODER]        = &imx_ic_prpenc_ops,
	[IC_TASK_VIEWFINDER]     = &imx_ic_prpvf_ops,
	[IC_TASK_POST_PROCESSOR] = &imx_ic_pp_ops,
};

static u32 ic_grp_id[IC_NUM_TASKS] = {
	[IC_TASK_ENCODER]        = IMX_MEDIA_GRP_ID_IC_PRPENC,
	[IC_TASK_VIEWFINDER]     = IMX_MEDIA_GRP_ID_IC_PRPVF,
	[IC_TASK_POST_PROCESSOR] = IMX_MEDIA_GRP_ID_IC_PP,
};

static int imx_ic_probe(struct platform_device *pdev)
{
	struct ipu_client_platformdata *pdata;
	struct imx_ic_priv *priv;
	int ret;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	platform_set_drvdata(pdev, &priv->sd);
	priv->dev = &pdev->dev;

	/* get parent IPU */
	priv->ipu = dev_get_drvdata(priv->dev->parent);

	/* get our IC task id */
	pdata = priv->dev->platform_data;
	priv->task_id = pdata->ic;

	v4l2_subdev_init(&priv->sd, ic_ops[priv->task_id]->subdev_ops);
	v4l2_set_subdevdata(&priv->sd, priv);
	priv->sd.internal_ops = ic_ops[priv->task_id]->internal_ops;
	priv->sd.entity.ops = ic_ops[priv->task_id]->entity_ops;
	priv->sd.entity.function = MEDIA_ENT_F_PROC_VIDEO_SCALER;
	priv->sd.grp_id = ic_grp_id[priv->task_id];
	priv->sd.dev = &pdev->dev;
	priv->sd.owner = THIS_MODULE;
	priv->sd.flags = V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;

	ret = ic_ops[priv->task_id]->init(priv);
	if (ret)
		return ret;

	return v4l2_async_register_subdev(&priv->sd);
}

static int imx_ic_remove(struct platform_device *pdev)
{
	struct v4l2_subdev *sd = platform_get_drvdata(pdev);
	struct imx_ic_priv *priv = container_of(sd, struct imx_ic_priv, sd);

	ic_ops[priv->task_id]->remove(priv);

	v4l2_async_unregister_subdev(&priv->sd);
	media_entity_cleanup(&priv->sd.entity);
	v4l2_device_unregister_subdev(sd);

	return 0;
}

static const struct platform_device_id imx_ic_ids[] = {
	{ .name = "imx-ipuv3-ic" },
	{ },
};
MODULE_DEVICE_TABLE(platform, imx_ic_ids);

static struct platform_driver imx_ic_driver = {
	.probe = imx_ic_probe,
	.remove = imx_ic_remove,
	.id_table = imx_ic_ids,
	.driver = {
		.name = "imx-ipuv3-ic",
		.owner = THIS_MODULE,
	},
};
module_platform_driver(imx_ic_driver);

MODULE_AUTHOR("Mentor Graphics Inc.");
MODULE_DESCRIPTION("i.MX IC subdev driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:imx-ipuv3-ic");
