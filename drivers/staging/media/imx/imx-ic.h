/*
 * Media Driver for Freescale i.MX5/6 SOC
 *
 * Copyright (c) 2016 Mentor Graphics Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#ifndef _IMX_IC_H
#define _IMX_IC_H

struct imx_ic_priv {
	struct device *dev;
	struct ipu_soc *ipu;
	struct v4l2_subdev sd;
	int    task_id;
	void   *task_priv;
};

struct imx_ic_ops {
	struct v4l2_subdev_ops *subdev_ops;
	struct v4l2_subdev_internal_ops *internal_ops;
	struct media_entity_operations *entity_ops;

	int (*init)(struct imx_ic_priv *ic_priv);
	void (*remove)(struct imx_ic_priv *ic_priv);
};

extern struct imx_ic_ops imx_ic_prpenc_ops;
extern struct imx_ic_ops imx_ic_prpvf_ops;
extern struct imx_ic_ops imx_ic_pp_ops;

#endif

