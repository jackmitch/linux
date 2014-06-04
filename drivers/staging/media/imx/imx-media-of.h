/*
 * V4L2 Media Controller Driver for Freescale i.MX5/6 SOC
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
#ifndef _IMX_MEDIA_OF_H
#define _IMX_MEDIA_OF_H

struct imx_media_subdev *
imx_media_of_find_subdev_by_node(struct imx_media_dev *imxmd,
				 struct device_node *np);

int imx_media_of_parse(struct imx_media_dev *dev, struct device_node *np);

#endif
