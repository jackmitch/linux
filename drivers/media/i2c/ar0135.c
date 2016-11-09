/*
 * Copyright (c) 2016 Jack Mitchell <jack@embed.me.uk>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/gpio/consumer.h>

#include <media/v4l2-subdev.h>
#include <media/v4l2-of.h>
#include <media/v4l2-clk.h>

struct ar0135 {
	struct v4l2_subdev subdev;
	struct i2c_client *i2c_client;
	struct media_pad pad;
	struct v4l2_clk *clk;
	struct mutex power_lock;
	struct v4l2_of_endpoint ep;
	struct v4l2_mbus_framefmt fmt;
	int power_count;
	u32 clk_rate;

        struct gpio_desc *reset_gpio;
};

struct reg_data {
	u16 reg;
	u16 value;
	u8 delete;
	u8 delete2;
};

#define AR0135_RESET_REG 0x301A
#define AR0135_RESET_REG_SOFT_RESET BIT(0)
#define AR0135_RESET_REG_STREAM BIT(2)
#define AR0135_RESET_REG_DISABLE_OUTPUT_DRIVERS BIT(6)
#define AR0135_RESET_REG_ENABLE_INPUT_PINS BIT(8)
#define AR0135_RESET_REG_FORCED_PLL BIT(11)


struct reg_data initial_data[] = {
	{0x301A,0x00d9,0,0}, // ASK PETE
	{0x301A,0x10d8,0,0}, // ASK PETE
	{0x3088,0x8000,0,0},
	{0x3086,0x3227,0,0},
	{0x3086,0x0101,0,0},
	{0x3086,0x0f25,0,0},
	{0x3086,0x0808,0,0},
	{0x3086,0x0227,0,0},
	{0x3086,0x0101,0,0},
	{0x3086,0x0837,0,0},
	{0x3086,0x2700,0,0},
	{0x3086,0x0138,0,0},
	{0x3086,0x2701,0,0},
	{0x3086,0x013a,0,0},
	{0x3086,0x2700,0,0},
	{0x3086,0x0125,0,0},
	{0x3086,0x0020,0,0},
	{0x3086,0x3c25,0,0},
	{0x3086,0x0040,0,0},
	{0x3086,0x3427,0,0},
	{0x3086,0x003f,0,0},
	{0x3086,0x2500,0,0},
	{0x3086,0x2037,0,0},
	{0x3086,0x2540,0,0},
	{0x3086,0x4036,0,0},
	{0x3086,0x2500,0,0},
	{0x3086,0x4031,0,0},
	{0x3086,0x2540,0,0},
	{0x3086,0x403d,0,0},
	{0x3086,0x6425,0,0},
	{0x3086,0x2020,0,0},
	{0x3086,0x3d64,0,0},
	{0x3086,0x2510,0,0},
	{0x3086,0x1037,0,0},
	{0x3086,0x2520,0,0},
	{0x3086,0x2010,0,0},
	{0x3086,0x2510,0,0},
	{0x3086,0x100f,0,0},
	{0x3086,0x2708,0,0},
	{0x3086,0x0802,0,0},
	{0x3086,0x2540,0,0},
	{0x3086,0x402d,0,0},
	{0x3086,0x2608,0,0},
	{0x3086,0x280d,0,0},
	{0x3086,0x1709,0,0},
	{0x3086,0x2600,0,0},
	{0x3086,0x2805,0,0},
	{0x3086,0x26a7,0,0},
	{0x3086,0x2807,0,0},
	{0x3086,0x2580,0,0},
	{0x3086,0x8029,0,0},
	{0x3086,0x1705,0,0},
	{0x3086,0x2500,0,0},
	{0x3086,0x4027,0,0},
	{0x3086,0x2222,0,0},
	{0x3086,0x1616,0,0},
	{0x3086,0x2726,0,0},
	{0x3086,0x2617,0,0},
	{0x3086,0x3626,0,0},
	{0x3086,0xa617,0,0},
	{0x3086,0x0326,0,0},
	{0x3086,0xa417,0,0},
	{0x3086,0x1f28,0,0},
	{0x3086,0x0526,0,0},
	{0x3086,0x2028,0,0},
	{0x3086,0x0425,0,0},
	{0x3086,0x2020,0,0},
	{0x3086,0x2700,0,0},
	{0x3086,0x2625,0,0},
	{0x3086,0x0000,0,0},
	{0x3086,0x171e,0,0},
	{0x3086,0x2500,0,0},
	{0x3086,0x0425,0,0},
	{0x3086,0x0020,0,0},
	{0x3086,0x2117,0,0},
	{0x3086,0x121b,0,0},
	{0x3086,0x1703,0,0},
	{0x3086,0x2726,0,0},
	{0x3086,0x2617,0,0},
	{0x3086,0x2828,0,0},
	{0x3086,0x0517,0,0},
	{0x3086,0x1a26,0,0},
	{0x3086,0x6017,0,0},
	{0x3086,0xae25,0,0},
	{0x3086,0x0080,0,0},
	{0x3086,0x2700,0,0},
	{0x3086,0x2626,0,0},
	{0x3086,0x1828,0,0},
	{0x3086,0x002e,0,0},
	{0x3086,0x2a28,0,0},
	{0x3086,0x081e,0,0},
	{0x3086,0x4127,0,0},
	{0x3086,0x1010,0,0},
	{0x3086,0x0214,0,0},
	{0x3086,0x6060,0,0},
	{0x3086,0x0a14,0,0},
	{0x3086,0x6060,0,0},
	{0x3086,0x0b14,0,0},
	{0x3086,0x6060,0,0},
	{0x3086,0x0c14,0,0},
	{0x3086,0x6060,0,0},
	{0x3086,0x0d14,0,0},
	{0x3086,0x6060,0,0},
	{0x3086,0x0217,0,0},
	{0x3086,0x3c14,0,0},
	{0x3086,0x0060,0,0},
	{0x3086,0x0a14,0,0},
	{0x3086,0x0060,0,0},
	{0x3086,0x0b14,0,0},
	{0x3086,0x0060,0,0},
	{0x3086,0x0c14,0,0},
	{0x3086,0x0060,0,0},
	{0x3086,0x0d14,0,0},
	{0x3086,0x0060,0,0},
	{0x3086,0x0811,0,0},
	{0x3086,0x2500,0,0},
	{0x3086,0x1027,0,0},
	{0x3086,0x0010,0,0},
	{0x3086,0x2f6f,0,0},
	{0x3086,0x0f3e,0,0},
	{0x3086,0x2500,0,0},
	{0x3086,0x0827,0,0},
	{0x3086,0x0008,0,0},
	{0x3086,0x3066,0,0},
	{0x3086,0x3225,0,0},
	{0x3086,0x0008,0,0},
	{0x3086,0x2700,0,0},
	{0x3086,0x0830,0,0},
	{0x3086,0x6631,0,0},
	{0x3086,0x3d64,0,0},
	{0x3086,0x2508,0,0},
	{0x3086,0x083d,0,0},
	{0x3086,0xff3d,0,0},
	{0x3086,0x2a27,0,0},
	{0x3086,0x083f,0,0},
	{0x3086,0x2c00,0,0},

	{0x301e,0x00a8,0,0},
	{0x3044,0x0400,0,0},

	{0x302c,0x0001,0,0},
	{0x302a,0x0008,0,0},
	{0x302e,0x0008,0,0},
	{0x3030,0x00c6,0,0},
	{0x30b0,0x0480,0,0},
	{0x3002,0x0000,0,0},
	{0x3004,0x0000,0,0},
	{0x3006,0x03bf,0,0},
	{0x3008,0x04ff,0,0},
	{0x300a,0x03e5,0,0},
	{0x300c,0x0672,0,0},
	{0x30b0,0x0080,0,0},
	{0x3012,0x0064,0,0},
	{0x30a2,0x0001,0,0},
	{0x30a6,0x0001,0,0},
	{0x3040,0x0000,0,0},
	{0x3032,0x0020,0,0},
	{0x3028,0x0010,0,0},
	{0x301a,0x10d8,0,0},
	{0x30d4,0x6007,0,0},
	{0x301a,0x10dc,0,0},
	{0x301a,0x10d8,0,0},
	{0x30d4,0xe007,0,0},

	{0x301a,0x10dc,0,0},
};

static struct ar0135 *client_to_ar0135(const struct i2c_client *client)
{
        return container_of(i2c_get_clientdata(client), struct ar0135, subdev);
}

static struct ar0135 *subdev_to_ar0135(const struct v4l2_subdev *sd)
{
        return container_of(sd, struct ar0135, subdev);
}

static int ar0135_write_register(struct i2c_client *client, u16 reg, u16 data)
{
	u8 buffer[4] = {0};
	int ret;

	buffer[0] = reg >> 8;
	buffer[1] = reg & 0xff;

	buffer[2] = data >> 8;
	buffer[3] = data & 0xff;

	ret = i2c_master_send(client, buffer, 4);
	if (ret < 0)
		dev_err(&client->dev, "%s: failed, reg %04x data %04x (%d)",
			__func__, reg, data, ret);

	return 0;
}

static int ar0135_load_registers(struct ar0135 *priv, struct reg_data *data,
				 int length)
{
	int i, ret;

	for (i=0; i<length; i++) {
		ret = ar0135_write_register(priv->i2c_client, data[i].reg,
					    data[i].value);
		if (ret)
			return ret;
		usleep_range(1000, 2000);
	}

	return 0;
}

static int ar0135_power(struct ar0135 *priv, int on)
{
	int ret;

	if (!on) {
		// reset high
		return 0;
	}

	ret = v4l2_clk_set_rate(priv->clk, priv->clk_rate);
	if (ret) {
		dev_err(priv->subdev.dev, "failed to set clk rate to %d",
			priv->clk_rate);
		return ret;
	}

	ret = v4l2_clk_enable(priv->clk);
	if (ret)
		return ret;

        gpiod_set_value(priv->reset_gpio, 1);
	usleep_range(8000, 10000);
        gpiod_set_value(priv->reset_gpio, 0);
	usleep_range(8000, 10000);
        gpiod_set_value(priv->reset_gpio, 1);
	usleep_range(8000, 10000);

	return ar0135_load_registers(priv, initial_data,
				     ARRAY_SIZE(initial_data));
}

static int ar0135_s_power(struct v4l2_subdev *sd, int on)
{
	struct ar0135 *priv = subdev_to_ar0135(sd);
	int ret = 0;

	mutex_lock(&priv->power_lock);

	if (on) {
		if (!priv->power_count)
			ret = ar0135_power(priv, 1);

		priv->power_count++;
	} else {
		if (priv->power_count == 1)
			ret = ar0135_power(priv, 0);
		else if (priv->power_count != 0)
			priv->power_count--;
	}

	mutex_unlock(&priv->power_lock);

	return ret;
}

static int ar0135_set_fmt(struct v4l2_subdev *sd,
                struct v4l2_subdev_pad_config *cfg,
                struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *mf = &format->format;
	struct ar0135 *priv = container_of(sd, struct ar0135, subdev);

	if (format->pad)
		return -EINVAL;

	if (mf->code != MEDIA_BUS_FMT_Y8_1X8) {
		dev_err(sd->dev, "unsupported MEDIA_BUS_FMT");
		return -EINVAL;
	}

	if (mf->width != priv->fmt.width || mf->height != priv->fmt.height) {
		dev_err(sd->dev, "unsupported image resolution");
		return -EINVAL;
	}

	if (format->which == V4L2_SUBDEV_FORMAT_TRY) {
		cfg->try_fmt = *mf;
		return 0;
	}

	priv->fmt = format->format;

	return 0;
}

static int ar0135_get_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	struct ar0135 *priv = container_of(sd, struct ar0135, subdev);

	if (format->pad)
		return -EINVAL;

	format->format = priv->fmt;

	return 0;
}

static int ar0135_enum_mbus_code(struct v4l2_subdev *sd,
                struct v4l2_subdev_pad_config *cfg,
                struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->pad || code->index)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_Y8_1X8;

	return 0;
}

static int ar0135_s_stream(struct v4l2_subdev *sd, int enable)
{
        return 0;
}

static int ar0135_g_mbus_config(struct v4l2_subdev *sd,
                                struct v4l2_mbus_config *cfg)
{
        struct ar0135 *priv = subdev_to_ar0135(sd);

        cfg->type = V4L2_MBUS_PARALLEL;
        cfg->flags = priv->ep.bus.parallel.flags;

        return 0;
}

static struct v4l2_subdev_core_ops ar0135_subdev_core_ops = {
        .s_power        = ar0135_s_power,
};

static const struct v4l2_subdev_pad_ops ar0135_subdev_pad_ops = {
        .enum_mbus_code = ar0135_enum_mbus_code,
        .get_fmt        = ar0135_get_fmt,
        .set_fmt        = ar0135_set_fmt,
};

static struct v4l2_subdev_video_ops ar0135_subdev_video_ops = {
        .s_stream = ar0135_s_stream,
	.g_mbus_config = ar0135_g_mbus_config,
};

static struct v4l2_subdev_ops ar0135_subdev_ops = {
        .core   = &ar0135_subdev_core_ops,
	.video	= &ar0135_subdev_video_ops,
	.pad	= &ar0135_subdev_pad_ops,
};

static int ar0135_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct ar0135 *priv;
	struct device_node *endpoint;
	int ret;

	priv = devm_kzalloc(dev, sizeof(struct ar0135),
			    GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->i2c_client = client;
	priv->subdev.dev = &client->dev;

	/* image size from intitial data array */
	priv->fmt.width = 1280;
	priv->fmt.height = 964;

        endpoint = of_graph_get_next_endpoint(dev->of_node, NULL);
        if (!endpoint) {
                dev_err(dev, "endpoint node not found\n");
                return -EINVAL;
        }

        v4l2_of_parse_endpoint(endpoint, &priv->ep);
        if (priv->ep.bus_type != V4L2_MBUS_PARALLEL) {
                dev_err(dev, "invalid bus type, must be parallel\n");
                return -EINVAL;
        }
        of_node_put(endpoint);

        mutex_init(&priv->power_lock);

	priv->clk = v4l2_clk_get(dev, "xclk");
	if (IS_ERR(priv->clk))
		return -EPROBE_DEFER;

	priv->clk_rate = 24000000;

        /* request reset pin */
        priv->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
        if (IS_ERR(priv->reset_gpio)) {
                dev_err(dev, "request for reset gpio failed\n");
                return PTR_ERR(priv->reset_gpio);
	}

	v4l2_i2c_subdev_init(&priv->subdev, client, &ar0135_subdev_ops);

	priv->subdev.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	priv->subdev.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

	priv->pad.flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&priv->subdev.entity, 1, &priv->pad);
	if (ret)
		goto err;

	ret = v4l2_async_register_subdev(&priv->subdev);
	if (ret)
		goto err;

	return 0;

err:
	v4l2_clk_put(priv->clk);

	return ret;
}

static int ar0135_remove(struct i2c_client *client)
{
	struct ar0135 *priv = client_to_ar0135(client);

	v4l2_async_unregister_subdev(&priv->subdev);
	v4l2_clk_put(priv->clk);

	return 0;
}

static const struct of_device_id ar0135_of_match[] = {
        { .compatible = "onsemi,ar0135", },
        {},
};
MODULE_DEVICE_TABLE(of, ar0135_of_match);

static const struct i2c_device_id ar0135_id[] = {
        { "ar0135", 0 },
        {},
};
MODULE_DEVICE_TABLE(i2c, ar0135_id);

static struct i2c_driver ar0135_i2c_driver = {
	.driver = {
		.name = "ar0135",
		.of_match_table = of_match_ptr(ar0135_of_match),
	},
	.probe 		= ar0135_probe,
	.remove 	= ar0135_remove,
	.id_table 	= ar0135_id,
};

module_i2c_driver(ar0135_i2c_driver);

MODULE_DESCRIPTION("OnSemi ar0135 Image Sensor");
MODULE_AUTHOR("Jack Mitchell <jack@embed.me.uk>");
MODULE_LICENSE("GPL");
