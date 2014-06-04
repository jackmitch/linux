/*
 * Frame Interval Monitor Control Indexes and default values
 */
enum {
	FIM_CL_ENABLE = 0,
	FIM_CL_NUM,
	FIM_CL_TOLERANCE_MIN,
	FIM_CL_TOLERANCE_MAX,
	FIM_CL_NUM_SKIP,
	FIM_NUM_CONTROLS,
};

#define FIM_CL_ENABLE_DEF      0 /* FIM disabled by default */
#define FIM_CL_NUM_DEF         8 /* average 8 frames */
#define FIM_CL_NUM_SKIP_DEF    2 /* skip 2 frames after restart */
#define FIM_CL_TOLERANCE_MIN_DEF  50 /* usec */
#define FIM_CL_TOLERANCE_MAX_DEF   0 /* no max tolerance (unbounded) */

/* frame interval monitor */
struct imxcam_fim {
	/* control cluster */
	struct v4l2_ctrl  *ctrl[FIM_NUM_CONTROLS];

	/* default ctrl values parsed from device tree */
	u32               of_defaults[FIM_NUM_CONTROLS];

	/* current control values */
	bool              enabled;
	int               num_avg;
	int               num_skip;
	unsigned long     tolerance_min; /* usec */
	unsigned long     tolerance_max; /* usec */

	int               counter;
	struct timespec   last_ts;
	unsigned long     sum;       /* usec */
	unsigned long     nominal;   /* usec */

	/*
	 * input capture method of measuring FI (channel and flags
	 * from device tree)
	 */
	int               icap_channel;
	int               icap_flags;
	struct completion icap_first_event;

	/*
	 * otherwise, the EOF method of measuring FI, called by
	 * streaming subdevs from eof irq
	 */
	int (*eof)(struct imxcam_dev *dev, struct timespec *ts);
};


static inline struct imxcam_dev *fim2dev(struct imxcam_fim *fim)
{
	return container_of(fim, struct imxcam_dev, fim);
}

static int fim_request_input_capture(struct imxcam_dev *dev);
static void fim_free_input_capture(struct imxcam_dev *dev);

static void update_fim(struct imxcam_dev *dev)
{
	struct imxcam_fim *fim = &dev->fim;

	if (dev->sensor_tpf.denominator == 0) {
		fim->enabled = false;
		return;
	}

	fim->nominal = DIV_ROUND_CLOSEST(
		1000 * 1000 * dev->sensor_tpf.numerator,
		dev->sensor_tpf.denominator);
}

static void reset_fim(struct imxcam_dev *dev, bool curval)
{
	struct imxcam_fim *fim = &dev->fim;
	struct v4l2_ctrl *en = fim->ctrl[FIM_CL_ENABLE];
	struct v4l2_ctrl *num = fim->ctrl[FIM_CL_NUM];
	struct v4l2_ctrl *skip = fim->ctrl[FIM_CL_NUM_SKIP];
	struct v4l2_ctrl *tol_min = fim->ctrl[FIM_CL_TOLERANCE_MIN];
	struct v4l2_ctrl *tol_max = fim->ctrl[FIM_CL_TOLERANCE_MAX];
	unsigned long flags;

	spin_lock_irqsave(&dev->irqlock, flags);

	if (curval) {
		fim->enabled = en->cur.val;
		fim->num_avg = num->cur.val;
		fim->num_skip = skip->cur.val;
		fim->tolerance_min = tol_min->cur.val;
		fim->tolerance_max = tol_max->cur.val;
	} else {
		fim->enabled = en->val;
		fim->num_avg = num->val;
		fim->num_skip = skip->val;
		fim->tolerance_min = tol_min->val;
		fim->tolerance_max = tol_max->val;
	}

	/* disable tolerance range if max <= min */
	if (fim->tolerance_max <= fim->tolerance_min)
		fim->tolerance_max = 0;

	fim->counter = -fim->num_skip;
	fim->sum = 0;

	spin_unlock_irqrestore(&dev->irqlock, flags);
}

/*
 * Monitor an averaged frame interval. If the average deviates too much
 * from the sensor's nominal frame rate, return -EIO. The frame intervals
 * are averaged in order to quiet noise from (presumably random) interrupt
 * latency.
 */
static int frame_interval_monitor(struct imxcam_fim *fim, struct timespec *ts)
{
	unsigned long interval, error, error_avg;
	struct imxcam_dev *dev = fim2dev(fim);
	struct timespec diff;
	int ret = 0;

	if (!vb2_is_streaming(&dev->buffer_queue) || !fim->enabled ||
	    ++fim->counter <= 0)
		goto out_update_ts;

	diff = timespec_sub(*ts, fim->last_ts);
	interval = diff.tv_sec * 1000 * 1000 + diff.tv_nsec / 1000;
	error = abs(interval - fim->nominal);

	if (fim->tolerance_max && error >= fim->tolerance_max) {
		dev_dbg(dev->dev,
			"FIM: %lu ignored, out of tolerance bounds\n",
			error);
		fim->counter--;
		goto out_update_ts;
	}

	fim->sum += error;

	if (fim->counter == fim->num_avg) {
		error_avg = DIV_ROUND_CLOSEST(fim->sum, fim->num_avg);

		if (error_avg > fim->tolerance_min)
			ret = -EIO;

		dev_dbg(dev->dev, "FIM: error: %lu usec%s\n",
			error_avg, ret ? " (!!!)" : "");

		fim->counter = 0;
		fim->sum = 0;
	}

out_update_ts:
	fim->last_ts = *ts;
	return ret;
}

/*
 * Called by the encode and vdic subdevs in their EOF interrupt
 * handlers with the irqlock held. This way of measuring frame
 * intervals is subject to errors introduced by interrupt latency.
 */
static int fim_eof_handler(struct imxcam_dev *dev, struct timespec *ts)
{
	struct imxcam_fim *fim = &dev->fim;

	return frame_interval_monitor(fim, ts);
}

/*
 * Input Capture method of measuring frame intervals. Not subject
 * to interrupt latency.
 */
static void fim_input_capture_handler(int channel, void *dev_id,
				      struct timespec *ts)
{
	struct imxcam_fim *fim = dev_id;
	struct imxcam_dev *dev = fim2dev(fim);
	unsigned long flags;
	int ret;

	ret = frame_interval_monitor(fim, ts);
	if (ret) {
		spin_lock_irqsave(&dev->notify_lock, flags);
		if (!dev->stop && !atomic_read(&dev->pending_restart))
			imxcam_bump_restart_timer(dev);
		spin_unlock_irqrestore(&dev->notify_lock, flags);
	}

	if (!completion_done(&fim->icap_first_event))
		complete(&fim->icap_first_event);
}

static int fim_request_input_capture(struct imxcam_dev *dev)
{
	struct imxcam_fim *fim = &dev->fim;

	if (fim->icap_channel < 0)
		return 0;

	init_completion(&fim->icap_first_event);

	return mxc_request_input_capture(fim->icap_channel,
					 fim_input_capture_handler,
					 fim->icap_flags, fim);
}

static void fim_free_input_capture(struct imxcam_dev *dev)
{
	struct imxcam_fim *fim = &dev->fim;

	if (fim->icap_channel < 0)
		return;

	mxc_free_input_capture(fim->icap_channel, fim);
}

/*
 * In case we are monitoring the first frame interval after streamon
 * (when fim->num_skip = 0), we need a valid fim->last_ts before we
 * can begin. This only applies to the input capture method. It is not
 * possible to accurately measure the first FI after streamon using the
 * EOF method, so fim->num_skip minimum is set to 1 in that case, so this
 * function is a noop when the EOF method is used.
 */
static void fim_acquire_first_ts(struct imxcam_dev *dev)
{
	struct imxcam_fim *fim = &dev->fim;
	unsigned long ret;

	if (!fim->enabled || fim->num_skip > 0)
		return;

	ret = wait_for_completion_timeout(&fim->icap_first_event,
					  msecs_to_jiffies(IMXCAM_EOF_TIMEOUT));
	if (ret == 0)
		v4l2_warn(&dev->sd, "wait first icap event timeout\n");
}


/*
  AT streamon, subdev must call:
	parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	ret = v4l2_subdev_call(dev->sensor->sd, video, g_parm, &parm);
	if (ret)
		memset(&dev->sensor_tpf, 0, sizeof(dev->sensor_tpf));
	else
		dev->sensor_tpf = parm.parm.capture.timeperframe;

	update_fim(dev);
	reset_fim(dev, true);
	fim_acquire_first_ts(dev);
*/

static const struct v4l2_ctrl_config imxcam_fim_ctrl[] = {
	[FIM_CL_ENABLE] = {
		.ops = &imxcam_ctrl_ops,
		.id = V4L2_CID_IMX_FIM_ENABLE,
		.name = "FIM Enable",
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.def = FIM_CL_ENABLE_DEF,
		.min = 0,
		.max = 1,
		.step = 1,
	},
	[FIM_CL_NUM] = {
		.ops = &imxcam_ctrl_ops,
		.id = V4L2_CID_IMX_FIM_NUM,
		.name = "FIM Num Average",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.def = FIM_CL_NUM_DEF,
		.min =  1, /* no averaging */
		.max = 64, /* average 64 frames */
		.step = 1,
	},
	[FIM_CL_TOLERANCE_MIN] = {
		.ops = &imxcam_ctrl_ops,
		.id = V4L2_CID_IMX_FIM_TOLERANCE_MIN,
		.name = "FIM Tolerance Min",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.def = FIM_CL_TOLERANCE_MIN_DEF,
		.min =    2,
		.max =  200,
		.step =   1,
	},
	[FIM_CL_TOLERANCE_MAX] = {
		.ops = &imxcam_ctrl_ops,
		.id = V4L2_CID_IMX_FIM_TOLERANCE_MAX,
		.name = "FIM Tolerance Max",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.def = FIM_CL_TOLERANCE_MAX_DEF,
		.min =    0,
		.max =  500,
		.step =   1,
	},
	[FIM_CL_NUM_SKIP] = {
		.ops = &imxcam_ctrl_ops,
		.id = V4L2_CID_IMX_FIM_NUM_SKIP,
		.name = "FIM Num Skip",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.def = FIM_CL_NUM_SKIP_DEF,
		.min =   0, /* skip no frames */
		.max = 256, /* skip 256 frames */
		.step =  1,
	},
};


static int imxcam_of_parse_fim(struct imxcam_dev *dev,
			       struct device_node *np)
{
	struct imxcam_fim *fim = &dev->fim;
	struct device_node *fim_np;
	u32 val, tol[2], icap[2];
	int ret;

	fim_np = of_get_child_by_name(np, "fim");
	if (!fim_np) {
		/* set to the default defaults */
		fim->of_defaults[FIM_CL_ENABLE] = FIM_CL_ENABLE_DEF;
		fim->of_defaults[FIM_CL_NUM] = FIM_CL_NUM_DEF;
		fim->of_defaults[FIM_CL_NUM_SKIP] = FIM_CL_NUM_SKIP_DEF;
		fim->of_defaults[FIM_CL_TOLERANCE_MIN] =
			FIM_CL_TOLERANCE_MIN_DEF;
		fim->of_defaults[FIM_CL_TOLERANCE_MAX] =
			FIM_CL_TOLERANCE_MAX_DEF;
		fim->icap_channel = -1;
		return 0;
	}

	ret = of_property_read_u32(fim_np, "enable", &val);
	if (ret)
		val = FIM_CL_ENABLE_DEF;
	fim->of_defaults[FIM_CL_ENABLE] = val;

	ret = of_property_read_u32(fim_np, "num-avg", &val);
	if (ret)
		val = FIM_CL_NUM_DEF;
	fim->of_defaults[FIM_CL_NUM] = val;

	ret = of_property_read_u32(fim_np, "num-skip", &val);
	if (ret)
		val = FIM_CL_NUM_SKIP_DEF;
	fim->of_defaults[FIM_CL_NUM_SKIP] = val;

	ret = of_property_read_u32_array(fim_np, "tolerance-range", tol, 2);
	if (ret) {
		tol[0] = FIM_CL_TOLERANCE_MIN_DEF;
		tol[1] = FIM_CL_TOLERANCE_MAX_DEF;
	}
	fim->of_defaults[FIM_CL_TOLERANCE_MIN] = tol[0];
	fim->of_defaults[FIM_CL_TOLERANCE_MAX] = tol[1];

	ret = of_property_read_u32_array(fim_np, "input-capture-channel",
					 icap, 2);
	if (!ret) {
		fim->icap_channel = icap[0];
		fim->icap_flags = icap[1];
	} else {
		fim->icap_channel = -1;
	}

	of_node_put(fim_np);
	return 0;
}

/*
  At probe time, subdev must call:
  
  ret = imxcam_of_parse_fim(dev, node);
  if (ret)
    return ret;
		
    if (dev->fim.icap_channel < 0)
      dev->fim.eof = fim_eof_handler;

 */

