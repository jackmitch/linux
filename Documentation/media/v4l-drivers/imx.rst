i.MX Video Capture Driver
=========================

Introduction
------------

The Freescale i.MX5/6 contains an Image Processing Unit (IPU), which
handles the flow of image frames to and from capture devices and
display devices.

For image capture, the IPU contains the following subunits:

- Image DMA Controller (IDMAC)
- Camera Serial Interface (CSI)
- Image Converter (IC)
- Sensor Multi-FIFO Controller (SMFC)
- Image Rotator (IRT)
- Video De-Interlace Controller (VDIC)

The IDMAC is the DMA controller for transfer of image frames to and from
memory. Various dedicated DMA channels exist for both video capture and
display paths.

The CSI is the frontend capture unit that interfaces directly with
capture devices over Parallel, BT.656, and MIPI CSI-2 busses.

The IC handles color-space conversion, resizing, and rotation
operations.

The SMFC is used to send image frames directly to memory, bypassing the
IC. The SMFC is used when no color-space conversion or resizing is
required, i.e. the requested V4L2 formats and color-space are identical
to raw frames from the capture device.

The IRT carries out 90 and 270 degree image rotation operations.

Finally, the VDIC handles the conversion of interlaced video to
progressive, with support for different motion compensation modes (low
and high).

For more info, refer to the latest versions of the i.MX5/6 reference
manuals listed under References.


Features
--------

Some of the features of this driver include:

- Supports parallel, BT.565, and MIPI CSI-2 interfaces.

- Multiple subdev sensors can be registered and controlled by a single
  interface driver instance. Input enumeration will list every registered
  sensor's inputs and input names, and setting an input will switch to
  a different sensor if the input index is handled by a different sensor.

- Simultaneous streaming from two separate sensors is possible with two
  interface driver instances, each instance controlling a different
  sensor. This is currently possible with the SabreSD reference board
  with OV5642 and MIPI CSI-2 OV5640 sensors.

- Scaling, color-space conversion, and image rotation.

- Many pixel formats supported (RGB, packed and planar YUV, partial
  planar YUV).

- Full device-tree support using OF graph bindings.

- Analog decoder input video source hot-swap support (during streaming)
  via decoder status change subdev notification.

- MMAP, and DMABUF importer/exporter buffers supported.

- Motion compensated de-interlacing using the VDIC, with three
  motion compensation modes: low, medium, and high motion. The mode is
  specified with a custom control.

- Includes a Frame Interval Monitor (FIM) that can correct vertical sync
  problems with the ADV718x video decoders. See below for a description
  of the FIM.


Usage Notes
-----------

The i.MX capture driver is a standardized driver that supports the
following community V4L2 tools:

- v4l2-ctl
- v4l2-cap
- v4l2src gstreamer plugin


The following platforms have been tested:


SabreLite with parallel-interface OV5642
----------------------------------------

This platform requires the OmniVision OV5642 module with a parallel
camera interface from Boundary Devices for the SabreLite
(http://boundarydevices.com/products/nit6x_5mp/).

There is a pin conflict between OV5642 and ethernet devices on this
platform, so by default video capture is disabled in the device tree. To
enable video capture, edit arch/arm/boot/dts/imx6qdl-sabrelite.dtsi and
uncomment the macro __OV5642_CAPTURE__.


SabreAuto with ADV7180 decoder
------------------------------

This platform accepts Composite Video analog inputs on Ain1 (connector
J42) and Ain3 (connector J43).

To switch to Ain1:

.. code-block:: none

   # v4l2-ctl -i0

To switch to Ain3:

.. code-block:: none

   # v4l2-ctl -i1


Frame Interval Monitor
----------------------

The adv718x decoders can occasionally send corrupt fields during
NTSC/PAL signal re-sync (too little or too many video lines). When
this happens, the IPU triggers a mechanism to re-establish vertical
sync by adding 1 dummy line every frame, which causes a rolling effect
from image to image, and can last a long time before a stable image is
recovered. Or sometimes the mechanism doesn't work at all, causing a
permanent split image (one frame contains lines from two consecutive
captured images).

From experiment it was found that during image rolling, the frame
intervals (elapsed time between two EOF's) drop below the nominal
value for the current standard, by about one frame time (60 usec),
and remain at that value until rolling stops.

While the reason for this observation isn't known (the IPU dummy
line mechanism should show an increase in the intervals by 1 line
time every frame, not a fixed value), we can use it to detect the
corrupt fields using a frame interval monitor. If the FIM detects a
bad frame interval, the camera interface driver restarts IPU capture
which corrects the rolling/split image.

Custom controls exist to tweak some dials for FIM. If one of these
controls is changed during streaming, the FIM will be reset and will
continue at the new settings.

- V4L2_CID_IMX_FIM_ENABLE

Enable/disable the FIM.

- V4L2_CID_IMX_FIM_NUM

How many frame interval errors to average before comparing against the nominal
frame interval reported by the sensor. This can reduce noise from interrupt
latency.

- V4L2_CID_IMX_FIM_TOLERANCE_MIN

If the averaged intervals fall outside nominal by this amount, in
microseconds, streaming will be restarted.

- V4L2_CID_IMX_FIM_TOLERANCE_MAX

If any interval errors are higher than this value, those error samples
are discarded and do not enter into the average. This can be used to
discard really high interval errors that might be due to very high
system load, causing excessive interrupt latencies.

- V4L2_CID_IMX_FIM_NUM_SKIP

How many frames to skip after a FIM reset or stream restart before
FIM begins to average intervals. It has been found that there can
be a few bad frame intervals after stream restart which are not
attributed to adv718x sending a corrupt field, so this is used to
skip those frames to prevent unnecessary restarts.

Finally, all the defaults for these controls can be modified via a
device tree child node of the capture node, see
Documentation/devicetree/bindings/media/imx.txt.


SabreSD with MIPI CSI-2 OV5640
------------------------------

The default device tree for SabreSD includes endpoints for both the
parallel OV5642 and the MIPI CSI-2 OV5640, but as of this writing only
the MIPI CSI-2 OV5640 has been tested. The OV5640 module connects to
MIPI connector J5 (sorry I don't have the compatible module part number
or URL).

Inputs are registered for both the OV5642 and OV5640, and by default the
OV5642 is selected. To switch to the OV5640:

.. code-block:: none

   # v4l2-ctl -i1


Known Issues
------------

1. When using 90 or 270 degree rotation control at capture resolutions
   near the IC resizer limit of 1024x1024, and combined with planar
   pixel formats (YUV420, YUV422p), frame capture will often fail with
   no end-of-frame interrupts from the IDMAC channel. To work around
   this, use lower resolution and/or packed formats (YUYV, RGB3, etc.)
   when 90 or 270 rotations are needed.

2. Simple IDMAC interleaving using the ILO field in the IDMAC cpmem
   doesn't work when combined with the 16-bit planar pixel formats
   (YUV422P and NV16). This looks like a silicon bug, and there is
   no satisfactory replies to queries about it from Freescale. So
   the driver works around the issue by forcing the format to the
   12-bit planar versions (YUV420 and NV12) when simple interleaving
   is used and the sensor sends interlaced fields (ADV718x). Another
   option to workaround the issue is to use motion compensation when
   combined with YUV422P or NV16.

File list
---------

drivers/staging/media/imx/
include/media/imx.h
include/uapi/media/imx.h

References
----------

[1] "i.MX 6Dual/6Quad Applications Processor Reference Manual"
[2] "i.MX 6Solo/6DualLite Applications Processor Reference Manual"


Author
------
Steve Longerbeam <steve_longerbeam@mentor.com>

Copyright (C) 2012-2016 Mentor Graphics Inc.
