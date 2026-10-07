"""Sensor-format helpers shared by simulated and real RGB-D inputs.

The Gazebo camera publishes floating-point depth in metres, while the common
RealSense ROS1 configuration publishes unsigned 16-bit depth in millimetres.
Keeping the conversion here makes that hardware boundary explicit and testable
without ROS, Gazebo, or a camera connected.
"""

import numpy as np


def depth_image_to_meters(depth_image, encoding, uint16_scale=0.001):
    """Return a float32 depth image expressed in metres.

    Args:
        depth_image: NumPy image produced by CvBridge with ``passthrough``.
        encoding: ROS Image encoding, normally ``32FC1`` or ``16UC1``.
        uint16_scale: Metres represented by one integer unit. RealSense depth
            uses millimetres by default, hence 0.001.

    Raises:
        ValueError: if the encoding is unsupported or the scale is invalid.
    """

    if uint16_scale <= 0.0:
        raise ValueError("uint16 depth scale must be positive")

    normalized = str(encoding).strip().upper()
    image = np.asarray(depth_image)

    if normalized in ("16UC1", "MONO16"):
        return image.astype(np.float32) * float(uint16_scale)
    if normalized == "32FC1":
        return image.astype(np.float32, copy=False)
    if normalized == "64FC1":
        return image.astype(np.float32)

    raise ValueError(
        "unsupported depth encoding {!r}; expected 16UC1, mono16, 32FC1 or 64FC1".format(
            encoding
        )
    )
