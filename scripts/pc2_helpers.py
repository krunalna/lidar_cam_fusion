"""PointCloud2 helpers shared by verification scripts."""

from __future__ import annotations

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


def _pc2_to_numpy(msg: PointCloud2) -> np.ndarray:
    """Convert sensor_msgs/PointCloud2 to an (N, 4) float32 array."""
    if msg.point_step == 0:
        raise ValueError("PointCloud2 has point_step == 0")
    if msg.point_step % 4 != 0:
        raise ValueError(f"point_step {msg.point_step} is not a multiple of 4 bytes")

    field_offsets = {field.name: field.offset for field in msg.fields}
    required = {"x", "y", "z"}
    missing = required - set(field_offsets)
    if missing:
        raise ValueError(f"PointCloud2 is missing required fields: {missing}")

    n_points = msg.width * msg.height
    stride = msg.point_step // 4
    raw = np.frombuffer(bytes(msg.data), dtype=np.float32).reshape(n_points, stride)

    result = np.zeros((n_points, 4), dtype=np.float32)
    for col, name in enumerate(("x", "y", "z")):
        result[:, col] = raw[:, field_offsets[name] // 4]
    if "intensity" in field_offsets:
        result[:, 3] = raw[:, field_offsets["intensity"] // 4]
    return result


def _numpy_to_pc2(points: np.ndarray, header: Header) -> PointCloud2:
    """Convert an (N, 3) or (N, 4) float32 array to sensor_msgs/PointCloud2."""
    if points.ndim == 1:
        points = points.reshape(-1, 3)

    if points.shape[1] == 3:
        intensity = np.zeros((points.shape[0], 1), dtype=np.float32)
        points = np.hstack([points, intensity])

    points = points.astype(np.float32)
    itemsize = np.dtype(np.float32).itemsize

    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = points.shape[0]
    msg.is_bigendian = False
    msg.is_dense = True
    msg.fields = [
        PointField(name="x",         offset=0 * itemsize, datatype=PointField.FLOAT32, count=1),
        PointField(name="y",         offset=1 * itemsize, datatype=PointField.FLOAT32, count=1),
        PointField(name="z",         offset=2 * itemsize, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=3 * itemsize, datatype=PointField.FLOAT32, count=1),
    ]
    msg.point_step = 4 * itemsize
    msg.row_step = msg.point_step * msg.width
    msg.data = points.tobytes()
    return msg
