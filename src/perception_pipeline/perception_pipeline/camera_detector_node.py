"""
Camera Detector Node
====================
Subscribes to raw camera images, runs YOLOv8 pre-trained inference, and
publishes 2D bounding-box detections for the downstream fusion node.

Subscribed topics:
  /camera/image_raw        sensor_msgs/Image (rgb8)   raw camera frame

Published topics:
  /detections_2d           vision_msgs/Detection2DArray   per-frame detections

Parameters:
  model_name      (str)    YOLOv8 model weights file or name (default: 'yolov8n.pt')
  conf_threshold  (float)  Minimum confidence to keep a detection (default: 0.5)
  device          (str)    Inference device: 'cpu', 'cuda', 'mps' (default: 'cpu')
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from vision_msgs.msg import (
    Detection2DArray,
    Detection2D,
    ObjectHypothesisWithPose,
    ObjectHypothesis,
    BoundingBox2D,
)


# ── Image conversion helper ───────────────────────────────────────────────────

def _imgmsg_to_numpy(msg: Image) -> np.ndarray:
    """Convert sensor_msgs/Image (rgb8) → (H, W, 3) uint8 numpy array."""
    img = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    return img.reshape((msg.height, msg.width, -1))


# ── Core detector (no ROS dependency) ────────────────────────────────────────

class CameraDetector:
    """Pure-Python YOLOv8 detector — no ROS dependency, fully unit-testable."""

    def __init__(
        self,
        model_name: str = "yolov8n.pt",
        conf_threshold: float = 0.5,
        device: str = "cpu",
    ):
        from ultralytics import YOLO  # deferred to avoid slow import at module level
        self.conf_threshold = conf_threshold
        self.device = device
        self._model = YOLO(model_name)

    def detect(self, image_rgb: np.ndarray) -> list[dict]:
        """
        Run YOLOv8 inference on an RGB image.

        Args:
            image_rgb: (H, W, 3) uint8 numpy array in RGB order.

        Returns:
            List of detection dicts, each with keys:
              x1, y1, x2, y2  (float) — pixel coords of the bounding box corners
              class_id         (int)   — COCO class index
              class_name       (str)   — human-readable class label
              confidence       (float) — detection confidence in [0, 1]
        """
        results = self._model(
            image_rgb,
            conf=self.conf_threshold,
            device=self.device,
            verbose=False,
        )
        detections = []
        for r in results:
            for box in r.boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().tolist()
                cls_idx = int(box.cls[0])
                detections.append({
                    "x1": float(x1),
                    "y1": float(y1),
                    "x2": float(x2),
                    "y2": float(y2),
                    "class_id": cls_idx,
                    "class_name": r.names[cls_idx],
                    "confidence": float(box.conf[0]),
                })
        return detections


# ── ROS 2 Node ────────────────────────────────────────────────────────────────

class CameraDetectorNode(Node):
    def __init__(self):
        super().__init__("camera_detector")

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter("model_name",     "yolov8n.pt")
        self.declare_parameter("conf_threshold",  0.5)
        self.declare_parameter("device",         "cpu")

        model_name     = self.get_parameter("model_name").value
        conf_threshold = self.get_parameter("conf_threshold").value
        device         = self.get_parameter("device").value

        self.get_logger().info(
            f"Loading YOLOv8 model '{model_name}' on device='{device}' "
            f"(conf≥{conf_threshold})"
        )
        self._detector = CameraDetector(
            model_name=model_name,
            conf_threshold=conf_threshold,
            device=device,
        )
        self.get_logger().info("YOLOv8 model loaded.")

        # ── QoS ───────────────────────────────────────────────────────────────
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # ── Sub / Pub ─────────────────────────────────────────────────────────
        self._sub = self.create_subscription(
            Image, "/camera/image_raw", self._on_image, sub_qos
        )
        self._pub = self.create_publisher(Detection2DArray, "/detections_2d", pub_qos)

        viz_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._viz_pub = self.create_publisher(Image, "/camera/detections_viz", viz_qos)

        self._frame_count = 0
        self.get_logger().info("CameraDetector ready — waiting for /camera/image_raw")

    def _draw_detections(
        self, image_rgb: np.ndarray, detections: list[dict], header
    ) -> Image:
        """Draw bounding boxes + labels onto the image and return a sensor_msgs/Image."""
        canvas = image_rgb.copy()
        for d in detections:
            x1, y1, x2, y2 = int(d["x1"]), int(d["y1"]), int(d["x2"]), int(d["y2"])
            label = f"{d['class_name']} {d['confidence']:.2f}"

            cv2.rectangle(canvas, (x1, y1), (x2, y2), (0, 255, 0), 2)

            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            tag_y1 = max(y1 - th - 4, 0)
            cv2.rectangle(canvas, (x1, tag_y1), (x1 + tw + 4, y1), (0, 255, 0), -1)
            cv2.putText(
                canvas, label, (x1 + 2, y1 - 3),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA,
            )

        msg = Image()
        msg.header = header
        msg.height, msg.width = canvas.shape[:2]
        msg.encoding = "rgb8"
        msg.is_bigendian = False
        msg.step = msg.width * 3
        msg.data = canvas.tobytes()
        return msg

    def _on_image(self, msg: Image) -> None:
        try:
            image_rgb = _imgmsg_to_numpy(msg)
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        try:
            detections = self._detector.detect(image_rgb)
        except Exception as e:
            self.get_logger().error(f"Inference failed on frame {self._frame_count}: {e}")
            return

        # Build Detection2DArray
        out_msg = Detection2DArray()
        out_msg.header = Header()
        out_msg.header.stamp = msg.header.stamp
        out_msg.header.frame_id = msg.header.frame_id

        for d in detections:
            det = Detection2D()
            det.header = out_msg.header

            # Bounding box: center + size
            cx = (d["x1"] + d["x2"]) / 2.0
            cy = (d["y1"] + d["y2"]) / 2.0
            w  = d["x2"] - d["x1"]
            h  = d["y2"] - d["y1"]

            bbox = BoundingBox2D()
            bbox.center.position.x = cx
            bbox.center.position.y = cy
            bbox.center.theta = 0.0
            bbox.size_x = w
            bbox.size_y = h
            det.bbox = bbox

            # Class hypothesis
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis = ObjectHypothesis(
                class_id=d["class_name"],
                score=d["confidence"],
            )
            det.results = [hyp]

            out_msg.detections.append(det)

        self._pub.publish(out_msg)

        # ── Visualization (only when Foxglove/RViz is subscribed) ─────────────
        if self._viz_pub.get_subscription_count() > 0:
            viz_msg = self._draw_detections(image_rgb, detections, msg.header)
            self._viz_pub.publish(viz_msg)

        self._frame_count += 1
        if self._frame_count % 20 == 0:
            self.get_logger().info(
                f"Frame {self._frame_count} | {len(detections)} detection(s)"
            )


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    try:
        node = CameraDetectorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
