"""Hailo person detection ROS 2 node.

Subscribes to combined StereoImage messages containing a rectified colour image
and depth map from ros2_stereo. Runs YOLOv5 person detection on the colour image
via the Hailo-8 accelerator.

Publishes two annotated output images (JPEG compressed):
    - Depth map with person bounding boxes and depth labels
    - Rectified RGB with person bounding boxes and depth labels

Also publishes pipeline latency metrics (DiagnosticArray) for tracing.
"""

from __future__ import annotations

import os
import time
from typing import Any
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import CompressedImage
from stereo_depth_node.msg import StereoImage

from hailo_detection_node.config import NodeConfig
from hailo_detection_node.hb_hailo_inference import HailoYolo


# Text label drawn above each bounding box
_DETECTION_LABEL = "PERSON"

# Pixel offsets for the annotation label relative to the box top-left corner
_LABEL_H_OFFSET = 4
_LABEL_V_OFFSET = 20


class HailoDetectionNode(Node):
    """ROS 2 node that detects persons using a combined stereo image message."""

    def __init__(self) -> None:
        """Initialize the Hailo detection node."""
        super().__init__("hailo_detection_node")

        # Declare ROS parameters
        self.declare_parameter("stereo_image_topic", "/stereo/image")
        self.declare_parameter("output_depth_topic", "/hailo/depth_annotated")
        self.declare_parameter("output_rgb_topic", "/hailo/rgb_annotated")
        self.declare_parameter("latency_topic", "/hailo/pipeline_latency")
        self.declare_parameter("config_file", "/etc/node_config.json")
        self.declare_parameter("jpeg_quality", 80)

        # Retrieve parameter values
        stereo_image_topic: str = self.get_parameter("stereo_image_topic").value
        output_depth_topic: str = self.get_parameter("output_depth_topic").value
        output_rgb_topic: str = self.get_parameter("output_rgb_topic").value
        latency_topic: str = self.get_parameter("latency_topic").value
        config_file: str = self.get_parameter("config_file").value
        self.jpeg_quality: int = self.get_parameter("jpeg_quality").value

        # Load node configuration
        self.config = NodeConfig.load_from_file(Path(config_file))
        if self.config.max_depth_m <= self.config.min_depth_m:
            self.get_logger().warning(
                "Invalid depth range in config "
                f"(min_depth_m={self.config.min_depth_m}, "
                f"max_depth_m={self.config.max_depth_m}). "
                "Falling back to min_depth_m + 1.0m for normalization."
            )
            self.config.max_depth_m = self.config.min_depth_m + 1.0

        # OpenCV <-> ROS image bridge
        self.bridge = CvBridge()

        # Hailo YOLO inferencer
        self.inferer = HailoYolo(
            model_location=Path(self.config.model_path),
            model_name=self.config.model_name,
            nms_output_node=self.config.nms_output_node,
        )
        self.detection_class_index = self.config.detection_class_index
        self.nms_output_key = self.config.nms_output_key
        self.logged_missing_nms_key = False
        self.logged_class_mismatch = False
        self.stereo_image_topic = stereo_image_topic
        self.frame_count = 0
        self.last_frame_time_s: float | None = None
        self.logged_input_layout = False

        # Use sensor-data QoS (best-effort) to match the stereo image publisher.
        input_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        output_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        # Subscribe directly to the combined stereo image message
        self.stereo_sub = self.create_subscription(
            StereoImage,
            stereo_image_topic,
            self._stereo_callback,
            input_qos,
        )

        # Publishers (compressed for efficient cross-host transport)
        self.depth_publisher = self.create_publisher(
            CompressedImage, output_depth_topic + "/compressed", output_qos
        )
        self.rgb_publisher = self.create_publisher(
            CompressedImage, output_rgb_topic + "/compressed", output_qos
        )
        self.latency_publisher = self.create_publisher(
            DiagnosticArray, latency_topic, output_qos
        )

        # Periodic diagnostics so silent stalls are visible in logs.
        self.health_timer = self.create_timer(5.0, self._health_callback)

        self.get_logger().info(
            f"Hailo detection node initialized. "
            f"Stereo input: {stereo_image_topic} | "
            f"Depth out: {output_depth_topic} | "
            f"RGB out: {output_rgb_topic} | "
            f"Latency out: {latency_topic}"
        )
        self.get_logger().info(
            "ROS env: "
            f"RMW_IMPLEMENTATION={os.getenv('RMW_IMPLEMENTATION', '<unset>')} | "
            f"ROS_DOMAIN_ID={os.getenv('ROS_DOMAIN_ID', '<unset>')} | "
            f"ROS_LOCALHOST_ONLY={os.getenv('ROS_LOCALHOST_ONLY', '<unset>')}"
        )

    def _stereo_callback(self, stereo_msg: StereoImage) -> None:
        """Process combined stereo image: detect persons and publish.

        Args:
            stereo_msg: Combined message with left colour image (bgr8) and
                        depth map (mono16) from the same source frame.
        """
        try:
            t_start = time.perf_counter()
            receive_latency_ms = self._header_receive_latency_ms(stereo_msg)
            self.frame_count += 1
            self.last_frame_time_s = time.time()

            left_msg = stereo_msg.left_image
            depth_msg = stereo_msg.depth_image

            # Decode messages
            color_frame = self.bridge.imgmsg_to_cv2(left_msg, desired_encoding="bgr8")
            depth_mm = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="mono16")

            if not self.logged_input_layout:
                self.get_logger().info(
                    f"Left frame: {left_msg.encoding} "
                    f"{left_msg.width}x{left_msg.height} | "
                    f"Depth frame: {depth_msg.encoding} "
                    f"{depth_msg.width}x{depth_msg.height}"
                )
                self.logged_input_layout = True

            t_decode = time.perf_counter()

            # Run Hailo YOLOv5 inference on the colour frame
            self.inferer.infer(color_frame)
            t_infer = time.perf_counter()

            # Collect person bounding boxes (normalised -> pixel coordinates)
            bounding_boxes: list[dict[str, int]] = []
            if self.inferer.latest_result is not None:
                nms_result = self.inferer.latest_result.get(self.nms_output_key)
                if nms_result is None:
                    nms_result = self._resolve_fallback_nms_result(
                        self.inferer.latest_result
                    )
                    if not self.logged_missing_nms_key:
                        self.get_logger().warning(
                            f"Missing configured NMS output key '{self.nms_output_key}' "
                            "in inference result; skipping detections"
                        )
                        self.logged_missing_nms_key = True
                if nms_result is None:
                    person_dets = np.empty((0, 5), dtype=np.float32)
                elif self.detection_class_index >= len(nms_result[0]):
                    self.get_logger().warning(
                        "Configured detection_class_index "
                        f"{self.detection_class_index} is out of range for NMS output "
                        f"(classes={len(nms_result[0])}); skipping detections"
                    )
                    person_dets = np.empty((0, 5), dtype=np.float32)
                else:
                    person_dets = nms_result[0][self.detection_class_index]

                if (
                    person_dets.size == 0
                    and nms_result is not None
                    and not self.logged_class_mismatch
                ):
                    total_dets = sum(
                        int(detections_by_class.shape[0])
                        for detections_by_class in nms_result[0]
                    )
                    if total_dets > 0:
                        self.get_logger().warning(
                            "No detections in configured class index "
                            f"{self.detection_class_index}, but found {total_dets} total "
                            "detections across all classes. Check detection_class_index."
                        )
                        self.logged_class_mismatch = True
                if person_dets.size != 0:
                    height, width = color_frame.shape[:2]
                    for det in person_dets:
                        ymin_n, xmin_n, ymax_n, xmax_n, _score = det
                        box = self._clamp_box_to_image(
                            {
                                "top": int(ymin_n * height),
                                "bottom": int(ymax_n * height),
                                "left": int(xmin_n * width),
                                "right": int(xmax_n * width),
                            },
                            height,
                            width,
                        )
                        if box is not None:
                            bounding_boxes.append(box)

            # Colorize depth for annotated output
            depth_colorized = self._colorize_depth(depth_mm)

            color_annotated = color_frame.copy()
            depth_annotated = depth_colorized.copy()

            for box in bounding_boxes:
                depth_m = self._median_depth_from_mono16(depth_mm, box)
                self._draw_detection(color_annotated, box, depth_m)
                self._draw_detection(depth_annotated, box, depth_m)
            t_annotate = time.perf_counter()

            # Publish compressed - preserve the original header (timestamp + frame_id)
            encode_params = [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
            depth_ok, depth_buf = cv2.imencode(".jpg", depth_annotated, encode_params)
            rgb_ok, rgb_buf = cv2.imencode(".jpg", color_annotated, encode_params)

            if not depth_ok or not rgb_ok:
                self.get_logger().error(
                    "JPEG encoding failed "
                    f"(depth_ok={depth_ok}, rgb_ok={rgb_ok}); skipping publish"
                )
                return

            depth_out = CompressedImage()
            depth_out.header = stereo_msg.header
            depth_out.format = "jpeg"
            depth_out.data = depth_buf.tobytes()

            rgb_out = CompressedImage()
            rgb_out.header = stereo_msg.header
            rgb_out.format = "jpeg"
            rgb_out.data = rgb_buf.tobytes()

            self.depth_publisher.publish(depth_out)
            self.rgb_publisher.publish(rgb_out)
            t_publish = time.perf_counter()

            self._publish_latency_metrics(
                input_header=stereo_msg.header,
                receive_latency_ms=receive_latency_ms,
                decode_ms=(t_decode - t_start) * 1000.0,
                infer_ms=(t_infer - t_decode) * 1000.0,
                annotate_ms=(t_annotate - t_infer) * 1000.0,
                publish_ms=(t_publish - t_annotate) * 1000.0,
                total_ms=(t_publish - t_start) * 1000.0,
                detection_count=len(bounding_boxes),
            )

            if self.frame_count % 30 == 0:
                self.get_logger().info(
                    f"Processed {self.frame_count} frames | "
                    f"rx={receive_latency_ms:.1f}ms "
                    f"decode={(t_decode - t_start) * 1000.0:.1f}ms "
                    f"infer={(t_infer - t_decode) * 1000.0:.1f}ms "
                    f"annotate={(t_annotate - t_infer) * 1000.0:.1f}ms "
                    f"publish={(t_publish - t_annotate) * 1000.0:.1f}ms "
                    f"total={(t_publish - t_start) * 1000.0:.1f}ms"
                )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Frame processing failed: {exc}")

    def _resolve_fallback_nms_result(self, latest_result: dict[str, Any]) -> Any | None:
        """Return a fallback NMS result when configured key is absent.

        Some exported models use different output node names. If exactly one
        NMS-like output key exists, use it as a runtime fallback.
        """
        nms_like_keys = [
            key for key in latest_result if "nms" in key and "postprocess" in key
        ]
        if len(nms_like_keys) == 1:
            fallback_key = nms_like_keys[0]
            self.get_logger().warning(
                f"Using fallback NMS output key '{fallback_key}' "
                f"instead of configured '{self.nms_output_key}'"
            )
            self.nms_output_key = fallback_key
            return latest_result[fallback_key]
        return None

    def _publish_latency_metrics(
        self,
        input_header,
        receive_latency_ms: float,
        decode_ms: float,
        infer_ms: float,
        annotate_ms: float,
        publish_ms: float,
        total_ms: float,
        detection_count: int,
    ) -> None:
        """Publish per-frame latency metrics in DiagnosticArray format."""
        diag = DiagnosticArray()
        diag.header = input_header

        status = DiagnosticStatus()
        status.name = "hailo_detection_pipeline_latency"
        status.level = DiagnosticStatus.OK
        status.message = "Pipeline timing breakdown (ms)"

        metrics = {
            "receive_latency_ms": receive_latency_ms,
            "decode_ms": decode_ms,
            "infer_ms": infer_ms,
            "annotate_ms": annotate_ms,
            "publish_ms": publish_ms,
            "total_processing_ms": total_ms,
            "frame_number": float(self.frame_count),
            "detection_count": float(detection_count),
        }

        for key, val in metrics.items():
            kv = KeyValue()
            kv.key = key
            kv.value = f"{val:.6f}"
            status.values.append(kv)

        diag.status.append(status)
        self.latency_publisher.publish(diag)

    def _health_callback(self) -> None:
        """Emit periodic health status for easier runtime debugging."""
        stereo_pubs = self.count_publishers(self.stereo_image_topic)

        if self.last_frame_time_s is None:
            self.get_logger().warning(
                f"No frames received yet (stereo publishers: {stereo_pubs})"
            )
            return

        seconds_since_last = time.time() - self.last_frame_time_s
        if seconds_since_last > 5.0:
            self.get_logger().warning(
                f"No frame for {seconds_since_last:.1f}s "
                f"(stereo publishers: {stereo_pubs})"
            )

    def _colorize_depth(self, depth_mm: np.ndarray) -> np.ndarray:
        """Convert mono16 depth (mm) to JET-colorized BGR image.

        Uses the same normalization as stereo_depth_node so the visual output
        matches the ``/depth/image/colorized`` topic.

        Args:
            depth_mm: Depth image in millimetres (uint16).

        Returns:
            BGR image with JET colormap applied.
        """
        depth_m = depth_mm.astype(np.float32) / 1000.0
        min_d = self.config.min_depth_m
        max_d = self.config.max_depth_m

        # Defensive guard for misconfigured ranges.
        if max_d <= min_d:
            max_d = min_d + 1.0

        depth_clipped = np.clip(depth_m, min_d, max_d)
        depth_norm = ((depth_clipped - min_d) * 255.0 / (max_d - min_d)).astype(
            np.uint8
        )

        invalid = depth_mm == 0
        depth_norm[invalid] = 0

        colorized = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)
        colorized[invalid] = 0
        return colorized

    def _clamp_box_to_image(
        self, box: dict[str, int], image_h: int, image_w: int
    ) -> dict[str, int] | None:
        """Clamp a detection box to image bounds and reject degenerate boxes.

        Coordinates are clamped for NumPy slicing semantics where bottom/right
        are exclusive bounds.
        """
        top = max(0, min(box["top"], image_h))
        bottom = max(0, min(box["bottom"], image_h))
        left = max(0, min(box["left"], image_w))
        right = max(0, min(box["right"], image_w))

        if bottom <= top or right <= left:
            return None

        return {
            "top": top,
            "bottom": bottom,
            "left": left,
            "right": right,
        }

    def _median_depth_from_mono16(
        self, depth_mm: np.ndarray, box: dict[str, int]
    ) -> float | None:
        """Compute median person depth (metres) from a mono16 depth map.

        Reads the raw uint16 depth values (millimetres) from the centre
        region of the bounding box and returns the median in metres.

        Args:
            depth_mm: Depth image in millimetres (uint16).
            box: Bounding box dict with keys 'top', 'bottom', 'left', 'right'.

        Returns:
            Median depth in metres, or None if no valid pixels.
        """
        r = self.config.person_center_ratio
        h = box["bottom"] - box["top"]
        w = box["right"] - box["left"]
        margin_y = int(h * (1 - r) / 2)
        margin_x = int(w * (1 - r) / 2)

        inner_top = box["top"] + margin_y
        inner_bottom = box["bottom"] - margin_y
        inner_left = box["left"] + margin_x
        inner_right = box["right"] - margin_x

        if inner_bottom <= inner_top or inner_right <= inner_left:
            return None

        crop = depth_mm[inner_top:inner_bottom, inner_left:inner_right]
        min_mm = int(self.config.min_depth_m * 1000.0)
        max_mm = int(self.config.max_depth_m * 1000.0)
        valid = crop[(crop >= min_mm) & (crop <= max_mm)]
        if valid.size == 0:
            return None

        return float(np.median(valid)) / 1000.0

    def _header_receive_latency_ms(self, msg: StereoImage) -> float:
        """Compute frame receive latency from message header timestamp."""
        stamp = msg.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            return -1.0

        now_ns = self.get_clock().now().nanoseconds
        stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        return max(0.0, (now_ns - stamp_ns) / 1_000_000.0)

    def _draw_detection(
        self, image: np.ndarray, box: dict[str, int], depth_m: float | None
    ) -> None:
        """Overlay a bounding box and depth label on *image* in-place.

        Args:
            image:   BGR image to annotate (modified in-place).
            box:     Dict with integer keys 'top', 'bottom', 'left', 'right'.
            depth_m: Median person depth in metres, or None if unavailable.
        """
        color = tuple(self.config.box_color_bgr)
        cv2.rectangle(
            image,
            (box["left"], box["top"]),
            (box["right"], box["bottom"]),
            color,
            self.config.box_thickness,
        )
        label = (
            f"{_DETECTION_LABEL}  {depth_m:.2f}m"
            if depth_m is not None
            else _DETECTION_LABEL
        )
        cv2.putText(
            image,
            label,
            (box["left"] + _LABEL_H_OFFSET, box["top"] - _LABEL_V_OFFSET),
            cv2.FONT_HERSHEY_SIMPLEX,
            self.config.text_scale,
            color,
            self.config.text_thickness,
            cv2.LINE_AA,
        )


def main(args: "list[str] | None" = None) -> None:
    """Main entry point for the ROS node.

    Args:
        args: Command line arguments.
    """
    rclpy.init(args=args)
    node = HailoDetectionNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
