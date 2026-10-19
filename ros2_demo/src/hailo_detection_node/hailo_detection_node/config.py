"""Node configuration loader for hailo_detection_node."""

from __future__ import annotations

import json
import logging
from dataclasses import dataclass, field
from pathlib import Path

LOGGER = logging.getLogger("hailo-detection-node")


@dataclass
class NodeConfig:
    """Configuration for the Hailo detection node."""

    # Path to the directory containing the Hailo HEF model file(s)
    model_path: str = "/networks"
    # Default to yolox_s_leaky, but can be overridden by config file. Should match the subdirectory and HEF filename in model_path.
    model_name: str = "yolox_s_leaky"
    # Name of the NMS output node produced by the network.
    nms_output_node: str = "yolox_nms_postprocess"
    # Class index to annotate from the NMS output.
    detection_class_index: int = 0

    # Fraction of the bounding-box interior used when computing the median
    # depth. Uses the central portion of the box in each dimension to reduce
    # contamination from foreground/background edges. 0.3 = central 30%.
    person_center_ratio: float = 0.3

    # Depth range for filtering. Depth measurements outside this range are
    # excluded from statistics.
    min_depth_m: float = 0.3
    max_depth_m: float = 10.0

    # Bounding box appearance (BGR order)
    box_color_bgr: list[int] = field(default_factory=lambda: [255, 255, 255])
    box_thickness: int = 4

    # Detection label text appearance
    text_scale: float = 1.0
    text_thickness: int = 2

    @property
    def nms_output_key(self) -> str:
        """Return the fully-qualified output key for NMS detections."""
        return f"{self.model_name}/{self.nms_output_node}"

    @staticmethod
    def load_from_file(config_path: Path) -> NodeConfig:
        """Load configuration from a JSON file, using defaults for missing keys.

        Args:
            config_path: Path to the JSON config file.

        Returns:
            NodeConfig populated from the file, or defaults if the file is
            absent or malformed.
        """
        if not config_path.is_file():
            LOGGER.info("Config file not found at %s, using defaults", config_path)
            return NodeConfig()

        try:
            with config_path.open() as f:
                config_dict = json.load(f)
            LOGGER.info("Loaded config from %s", config_path)
            return NodeConfig(**config_dict)
        except (json.JSONDecodeError, TypeError) as e:
            LOGGER.warning(
                "Error loading config from %s: %s. Using defaults.",
                config_path,
                e,
            )
            return NodeConfig()
