"""Tests for hailo_detection_node configuration."""

import json
import tempfile
from pathlib import Path

from hailo_detection_node.config import NodeConfig


def test_node_config_defaults() -> None:
    """Verify NodeConfig defaults."""
    config = NodeConfig()
    assert config.model_path == "/networks"
    assert config.model_name == "yolox_s_leaky"
    assert config.nms_output_node == "yolox_nms_postprocess"
    assert config.detection_class_index == 0
    assert config.nms_output_key == "yolox_s_leaky/yolox_nms_postprocess"
    assert config.person_center_ratio == 0.3
    assert config.min_depth_m == 0.3
    assert config.max_depth_m == 10.0
    assert config.box_color_bgr == [255, 255, 255]
    assert config.box_thickness == 4
    assert config.text_scale == 1.0
    assert config.text_thickness == 2


def test_node_config_load_from_missing_file() -> None:
    """Verify NodeConfig loads defaults when file is missing."""
    config = NodeConfig.load_from_file(Path("/nonexistent/path.json"))
    assert config.model_path == "/networks"
    assert config.model_name == "yolox_s_leaky"
    assert config.nms_output_node == "yolox_nms_postprocess"
    assert config.detection_class_index == 0
    assert config.nms_output_key == "yolox_s_leaky/yolox_nms_postprocess"


def test_node_config_load_from_file() -> None:
    """Verify NodeConfig loads correctly from a JSON file."""
    with tempfile.NamedTemporaryFile(mode="w", suffix=".json", delete=False) as f:
        config_data = {
            "model_path": "/custom/networks",
            "model_name": "yolox_s_leaky",
            "nms_output_node": "custom_nms_postprocess",
            "detection_class_index": 32,
            "person_center_ratio": 0.5,
            "min_depth_m": 0.5,
            "max_depth_m": 20.0,
            "box_color_bgr": [0, 255, 0],
            "box_thickness": 2,
            "text_scale": 0.8,
            "text_thickness": 1,
        }
        json.dump(config_data, f)
        temp_path = Path(f.name)

    try:
        config = NodeConfig.load_from_file(temp_path)
        assert config.model_path == "/custom/networks"
        assert config.model_name == "yolox_s_leaky"
        assert config.nms_output_node == "custom_nms_postprocess"
        assert config.detection_class_index == 32
        assert config.nms_output_key == "yolox_s_leaky/custom_nms_postprocess"
        assert config.person_center_ratio == 0.5
        assert config.min_depth_m == 0.5
        assert config.max_depth_m == 20.0
        assert config.box_color_bgr == [0, 255, 0]
        assert config.box_thickness == 2
        assert config.text_scale == 0.8
        assert config.text_thickness == 1
    finally:
        temp_path.unlink()


def test_node_config_load_partial_file() -> None:
    """Verify NodeConfig falls back to defaults for a malformed file."""
    with tempfile.NamedTemporaryFile(mode="w", suffix=".json", delete=False) as f:
        f.write("{invalid json")
        temp_path = Path(f.name)

    try:
        config = NodeConfig.load_from_file(temp_path)
        assert config.model_path == "/networks"
    finally:
        temp_path.unlink()
