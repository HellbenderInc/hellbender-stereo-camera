"""Hailo YOLOv5 inference wrapper."""

from pathlib import Path

import cv2
import hailo_platform as hp  # type: ignore  # installed at Docker build time
import numpy as np


class HailoYolo:
    """Class to handle Hailo YOLO inference. Confirmed to work with yolox_s_leaky and yolov5m"""

    def __init__(
        self,
        model_location: Path = Path("/networks"),
        model_name: str = "yolox_s_leaky",
        nms_output_node: str = "yolox_nms_postprocess",
    ) -> None:
        """Initialize the HailoYolo class.

        Args:
            model_location: Path to the directory containing the HEF model file.
            model_name: Name of the model subdirectory and HEF file (without extension).
            nms_output_node: Output node name for NMS post-processing detections.
        """
        self.latest_result = None

        self.target = hp.VDevice()
        self.model_name = model_name
        self.nms_output_key = f"{self.model_name}/{nms_output_node}"
        hef_path = model_location / self.model_name / f"{self.model_name}.hef"

        hef = hp.HEF(str(hef_path))

        configure_params = hp.ConfigureParams.create_from_hef(
            hef=hef, interface=hp.HailoStreamInterface.PCIe
        )
        network_groups = self.target.configure(hef, configure_params)
        self.network_group = network_groups[0]
        self.network_group_params = self.network_group.create_params()

        self.input_vstreams_params = hp.InputVStreamParams.make(
            self.network_group, format_type=hp.FormatType.UINT8
        )
        self.output_vstreams_params = hp.OutputVStreamParams.make(
            self.network_group, format_type=hp.FormatType.FLOAT32
        )

        self.input_vstream_info = hef.get_input_vstream_infos()[0]
        self.output_vstream_info = hef.get_output_vstream_infos()[0]
        self.image_height, self.image_width, self.channels = (
            self.input_vstream_info.shape
        )

        self.input_image = np.zeros(
            (1, self.image_height, self.image_width, self.channels), dtype=np.uint8
        )

    def preprocess_image(self, image: np.ndarray) -> None:
        """Resize and pad the image to fit the model input shape."""
        height, width, _ = image.shape

        if height == width:
            resized = cv2.resize(image, (self.image_width, self.image_height))
            self.width_ratio = 1
            self.height_ratio = 1
        elif height / self.image_height > width / self.image_width:
            resized = cv2.resize(
                image, (int(width * self.image_height / height), self.image_height)
            )
            self.width_ratio = width / height
            self.height_ratio = 1
        else:
            resized = cv2.resize(
                image, (self.image_width, int(height * self.image_width / width))
            )
            self.width_ratio = 1
            self.height_ratio = height / width

        self.input_image.fill(0)
        self.input_image[0, : resized.shape[0], : resized.shape[1], :] = resized

    def postprocess_result(self, result: dict) -> None:
        """Scale detection locations to account for letterbox padding."""
        for detections_by_class in result[self.nms_output_key][0]:
            if detections_by_class.size != 0:
                for detection in detections_by_class:
                    detection[0] /= self.height_ratio
                    detection[1] /= self.width_ratio
                    detection[2] /= self.height_ratio
                    detection[3] /= self.width_ratio

        self.latest_result = result

    def infer(self, image: np.ndarray) -> None:
        """Run YOLOv5 on an image; result is stored in self.latest_result."""
        self.preprocess_image(image)

        with hp.InferVStreams(
            self.network_group, self.input_vstreams_params, self.output_vstreams_params
        ) as infer_pipeline:
            input_data = {self.input_vstream_info.name: self.input_image}

            with self.network_group.activate(self.network_group_params):
                result = infer_pipeline.infer(input_data)

        self.postprocess_result(result)
