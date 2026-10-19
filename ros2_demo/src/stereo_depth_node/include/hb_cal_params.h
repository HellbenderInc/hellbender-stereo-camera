#pragma once

#include <opencv2/opencv.hpp>

class StereoRgbCalParamHandler {
    public:
        StereoRgbCalParamHandler(const std::string& file_path);
        cv::Mat camera_matrix_left;
        cv::Mat camera_matrix_right;
        cv::Mat distortion_coeffs_left;
        cv::Mat distortion_coeffs_right;
        cv::Mat extrinsics_left_T_right;
        cv::Mat stereo_rectification_rotation_left;
        cv::Mat stereo_rectification_rotation_right;
        cv::Mat stereo_rectification_projection_left;
        cv::Mat stereo_rectification_projection_right;
        void read_from_json(const std::string& file_path);
        void scale_calibration(const int& old_image_height, const int& new_image_height, const int& old_image_width, const int& new_image_width);   
    };
