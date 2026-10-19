#pragma once

#include <opencv2/opencv.hpp>
#include "hb_cal_params.h"
#include "stereo_demo_config.h"

class HbImageRectification
{
    public:
        HbImageRectification(const std::string &file_path, const int &image_width, const int &image_height);
        void rectify(const cv::Mat &input_image, cv::Mat &output_image, std::string image_name);
        StereoRgbCalParamHandler stereo_calibration;
        cv::Mat left_map_1;
        cv::Mat left_map_2;
        cv::Mat right_map_1;
        cv::Mat right_map_2;
};

class HbDisparityEstimation
{
    public:
        HbDisparityEstimation(const std::string &file_path, const StereoDemoConfig &config_);
        void get_disparity(const cv::Mat &left_image, const cv::Mat &right_image, cv::Mat &disparity_map);
        void match(const cv::Mat &left_image, const cv::Mat &right_image, cv::Mat &disparity_map);
        cv::Ptr<cv::StereoMatcher> matcher;
        HbImageRectification rectifier;
        cv::Mat kernel;
        cv::Mat left_rectified;
        cv::Mat right_rectified;
        cv::Mat left_gray;
        cv::Mat right_gray;
        cv::Mat disparity_temp;
        cv::Mat disparity_small;

        cv::Mat matcher_input_left;
        cv::Mat matcher_input_right;
        cv::Mat roi_left;
        cv::Mat roi_right;
        cv::Mat matcher_output;

        int calculation_width;
        int calculation_height;
        const StereoDemoConfig &config;
        cv::Ptr<cv::CLAHE> clahe;
};
