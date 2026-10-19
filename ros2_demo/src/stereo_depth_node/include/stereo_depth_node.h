#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/header.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <opencv2/opencv.hpp>

#include <chrono>
#include <thread>
#include <atomic>
#include <string>
#include <memory>

#include "stereo_depth_node/msg/stereo_image.hpp"
#include "hb_cal_params.h"
#include "stereo_demo_config.h"

class StereoDepthNode : public rclcpp::Node
{
public:
    StereoDepthNode();
    ~StereoDepthNode();

private:
    void init_rectification_maps();
    void init_matcher();
    void capture_loop();
    sensor_msgs::msg::Image cv_to_ros_image(
        const cv::Mat &cv_image, const std::string &encoding,
        const std_msgs::msg::Header &header);

    // ROS publishers
    rclcpp::Publisher<stereo_depth_node::msg::StereoImage>::SharedPtr stereo_image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_vis_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr latency_pub_;

    // Parameters
    std::string depth_topic_;
    std::string stereo_image_topic_;
    std::string calib_file_;
    std::string config_file_;
    double baseline_m_;
    std::string gstreamer_pipeline_;

    // Calibration & config
    std::unique_ptr<StereoRgbCalParamHandler> calib_;
    std::unique_ptr<StereoDemoConfig> config_;

    // Rectification maps
    cv::Mat left_map1_, left_map2_;
    cv::Mat right_map1_, right_map2_;

    // Stereo matcher
    cv::Ptr<cv::StereoMatcher> matcher_;
    int num_disparities_;
    double focal_length_pix_;  // fx from projection matrix
    double focal_length_pix_y_; // fy from projection matrix
    double cx_;
    double cy_;

    // Capture
    int capture_width_;
    int capture_height_;
    std::thread capture_thread_;
    std::atomic<bool> running_{true};
};
