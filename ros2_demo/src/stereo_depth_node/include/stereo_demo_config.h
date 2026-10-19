#pragma once

#include <opencv2/opencv.hpp>

class StereoDemoConfig
{
public:
    StereoDemoConfig(const std::string &config_file);
    bool h264 = false;
    float person_center_ratio = 0.3;
    bool show_input_image = true;
    bool colorize_by_depth = false;
    bool output_arange_vertical = true;
    bool annotate_input_image = true;
    float max_range_m = 10.0;
    float nominal_baseline_m = 0.12;
    bool erode_dialate_disparity = true;
    bool show_cpu = true;
    bool show_fps = true;
    bool show_hailo = false;
    bool flip_visualization = false;
    float stats_font_scale = 1.0;
    bool sgbm = true;
    int sgbm_min_disparity = 0;
    int sgbm_block_size = 7;
    int sgbm_p1 = 0;
    int sgbm_p2 = 0;
    int sgbm_uniqueness_ratio = 55;
    int sgbm_speckle_window_size = 0;
    int sgbm_speckle_range = 0;
    int sgbm_disp12_max_diff = 0;
    bool downsize_disparity = false;
    int downsize_disparity_width = 960;
    int downsize_disparity_height = 600;
    int num_disparities = 64;
    int bm_block_size = 7;
    int bm_uniqueness_ratio = 55;
    int isp_image_width = 960;
    int isp_image_height = 600;
    int dialate_erode_kernel_size = 3;
    bool downsize_dialate_erode = false;
    bool publish_pointcloud = false;
};