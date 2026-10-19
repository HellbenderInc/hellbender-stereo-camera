#include "stereo_demo_config.h"

StereoDemoConfig::StereoDemoConfig(const std::string &config_file)
{
    // Load configuration from the file
    try {
        cv::FileStorage fs(config_file, cv::FileStorage::READ);
        if (!fs.isOpened())
        {
            throw std::runtime_error("Failed to open config file: " + config_file);
        }

        fs["h264"] >> h264;
        fs["person_center_ratio"] >> person_center_ratio;
        fs["show_input_image"] >> show_input_image;
        fs["colorize_by_depth"] >> colorize_by_depth;
        fs["output_arange_vertical"] >> output_arange_vertical;
        fs["annotate_input_image"] >> annotate_input_image;
        fs["max_range_m"] >> max_range_m;
        fs["nominal_baseline_m"] >> nominal_baseline_m;
        fs["erode_dialate_disparity"] >> erode_dialate_disparity;
        fs["dialate_erode_kernel_size"] >> dialate_erode_kernel_size;
        fs["downsize_dialate_erode"] >> downsize_dialate_erode;
        fs["show_cpu"] >> show_cpu;
        fs["show_fps"] >> show_fps;
        fs["show_hailo"] >> show_hailo;
        fs["flip_visualization"] >> flip_visualization;
        fs["stats_font_scale"] >> stats_font_scale;
        fs["downsize_disparity"] >> downsize_disparity;
        fs["downsize_disparity_width"] >> downsize_disparity_width;
        fs["downsize_disparity_height"] >> downsize_disparity_height;
        fs["isp_image_width"] >> isp_image_width;
        fs["isp_image_height"] >> isp_image_height;
        fs["num_disparities"] >> num_disparities;

        // SGBM parameters
        fs["sgbm"] >> sgbm;
        fs["sgbm_min_disparity"] >> sgbm_min_disparity;
        fs["sgbm_block_size"] >> sgbm_block_size;
        fs["sgbm_p1"] >> sgbm_p1;
        fs["sgbm_p2"] >> sgbm_p2;
        fs["sgbm_uniqueness_ratio"] >> sgbm_uniqueness_ratio;
        fs["sgbm_speckle_window_size"] >> sgbm_speckle_window_size;
        fs["sgbm_speckle_range"] >> sgbm_speckle_range;
        fs["sgbm_disp12_max_diff"] >> sgbm_disp12_max_diff;

        // BM parameters
        fs["bm_block_size"] >> bm_block_size;
        fs["bm_uniqueness_ratio"] >> bm_uniqueness_ratio;

        // Point cloud
        if (!fs["publish_pointcloud"].empty())
            fs["publish_pointcloud"] >> publish_pointcloud;

        fs.release();
    } catch (const std::exception& e) {
        std::cerr << "Error loading config file '" << config_file << "': " << e.what() << std::endl;
        throw;
    }
}