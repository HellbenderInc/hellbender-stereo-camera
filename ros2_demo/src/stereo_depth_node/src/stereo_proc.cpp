#include "stereo_proc.h"

HbImageRectification::HbImageRectification(const std::string &file_path, const int &image_width, const int &image_height)
    : stereo_calibration(file_path)
{
    try {
        // Scale calibration parameters
        stereo_calibration.scale_calibration(1200, image_height, 1920, image_width);

        // Initialize rectification maps
        cv::initUndistortRectifyMap(
            stereo_calibration.camera_matrix_left,
            stereo_calibration.distortion_coeffs_left,
            stereo_calibration.stereo_rectification_rotation_left,
            stereo_calibration.stereo_rectification_projection_left,
            cv::Size(image_width, image_height),
            CV_32FC1,
            left_map_1,
            left_map_2);

        cv::initUndistortRectifyMap(
            stereo_calibration.camera_matrix_right,
            stereo_calibration.distortion_coeffs_right,
            stereo_calibration.stereo_rectification_rotation_right,
            stereo_calibration.stereo_rectification_projection_right,
            cv::Size(image_width, image_height),
            CV_32FC1,
            right_map_1,
            right_map_2);
    } catch (const cv::Exception& e) {
        throw std::runtime_error("Error initializing rectification maps: " + std::string(e.what()));
    }
}

void HbImageRectification::rectify(const cv::Mat &input_image, cv::Mat &output_image, std::string image_name)
{
    try {
        if (image_name == "left")
        {
            cv::remap(input_image, output_image, left_map_1, left_map_2, cv::INTER_LINEAR);
        }
        else if (image_name == "right")
        {
            cv::remap(input_image, output_image, right_map_1, right_map_2, cv::INTER_LINEAR);
        }
        else
        {
            throw std::invalid_argument("Invalid image name: " + image_name);
        }
    } catch (const cv::Exception& e) {
        throw std::runtime_error("Error in rectify() for " + image_name + ": " + std::string(e.what()));
    }
}

HbDisparityEstimation::HbDisparityEstimation(const std::string &file_path, const StereoDemoConfig &config_)
    : rectifier(file_path, config_.isp_image_width, config_.isp_image_height),
      config(config_)
{
    try {
        // Validate kernel size before creating
        if (config.dialate_erode_kernel_size <= 0 || config.dialate_erode_kernel_size % 2 == 0) {
            throw std::invalid_argument("dialate_erode_kernel_size must be positive and odd, got: " + std::to_string(config.dialate_erode_kernel_size));
        }
        
        kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(config.dialate_erode_kernel_size, config.dialate_erode_kernel_size));
        
        // Initialize StereoBM matcher
        if (config.sgbm)
        {
            std::cout << "Using SGBM" << std::endl;
            cv::Ptr<cv::StereoSGBM> matcher_sgbm = cv::StereoSGBM::create(config.sgbm_min_disparity,
                                                                          config.num_disparities,
                                                                          config.sgbm_block_size,
                                                                          config.sgbm_p1,
                                                                          config.sgbm_p2,
                                                                          config.sgbm_speckle_window_size,
                                                                          config.sgbm_speckle_range,
                                                                          config.sgbm_disp12_max_diff,
                                                                          cv::StereoSGBM::MODE_SGBM_3WAY);
            matcher = matcher_sgbm;
        }
        else
        {
            cv::Ptr<cv::StereoBM> matcher_bm = cv::StereoBM::create(config.num_disparities,
                                                                    config.bm_block_size);
            matcher_bm->setUniquenessRatio(config.bm_uniqueness_ratio);
            matcher = matcher_bm;
        }

        disparity_temp = cv::Mat(config.isp_image_height, config.isp_image_width, CV_16S);
        left_rectified = cv::Mat(config.isp_image_height, config.isp_image_width, CV_8UC1);
        right_rectified = cv::Mat(config.isp_image_height, config.isp_image_width, CV_8UC1);
        left_gray = cv::Mat(config.downsize_disparity_height, config.downsize_disparity_width, CV_8UC1);
        right_gray = cv::Mat(config.downsize_disparity_height, config.downsize_disparity_width, CV_8UC1);

        disparity_small = cv::Mat(config.downsize_disparity_height, config.downsize_disparity_width, CV_16S);

        int disparity_width;

        if (config.downsize_disparity)
        {
            calculation_height = config.downsize_disparity_height;
            calculation_width = config.downsize_disparity_width;
        }
        else
        {
            calculation_height = config.isp_image_height;
            calculation_width = config.isp_image_width;
        }

        disparity_width = calculation_width + config.num_disparities;
        matcher_input_left = cv::Mat::zeros(calculation_height, disparity_width, CV_8UC1);
        matcher_input_right = cv::Mat::zeros(calculation_height, disparity_width, CV_8UC1);
        matcher_output = cv::Mat::zeros(calculation_height, disparity_width, CV_16S);
        roi_left = matcher_input_left(cv::Rect(config.num_disparities, 0, calculation_width, calculation_height));
        roi_right = matcher_input_right(cv::Rect(config.num_disparities, 0, calculation_width, calculation_height));

        clahe = cv::createCLAHE();
    } catch (const cv::Exception& e) {
        throw std::runtime_error("Error initializing HbDisparityEstimation: " + std::string(e.what()));
    }
}

void HbDisparityEstimation::match(const cv::Mat &left_image, const cv::Mat &right_image, cv::Mat &disparity_map)
{
    try {
        left_image.copyTo(roi_left);
        right_image.copyTo(roi_right);
        matcher->compute(matcher_input_left, matcher_input_right, matcher_output);
        matcher_output(cv::Rect(config.num_disparities, 0, calculation_width, calculation_height)).copyTo(disparity_map);
    } catch (const cv::Exception& e) {
        throw std::runtime_error("Error in match() stereo computation: " + std::string(e.what()));
    }
}

void HbDisparityEstimation::get_disparity(const cv::Mat &left_image, const cv::Mat &right_image, cv::Mat &disparity_map)
{
    try {
        rectifier.rectify(left_image, left_rectified, "left");
        rectifier.rectify(right_image, right_rectified, "right");

        clahe->apply(left_rectified, left_rectified);
        clahe->apply(right_rectified, right_rectified);

        if (config.downsize_disparity)
        {

            cv::resize(left_rectified, left_gray, cv::Size(config.downsize_disparity_width, config.downsize_disparity_height), 0, 0, cv::INTER_LINEAR);
            cv::resize(right_rectified, right_gray, cv::Size(config.downsize_disparity_width, config.downsize_disparity_height), 0, 0, cv::INTER_LINEAR);
            // Compute disparity map

            this->match(left_gray, right_gray, disparity_small);

            if (config.downsize_dialate_erode)
            {
                cv::dilate(disparity_small, disparity_small, kernel);
                cv::dilate(disparity_small, disparity_small, kernel);
                cv::erode(disparity_small, disparity_small, kernel);
            }

            // Resize the disparity map to the original size
            disparity_small *= config.isp_image_width / static_cast<float>(config.downsize_disparity_width);
            cv::resize(disparity_small, disparity_temp, cv::Size(config.isp_image_width, config.isp_image_height), 0, 0, cv::INTER_LINEAR);
        }
        else
        {
            // Compute disparity map
            this->match(left_rectified, right_rectified, disparity_temp);
        }

        disparity_temp = 255.0 * disparity_temp / 1024.0f; // Scale the disparity map

        if (config.erode_dialate_disparity)
        {
            // Apply morphological operations to clean up the disparity map
            cv::erode(disparity_temp, disparity_temp, kernel);
            cv::dilate(disparity_temp, disparity_temp, kernel);
        }
        cv::Mat disparity_8u;
        cv::convertScaleAbs(disparity_temp, disparity_8u);
        // Convert to 3-channel image for visualization
        cv::applyColorMap(disparity_8u, disparity_map, cv::COLORMAP_JET);
    } catch (const cv::Exception& e) {
        throw std::runtime_error("Error in get_disparity(): " + std::string(e.what()));
    }
}