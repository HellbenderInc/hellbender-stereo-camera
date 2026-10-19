#include "hb_cal_params.h"
#include "json.hpp"
#include <fstream>

using json = nlohmann::json;

// Helper function to convert 2D JSON array to cv::Mat
static cv::Mat json_to_mat_2d(const json& j) {
    std::vector<double> flat_data;
    int rows = j.size();
    int cols = j[0].size();
    
    for (const auto& row : j) {
        for (const auto& val : row) {
            flat_data.push_back(val.get<double>());
        }
    }
    
    return cv::Mat(rows, cols, CV_64F, flat_data.data()).clone();
}

// Helper function to convert 1D JSON array to cv::Mat
static cv::Mat json_to_mat_1d(const json& j) {
    std::vector<double> data(j.begin(), j.end());
    return cv::Mat(data.size(), 1, CV_64F, data.data()).clone();
}

StereoRgbCalParamHandler::StereoRgbCalParamHandler(const std::string &file_path)
{
    read_from_json(file_path);
}

void StereoRgbCalParamHandler::read_from_json(const std::string &file_path)
{
    std::cout << "Initializing StereoRgbCalParamHandler with file: " << file_path << std::endl;
    
    std::ifstream file(file_path);
    if (!file.is_open())
    {
        throw std::runtime_error("Failed to open calibration file: " + file_path);
    }

    try {
        json j;
        file >> j;
        
        camera_matrix_left = json_to_mat_2d(j["camera_matrix_left"]);
        camera_matrix_right = json_to_mat_2d(j["camera_matrix_right"]);
        distortion_coeffs_left = json_to_mat_1d(j["distortion_coeffs_left"]);
        distortion_coeffs_right = json_to_mat_1d(j["distortion_coeffs_right"]);
        extrinsics_left_T_right = json_to_mat_2d(j["extrinsics_left_T_right"]);
        stereo_rectification_rotation_left = json_to_mat_2d(j["stereo_rectification_rotation_left"]);
        stereo_rectification_rotation_right = json_to_mat_2d(j["stereo_rectification_rotation_right"]);
        stereo_rectification_projection_left = json_to_mat_2d(j["stereo_rectification_projection_left"]);
        stereo_rectification_projection_right = json_to_mat_2d(j["stereo_rectification_projection_right"]);
        
        std::cout << "Calibration parameters loaded successfully" << std::endl;
    } catch (const json::exception& e) {
        throw std::runtime_error("JSON parsing error in " + file_path + ": " + std::string(e.what()));
    }
}
void StereoRgbCalParamHandler::scale_calibration(const int &old_image_height, const int &new_image_height, const int &old_image_width, const int &new_image_width)
{
    std::cout << "Scaling calibration parameters:" << old_image_width << " -> " << new_image_width << ", " << old_image_height << " -> " << new_image_height << std::endl;
    
    // Validate inputs
    if (old_image_height <= 0 || old_image_width <= 0) {
        throw std::invalid_argument("Invalid old image dimensions: " + std::to_string(old_image_width) + "x" + std::to_string(old_image_height));
    }
    if (new_image_height <= 0 || new_image_width <= 0) {
        throw std::invalid_argument("Invalid new image dimensions: " + std::to_string(new_image_width) + "x" + std::to_string(new_image_height) + ". Check config isp_image_width and isp_image_height");
    }
    
    double scale_x = static_cast<double>(new_image_width) / old_image_width;
    double scale_y = static_cast<double>(new_image_height) / old_image_height;

    camera_matrix_left.at<double>(0, 0) *= scale_x;    
    camera_matrix_left.at<double>(0, 2) *= scale_x;

    camera_matrix_left.at<double>(1, 1) *= scale_y;
    camera_matrix_left.at<double>(1, 2) *= scale_y;
    
    camera_matrix_right.at<double>(0, 0) *= scale_x;
    camera_matrix_right.at<double>(0, 2) *= scale_x;

    camera_matrix_right.at<double>(1, 1) *= scale_y;
    camera_matrix_right.at<double>(1, 2) *= scale_y;

    stereo_rectification_projection_left.at<double>(0, 0) *= scale_x;    
    stereo_rectification_projection_left.at<double>(0, 2) *= scale_x;

    stereo_rectification_projection_left.at<double>(1, 1) *= scale_y;
    stereo_rectification_projection_left.at<double>(1, 2) *= scale_y;
    
    stereo_rectification_projection_right.at<double>(0, 0) *= scale_x;
    stereo_rectification_projection_right.at<double>(0, 2) *= scale_x;

    stereo_rectification_projection_right.at<double>(1, 1) *= scale_y;
    stereo_rectification_projection_right.at<double>(1, 2) *= scale_y;


}