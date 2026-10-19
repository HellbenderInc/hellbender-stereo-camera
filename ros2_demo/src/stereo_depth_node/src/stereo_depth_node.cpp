#include "stereo_depth_node.h"
#include <algorithm>
#include <vector>
#include <cstring>

StereoDepthNode::StereoDepthNode()
    : Node("stereo_depth_node")
{
    // Declare parameters
    this->declare_parameter("depth_topic", "/depth/image");
    this->declare_parameter("stereo_image_topic", "/stereo/image");
    this->declare_parameter("calib_file", "/etc/hb_calib_params.json");
    this->declare_parameter("config_file", "/etc/stereo_config.yaml");
    this->declare_parameter("baseline_m", 0.14);
    this->declare_parameter("gstreamer_pipeline", std::string(""));

    // Get parameters
    depth_topic_ = this->get_parameter("depth_topic").as_string();
    stereo_image_topic_ = this->get_parameter("stereo_image_topic").as_string();
    calib_file_ = this->get_parameter("calib_file").as_string();
    config_file_ = this->get_parameter("config_file").as_string();
    baseline_m_ = this->get_parameter("baseline_m").as_double();
    gstreamer_pipeline_ = this->get_parameter("gstreamer_pipeline").as_string();

    // Load config
    config_ = std::make_unique<StereoDemoConfig>(config_file_);
    capture_width_ = config_->isp_image_width;
    capture_height_ = config_->isp_image_height;

    // Load calibration
    calib_ = std::make_unique<StereoRgbCalParamHandler>(calib_file_);

    // Initialize rectification and matcher
    init_rectification_maps();
    init_matcher();

    // Create publishers with QoS best-effort, depth 1
    auto qos = rclcpp::QoS(1).best_effort();
    qos.durability_volatile();
    qos.keep_last(1);
    stereo_image_pub_ = this->create_publisher<stereo_depth_node::msg::StereoImage>(stereo_image_topic_, qos);
    depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>(depth_topic_, qos);
    left_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/left/image", qos);
    depth_vis_pub_ = this->create_publisher<sensor_msgs::msg::Image>(depth_topic_ + "/colorized", qos);
    latency_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "/stereo/pipeline_latency", qos);

    // Conditionally create point cloud publisher
    if (config_->publish_pointcloud) {
        pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/stereo/pointcloud", qos);
        RCLCPP_INFO(this->get_logger(), "Point cloud publishing enabled on /stereo/pointcloud");
    }

    // Build GStreamer pipeline if none provided
    if (gstreamer_pipeline_.empty()) {
        gstreamer_pipeline_ =
            "v4l2src device=/dev/video0 ! video/x-raw,format=UYVY,width=" +
            std::to_string(capture_width_) + ",height=" +
            std::to_string(capture_height_) +
            ",framerate=30/1 ! appsink drop=1";
    }

    RCLCPP_INFO(this->get_logger(), "Stereo depth node initialized. Publishing depth to %s and stereo image to %s",
                depth_topic_.c_str(), stereo_image_topic_.c_str());

    // Start capture thread
    capture_thread_ = std::thread(&StereoDepthNode::capture_loop, this);
}

StereoDepthNode::~StereoDepthNode()
{
    running_ = false;
    if (capture_thread_.joinable()) {
        capture_thread_.join();
    }
}

void StereoDepthNode::init_rectification_maps()
{
    int eye_w = capture_width_ / 2;
    int eye_h = capture_height_;

    // Scale calibration from original 1920x1200 to per-eye capture resolution
    calib_->scale_calibration(1200, eye_h, 1920, eye_w);

    cv::initUndistortRectifyMap(
        calib_->camera_matrix_left,
        calib_->distortion_coeffs_left,
        calib_->stereo_rectification_rotation_left,
        calib_->stereo_rectification_projection_left,
        cv::Size(eye_w, eye_h),
        CV_32FC1,
        left_map1_, left_map2_);

    cv::initUndistortRectifyMap(
        calib_->camera_matrix_right,
        calib_->distortion_coeffs_right,
        calib_->stereo_rectification_rotation_right,
        calib_->stereo_rectification_projection_right,
        cv::Size(eye_w, eye_h),
        CV_32FC1,
        right_map1_, right_map2_);

    // Focal length and principal point from scaled left projection matrix
    focal_length_pix_ = calib_->stereo_rectification_projection_left.at<double>(0, 0);
    focal_length_pix_y_ = calib_->stereo_rectification_projection_left.at<double>(1, 1);
    cx_ = calib_->stereo_rectification_projection_left.at<double>(0, 2);
    cy_ = calib_->stereo_rectification_projection_left.at<double>(1, 2);

    RCLCPP_INFO(this->get_logger(), "Rectification maps initialized (eye: %dx%d, focal: (%.1f, %.1f) px, principal: (%.1f, %.1f))",
                eye_w, eye_h, focal_length_pix_, focal_length_pix_y_, cx_, cy_);
}

void StereoDepthNode::init_matcher()
{
    num_disparities_ = config_->num_disparities;

    if (config_->sgbm) {
        RCLCPP_INFO(this->get_logger(), "Using SGBM matcher (numDisparities=%d, blockSize=%d)",
                    num_disparities_, config_->sgbm_block_size);
        auto sgbm = cv::StereoSGBM::create(
            config_->sgbm_min_disparity,
            num_disparities_,
            config_->sgbm_block_size,
            config_->sgbm_p1,
            config_->sgbm_p2,
            config_->sgbm_disp12_max_diff,
            0, // preFilterCap
            config_->sgbm_uniqueness_ratio,
            config_->sgbm_speckle_window_size,
            config_->sgbm_speckle_range,
            cv::StereoSGBM::MODE_SGBM_3WAY);
        matcher_ = sgbm;
    } else {
        RCLCPP_INFO(this->get_logger(), "Using BM matcher (numDisparities=%d, blockSize=%d)",
                    num_disparities_, config_->bm_block_size);
        auto bm = cv::StereoBM::create(num_disparities_, config_->bm_block_size);
        bm->setUniquenessRatio(config_->bm_uniqueness_ratio);
        matcher_ = bm;
    }
}

sensor_msgs::msg::Image StereoDepthNode::cv_to_ros_image(
    const cv::Mat &cv_image, const std::string &encoding,
    const std_msgs::msg::Header &header)
{
    sensor_msgs::msg::Image msg;
    msg.header = header;
    msg.height = cv_image.rows;
    msg.width = cv_image.cols;
    msg.encoding = encoding;
    msg.is_bigendian = false;

    if (cv_image.empty()) {
        msg.step = 0;
        return msg;
    }

    msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(cv_image.step);

    const size_t size = static_cast<size_t>(msg.step) * msg.height;
    msg.data.resize(size);

    if (cv_image.isContinuous()) {
        std::memcpy(msg.data.data(), cv_image.data, size);
    } else {
        const size_t row_bytes = static_cast<size_t>(cv_image.cols) * cv_image.elemSize();
        for (int row = 0; row < cv_image.rows; ++row) {
            const uint8_t *src = cv_image.ptr<uint8_t>(row);
            uint8_t *dst = msg.data.data() + static_cast<size_t>(row) * msg.step;
            std::memcpy(dst, src, row_bytes);
        }
    }
    return msg;
}

void StereoDepthNode::capture_loop()
{
    cv::VideoCapture cap(gstreamer_pipeline_, cv::CAP_GSTREAMER);
    if (!cap.isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open GStreamer pipeline: %s",
                     gstreamer_pipeline_.c_str());
        return;
    }

    int eye_w = capture_width_ / 2;
    int frame_count = 0;

    while (running_ && rclcpp::ok()) {
        cv::Mat frame;
        cap >> frame;
        if (frame.empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "Failed to read frame from GStreamer");
            continue;
        }

        frame_count++;

        // Tracing: record capture timestamp for end-to-end latency measurement
        auto capture_stamp = this->now();
        auto t_start = std::chrono::steady_clock::now();

        // Convert UYVY to BGR
        cv::Mat frame_bgr;
        cv::cvtColor(frame, frame_bgr, cv::COLOR_YUV2BGR_UYVY);

        // Split side-by-side stereo
        cv::Mat left_bgr = frame_bgr(cv::Rect(0, 0, eye_w, capture_height_));
        cv::Mat right_bgr = frame_bgr(cv::Rect(eye_w, 0, eye_w, capture_height_));

        // Convert to grayscale
        cv::Mat left_gray, right_gray;
        cv::cvtColor(left_bgr, left_gray, cv::COLOR_BGR2GRAY);
        cv::cvtColor(right_bgr, right_gray, cv::COLOR_BGR2GRAY);

        auto t_preprocess = std::chrono::steady_clock::now();

        try {
            // Rectify
            cv::Mat left_rect, right_rect;
            cv::remap(left_gray, left_rect, left_map1_, left_map2_, cv::INTER_LINEAR);
            cv::remap(right_gray, right_rect, right_map1_, right_map2_, cv::INTER_LINEAR);

            auto t_rectify = std::chrono::steady_clock::now();

            // Compute disparity (result is fixed-point with 4 fractional bits)
            cv::Mat disparity_raw;
            matcher_->compute(left_rect, right_rect, disparity_raw);

            cv::Mat disparity_float;
            disparity_raw.convertTo(disparity_float, CV_32F, 1.0 / 16.0);

            auto t_disparity = std::chrono::steady_clock::now();

            // Compute depth from disparity: depth = baseline * focal_length / disparity
            const float bf = static_cast<float>(baseline_m_ * focal_length_pix_);
            cv::Mat valid_disparity = disparity_float > 0.0f;
            cv::Mat disparity_safe = disparity_float.clone();
            disparity_safe.setTo(1.0f, ~valid_disparity);

            cv::Mat depth;
            cv::divide(bf, disparity_safe, depth, 1.0, CV_32F);
            depth.setTo(0.0f, ~valid_disparity);

            // Convert depth to millimeters as uint16
            cv::Mat depth_mm;
            depth.convertTo(depth_mm, CV_16U, 1000.0);

            auto t_depth = std::chrono::steady_clock::now();

            // Build header with capture timestamp for end-to-end latency tracking
            std_msgs::msg::Header header;
            auto publish_stamp = this->now();
            header.stamp = publish_stamp;
            header.frame_id = "stereo";

            // Rectify left colour image
            cv::Mat left_rect_bgr;
            cv::remap(left_bgr, left_rect_bgr, left_map1_, left_map2_, cv::INTER_LINEAR);

            // Convert once per frame and reuse for all publications
            auto left_msg = cv_to_ros_image(left_rect_bgr, "bgr8", header);
            auto depth_msg = cv_to_ros_image(depth_mm, "mono16", header);

            // Publish combined stereo image (left RGB + depth in a single message)
            {
                stereo_depth_node::msg::StereoImage stereo_msg;
                stereo_msg.header = header;
                stereo_msg.left_image = left_msg;
                stereo_msg.depth_image = depth_msg;
                stereo_image_pub_->publish(stereo_msg);
            }

            // Publish standalone depth and left image topics
            auto depth_out = std::make_unique<sensor_msgs::msg::Image>(std::move(depth_msg));
            auto left_out = std::make_unique<sensor_msgs::msg::Image>(std::move(left_msg));
            depth_pub_->publish(std::move(depth_out));
            left_image_pub_->publish(std::move(left_out));

            // Publish colorized depth only when subscribed
            if (depth_vis_pub_->get_subscription_count() > 0) {
                const float min_vis_m = 0.0f;
                const float max_vis_m = std::max(config_->max_range_m, min_vis_m + 1e-3f);

                cv::Mat depth_clipped;
                cv::threshold(depth, depth_clipped, max_vis_m, max_vis_m, cv::THRESH_TRUNC);

                cv::Mat depth_norm;
                depth_clipped.convertTo(
                    depth_norm,
                    CV_8U,
                    255.0f / (max_vis_m - min_vis_m),
                    -255.0f * min_vis_m / (max_vis_m - min_vis_m));

                cv::Mat invalid_mask = (depth <= 0.0f);
                depth_norm.setTo(0, invalid_mask);

                cv::Mat depth_color;
                cv::applyColorMap(depth_norm, depth_color, cv::COLORMAP_JET);
                depth_color.setTo(cv::Scalar(0, 0, 0), invalid_mask);

                auto vis_msg = cv_to_ros_image(depth_color, "bgr8", header);
                depth_vis_pub_->publish(vis_msg);
            }

            // Publish point cloud from depth map when enabled
            if (pointcloud_pub_) {
                sensor_msgs::msg::PointCloud2 cloud;
                cloud.header = header;
                cloud.height = 1;
                cloud.is_dense = false;
                cloud.is_bigendian = false;

                // Fields: x, y, z (float32 each)
                sensor_msgs::msg::PointField fx, fy, fz;
                fx.name = "x"; fx.offset = 0;  fx.datatype = sensor_msgs::msg::PointField::FLOAT32; fx.count = 1;
                fy.name = "y"; fy.offset = 4;  fy.datatype = sensor_msgs::msg::PointField::FLOAT32; fy.count = 1;
                fz.name = "z"; fz.offset = 8;  fz.datatype = sensor_msgs::msg::PointField::FLOAT32; fz.count = 1;
                cloud.fields = {fx, fy, fz};
                cloud.point_step = 12; // 3 x float32

                const float inv_fx = 1.0f / static_cast<float>(focal_length_pix_);
                const float inv_fy = 1.0f / static_cast<float>(focal_length_pix_y_);
                const float cx_f = static_cast<float>(cx_);
                const float cy_f = static_cast<float>(cy_);

                // Count valid points and build data
                std::vector<float> points;
                points.reserve(depth_mm.rows * depth_mm.cols * 3);

                for (int y = 0; y < depth_mm.rows; y++) {
                    const uint16_t *row = depth_mm.ptr<uint16_t>(y);
                    for (int x = 0; x < depth_mm.cols; x++) {
                        if (row[x] == 0) continue;
                        float z = static_cast<float>(row[x]) * 0.001f; // mm → m
                        float px = (static_cast<float>(x) - cx_f) * z * inv_fx;
                        float py = (static_cast<float>(y) - cy_f) * z * inv_fy;
                        points.push_back(px);
                        points.push_back(py);
                        points.push_back(z);
                    }
                }

                uint32_t num_points = static_cast<uint32_t>(points.size() / 3);
                cloud.width = num_points;
                cloud.row_step = cloud.point_step * num_points;
                cloud.data.resize(points.size() * sizeof(float));
                std::memcpy(cloud.data.data(), points.data(), cloud.data.size());

                pointcloud_pub_->publish(cloud);

                if (frame_count % 30 == 0) {
                    RCLCPP_INFO(this->get_logger(), "PointCloud: %u points from %dx%d depth map",
                                num_points, depth_mm.cols, depth_mm.rows);
                }
            }

            auto t_publish = std::chrono::steady_clock::now();

            // Publish pipeline latency diagnostics for receiver-side tracing
            {
                auto to_ms = [](std::chrono::steady_clock::duration d) {
                    return std::chrono::duration<double, std::milli>(d).count();
                };

                double preprocess_ms = to_ms(t_preprocess - t_start);
                double rectify_ms   = to_ms(t_rectify - t_preprocess);
                double disparity_ms = to_ms(t_disparity - t_rectify);
                double depth_ms     = to_ms(t_depth - t_disparity);
                double publish_ms   = to_ms(t_publish - t_depth);
                double total_ms     = to_ms(t_publish - t_start);

                diagnostic_msgs::msg::DiagnosticArray diag;
                diag.header.stamp = capture_stamp;

                diagnostic_msgs::msg::DiagnosticStatus status;
                status.name = "stereo_pipeline_latency";
                status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
                status.message = "Pipeline timing breakdown (ms)";

                auto add_kv = [&](const std::string &key, double val) {
                    diagnostic_msgs::msg::KeyValue kv;
                    kv.key = key;
                    kv.value = std::to_string(val);
                    status.values.push_back(kv);
                };

                add_kv("preprocess_ms", preprocess_ms);
                add_kv("rectify_ms", rectify_ms);
                add_kv("disparity_ms", disparity_ms);
                add_kv("depth_compute_ms", depth_ms);
                add_kv("publish_ms", publish_ms);
                add_kv("total_processing_ms", total_ms);
                add_kv("frame_number", static_cast<double>(frame_count));

                diag.status.push_back(status);
                latency_pub_->publish(diag);

                if (frame_count % 30 == 0) {
                    RCLCPP_INFO(this->get_logger(),
                        "Frame %d latency: preprocess=%.1fms rectify=%.1fms "
                        "disparity=%.1fms depth=%.1fms publish=%.1fms total=%.1fms",
                        frame_count, preprocess_ms, rectify_ms, disparity_ms,
                        depth_ms, publish_ms, total_ms);
                }
            }

        } catch (const cv::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "OpenCV error: %s", e.what());
        }
    }

    cap.release();
    RCLCPP_WARN(this->get_logger(), "Capture loop exited");
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StereoDepthNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
