cmake_minimum_required(VERSION 3.8)
project(stereo_depth_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/StereoImage.msg"
  DEPENDENCIES sensor_msgs std_msgs
)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# Stereo depth ROS2 node
add_executable(stereo_depth_node_exe
    src/stereo_depth_node.cpp
    src/stereo_proc.cpp
    src/stereo_demo_config.cpp
    src/hb_cal_params.cpp
)
set_target_properties(stereo_depth_node_exe PROPERTIES OUTPUT_NAME stereo_depth_node)

target_include_directories(stereo_depth_node_exe PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(stereo_depth_node_exe
    rclcpp
    sensor_msgs
    std_msgs
    diagnostic_msgs
)

target_link_libraries(stereo_depth_node_exe
    ${OpenCV_LIBRARIES}
)

# Link generated message typesupport to the executable
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(stereo_depth_node_exe "${cpp_typesupport_target}")

# Install targets
install(TARGETS stereo_depth_node_exe
    DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
