cmake_minimum_required(VERSION 3.8)
project(map_publisher)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(OpenCV REQUIRED COMPONENTS imgcodecs)

ament_auto_add_library(${PROJECT_NAME} SHARED
    DIRECTORY src
)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} yaml-cpp::yaml-cpp)

rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "map_publisher::MapPublisher"
    EXECUTABLE map_publisher_node
)

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
