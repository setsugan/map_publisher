#include "map_publisher/map_publisher.hpp"

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

#include "opencv2/imgcodecs.hpp"
#include "yaml-cpp/yaml.h"

namespace map_publisher {

map_publisher::map_publisher(const rclcpp::NodeOptions & node_options) : Node("map_publisher", node_options) {
    const std::string map_yaml_path = this->declare_parameter<std::string>("map_yaml_path", "");
    if (map_yaml_path.empty()) {
        throw std::invalid_argument("parameter 'map_yaml_path' must not be empty");
    }

    publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", rclcpp::QoS(1).reliable().transient_local());

    auto map               = load_map(map_yaml_path);
    map.header.stamp       = this->now();
    map.info.map_load_time = map.header.stamp;
    publisher_->publish(map);

    RCLCPP_INFO(this->get_logger(), "Map loaded successfully: %ux%u, resolution: %.3f m/px", map.info.width, map.info.height, map.info.resolution);
}

nav_msgs::msg::OccupancyGrid map_publisher::load_map(const std::string & map_yaml_path) {
    const MapConfig config = load_map_config(map_yaml_path);
    const cv::Mat   image  = cv::imread(config.image_path, cv::IMREAD_GRAYSCALE);
    if (image.empty()) {
        throw std::runtime_error("failed to load map image: " + config.image_path);
    }

    return create_occupancy_grid(image, config);
}

map_publisher::MapConfig map_publisher::load_map_config(const std::string & map_yaml_path) {
    try {
        const YAML::Node  yaml = YAML::LoadFile(map_yaml_path);
        const std::string mode = yaml["mode"].as<std::string>("trinary");

        MapConfig config;
        config.image_path         = yaml["image"].as<std::string>();
        config.resolution         = yaml["resolution"].as<double>();
        config.origin             = yaml["origin"].as<std::vector<double>>();
        config.occupied_threshold = yaml["occupied_thresh"].as<double>();
        config.free_threshold     = yaml["free_thresh"].as<double>();
        config.negate             = yaml["negate"].as<int>() != 0;

        if (config.origin.size() != 3 || config.resolution <= 0.0 || config.free_threshold < 0.0 || config.occupied_threshold > 1.0 || config.free_threshold >= config.occupied_threshold || mode != "trinary") {
            throw std::invalid_argument("invalid map YAML parameters");
        }

        std::filesystem::path image_path(config.image_path);
        if (image_path.is_relative()) {
            image_path = std::filesystem::path(map_yaml_path).parent_path() / image_path;
        }
        config.image_path = image_path.string();
        return config;
    } catch (const std::exception & error) {
        throw std::runtime_error("failed to load map YAML '" + map_yaml_path + "': " + error.what());
    }
}

nav_msgs::msg::OccupancyGrid map_publisher::create_occupancy_grid(const cv::Mat & image, const MapConfig & config) {
    nav_msgs::msg::OccupancyGrid map;
    map.header.frame_id           = "map";
    map.info.resolution           = static_cast<float>(config.resolution);
    map.info.width                = static_cast<std::uint32_t>(image.cols);
    map.info.height               = static_cast<std::uint32_t>(image.rows);
    map.info.origin.position.x    = config.origin[0];
    map.info.origin.position.y    = config.origin[1];
    map.info.origin.orientation.z = std::sin(config.origin[2] / 2.0);
    map.info.origin.orientation.w = std::cos(config.origin[2] / 2.0);
    map.data.reserve(image.total());

    for (int row = image.rows - 1; row >= 0; --row) {
        for (int column = 0; column < image.cols; ++column) {
            const double color    = image.at<std::uint8_t>(row, column);
            const double occupied = config.negate ? color / 255.0 : (255.0 - color) / 255.0;

            if (occupied > config.occupied_threshold) {
                map.data.push_back(100);
            } else if (occupied < config.free_threshold) {
                map.data.push_back(0);
            } else {
                map.data.push_back(-1);
            }
        }
    }

    return map;
}

}  // namespace map_publisher

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(map_publisher::map_publisher)
