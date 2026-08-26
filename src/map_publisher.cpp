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
    map_yaml_path_ = this->declare_parameter<std::string>("map_yaml_path", "");
    if (map_yaml_path_.empty()) {
        throw std::invalid_argument("parameter 'map_yaml_path' must not be empty");
    }

    publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", rclcpp::QoS(1).reliable().transient_local());

    RCLCPP_INFO(this->get_logger(), "map_publisher node has been initialized.");
    RCLCPP_INFO(this->get_logger(), "map_yaml_path: %s", map_yaml_path_.c_str());

    if (!this->load_and_publish_map()) {
        throw std::runtime_error("failed to load map");
    }
}

map_publisher::~map_publisher() {
    // nop
}

bool map_publisher::load_and_publish_map() {
    if (!this->load_yaml()) {
        return false;
    }

    const cv::Mat image = this->load_image();
    if (image.empty()) {
        return false;
    }

    auto map               = this->create_occupancy_grid(image);
    map.header.stamp       = this->now();
    map.info.map_load_time = map.header.stamp;
    publisher_->publish(map);

    RCLCPP_INFO(this->get_logger(), "Map loaded successfully: %dx%d, resolution: %.3f m/px", image.cols, image.rows, resolution_);
    return true;
}

bool map_publisher::load_yaml() {
    try {
        const YAML::Node  yaml = YAML::LoadFile(map_yaml_path_);
        const std::string mode = yaml["mode"].as<std::string>("trinary");

        image_path_         = yaml["image"].as<std::string>();
        resolution_         = yaml["resolution"].as<double>();
        origin_             = yaml["origin"].as<std::vector<double>>();
        occupied_threshold_ = yaml["occupied_thresh"].as<double>();
        free_threshold_     = yaml["free_thresh"].as<double>();
        negate_             = yaml["negate"].as<int>();

        if (origin_.size() != 3 || resolution_ <= 0.0 || free_threshold_ < 0.0 || occupied_threshold_ > 1.0 || free_threshold_ >= occupied_threshold_ || mode != "trinary") {
            throw std::invalid_argument("invalid map YAML parameters");
        }

        std::filesystem::path resolved_image_path(image_path_);
        if (resolved_image_path.is_relative()) {
            resolved_image_path = std::filesystem::path(map_yaml_path_).parent_path() / resolved_image_path;
        }
        image_path_ = resolved_image_path.string();
    } catch (const std::exception & error) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load map YAML: %s", error.what());
        return false;
    }

    return true;
}

cv::Mat map_publisher::load_image() const {
    const cv::Mat image = cv::imread(image_path_, cv::IMREAD_GRAYSCALE);
    if (image.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load map image: %s", image_path_.c_str());
    }

    return image;
}

nav_msgs::msg::OccupancyGrid map_publisher::create_occupancy_grid(cv::Mat image) const {
    nav_msgs::msg::OccupancyGrid map;
    map.header.frame_id           = "map";
    map.info.resolution           = static_cast<float>(resolution_);
    map.info.width                = static_cast<std::uint32_t>(image.cols);
    map.info.height               = static_cast<std::uint32_t>(image.rows);
    map.info.origin.position.x    = origin_[0];
    map.info.origin.position.y    = origin_[1];
    map.info.origin.orientation.z = std::sin(origin_[2] / 2.0);
    map.info.origin.orientation.w = std::cos(origin_[2] / 2.0);
    map.data.reserve(image.total());

    for (int row = image.rows - 1; row >= 0; --row) {
        for (int column = 0; column < image.cols; ++column) {
            const double color    = image.at<std::uint8_t>(row, column);
            const double occupied = negate_ ? color / 255.0 : (255.0 - color) / 255.0;

            if (occupied > occupied_threshold_) {
                map.data.push_back(100);
            } else if (occupied < free_threshold_) {
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
