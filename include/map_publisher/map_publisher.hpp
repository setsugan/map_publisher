#ifndef MAP_PUBLISHER__MAP_PUBLISHER_HPP_
#define MAP_PUBLISHER__MAP_PUBLISHER_HPP_

#include <string>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "opencv2/core/mat.hpp"
#include "rclcpp/rclcpp.hpp"

namespace map_publisher {

class MapPublisher : public rclcpp::Node {
public:
    explicit MapPublisher(const rclcpp::NodeOptions & node_options);
    ~MapPublisher();

private:
    bool                         load_and_publish_map();
    bool                         load_yaml();
    cv::Mat                      load_image() const;
    nav_msgs::msg::OccupancyGrid create_occupancy_grid(cv::Mat image) const;

    std::string         map_yaml_path_;
    std::string         image_path_;
    double              resolution_{};
    std::vector<double> origin_;
    double              occupied_threshold_{};
    double              free_threshold_{};
    int                 negate_{};

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
};

}  // namespace map_publisher

#endif  // MAP_PUBLISHER__MAP_PUBLISHER_HPP_
