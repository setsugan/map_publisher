#ifndef MAP_PUBLISHER__MAP_PUBLISHER_HPP_
#define MAP_PUBLISHER__MAP_PUBLISHER_HPP_

#include <string>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "opencv2/core/mat.hpp"
#include "rclcpp/rclcpp.hpp"

namespace map_publisher {

class map_publisher : public rclcpp::Node {
public:
    explicit map_publisher(const rclcpp::NodeOptions & node_options);

private:
    struct MapConfig {
        std::string         image_path;
        double              resolution;
        std::vector<double> origin;
        double              occupied_threshold;
        double              free_threshold;
        bool                negate;
    };

    static MapConfig                    load_map_config(const std::string & map_yaml_path);
    static nav_msgs::msg::OccupancyGrid create_occupancy_grid(const cv::Mat & image, const MapConfig & config);
    static nav_msgs::msg::OccupancyGrid load_map(const std::string & map_yaml_path);

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
};

}  // namespace map_publisher

#endif  // MAP_PUBLISHER__MAP_PUBLISHER_HPP_
