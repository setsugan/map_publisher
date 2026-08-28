// Copyright 2026 Riku Yukimaru
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
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
