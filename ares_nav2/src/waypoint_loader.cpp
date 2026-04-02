//
// Created by karisora on 2025/09/10.
//

#include "ares_nav2/waypoint_loader.hpp"
#include <yaml-cpp/yaml.h>
#include <stdexcept>

namespace ares_nav2 {
    std::vector<GPSWaypoint> load_waypoints(const std::string &waypoint_load_path) {
        YAML::Node root = YAML::LoadFile(waypoint_load_path);
        if (!root["waypoints"]) throw std::runtime_error("No waypoints found");
        std::vector<GPSWaypoint> out;
        for (const auto &node : root["waypoints"]) {
            GPSWaypoint wp;
            wp.latitude = node["latitude"].as<double>();
            wp.longitude = node["longitude"].as<double>();
            wp.yaw = node["yaw"].as<double>(0.0); // default to 0.0 if not provided
            wp.spiral_search = node["spiral_search"].as<bool>(false); // default to false if not provided
            wp.marker_id = node["marker_id"].as<int>(-1); // default to -1 if not provided
            out.push_back(wp);
        }
        return out;
    }

}