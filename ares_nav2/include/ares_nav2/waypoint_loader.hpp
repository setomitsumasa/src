//
// Created by karisora on 2025/09/10.
//

#ifndef ARES_NAV2_WAYPOINT_LOADER_HPP
#define ARES_NAV2_WAYPOINT_LOADER_HPP

#include <vector>
#include <string>
#include "ares_nav2/gps_waypoint_follower.hpp"

namespace ares_nav2 {

    std::vector<GPSWaypoint> load_waypoints(const std::string& waypoint_load_path);
}

#endif //ARES_NAV2_WAYPOINT_LOADER_HPP