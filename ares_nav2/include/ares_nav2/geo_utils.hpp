//
// Created by karisora on 2025/09/10.
//

#ifndef ARES_NAV2_GEO_UTILS_HPP
#define ARES_NAV2_GEO_UTILS_HPP

#include <cmath>
#include <utility>

namespace ares_nav2::geo {

    struct Quaternion {
        double x, y, z, w;
    };

    std::pair<double, double> latlon_to_enu(double lat, double lon, double ref_lat, double ref_lon);
    Quaternion yaw_to_quaternion(double yaw);
}

#endif //ARES_NAV2_GEO_UTILS_HPP