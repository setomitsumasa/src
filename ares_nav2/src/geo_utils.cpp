//
// Created by karisora on 2025/09/10.
//

#include <utility>
#include <cmath>
#include "ares_nav2/geo_utils.hpp"

namespace ares_nav2::geo{

    inline double deg2rad(double deg) {return deg * M_PI / 180.0;}
    inline double rad2deg(double rad) {return rad * 180.0 / M_PI;}

    std::pair<double, double> latlon_to_enu(double lat, double lon, double ref_lat, double ref_lon) {
        constexpr double R = 6378137.0; // Earth's radius in meters
        const double lat_rad = deg2rad(lat);
        const double lon_rad = deg2rad(lon);
        const double ref_lat_rad = deg2rad(ref_lat);
        const double ref_lon_rad = deg2rad(ref_lon);
        const double dlat = lat_rad - ref_lat_rad;
        const double dlon = lon_rad - ref_lon_rad;
        const double avg_lat = 0.5 * (lat_rad + ref_lat_rad);
        const double east = dlon * R * std::cos(avg_lat);
        const double north = dlat * R;
        return {east, north};
    }

     Quaternion yaw_to_quaternion(double yaw) {
        double w = std::cos(yaw / 2.0);
        double x = 0.0;
        double y = 0.0;
        double z = std::sin(yaw / 2.0);
        return Quaternion{x, y, z, w};
    }

}