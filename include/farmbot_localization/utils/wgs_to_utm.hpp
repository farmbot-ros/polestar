#pragma once

#include <cmath>
#include <tuple>
#include <stdexcept>
#include <GeographicLib/UTMUPS.hpp>

// Constants used in the UTM conversion process
constexpr double a = 6378137.0; // WGS-84 major axis
constexpr double f = 1.0 / 298.257223563; // WGS-84 flattening
constexpr double k0 = 0.9996; // UTM scale factor
constexpr double e2 = f * (2 - f); // Square of eccentricity
constexpr double e4 = e2 * e2;
constexpr double e6 = e4 * e2;
constexpr double ep2 = e2 / (1 - e2); // Second eccentricity squared

namespace loc_utils {
    // Function to compute UTM zone from longitude
    int get_utm_zone(double longitude) {
        return static_cast<int>((longitude + 180.0) / 6) + 1;
    }

    // Function to determine whether the point is in the northern hemisphere
    bool is_northern_hemisphere(double latitude) {
        return latitude >= 0.0;
    }

    std::tuple<double, double, int, bool> wgs_to_utm(double latitude, double longitude);
    std::tuple<double, double> utm_to_wgs(double easting, double northing, int zone, bool is_northern_hemisphere);
}
