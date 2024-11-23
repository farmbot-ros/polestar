#pragma once

#include <tuple>
#include <cmath>
#include <GeographicLib/LocalCartesian.hpp>
#include <GeographicLib/Geocentric.hpp>

// Constants
const double R = 6378137.0;                // Earth's radius in meters (equatorial radius)
const double f = 1.0 / 298.257223563;      // Flattening factor
const double e2 = 2 * f - f * f;           // Square of eccentricity

namespace loc_utils {
    std::tuple<double, double, double> gps_to_ecef(double latitude, double longitude, double altitude);
    std::tuple<double, double, double> ecef_to_enu(std::tuple<double, double, double> ecef, std::tuple<double, double, double> datum);
    std::tuple<double, double, double> gps_to_enu(double latitude, double longitude, double altitude, double latRef, double longRef, double altRef);
    std::tuple<double, double, double> enu_to_ecef(std::tuple<double, double, double> enu, std::tuple<double, double, double> datum);
    std::tuple<double, double, double> ecef_to_gps(double x, double y, double z);
    std::tuple<double, double, double> enu_to_gps(double xEast, double yNorth, double zUp, double latRef, double longRef, double altRef);
}
