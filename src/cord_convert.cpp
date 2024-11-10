#include <memory>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "farmbot_interfaces/srv/ecef2_enu.hpp"
#include "farmbot_interfaces/srv/ecef2_gps.hpp"
#include "farmbot_interfaces/srv/enu2_ecef.hpp"
#include "farmbot_interfaces/srv/enu2_gps.hpp"
#include "farmbot_interfaces/srv/gps2_ecef.hpp"
#include "farmbot_interfaces/srv/gps2_enu.hpp"

#include "farmbot_localization/utils/wgs_to_enu.hpp"

using namespace std::placeholders;

class CoordinateTransformer : public rclcpp::Node {
    private:
        sensor_msgs::msg::NavSatFix geo_datum;
        rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr geo_datum_sub_;
        nav_msgs::msg::Odometry ecef_datum;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ecef_datum_sub_;
        rclcpp::Service<farmbot_interfaces::srv::Ecef2Enu>::SharedPtr ecef2enu_service_;
        rclcpp::Service<farmbot_interfaces::srv::Ecef2Gps>::SharedPtr ecef2gps_service_;
        rclcpp::Service<farmbot_interfaces::srv::Enu2Ecef>::SharedPtr enu2ecef_service_;
        rclcpp::Service<farmbot_interfaces::srv::Enu2Gps>::SharedPtr enu2gps_service_;
        rclcpp::Service<farmbot_interfaces::srv::Gps2Ecef>::SharedPtr gps2ecef_service_;
        rclcpp::Service<farmbot_interfaces::srv::Gps2Enu>::SharedPtr gps2enu_service_;
        std::string name;
        bool datum_set;
    public:
        CoordinateTransformer() : Node(
            "coordinate_transformer",
            rclcpp::NodeOptions()
            .allow_undeclared_parameters(true)
            .automatically_declare_parameters_from_overrides(true)
        ){
            name = this->get_parameter_or<std::string>("name", "using_enu");


            ecef2enu_service_ = this->create_service<farmbot_interfaces::srv::Ecef2Enu>(
                "loc/ecef2enu", std::bind(&CoordinateTransformer::ecef2enuCallback, this, _1, _2));

            ecef2gps_service_ = this->create_service<farmbot_interfaces::srv::Ecef2Gps>(
                "loc/ecef2gps", std::bind(&CoordinateTransformer::ecef2gpsCallback, this, _1, _2));

            enu2ecef_service_ = this->create_service<farmbot_interfaces::srv::Enu2Ecef>(
                "loc/enu2ecef", std::bind(&CoordinateTransformer::enu2ecefCallback, this, _1, _2));

            enu2gps_service_ = this->create_service<farmbot_interfaces::srv::Enu2Gps>(
                "loc/enu2gps", std::bind(&CoordinateTransformer::enu2gpsCallback, this, _1, _2));

            gps2ecef_service_ = this->create_service<farmbot_interfaces::srv::Gps2Ecef>(
                "loc/gps2ecef", std::bind(&CoordinateTransformer::gps2ecefCallback, this, _1, _2));

            gps2enu_service_ = this->create_service<farmbot_interfaces::srv::Gps2Enu>(
                "loc/gps2enu", std::bind(&CoordinateTransformer::gps2enuCallback, this, _1, _2));

            geo_datum_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
                "loc/ref/geo", 10, [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
                    datum_set = true;
                    geo_datum = *msg;
                });

            ecef_datum_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                "loc/ref", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                    // datum_set = true;
                    ecef_datum = *msg;
                });

            RCLCPP_INFO(this->get_logger(), "Coordinate Transformer Node started");
        }

    private:
        void ecef2enuCallback(
            const std::shared_ptr<farmbot_interfaces::srv::Ecef2Enu::Request> request,
            std::shared_ptr<farmbot_interfaces::srv::Ecef2Enu::Response> response) {
            if (!datum_set) {
                RCLCPP_INFO(this->get_logger(), "Datum not set, cannot convert ECEF to ENU");
                response->message = "Datum not set, cannot convert ECEF to ENU";
                return;
            }
            // RCLCPP_INFO(this->get_logger(), "Converting ECEF to ENU...");
            for (const auto &pose : request->ecef) {
                geometry_msgs::msg::Pose enu_pose;
                std::tuple<double, double, double> ecef_to_enu = loc_utils::ecef_to_enu(std::make_tuple(pose.position.x, pose.position.y, pose.position.z),
                    std::make_tuple(geo_datum.latitude, geo_datum.longitude, geo_datum.altitude));
                enu_pose.position.x = std::get<0>(ecef_to_enu);
                enu_pose.position.y = std::get<1>(ecef_to_enu);
                enu_pose.position.z = std::get<2>(ecef_to_enu);
                response->enu.push_back(enu_pose);
            }
            response->message = "Converted ECEF to ENU";
            return;
        }

        void ecef2gpsCallback(
            const std::shared_ptr<farmbot_interfaces::srv::Ecef2Gps::Request> request,
            std::shared_ptr<farmbot_interfaces::srv::Ecef2Gps::Response> response) {
            // RCLCPP_INFO(this->get_logger(), "Converting ECEF to GPS...");
            for (const auto &pose : request->ecef) {
                sensor_msgs::msg::NavSatFix gps;
                std::tuple<double, double, double> ecef_to_gps = loc_utils::ecef_to_gps(pose.position.x, pose.position.y, pose.position.z);
                gps.latitude = std::get<0>(ecef_to_gps);
                gps.longitude = std::get<1>(ecef_to_gps);
                gps.altitude = std::get<2>(ecef_to_gps);
                response->gps.push_back(gps);
            }
            response->message = "Converted ECEF to GPS";
            return;
        }

        void enu2ecefCallback(
            const std::shared_ptr<farmbot_interfaces::srv::Enu2Ecef::Request> request,
            std::shared_ptr<farmbot_interfaces::srv::Enu2Ecef::Response> response) {
            if (!datum_set) {
                RCLCPP_INFO(this->get_logger(), "Datum not set, cannot convert ENU to ECEF");
                response->message = "Datum not set, cannot convert ENU to ECEF";
                return;
            }
            // RCLCPP_INFO(this->get_logger(), "Converting ENU to ECEF...");
            for (const auto &pose : request->enu) {
                geometry_msgs::msg::Pose ecef_pose;
                std::tuple<double, double, double> enu_to_ecef = loc_utils::enu_to_ecef(std::make_tuple(pose.position.x, pose.position.y, pose.position.z),
                    std::make_tuple(geo_datum.latitude, geo_datum.longitude, geo_datum.altitude));
                ecef_pose.position.x = std::get<0>(enu_to_ecef);
                ecef_pose.position.y = std::get<1>(enu_to_ecef);
                ecef_pose.position.z = std::get<2>(enu_to_ecef);
                response->ecef.push_back(ecef_pose);
            }
            response->message = "Converted ENU to ECEF";
            return;
        }

        void enu2gpsCallback(
            const std::shared_ptr<farmbot_interfaces::srv::Enu2Gps::Request> request,
            std::shared_ptr<farmbot_interfaces::srv::Enu2Gps::Response> response) {
            if (!datum_set) {
                RCLCPP_INFO(this->get_logger(), "Datum not set, cannot convert ENU to GPS");
                response->message = "Datum not set, cannot convert ENU to GPS";
                return;
            }
            // RCLCPP_INFO(this->get_logger(), "Converting ENU to GPS...");
            for (const auto &pose : request->enu) {
                sensor_msgs::msg::NavSatFix gps;
                std::tuple<double, double, double> enu_to_gps = loc_utils::enu_to_gps(pose.position.x, pose.position.y, pose.position.z,
                    geo_datum.latitude, geo_datum.longitude, geo_datum.altitude);
                gps.latitude = std::get<0>(enu_to_gps);
                gps.longitude = std::get<1>(enu_to_gps);
                gps.altitude = std::get<2>(enu_to_gps);
                response->gps.push_back(gps);
            }
            response->message = "Converted ENU to GPS";
            return;
        }

        void gps2ecefCallback(
            const std::shared_ptr<farmbot_interfaces::srv::Gps2Ecef::Request> request,
            std::shared_ptr<farmbot_interfaces::srv::Gps2Ecef::Response> response) {
            // RCLCPP_INFO(this->get_logger(), "Converting GPS to ECEF...");
            for (const auto &gps : request->gps) {
                geometry_msgs::msg::Pose ecef_pose;
                std::tuple<double, double, double> gps_to_ecef = loc_utils::gps_to_ecef(gps.latitude, gps.longitude, gps.altitude);
                ecef_pose.position.x = std::get<0>(gps_to_ecef);
                ecef_pose.position.y = std::get<1>(gps_to_ecef);
                ecef_pose.position.z = std::get<2>(gps_to_ecef);
                response->ecef.push_back(ecef_pose);
            }
            response->message = "Converted GPS to ECEF";
            return;
        }

        void gps2enuCallback(
            const std::shared_ptr<farmbot_interfaces::srv::Gps2Enu::Request> request,
            std::shared_ptr<farmbot_interfaces::srv::Gps2Enu::Response> response) {
            if (!datum_set) {
                RCLCPP_INFO(this->get_logger(), "Datum not set, cannot convert GPS to ENU");
                response->message = "Datum not set, cannot convert GPS to ENU";
                return;
            }
            // RCLCPP_INFO(this->get_logger(), "Converting GPS to ENU...");
            for (const auto &gps : request->gps) {
                geometry_msgs::msg::Pose enu_pose;
                std::tuple<double, double, double> gps_to_enu = loc_utils::gps_to_enu(gps.latitude, gps.longitude, gps.altitude,
                    geo_datum.latitude, geo_datum.longitude, geo_datum.altitude);
                enu_pose.position.x = std::get<0>(gps_to_enu);
                enu_pose.position.y = std::get<1>(gps_to_enu);
                enu_pose.position.z = std::get<2>(gps_to_enu);
                response->enu.push_back(enu_pose);
            }
        }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CoordinateTransformer>());
    rclcpp::shutdown();
    return 0;
}
