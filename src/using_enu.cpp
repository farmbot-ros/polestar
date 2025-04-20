#include <cmath>
#include <vector>

#include "farmbot_interfaces/msg/float32_stamped.hpp"
#include "farmbot_interfaces/msg/float64_stamped.hpp"
#include "farmbot_interfaces/srv/datum.hpp"
#include "farmbot_interfaces/srv/trigger.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/time_synchronizer.h"

#include <concord/wgs_to_enu.hpp>

class Gps2Enu {
  private:
    rclcpp::Node::SharedPtr node_;
    std::string namespace_;
    sensor_msgs::msg::NavSatFix curr_gps;
    sensor_msgs::msg::NavSatFix datum;
    nav_msgs::msg::Odometry ecef_datum;
    bool datum_set = false;

    std::string frame_id;
    std::string autodatum;

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ecef_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr enu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr geo_dat_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ecef_datum_pub_;

  public:
    Gps2Enu(rclcpp::Node::SharedPtr node) : node_(node) {
        RCLCPP_INFO(node_->get_logger(), "Starting GPS2ENU Node");

        namespace_ = node_->get_namespace();
        if (!namespace_.empty() && namespace_[0] == '/') {
            namespace_ = namespace_.substr(1); // Remove leading slash
        }
        frame_id += namespace_ + "/map";

        autodatum = node_->get_parameter_or<std::string>("autodatum", "auto");

        fix_sub_ = node_->create_subscription<sensor_msgs::msg::NavSatFix>(
            "loc/fix", 10, std::bind(&Gps2Enu::callback, this, std::placeholders::_1));
        ecef_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("loc/ecef", 10);
        enu_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("loc/enu", 10);
        ecef_datum_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("loc/ref", 10);
        geo_dat_pub_ = node_->create_publisher<sensor_msgs::msg::NavSatFix>("loc/ref/geo", 10);
    }

  private:
    void callback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr &fix) {
        curr_gps = *fix;
        if (!datum_set) {
            RCLCPP_ERROR(node_->get_logger(), "NO DATUM SET, PLEASE SET DATUM FIRST!");
            return;
        }

        ecef_datum.header = fix->header;
        ecef_datum_pub_->publish(ecef_datum);
        datum.header = fix->header;
        geo_dat_pub_->publish(datum);

        double lat = fix->latitude, lon = fix->longitude, alt = fix->altitude;
        nav_msgs::msg::Odometry ecef_msg;
        ecef_msg.header = fix->header;
        double ecef_x, ecef_y, ecef_z;
        std::tie(ecef_x, ecef_y, ecef_z) = concord::gps_to_ecef(lat, lon, alt);
        ecef_msg.pose.pose.position.x = ecef_x;
        ecef_msg.pose.pose.position.y = ecef_y;
        ecef_msg.pose.pose.position.z = ecef_z;

        nav_msgs::msg::Odometry enu_msg;
        enu_msg.header = fix->header;
        enu_msg.child_frame_id = frame_id;
        double d_lat = datum.latitude, d_lon = datum.longitude, d_alt = datum.altitude;
        double enu_x, enu_y, enu_z;
        std::tie(enu_x, enu_y, enu_z) =
            concord::ecef_to_enu(std::make_tuple(ecef_x, ecef_y, ecef_z), std::make_tuple(d_lat, d_lon, d_alt));
        enu_msg.pose.pose.position.x = enu_x;
        enu_msg.pose.pose.position.y = enu_y;
        enu_msg.pose.pose.position.z = curr_gps.altitude - datum.altitude;
        ecef_pub_->publish(ecef_msg);
        enu_pub_->publish(enu_msg);
    }

    void set_datum(const sensor_msgs::msg::NavSatFix::ConstSharedPtr &ref) {
        datum = *ref;
        datum.header = ref->header;
        ecef_datum.header = ref->header;
        double lat = ref->latitude;
        double lon = ref->longitude;
        double alt = ref->altitude;
        double x, y, z;
        std::tie(x, y, z) = concord::gps_to_ecef(lat, lon, alt);
        ecef_datum.pose.pose.position.x = x;
        ecef_datum.pose.pose.position.y = y;
        ecef_datum.pose.pose.position.z = z;
        datum_set = true;
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);
    options.automatically_declare_parameters_from_overrides(true);

    rclcpp::Node::SharedPtr node1 = rclcpp::Node::make_shared("using_enu", options);
    std::shared_ptr<Gps2Enu> taskerrr = std::make_shared<Gps2Enu>(node1);

    try {
        executor.add_node(node1);
        executor.spin();
    } catch (const std::exception &e) {
        return 1;
    }
    // rclcpp::shutdown();
    return 0;
}
