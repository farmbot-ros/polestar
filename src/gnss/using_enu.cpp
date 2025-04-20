#include "farmbot_interfaces/msg/agent.hpp"
#include "farmbot_interfaces/msg/float32_stamped.hpp"
#include "farmbot_interfaces/msg/float64_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include <cmath>
#include <vector>

#include <concord/wgs_to_enu.hpp>

class Gps2Enu {
  private:
    rclcpp::Node::SharedPtr node_;
    std::string namespace_;
    farmbot_interfaces::msg::Agent my_beacon_;
    sensor_msgs::msg::NavSatFix curr_gps;
    sensor_msgs::msg::NavSatFix datum;
    geometry_msgs::msg::PoseStamped ecef_datum;
    bool datum_set = false;

    std::string frame_id;
    std::string autodatum;

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub_;
    rclcpp::Subscription<farmbot_interfaces::msg::Agent>::SharedPtr beacon_sub_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ecef_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr enu_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ecef_datum_pub_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr geo_dat_pub_;

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
        ecef_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("loc/ecf", 10);
        enu_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("loc/enu", 10);
        ecef_datum_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("ref/ecf", 10);
        geo_dat_pub_ = node_->create_publisher<sensor_msgs::msg::NavSatFix>("ref/wgs", 10);
        beacon_sub_ = node_->create_subscription<farmbot_interfaces::msg::Agent>(
            "beacon/rci", 10, [this](const farmbot_interfaces::msg::Agent::SharedPtr msg) {
                my_beacon_ = *msg;
                set_datum(my_beacon_.zero_ref);
                RCLCPP_INFO(node_->get_logger(), "DATUM SET TO: [%.7f, %.7f, %.7f]", datum.latitude, datum.longitude,
                            datum.altitude);
                beacon_sub_.reset();
            });
    }

  private:
    void callback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr &fix) {
        curr_gps = *fix;
        if (!datum_set) {
            RCLCPP_INFO_ONCE(node_->get_logger(), "NO DATUM SET, PLEASE LAUNCH THE BEACON NODE FIRST!");
            return;
        }

        ecef_datum.header = fix->header;
        ecef_datum_pub_->publish(ecef_datum);
        datum.header = fix->header;
        geo_dat_pub_->publish(datum);

        double lat = fix->latitude, lon = fix->longitude, alt = fix->altitude;
        geometry_msgs::msg::PoseStamped ecef_msg;
        ecef_msg.header = fix->header;
        double ecef_x, ecef_y, ecef_z;
        std::tie(ecef_x, ecef_y, ecef_z) = concord::gps_to_ecef(lat, lon, alt);
        ecef_msg.pose.position.x = ecef_x;
        ecef_msg.pose.position.y = ecef_y;
        ecef_msg.pose.position.z = ecef_z;

        geometry_msgs::msg::PoseStamped enu_msg;
        enu_msg.header = fix->header;
        double d_lat = datum.latitude, d_lon = datum.longitude, d_alt = datum.altitude;
        double enu_x, enu_y, enu_z;
        std::tie(enu_x, enu_y, enu_z) =
            concord::ecef_to_enu(std::make_tuple(ecef_x, ecef_y, ecef_z), std::make_tuple(d_lat, d_lon, d_alt));
        enu_msg.pose.position.x = enu_x;
        enu_msg.pose.position.y = enu_y;
        enu_msg.pose.position.z = curr_gps.altitude - datum.altitude;
        ecef_pub_->publish(ecef_msg);
        enu_pub_->publish(enu_msg);
    }

    void set_datum(geometry_msgs::msg::Point ref) {
        ecef_datum.header = curr_gps.header;
        datum.header = curr_gps.header;
        double lat = ref.x;
        double lon = ref.y;
        double alt = ref.z;
        datum.latitude = lat;
        datum.longitude = lon;
        datum.altitude = alt;
        auto ecef = concord::gps_to_ecef(lat, lon, alt);
        ecef_datum.pose.position.x = std::get<0>(ecef);
        ecef_datum.pose.position.y = std::get<1>(ecef);
        ecef_datum.pose.position.z = std::get<2>(ecef);
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
    rclcpp::shutdown();
    return 0;
}
