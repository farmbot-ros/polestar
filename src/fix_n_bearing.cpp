#include "farmbot_interfaces/msg/float32_stamped.hpp"
#include "farmbot_interfaces/srv/datum.hpp"
#include "farmbot_interfaces/srv/trigger.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/time_synchronizer.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/float32.hpp"
#include <cmath>
#include <rclcpp/logging.hpp>

double toRadians(double degrees) {
    auto rads = std::fmod(degrees * M_PI / 180.0, 360.0);
    return rads;
}

double toDegrees(double radians) {
    auto degs = std::fmod(radians * 180.0 / M_PI, 360.0);
    return degs;
}

class GpsAndDeg {
  private:
    rclcpp::Node::SharedPtr node_;
    std::string namespace_;
    sensor_msgs::msg::NavSatFix curr_gps;
    std_msgs::msg::Float32 heading;
    std_msgs::msg::Float32 compass;

    std::string fix_topic;
    std::string orientation_topic;
    std::string units;

    std::string frame_id;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_corr_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr head_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr comp_sub_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;
    rclcpp::Publisher<farmbot_interfaces::msg::Float32Stamped>::SharedPtr deg_;
    rclcpp::Publisher<farmbot_interfaces::msg::Float32Stamped>::SharedPtr rad_;

  public:
    GpsAndDeg(rclcpp::Node::SharedPtr node) : node_(node) {
        RCLCPP_INFO(node_->get_logger(), "Starting GPS & DEG Node");
        namespace_ = node_->get_namespace();
        if (!namespace_.empty() && namespace_[0] == '/') {
            namespace_ = namespace_.substr(1);
        }
        frame_id = namespace_ + "/gps";

        fix_topic = node_->get_parameter_or<std::string>("fix", "gnss/fix");
        orientation_topic = node_->get_parameter_or<std::string>("orientation", "gnss/heading");
        units = node_->get_parameter_or<std::string>("units", "degrees");
        gps_corr_ = node_->create_subscription<sensor_msgs::msg::NavSatFix>(
            fix_topic, 10,
            [this](const sensor_msgs::msg::NavSatFix::ConstSharedPtr &fix_topic_msg) { curr_gps = *fix_topic_msg; });
        head_sub_ = node_->create_subscription<std_msgs::msg::Float32>(
            orientation_topic, 10, [this](const std_msgs::msg::Float32::ConstSharedPtr &orientation_topic_msg) {
                heading = *orientation_topic_msg;
            });
        timer_ = node_->create_wall_timer(std::chrono::milliseconds(10), std::bind(&GpsAndDeg::timer_callback, this));
        gps_pub_ = node_->create_publisher<sensor_msgs::msg::NavSatFix>("loc/fix", 10);
        deg_ = node_->create_publisher<farmbot_interfaces::msg::Float32Stamped>("loc/deg", 10);
        rad_ = node_->create_publisher<farmbot_interfaces::msg::Float32Stamped>("loc/rad", 10);
    }

  private:
    void timer_callback() {
        sensor_msgs::msg::NavSatFix curr_pose;
        curr_pose = curr_gps;
        curr_pose.header.frame_id = frame_id;
        curr_pose.header.stamp = node_->now();
        gps_pub_->publish(curr_pose);

        farmbot_interfaces::msg::Float32Stamped deg_msg;
        deg_msg.header = curr_pose.header;
        farmbot_interfaces::msg::Float32Stamped rad_msg;
        rad_msg.header = curr_pose.header;

        if (units == "degrees") {
            deg_msg.data = heading.data;
            rad_msg.data = toRadians(heading.data);
        } else if (units == "radians") {
            deg_msg.data = toDegrees(heading.data);
            rad_msg.data = heading.data;
        } else {
            RCLCPP_WARN(node_->get_logger(), "Invalid units parameter: %s", units.c_str());
            return;
        }
        deg_->publish(deg_msg);
        rad_->publish(rad_msg);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);
    options.automatically_declare_parameters_from_overrides(true);

    rclcpp::Node::SharedPtr node1 = rclcpp::Node::make_shared("fix_n_bearing", options);
    std::shared_ptr<GpsAndDeg> taskerrr = std::make_shared<GpsAndDeg>(node1);

    try {
        executor.add_node(node1);
        executor.spin();
    } catch (const std::exception &e) {
        return 1;
    }
    // rclcpp::shutdown();
    return 0;
}
