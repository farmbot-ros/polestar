#include <cmath>
#include <rclcpp/logging.hpp>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "farmbot_interfaces/msg/float32_stamped.hpp"
#include "std_msgs/msg/float32.hpp"
#include "farmbot_interfaces/srv/datum.hpp"
#include "farmbot_interfaces/srv/trigger.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

double toRadians(double degrees) {
    auto rads = std::fmod(degrees * M_PI / 180.0, 360.0);
    return rads;
}

double toDegrees(double radians) {
    auto degs = std::fmod(radians * 180.0 / M_PI, 360.0);
    return degs;
}

class GpsAndDEg : public rclcpp::Node {
    private:
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
        GpsAndDEg() : Node(
            "fix_n_bearing",
            rclcpp::NodeOptions()
            .allow_undeclared_parameters(true)
            .automatically_declare_parameters_from_overrides(true)
        ){
            RCLCPP_INFO(this->get_logger(), "Starting GPS & DEG Node");

            fix_topic = this->get_parameter_or<std::string>("fix", "gnss/fix");
            orientation_topic = this->get_parameter_or<std::string>("orientation", "gnss/heading");
            units = this->get_parameter_or<std::string>("units", "degrees");

            RCLCPP_INFO(this->get_logger(), "Subscribing to %s and %s (%s)", fix_topic.c_str(), orientation_topic.c_str(), units.c_str());

            gps_corr_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(fix_topic, 10, std::bind(&GpsAndDEg::fix_callback, this, std::placeholders::_1));
            head_sub_ = this->create_subscription<std_msgs::msg::Float32>(orientation_topic, 10, std::bind(&GpsAndDEg::orientation_callback, this, std::placeholders::_1));
            timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&GpsAndDEg::timer_callback, this));
            gps_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("loc/fix", 10);
            deg_ = this->create_publisher<farmbot_interfaces::msg::Float32Stamped>("loc/deg", 10);
            rad_ = this->create_publisher<farmbot_interfaces::msg::Float32Stamped>("loc/rad", 10);

            frame_id = this->get_namespace();
            if (!frame_id.empty() && frame_id[0] == '/') {
                frame_id = frame_id.substr(1); // Remove leading slash
            }
            frame_id += "/gps";

        }

    private:

        void fix_callback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr& fix_topic_msg) {
            curr_gps = *fix_topic_msg;
        }

        void orientation_callback(const std_msgs::msg::Float32::ConstSharedPtr& orientation_topic_msg) {
            heading = *orientation_topic_msg;
        }


        void timer_callback() {
            sensor_msgs::msg::NavSatFix curr_pose;
            curr_pose = curr_gps;
            curr_pose.header.frame_id = frame_id;
            curr_pose.header.stamp = this->now();
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
                RCLCPP_WARN(this->get_logger(), "Invalid units parameter: %s", units.c_str());
                return;
            }
            deg_->publish(deg_msg);
            rad_->publish(rad_msg);
        }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto navfix = std::make_shared<GpsAndDEg>();
    rclcpp::spin(navfix);
    rclcpp::shutdown();
    return 0;
}
