#include <cmath>
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
    return -rads;
}

class GpsAndDEg : public rclcpp::Node {
    private:
        sensor_msgs::msg::NavSatFix curr_gps;
        std_msgs::msg::Float32 heading;
        std::string gps_corr_topic;
        std::string heading_topic;

        std::string name;
        std::string frame_id;

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_corr_;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr head_pub_;
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
            try {
                name = this->get_parameter("name").as_string();
            } catch (...) {
                name = "fix_n_bearing";
            }

            //try to get the parameters of gps_corr and heading topics
            try{
                rclcpp::Parameter gps_corr_param = this->get_parameter("gps_corr");
                gps_corr_topic = gps_corr_param.as_string();
                rclcpp::Parameter angle_gpses_param = this->get_parameter("heading");
                heading_topic = angle_gpses_param.as_string();
            } catch(const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "Could not find one of those parameters: gps_corr, heading");
                gps_corr_topic = "gps_corr";
                heading_topic = "heading";
            }
            RCLCPP_INFO(this->get_logger(), "Subscribing to %s and %s", gps_corr_topic.c_str(), heading_topic.c_str());

            gps_corr_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(gps_corr_topic, 10, std::bind(&GpsAndDEg::gps_corr_callback, this, std::placeholders::_1));
            head_pub_ = this->create_subscription<std_msgs::msg::Float32>(heading_topic, 10, std::bind(&GpsAndDEg::angle_deg_callback, this, std::placeholders::_1));
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

        void gps_corr_callback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr& gps_corr_msg) {
            curr_gps = *gps_corr_msg;
        }

        void angle_deg_callback(const std_msgs::msg::Float32::ConstSharedPtr& angle_deg_msg) {
            heading = *angle_deg_msg;
        }

        void timer_callback() {
            sensor_msgs::msg::NavSatFix curr_pose;
            curr_pose = curr_gps;
            curr_pose.header.frame_id = frame_id;
            curr_pose.header.stamp = this->now();
            gps_pub_->publish(curr_pose);
            farmbot_interfaces::msg::Float32Stamped deg_msg;
            deg_msg.header = curr_pose.header;
            deg_msg.data = heading.data;
            deg_->publish(deg_msg);
            farmbot_interfaces::msg::Float32Stamped rad_msg;
            rad_msg.header = curr_pose.header;
            rad_msg.data = toRadians(heading.data);
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
