#include <cmath>
#include <rclcpp/logging.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "farmbot_interfaces/msg/float32_stamped.hpp"
#include "farmbot_interfaces/srv/datum.hpp"
#include "farmbot_interfaces/srv/trigger.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

// Function to convert degrees to radians
double toRadians(double degrees) {
    return degrees * M_PI / 180.0;
}

// Function to convert radians to degrees
double toDegrees(double radians) {
    return radians * 180.0 / M_PI;
}

// Function to calculate the bearing between two GPS coordinates with an optional angle offset
std::pair<float, float> calc_bearing(double lat1_in, double long1_in, double lat2_in, double long2_in, double angle_gpses = 0.0) {
    // Convert latitude and longitude to radians
    double lat1 = toRadians(lat1_in);
    double long1 = toRadians(long1_in);
    double lat2 = toRadians(lat2_in);
    double long2 = toRadians(long2_in);

    // Calculate the bearing in radians
    double bearing_rad = atan2(
        sin(long2 - long1) * cos(lat2),
        cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(long2 - long1)
    );
    // Add 90 degrees from the bearing to make it relative to the y-axis
    //bearing_rad += M_PI / 2;

    // Add the angle-offset (convert it to radians first)
    bearing_rad += toRadians(angle_gpses);

    // Normalize bearing to be within 0 and 2pi
    bearing_rad = fmod(bearing_rad + 2 * M_PI, 2 * M_PI);

    // Return the bearing in both degrees and radians
    return std::make_pair(static_cast<float>(toDegrees(bearing_rad)), static_cast<float>(bearing_rad));
}

class AntennaFuse : public rclcpp::Node {
    private:
        sensor_msgs::msg::NavSatFix curr_pose;
        double angle_gpses = 0.0;
        std::string gps_main_topic;
        std::string gps_aux_topic;

        std::string name;
        std::string frame_id;

        message_filters::Subscriber<sensor_msgs::msg::NavSatFix> gps_main_;
        message_filters::Subscriber<sensor_msgs::msg::NavSatFix> gps_aux_;
        std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::NavSatFix, sensor_msgs::msg::NavSatFix>>> sync_;

        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;
        rclcpp::Publisher<farmbot_interfaces::msg::Float32Stamped>::SharedPtr deg_;
        rclcpp::Publisher<farmbot_interfaces::msg::Float32Stamped>::SharedPtr rad_;
        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr dat_pub_;

        rclcpp::Service<farmbot_interfaces::srv::Datum>::SharedPtr datum_gps_;
        rclcpp::Service<farmbot_interfaces::srv::Trigger>::SharedPtr datum_set_;

    public:
        AntennaFuse() : Node(
            "dual_antenna",
            rclcpp::NodeOptions()
            .allow_undeclared_parameters(true)
            .automatically_declare_parameters_from_overrides(true)
        ){
            RCLCPP_INFO(this->get_logger(), "Starting GPS Fuser");

            try {
                name = this->get_parameter("name").as_string();
            } catch (...) {
                name = "dual_antenna";
            }

            //try to get the parameters of gps_main, gps_aux and angle_gpses topics
            try{
                rclcpp::Parameter gps_main_param = this->get_parameter("gps_main");
                gps_main_topic = gps_main_param.as_string();
                rclcpp::Parameter gps_aux_param = this->get_parameter("gps_aux");
                gps_aux_topic = gps_aux_param.as_string();
                rclcpp::Parameter angle_gpses_param = this->get_parameter("angle_gpses");
                angle_gpses = angle_gpses_param.as_double();
            } catch(const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "Could not find parameters: gps_main, gps_aux, angle_gpses, using defaults");
                gps_main_topic = "gps_main";
                gps_aux_topic = "gps_aux";
                angle_gpses = 0.0;
            }

            gps_main_.subscribe(this, gps_main_topic);
            gps_aux_.subscribe(this, gps_aux_topic);

            RCLCPP_INFO(this->get_logger(), "Subscribing to GPS topics: %s, %s and gps angle offset: %f", gps_main_topic.c_str(), gps_aux_topic.c_str(), angle_gpses);


            sync_ = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::NavSatFix, sensor_msgs::msg::NavSatFix>>>(10);
            sync_->connectInput(gps_main_, gps_aux_);
            sync_->registerCallback(std::bind(&AntennaFuse::callback, this, std::placeholders::_1, std::placeholders::_2));

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

        void callback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr& gps_main_msg, const sensor_msgs::msg::NavSatFix::ConstSharedPtr& gps_aux_msg) {
            RCLCPP_INFO_ONCE(this->get_logger(), "RECEIVED FIRST DATA ON BOTH TOPICS, STARTING FUSION");

            curr_pose = *gps_main_msg;
            curr_pose.header.frame_id = frame_id;

            farmbot_interfaces::msg::Float32Stamped deg_msg;
            farmbot_interfaces::msg::Float32Stamped rad_msg;
            deg_msg.header = curr_pose.header;
            rad_msg.header = curr_pose.header;
            auto [bearing_deg, bearing_rad] = calc_bearing(curr_pose.latitude, curr_pose.longitude, gps_aux_msg->latitude, gps_aux_msg->longitude, angle_gpses);
            deg_msg.data = bearing_deg;
            rad_msg.data = bearing_rad;

            gps_pub_->publish(curr_pose);
            deg_->publish(deg_msg);
            rad_->publish(rad_msg);
        }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto navfix = std::make_shared<AntennaFuse>();
    rclcpp::spin(navfix);
    rclcpp::shutdown();
    return 0;
}
