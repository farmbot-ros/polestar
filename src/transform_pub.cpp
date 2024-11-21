#include <cmath>
#include <rclcpp/logging.hpp>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"

using namespace std::chrono_literals;

float removeFirstFourDigits(float number, unsigned long digits=5) {
    std::string numberStr = std::to_string(number);
    if (numberStr.length() > digits+2) {
        numberStr = numberStr.substr(digits); // Remove the first n digits
        // return std::stoi(numberStr); // Convert back to an integer
        return std::stof(numberStr);
    } else {
        // Handle case where number has fewer than n digits
        return number;
    }
}

class TransformPub : public rclcpp::Node {
    private:
        bool world_to_map;
        bool map_to_odom;
        bool odom_to_basef;
        bool basef_to_basel;

        nav_msgs::msg::Odometry ecef_msg;
        nav_msgs::msg::Odometry odom_msg;
        nav_msgs::msg::Odometry sens_msg;

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ecef_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sens_sub_;

        std::string namespace_;
        std::string world_ref_;
        bool altitude;

        std::unique_ptr<tf2_ros::TransformBroadcaster> base_tf;
        std::unique_ptr<tf2_ros::TransformBroadcaster> foot_tf;
        std::unique_ptr<tf2_ros::StaticTransformBroadcaster> odom_tf;
        std::unique_ptr<tf2_ros::StaticTransformBroadcaster> map_tf;

        //Diagnostic Updater
        diagnostic_updater::Updater updater_;
        rclcpp::TimerBase::SharedPtr diagnostic_timer_;

    public:
        TransformPub() : Node(
            "transform_pub",
            rclcpp::NodeOptions()
            .allow_undeclared_parameters(true)
            .automatically_declare_parameters_from_overrides(true)
        ), updater_(this) {
            RCLCPP_INFO(this->get_logger(), "Starting TransformPub node");

            world_ref_ = this->get_parameter_or<std::string>("world_ref", "datum");
            altitude = this->get_parameter_or<bool>("altitude", false);


            namespace_ = this->get_namespace();
            if (!namespace_.empty() && namespace_[0] == '/') {
                namespace_ = namespace_.substr(1);
            }


            odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("loc/odom", 10, std::bind(&TransformPub::footprint_transform, this, std::placeholders::_1));
            ecef_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("loc/ref", 10, std::bind(&TransformPub::ecef_callback, this, std::placeholders::_1));
            // sens_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("loc/sens", 10, std::bind(&TransformPub::odom_transform, this, std::placeholders::_1));

            base_tf = std::make_unique<tf2_ros::TransformBroadcaster>(this);
            foot_tf = std::make_unique<tf2_ros::TransformBroadcaster>(this);
            odom_tf = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
            map_tf = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);

            // Diagnostic Updater
            updater_.setHardwareID(static_cast<std::string>(this->get_namespace()) + "/loc");
            updater_.add("Transformation Status", this, &TransformPub::check_system);
            diagnostic_timer_ = this->create_wall_timer(1s, std::bind(&TransformPub::diagnostic_callback, this));
        }

    private:

        void diagnostic_callback() {
            updater_.force_update();
        }

        void check_system(diagnostic_updater::DiagnosticStatusWrapper &stat) {
            if (world_to_map && map_to_odom && odom_to_basef && basef_to_basel) {
                stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Transforms are A OK!");
            } else {
                stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Transforms are not OK!");
            }
            stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Transforms are A OK!");
        }

        void footprint_transform(const nav_msgs::msg::Odometry::ConstSharedPtr& odom) {
            odom_msg = *odom;
            geometry_msgs::msg::TransformStamped dyna_t;
            dyna_t.header.stamp = odom_msg.header.stamp;
            dyna_t.header.frame_id = namespace_ + "/odom";
            dyna_t.child_frame_id = namespace_ + "/base_footprint";
            dyna_t.transform.translation.x = odom_msg.pose.pose.position.x;
            dyna_t.transform.translation.y = odom_msg.pose.pose.position.y;
            dyna_t.transform.translation.z = 0.0;
            dyna_t.transform.rotation.x = odom_msg.pose.pose.orientation.x;
            dyna_t.transform.rotation.y = odom_msg.pose.pose.orientation.y;
            dyna_t.transform.rotation.z = odom_msg.pose.pose.orientation.z;
            dyna_t.transform.rotation.w = odom_msg.pose.pose.orientation.w;
            base_tf->sendTransform(dyna_t);
            basef_to_basel = true;
            baselink_transform(odom);
            odom_transform(odom);
        }

        void baselink_transform(const nav_msgs::msg::Odometry::ConstSharedPtr& odom) {
            geometry_msgs::msg::TransformStamped foot_t;
            foot_t.header.stamp = this->get_clock()->now();
            foot_t.header.frame_id = namespace_ + "/base_footprint";
            foot_t.child_frame_id = namespace_ + "/base_link";
            foot_t.transform.translation.x = 0.0;
            foot_t.transform.translation.y = 0.0;
            foot_t.transform.translation.z = odom->pose.pose.position.z;
            if (!altitude){
                foot_t.transform.translation.z = 0.0;
            }
            foot_t.transform.rotation.x = 0.0;
            foot_t.transform.rotation.y = 0.0;
            foot_t.transform.rotation.z = 0.0;
            foot_t.transform.rotation.w = 1.0;
            foot_tf->sendTransform(foot_t);
            odom_to_basef = true;
        }

        void odom_transform(const nav_msgs::msg::Odometry::ConstSharedPtr& sens) {
            sens_msg = *sens;
            geometry_msgs::msg::TransformStamped stat_t;
            stat_t.header.stamp = this->get_clock()->now();
            stat_t.header.frame_id = namespace_ + "/map";
            stat_t.child_frame_id = namespace_ + "/odom";
            stat_t.transform.translation.x = 0.0;
            stat_t.transform.translation.y = 0.0;
            stat_t.transform.translation.z = 0.0;
            stat_t.transform.rotation.x = 0.0;
            stat_t.transform.rotation.y = 0.0;
            stat_t.transform.rotation.z = 0.0;
            stat_t.transform.rotation.w = 1.0;
            odom_tf->sendTransform(stat_t);
            map_to_odom = true;
        }

        // void map_transform() {
        void ecef_callback(const nav_msgs::msg::Odometry::ConstSharedPtr& ecef) {
            ecef_msg = *ecef;
            auto x = world_ref_ == "datum" ? 0.0 : ecef_msg.pose.pose.position.x;
            auto y = world_ref_ == "datum" ? 0.0 : ecef_msg.pose.pose.position.y;
            geometry_msgs::msg::TransformStamped stat_t;
            stat_t.header.stamp = this->get_clock()->now();
            stat_t.header.frame_id = "world";
            stat_t.child_frame_id = namespace_ + "/map";
            stat_t.transform.translation.x = x;
            stat_t.transform.translation.y = y;
            stat_t.transform.translation.z = 0.0;
            stat_t.transform.rotation.x = 0.0;
            stat_t.transform.rotation.y = 0.0;
            stat_t.transform.rotation.z = 0.0;
            stat_t.transform.rotation.w = 1.0;
            map_tf->sendTransform(stat_t);
            world_to_map = true;
            ecef_sub_.reset();
        }


};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto navfix = std::make_shared<TransformPub>();
    rclcpp::spin(navfix);
    rclcpp::shutdown();
    return 0;
}
