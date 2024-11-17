#include <cmath>
#include <rclcpp/logging.hpp>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

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
        nav_msgs::msg::Odometry ecef_msg;
        bool is_transformed = false;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ecef_sub_;
        rclcpp::TimerBase::SharedPtr transform_timer_;

        std::string name;
        std::string transform_from_;
        std::string namespace_;
        std::string namespace_2;
        bool altitude;

        std::unique_ptr<tf2_ros::TransformBroadcaster> base_tf;
        std::unique_ptr<tf2_ros::StaticTransformBroadcaster> odom_tf;
        std::unique_ptr<tf2_ros::StaticTransformBroadcaster> map_tf;

    public:
        TransformPub() : Node(
            "transform_pub",
            rclcpp::NodeOptions()
            .allow_undeclared_parameters(true)
            .automatically_declare_parameters_from_overrides(true)
        ){
            RCLCPP_INFO(this->get_logger(), "Starting TransformPub node");

            try {
                name = this->get_parameter("name").as_string();

            } catch (...) {
                name = "transform_pub";
            }

            altitude = this->get_parameter_or<bool>("altitude", false);

            namespace_ = this->get_namespace();
            if (!namespace_.empty() && namespace_[0] == '/') {
                namespace_ = namespace_.substr(1); // Remove leading slash
            }
            namespace_2 = namespace_;


            odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("loc/odom", 10, std::bind(&TransformPub::base_transform, this, std::placeholders::_1));
            ecef_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("loc/ref", 10, std::bind(&TransformPub::ecef_callback, this, std::placeholders::_1));
            transform_timer_ = this->create_wall_timer(std::chrono::milliseconds(10000), std::bind(&TransformPub::ecef_timer, this));

            base_tf = std::make_unique<tf2_ros::TransformBroadcaster>(this);
            odom_tf = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
            map_tf = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
            RCLCPP_INFO(this->get_logger(), "--------------------------------------------------");
            RCLCPP_INFO(this->get_logger(), "NAMESPACE: %s", namespace_.c_str());
            RCLCPP_INFO(this->get_logger(), "--------------------------------------------------");
        }

    private:
        void ecef_callback(const nav_msgs::msg::Odometry::ConstSharedPtr& odom) {
            ecef_msg = *odom;
            map_transform();
            odom_transform();
            is_transformed = true;
            RCLCPP_INFO(this->get_logger(), "TRANSFORMS SET");
            ecef_sub_.reset();
            RCLCPP_INFO(this->get_logger(), "SUBSCRIPTION CLOSED");
        }

        void ecef_timer() {
            if (is_transformed) {
                map_transform();
                odom_transform();
                return;
            }
            // RCLCPP_INFO(this->get_logger(), "WAITING FOR TRANSFORM, PLEASE SET DATUM FIRST!");
        }

        void base_transform(const nav_msgs::msg::Odometry::ConstSharedPtr& odom) {
            geometry_msgs::msg::TransformStamped dyna_t;
            dyna_t.header.stamp = odom->header.stamp;
            dyna_t.header.frame_id = namespace_ + "/odom";
            dyna_t.child_frame_id = namespace_ + "/base_link";
            dyna_t.transform.translation.x = odom->pose.pose.position.x;
            dyna_t.transform.translation.y = odom->pose.pose.position.y;
            dyna_t.transform.translation.z = odom->pose.pose.position.z;
            if (!altitude) {
                dyna_t.transform.translation.z = 0.0;
            }
            dyna_t.transform.rotation.x = odom->pose.pose.orientation.x;
            dyna_t.transform.rotation.y = odom->pose.pose.orientation.y;
            dyna_t.transform.rotation.z = odom->pose.pose.orientation.z;
            dyna_t.transform.rotation.w = odom->pose.pose.orientation.w;
            base_tf->sendTransform(dyna_t);
        }

        void odom_transform() {
            geometry_msgs::msg::TransformStamped stat_t;
            stat_t.header.stamp = this->get_clock()->now();
            stat_t.header.frame_id = namespace_2 + "/map";
            stat_t.child_frame_id = namespace_ + "/odom";
            stat_t.transform.translation.x = 0.0;
            stat_t.transform.translation.y = 0.0;
            stat_t.transform.translation.z = 0.0;
            stat_t.transform.rotation.x = 0.0;
            stat_t.transform.rotation.y = 0.0;
            stat_t.transform.rotation.z = 0.0;
            stat_t.transform.rotation.w = 1.0;
            odom_tf->sendTransform(stat_t);
        }

        void map_transform() {
            auto x = removeFirstFourDigits(ecef_msg.pose.pose.position.x);
            auto y = removeFirstFourDigits(ecef_msg.pose.pose.position.y);
            auto z = removeFirstFourDigits(ecef_msg.pose.pose.position.z);
            // RCLCPP_INFO(this->get_logger(), "ECEF: %f, %f, %f", x, y, z);
            geometry_msgs::msg::TransformStamped stat_t;
            stat_t.header.stamp = this->get_clock()->now();
            stat_t.header.frame_id = "/world";
            stat_t.child_frame_id = namespace_2 + "/map";
            stat_t.transform.translation.x = x;
            stat_t.transform.translation.y = y;
            stat_t.transform.translation.z = z;
            if (!altitude){
                stat_t.transform.translation.z = 0.0;
            }
            stat_t.transform.rotation.x = 0.0;
            stat_t.transform.rotation.y = 0.0;
            stat_t.transform.rotation.z = 0.0;
            stat_t.transform.rotation.w = 1.0;
            map_tf->sendTransform(stat_t);
        }


};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto navfix = std::make_shared<TransformPub>();
    rclcpp::spin(navfix);
    rclcpp::shutdown();
    return 0;
}
