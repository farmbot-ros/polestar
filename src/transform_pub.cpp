#include "diagnostic_updater/diagnostic_updater.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include <cmath>
#include <rclcpp/logging.hpp>

using namespace std::chrono_literals;

float removeFirstFourDigits(float number, unsigned long digits = 5) {
    std::string numberStr = std::to_string(number);
    if (numberStr.length() > digits + 2) {
        numberStr = numberStr.substr(digits);
        return std::stof(numberStr);
    } else {
        return number;
    }
}

class TransformPub {
  private:
    rclcpp::Node::SharedPtr node_;
    std::string namespace_;
    std::string world_ref_;
    bool altitude;

    bool world_to_map;
    bool map_to_odom;
    bool odom_to_basef;
    bool basef_to_basel;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr to_map_, to_base_, to_foot_, to_odom_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> base_tf, foot_tf;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> odom_tf, map_tf;

    // Diagnostic Updater
    diagnostic_updater::Updater updater_;
    rclcpp::TimerBase::SharedPtr diagnostic_timer_;

  public:
    TransformPub(rclcpp::Node::SharedPtr node) : node_(node), updater_(node) {
        RCLCPP_INFO(node_->get_logger(), "Starting TransformPub node");

        world_ref_ = node_->get_parameter_or<std::string>("world_ref", "datum");
        altitude = node_->get_parameter_or<bool>("altitude", false);

        namespace_ = node_->get_namespace();
        if (!namespace_.empty() && namespace_[0] == '/') {
            namespace_ = namespace_.substr(1);
        }

        // Diagnostic Updater
        updater_.setHardwareID(static_cast<std::string>(node_->get_namespace()) + "loc");
        updater_.add("Transformation Status", this, &TransformPub::check_system);
        diagnostic_timer_ = node_->create_wall_timer(1s, std::bind(&TransformPub::diagnostic_callback, this));

        base_tf = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
        foot_tf = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
        odom_tf = std::make_unique<tf2_ros::StaticTransformBroadcaster>(node_);
        map_tf = std::make_unique<tf2_ros::StaticTransformBroadcaster>(node_);

        to_base_ = node_->create_subscription<nav_msgs::msg::Odometry>(
            "loc/odom", 10, [this](const nav_msgs::msg::Odometry::ConstSharedPtr &odom) { base_2_foot(odom); });
        to_foot_ = node_->create_subscription<nav_msgs::msg::Odometry>(
            "loc/odom", 10, [this](const nav_msgs::msg::Odometry::ConstSharedPtr &odom) { foot_2_odom(odom); });
        to_odom_ = node_->create_subscription<nav_msgs::msg::Odometry>(
            "loc/odom", 10, [this](const nav_msgs::msg::Odometry::ConstSharedPtr &odom) { odom_2_map(odom); });
        to_map_ = node_->create_subscription<nav_msgs::msg::Odometry>(
            "loc/ref", 10, [this](const nav_msgs::msg::Odometry::ConstSharedPtr &ecef) { map_2_world(ecef); });
    }

  private:
    void diagnostic_callback() { updater_.force_update(); }

    void check_system(diagnostic_updater::DiagnosticStatusWrapper &stat) {
        if (world_to_map && map_to_odom && odom_to_basef && basef_to_basel) {
            stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Transforms are A OK!");
        } else {
            stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Transforms are not OK!");
        }
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Transforms are A OK!");
    }

    void base_2_foot(const nav_msgs::msg::Odometry::ConstSharedPtr &odom) {
        geometry_msgs::msg::TransformStamped dyna_t;
        dyna_t.header.stamp = node_->get_clock()->now();
        dyna_t.child_frame_id = namespace_ + "/base_link";
        dyna_t.header.frame_id = namespace_ + "/base_footprint";
        dyna_t.transform.translation.x = 0.0;
        dyna_t.transform.translation.y = 0.0;
        dyna_t.transform.translation.z = odom->pose.pose.position.z;
        if (!altitude) {
            dyna_t.transform.translation.z = 0.0;
        }
        dyna_t.transform.rotation.x = 0.0;
        dyna_t.transform.rotation.y = 0.0;
        dyna_t.transform.rotation.z = 0.0;
        dyna_t.transform.rotation.w = 1.0;
        foot_tf->sendTransform(dyna_t);
        odom_to_basef = true;
    }
    void foot_2_odom(const nav_msgs::msg::Odometry::ConstSharedPtr &odom) {
        geometry_msgs::msg::TransformStamped dyna_t;
        dyna_t.header.stamp = odom->header.stamp;
        dyna_t.child_frame_id = namespace_ + "/base_footprint";
        dyna_t.header.frame_id = namespace_ + "/odom";
        dyna_t.transform.translation.x = odom->pose.pose.position.x;
        dyna_t.transform.translation.y = odom->pose.pose.position.y;
        dyna_t.transform.translation.z = 0.0;
        dyna_t.transform.rotation.x = odom->pose.pose.orientation.x;
        dyna_t.transform.rotation.y = odom->pose.pose.orientation.y;
        dyna_t.transform.rotation.z = odom->pose.pose.orientation.z;
        dyna_t.transform.rotation.w = odom->pose.pose.orientation.w;
        base_tf->sendTransform(dyna_t);
        basef_to_basel = true;
    }

    void odom_2_map(const nav_msgs::msg::Odometry::ConstSharedPtr &odom) {
        geometry_msgs::msg::TransformStamped stat_t;
        stat_t.header.stamp = node_->get_clock()->now();
        stat_t.child_frame_id = namespace_ + "/odom";
        stat_t.header.frame_id = namespace_ + "/map";
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

    void map_2_world(const nav_msgs::msg::Odometry::ConstSharedPtr &ecef) {
        auto x = world_ref_ == "datum" ? 0.0 : ecef->pose.pose.position.x;
        auto y = world_ref_ == "datum" ? 0.0 : ecef->pose.pose.position.y;
        geometry_msgs::msg::TransformStamped stat_t;
        stat_t.header.stamp = node_->get_clock()->now();
        stat_t.child_frame_id = namespace_ + "/map";
        stat_t.header.frame_id = "world";
        stat_t.transform.translation.x = x;
        stat_t.transform.translation.y = y;
        stat_t.transform.translation.z = 0.0;
        stat_t.transform.rotation.x = 0.0;
        stat_t.transform.rotation.y = 0.0;
        stat_t.transform.rotation.z = 0.0;
        stat_t.transform.rotation.w = 1.0;
        map_tf->sendTransform(stat_t);
        world_to_map = true;
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);
    options.automatically_declare_parameters_from_overrides(true);

    rclcpp::Node::SharedPtr node1 = rclcpp::Node::make_shared("transform_pub", options);
    std::shared_ptr<TransformPub> taskerrr = std::make_shared<TransformPub>(node1);

    try {
        executor.add_node(node1);
        executor.spin();
    } catch (const std::exception &e) {
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
