#include "farmbot_interfaces/msg/float32_stamped.hpp"
#include "farmbot_interfaces/srv/datum.hpp"
#include "farmbot_interfaces/srv/trigger.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/time_synchronizer.h"
#include <rclcpp/logging.hpp>

std::array<double, 4> theta_to_quaternion(double theta) {
    // rotate around z axis for 90 degrees
    return {std::cos(theta / 2), 0, 0, std::sin(theta / 2)};
}

class OdomNPath {
  private:
    rclcpp::Node::SharedPtr node_;
    std::string namespace_;

    nav_msgs::msg::Odometry enu_odom;
    nav_msgs::msg::Odometry odom_map;
    nav_msgs::msg::Path path;
    nav_msgs::msg::Path footprint;
    bool reset_path = false;
    geometry_msgs::msg::PoseStamped prev_point_path;
    geometry_msgs::msg::PoseStamped prev_point_dist;
    farmbot_interfaces::msg::Float32Stamped cumulative_dist;

    std::string frame_id;
    float distance;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<farmbot_interfaces::msg::Float32Stamped>::SharedPtr dist_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr footprint_pub_;
    rclcpp::TimerBase::SharedPtr timer_pub_;
    rclcpp::Service<farmbot_interfaces::srv::Trigger>::SharedPtr dist_reset;
    rclcpp::Service<farmbot_interfaces::srv::Trigger>::SharedPtr path_reset;

    message_filters::Subscriber<nav_msgs::msg::Odometry> enu_sub_;
    message_filters::Subscriber<farmbot_interfaces::msg::Float32Stamped> rad_sub_;
    std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<
        nav_msgs::msg::Odometry, farmbot_interfaces::msg::Float32Stamped>>>
        sync_;

  public:
    OdomNPath(rclcpp::Node::SharedPtr node) : node_(node) {
        RCLCPP_INFO(node_->get_logger(), "Starting Odom&Path");
        namespace_ = node_->get_namespace();
        if (!namespace_.empty() && namespace_[0] == '/') {
            namespace_ = namespace_.substr(1); // Remove leading slash
        }
        frame_id = namespace_ + "/map";

        odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("loc/odom", 10);
        dist_pub_ = node_->create_publisher<farmbot_interfaces::msg::Float32Stamped>("loc/dist", 10);
        path_pub_ = node_->create_publisher<nav_msgs::msg::Path>("loc/path", 10);
        footprint_pub_ = node_->create_publisher<nav_msgs::msg::Path>("loc/footprint", 10);

        timer_pub_ = node_->create_wall_timer(std::chrono::milliseconds(100), [this]() {
            odom_pub_->publish(enu_odom);
            dist_pub_->publish(cumulative_dist);
            path_pub_->publish(path);
            footprint_pub_->publish(footprint);
        });

        dist_reset = node_->create_service<farmbot_interfaces::srv::Trigger>(
            "loc/dist_reset", [this](const std::shared_ptr<farmbot_interfaces::srv::Trigger::Request> _request,
                                     std::shared_ptr<farmbot_interfaces::srv::Trigger::Response> _response) {
                cumulative_dist.data = 0;
                auto req = _request;  // TODO: fix, this is a hack to get rid of unused variable warning
                auto res = _response; // TODO: fix, this is a hack to get rid of unused variable warning
                return;
            });
        path_reset = node_->create_service<farmbot_interfaces::srv::Trigger>(
            "loc/path_reset", [this](const std::shared_ptr<farmbot_interfaces::srv::Trigger::Request> _request,
                                     std::shared_ptr<farmbot_interfaces::srv::Trigger::Response> _response) {
                path.poses.clear();
                footprint.poses.clear();
                auto req = _request;  // TODO: fix, this is a hack to get rid of unused variable warning
                auto res = _response; // TODO: fix, this is a hack to get rid of unused variable warning
                return;
            });

        enu_sub_.subscribe(node_, "loc/enu");
        rad_sub_.subscribe(node_, "loc/rad");
        sync_ = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<
            nav_msgs::msg::Odometry, farmbot_interfaces::msg::Float32Stamped>>>(10);
        sync_->connectInput(enu_sub_, rad_sub_);
        sync_->registerCallback(std::bind(&OdomNPath::callback, this, std::placeholders::_1, std::placeholders::_2));
    }

  private:
    void callback(const nav_msgs::msg::Odometry::ConstSharedPtr &enu_msg,
                  const farmbot_interfaces::msg::Float32Stamped::ConstSharedPtr &rad_msg) {
        enu_odom = *enu_msg;
        enu_odom.header.frame_id = frame_id;
        std::array<double, 4> quaternions = theta_to_quaternion(rad_msg->data);
        enu_odom.pose.pose.orientation.w = quaternions[0];
        enu_odom.pose.pose.orientation.x = quaternions[1];
        enu_odom.pose.pose.orientation.y = quaternions[2];
        enu_odom.pose.pose.orientation.z = quaternions[3];

        geometry_msgs::msg::PoseStamped pose;
        pose.pose = enu_odom.pose.pose;

        // create path
        path.header.frame_id = frame_id;
        footprint.header.frame_id = frame_id;
        cumulative_dist.header.frame_id = frame_id;
        distance = point_distance(prev_point_dist, pose);
        if (distance > 0.1) {
            prev_point_dist = pose;
            cumulative_dist.data += distance;
            if (distance < 1) {
                pose.header.frame_id = frame_id;
                path.poses.push_back(pose);
                pose.pose.position.z = 0;
                footprint.poses.push_back(pose);
            }
        }
        if (path.poses.size() > 100 && reset_path) {
            path.poses.erase(path.poses.begin());
            footprint.poses.erase(footprint.poses.begin());
        }
    }

    float point_distance(geometry_msgs::msg::PoseStamped p1, geometry_msgs::msg::PoseStamped p2) {
        return std::sqrt(std::pow(p1.pose.position.x - p2.pose.position.x, 2) +
                         std::pow(p1.pose.position.y - p2.pose.position.y, 2) +
                         std::pow(p1.pose.position.z - p2.pose.position.z, 2));
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);
    options.automatically_declare_parameters_from_overrides(true);

    rclcpp::Node::SharedPtr node1 = rclcpp::Node::make_shared("odom_n_path", options);
    std::shared_ptr<OdomNPath> taskerrr = std::make_shared<OdomNPath>(node1);

    try {
        executor.add_node(node1);
        executor.spin();
    } catch (const std::exception &e) {
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
