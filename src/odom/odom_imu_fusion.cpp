#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <eigen3/Eigen/Dense>

using std::placeholders::_1;
using std::placeholders::_2;

class OdomImuFusion {
  private:
    rclcpp::Node::SharedPtr node_;
    // Subscribers for wheel odom and IMU
    message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Imu> imu_sub_;
    std::shared_ptr<message_filters::Synchronizer<
        message_filters::sync_policies::ApproximateTime<nav_msgs::msg::Odometry, sensor_msgs::msg::Imu>>>
        sync_;

    // Publisher for fused odometry
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr fused_odom_pub_;

    // EKF state & covariance (x = [x, y, theta, v])
    Eigen::Vector4d x_;
    Eigen::Matrix4d P_;
    Eigen::Matrix4d Q_; // process noise
    double R_imu_;      // IMU heading noise

    rclcpp::Time last_time_;

  public:
    OdomImuFusion(rclcpp::Node::SharedPtr node) : node_(node) {
        RCLCPP_INFO(node_->get_logger(), "Starting Odom-IMU Fusion Node");

        // Publisher
        fused_odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("fused/odom", 10);

        // Initialize EKF parameters
        x_.setZero();
        P_.setIdentity();
        Q_.setIdentity();
        Q_ *= 0.1;     // tune as needed
        R_imu_ = 0.01; // tune as needed

        last_time_ = node_->now();

        // Time synchronizer for odometry + IMU
        using namespace message_filters::sync_policies;
        sync_.reset(
            new message_filters::Synchronizer<ApproximateTime<nav_msgs::msg::Odometry, sensor_msgs::msg::Imu>>(10));
        sync_->connectInput(odom_sub_, imu_sub_);
        sync_->registerCallback(std::bind(&OdomImuFusion::callback, this, _1, _2));
    }

  private:
    void callback(const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg,
                  const sensor_msgs::msg::Imu::ConstSharedPtr &imu_msg) {
        // Compute dt
        rclcpp::Time now = odom_msg->header.stamp;
        double dt = (now - last_time_).seconds();
        if (dt <= 0) dt = 1e-3;

        // 1) Predict step using wheel odometry
        double v = odom_msg->twist.twist.linear.x;
        double omega = odom_msg->twist.twist.angular.z;
        double theta = x_(2);
        // State prediction
        x_(0) += v * std::cos(theta) * dt;
        x_(1) += v * std::sin(theta) * dt;
        x_(2) += omega * dt;
        x_(3) = v;
        // Jacobian F
        Eigen::Matrix4d F = Eigen::Matrix4d::Identity();
        F(0, 2) = -v * std::sin(theta) * dt;
        F(0, 3) = std::cos(theta) * dt;
        F(1, 2) = v * std::cos(theta) * dt;
        F(1, 3) = std::sin(theta) * dt;
        // Covariance prediction
        P_ = F * P_ * F.transpose() + Q_;

        // 2) Update step using IMU yaw
        // Extract yaw from IMU quaternion
        tf2::Quaternion q(imu_msg->orientation.x, imu_msg->orientation.y, imu_msg->orientation.z,
                          imu_msg->orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        // Measurement model h(x) = theta
        double y_res = normalizeAngle(yaw - x_(2));
        Eigen::RowVector4d H;
        H << 0, 0, 1, 0;
        double S = (H * P_ * H.transpose())(0) + R_imu_;
        Eigen::Vector4d K = P_ * H.transpose() / S;
        // State update
        x_ += K * y_res;
        // Covariance update
        P_ = (Eigen::Matrix4d::Identity() - K * H) * P_;

        last_time_ = now;

        // 3) Publish fused odometry
        publishFused(odom_msg->header.stamp);
    }

    void publishFused(const rclcpp::Time &stamp) {
        nav_msgs::msg::Odometry out;
        out.header.stamp = stamp;
        out.header.frame_id = "map";
        out.child_frame_id = "odom";
        // Position
        out.pose.pose.position.x = x_(0);
        out.pose.pose.position.y = x_(1);
        out.pose.pose.position.z = 0.0;
        // Orientation from yaw
        tf2::Quaternion q;
        q.setRPY(0, 0, x_(2));
        out.pose.pose.orientation.x = q.x();
        out.pose.pose.orientation.y = q.y();
        out.pose.pose.orientation.z = q.z();
        out.pose.pose.orientation.w = q.w();
        // Velocity
        out.twist.twist.linear.x = x_(3);
        out.twist.twist.angular.z = 0.0;
        // Covariance (map 4x4 into 6x6 pose covariance)
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                out.pose.covariance[i * 6 + j] = P_(i, j);
            }
        }
        fused_odom_pub_->publish(out);
    }

    double normalizeAngle(double angle) {
        while (angle > M_PI)
            angle -= 2.0 * M_PI;
        while (angle < -M_PI)
            angle += 2.0 * M_PI;
        return angle;
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);
    options.automatically_declare_parameters_from_overrides(true);

    rclcpp::Node::SharedPtr node1 = rclcpp::Node::make_shared("odom_imu_fusion", options);
    std::shared_ptr<OdomImuFusion> taskerrr = std::make_shared<OdomImuFusion>(node1);

    try {
        executor.add_node(node1);
        executor.spin();
    } catch (const std::exception &e) {
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
