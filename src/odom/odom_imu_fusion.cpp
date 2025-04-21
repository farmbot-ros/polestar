// odom_imu_fusion.cpp
// -----------------------------------------------------------------------------
// Extended EKF ▸ 6‑state  [x y z θ v a]ᵀ   |   ROS 2 wheel‑odom + IMU fusion
// -----------------------------------------------------------------------------
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

namespace fusion {

    // ===========================================================================
    //                              EKF CLASS
    // ==========================================================================
    class EKF {
        // ---------- member variables ------------------------------------------------
      private:
        Eigen::Matrix<double, 6, 1> x_{}; // state [x y z θ v a]ᵀ
        Eigen::Matrix<double, 6, 6> P_{}; // covariance
        Eigen::Matrix<double, 6, 6> Q_{}; // process noise
        double R_yaw_{0.01}, R_acc_{0.20}, R_speed_{0.05}, R_z_{0.04};

        // ---------- public interface -----------------------------------------------
      public:
        EKF() { reset(); }

        void reset() {
            x_.setZero();
            P_.setIdentity();
            Q_.setZero();
            Q_.diagonal() << 0.05, 0.05, 0.05, 1e-3, 0.1, 0.5; // tune per robot
        }

        /** Predict `dt` seconds forward using IMU yaw‑rate `omega` (rad/s). */
        void predict(double omega, double dt) {
            if (dt <= 0.0) return;

            const double th = x_(3);
            const double v = x_(4);
            const double acc = x_(5);
            const double c = std::cos(th), s = std::sin(th), dt2 = dt * dt;

            // state propagation (constant forward acceleration)
            x_(0) += v * c * dt + 0.5 * acc * c * dt2; // x
            x_(1) += v * s * dt + 0.5 * acc * s * dt2; // y
            // z remains unchanged (index 2)
            x_(3) = normalize(th + omega * dt); // yaw
            x_(4) += acc * dt;                  // speed
            // a (index 5) assumed constant

            // Jacobian F (6×6)
            Eigen::Matrix<double, 6, 6> F = Eigen::Matrix<double, 6, 6>::Identity();
            F(0, 3) = -(v * s + 0.5 * acc * s * dt) * dt;
            F(0, 4) = c * dt;
            F(0, 5) = 0.5 * c * dt2;
            F(1, 3) = (v * c + 0.5 * acc * c * dt) * dt;
            F(1, 4) = s * dt;
            F(1, 5) = 0.5 * s * dt2;
            F(4, 5) = dt;

            P_ = F * P_ * F.transpose() + Q_;
        }

        // ---------------------- measurement updates ---------------------------
        void updateYaw(double yaw) {
            static const Eigen::Matrix<double, 1, 6> H = (Eigen::Matrix<double, 1, 6>() << 0, 0, 0, 1, 0, 0).finished();
            scalarUpdate(yaw, H, R_yaw_, true);
        }

        void updateAccel(double a) {
            static const Eigen::Matrix<double, 1, 6> H = (Eigen::Matrix<double, 1, 6>() << 0, 0, 0, 0, 0, 1).finished();
            scalarUpdate(a, H, R_acc_, false);
        }

        void updateSpeed(double v) {
            static const Eigen::Matrix<double, 1, 6> H = (Eigen::Matrix<double, 1, 6>() << 0, 0, 0, 0, 1, 0).finished();
            scalarUpdate(v, H, R_speed_, false);
        }

        void updateZ(double z) {
            static const Eigen::Matrix<double, 1, 6> H = (Eigen::Matrix<double, 1, 6>() << 0, 0, 1, 0, 0, 0).finished();
            scalarUpdate(z, H, R_z_, false);
        }

        // accessors --------------------------------------------------------------
        const Eigen::Matrix<double, 6, 1> &state() const { return x_; }
        const Eigen::Matrix<double, 6, 6> &covariance() const { return P_; }

        // ---------- private helper functions ---------------------------------------
      private:
        // 1‑D EKF correction
        void scalarUpdate(double z, const Eigen::Matrix<double, 1, 6> &H, double R, bool wrap_angle) {
            const double h_x = (H * x_)(0);
            const double y = wrap_angle ? normalize(z - h_x) : (z - h_x);
            const double S = (H * P_ * H.transpose())(0) + R;
            const Eigen::Matrix<double, 6, 1> K = P_ * H.transpose() / S;

            x_ += K * y;
            P_ = (Eigen::Matrix<double, 6, 6>::Identity() - K * H) * P_;
            x_(3) = normalize(x_(3)); // keep yaw wrapped
        }

        static double normalize(double a) { return std::atan2(std::sin(a), std::cos(a)); }
    };

} // namespace fusion

// ===========================================================================
//                             ROS 2 FUSION NODE
// ==========================================================================
class OdomImuFusion {
    // ---------- member variables ------------------------------------------------
  private:
    rclcpp::Node::SharedPtr node_;
    fusion::EKF ekf_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr fused_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    nav_msgs::msg::Odometry odom_msg_;
    sensor_msgs::msg::Imu imu_msg_;
    bool got_odom_{false};
    bool got_imu_{false};
    rclcpp::Time last_stamp_;

    // ---------- public interface -----------------------------------------------
  public:
    explicit OdomImuFusion(const rclcpp::Node::SharedPtr &node) : node_(node) {
        fused_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("loc/odometry", 10);

        odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
            "wheel/odom", 10, [this](const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
                odom_msg_ = *msg;
                got_odom_ = true;
            });

        imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
            "imu/data", 10, [this](const sensor_msgs::msg::Imu::ConstSharedPtr msg) {
                imu_msg_ = *msg;
                got_imu_ = true;
            });

        using namespace std::chrono_literals;
        timer_ = node_->create_wall_timer(10ms, std::bind(&OdomImuFusion::loop, this));
        last_stamp_ = node_->now();
    }

    // ---------- private helper functions ---------------------------------------
  private:
    void loop() {
        if (!got_odom_ || !got_imu_) return;

        const rclcpp::Time stamp = odom_msg_.header.stamp;
        double dt = (stamp - last_stamp_).seconds();
        if (dt <= 0.0) dt = 1e-3;

        // EKF prediction using IMU yaw‑rate
        ekf_.predict(imu_msg_.angular_velocity.z, dt);

        // EKF corrections
        ekf_.updateSpeed(odom_msg_.twist.twist.linear.x);

        tf2::Quaternion q_imu(imu_msg_.orientation.x, imu_msg_.orientation.y, imu_msg_.orientation.z,
                              imu_msg_.orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q_imu).getRPY(roll, pitch, yaw);
        ekf_.updateYaw(yaw);

        ekf_.updateAccel(imu_msg_.linear_acceleration.x);
        ekf_.updateZ(odom_msg_.pose.pose.position.z);

        publish(stamp);
        last_stamp_ = stamp;
    }

    void publish(const rclcpp::Time &stamp) {
        nav_msgs::msg::Odometry out;
        out.header.stamp = stamp;
        out.header.frame_id = "map";
        out.child_frame_id = "base_link";

        const auto &x = ekf_.state();
        out.pose.pose.position.x = x(0);
        out.pose.pose.position.y = x(1);
        out.pose.pose.position.z = x(2);

        tf2::Quaternion q;
        q.setRPY(0, 0, x(3));
        out.pose.pose.orientation.x = q.x();
        out.pose.pose.orientation.y = q.y();
        out.pose.pose.orientation.z = q.z();
        out.pose.pose.orientation.w = q.w();

        out.twist.twist.linear.x = x(4);
        out.twist.twist.angular.z = imu_msg_.angular_velocity.z;

        // copy 6×6 covariance block row‑major
        const auto &P = ekf_.covariance();
        for (int r = 0; r < 6; ++r)
            for (int c = 0; c < 6; ++c)
                out.pose.covariance[r * 6 + c] = P(r, c);

        fused_pub_->publish(out);
    }
};

// ===========================================================================
//                                    main
// ==========================================================================
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor exec;
    auto node = rclcpp::Node::make_shared("odom_imu_fusion");
    auto _fusion = std::make_shared<OdomImuFusion>(node);
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
