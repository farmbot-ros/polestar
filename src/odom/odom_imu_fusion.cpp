#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

// EXPLANATION OF EKF IN BICYCLE METAPHOR (BIKEING IN THE DARK WITH A FLASHLIGHT)
// -------------------------------------------------------------------------------
//
// 1.You keep pedalling even when you can’t clearly see the road. That pedalling is your model
//      of where you really are. “If I hold 12 km/h and the handlebar is turned 2 ° s⁻¹ to the
//      right, in half a second I ought to be roughly here.” → EKF’s prediction step.
//
// 2.Every so often the lamp bounces off a reflector or a white line, giving you a fuzzy glimpse
//      of where you really are. That glimpse is a measurement—blurry, noisy, sometimes wrong.
//
// 3.You blend the two.
//     - If you trust your pedalling model more than blurry glimpse, you stick to your internal guess.
//     - If the glimpse looks very reliable , you shift your belief toward it.
//     → EKF’s update step.
//
// 4.And you repeat this rhythm dozens of times a second. Each cycle gives you a best‑guess position
//      and a sense of how uncertain you still are (a spreading or shrinking “bubble” around the guess).
//
// 5.The bubble is the uncertainty in your prediction. The more you trust your pedalling model, the
//      bigger the bubble gets. The more you trust the measurement, the smaller the bubble gets.
//
// 6.If you have non-linear street ahead, you need to use a nonlinear filter. Extended Kalman filter
//      approximates the nonlinearity by a linear approximation in a very brief time window. You
//      "lie" to the filter pretending curves are straight lines, but you correct that lie before
//      it drifts too far away from the truth.

// Piece                        Bicycle metaphor                                In EKF code terms
// ============================ =============================================== ====================
// State x                      Where you think you are and
//                              how fast you’re heading                         Vector x_
//----------------------------- ----------------------------------------------- --------------------
// Covariance P                 How big the bubble of uncertainty is,
//                              and in which directions it stretches            Matrix P_
//----------------------------- ----------------------------------------------- --------------------
// Process model f( )           Pedal & steer → where you expect
//                              to be a moment later                            predict()
//----------------------------- ----------------------------------------------- --------------------
// Process‑noise Q              Road bumps you didn’t model (wind, slip)        Q_
//----------------------------- ----------------------------------------------- --------------------
// Measurement model h( )       What a reflector reading should look
//                              like if you really were at x                    update()
//----------------------------- ----------------------------------------------- --------------------
// Measurement‑noise R          Blurriness of that reflector reading            R_
//----------------------------- ----------------------------------------------- --------------------
// Jacobian F, H                Instantaneous slopes that turn curves
//                              into straight lines                             matrices F and H
//----------------------------- ----------------------------------------------- --------------------
//
// IMPORTANT:
// Garbage in → garbage out. If your models or noise statistics are badly off, the elegant math won’t
// rescue you—tuning Q and R is where the art lives.

namespace fusion {
    class EKF {
      public:
        EKF() { reset(); }

        void reset() {
            x_.setZero();
            P_.setIdentity();
            Q_ = 0.1 * Eigen::Matrix4d::Identity();
            R_ = 0.01;
        }

        void predict(double v, double omega, double dt) {
            if (dt <= 0.0) return;
            double theta = x_(2);
            x_(0) += v * std::cos(theta) * dt;
            x_(1) += v * std::sin(theta) * dt;
            x_(2) = normalize(theta + omega * dt);
            x_(3) = v;

            Eigen::Matrix4d F = Eigen::Matrix4d::Identity();
            F(0, 2) = -v * std::sin(theta) * dt;
            F(0, 3) = std::cos(theta) * dt;
            F(1, 2) = v * std::cos(theta) * dt;
            F(1, 3) = std::sin(theta) * dt;

            P_ = F * P_ * F.transpose() + Q_;
        }

        void update(double yaw) {
            double y = normalize(yaw - x_(2));
            Eigen::RowVector4d H;
            H << 0, 0, 1, 0;
            double S = (H * P_ * H.transpose())(0) + R_;
            Eigen::Vector4d K = P_ * H.transpose() / S;
            x_ += K * y;
            P_ = (Eigen::Matrix4d::Identity() - K * H) * P_;
            x_(2) = normalize(x_(2));
        }

        const Eigen::Vector4d &state() const { return x_; }
        const Eigen::Matrix4d &covariance() const { return P_; }

      private:
        Eigen::Vector4d x_;
        Eigen::Matrix4d P_;
        Eigen::Matrix4d Q_;
        double R_;

        static double normalize(double a) {
            while (a > M_PI)
                a -= 2.0 * M_PI;
            while (a < -M_PI)
                a += 2.0 * M_PI;
            return a;
        }
    };
} // namespace fusion

class OdomImuFusion {
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

  public:
    explicit OdomImuFusion(const rclcpp::Node::SharedPtr &node) : node_(node) {
        fused_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("loc/fused/odom", 10);
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

  private:
    void loop() {
        if (!got_odom_ || !got_imu_) return;
        RCLCPP_INFO_ONCE(node_->get_logger(), "Odom-IMU Fusion node started");
        rclcpp::Time stamp = odom_msg_.header.stamp;
        double dt = (stamp - last_stamp_).seconds();
        if (dt <= 0.0) dt = 1e-3;
        double v = odom_msg_.twist.twist.linear.x;
        double omega = odom_msg_.twist.twist.angular.z;
        ekf_.predict(v, omega, dt);
        tf2::Quaternion q(imu_msg_.orientation.x, imu_msg_.orientation.y, imu_msg_.orientation.z,
                          imu_msg_.orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        ekf_.update(yaw);
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
        tf2::Quaternion q;
        q.setRPY(0, 0, x(2));
        out.pose.pose.orientation.x = q.x();
        out.pose.pose.orientation.y = q.y();
        out.pose.pose.orientation.z = q.z();
        out.pose.pose.orientation.w = q.w();
        out.twist.twist.linear.x = x(3);
        out.twist.twist.angular.z = 0.0;
        const auto &P = ekf_.covariance();
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                out.pose.covariance[i * 6 + j] = P(i, j);
        fused_pub_->publish(out);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);
    options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("odom_imu_fusion", options);
    auto fusion_node = std::make_shared<OdomImuFusion>(node);
    try {
        executor.add_node(node);
        executor.spin();
    } catch (const std::exception &e) {
        RCLCPP_FATAL(node->get_logger(), "Exception: %s", e.what());
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
