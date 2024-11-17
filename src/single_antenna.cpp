#include <cmath>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "farmbot_interfaces/msg/float32_stamped.hpp"
#include "farmbot_interfaces/msg/float64_stamped.hpp"
#include "farmbot_interfaces/srv/datum.hpp"
#include "farmbot_interfaces/srv/threshold.hpp"
#include "farmbot_interfaces/srv/trigger.hpp"


class KalmanFilter {
    public:
        KalmanFilter(double process_noise = 0.01, double measurement_noise = 0.1)
            : estimated_state{0.0, 0.0},
            estimated_error{{1.0, 0.0}, {0.0, 1.0}},
            process_noise(process_noise),
            measurement_noise(measurement_noise) {}

        std::vector<double> step(std::vector<double> measurement) {
            // Prediction step
            std::vector<double> predicted_state = estimated_state;
            std::vector<std::vector<double>> predicted_error = estimated_error;
            for (long unsigned int i = 0; i < predicted_error.size(); ++i) {
                for (long unsigned int j = 0; j < predicted_error[i].size(); ++j) {
                    if (i == j) {
                        predicted_error[i][j] += process_noise;
                    }
                }
            }
            // Update step
            std::vector<std::vector<double>> kalman_gain_num = predicted_error;
            // Adding a small value to avoid division by near-zero values
            double epsilon = 1e-8;
            std::vector<std::vector<double>> kalman_gain_denom = predicted_error;
            for (long unsigned int i = 0; i < kalman_gain_denom.size(); ++i) {
                for (long unsigned int j = 0; j < kalman_gain_denom[i].size(); ++j) {
                    if (i == j) {
                        kalman_gain_denom[i][j] += measurement_noise + epsilon;
                    } else {
                        kalman_gain_denom[i][j] += epsilon;
                    }
                }
            }
            std::vector<std::vector<double>> kalman_gain = kalman_gain_num;
            for (long unsigned int i = 0; i < kalman_gain.size(); ++i) {
                for (long unsigned int j = 0; j < kalman_gain[i].size(); ++j) {
                    kalman_gain[i][j] /= kalman_gain_denom[i][j];
                }
            }
            std::vector<double> innovation = {measurement[0] - predicted_state[0], measurement[1] - predicted_state[1]};
            for (long unsigned int i = 0; i < estimated_state.size(); ++i) {
                estimated_state[i] = predicted_state[i] + kalman_gain[i][0] * innovation[0] + kalman_gain[i][1] * innovation[1];
            }
            for (long unsigned int i = 0; i < estimated_error.size(); ++i) {
                for (long unsigned int j = 0; j < estimated_error[i].size(); ++j) {
                    estimated_error[i][j] = (1 - kalman_gain[i][j]) * predicted_error[i][j];
                }
            }
            return estimated_state;
        }

    private:
        std::vector<double> estimated_state;
        std::vector<std::vector<double>> estimated_error;
        double process_noise;
        double measurement_noise;
};

double distance(std::tuple<double, double> coord1, std::tuple<double, double> coord2) {
    double radius_earth = 6367449.654657;
    double lat1, lon1;
    std::tie(lat1, lon1) = coord1;
    lat1 = lat1 * M_PI / 180;
    lon1 = lon1 * M_PI / 180;
    double lat2, lon2;
    std::tie(lat2, lon2) = coord2;
    lat2 = lat2 * M_PI / 180;
    lon2 = lon2 * M_PI / 180;
    double dlat = lat2 - lat1;
    double dlon = lon2 - lon1;
    double a = std::pow(std::sin(dlat / 2), 2) + std::cos(lat1) * std::cos(lat2) * std::pow(std::sin(dlon / 2), 2);
    double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));
    double distance = radius_earth * c;
    return distance;
}

double distance(sensor_msgs::msg::NavSatFix coord1, sensor_msgs::msg::NavSatFix coord2) {
    return distance(std::make_pair(coord1.latitude, coord1.longitude), std::make_pair(coord2.latitude, coord2.longitude));
}

class AntennaSplit : public rclcpp::Node {
    private:
        sensor_msgs::msg::NavSatFix datum;
        sensor_msgs::msg::NavSatFix curr_pose;
        int step = 0;
        std::vector<sensor_msgs::msg::NavSatFix> positions;
        float threshold = 0.2;
        double process_noise = 0.01;
        double measurement_noise = 0.1;
        KalmanFilter front_filter = KalmanFilter(process_noise, measurement_noise);
        KalmanFilter back_filter = KalmanFilter(process_noise, measurement_noise);

        std::string name;
        std::string frame_id;

        std::string gps_sub_topic;
        std::string gps_main_pub_topic;
        std::string gps_aux_pub_topic;
        int kallman_type;

        rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_front_;
        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_back_;

        rclcpp::Service<farmbot_interfaces::srv::Threshold>::SharedPtr thresh_ser_;

    public:
        AntennaSplit() : Node(
            "single_antenna",
            rclcpp::NodeOptions()
            .allow_undeclared_parameters(true)
            .automatically_declare_parameters_from_overrides(true)
        ){
            try {
                name = this->get_parameter("name").as_string();
            } catch (...) {
                name = "single_antenna";
            }

            //try to get the parameters of gps_topic, gps_main, gps_aux, and kallman_type
            try{
                rclcpp::Parameter gps_topic = this->get_parameter("gps_topic");
                gps_sub_topic = gps_topic.as_string();
                rclcpp::Parameter gps_front_param = this->get_parameter("gps_main");
                gps_main_pub_topic = gps_front_param.as_string();
                rclcpp::Parameter gps_back_param = this->get_parameter("gps_aux");
                gps_aux_pub_topic = gps_back_param.as_string();
                rclcpp::Parameter kallman_type_param = this->get_parameter("kallman_type");
                kallman_type = kallman_type_param.as_int();
            } catch(const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "Could not find one of those parameters: gps_topic, gps_main, gps_aux, kallman_type, using defaults");
                gps_sub_topic = "gps";
                gps_main_pub_topic = "gps_main";
                gps_aux_pub_topic = "gps_aux";
                kallman_type = 0;
            }

            gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(gps_sub_topic, 10, std::bind(&AntennaSplit::callback, this, std::placeholders::_1));
            gps_front_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(gps_main_pub_topic, 10);
            gps_back_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(gps_aux_pub_topic, 10);

            thresh_ser_ = this->create_service<farmbot_interfaces::srv::Threshold>("gps_fuse_dist", std::bind(&AntennaSplit::thresh_callback, this, std::placeholders::_1, std::placeholders::_2));

            frame_id = this->get_namespace();
            if (!frame_id.empty() && frame_id[0] == '/') {
                frame_id = frame_id.substr(1); // Remove leading slash
            }
            frame_id += "/gps";
            RCLCPP_INFO(this->get_logger(), "Node %s started and subscribed to %s", name.c_str(), gps_sub_topic.c_str());
        }

    private:
        void callback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr& gps) {
            sensor_msgs::msg::NavSatFix main_gps = *gps;
            step++;
            if (positions.size() < 2) {
                if (positions.size() == 0) {
                    positions.push_back(main_gps);
                    return;
                }
                double dist = distance(main_gps, positions[0]);
                if (dist > threshold) {
                    positions.push_back(main_gps);
                    RCLCPP_INFO(this->get_logger(), "VEHICLE HAS MOVED SUFFICIENTLY, SWITCHING TO SIMULATED DUAL ANTENNA MODE");
                    return;
                }
                RCLCPP_INFO(this->get_logger(), "WAITING FOR THE VEHICLE MOVEMENT TO INITIALIZE SIMULATED DUAL ANTENNA MODE");
                return;
            }

            if (kallman_type == 0) {
                double dist = distance(main_gps, positions[0]);
                // RCLCPP_INFO(this->get_logger(), "Distance: %f", dist);
                if (dist > threshold) {
                    positions[0] = positions[1];
                    positions[1] = main_gps;
                }
                main_gps.header.frame_id = frame_id;
                positions[0].header.stamp = main_gps.header.stamp;
                gps_front_->publish(main_gps);
                gps_back_->publish(positions[0]);
            } else if (kallman_type == 1) {
                std::vector<double> measurement = {main_gps.latitude, main_gps.longitude};
                std::vector<double> estimated_state = front_filter.step(measurement);
                // temp_gps = main_gps;
                main_gps.latitude = estimated_state[0];
                main_gps.longitude = estimated_state[1];
                // if (distance(temp_gps, main_gps) > threshold) {
                //     positions[0] = positions[1];
                //     positions[1] = main_gps;
                // }
                if (distance(main_gps, positions[0]) > threshold) {
                    std::vector<double> measurement = {positions[1].latitude, positions[1].longitude};
                    std::vector<double> estimated_state = back_filter.step(measurement);
                    positions[1].latitude = estimated_state[0];
                    positions[1].longitude = estimated_state[1];
                    positions[0] = positions[1];
                    positions[1] = main_gps;
                }
                positions[0].header.stamp = gps->header.stamp;
                gps_front_->publish(main_gps);
                gps_back_->publish(positions[0]);
            }
        }
        void thresh_callback(const std::shared_ptr<farmbot_interfaces::srv::Threshold::Request> _request, std::shared_ptr<farmbot_interfaces::srv::Threshold::Response> _response) {
            threshold = _request->distance.data;
            std::string message = "Threshold set to " + std::to_string(threshold) + " meters";
            RCLCPP_INFO(this->get_logger(), message.c_str());
            _response->message = message;
            return;
        }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto navfix = std::make_shared<AntennaSplit>();
    rclcpp::spin(navfix);
    rclcpp::shutdown();
    return 0;
}
