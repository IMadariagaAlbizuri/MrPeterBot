#ifndef MR_PETERBOT_CONTROL__VEHICLE_CONTROLLER_HPP_
#define MR_PETERBOT_CONTROL__VEHICLE_CONTROLLER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class VehicleController : public rclcpp::Node
{
public:
  VehicleController(const double timer_period = 1e-2,
                    const double timeout_duration = 5e9);

private:
  // FUNCTIONS
  // Ackermann steering geometry
  std::pair<double, double> ackermann_steering_angle();

  // Rear differential velocity
  std::pair<double, double> rear_differential_velocity();

  // Timer callback — publishes commands at fixed rate
  void timer_callback();

  // Subscriber callbacks
  void steering_angle_callback(const std_msgs::msg::Float64::SharedPtr msg);
  void velocity_callback(const std_msgs::msg::Float64::SharedPtr msg);

  // Timeout
  double timeout_duration_;
  rclcpp::Time last_velocity_time_;
  rclcpp::Time last_steering_time_;

  // Vehicle parameters
  double chassis_width_;
  double chassis_length_;
  double wheel_radius_;
  double wheel_width_;
  double max_steering_angle_;
  double max_velocity_;
  double wheel_base_;
  double track_width_;

  // Current state
  double steering_angle_;
  double velocity_;

  // Commands
  std::vector<double> wheel_angular_velocity_;
  std::vector<double> wheel_steering_angle_;

  // ROS interfaces
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr steering_angle_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr velocity_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr position_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // MR_PETERBOT_CONTROL__VEHICLE_CONTROLLER_HPP_