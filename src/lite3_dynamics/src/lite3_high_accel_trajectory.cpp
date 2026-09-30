#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class Lite3HighAccelTrajectory : public rclcpp::Node
{
public:
  Lite3HighAccelTrajectory()
  : Node("lite3_high_accel_trajectory")
  {
    amplitude_ = declare_parameter<double>("amplitude", 0.20);
    frequency_ = declare_parameter<double>("frequency", 5.0);
    ramp_time_ = declare_parameter<double>("ramp_time", 1.0);
    hold_time_ = declare_parameter<double>("hold_time", 30.0);
    publish_hz_ = declare_parameter<double>("publish_hz", 500.0);

    // Known standing reference used in the current Lite3 simulation work.
    q0_ = {
      -0.02073, -0.67214, 1.32366,
       0.01497, -0.67765, 1.33907,
      -0.02465, -0.64953, 1.32289,
       0.01714, -0.65116, 1.33529
    };

    pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/lite3_position_controller/commands",
      rclcpp::QoS(5));

    const auto period =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / publish_hz_));

    timer_ = create_wall_timer(
      period,
      std::bind(
        &Lite3HighAccelTrajectory::timerCallback,
        this));

    start_wall_ = std::chrono::steady_clock::now();

    RCLCPP_INFO(
      get_logger(),
      "Lite3 high-acceleration trajectory started.");
    RCLCPP_INFO(
      get_logger(),
      "FL Knee: q0=%.6f rad, amplitude=%.4f rad, f=%.3f Hz",
      q0_[2], amplitude_, frequency_);
    RCLCPP_INFO(
      get_logger(),
      "Ideal sine peak velocity  = %.3f rad/s",
      2.0 * M_PI * frequency_ * amplitude_);
    RCLCPP_INFO(
      get_logger(),
      "Ideal sine peak acceleration = %.3f rad/s^2",
      std::pow(2.0 * M_PI * frequency_, 2) * amplitude_);
    RCLCPP_INFO(
      get_logger(),
      "Ramp=%.2f s, steady motion=%.2f s.",
      ramp_time_, hold_time_);
  }

private:
  void timerCallback()
  {
    const double t =
      std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start_wall_).count();

    std_msgs::msg::Float64MultiArray msg;
    msg.data.resize(12);

    std::copy(
      q0_.begin(),
      q0_.end(),
      msg.data.begin());

    const double two_pi_f = 2.0 * M_PI * frequency_;

    if (t < ramp_time_) {
      // Smoothstep ramp: 10r^3 - 15r^4 + 6r^5.
      const double r =
        std::clamp(t / ramp_time_, 0.0, 1.0);

      const double r3 = r * r * r;
      const double r4 = r3 * r;
      const double r5 = r4 * r;

      const double a = 10.0 * r3 -
                       15.0 * r4 +
                        6.0 * r5;

      msg.data[2] =
        q0_[2] + amplitude_ * a * std::sin(two_pi_f * t);
    } else if (t <= ramp_time_ + hold_time_) {
      const double tr = t - ramp_time_;

      msg.data[2] =
        q0_[2] + amplitude_ * std::sin(two_pi_f * tr);
    } else {
      finished_ = true;
      msg.data[2] = q0_[2];

      pub_->publish(msg);
      RCLCPP_INFO(
        get_logger(),
        "Trajectory finished. FL Knee returned to standing.");
      timer_->cancel();
      return;
    }

    pub_->publish(msg);
  }

  std::array<double, 12> q0_{};

  double amplitude_{0.20};
  double frequency_{5.0};
  double ramp_time_{1.0};
  double hold_time_{30.0};
  double publish_hz_{500.0};

  bool finished_{false};

  std::chrono::steady_clock::time_point start_wall_;

  rclcpp::Publisher<
    std_msgs::msg::Float64MultiArray
  >::SharedPtr pub_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<Lite3HighAccelTrajectory>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
