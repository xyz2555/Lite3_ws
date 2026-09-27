#include <algorithm>
#include <array>
#include <chrono>
#include <cstdlib>
#include <functional>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <limits>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <builtin_interfaces/msg/time.hpp>

struct ClockSample
{
  double wall_t{0.0};
  double sim_t{0.0};
};

struct RawSample
{
  std::uint64_t index{0};
  double wall_t{0.0};
  double sim_t{std::numeric_limits<double>::quiet_NaN()};
  double mapped_clock_t{std::numeric_limits<double>::quiet_NaN()};

  std::array<double, 12> q{};
  std::array<double, 12> qdot{};
  std::array<double, 12> effort{};
  bool valid{false};
};

class Lite3RawJointLogger : public rclcpp::Node
{
public:
  Lite3RawJointLogger()
  : Node("lite3_raw_joint_logger")
  {
    output_file_ = declare_parameter<std::string>(
      "output_file",
      std::string(std::getenv("HOME") ? std::getenv("HOME") : "/tmp") +
      "/lite3_raw_joint_log.csv");

    max_samples_ = declare_parameter<std::int64_t>(
      "max_samples", 300000);

    RCLCPP_INFO(get_logger(), "Lite3 raw joint logger started.");
    RCLCPP_INFO(
      get_logger(),
      "Output: %s", output_file_.c_str());
    RCLCPP_INFO(
      get_logger(),
      "Maximum samples: %ld", static_cast<long>(max_samples_));
    RCLCPP_INFO(
      get_logger(),
      "No inverse dynamics, polynomial fit, filtering, or CSV write in callbacks.");

    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states",
      rclcpp::SensorDataQoS(),
      std::bind(
        &Lite3RawJointLogger::jointCallback,
        this,
        std::placeholders::_1));

    clock_sub_ = create_subscription<rosgraph_msgs::msg::Clock>(
      "/clock",
      rclcpp::QoS(rclcpp::KeepLast(20)).reliable(),
      std::bind(
        &Lite3RawJointLogger::clockCallback,
        this,
        std::placeholders::_1));

    status_timer_ = create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&Lite3RawJointLogger::printStatus, this));
  }

  ~Lite3RawJointLogger() override
  {
    writeCsv();
  }

private:
  static double wallNow()
  {
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    return std::chrono::duration<double>(now).count();
  }

  static double rosClockToSec(const builtin_interfaces::msg::Time & t)
  {
    return static_cast<double>(t.sec) +
           static_cast<double>(t.nanosec) * 1e-9;
  }

  void clockCallback(const rosgraph_msgs::msg::Clock::SharedPtr msg)
  {
    const double wall_t = wallNow();
    const double sim_t = rosClockToSec(msg->clock);

    std::lock_guard<std::mutex> lock(mutex_);

    if (!clock_history_.empty() &&
        sim_t <= clock_history_.back().sim_t)
    {
      return;
    }

    clock_history_.push_back({wall_t, sim_t});

    // Keep only a small recent history.
    constexpr std::size_t MAX_CLOCK_HISTORY = 20;
    if (clock_history_.size() > MAX_CLOCK_HISTORY) {
      clock_history_.erase(clock_history_.begin());
    }

    latest_sim_t_ = sim_t;
    latest_clock_wall_t_ = wall_t;
    ++clock_count_;
  }

  bool mapWallToSim(double wall_t, double & sim_t_out) const
  {
    if (clock_history_.empty()) {
      return false;
    }

    if (clock_history_.size() == 1) {
      sim_t_out = clock_history_.back().sim_t;
      return true;
    }

    // Use the two newest clock samples for local interpolation/extrapolation.
    const auto & c1 = clock_history_[clock_history_.size() - 2];
    const auto & c2 = clock_history_[clock_history_.size() - 1];

    const double wall_dt = c2.wall_t - c1.wall_t;
    const double sim_dt = c2.sim_t - c1.sim_t;

    if (wall_dt <= 1e-9 || sim_dt < 0.0) {
      sim_t_out = c2.sim_t;
      return true;
    }

    const double sim_rate = sim_dt / wall_dt;
    double mapped = c2.sim_t + (wall_t - c2.wall_t) * sim_rate;

    // The logger is for raw data collection. Do not allow artificial
    // backward motion in the recorded simulation timestamp.
    if (std::isfinite(last_recorded_sim_t_)) {
      mapped = std::max(mapped, last_recorded_sim_t_);
    }

    sim_t_out = mapped;
    return true;
  }

  void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    const double wall_t = wallNow();

    RawSample sample;
    sample.index = joint_count_++;
    sample.wall_t = wall_t;

    for (std::size_t i = 0; i < msg->name.size() && i < 12; ++i) {
      if (joint_names_[i].empty()) {
        joint_names_[i] = msg->name[i];
      }

      if (i < msg->position.size()) {
        sample.q[i] = msg->position[i];
      } else {
        sample.q[i] = std::numeric_limits<double>::quiet_NaN();
      }

      if (i < msg->velocity.size()) {
        sample.qdot[i] = msg->velocity[i];
      } else {
        sample.qdot[i] = std::numeric_limits<double>::quiet_NaN();
      }

      if (i < msg->effort.size()) {
        sample.effort[i] = msg->effort[i];
      } else {
        sample.effort[i] = std::numeric_limits<double>::quiet_NaN();
      }
    }

    if (msg->name.size() < 12 ||
        msg->position.size() < 12 ||
        msg->velocity.size() < 12 ||
        msg->effort.size() < 12)
    {
      ++rejected_messages_;
      return;
    }

    bool clock_ok = false;
    double mapped_sim_t = std::numeric_limits<double>::quiet_NaN();

    {
      std::lock_guard<std::mutex> lock(mutex_);

      if (clock_history_.size() >= 1) {
        clock_ok = mapWallToSim(wall_t, mapped_sim_t);
      }

      sample.sim_t = mapped_sim_t;
      sample.mapped_clock_t = latest_sim_t_;
    }

    if (!clock_ok) {
      ++no_time_messages_;
    }

    // Header stamp is retained separately for diagnostic purposes by storing
    // it below through the synthetic wall/sim fields only; this logger does
    // not overwrite it with ROS "now()".
    sample.valid = true;

    {
      std::lock_guard<std::mutex> lock(data_mutex_);

      if (max_samples_ > 0 &&
          static_cast<std::int64_t>(samples_.size()) >= max_samples_)
      {
        ++dropped_due_limit_;
        return;
      }

      samples_.push_back(sample);

      if (std::isfinite(mapped_sim_t)) {
        last_recorded_sim_t_ = mapped_sim_t;
      }
    }

    ++accepted_messages_;
  }

  void printStatus()
  {
    std::lock_guard<std::mutex> data_lock(data_mutex_);
    std::lock_guard<std::mutex> clock_lock(mutex_);

    double wall_rate = 0.0;
    if (!samples_.empty()) {
      const double t_last = samples_.back().wall_t;
      const double t_first = samples_.front().wall_t;
      if (t_last > t_first) {
        wall_rate =
          static_cast<double>(samples_.size() - 1) /
          (t_last - t_first);
      }
    }

    double sim_rate = 0.0;
    if (samples_.size() >= 2) {
      const double t_last = samples_.back().sim_t;
      const double t_first = samples_.front().sim_t;
      if (std::isfinite(t_last) &&
          std::isfinite(t_first) &&
          t_last > t_first)
      {
        sim_rate =
          static_cast<double>(samples_.size() - 1) /
          (t_last - t_first);
      }
    }

    RCLCPP_INFO(
      get_logger(),
      "joint_msg=%zu accepted=%zu stored=%zu no_time=%zu "
      "limit_drop=%zu clock=%zu wall_rate=%.2f Hz sim_rate=%.2f Hz",
      joint_count_,
      accepted_messages_,
      samples_.size(),
      no_time_messages_,
      dropped_due_limit_,
      clock_count_,
      wall_rate,
      sim_rate);
  }

  void writeCsv()
  {
    std::lock_guard<std::mutex> data_lock(data_mutex_);

    if (samples_.empty()) {
      RCLCPP_WARN(get_logger(), "No samples to write.");
      return;
    }

    std::ofstream csv(output_file_);
    if (!csv.is_open()) {
      RCLCPP_ERROR(
        get_logger(),
        "Cannot open output file: %s",
        output_file_.c_str());
      return;
    }

    csv << "index,wall_t,sim_t,mapped_clock_t";

    for (std::size_t i = 0; i < 12; ++i) {
      std::string name = joint_names_[i];
      if (name.empty()) {
        name = "joint_" + std::to_string(i);
      }

      csv
        << "," << name << "_q"
        << "," << name << "_qdot"
        << "," << name << "_effort";
    }

    csv << "\n";
    csv << std::fixed << std::setprecision(9);

    for (const auto & s : samples_) {
      csv
        << s.index
        << "," << s.wall_t
        << "," << s.sim_t
        << "," << s.mapped_clock_t;

      for (std::size_t i = 0; i < 12; ++i) {
        csv
          << "," << s.q[i]
          << "," << s.qdot[i]
          << "," << s.effort[i];
      }

      csv << "\n";
    }

    RCLCPP_INFO(
      get_logger(),
      "Raw CSV written: %s (%zu samples)",
      output_file_.c_str(),
      samples_.size());
  }

  std::string output_file_;
  std::int64_t max_samples_{300000};

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  mutable std::mutex mutex_;
  std::vector<ClockSample> clock_history_;
  double latest_sim_t_{std::numeric_limits<double>::quiet_NaN()};
  double latest_clock_wall_t_{std::numeric_limits<double>::quiet_NaN()};

  std::mutex data_mutex_;
  std::vector<RawSample> samples_;
  std::array<std::string, 12> joint_names_{};

  std::uint64_t joint_count_{0};
  std::uint64_t accepted_messages_{0};
  std::uint64_t rejected_messages_{0};
  std::uint64_t no_time_messages_{0};
  std::uint64_t dropped_due_limit_{0};
  std::uint64_t clock_count_{0};

  double last_recorded_sim_t_{
    std::numeric_limits<double>::quiet_NaN()};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Lite3RawJointLogger>());
  rclcpp::shutdown();
  return 0;
}
