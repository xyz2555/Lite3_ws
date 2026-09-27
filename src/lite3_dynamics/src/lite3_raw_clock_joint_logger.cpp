#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

struct ClockRecord
{
  std::uint64_t index{0};
  double wall_t{0.0};
  double sim_t{0.0};
};

struct JointRecord
{
  std::uint64_t index{0};
  double wall_t{0.0};
  double latest_clock_sim_t{std::numeric_limits<double>::quiet_NaN()};

  std::array<double, 12> q{};
  std::array<double, 12> qdot{};
  std::array<double, 12> effort{};
};

class Lite3RawClockJointLogger : public rclcpp::Node
{
public:
  Lite3RawClockJointLogger()
  : Node("lite3_raw_clock_joint_logger")
  {
    const char * home = std::getenv("HOME");
    const std::string home_dir = home ? home : "/tmp";

    joint_output_file_ = declare_parameter<std::string>(
      "joint_output_file",
      home_dir + "/lite3_raw_joint_log.csv");

    clock_output_file_ = declare_parameter<std::string>(
      "clock_output_file",
      home_dir + "/lite3_clock_log.csv");

    max_joint_samples_ = declare_parameter<std::int64_t>(
      "max_joint_samples", 300000);

    max_clock_samples_ = declare_parameter<std::int64_t>(
      "max_clock_samples", 1000000);

    if (max_joint_samples_ > 0) {
      joint_records_.reserve(
        static_cast<std::size_t>(max_joint_samples_));
    }

    if (max_clock_samples_ > 0) {
      clock_records_.reserve(
        static_cast<std::size_t>(max_clock_samples_));
    }

    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states",
      rclcpp::SensorDataQoS(),
      std::bind(
        &Lite3RawClockJointLogger::jointCallback,
        this,
        std::placeholders::_1));

    clock_sub_ = create_subscription<rosgraph_msgs::msg::Clock>(
      "/clock",
      rclcpp::QoS(rclcpp::KeepLast(100)).reliable(),
      std::bind(
        &Lite3RawClockJointLogger::clockCallback,
        this,
        std::placeholders::_1));

    status_timer_ = create_wall_timer(
      std::chrono::seconds(1),
      std::bind(
        &Lite3RawClockJointLogger::printStatus,
        this));

    RCLCPP_INFO(
      get_logger(),
      "Raw joint + full /clock logger started.");
    RCLCPP_INFO(
      get_logger(),
      "Joint CSV : %s",
      joint_output_file_.c_str());
    RCLCPP_INFO(
      get_logger(),
      "Clock CSV : %s",
      clock_output_file_.c_str());
    RCLCPP_INFO(
      get_logger(),
      "Callbacks perform no inverse dynamics, fitting, filtering, or disk I/O.");
  }

  ~Lite3RawClockJointLogger() override
  {
    writeJointCsv();
    writeClockCsv();
  }

private:
  static double wallNow()
  {
    const auto now =
      std::chrono::steady_clock::now().time_since_epoch();

    return std::chrono::duration<double>(now).count();
  }

  void clockCallback(
    const rosgraph_msgs::msg::Clock::SharedPtr msg)
  {
    const double wall_t = wallNow();

    const double sim_t =
      static_cast<double>(msg->clock.sec) +
      static_cast<double>(msg->clock.nanosec) * 1e-9;

    std::lock_guard<std::mutex> lock(mutex_);

    if (have_last_clock_ && sim_t < last_clock_sim_t_) {
      ++backward_clock_count_;
      return;
    }

    if (max_clock_samples_ > 0 &&
        static_cast<std::int64_t>(clock_records_.size()) >=
        max_clock_samples_)
    {
      ++clock_limit_drop_count_;
      return;
    }

    clock_records_.push_back({
      clock_records_.size(),
      wall_t,
      sim_t
    });

    last_clock_sim_t_ = sim_t;
    last_clock_wall_t_ = wall_t;
    have_last_clock_ = true;
  }

  void jointCallback(
    const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    const double wall_t = wallNow();

    ++joint_message_count_;

    if (msg->name.size() < 12 ||
        msg->position.size() < 12 ||
        msg->velocity.size() < 12 ||
        msg->effort.size() < 12)
    {
      ++incomplete_joint_count_;
      return;
    }

    for (std::size_t i = 0; i < 12; ++i) {
      if (joint_names_[i].empty()) {
        joint_names_[i] = msg->name[i];
      }
    }

    JointRecord record;
    record.index = joint_records_.size();
    record.wall_t = wall_t;

    {
      std::lock_guard<std::mutex> lock(mutex_);

      if (have_last_clock_) {
        record.latest_clock_sim_t = last_clock_sim_t_;
      }

      if (max_joint_samples_ > 0 &&
          static_cast<std::int64_t>(joint_records_.size()) >=
          max_joint_samples_)
      {
        ++joint_limit_drop_count_;
        return;
      }
    }

    for (std::size_t i = 0; i < 12; ++i) {
      record.q[i] = msg->position[i];
      record.qdot[i] = msg->velocity[i];
      record.effort[i] = msg->effort[i];
    }

    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      joint_records_.push_back(record);
    }
  }

  void printStatus()
  {
    std::lock_guard<std::mutex> lock1(mutex_);
    std::lock_guard<std::mutex> lock2(data_mutex_);

    double joint_wall_rate = 0.0;
    if (joint_records_.size() >= 2) {
      const double dt =
        joint_records_.back().wall_t -
        joint_records_.front().wall_t;

      if (dt > 0.0) {
        joint_wall_rate =
          static_cast<double>(joint_records_.size() - 1) / dt;
      }
    }

    double clock_wall_rate = 0.0;
    if (clock_records_.size() >= 2) {
      const double dt =
        clock_records_.back().wall_t -
        clock_records_.front().wall_t;

      if (dt > 0.0) {
        clock_wall_rate =
          static_cast<double>(clock_records_.size() - 1) / dt;
      }
    }

    RCLCPP_INFO(
      get_logger(),
      "joint_msg=%llu stored=%zu incomplete=%llu | "
      "clock=%zu backward=%llu | "
      "joint_wall_rate=%.2f Hz clock_wall_rate=%.2f Hz | "
      "joint_limit_drop=%llu clock_limit_drop=%llu",
      static_cast<unsigned long long>(joint_message_count_),
      joint_records_.size(),
      static_cast<unsigned long long>(incomplete_joint_count_),
      clock_records_.size(),
      static_cast<unsigned long long>(backward_clock_count_),
      joint_wall_rate,
      clock_wall_rate,
      static_cast<unsigned long long>(joint_limit_drop_count_),
      static_cast<unsigned long long>(clock_limit_drop_count_));
  }

  void writeJointCsv()
  {
    std::lock_guard<std::mutex> lock(data_mutex_);

    if (joint_records_.empty()) {
      RCLCPP_WARN(get_logger(), "No joint records to write.");
      return;
    }

    std::ofstream csv(joint_output_file_);

    if (!csv.is_open()) {
      RCLCPP_ERROR(
        get_logger(),
        "Cannot open joint CSV: %s",
        joint_output_file_.c_str());
      return;
    }

    csv << "index,wall_t,latest_clock_sim_t";

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

    for (const auto & r : joint_records_) {
      csv
        << r.index
        << "," << r.wall_t
        << "," << r.latest_clock_sim_t;

      for (std::size_t i = 0; i < 12; ++i) {
        csv
          << "," << r.q[i]
          << "," << r.qdot[i]
          << "," << r.effort[i];
      }

      csv << "\n";
    }

    RCLCPP_INFO(
      get_logger(),
      "Joint CSV written: %s (%zu samples)",
      joint_output_file_.c_str(),
      joint_records_.size());
  }

  void writeClockCsv()
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (clock_records_.empty()) {
      RCLCPP_WARN(get_logger(), "No clock records to write.");
      return;
    }

    std::ofstream csv(clock_output_file_);

    if (!csv.is_open()) {
      RCLCPP_ERROR(
        get_logger(),
        "Cannot open clock CSV: %s",
        clock_output_file_.c_str());
      return;
    }

    csv << "index,wall_t,sim_t\n";
    csv << std::fixed << std::setprecision(9);

    for (const auto & r : clock_records_) {
      csv
        << r.index
        << "," << r.wall_t
        << "," << r.sim_t
        << "\n";
    }

    RCLCPP_INFO(
      get_logger(),
      "Clock CSV written: %s (%zu samples)",
      clock_output_file_.c_str(),
      clock_records_.size());
  }

  std::string joint_output_file_;
  std::string clock_output_file_;

  std::int64_t max_joint_samples_{300000};
  std::int64_t max_clock_samples_{1000000};

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  std::mutex mutex_;
  std::mutex data_mutex_;

  std::vector<ClockRecord> clock_records_;
  std::vector<JointRecord> joint_records_;

  std::array<std::string, 12> joint_names_{};

  std::uint64_t joint_message_count_{0};
  std::uint64_t incomplete_joint_count_{0};
  std::uint64_t backward_clock_count_{0};
  std::uint64_t joint_limit_drop_count_{0};
  std::uint64_t clock_limit_drop_count_{0};

  bool have_last_clock_{false};
  double last_clock_sim_t_{
    std::numeric_limits<double>::quiet_NaN()};
  double last_clock_wall_t_{
    std::numeric_limits<double>::quiet_NaN()};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<Lite3RawClockJointLogger>());
  rclcpp::shutdown();
  return 0;
}
