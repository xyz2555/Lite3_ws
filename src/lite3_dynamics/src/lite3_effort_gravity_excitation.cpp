#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <lite3_dynamics/dynamics.hpp>

using lite3_kinematics::Leg;
using lite3_dynamics::Lite3SingleLegDynamics;

namespace
{
constexpr int NJ = 12;

const std::array<std::string, NJ> kJointNames = {
  "FL_HipX_joint", "FL_HipY_joint", "FL_Knee_joint",
  "FR_HipX_joint", "FR_HipY_joint", "FR_Knee_joint",
  "HL_HipX_joint", "HL_HipY_joint", "HL_Knee_joint",
  "HR_HipX_joint", "HR_HipY_joint", "HR_Knee_joint"
};

const std::array<Leg, 4> kLegs = {
  Leg::FL, Leg::FR, Leg::HL, Leg::HR
};

int jointIndex(const std::string & name)
{
  for (int i = 0; i < NJ; ++i) {
    if (kJointNames[static_cast<std::size_t>(i)] == name) {
      return i;
    }
  }
  return -1;
}
}  // namespace

class Lite3EffortPdExcitation : public rclcpp::Node
{
public:
  Lite3EffortPdExcitation()
  : Node("lite3_effort_pd_excitation")
  {
    publish_hz_ = declare_parameter<double>("publish_hz", 500.0);
    gravity_scale_ = declare_parameter<double>("gravity_scale", 1.0);
    gravity_sign_ = declare_parameter<double>("gravity_sign", 1.0);

    kp_ = declare_parameter<double>("kp", 5.0);
    kd_ = declare_parameter<double>("kd", 1.0);
    max_command_effort_ = declare_parameter<double>("max_command_effort", 5.0);

    excitation_amplitude_ =
      declare_parameter<double>("excitation_amplitude", 0.0);
    excitation_frequency_ =
      declare_parameter<double>("excitation_frequency", 1.0);
    ramp_time_ = declare_parameter<double>("ramp_time", 2.0);

    settle_time_ = declare_parameter<double>("settle_time", 5.0);
    excitation_duration_ =
      declare_parameter<double>("excitation_duration", 10.0);
    recovery_time_ = declare_parameter<double>("recovery_time", 5.0);

    excitation_joint_ = declare_parameter<int>("excitation_joint", 2);
    start_experiment_ = declare_parameter<bool>("start_experiment", false);

    q_des_ = declare_parameter<std::vector<double>>(
      "q_des",
      {
        -0.02073, -0.67214, 1.32366,
         0.01497, -0.67765, 1.33907,
        -0.02465, -0.64953, 1.32289,
         0.01714, -0.65116, 1.33529
      });

    command_output_file_ = declare_parameter<std::string>(
      "command_output_file", "/tmp/lite3_effort_pd_excitation_v5_command.csv");

    if (publish_hz_ <= 0.0) {
      throw std::runtime_error("publish_hz must be > 0");
    }
    if (excitation_joint_ < 0 || excitation_joint_ >= NJ) {
      throw std::runtime_error("excitation_joint must be in [0, 11]");
    }
    if (q_des_.size() != NJ) {
      throw std::runtime_error("q_des must contain exactly 12 values");
    }
    if (settle_time_ < 0.0 || excitation_duration_ < 0.0 ||
        recovery_time_ < 0.0 || ramp_time_ < 0.0) {
      throw std::runtime_error(
        "settle_time, excitation_duration, recovery_time, ramp_time must be >= 0");
    }
    if (kp_ < 0.0 || kd_ < 0.0) {
      throw std::runtime_error("kp and kd must be >= 0");
    }
    if (max_command_effort_ <= 0.0) {
      throw std::runtime_error("max_command_effort must be > 0");
    }

    pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/lite3_effort_controller/commands", rclcpp::QoS(10));

    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states",
      rclcpp::SensorDataQoS(),
      std::bind(
        &Lite3EffortPdExcitation::jointCallback,
        this,
        std::placeholders::_1));

    csv_.open(command_output_file_, std::ios::out | std::ios::trunc);
    if (!csv_.is_open()) {
      throw std::runtime_error(
        "Cannot open command_output_file: " + command_output_file_);
    }

    csv_ << "wall_t,sim_t,phase,excitation,q_error_norm";
    for (const auto & name : kJointNames) {
      csv_ << "," << name;
    }
    csv_ << "\n";
    csv_ << std::fixed << std::setprecision(9);

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / publish_hz_));

    timer_ = create_wall_timer(
      period,
      std::bind(&Lite3EffortPdExcitation::timerCallback, this));

    start_wall_time_ = std::chrono::steady_clock::now();

    RCLCPP_INFO(
      get_logger(),
      "v5 started: kp=%.3f kd=%.3f max_cmd=%.3f",
      kp_, kd_, max_command_effort_);
    RCLCPP_INFO(
      get_logger(),
      "IMPORTANT: experiment timing starts only when start_experiment=true.");
    RCLCPP_INFO(
      get_logger(),
      "gravity_sign=%.1f scale=%.3f amp=%.4f Nm f=%.3f Hz joint=%d",
      gravity_sign_, gravity_scale_,
      excitation_amplitude_, excitation_frequency_, excitation_joint_);
    RCLCPP_INFO(
      get_logger(),
      "pre-start = gravity+PD hold; no zero-torque shutdown while effort controller is active");
  }

  ~Lite3EffortPdExcitation() override
  {
    if (csv_.is_open()) {
      csv_.flush();
      csv_.close();
    }
  }

private:
  void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::array<double, NJ> q{};
    std::array<double, NJ> qdot{};
    std::array<bool, NJ> got_q{};
    std::array<bool, NJ> got_qdot{};

    for (std::size_t i = 0; i < msg->name.size(); ++i) {
      const int idx = jointIndex(msg->name[i]);
      if (idx < 0) {
        continue;
      }

      if (i < msg->position.size()) {
        q[static_cast<std::size_t>(idx)] = msg->position[i];
        got_q[static_cast<std::size_t>(idx)] = true;
      }
      if (i < msg->velocity.size()) {
        qdot[static_cast<std::size_t>(idx)] = msg->velocity[i];
        got_qdot[static_cast<std::size_t>(idx)] = true;
      }
    }

    for (int i = 0; i < NJ; ++i) {
      if (!got_q[static_cast<std::size_t>(i)] ||
          !got_qdot[static_cast<std::size_t>(i)]) {
        return;
      }
    }

    std::lock_guard<std::mutex> lock(mutex_);
    q_ = q;
    qdot_ = qdot;
    have_joint_state_ = true;
  }

  Eigen::Matrix<double, NJ, 1> gravityTorque(
    const std::array<double, NJ> & q) const
  {
    Eigen::Matrix<double, NJ, 1> G =
      Eigen::Matrix<double, NJ, 1>::Zero();

    Lite3SingleLegDynamics dynamics;

    for (std::size_t li = 0; li < kLegs.size(); ++li) {
      Eigen::Vector3d q_leg;
      q_leg <<
        q[li * 3 + 0],
        q[li * 3 + 1],
        q[li * 3 + 2];

      const Eigen::Vector3d g =
        dynamics.gravityTorque(kLegs[li], q_leg);

      G.segment<3>(static_cast<Eigen::Index>(li * 3)) = g;
    }

    return G;
  }

  double totalDuration() const
  {
    return settle_time_ + excitation_duration_ + recovery_time_;
  }

  Eigen::Matrix<double, NJ, 1> makeHoldCommand(
    const std::array<double, NJ> & q,
    const std::array<double, NJ> & qdot,
    double * q_error_norm) const
  {
    const Eigen::Matrix<double, NJ, 1> G = gravityTorque(q);

    // Physical target:
    // tau_phys = G - Kp(q-q_des) - Kd*qdot + tau_exc
    // For the Gazebo effort interface used here, the effort command is
    // applied directly to the joint. Therefore command = desired physical torque.
    Eigen::Matrix<double, NJ, 1> tau_cmd =
      gravity_sign_ * gravity_scale_ * G;

    double e2 = 0.0;
    for (int i = 0; i < NJ; ++i) {
      const double e =
        q[static_cast<std::size_t>(i)] -
        q_des_[static_cast<std::size_t>(i)];
      e2 += e * e;

      tau_cmd[i] +=
        -kp_ * e -
        kd_ * qdot[static_cast<std::size_t>(i)];
    }

    if (q_error_norm != nullptr) {
      *q_error_norm = std::sqrt(e2);
    }

    return tau_cmd;
  }

  void timerCallback()
  {
    const double sim_t = get_clock()->now().seconds();

    bool requested_start = false;
    get_parameter("start_experiment", requested_start);

    // No timing reference is created in the constructor. This prevents the
    // /clock startup jump from making the experiment finish immediately.
    if (requested_start && !experiment_started_) {
      if (!have_joint_state_) {
        return;
      }

      start_sim_time_ = sim_t;
      start_wall_time_ = std::chrono::steady_clock::now();
      experiment_started_ = true;
      finished_ = false;

      RCLCPP_INFO(
        get_logger(),
        "Experiment armed at sim_t=%.6f; t=0 starts now.",
        start_sim_time_);
    }

    // A false start parameter means: remain in stable gravity+PD hold,
    // but do not advance the experiment timeline.
    if (!experiment_started_) {
      publishHoldCommand("prestart", 0.0);
      return;
    }

    const double t = sim_t - start_sim_time_;

    // If simulation time goes backwards/reset, stop the active timeline and
    // require start_experiment to be toggled again.
    if (t < -1e-6) {
      experiment_started_ = false;
      RCLCPP_WARN(
        get_logger(),
        "Simulation time moved backwards. Experiment disarmed; holding PD+gravity.");
      publishHoldCommand("prestart", 0.0);
      return;
    }

    std::array<double, NJ> q_copy{};
    std::array<double, NJ> qdot_copy{};
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!have_joint_state_) {
        return;
      }
      q_copy = q_;
      qdot_copy = qdot_;
    }

    double q_error_norm = 0.0;
    Eigen::Matrix<double, NJ, 1> tau_cmd =
      makeHoldCommand(q_copy, qdot_copy, &q_error_norm);

    double excitation = 0.0;
    std::string phase = "settle";

    const double excite_start = settle_time_;
    const double excite_stop = settle_time_ + excitation_duration_;
    const double total_stop = totalDuration();

    if (t >= excite_start && t < excite_stop) {
      phase = "excite";

      const double te = t - excite_start;
      double envelope = 1.0;
      if (ramp_time_ > 0.0 && te < ramp_time_) {
        const double r = std::clamp(te / ramp_time_, 0.0, 1.0);
        const double r2 = r * r;
        const double r3 = r2 * r;
        const double r4 = r3 * r;
        const double r5 = r4 * r;
        envelope = 10.0 * r3 - 15.0 * r4 + 6.0 * r5;
      }

      if (excitation_amplitude_ != 0.0 && excitation_frequency_ > 0.0) {
        excitation = envelope * excitation_amplitude_ *
          std::sin(2.0 * M_PI * excitation_frequency_ * te);

        // Desired physical +excitation is added directly to the effort command.
        tau_cmd[excitation_joint_] += excitation;
      }
    } else if (t >= excite_stop && t < total_stop) {
      phase = "recovery";
    } else if (t >= total_stop) {
      phase = "post_hold";
      if (!finished_) {
        finished_ = true;
        RCLCPP_INFO(
          get_logger(),
          "Experiment finished at t=%.3f s; continuing PD+gravity hold (not zero torque).",
          t);
      }
    }

    for (int i = 0; i < NJ; ++i) {
      tau_cmd[i] = std::clamp(
        tau_cmd[i], -max_command_effort_, max_command_effort_);
    }

    std_msgs::msg::Float64MultiArray msg;
    msg.data.resize(NJ);
    for (int i = 0; i < NJ; ++i) {
      msg.data[static_cast<std::size_t>(i)] = tau_cmd[i];
    }
    pub_->publish(msg);

    const double wall_t =
      std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start_wall_time_).count();

    csv_ << wall_t << ','
         << t << ','
         << phase << ','
         << excitation << ','
         << q_error_norm;
    for (int i = 0; i < NJ; ++i) {
      csv_ << ',' << tau_cmd[i];
    }
    csv_ << '\n';

    if ((sample_count_++ % 500) == 0) {
      RCLCPP_INFO(
        get_logger(),
        "t=%.3f phase=%s qerr=%.5f FLKnee q=%.5f qdot=%.5f cmd=%.5f exc=%.5f",
        t, phase.c_str(), q_error_norm,
        q_copy[2], qdot_copy[2], tau_cmd[2], excitation);
    }
  }

  void publishHoldCommand(const std::string & phase, double excitation)
  {
    std::array<double, NJ> q_copy{};
    std::array<double, NJ> qdot_copy{};
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!have_joint_state_) {
        return;
      }
      q_copy = q_;
      qdot_copy = qdot_;
    }

    double q_error_norm = 0.0;
    auto tau_cmd = makeHoldCommand(q_copy, qdot_copy, &q_error_norm);

    for (int i = 0; i < NJ; ++i) {
      tau_cmd[i] = std::clamp(
        tau_cmd[i], -max_command_effort_, max_command_effort_);
    }

    std_msgs::msg::Float64MultiArray msg;
    msg.data.resize(NJ);
    for (int i = 0; i < NJ; ++i) {
      msg.data[static_cast<std::size_t>(i)] = tau_cmd[i];
    }
    pub_->publish(msg);

    const double sim_t = get_clock()->now().seconds();
    const double wall_t =
      std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start_wall_time_).count();

    csv_ << wall_t << ','
         << sim_t << ','
         << phase << ','
         << excitation << ','
         << q_error_norm;
    for (int i = 0; i < NJ; ++i) {
      csv_ << ',' << tau_cmd[i];
    }
    csv_ << '\n';
  }

  double publish_hz_{500.0};
  double gravity_scale_{1.0};
  double gravity_sign_{-1.0};

  double kp_{5.0};
  double kd_{1.0};
  double max_command_effort_{5.0};

  double excitation_amplitude_{0.0};
  double excitation_frequency_{1.0};
  double ramp_time_{2.0};

  double settle_time_{5.0};
  double excitation_duration_{10.0};
  double recovery_time_{5.0};

  int excitation_joint_{2};
  bool start_experiment_{false};
  bool experiment_started_{false};
  bool finished_{false};

  std::vector<double> q_des_;
  std::string command_output_file_;

  bool have_joint_state_{false};
  std::array<double, NJ> q_{};
  std::array<double, NJ> qdot_{};
  std::mutex mutex_;

  double start_sim_time_{0.0};
  std::chrono::steady_clock::time_point start_wall_time_;
  std::size_t sample_count_{0};

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::ofstream csv_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Lite3EffortPdExcitation>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
