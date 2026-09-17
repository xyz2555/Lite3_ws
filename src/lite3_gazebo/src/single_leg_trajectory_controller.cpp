#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "lite3_dynamics/dynamics.hpp"
#include "lite3_kinematics/kinematics.hpp"

class SingleLegTrajectoryController : public rclcpp::Node
{
public:
    SingleLegTrajectoryController()
    : Node("single_leg_trajectory_controller"),
      got_joint_state_(false),
      trajectory_start_time_(this->now())
    {
        command_pub_ =
            this->create_publisher<std_msgs::msg::Float64MultiArray>(
                "/lite3_effort_controller/commands", 10);

        joint_sub_ =
            this->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states",
                10,
                std::bind(
                    &SingleLegTrajectoryController::jointStateCallback,
                    this,
                    std::placeholders::_1));

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(2),
            std::bind(
                &SingleLegTrajectoryController::controlLoop,
                this));

        joint_names_ = {
            "FL_HipX_joint",
            "FL_HipY_joint",
            "FL_Knee_joint",

            "FR_HipX_joint",
            "FR_HipY_joint",
            "FR_Knee_joint",

            "HL_HipX_joint",
            "HL_HipY_joint",
            "HL_Knee_joint",

            "HR_HipX_joint",
            "HR_HipY_joint",
            "HR_Knee_joint"
        };

        for (size_t i = 0; i < joint_names_.size(); ++i) {
            joint_index_[joint_names_[i]] = i;
        }

        // Standing configuration
        q_des_standing_ =
            (Eigen::VectorXd(12) <<
                -0.02073, -0.67214, 1.32366,
                 0.01497, -0.67765, 1.33907,
                -0.02465, -0.64953, 1.32289,
                 0.01714, -0.65116, 1.33529
            ).finished();

        // FL standing joint configuration
        q_fl_standing_ =
            Eigen::Vector3d(
                q_des_standing_(0),
                q_des_standing_(1),
                q_des_standing_(2));

        // Compute exact FL standing foot position from FK
        fl_foot_start_ =
            kinematics_.forward(
                lite3_kinematics::Leg::FL,
                q_fl_standing_);

        q_des_current_fl_ = q_fl_standing_;
        qdot_des_current_fl_.setZero();

        RCLCPP_INFO(
            this->get_logger(),
            "Single-leg Cartesian trajectory controller started at 500 Hz");

        RCLCPP_INFO(
            this->get_logger(),
            "Controller: tau = G(q) + Kp(qd-q) + Kd(qdotd-qdot)");

        RCLCPP_INFO(
            this->get_logger(),
            "Kp = %.2f Nm/rad, Kd = %.2f Nms/rad",
            Kp_,
            Kd_);

        RCLCPP_INFO(
    this->get_logger(),
    "Periodic FL trajectory: "
    "step=%.1f mm, swing height=%.1f mm, "
    "cycle=%.2f s, duty=%.2f",
    step_length_ * 1000.0,
    swing_height_ * 1000.0,
    cycle_duration_,
    duty_factor_);

        RCLCPP_INFO(
            this->get_logger(),
            "FL initial foot = [%+.6f %+.6f %+.6f] m",
            fl_foot_start_(0),
            fl_foot_start_(1),
            fl_foot_start_(2));

        RCLCPP_INFO(
    this->get_logger(),
    "Trajectory: FL swing +30 mm X, +40 mm Z");
    }

private:

    // ========================================================
    // Joint state callback
    // ========================================================

    void jointStateCallback(
        const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (msg->name.empty() ||
            msg->position.size() != msg->name.size())
        {
            return;
        }

        q_.setZero();
        qdot_.setZero();

        for (size_t i = 0; i < msg->name.size(); ++i) {

            auto it = joint_index_.find(msg->name[i]);

            if (it == joint_index_.end()) {
                continue;
            }

            const int idx =
                static_cast<int>(it->second);

            q_(idx) = msg->position[i];

            if (i < msg->velocity.size()) {
                qdot_(idx) = msg->velocity[i];
            }
        }

        got_joint_state_ = true;
    }

    // ========================================================
    // Get one leg joint vector
    // ========================================================

    Eigen::Vector3d getLegQ(int leg) const
    {
        return Eigen::Vector3d(
            q_(3 * leg + 0),
            q_(3 * leg + 1),
            q_(3 * leg + 2));
    }

    // ========================================================
    // Get one leg joint velocity
    // ========================================================

    Eigen::Vector3d getLegQdot(int leg) const
    {
        return Eigen::Vector3d(
            qdot_(3 * leg + 0),
            qdot_(3 * leg + 1),
            qdot_(3 * leg + 2));
    }

    // ========================================================
    // Store leg torque
    // ========================================================

    void setLegTau(
        Eigen::VectorXd &tau,
        int leg,
        const Eigen::Vector3d &tau_leg)
    {
        tau(3 * leg + 0) = tau_leg(0);
        tau(3 * leg + 1) = tau_leg(1);
        tau(3 * leg + 2) = tau_leg(2);
    }

    // ========================================================
    // Torque saturation
    // ========================================================

    Eigen::Vector3d clampTorque(
        const Eigen::Vector3d &tau)
    {
        Eigen::Vector3d out = tau;

        // HipX
        out(0) = std::clamp(out(0), -24.0, 24.0);

        // HipY
        out(1) = std::clamp(out(1), -24.0, 24.0);

        // Knee
        out(2) = std::clamp(out(2), -36.0, 36.0);

        return out;
    }

    // ========================================================
    // Quintic interpolation
    // s(u) = 10u^3 - 15u^4 + 6u^5
    // ds/du = 30u^2 - 60u^3 + 30u^4
    // ========================================================

    double quinticPosition(double u) const
    {
        return
            10.0 * std::pow(u, 3) -
            15.0 * std::pow(u, 4) +
             6.0 * std::pow(u, 5);
    }

    double quinticVelocity(double u) const
    {
        return
            30.0 * std::pow(u, 2) -
            60.0 * std::pow(u, 3) +
            30.0 * std::pow(u, 4);
    }

    // ========================================================
    // Generate FL Cartesian trajectory
    // ========================================================

    void generateTrajectory(
    double t,
    Eigen::Vector3d &p_des,
    Eigen::Vector3d &pdot_des)
{
    // ========================================================
    // Default
    // ========================================================

    p_des = fl_foot_start_;
    pdot_des.setZero();

    // ========================================================
    // Pre-position:
    // standing -> front position
    //
    // Kita lakukan ini supaya trajectory periodik dimulai
    // dari posisi yang memang sudah disiapkan.
    // ========================================================

    const double x_front =
    fl_foot_start_(0) + 0.5 * step_length_;

const double x_back =
    fl_foot_start_(0) - 0.5 * step_length_;

    const double z_ground =
        fl_foot_start_(2);

    const double z_apex =
        z_ground + swing_height_;

    // --------------------------------------------------------
    // 0 -> pre-position
    // --------------------------------------------------------

    if (t <= preposition_duration_) {

        const double u =
            std::clamp(
                t / preposition_duration_,
                0.0,
                1.0);

        const double s =
            quinticPosition(u);

        const double dsdu =
            quinticVelocity(u);

        p_des(0) =
            fl_foot_start_(0) +
            (x_front - fl_foot_start_(0)) * s;

        p_des(1) =
            fl_foot_start_(1);

        p_des(2) =
            z_ground;

        pdot_des(0) =
            (x_front - fl_foot_start_(0)) *
            dsdu /
            preposition_duration_;

        pdot_des(1) = 0.0;
        pdot_des(2) = 0.0;

        return;
    }

    // ========================================================
    // Periodic gait phase
    // ========================================================

    const double gait_time =
        t - preposition_duration_;

    const double phase_time =
        std::fmod(
            gait_time,
            cycle_duration_);

    const double stance_duration =
        duty_factor_ *
        cycle_duration_;

    const double swing_duration =
        cycle_duration_ -
        stance_duration;

    // ========================================================
    // STANCE PHASE
    //
    // Foot moves from front -> back
    // ========================================================

    if (phase_time < stance_duration) {

        const double u =
            std::clamp(
                phase_time / stance_duration,
                0.0,
                1.0);

        const double s =
            quinticPosition(u);

        const double dsdu =
            quinticVelocity(u);

        p_des(0) =
            x_front +
            (x_back - x_front) * s;

        p_des(1) =
            fl_foot_start_(1);

        p_des(2) =
            z_ground;

        pdot_des(0) =
            (x_back - x_front) *
            dsdu /
            stance_duration;

        pdot_des(1) = 0.0;
        pdot_des(2) = 0.0;

        return;
    }

    // ========================================================
    // SWING PHASE
    //
    // Foot moves back -> front
    // while lifting to apex
    // ========================================================

    const double swing_time =
        phase_time -
        stance_duration;

    const double u =
        std::clamp(
            swing_time / swing_duration,
            0.0,
            1.0);

    const double s =
        quinticPosition(u);

    const double dsdu =
        quinticVelocity(u);

    // --------------------------------------------------------
    // X: back -> front
    // --------------------------------------------------------

    p_des(0) =
        x_back +
        (x_front - x_back) * s;

    pdot_des(0) =
        (x_front - x_back) *
        dsdu /
        swing_duration;

    // --------------------------------------------------------
    // Y: unchanged
    // --------------------------------------------------------

    p_des(1) =
        fl_foot_start_(1);

    pdot_des(1) = 0.0;

    // --------------------------------------------------------
    // Z: smooth bump
    //
    // b(u) = 16 u² (1-u)²
    //
    // b(0)=0
    // b(0.5)=1
    // b(1)=0
    // --------------------------------------------------------

    const double bump =
        16.0 *
        u * u *
        (1.0 - u) *
        (1.0 - u);

    const double dbdu =
        32.0 *
        u *
        (1.0 - u) *
        (1.0 - 2.0 * u);

    p_des(2) =
        z_ground +
        swing_height_ *
        bump;

    pdot_des(2) =
        swing_height_ *
        dbdu /
        swing_duration;
}
    // ========================================================
    // Control loop
    // ========================================================

    void controlLoop()
    {
        if (!got_joint_state_) {
            return;
        }

        using lite3_kinematics::Leg;

        const std::array<Leg, 4> legs = {
            Leg::FL,
            Leg::FR,
            Leg::HL,
            Leg::HR
        };

        // ----------------------------------------------------
        // Time
        // ----------------------------------------------------

        const double t =
            (this->now() - trajectory_start_time_).seconds();

        // ----------------------------------------------------
        // Generate FL Cartesian trajectory
        // ----------------------------------------------------

        Eigen::Vector3d p_fl_des;
        Eigen::Vector3d pdot_fl_des;

        generateTrajectory(
            t,
            p_fl_des,
            pdot_fl_des);

        // ----------------------------------------------------
        // FL IK
        // ----------------------------------------------------

        const Eigen::Vector3d q_fl =
            getLegQ(0);

        const auto ik_result =
            kinematics_.inverse(
                Leg::FL,
                p_fl_des,
                &q_fl);

        if (!ik_result.success) {

            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "FL IK failed");

            return;
        }

        q_des_current_fl_ =
            ik_result.q;

        // ----------------------------------------------------
        // Convert Cartesian velocity to joint velocity
        //
        // pdot = J qdot
        // qdot = J^-1 pdot
        // ----------------------------------------------------

        const Eigen::Matrix3d J_fl =
            kinematics_.jacobian(
                Leg::FL,
                q_des_current_fl_);

        Eigen::Vector3d qdot_des_fl =
            Eigen::Vector3d::Zero();

        const double determinant =
            J_fl.determinant();

        if (std::abs(determinant) > 1e-8) {

            qdot_des_fl =
                J_fl.fullPivLu().solve(
                    pdot_fl_des);

        } else {

            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "FL Jacobian near singularity");

        }

        qdot_des_current_fl_ =
            qdot_des_fl;

        // ----------------------------------------------------
        // Desired 12-joint state
        // ----------------------------------------------------

        Eigen::VectorXd q_des =
            q_des_standing_;

        Eigen::VectorXd qdot_des =
            Eigen::VectorXd::Zero(12);

        // Override FL
        q_des(0) = q_des_current_fl_(0);
        q_des(1) = q_des_current_fl_(1);
        q_des(2) = q_des_current_fl_(2);

        qdot_des(0) =
            qdot_des_current_fl_(0);

        qdot_des(1) =
            qdot_des_current_fl_(1);

        qdot_des(2) =
            qdot_des_current_fl_(2);

        // ----------------------------------------------------
        // Joint errors
        // ----------------------------------------------------

        const Eigen::VectorXd error =
            q_des - q_;

        const Eigen::VectorXd error_dot =
            qdot_des - qdot_;

        // ----------------------------------------------------
        // Torque
        // ----------------------------------------------------

        Eigen::VectorXd tau(12);
        tau.setZero();

        for (int i = 0; i < 4; ++i) {

            const Eigen::Vector3d q_leg =
                getLegQ(i);

            const Eigen::Vector3d tau_g =
                dynamics_.gravityTorque(
                    legs[i],
                    q_leg);

            const Eigen::Vector3d error_leg(
                error(3 * i + 0),
                error(3 * i + 1),
                error(3 * i + 2));

            const Eigen::Vector3d error_dot_leg(
                error_dot(3 * i + 0),
                error_dot(3 * i + 1),
                error_dot(3 * i + 2));

            Eigen::Vector3d tau_leg =
                tau_g +
                Kp_ * error_leg +
                Kd_ * error_dot_leg;

            tau_leg =
                clampTorque(tau_leg);

            setLegTau(
                tau,
                i,
                tau_leg);
        }

        // ----------------------------------------------------
        // Publish
        // ----------------------------------------------------

        std_msgs::msg::Float64MultiArray msg;
        msg.data.resize(12);

        for (int i = 0; i < 12; ++i) {
            msg.data[i] = tau(i);
        }

        command_pub_->publish(msg);

        // ----------------------------------------------------
        // Debug
        // ----------------------------------------------------

        if (++debug_counter_ >= 250) {

            debug_counter_ = 0;

            const Eigen::Vector3d p_fl_actual =
                kinematics_.forward(
                    Leg::FL,
                    q_fl);

            const Eigen::Vector3d foot_error =
                p_fl_des -
                p_fl_actual;

            
        RCLCPP_INFO(
    this->get_logger(),

    "t=%.2f | "
    "pd=[%+.4f %+.4f %+.4f] | "
    "p=[%+.4f %+.4f %+.4f] | "
    "e=[%+.2f %+.2f %+.2f] mm | "
    "|e|=%.2f mm",

    t,

    p_fl_des(0),
    p_fl_des(1),
    p_fl_des(2),

    p_fl_actual(0),
    p_fl_actual(1),
    p_fl_actual(2),

    foot_error(0) * 1000.0,
    foot_error(1) * 1000.0,
    foot_error(2) * 1000.0,

    foot_error.norm() * 1000.0);
        }
    }

    // ========================================================
    // ROS
    // ========================================================

    rclcpp::Publisher<
        std_msgs::msg::Float64MultiArray>::SharedPtr
        command_pub_;

    rclcpp::Subscription<
        sensor_msgs::msg::JointState>::SharedPtr
        joint_sub_;

    rclcpp::TimerBase::SharedPtr
        timer_;

    // ========================================================
    // Model
    // ========================================================

    lite3_dynamics::Lite3SingleLegDynamics
        dynamics_;

    lite3_kinematics::Lite3Kinematics
        kinematics_;

    // ========================================================
    // State
    // ========================================================

    Eigen::VectorXd q_ =
        Eigen::VectorXd::Zero(12);

    Eigen::VectorXd qdot_ =
        Eigen::VectorXd::Zero(12);

    Eigen::VectorXd q_des_standing_ =
        Eigen::VectorXd::Zero(12);

    Eigen::Vector3d q_fl_standing_ =
        Eigen::Vector3d::Zero();

    Eigen::Vector3d fl_foot_start_ =
        Eigen::Vector3d::Zero();

    Eigen::Vector3d q_des_current_fl_ =
        Eigen::Vector3d::Zero();

    Eigen::Vector3d qdot_des_current_fl_ =
        Eigen::Vector3d::Zero();

    // ========================================================
    // Trajectory parameters
    // ========================================================

    // ========================================================
// Gait trajectory parameters
// ========================================================

const double preposition_duration_ = 1.0;

// Complete stance + swing cycle
const double cycle_duration_ = 2.0;

// 60% stance, 40% swing
const double duty_factor_ = 0.60;

// Step length = 60 mm
const double step_length_ = 0.060;

// Swing height = 40 mm
const double swing_height_ = 0.040;

    // ========================================================
    // Controller gains
    // ========================================================

    const double Kp_ = 5.0;
    const double Kd_ = 0.5;

    // ========================================================
    // Joint map
    // ========================================================

    std::vector<std::string>
        joint_names_;

    std::unordered_map<std::string, size_t>
        joint_index_;

    bool got_joint_state_;

    int debug_counter_ = 0;

    rclcpp::Time trajectory_start_time_;
};


int main(
    int argc,
    char **argv)
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<
            SingleLegTrajectoryController>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}