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

class FourLegGaitController : public rclcpp::Node
{
public:
    FourLegGaitController()
    : Node("four_leg_gait_controller"),
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
                    &FourLegGaitController::jointStateCallback,
                    this,
                    std::placeholders::_1));

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(2),
            std::bind(
                &FourLegGaitController::controlLoop,
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

        // ----------------------------------------------------
        // Standing configuration
        // ----------------------------------------------------

        q_standing_ =
            (Eigen::VectorXd(12) <<
                -0.02073, -0.67214, 1.32366,
                 0.01497, -0.67765, 1.33907,
                -0.02465, -0.64953, 1.32289,
                 0.01714, -0.65116, 1.33529
            ).finished();

        // ----------------------------------------------------
        // Leg definitions
        // ----------------------------------------------------

        legs_ = {
            lite3_kinematics::Leg::FL,
            lite3_kinematics::Leg::FR,
            lite3_kinematics::Leg::HL,
            lite3_kinematics::Leg::HR
        };

        // ----------------------------------------------------
        // Phase offsets
        //
        // FL / HR : phase 0
        // FR / HL : phase 0.5
        // ----------------------------------------------------

        phase_offset_[0] = 0.0;  // FL
        phase_offset_[1] = 0.5;  // FR
        phase_offset_[2] = 0.5;  // HL
        phase_offset_[3] = 0.0;  // HR

        // ----------------------------------------------------
        // Calculate standing foot position
        // ----------------------------------------------------

        for (int i = 0; i < 4; ++i) {

            const Eigen::Vector3d q_leg(
                q_standing_(3 * i + 0),
                q_standing_(3 * i + 1),
                q_standing_(3 * i + 2));

            foot_standing_[i] =
                kinematics_.forward(
                    legs_[i],
                    q_leg);
        }

        RCLCPP_INFO(
            this->get_logger(),
            "Four-leg gait controller started at 500 Hz");

        RCLCPP_INFO(
            this->get_logger(),
            "Controller: "
            "tau = G(q) + Kp(qd-q) + Kd(qdotd-qdot)");

        RCLCPP_INFO(
            this->get_logger(),
            "Kp = %.2f Nm/rad, Kd = %.2f Nms/rad",
            Kp_,
            Kd_);

        RCLCPP_INFO(
            this->get_logger(),
            "Step length = %.1f mm",
            step_length_ * 1000.0);

        RCLCPP_INFO(
            this->get_logger(),
            "Swing height = %.1f mm",
            swing_height_ * 1000.0);

        RCLCPP_INFO(
            this->get_logger(),
            "Cycle = %.2f s, duty factor = %.2f",
            cycle_duration_,
            duty_factor_);

        RCLCPP_INFO(
            this->get_logger(),
            "Phase: FL=0 HR=0 FR=0.5 HL=0.5");
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

            const auto it =
                joint_index_.find(msg->name[i]);

            if (it == joint_index_.end()) {
                continue;
            }

            const int idx =
                static_cast<int>(it->second);

            q_(idx) =
                msg->position[i];

            if (i < msg->velocity.size()) {
                qdot_(idx) =
                    msg->velocity[i];
            }
        }

        got_joint_state_ = true;
    }

    // ========================================================
    // Get q for one leg
    // ========================================================

    Eigen::Vector3d getLegQ(int leg) const
    {
        return Eigen::Vector3d(
            q_(3 * leg + 0),
            q_(3 * leg + 1),
            q_(3 * leg + 2));
    }

    // ========================================================
    // Get qdot for one leg
    // ========================================================

    Eigen::Vector3d getLegQdot(int leg) const
    {
        return Eigen::Vector3d(
            qdot_(3 * leg + 0),
            qdot_(3 * leg + 1),
            qdot_(3 * leg + 2));
    }

    // ========================================================
    // Store torque
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

        out(0) =
            std::clamp(out(0), -24.0, 24.0);

        out(1) =
            std::clamp(out(1), -24.0, 24.0);

        out(2) =
            std::clamp(out(2), -36.0, 36.0);

        return out;
    }

    // ========================================================
    // Quintic trajectory
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
    // Swing height bump
    // ========================================================

    double swingBump(double u) const
    {
        return
            16.0 *
            u * u *
            (1.0 - u) *
            (1.0 - u);
    }

    double swingBumpDerivative(double u) const
    {
        return
            32.0 *
            u *
            (1.0 - u) *
            (1.0 - 2.0 * u);
    }

    // ========================================================
    // Generate trajectory for one leg
    //
    // phase in [0,1)
    //
    // 0 -> duty factor  : STANCE
    // duty -> 1         : SWING
    // ========================================================

    void generateLegTrajectory(
        int leg,
        double phase,
        Eigen::Vector3d &p_des,
        Eigen::Vector3d &pdot_des)
    {
        const Eigen::Vector3d &p0 =
            foot_standing_[leg];

        const double x_front =
            p0(0) +
            0.5 * step_length_;

        const double x_back =
            p0(0) -
            0.5 * step_length_;

        const double z_ground =
            p0(2);

        const double z_apex =
            z_ground +
            swing_height_;

        // ----------------------------------------------------
        // STANCE
        // ----------------------------------------------------

        if (phase < duty_factor_) {

            const double u =
                std::clamp(
                    phase / duty_factor_,
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
                p0(1);

            p_des(2) =
                z_ground;

            pdot_des(0) =
                (x_back - x_front) *
                dsdu /
                (duty_factor_ *
                 cycle_duration_);

            pdot_des(1) = 0.0;
            pdot_des(2) = 0.0;

            return;
        }

        // ----------------------------------------------------
        // SWING
        // ----------------------------------------------------

        const double swing_fraction =
            (phase - duty_factor_) /
            (1.0 - duty_factor_);

        const double u =
            std::clamp(
                swing_fraction,
                0.0,
                1.0);

        const double s =
            quinticPosition(u);

        const double dsdu =
            quinticVelocity(u);

        // X back -> front
        p_des(0) =
            x_back +
            (x_front - x_back) *
            s;

        pdot_des(0) =
            (x_front - x_back) *
            dsdu /
            ((1.0 - duty_factor_) *
             cycle_duration_);

        // Y constant
        p_des(1) =
            p0(1);

        pdot_des(1) = 0.0;

        // Z swing bump
        const double bump =
            swingBump(u);

        const double dbdu =
            swingBumpDerivative(u);

        p_des(2) =
            z_ground +
            swing_height_ *
            bump;

        pdot_des(2) =
            swing_height_ *
            dbdu /
            ((1.0 - duty_factor_) *
             cycle_duration_);
    }

    // ========================================================
    // Controller
    // ========================================================

    void controlLoop()
    {
        if (!got_joint_state_) {
            return;
        }

        const double t =
            (this->now() -
             trajectory_start_time_).seconds();

        // ----------------------------------------------------
        // Pre-position all feet at front position
        // ----------------------------------------------------

        if (t < preposition_duration_) {

            Eigen::VectorXd q_des =
                q_standing_;

            Eigen::VectorXd qdot_des =
                Eigen::VectorXd::Zero(12);

            const double u =
                std::clamp(
                    t / preposition_duration_,
                    0.0,
                    1.0);

            const double s =
                quinticPosition(u);

            const double dsdu =
                quinticVelocity(u);

            for (int leg = 0; leg < 4; ++leg) {

                const Eigen::Vector3d p0 =
                    foot_standing_[leg];

                const double x_front =
                    p0(0) +
                    0.5 * step_length_;

                Eigen::Vector3d p_des =
                    p0;

                Eigen::Vector3d pdot_des =
                    Eigen::Vector3d::Zero();

                p_des(0) =
                    p0(0) +
                    (x_front - p0(0)) * s;

                pdot_des(0) =
                    (x_front - p0(0)) *
                    dsdu /
                    preposition_duration_;

                const Eigen::Vector3d q_leg =
                    getLegQ(leg);

                const auto ik =
                    kinematics_.inverse(
                        legs_[leg],
                        p_des,
                        &q_leg);

                if (!ik.success) {
                    continue;
                }

                q_des.segment<3>(3 * leg) =
                    ik.q;

                const Eigen::Matrix3d J =
                    kinematics_.jacobian(
                        legs_[leg],
                        ik.q);

                if (std::abs(J.determinant()) > 1e-8) {

                    qdot_des.segment<3>(3 * leg) =
                        J.fullPivLu().solve(
                            pdot_des);
                }
            }

            publishTorque(
                q_des,
                qdot_des);

            return;
        }

        // ----------------------------------------------------
        // Periodic trajectory
        // ----------------------------------------------------

        const double gait_time =
            t - preposition_duration_;

        double phase_base =
            std::fmod(
                gait_time / cycle_duration_,
                1.0);

        if (phase_base < 0.0) {
            phase_base += 1.0;
        }

        Eigen::VectorXd q_des =
            q_standing_;

        Eigen::VectorXd qdot_des =
            Eigen::VectorXd::Zero(12);

        for (int leg = 0; leg < 4; ++leg) {

            double phase =
                phase_base +
                phase_offset_[leg];

            phase =
                std::fmod(
                    phase,
                    1.0);

            if (phase < 0.0) {
                phase += 1.0;
            }

            Eigen::Vector3d p_des;
            Eigen::Vector3d pdot_des;

            generateLegTrajectory(
                leg,
                phase,
                p_des,
                pdot_des);

            const Eigen::Vector3d q_leg =
                getLegQ(leg);

            const auto ik =
                kinematics_.inverse(
                    legs_[leg],
                    p_des,
                    &q_leg);

            if (!ik.success) {

                RCLCPP_WARN_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    1000,
                    "IK failed for leg %d",
                    leg);

                continue;
            }

            q_des.segment<3>(3 * leg) =
                ik.q;

            const Eigen::Matrix3d J =
                kinematics_.jacobian(
                    legs_[leg],
                    ik.q);

            if (std::abs(J.determinant()) > 1e-8) {

                qdot_des.segment<3>(3 * leg) =
                    J.fullPivLu().solve(
                        pdot_des);
            }
        }

        publishTorque(
            q_des,
            qdot_des);

        // ----------------------------------------------------
        // Debug
        // ----------------------------------------------------

        if (++debug_counter_ >= 250) {

            debug_counter_ = 0;

            double max_error = 0.0;

            for (int leg = 0; leg < 4; ++leg) {

                const Eigen::Vector3d q_leg =
                    getLegQ(leg);

                Eigen::Vector3d p_actual =
                    kinematics_.forward(
                        legs_[leg],
                        q_leg);

                double phase =
                    phase_base +
                    phase_offset_[leg];

                phase =
                    std::fmod(
                        phase,
                        1.0);

                if (phase < 0.0) {
                    phase += 1.0;
                }

                Eigen::Vector3d p_des;
                Eigen::Vector3d pdot_des;

                generateLegTrajectory(
                    leg,
                    phase,
                    p_des,
                    pdot_des);

                const double error =
                    (p_des - p_actual).norm();

                max_error =
                    std::max(
                        max_error,
                        error);
            }

            RCLCPP_INFO(
                this->get_logger(),
                "t=%.2f | gait_phase=%.3f | "
                "max foot error=%.2f mm",
                t,
                phase_base,
                max_error * 1000.0);

            double foot_error_mm[4];
int worst_leg = 0;

for (int leg = 0; leg < 4; ++leg) {

    const Eigen::Vector3d q_leg =
        getLegQ(leg);

    const Eigen::Vector3d p_actual =
        kinematics_.forward(
            legs_[leg],
            q_leg);

    double phase =
        phase_base + phase_offset_[leg];

    phase = std::fmod(phase, 1.0);

    if (phase < 0.0) {
        phase += 1.0;
    }

    Eigen::Vector3d p_des;
    Eigen::Vector3d pdot_des;

    generateLegTrajectory(
        leg,
        phase,
        p_des,
        pdot_des);

    foot_error_mm[leg] =
        (p_des - p_actual).norm() * 1000.0;

    if (foot_error_mm[leg] >
        foot_error_mm[worst_leg]) {
        worst_leg = leg;
    }
}

RCLCPP_INFO(
    this->get_logger(),
    "t=%.2f phase=%.3f | "
    "FL=%.2f FR=%.2f HL=%.2f HR=%.2f | "
    "MAX=%.2f mm",
    t,
    phase_base,
    foot_error_mm[0],
    foot_error_mm[1],
    foot_error_mm[2],
    foot_error_mm[3],
    foot_error_mm[worst_leg]);
        }
    }

    // ========================================================
    // Publish torque
    // ========================================================

    void publishTorque(
        const Eigen::VectorXd &q_des,
        const Eigen::VectorXd &qdot_des)
    {
        using lite3_kinematics::Leg;

        Eigen::VectorXd error =
            q_des - q_;

        Eigen::VectorXd error_dot =
            qdot_des - qdot_;

        Eigen::VectorXd tau(12);
        tau.setZero();

        for (int leg = 0; leg < 4; ++leg) {

            const Eigen::Vector3d q_leg =
                getLegQ(leg);

            const Eigen::Vector3d tau_g =
                dynamics_.gravityTorque(
                    legs_[leg],
                    q_leg);

            const Eigen::Vector3d e(
                error(3 * leg + 0),
                error(3 * leg + 1),
                error(3 * leg + 2));

            const Eigen::Vector3d edot(
                error_dot(3 * leg + 0),
                error_dot(3 * leg + 1),
                error_dot(3 * leg + 2));

            Eigen::Vector3d tau_leg =
                tau_g +
                Kp_ * e +
                Kd_ * edot;

            tau_leg =
                clampTorque(tau_leg);

            setLegTau(
                tau,
                leg,
                tau_leg);
        }

        std_msgs::msg::Float64MultiArray msg;
        msg.data.resize(12);

        for (int i = 0; i < 12; ++i) {
            msg.data[i] = tau(i);
        }

        command_pub_->publish(msg);
    }

    // ========================================================
    // ROS interfaces
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
    // Models
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

    Eigen::VectorXd q_standing_ =
        Eigen::VectorXd::Zero(12);

    std::array<Eigen::Vector3d, 4>
        foot_standing_;

    std::array<lite3_kinematics::Leg, 4>
        legs_;

    std::array<double, 4>
        phase_offset_;

    bool got_joint_state_;

    int debug_counter_ = 0;

    rclcpp::Time trajectory_start_time_;

    // ========================================================
// Joint map
// ========================================================

std::vector<std::string>
    joint_names_;

std::unordered_map<std::string, size_t>
    joint_index_;

    // ========================================================
    // Trajectory parameters
    // ========================================================

    const double preposition_duration_ = 1.0;

    const double cycle_duration_ = 2.0;

    const double duty_factor_ = 0.60;

    const double step_length_ = 0.040;

    const double swing_height_ = 0.025;

    // ========================================================
    // Controller gains
    // ========================================================

    const double Kp_ = 5.0;

    const double Kd_ = 0.5;
};


int main(
    int argc,
    char **argv)
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<
            FourLegGaitController>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}