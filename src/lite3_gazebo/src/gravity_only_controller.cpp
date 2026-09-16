#include <algorithm>
#include <array>
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

class GravityOnlyController : public rclcpp::Node
{
public:
    GravityOnlyController()
    : Node("gravity_only_controller"),
      got_joint_state_(false)
    {
        command_pub_ =
            this->create_publisher<std_msgs::msg::Float64MultiArray>(
                "/lite3_effort_controller/commands", 10);

        joint_sub_ =
            this->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states",
                10,
                std::bind(
                    &GravityOnlyController::jointStateCallback,
                    this,
                    std::placeholders::_1));

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(2),
            std::bind(
                &GravityOnlyController::controlLoop,
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

        RCLCPP_INFO(
            this->get_logger(),
            "Gravity + P controller started at 500 Hz");

        RCLCPP_INFO(
            this->get_logger(),
            "Controller: tau = G(q) + Kp(qd - q)");

        RCLCPP_INFO(
            this->get_logger(),
            "Kp = %.2f Nm/rad",
            Kp_);
    }

private:

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

            q_(idx) =
                msg->position[i];

            if (i < msg->velocity.size()) {
                qdot_(idx) =
                    msg->velocity[i];
            }
        }

        got_joint_state_ = true;
    }

    Eigen::Vector3d getLegQ(int leg) const
    {
        return Eigen::Vector3d(
            q_(3 * leg + 0),
            q_(3 * leg + 1),
            q_(3 * leg + 2));
    }

    void setLegTau(
        Eigen::VectorXd &tau,
        int leg,
        const Eigen::Vector3d &tau_leg)
    {
        tau(3 * leg + 0) = tau_leg(0);
        tau(3 * leg + 1) = tau_leg(1);
        tau(3 * leg + 2) = tau_leg(2);
    }

    Eigen::Vector3d clampTorque(
        const Eigen::Vector3d &tau)
    {
        Eigen::Vector3d out = tau;

        // HipX / HipY
        out(0) =
            std::clamp(out(0), -24.0, 24.0);

        out(1) =
            std::clamp(out(1), -24.0, 24.0);

        // Knee
        out(2) =
            std::clamp(out(2), -36.0, 36.0);

        return out;
    }

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
        // Position error
        // ----------------------------------------------------

        const Eigen::VectorXd error =
            q_des_ - q_;

        // ----------------------------------------------------
        // Allocate torque vector
        // ----------------------------------------------------

        Eigen::VectorXd tau(12);
        tau.setZero();

        // ----------------------------------------------------
        // Calculate gravity + P torque for each leg
        // ----------------------------------------------------

        for (int i = 0; i < 4; ++i) {

            const Eigen::Vector3d q_leg =
                getLegQ(i);

            // Gravity compensation
            const Eigen::Vector3d tau_g =
                dynamics_.gravityTorque(
                    legs[i],
                    q_leg);

            // P feedback for this leg
            const Eigen::Vector3d error_leg(
                error(3 * i + 0),
                error(3 * i + 1),
                error(3 * i + 2));

            Eigen::Vector3d tau_leg =
                tau_g + Kp_ * error_leg;

            // Torque saturation
            tau_leg =
                clampTorque(tau_leg);

            // Store
            setLegTau(
                tau,
                i,
                tau_leg);
        }

        // ----------------------------------------------------
        // Publish torque
        // ----------------------------------------------------

        std_msgs::msg::Float64MultiArray msg;
        msg.data.resize(12);

        for (int i = 0; i < 12; ++i) {
            msg.data[i] = tau(i);
        }

        command_pub_->publish(msg);

        // ----------------------------------------------------
        // Debug output ~2 Hz
        // ----------------------------------------------------

        if (++debug_counter_ >= 250) {

            debug_counter_ = 0;

            RCLCPP_INFO(
                this->get_logger(),

                "FL q=[%+.4f %+.4f %+.4f] "
                "e=[%+.4f %+.4f %+.4f] "
                "tau=[%+.4f %+.4f %+.4f]",

                q_(0),
                q_(1),
                q_(2),

                error(0),
                error(1),
                error(2),

                tau(0),
                tau(1),
                tau(2));
        }
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
    // Dynamics
    // ========================================================

    lite3_dynamics::Lite3SingleLegDynamics
        dynamics_;

    // ========================================================
    // State
    // ========================================================

    Eigen::VectorXd q_ =
        Eigen::VectorXd::Zero(12);

    Eigen::VectorXd qdot_ =
        Eigen::VectorXd::Zero(12);

    // ========================================================
    // Desired standing configuration
    // ========================================================

    const Eigen::VectorXd q_des_ =
        (Eigen::VectorXd(12) <<
            -0.02073, -0.67214, 1.32366,
             0.01497, -0.67765, 1.33907,
            -0.02465, -0.64953, 1.32289,
             0.01714, -0.65116, 1.33529
        ).finished();

    // ========================================================
    // Control gain
    // ========================================================

    const double Kp_ = 5.0;

    // ========================================================
    // Joint map
    // ========================================================

    std::vector<std::string>
        joint_names_;

    std::unordered_map<std::string, size_t>
        joint_index_;

    bool got_joint_state_;

    int debug_counter_ = 0;
};


int main(
    int argc,
    char **argv)
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<
            GravityOnlyController>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}