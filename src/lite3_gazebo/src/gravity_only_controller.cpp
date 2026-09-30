#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "lite3_dynamics/dynamics.hpp"
#include "lite3_kinematics/kinematics.hpp"

using namespace std::chrono_literals;

class StaticEquilibriumController : public rclcpp::Node
{
public:

    StaticEquilibriumController()
    : Node("gravity_only_controller"),
      got_joint_state_(false),
      effort_valid_(false)
    {
        // ============================================================
        // Parameters
        // ============================================================

        this->declare_parameter<double>("kp", 1.0);
        this->declare_parameter<double>("kd", 1.0);

        this->declare_parameter<bool>(
            "start_handover",
            false);

        this->declare_parameter<double>(
            "handover_duration",
            1.5);

        this->declare_parameter<double>(
            "diagnostic_period",
            0.5);

        this->declare_parameter<double>(
            "ready_torque_error_threshold",
            0.5);

        this->get_parameter("kp", Kp_);
        this->get_parameter("kd", Kd_);
        this->get_parameter(
            "start_handover",
            start_handover_);
        this->get_parameter(
            "handover_duration",
            handover_duration_);
        this->get_parameter(
            "diagnostic_period",
            diagnostic_period_);
        this->get_parameter(
            "ready_torque_error_threshold",
            ready_torque_error_threshold_);

        // ============================================================
        // Publisher
        // ============================================================

        command_pub_ =
            this->create_publisher<
                std_msgs::msg::Float64MultiArray>(
                "/lite3_effort_controller/commands",
                10);

        // ============================================================
        // Joint state
        // ============================================================

        joint_sub_ =
            this->create_subscription<
                sensor_msgs::msg::JointState>(
                "/joint_states",
                20,
                std::bind(
                    &StaticEquilibriumController::jointStateCallback,
                    this,
                    std::placeholders::_1));

        // ============================================================
        // Controller timer
        // ============================================================

        timer_ =
            this->create_wall_timer(
                2ms,
                std::bind(
                    &StaticEquilibriumController::controlLoop,
                    this));

        // ============================================================
        // Joint order
        // ============================================================

        joint_names_ =
        {
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

        for (std::size_t i = 0;
             i < joint_names_.size();
             ++i)
        {
            joint_index_[joint_names_[i]] = i;
        }

        // ============================================================
        // CSV
        // ============================================================

        csv_.open(
            "/home/lexion/lite3_ws/data/handover_torque_diagnostic.csv",
            std::ios::out | std::ios::trunc);

        if (csv_.is_open())
        {
            writeCsvHeader();
        }
        else
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Tidak bisa membuka CSV diagnostic.");
        }

        // ============================================================
        // Information
        // ============================================================

        RCLCPP_INFO(
            this->get_logger(),
            "==============================================");

        RCLCPP_INFO(
            this->get_logger(),
            "Lite3 Static Equilibrium Torque Diagnostic");

        RCLCPP_INFO(
            this->get_logger(),
            "tau_static = G - J^T F");

        RCLCPP_INFO(
            this->get_logger(),
            "tau_target = tau_static + Kp(qd-q) - Kd*qdot");

        RCLCPP_INFO(
            this->get_logger(),
            "Kp = %.3f",
            Kp_);

        RCLCPP_INFO(
            this->get_logger(),
            "Kd = %.3f",
            Kd_);

        RCLCPP_INFO(
            this->get_logger(),
            "Handover = %s",
            start_handover_ ? "TRUE" : "FALSE");

        RCLCPP_INFO(
            this->get_logger(),
            "Diagnostic period = %.3f s",
            diagnostic_period_);

        RCLCPP_INFO(
            this->get_logger(),
            "==============================================");
    }

    ~StaticEquilibriumController()
    {
        if (csv_.is_open())
        {
            csv_.close();
        }
    }

private:

    // ================================================================
    // Joint state callback
    // ================================================================

    void jointStateCallback(
        const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (msg->name.empty())
        {
            return;
        }

        for (std::size_t i = 0;
             i < msg->name.size();
             ++i)
        {
            auto it =
                joint_index_.find(msg->name[i]);

            if (it == joint_index_.end())
            {
                continue;
            }

            const int idx =
                static_cast<int>(it->second);

            if (i < msg->position.size())
            {
                q_(idx) =
                    msg->position[i];
            }

            if (i < msg->velocity.size())
            {
                qdot_(idx) =
                    msg->velocity[i];
            }

            if (i < msg->effort.size())
            {
                latest_effort_(idx) =
                    msg->effort[i];

                effort_valid_ = true;
            }
        }

        got_joint_state_ = true;
    }

    // ================================================================
    // Get one leg q
    // ================================================================

    Eigen::Vector3d getLegQ(int leg) const
    {
        return q_.segment<3>(3 * leg);
    }

    // ================================================================
    // Store leg torque
    // ================================================================

    void setLegTau(
        Eigen::VectorXd & tau,
        int leg,
        const Eigen::Vector3d & tau_leg)
    {
        tau.segment<3>(3 * leg) = tau_leg;
    }

    // ================================================================
    // Torque saturation
    // ================================================================

    Eigen::Vector3d clampTorque(
        const Eigen::Vector3d & tau)
    {
        Eigen::Vector3d out = tau;

        // HipX
        out(0) =
            std::clamp(
                out(0),
                -24.0,
                24.0);

        // HipY
        out(1) =
            std::clamp(
                out(1),
                -24.0,
                24.0);

        // Knee
        out(2) =
            std::clamp(
                out(2),
                -36.0,
                36.0);

        return out;
    }

    // ================================================================
    // Static vertical contact loads
    // ================================================================

    Eigen::Vector4d solveContactLoads(
        const std::array<Eigen::Vector3d, 4> & feet)
    {
        constexpr double MASS = 11.9376;
        constexpr double G = 9.81;

        const double weight =
            MASS * G;

        Eigen::Matrix<double, 3, 4> A;

        A <<
            1.0, 1.0, 1.0, 1.0,

            feet[0].x(),
            feet[1].x(),
            feet[2].x(),
            feet[3].x(),

            feet[0].y(),
            feet[1].y(),
            feet[2].y(),
            feet[3].y();

        Eigen::Vector3d b;

        b <<
            weight,
            0.0,
            0.0;

        Eigen::Matrix3d AAt =
            A * A.transpose();

        if (std::abs(AAt.determinant()) < 1e-10)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "Contact-load matrix singular.");

            return
                Eigen::Vector4d::Constant(
                    weight / 4.0);
        }

        return
            A.transpose() *
            AAt.ldlt().solve(b);
    }

    // ================================================================
    // Main control loop
    // ================================================================

    void controlLoop()
    {
        if (!got_joint_state_)
        {
            return;
        }

        using lite3_kinematics::Leg;

        const std::array<Leg, 4> legs =
        {
            Leg::FL,
            Leg::FR,
            Leg::HL,
            Leg::HR
        };

        // ------------------------------------------------------------
        // Standing error
        // ------------------------------------------------------------

        const Eigen::VectorXd error =
            q_des_ - q_;

        const Eigen::VectorXd error_dot =
            -qdot_;

        // ------------------------------------------------------------
        // Foot position
        // ------------------------------------------------------------

        std::array<Eigen::Vector3d, 4> feet;

        for (int i = 0; i < 4; ++i)
        {
            feet[i] =
                kinematics_.forward(
                    legs[i],
                    getLegQ(i));
        }

        // ------------------------------------------------------------
        // Contact load distribution
        // ------------------------------------------------------------

        const Eigen::Vector4d Fz =
            solveContactLoads(feet);

        // ------------------------------------------------------------
        // Torque vectors
        // ------------------------------------------------------------

        Eigen::VectorXd tau_static =
            Eigen::VectorXd::Zero(12);

        Eigen::VectorXd tau_target =
            Eigen::VectorXd::Zero(12);

        // ------------------------------------------------------------
        // Per leg
        // ------------------------------------------------------------

        for (int i = 0; i < 4; ++i)
        {
            const Eigen::Vector3d q_leg =
                getLegQ(i);

            const Eigen::Vector3d qdot_leg =
                qdot_.segment<3>(3 * i);

            // Gravity
            const Eigen::Vector3d tau_g =
                dynamics_.gravityTorque(
                    legs[i],
                    q_leg);

            // Contact
            const Eigen::Vector3d F_contact(
                0.0,
                0.0,
                Fz(i));

            const Eigen::Vector3d tau_contact =
                dynamics_.contactTorque(
                    legs[i],
                    q_leg,
                    F_contact);

            // Static equilibrium
            const Eigen::Vector3d tau_static_leg =
                tau_g - tau_contact;

            // PD
            const Eigen::Vector3d e =
                error.segment<3>(3 * i);

            const Eigen::Vector3d edot =
                error_dot.segment<3>(3 * i);

            const Eigen::Vector3d tau_pd =
                Kp_ * e
                +
                Kd_ * edot;

            // Target effort
            Eigen::Vector3d tau_target_leg =
                tau_static_leg +
                tau_pd;

            // Saturation
            tau_target_leg =
                clampTorque(
                    tau_target_leg);

            setLegTau(
                tau_static,
                i,
                tau_static_leg);

            setLegTau(
                tau_target,
                i,
                tau_target_leg);
        }

        // ------------------------------------------------------------
        // Save latest target
        // ------------------------------------------------------------

        tau_static_ = tau_static;
        tau_target_ = tau_target;

        // ------------------------------------------------------------
        // Publish target to effort controller
        //
        // IMPORTANT:
        // ForwardCommandController can receive this while inactive.
        // It does NOT mean it is currently actuating the robot.
        // ------------------------------------------------------------

        publishTorque(tau_target);

        // ------------------------------------------------------------
        // Diagnostic
        // ------------------------------------------------------------

        const double now_sec =
            this->now().seconds();

        if (
            last_diagnostic_time_ < 0.0 ||
            now_sec - last_diagnostic_time_
                >= diagnostic_period_)
        {
            last_diagnostic_time_ = now_sec;

            printDiagnostic(
                now_sec,
                Fz);
        }

        writeCsv(
            now_sec,
            tau_static,
            tau_target,
            Fz);
    }

    // ================================================================
    // Publish torque
    // ================================================================

    void publishTorque(
        const Eigen::VectorXd & tau)
    {
        std_msgs::msg::Float64MultiArray msg;

        msg.data.resize(12);

        for (int i = 0; i < 12; ++i)
        {
            msg.data[
                static_cast<std::size_t>(i)]
                = tau(i);
        }

        command_pub_->publish(msg);
    }

    // ================================================================
    // Diagnostic print
    // ================================================================

    void printDiagnostic(
        double time,
        const Eigen::Vector4d & Fz)
    {
        std::cout << "\n";
        std::cout
            << "============================================================\n";

        std::cout
            << "TIME = "
            << std::fixed
            << std::setprecision(3)
            << time
            << " s\n";

        std::cout
            << "Kp = "
            << Kp_
            << " | Kd = "
            << Kd_
            << "\n";

        std::cout
            << "Fz = [ "
            << Fz(0) << " "
            << Fz(1) << " "
            << Fz(2) << " "
            << Fz(3) << " ] N\n";

        if (!effort_valid_)
        {
            std::cout
                << "\nWARNING: /joint_states effort belum tersedia.\n";

            std::cout
                << "Tidak bisa menghitung selisih torque aktual.\n";
        }
        else
        {
            Eigen::VectorXd delta =
                latest_effort_ - tau_target_;

            double max_delta =
                delta.cwiseAbs().maxCoeff();

            double rms_delta =
                std::sqrt(
                    delta.squaredNorm() / 12.0);

            bool ready =
                max_delta <=
                ready_torque_error_threshold_;

            std::cout
                << "\nMAX |tau_actual - tau_target| = "
                << max_delta
                << " Nm\n";

            std::cout
                << "RMS |tau_actual - tau_target| = "
                << rms_delta
                << " Nm\n";

            std::cout
                << "TORQUE READY = "
                << (ready ? "YES" : "NO")
                << "\n";

            printVector(
                "tau_actual",
                latest_effort_);

            printVector(
                "tau_static",
                tau_static_);

            printVector(
                "tau_target",
                tau_target_);

            printVector(
                "delta_tau",
                delta);
        }

        std::cout
            << "============================================================\n";
    }

    // ================================================================
    // Vector print
    // ================================================================

    void printVector(
        const std::string & label,
        const Eigen::VectorXd & v)
    {
        std::cout
            << label
            << " = [ ";

        for (int i = 0; i < v.size(); ++i)
        {
            std::cout
                << std::fixed
                << std::setprecision(3)
                << v(i);

            if (i < v.size() - 1)
            {
                std::cout << " ";
            }
        }

        std::cout
            << " ]\n";
    }

    // ================================================================
    // CSV header
    // ================================================================

    void writeCsvHeader()
    {
        csv_
            << "time";

        for (int i = 0; i < 12; ++i)
        {
            csv_
                << ",tau_static_" << i
                << ",tau_target_" << i
                << ",tau_actual_" << i
                << ",delta_tau_" << i;
        }

        csv_
            << ",Fz_FL"
            << ",Fz_FR"
            << ",Fz_HL"
            << ",Fz_HR";

        csv_
            << "\n";

        csv_.flush();
    }

    // ================================================================
    // CSV
    // ================================================================

    void writeCsv(
        double time,
        const Eigen::VectorXd & tau_static,
        const Eigen::VectorXd & tau_target,
        const Eigen::Vector4d & Fz)
    {
        if (!csv_.is_open())
        {
            return;
        }

        csv_
            << std::fixed
            << std::setprecision(9)
            << time;

        for (int i = 0; i < 12; ++i)
        {
            double actual =
                effort_valid_
                ? latest_effort_(i)
                : 0.0;

            double delta =
                effort_valid_
                ? latest_effort_(i)
                  - tau_target(i)
                : 0.0;

            csv_
                << ","
                << tau_static(i)
                << ","
                << tau_target(i)
                << ","
                << actual
                << ","
                << delta;
        }

        csv_
            << ","
            << Fz(0)
            << ","
            << Fz(1)
            << ","
            << Fz(2)
            << ","
            << Fz(3)
            << "\n";

        csv_.flush();
    }

    // ================================================================
    // ROS interfaces
    // ================================================================

    rclcpp::Publisher<
        std_msgs::msg::Float64MultiArray
    >::SharedPtr command_pub_;

    rclcpp::Subscription<
        sensor_msgs::msg::JointState
    >::SharedPtr joint_sub_;

    rclcpp::TimerBase::SharedPtr timer_;

    // ================================================================
    // Dynamics
    // ================================================================

    lite3_dynamics::Lite3SingleLegDynamics
        dynamics_;

    lite3_kinematics::Lite3Kinematics
        kinematics_;

    // ================================================================
    // State
    // ================================================================

    Eigen::VectorXd q_ =
        Eigen::VectorXd::Zero(12);

    Eigen::VectorXd qdot_ =
        Eigen::VectorXd::Zero(12);

    Eigen::VectorXd latest_effort_ =
        Eigen::VectorXd::Zero(12);

    Eigen::VectorXd tau_static_ =
        Eigen::VectorXd::Zero(12);

    Eigen::VectorXd tau_target_ =
        Eigen::VectorXd::Zero(12);

    // ================================================================
    // Standing configuration
    // ================================================================

    const Eigen::VectorXd q_des_ =
        (Eigen::VectorXd(12) <<

            -0.02073,
            -0.67214,
             1.32366,

             0.01497,
            -0.67765,
             1.33907,

            -0.02465,
            -0.64953,
             1.32289,

             0.01714,
            -0.65116,
             1.33529

        ).finished();

    // ================================================================
    // Parameters
    // ================================================================

    double Kp_ = 1.0;
    double Kd_ = 1.0;

    bool start_handover_ = false;

    double handover_duration_ = 1.5;

    double diagnostic_period_ = 0.5;

    double ready_torque_error_threshold_ = 0.5;

    // ================================================================
    // Joint map
    // ================================================================

    std::vector<std::string>
        joint_names_;

    std::unordered_map<
        std::string,
        std::size_t
    > joint_index_;

    // ================================================================
    // Flags
    // ================================================================

    bool got_joint_state_;

    bool effort_valid_;

    double last_diagnostic_time_ = -1.0;

    // ================================================================
    // CSV
    // ================================================================

    std::ofstream csv_;
};


int main(
    int argc,
    char ** argv)
{
    rclcpp::init(
        argc,
        argv);

    auto node =
        std::make_shared<
            StaticEquilibriumController>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}