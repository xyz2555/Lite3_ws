#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
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


class ThreeLegStabilityTest : public rclcpp::Node
{
public:

    enum class Phase
    {
        WAITING,

        PREPARE,

        BODY_SHIFT,

        SHIFT_HOLD,

        FL_LIFT,

        FL_HOLD,

        FL_LOWER,

        BODY_RETURN,

        DONE
    };


    ThreeLegStabilityTest()
    : Node("three_leg_stability_test"),
      phase_(Phase::WAITING),
      experiment_started_(false),
      got_joint_state_(false),
      initialized_(false)
    {

        // =====================================================
        // PARAMETERS
        // =====================================================

        declare_parameter<bool>(
            "start_experiment",
            false);

        declare_parameter<double>(
            "kp",
            2.0);

        declare_parameter<double>(
            "kd",
            0.5);

        declare_parameter<double>(
            "publish_rate",
            500.0);

        // Desired body displacement.
        //
        // Negative X = backward
        // Negative Y = toward HR/FR side
        //
        // 10 mm is intentionally small.

        declare_parameter<double>(
            "body_shift_x",
            -0.010);

        declare_parameter<double>(
            "body_shift_y",
            -0.010);

        declare_parameter<double>(
            "body_shift_duration",
            4.0);

        declare_parameter<double>(
            "shift_hold_duration",
            1.5);

        // FL vertical lift.
        //
        // Start VERY small.
        //
        // This is not a stair trajectory.
        // It is only a support-stability experiment.

        declare_parameter<double>(
            "fl_lift_height",
            0.005);

        declare_parameter<double>(
            "fl_lift_duration",
            2.5);

        declare_parameter<double>(
            "fl_hold_duration",
            1.0);

        declare_parameter<double>(
            "prepare_duration",
            2.0);


        kp_ =
            get_parameter("kp").as_double();

        kd_ =
            get_parameter("kd").as_double();

        publish_rate_ =
            get_parameter("publish_rate").as_double();

        body_shift_x_ =
            get_parameter("body_shift_x").as_double();

        body_shift_y_ =
            get_parameter("body_shift_y").as_double();

        body_shift_duration_ =
            get_parameter("body_shift_duration").as_double();

        shift_hold_duration_ =
            get_parameter("shift_hold_duration").as_double();

        fl_lift_height_ =
            get_parameter("fl_lift_height").as_double();

        fl_lift_duration_ =
            get_parameter("fl_lift_duration").as_double();

        fl_hold_duration_ =
            get_parameter("fl_hold_duration").as_double();

        prepare_duration_ =
            get_parameter("prepare_duration").as_double();


        // =====================================================
        // JOINT ORDER
        // =====================================================

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
            joint_index_[
                joint_names_[i]] = i;
        }


        // =====================================================
        // LEG DEFINITIONS
        // =====================================================

        legs_ =
        {
            lite3_kinematics::Leg::FL,
            lite3_kinematics::Leg::FR,
            lite3_kinematics::Leg::HL,
            lite3_kinematics::Leg::HR
        };


        // =====================================================
        // VALIDATED STANDING CONFIGURATION
        // =====================================================

        q_standing_ =
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


        q_actual_ =
            Eigen::VectorXd::Zero(12);

        qdot_actual_ =
            Eigen::VectorXd::Zero(12);

        q_des_ =
            q_standing_;

        qdot_des_ =
            Eigen::VectorXd::Zero(12);


        // =====================================================
        // CALCULATE NOMINAL FOOT POSITIONS
        // =====================================================

        for (int leg = 0;
             leg < 4;
             ++leg)
        {
            const Eigen::Vector3d q_leg =
                q_standing_.segment<3>(
                    3 * leg);

            foot_nominal_[leg] =
                kinematics_.forward(
                    legs_[leg],
                    q_leg);
        }


        // =====================================================
        // ROS PUBLISHER
        // =====================================================

        effort_pub_ =
            create_publisher<
                std_msgs::msg::Float64MultiArray>(
                    "/lite3_effort_controller/commands",
                    10);


        // =====================================================
        // JOINT STATE
        // =====================================================

        joint_sub_ =
            create_subscription<
                sensor_msgs::msg::JointState>(
                    "/joint_states",
                    rclcpp::SensorDataQoS(),

                    std::bind(
                        &ThreeLegStabilityTest::jointStateCallback,
                        this,
                        std::placeholders::_1));


        // =====================================================
        // TIMER
        // =====================================================

        const auto period =
            std::chrono::duration_cast<
                std::chrono::nanoseconds>(
                    std::chrono::duration<double>(
                        1.0 / publish_rate_));


        timer_ =
            create_wall_timer(
                period,

                std::bind(
                    &ThreeLegStabilityTest::controlLoop,
                    this));


        RCLCPP_INFO(
            get_logger(),
            "==============================================");

        RCLCPP_INFO(
            get_logger(),
            "Three-leg stability test");

        RCLCPP_INFO(
            get_logger(),
            "Controller:");

        RCLCPP_INFO(
            get_logger(),
            "tau = G(q) + Kp(qd-q) + Kd(qdotd-qdot)");

        RCLCPP_INFO(
            get_logger(),
            "Kp = %.3f",
            kp_);

        RCLCPP_INFO(
            get_logger(),
            "Kd = %.3f",
            kd_);

        RCLCPP_INFO(
            get_logger(),
            "Body shift X = %.1f mm",
            body_shift_x_ * 1000.0);

        RCLCPP_INFO(
            get_logger(),
            "Body shift Y = %.1f mm",
            body_shift_y_ * 1000.0);

        RCLCPP_INFO(
            get_logger(),
            "FL lift = %.1f mm",
            fl_lift_height_ * 1000.0);

        RCLCPP_INFO(
            get_logger(),
            "==============================================");
    }


private:

    // =========================================================
    // JOINT STATE CALLBACK
    // =========================================================

    void jointStateCallback(
        const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (msg->name.empty())
        {
            return;
        }

        if (msg->position.size() != msg->name.size())
        {
            return;
        }


        for (std::size_t i = 0;
             i < msg->name.size();
             ++i)
        {
            const auto it =
                joint_index_.find(
                    msg->name[i]);

            if (it ==
                joint_index_.end())
            {
                continue;
            }


            const std::size_t index =
                it->second;


            q_actual_(
                static_cast<Eigen::Index>(index))
                =
                msg->position[i];


            if (i < msg->velocity.size())
            {
                qdot_actual_(
                    static_cast<Eigen::Index>(index))
                    =
                    msg->velocity[i];
            }
        }


        got_joint_state_ = true;
    }


    // =========================================================
    // QUINTIC
    // =========================================================

    static double quintic(
        double u)
    {
        u =
            std::clamp(
                u,
                0.0,
                1.0);


        return

            10.0 * std::pow(u, 3)
            -
            15.0 * std::pow(u, 4)
            +
             6.0 * std::pow(u, 5);
    }


    static double quinticDerivative(
        double u)
    {
        u =
            std::clamp(
                u,
                0.0,
                1.0);


        return

            30.0 * std::pow(u, 2)
            -
            60.0 * std::pow(u, 3)
            +
            30.0 * std::pow(u, 4);
    }


    // =========================================================
    // PHASE
    // =========================================================

    void setPhase(
        Phase next)
    {
        phase_ = next;

        phase_start_time_ =
            now();


        RCLCPP_INFO(
            get_logger(),
            "State -> %s",
            phaseName(next).c_str());
    }


    double phaseElapsed() const
    {
        return
            (now() -
             phase_start_time_).seconds();
    }


    // =========================================================
    // INITIALIZATION
    // =========================================================

    bool initialize()
    {
        if (!got_joint_state_)
        {
            return false;
        }


        // Verify that actual q is close to
        // the validated standing configuration.

        const double error =
            (q_actual_ -
             q_standing_).norm();


        RCLCPP_INFO(
            get_logger(),
            "Initial joint error from standing = %.3f rad",
            error);


        if (error > 0.20)
        {
            RCLCPP_WARN(
                get_logger(),
                "Robot is not close to standing pose. "
                "Experiment will not start.");

            return false;
        }


        initialized_ = true;


        RCLCPP_INFO(
            get_logger(),
            "Standing configuration accepted.");


        return true;
    }


    // =========================================================
    // GENERATE BODY-SHIFTED FOOT TARGET
    //
    // Desired body movement:
    //
    //     body_shift_x
    //     body_shift_y
    //
    // If the feet remain in contact with ground,
    // moving the desired foot pose relative to the torso
    // in the opposite direction causes the body to move
    // approximately toward body_shift.
    //
    // p_foot_des =
    //      p_foot_nominal
    //      - body_shift
    // =========================================================

    void generateTargets(
        double body_shift_scale,
        double body_shift_scale_dot,
        double fl_lift_scale,
        double fl_lift_scale_dot,
        std::array<Eigen::Vector3d, 4> &p_des,
        std::array<Eigen::Vector3d, 4> &pdot_des)
    {
        const Eigen::Vector3d body_shift(
            body_shift_x_ *
                body_shift_scale,

            body_shift_y_ *
                body_shift_scale,

            0.0);


        const Eigen::Vector3d body_shift_dot(
            body_shift_x_ *
                body_shift_scale_dot,

            body_shift_y_ *
                body_shift_scale_dot,

            0.0);


        for (int leg = 0;
             leg < 4;
             ++leg)
        {
            p_des[leg] =
                foot_nominal_[leg]
                -
                body_shift;


            pdot_des[leg] =
                -body_shift_dot;
        }


        // Only FL leaves the ground.

        p_des[0].z() +=
            fl_lift_height_ *
            fl_lift_scale;


        pdot_des[0].z() +=
            fl_lift_height_ *
            fl_lift_scale_dot;
    }


    // =========================================================
    // CARTESIAN -> JOINT
    // =========================================================

    bool computeJointTargets(
        const std::array<Eigen::Vector3d, 4> &p_des,
        const std::array<Eigen::Vector3d, 4> &pdot_des)
    {
        q_des_ =
            q_standing_;

        qdot_des_ =
            Eigen::VectorXd::Zero(12);


        for (int leg = 0;
             leg < 4;
             ++leg)
        {
            const Eigen::Vector3d q_seed =
                q_actual_.segment<3>(
                    3 * leg);


            const auto ik =
                kinematics_.inverse(
                    legs_[leg],
                    p_des[leg],
                    &q_seed);


            if (!ik.success)
            {
                RCLCPP_WARN_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    1000,
                    "IK failed for leg %d",
                    leg);

                return false;
            }


            q_des_.segment<3>(
                3 * leg)
                =
                ik.q;


            const Eigen::Matrix3d J =
                kinematics_.jacobian(
                    legs_[leg],
                    ik.q);


            if (std::abs(
                    J.determinant())
                > 1e-8)
            {
                qdot_des_.segment<3>(
                    3 * leg)
                    =
                    J.fullPivLu().solve(
                        pdot_des[leg]);
            }
        }


        return true;
    }


    // =========================================================
    // TORQUE SATURATION
    // =========================================================

    Eigen::Vector3d clampTorque(
        const Eigen::Vector3d &tau)
    {
        Eigen::Vector3d out =
            tau;


        out(0) =
            std::clamp(
                out(0),
                -24.0,
                24.0);


        out(1) =
            std::clamp(
                out(1),
                -24.0,
                24.0);


        out(2) =
            std::clamp(
                out(2),
                -36.0,
                36.0);


        return out;
    }


    // =========================================================
    // PUBLISH EFFORT
    // =========================================================

    void publishTorque()
    {
        Eigen::VectorXd tau =
            Eigen::VectorXd::Zero(12);


        const Eigen::VectorXd error =
            q_des_ -
            q_actual_;


        const Eigen::VectorXd
            error_dot =
                qdot_des_ -
                qdot_actual_;


        for (int leg = 0;
             leg < 4;
             ++leg)
        {
            const Eigen::Vector3d q_leg =
                q_actual_.segment<3>(
                    3 * leg);


            const Eigen::Vector3d qdot_leg =
                qdot_actual_.segment<3>(
                    3 * leg);


            const Eigen::Vector3d gravity =
                dynamics_.gravityTorque(
                    legs_[leg],
                    q_leg);


            const Eigen::Vector3d
                e =
                    error.segment<3>(
                        3 * leg);


            const Eigen::Vector3d
                edot =
                    error_dot.segment<3>(
                        3 * leg);


            Eigen::Vector3d
                tau_leg =

                gravity
                +
                kp_ * e
                +
                kd_ * edot;


            tau_leg =
                clampTorque(
                    tau_leg);


            tau.segment<3>(
                3 * leg)
                =
                tau_leg;
        }


        std_msgs::msg::Float64MultiArray msg;

        msg.data.resize(12);


        for (int i = 0;
             i < 12;
             ++i)
        {
            msg.data[
                static_cast<std::size_t>(i)]
                =
                tau(i);
        }


        effort_pub_->publish(
            msg);
    }


    // =========================================================
    // CONTROL LOOP
    // =========================================================

    void controlLoop()
    {
        if (!got_joint_state_)
        {
            return;
        }


        const bool
            start_requested =
                get_parameter(
                    "start_experiment")
                .as_bool();


        // -----------------------------------------------------
        // Start
        // -----------------------------------------------------

        if (!experiment_started_ &&
            start_requested)
        {
            if (!initialized_)
            {
                if (!initialize())
                {
                    return;
                }
            }


            experiment_started_ =
                true;


            setPhase(
                Phase::PREPARE);
        }


        // -----------------------------------------------------
        // WAITING
        // -----------------------------------------------------

        if (!experiment_started_)
        {
            q_des_ =
                q_standing_;

            qdot_des_.setZero();

            publishTorque();

            return;
        }


        // -----------------------------------------------------
        // PREPARE
        // -----------------------------------------------------

        if (phase_ ==
            Phase::PREPARE)
        {
            q_des_ =
                q_standing_;

            qdot_des_.setZero();

            publishTorque();


            if (phaseElapsed()
                >=
                prepare_duration_)
            {
                setPhase(
                    Phase::BODY_SHIFT);
            }

            return;
        }


        // -----------------------------------------------------
        // BODY SHIFT
        // -----------------------------------------------------

        if (phase_ ==
            Phase::BODY_SHIFT)
        {
            const double u =
                phaseElapsed()
                /
                body_shift_duration_;


            const double s =
                quintic(u);


            const double ds =
                quinticDerivative(u)
                /
                body_shift_duration_;


            std::array<
                Eigen::Vector3d,
                4>
                p_des;


            std::array<
                Eigen::Vector3d,
                4>
                pdot_des;


            generateTargets(
                s,
                ds,
                0.0,
                0.0,
                p_des,
                pdot_des);


            if (computeJointTargets(
                    p_des,
                    pdot_des))
            {
                publishTorque();
            }


            if (u >= 1.0)
            {
                setPhase(
                    Phase::SHIFT_HOLD);
            }


            printDebug(
                "BODY_SHIFT");


            return;
        }


        // -----------------------------------------------------
        // SHIFT HOLD
        // -----------------------------------------------------

        if (phase_ ==
            Phase::SHIFT_HOLD)
        {
            std::array<
                Eigen::Vector3d,
                4>
                p_des;


            std::array<
                Eigen::Vector3d,
                4>
                pdot_des;


            generateTargets(
                1.0,
                0.0,
                0.0,
                0.0,
                p_des,
                pdot_des);


            if (computeJointTargets(
                    p_des,
                    pdot_des))
            {
                publishTorque();
            }


            if (phaseElapsed()
                >=
                shift_hold_duration_)
            {
                setPhase(
                    Phase::FL_LIFT);
            }


            return;
        }


        // -----------------------------------------------------
        // FL LIFT
        // -----------------------------------------------------

        if (phase_ ==
            Phase::FL_LIFT)
        {
            const double u =
                phaseElapsed()
                /
                fl_lift_duration_;


            const double s =
                quintic(u);


            const double ds =
                quinticDerivative(u)
                /
                fl_lift_duration_;


            std::array<
                Eigen::Vector3d,
                4>
                p_des;


            std::array<
                Eigen::Vector3d,
                4>
                pdot_des;


            generateTargets(
                1.0,
                0.0,
                s,
                ds,
                p_des,
                pdot_des);


            if (computeJointTargets(
                    p_des,
                    pdot_des))
            {
                publishTorque();
            }


            if (u >= 1.0)
            {
                setPhase(
                    Phase::FL_HOLD);
            }


            printDebug(
                "FL_LIFT");


            return;
        }


        // -----------------------------------------------------
        // FL HOLD
        // -----------------------------------------------------

        if (phase_ ==
            Phase::FL_HOLD)
        {
            std::array<
                Eigen::Vector3d,
                4>
                p_des;


            std::array<
                Eigen::Vector3d,
                4>
                pdot_des;


            generateTargets(
                1.0,
                0.0,
                1.0,
                0.0,
                p_des,
                pdot_des);


            if (computeJointTargets(
                    p_des,
                    pdot_des))
            {
                publishTorque();
            }


            if (phaseElapsed()
                >=
                fl_hold_duration_)
            {
                setPhase(
                    Phase::FL_LOWER);
            }


            return;
        }


        // -----------------------------------------------------
        // FL LOWER
        // -----------------------------------------------------

        if (phase_ ==
            Phase::FL_LOWER)
        {
            const double u =
                phaseElapsed()
                /
                fl_lift_duration_;


            const double s =
                quintic(u);


            const double ds =
                quinticDerivative(u)
                /
                fl_lift_duration_;


            const double
                lift_scale =
                1.0 - s;


            const double
                lift_scale_dot =
                -ds;


            std::array<
                Eigen::Vector3d,
                4>
                p_des;


            std::array<
                Eigen::Vector3d,
                4>
                pdot_des;


            generateTargets(
                1.0,
                0.0,
                lift_scale,
                lift_scale_dot,
                p_des,
                pdot_des);


            if (computeJointTargets(
                    p_des,
                    pdot_des))
            {
                publishTorque();
            }


            if (u >= 1.0)
            {
                setPhase(
                    Phase::BODY_RETURN);
            }


            return;
        }


        // -----------------------------------------------------
        // RETURN BODY
        // -----------------------------------------------------

        if (phase_ ==
            Phase::BODY_RETURN)
        {
            const double u =
                phaseElapsed()
                /
                body_shift_duration_;


            const double s =
                quintic(u);


            const double ds =
                quinticDerivative(u)
                /
                body_shift_duration_;


            const double
                scale =
                1.0 - s;


            const double
                scale_dot =
                -ds;


            std::array<
                Eigen::Vector3d,
                4>
                p_des;


            std::array<
                Eigen::Vector3d,
                4>
                pdot_des;


            generateTargets(
                scale,
                scale_dot,
                0.0,
                0.0,
                p_des,
                pdot_des);


            if (computeJointTargets(
                    p_des,
                    pdot_des))
            {
                publishTorque();
            }


            if (u >= 1.0)
            {
                setPhase(
                    Phase::DONE);
            }


            return;
        }


        // -----------------------------------------------------
        // DONE
        // -----------------------------------------------------

        if (phase_ ==
            Phase::DONE)
        {
            q_des_ =
                q_standing_;

            qdot_des_.setZero();

            publishTorque();
        }
    }


    // =========================================================
    // DEBUG
    // =========================================================

    void printDebug(
        const std::string &state)
    {
        if (++debug_counter_ < 250)
        {
            return;
        }

        debug_counter_ = 0;


        RCLCPP_INFO(
            get_logger(),

            "[%s] "
            "FL q=[%.3f %.3f %.3f] "
            "qerr=%.4f",

            state.c_str(),

            q_actual_(0),
            q_actual_(1),
            q_actual_(2),

            (
                q_des_.segment<3>(0)
                -
                q_actual_.segment<3>(0)
            ).norm());
    }


    // =========================================================
    // PHASE NAME
    // =========================================================

    static std::string phaseName(
        Phase phase)
    {
        switch (phase)
        {
            case Phase::WAITING:
                return "WAITING";

            case Phase::PREPARE:
                return "PREPARE";

            case Phase::BODY_SHIFT:
                return "BODY_SHIFT";

            case Phase::SHIFT_HOLD:
                return "SHIFT_HOLD";

            case Phase::FL_LIFT:
                return "FL_LIFT";

            case Phase::FL_HOLD:
                return "FL_HOLD";

            case Phase::FL_LOWER:
                return "FL_LOWER";

            case Phase::BODY_RETURN:
                return "BODY_RETURN";

            case Phase::DONE:
                return "DONE";

            default:
                return "UNKNOWN";
        }
    }


    // =========================================================
    // MEMBERS
    // =========================================================

    lite3_dynamics::Lite3SingleLegDynamics
        dynamics_;

    lite3_kinematics::Lite3Kinematics
        kinematics_;


    std::array<
        lite3_kinematics::Leg,
        4>
        legs_;


    std::vector<
        std::string>
        joint_names_;


    std::unordered_map<
        std::string,
        std::size_t>
        joint_index_;


    Eigen::VectorXd
        q_actual_;

    Eigen::VectorXd
        qdot_actual_;

    Eigen::VectorXd
        q_standing_;

    Eigen::VectorXd
        q_des_;

    Eigen::VectorXd
        qdot_des_;


    std::array<
        Eigen::Vector3d,
        4>
        foot_nominal_;


    rclcpp::Publisher<
        std_msgs::msg::Float64MultiArray>::SharedPtr
        effort_pub_;


    rclcpp::Subscription<
        sensor_msgs::msg::JointState>::SharedPtr
        joint_sub_;


    rclcpp::TimerBase::SharedPtr
        timer_;


    double kp_;
    double kd_;
    double publish_rate_;


    double body_shift_x_;
    double body_shift_y_;

    double body_shift_duration_;
    double shift_hold_duration_;

    double fl_lift_height_;
    double fl_lift_duration_;
    double fl_hold_duration_;

    double prepare_duration_;


    bool experiment_started_;
    bool got_joint_state_;
    bool initialized_;


    Phase phase_;

    rclcpp::Time
        phase_start_time_;


    int debug_counter_ = 0;
};


int main(
    int argc,
    char **argv)
{
    rclcpp::init(
        argc,
        argv);


    auto node =
        std::make_shared<
            ThreeLegStabilityTest>();


    rclcpp::spin(node);


    rclcpp::shutdown();


    return 0;
}