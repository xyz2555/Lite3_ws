#include <Eigen/Dense>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <array>
#include <string>

#include "lite3_kinematics/kinematics.hpp"
#include "lite3_dynamics/dynamics.hpp"

using Eigen::Vector3d;
using Eigen::Matrix3d;

namespace
{
struct LegState
{
    lite3_kinematics::Leg leg;
    const char* name;
    Eigen::Vector3d q_standing;
    Eigen::Vector3d foot_standing;
};

std::string statusToString(lite3_kinematics::IKStatus status)
{
    switch (status)
    {
        case lite3_kinematics::IKStatus::SUCCESS:
            return "SUCCESS";
        case lite3_kinematics::IKStatus::GEOMETRICALLY_UNREACHABLE:
            return "GEOMETRICALLY_UNREACHABLE";
        case lite3_kinematics::IKStatus::JOINT_LIMIT_VIOLATION:
            return "JOINT_LIMIT_VIOLATION";
        case lite3_kinematics::IKStatus::INVALID_TARGET:
            return "INVALID_TARGET";
        default:
            return "UNKNOWN";
    }
}

// Minimum-norm vertical contact force distribution.
//
// Unknown:
//      F = [F_FL, F_FR, F_HL, F_HR]^T
//
// Constraints:
//      sum(F_i) = total_weight
//      sum(x_i F_i) = 0
//      sum(y_i F_i) = 0
//
// This is a simple static equilibrium model.
// It assumes:
//      Fx = Fy = 0
Eigen::Vector4d solveVerticalLoads(
    const std::array<Vector3d, 4>& feet,
    double total_weight)
{
    Eigen::Matrix<double, 3, 4> A;
    Eigen::Vector3d b;

    A <<
        1.0, 1.0, 1.0, 1.0,
        feet[0].x(), feet[1].x(), feet[2].x(), feet[3].x(),
        feet[0].y(), feet[1].y(), feet[2].y(), feet[3].y();

    b <<
        total_weight,
        0.0,
        0.0;

    // Minimum-norm solution:
    //
    // F = A^T (A A^T)^-1 b
    //
    // A      : 3x4
    // A A^T  : 3x3
    // result : 4x1

    return A.transpose() *
           (A * A.transpose()).ldlt().solve(b);
}

} // namespace

int main()
{
    using namespace lite3_kinematics;

    std::cout << std::fixed << std::setprecision(8);

    Lite3Kinematics kin;
    lite3_dynamics::Lite3SingleLegDynamics dyn;

    constexpr double g = 9.81;

    // From the MJCF model already used in our dynamics validation.
    constexpr double total_mass = 11.9376;
    const double total_weight = total_mass * g;

    const std::array<double, 3> stair_heights =
    {
        0.185,
        0.200,
        0.215
    };

    const std::array<LegState, 4> legs =
    {{
        {
    Leg::FL,
    "FL",
    Vector3d(-0.02073, -0.67214, 1.32366),
    Vector3d( 0.177383,  0.166036, -0.321490)
},
{
    Leg::FR,
    "FR",
    Vector3d( 0.01497, -0.67765, 1.33907),
    Vector3d( 0.178200, -0.164200, -0.320100)
},
{
    Leg::HL,
    "HL",
    Vector3d(-0.02465, -0.64953, 1.32289),
    Vector3d(-0.164400,  0.167300, -0.321000)
},
{
    Leg::HR,
    "HR",
    Vector3d( 0.01714, -0.65116, 1.33529),
    Vector3d(-0.162900, -0.164900, -0.320200)
}
    }};

    std::cout
        << "====================================================\n"
        << "Lite3 Stair Contact Equilibrium - IK Posture\n"
        << "====================================================\n"
        << "Total mass   : " << total_mass << " kg\n"
        << "Total weight : " << total_weight << " N\n";

    for (double h : stair_heights)
    {
        std::cout
            << "\n====================================================\n"
            << "Stair height = " << h * 100.0 << " cm\n"
            << "====================================================\n";

        std::array<Vector3d, 4> q_stair;
        std::array<Vector3d, 4> feet_stair;

        bool ik_all_ok = true;

        // --------------------------------------------------
        // 1. Generate actual stair posture through IK
        // --------------------------------------------------
        for (std::size_t i = 0; i < legs.size(); ++i)
        {
            const auto& L = legs[i];

            Vector3d target = L.foot_standing;

            // Front legs stand on the upper stair surface.
            if (L.leg == Leg::FL || L.leg == Leg::FR)
            {
                target.z() += h;
            }

            auto result = kin.inverse(
                L.leg,
                target,
                &L.q_standing   // IMPORTANT:
                                    // this seed is only used to obtain
                                    // the physically relevant IK branch
            );

            if (!result.success)
            {
                ik_all_ok = false;

                std::cout
                    << "\n" << L.name
                    << " IK FAILED: "
                    << statusToString(result.status)
                    << "\n";

                continue;
            }

            q_stair[i] = result.q;
            feet_stair[i] = kin.forward(L.leg, q_stair[i]);

            std::cout
                << "\n" << L.name << "\n"
                << "  q_stair = "
                << q_stair[i].transpose() << " rad\n"
                << "  FK      = "
                << feet_stair[i].transpose() << " m\n"
                << "  error   = "
                << (feet_stair[i] - target).norm()
                << " m\n";
        }

        if (!ik_all_ok)
        {
            std::cout
                << "\nOverall IK: FAIL\n";
            continue;
        }

        // --------------------------------------------------
        // 2. Static contact force distribution
        // --------------------------------------------------
        const Eigen::Vector4d Fz =
            solveVerticalLoads(feet_stair, total_weight);

        std::cout
            << "\nContact vertical loads:\n";

        for (std::size_t i = 0; i < legs.size(); ++i)
        {
            const double ratio = Fz(i) / total_weight;

            std::cout
                << "  " << legs[i].name
                << " : Fz = " << Fz(i)
                << " N  ("
                << ratio * 100.0
                << " % BW)\n";
        }

        // --------------------------------------------------
        // 3. Check whole-body static equilibrium
        // --------------------------------------------------
        const double sumFz =
            Fz.sum();

        const double Mx =
            feet_stair[0].y() * Fz(0) +
            feet_stair[1].y() * Fz(1) +
            feet_stair[2].y() * Fz(2) +
            feet_stair[3].y() * Fz(3);

        const double My =
            -(feet_stair[0].x() * Fz(0) +
              feet_stair[1].x() * Fz(1) +
              feet_stair[2].x() * Fz(2) +
              feet_stair[3].x() * Fz(3));

        std::cout
            << "\nWhole-body equilibrium:\n"
            << "  Sum Fz = " << sumFz
            << " N\n"
            << "  Mx     = " << Mx
            << " N m\n"
            << "  My     = " << My
            << " N m\n";

        // --------------------------------------------------
        // 4. Joint static torque for each leg
        //
        // tau = J^T F + G
        //
        // We currently assume:
        //      F = [0, 0, Fz]
        // --------------------------------------------------
        std::cout
            << "\nJoint static torque:\n";

        double worst_norm = -1.0;
        const char* worst_leg = "";
        int worst_joint = -1;

        for (std::size_t i = 0; i < legs.size(); ++i)
        {
            const Vector3d q = q_stair[i];

            // Gravity torque
            const Vector3d G =
                dyn.gravityTorque(
                    legs[i].leg,
                    q);

            // Foot Jacobian
            const Matrix3d J =
                dyn.footJacobian(
                    legs[i].leg,
                    q);

            Vector3d F(0.0, 0.0, Fz(i));

            const Vector3d tau_contact =
                J.transpose() * F;

            const Vector3d tau_static =
                tau_contact + G;

            std::cout
                << "\n  " << legs[i].name << "\n"
                << "    G          = "
                << G.transpose() << " Nm\n"
                << "    J^T F      = "
                << tau_contact.transpose() << " Nm\n"
                << "    tau_static = "
                << tau_static.transpose() << " Nm\n"
                << "    |tau|      = "
                << tau_static.norm() << " Nm\n";

            for (int j = 0; j < 3; ++j)
            {
                if (std::abs(tau_static(j)) > worst_norm)
                {
                    worst_norm = std::abs(tau_static(j));
                    worst_leg = legs[i].name;
                    worst_joint = j;
                }
            }
        }

        const char* joint_names[3] =
        {
            "HipX",
            "HipY",
            "Knee"
        };

        std::cout
            << "\nWorst individual joint torque:\n"
            << "  Leg   = " << worst_leg << "\n"
            << "  Joint = " << joint_names[worst_joint] << "\n"
            << "  |tau| = " << worst_norm << " Nm\n";

        // --------------------------------------------------
        // 5. Static feasibility indication
        // --------------------------------------------------
        bool positive_loads = true;

        for (int i = 0; i < 4; ++i)
        {
            if (Fz(i) <= 0.0)
            {
                positive_loads = false;
            }
        }

        std::cout
            << "\nStatic contact feasibility:\n"
            << "  All Fz > 0 = "
            << (positive_loads ? "YES" : "NO")
            << "\n";

        std::cout
            << "----------------------------------------------------\n";
    }

    return 0;
}