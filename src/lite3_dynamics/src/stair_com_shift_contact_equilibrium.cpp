#include <Eigen/Dense>

#include <array>
#include <iomanip>
#include <iostream>
#include <cmath>

#include "lite3_kinematics/kinematics.hpp"
#include "lite3_dynamics/dynamics.hpp"

using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::Matrix3d;

using lite3_kinematics::Leg;
using lite3_kinematics::Lite3Kinematics;

namespace
{

struct LegInfo
{
    Leg leg;
    const char* name;

    Vector3d q_standing;
    Vector3d foot_standing;
};

const std::array<LegInfo, 4> legs =
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

const char* jointName(int j)
{
    if (j == 0) return "HipX";
    if (j == 1) return "HipY";
    return "Knee";
}

// ------------------------------------------------------------
// 4-contact minimum-norm equilibrium
// ------------------------------------------------------------

Vector4d solveFourContactLoads(
    const std::array<Vector3d, 4>& feet,
    double weight)
{
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

    Vector3d b;

    b <<
        weight,
        0.0,
        0.0;

    return A.transpose()
        * (A * A.transpose())
            .ldlt()
            .solve(b);
}

// ------------------------------------------------------------
// 3-contact equilibrium for an arbitrary CoM projection
//
// Contacts are FR, HL, HR.
//
// Equilibrium:
//
// sum(Fz) = W
// sum((x_i - x_com) F_i) = 0
// sum((y_i - y_com) F_i) = 0
// ------------------------------------------------------------

Vector3d solveThreeContactLoads(
    const std::array<Vector3d, 3>& feet,
    const Vector3d& com_xy,
    double weight)
{
    Eigen::Matrix3d A;

    A <<
        1.0, 1.0, 1.0,

        feet[0].x() - com_xy.x(),
        feet[1].x() - com_xy.x(),
        feet[2].x() - com_xy.x(),

        feet[0].y() - com_xy.y(),
        feet[1].y() - com_xy.y(),
        feet[2].y() - com_xy.y();

    Vector3d b;

    b <<
        weight,
        0.0,
        0.0;

    return A.fullPivLu().solve(b);
}

// ------------------------------------------------------------
// Centroid of support triangle
// ------------------------------------------------------------

Vector3d triangleCentroid(
    const std::array<Vector3d, 3>& feet)
{
    Vector3d c = Vector3d::Zero();

    for (const auto& p : feet)
    {
        c += p;
    }

    c /= 3.0;

    return c;
}

// ------------------------------------------------------------
// Calculate q on stair using IK
// ------------------------------------------------------------

bool stairIK(
    double h,
    Lite3Kinematics& kin,
    std::array<Vector3d, 4>& q,
    std::array<Vector3d, 4>& feet)
{
    for (std::size_t i = 0; i < legs.size(); ++i)
    {
        Vector3d target =
            legs[i].foot_standing;

        // Front feet are on upper stair
        if (legs[i].leg == Leg::FL ||
            legs[i].leg == Leg::FR)
        {
            target.z() += h;
        }

        auto result =
            kin.inverse(
                legs[i].leg,
                target,
                &legs[i].q_standing);

        if (!result.success)
        {
            std::cerr
                << "IK failed for "
                << legs[i].name
                << "\n";

            return false;
        }

        q[i] = result.q;

        feet[i] =
            kin.forward(
                legs[i].leg,
                q[i]);
    }

    return true;
}

} // namespace


int main()
{
    std::cout << std::fixed
              << std::setprecision(8);

    std::cout
        << "====================================================\n"
        << "Lite3 Stair CoM-Shift Contact Equilibrium\n"
        << "====================================================\n";

    constexpr double mass = 11.9376;
    constexpr double g = 9.81;

    const double weight = mass * g;

    Lite3Kinematics kin;
    lite3_dynamics::Lite3SingleLegDynamics dyn;

    const std::array<double, 3> heights =
    {
        0.185,
        0.200,
        0.215
    };

    for (double h : heights)
    {
        std::cout
            << "\n====================================================\n"
            << "Stair height = "
            << h * 100.0
            << " cm\n"
            << "====================================================\n";

        std::array<Vector3d, 4> q;
        std::array<Vector3d, 4> feet;

        if (!stairIK(h, kin, q, feet))
        {
            return 1;
        }

        // ------------------------------------------------
        // Four-leg baseline
        // ------------------------------------------------

        Vector4d F4 =
            solveFourContactLoads(
                feet,
                weight);

        std::cout
            << "\n4-contact reference loads:\n";

        for (int i = 0; i < 4; ++i)
        {
            std::cout
                << "  "
                << legs[i].name
                << " = "
                << F4(i)
                << " N\n";
        }

        // ------------------------------------------------
        // Support triangle when FL is swinging
        //
        // FR, HL, HR
        // ------------------------------------------------

        std::array<Vector3d, 3> supportFeet =
        {
            feet[1],
            feet[2],
            feet[3]
        };

        // ------------------------------------------------
        // Ideal CoM location = triangle centroid
        // ------------------------------------------------

        Vector3d com =
            triangleCentroid(
                supportFeet);

        std::cout
            << "\nIdealized CoM projection:\n"
            << "  Xcom = "
            << com.x()
            << " m\n"
            << "  Ycom = "
            << com.y()
            << " m\n";

        // ------------------------------------------------
        // 3-leg load distribution
        // ------------------------------------------------

        Vector3d F3 =
            solveThreeContactLoads(
                supportFeet,
                com,
                weight);

        std::cout
            << "\n3-contact balanced loads:\n"
            << "  FR = "
            << F3(0)
            << " N\n"
            << "  HL = "
            << F3(1)
            << " N\n"
            << "  HR = "
            << F3(2)
            << " N\n";

        // ------------------------------------------------
        // Map loads to 4-leg vector
        // ------------------------------------------------

        Vector4d Fbalanced =
            Vector4d::Zero();

        Fbalanced[0] = 0.0;
        Fbalanced[1] = F3[0];
        Fbalanced[2] = F3[1];
        Fbalanced[3] = F3[2];

        // ------------------------------------------------
        // Static torque
        // ------------------------------------------------

        std::cout
            << "\nJoint torques with idealized CoM shift:\n";

        double globalMax = 0.0;

        int worstLeg = -1;
        int worstJoint = -1;

        for (int i = 0; i < 4; ++i)
        {
            const Vector3d& qi =
                q[i];

            const Vector3d G =
                dyn.gravityTorque(
                    legs[i].leg,
                    qi);

            const Matrix3d J =
                dyn.footJacobian(
                    legs[i].leg,
                    qi);

            Vector3d F(
                0.0,
                0.0,
                Fbalanced[i]);

            Vector3d tau =
                G +
                J.transpose() * F;

            std::cout
                << "\n  "
                << legs[i].name
                << "\n"
                << "    Fz = "
                << Fbalanced[i]
                << " N\n"
                << "    G  = "
                << G.transpose()
                << " Nm\n"
                << "    JTF = "
                << (J.transpose() * F).transpose()
                << " Nm\n"
                << "    tau = "
                << tau.transpose()
                << " Nm\n"
                << "    |tau| = "
                << tau.norm()
                << " Nm\n";

            for (int j = 0; j < 3; ++j)
            {
                const double a =
                    std::abs(tau[j]);

                if (a > globalMax)
                {
                    globalMax = a;
                    worstLeg = i;
                    worstJoint = j;
                }
            }
        }

        std::cout
            << "\n====================================================\n"
            << "Result\n"
            << "====================================================\n";

        std::cout
            << "Global max |tau| = "
            << globalMax
            << " Nm\n";

        std::cout
            << "Worst = "
            << legs[worstLeg].name
            << " "
            << jointName(worstJoint)
            << "\n";

        // ------------------------------------------------
        // Compare to original fixed-CoM 3-leg assumption
        // ------------------------------------------------

        std::cout
            << "\nInterpretation:\n"
            << "  CoM model = support-triangle centroid\n"
            << "  Contact model = ideal balanced vertical load\n"
            << "  This is a best-case / lower-bound estimate.\n";
    }

    return 0;
}