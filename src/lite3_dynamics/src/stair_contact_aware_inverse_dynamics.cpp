#include <Eigen/Dense>

#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include "lite3_kinematics/kinematics.hpp"
#include "lite3_dynamics/dynamics.hpp"

using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::Matrix3d;

using lite3_kinematics::Leg;
using lite3_kinematics::Lite3Kinematics;

namespace
{

// ------------------------------------------------------------
// Standing configuration
// ------------------------------------------------------------

struct LegInfo
{
    Leg leg;
    const char* name;
    Vector3d q_standing;
    Vector3d foot_standing;
};

const std::array<LegInfo, 4> LEG_INFO =
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

// ------------------------------------------------------------
// CSV sample
// ------------------------------------------------------------

struct Sample
{
    double t;

    Vector3d p;
    Vector3d q;
    Vector3d qdot;
    Vector3d qddot;
};

// ------------------------------------------------------------
// Parse CSV
//
// We only use the first 19 numerical fields.
// Extra fields at the end of the CSV are ignored.
// ------------------------------------------------------------

bool parseCSVLine(
    const std::string& line,
    Sample& s)
{
    if (line.empty())
        return false;

    std::stringstream ss(line);
    std::string token;
    std::vector<double> values;

    while (std::getline(ss, token, ','))
    {
        if (token.empty())
            continue;

        try
        {
            values.push_back(std::stod(token));
        }
        catch (...)
        {
            return false;
        }
    }

    if (values.size() < 19)
        return false;

    s.t = values[0];

    // position
    s.p <<
        values[1],
        values[2],
        values[3];

    // joint position
    s.q <<
        values[4],
        values[5],
        values[6];

    // joint velocity
    s.qdot <<
        values[7],
        values[8],
        values[9];

    // joint acceleration
    s.qddot <<
        values[10],
        values[11],
        values[12];

    return true;
}

// ------------------------------------------------------------
// Load trajectory
// ------------------------------------------------------------

bool loadTrajectory(
    const std::string& filename,
    std::vector<Sample>& samples)
{
    std::ifstream file(filename);

    if (!file.is_open())
    {
        std::cerr
            << "ERROR: Cannot open "
            << filename << "\n";

        return false;
    }

    std::string line;

    // Skip header
    if (!std::getline(file, line))
        return false;

    while (std::getline(file, line))
    {
        Sample s;

        if (parseCSVLine(line, s))
            samples.push_back(s);
    }

    return !samples.empty();
}

// ------------------------------------------------------------
// Minimum norm vertical loads for 4 contacts
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

    return A.transpose() *
           (A * A.transpose()).ldlt().solve(b);
}

// ------------------------------------------------------------
// Exact 3-contact vertical equilibrium
// ------------------------------------------------------------

Vector3d solveThreeContactLoads(
    const std::array<Vector3d, 3>& feet,
    double weight)
{
    Eigen::Matrix3d A;

    A <<
        1.0, 1.0, 1.0,

        feet[0].x(),
        feet[1].x(),
        feet[2].x(),

        feet[0].y(),
        feet[1].y(),
        feet[2].y();

    Vector3d b;

    b <<
        weight,
        0.0,
        0.0;

    return A.fullPivLu().solve(b);
}

// ------------------------------------------------------------
// Contact state
//
// Segment structure:
//
// P0 -> P1 : FL still contact
// P1 -> P4 : FL swing
// P4 -> P5 : FL still swing
// final P5 : FL contact
//
// Segment ratios from trajectory analyzer:
//
// 0.20 : 0.30 : 0.20 : 0.30 : 0.20
//
// Total = 1.20
//
// Therefore normalized boundaries are:
// P1 = 1/6
// P2 = 5/12
// P3 = 7/12
// P4 = 5/6
// P5 = 1
// ------------------------------------------------------------

bool flInContact(double t, double T)
{
    const double p1 = T * (1.0 / 6.0);
    const double p5 = T;

    constexpr double eps = 1e-9;

    return
        (t <= p1 + eps) ||
        (t >= p5 - eps);
}

// ------------------------------------------------------------
// Stair configuration
// ------------------------------------------------------------

bool getStairConfiguration(
    double h,
    Lite3Kinematics& kin,
    std::array<Vector3d, 4>& q,
    std::array<Vector3d, 4>& feet)
{
    for (std::size_t i = 0; i < LEG_INFO.size(); ++i)
    {
        const auto& L = LEG_INFO[i];

        Vector3d target = L.foot_standing;

        if (L.leg == Leg::FL ||
            L.leg == Leg::FR)
        {
            target.z() += h;
        }

        auto result = kin.inverse(
            L.leg,
            target,
            &L.q_standing);

        if (!result.success)
        {
            std::cerr
                << "IK failed for "
                << L.name
                << " at h = "
                << h
                << "\n";

            return false;
        }

        q[i] = result.q;
        feet[i] = kin.forward(
            L.leg,
            q[i]);
    }

    return true;
}

// ------------------------------------------------------------
// Convert leg enum to index
// ------------------------------------------------------------

int legIndex(Leg leg)
{
    switch (leg)
    {
        case Leg::FL: return 0;
        case Leg::FR: return 1;
        case Leg::HL: return 2;
        case Leg::HR: return 3;
        default: return -1;
    }
}

} // namespace

// ============================================================
// MAIN
// ============================================================

int main()
{
    std::cout
        << "====================================================\n"
        << "Lite3 Stair Contact-Aware Inverse Dynamics\n"
        << "====================================================\n";

    constexpr double total_mass = 11.9376;
    constexpr double g = 9.81;

    const double weight =
        total_mass * g;

    Lite3Kinematics kin;
    lite3_dynamics::Lite3SingleLegDynamics dyn;

    const std::array<double, 3> heights =
    {
        0.185,
        0.200,
        0.215
    };

    const std::array<double, 6> durations =
    {
        0.6,
        0.8,
        1.0,
        1.2,
        1.5,
        2.0
    };

    // --------------------------------------------------------
    // Summary output
    // --------------------------------------------------------

    std::ofstream summary(
        "lite3_stair_contact_aware_summary.csv");

    summary
        << "height_m,"
        << "duration_s,"
        << "max_FL_HipX,"
        << "max_FL_HipY,"
        << "max_FL_Knee,"
        << "max_FR_HipX,"
        << "max_FR_HipY,"
        << "max_FR_Knee,"
        << "max_HL_HipX,"
        << "max_HL_HipY,"
        << "max_HL_Knee,"
        << "max_HR_HipX,"
        << "max_HR_HipY,"
        << "max_HR_Knee,"
        << "max_abs_tau,"
        << "worst_leg,"
        << "worst_joint,"
        << "min_Fz,"
        << "min_sigma_FL"
        << "\n";

    for (double h : heights)
    {
        // ----------------------------------------------------
        // Generate stair posture
        // ----------------------------------------------------

        std::array<Vector3d, 4> q_stair;
        std::array<Vector3d, 4> feet_stair;

        if (!getStairConfiguration(
                h,
                kin,
                q_stair,
                feet_stair))
        {
            return 1;
        }

        for (double T : durations)
        {
            // ------------------------------------------------
            // Find trajectory file
            // ------------------------------------------------

            int h_mm =
                static_cast<int>(
                    std::round(h * 1000.0));

            std::ostringstream dur;
            dur << std::fixed
                << std::setprecision(1)
                << T;

            std::string durationTag =
                dur.str();

            // 0.6 -> 0p6
            for (char& c : durationTag)
            {
                if (c == '.')
                    c = 'p';
            }

            std::ostringstream filename;

            filename
                << "lite3_FL_stair_"
                << h_mm
                << "mm_T"
                << durationTag
                << "s.csv";

            std::vector<Sample> samples;

            if (!loadTrajectory(
                    filename.str(),
                    samples))
            {
                std::cerr
                    << "Skipping "
                    << filename.str()
                    << "\n";

                continue;
            }

            // ------------------------------------------------
            // Max torque tracker
            // ------------------------------------------------

            double max_tau[4][3] = {};

            double global_max =
                0.0;

            const char* worst_leg = "";
            const char* worst_joint = "";

            double min_Fz =
                std::numeric_limits<double>::max();

            double min_sigma_FL =
                std::numeric_limits<double>::max();

            // ------------------------------------------------
            // Time samples
            // ------------------------------------------------

            for (const auto& s : samples)
            {
                const bool fl_contact =
                    flInContact(s.t, T);

                // --------------------------------------------
                // Contact force calculation
                // --------------------------------------------

                Vector4d Fz4 =
                    Vector4d::Zero();

                bool all_positive =
                    true;

                if (fl_contact)
                {
                    // Four contacts
                    Fz4 =
                        solveFourContactLoads(
                            feet_stair,
                            weight);
                }
                else
                {
                    // FL is swinging.
                    //
                    // Support:
                    // FR, HL, HR
                    //
                    std::array<Vector3d, 3> support_feet =
                    {
                        feet_stair[1],
                        feet_stair[2],
                        feet_stair[3]
                    };

                    Vector3d Fz3 =
                        solveThreeContactLoads(
                            support_feet,
                            weight);

                    Fz4[0] = 0.0;
                    Fz4[1] = Fz3[0];
                    Fz4[2] = Fz3[1];
                    Fz4[3] = Fz3[2];
                }

                for (int i = 0; i < 4; ++i)
                {
                    min_Fz =
                        std::min(
                            min_Fz,
                            Fz4[i]);

                    if (Fz4[i] <= 0.0)
                    {
                        if (!(i == 0 && !fl_contact))
                        {
                            all_positive = false;
                        }
                    }
                }

                // --------------------------------------------
                // FL dynamic torque
                // --------------------------------------------

                Vector3d tau_fl =
                    dyn.inverseDynamics(
                        Leg::FL,
                        s.q,
                        s.qdot,
                        s.qddot);

                // Contact torque for FL
                if (fl_contact)
                {
                    Eigen::Matrix3d J =
    dyn.footJacobian(
        Leg::FL,
        s.q);

                    Vector3d F(
                        0.0,
                        0.0,
                        Fz4[0]);

                    tau_fl +=
                        J.transpose() * F;
                }

                // --------------------------------------------
                // FL diagnostics
                // --------------------------------------------

                Eigen::Matrix3d Jfl =
    dyn.footJacobian(
        Leg::FL,
        s.q);

                
                Eigen::JacobiSVD<Matrix3d> svd(
                    Jfl);

                const auto singular_values =
                    svd.singularValues();

                const double sigma_min =
                    singular_values.minCoeff();

                min_sigma_FL =
                    std::min(
                        min_sigma_FL,
                        sigma_min);

                // --------------------------------------------
                // Store FL maximum
                // --------------------------------------------

                for (int j = 0; j < 3; ++j)
                {
                    const double tau =
                        std::abs(tau_fl(j));

                    if (tau > max_tau[0][j])
                        max_tau[0][j] = tau;

                    if (tau > global_max)
                    {
                        global_max = tau;
                        worst_leg = "FL";

                        if (j == 0)
                            worst_joint = "HipX";
                        else if (j == 1)
                            worst_joint = "HipY";
                        else
                            worst_joint = "Knee";
                    }
                }

                // --------------------------------------------
                // Static torque for support legs
                // --------------------------------------------

                const std::array<Leg, 3> support_legs =
                {
                    Leg::FR,
                    Leg::HL,
                    Leg::HR
                };

                for (Leg leg : support_legs)
                {
                    const int i =
                        legIndex(leg);

                    const Vector3d q =
                        q_stair[i];

                    const Vector3d G =
                        dyn.gravityTorque(
                            leg,
                            q);

                    const Eigen::Matrix3d J =
    dyn.footJacobian(
        leg,
        q);

                    Vector3d F(
                        0.0,
                        0.0,
                        Fz4[i]);

                    const Vector3d tau =
                        G +
                        J.transpose() * F;

                    for (int j = 0; j < 3; ++j)
                    {
                        const double torque =
                            std::abs(tau(j));

                        if (torque > max_tau[i][j])
                            max_tau[i][j] =
                                torque;

                        if (torque > global_max)
                        {
                            global_max =
                                torque;

                            if (i == 1)
                                worst_leg = "FR";
                            else if (i == 2)
                                worst_leg = "HL";
                            else
                                worst_leg = "HR";

                            if (j == 0)
                                worst_joint = "HipX";
                            else if (j == 1)
                                worst_joint = "HipY";
                            else
                                worst_joint = "Knee";
                        }
                    }
                }
            }

            // ------------------------------------------------
            // Print result
            // ------------------------------------------------

            std::cout
                << "\n"
                << "Height = "
                << h * 100.0
                << " cm, T = "
                << T
                << " s\n";

            std::cout
                << "  FL  : "
                << max_tau[0][0]
                << ", "
                << max_tau[0][1]
                << ", "
                << max_tau[0][2]
                << " Nm\n";

            std::cout
                << "  FR  : "
                << max_tau[1][0]
                << ", "
                << max_tau[1][1]
                << ", "
                << max_tau[1][2]
                << " Nm\n";

            std::cout
                << "  HL  : "
                << max_tau[2][0]
                << ", "
                << max_tau[2][1]
                << ", "
                << max_tau[2][2]
                << " Nm\n";

            std::cout
                << "  HR  : "
                << max_tau[3][0]
                << ", "
                << max_tau[3][1]
                << ", "
                << max_tau[3][2]
                << " Nm\n";

            std::cout
                << "  Global max |tau| = "
                << global_max
                << " Nm\n"
                << "  Worst            = "
                << worst_leg
                << " "
                << worst_joint
                << "\n"
                << "  Min Fz           = "
                << min_Fz
                << " N\n"
                << "  Min sigma FL     = "
                << min_sigma_FL
                << "\n";

            // ------------------------------------------------
            // CSV summary
            // ------------------------------------------------

            summary
                << h << ","
                << T << ",";

            for (int i = 0; i < 4; ++i)
            {
                for (int j = 0; j < 3; ++j)
                {
                    summary
                        << max_tau[i][j]
                        << ",";
                }
            }

            summary
                << global_max << ","
                << worst_leg << ","
                << worst_joint << ","
                << min_Fz << ","
                << min_sigma_FL
                << "\n";
        }
    }

    summary.close();

    std::cout
        << "\n====================================================\n"
        << "Analysis complete.\n"
        << "Output:\n"
        << "  lite3_stair_contact_aware_summary.csv\n"
        << "====================================================\n";

    return 0;
}