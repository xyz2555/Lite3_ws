#include "lite3_kinematics/kinematics.hpp"

#include <Eigen/Dense>

#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <string>

using lite3_kinematics::Leg;
using lite3_kinematics::Lite3Kinematics;

int main(){
    Lite3Kinematics kin;

    constexpr std::size_t NUM_SAMPLES = 200000;

    const auto& lim = kin.limits();

    std::mt19937_64 rng(42);

    std::uniform_real_distribution<double> hipx_dist(
        lim.hipx_min,
        lim.hipx_max);

    std::uniform_real_distribution<double> hipy_dist(
        lim.hipy_min,
        lim.hipy_max);

    std::uniform_real_distribution<double> knee_dist(
        lim.knee_min,
        lim.knee_max);

    std::ofstream file("lite3_fl_workspace.csv");

    if(!file.is_open()){
        std::cerr
        << "Failed to open output CSV."
        << std::endl;

        return 1;
    }

    file 
    << "hipx,hipy,knee,x,y,z\n";

    double min_x = 1e9;
    double min_y = 1e9;
    double min_z = 1e9;

    double max_x = -1e9;
    double max_y = -1e9;
    double max_z = -1e9;

    for (std::size_t i = 0;
        i < NUM_SAMPLES;
        ++i)
    {
        Eigen::Vector3d q; 

        q << 
          hipx_dist(rng),
          hipy_dist(rng),
          knee_dist(rng);

        const Eigen::Vector3d p =
          kin.forward(Leg::FL, q);

        file
          << std::setprecision(12)
          << q.x() << ","
          << q.y() << ","
          << q.z() << ","
          << p.x() << ","
          << p.y() << ","
          << p.z()
          << "\n";

        min_x = std::min(min_x, p.x());
        max_x = std::max(max_x, p.x());

        min_y = std::min(min_y, p.y());
        max_y = std::max(max_y, p.y());

        min_z = std::min(min_z, p.z());
        max_z = std::max(max_z, p.z());
    }

    file.close();

    std::cout
    << "=============================================\n"
    << " Lite3 FL Workspace Generator\n"
    << "=============================================\n";

    std::cout
    << "Samples : "
    << NUM_SAMPLES
    << "\n\n";

  std::cout
    << "X range : ["
    << min_x << ", "
    << max_x << "] m\n";

  std::cout
    << "Y range : ["
    << min_y << ", "
    << max_y << "] m\n";

  std::cout
    << "Z range : ["
    << min_z << ", "
    << max_z << "] m\n";

  std::cout
    << "\nCSV output:\n"
    << "lite3_fl_workspace.csv\n";

  return 0;
}