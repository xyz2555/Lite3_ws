#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

struct Result
{
  int height_mm;
  double duration_s;

  double hipx;
  double hipy;
  double knee;
  double norm;
};

bool parseLine(
  const std::string &line,
  Result &r)
{
  std::stringstream ss(line);
  std::string cell;
  std::vector<std::string> fields;

  while (std::getline(ss, cell, ','))
  {
    fields.push_back(cell);
  }

  /*
   * From summary CSV:
   *
   * 0  height_mm
   * 1  duration_s
   *
   * ...
   *
   * 14 max_tauID_hipx
   * 15 max_tauID_hipy
   * 16 max_tauID_knee
   * 17 max_tauID_norm
   */

  if (fields.size() < 18)
  {
    return false;
  }

  try
  {
    r.height_mm = std::stoi(fields[0]);
    r.duration_s = std::stod(fields[1]);

    r.hipx = std::stod(fields[14]);
    r.hipy = std::stod(fields[15]);
    r.knee = std::stod(fields[16]);
    r.norm = std::stod(fields[17]);
  }
  catch (...)
  {
    return false;
  }

  return true;
}

int main()
{
  constexpr double J60_6_PEAK = 19.94;
  constexpr double J60_10_PEAK = 30.50;

  std::ifstream input(
    "lite3_stair_inverse_dynamics_summary.csv");

  if (!input.is_open())
  {
    std::cerr
      << "Cannot open "
      << "lite3_stair_inverse_dynamics_summary.csv\n";

    return 1;
  }

  std::string line;

  // Skip header.
  if (!std::getline(input, line))
  {
    return 1;
  }

  std::vector<Result> results;

  while (std::getline(input, line))
  {
    if (line.empty())
    {
      continue;
    }

    Result r;

    if (parseLine(line, r))
    {
      results.push_back(r);
    }
  }

  if (results.empty())
  {
    std::cerr
      << "No valid rows found.\n";

    return 1;
  }

  std::ofstream output(
    "lite3_actuator_feasibility.csv");

  if (!output.is_open())
  {
    std::cerr
      << "Cannot create output CSV.\n";

    return 1;
  }

  output
    << "height_mm,duration_s,"
    << "hipx_pct_J60_6,hipy_pct_J60_6,knee_pct_J60_6,"
    << "norm_pct_J60_6,"
    << "hipx_pct_J60_10,hipy_pct_J60_10,knee_pct_J60_10,"
    << "norm_pct_J60_10\n";

  std::cout << std::setprecision(6);

  double worst_J60_6 = 0.0;
  double worst_J60_10 = 0.0;

  Result worst_result_6{};
  Result worst_result_10{};

  for (const auto &r : results)
  {
    const double p6_hipx =
      100.0 * r.hipx / J60_6_PEAK;

    const double p6_hipy =
      100.0 * r.hipy / J60_6_PEAK;

    const double p6_knee =
      100.0 * r.knee / J60_6_PEAK;

    const double p6_norm =
      100.0 * r.norm / J60_6_PEAK;

    const double p10_hipx =
      100.0 * r.hipx / J60_10_PEAK;

    const double p10_hipy =
      100.0 * r.hipy / J60_10_PEAK;

    const double p10_knee =
      100.0 * r.knee / J60_10_PEAK;

    const double p10_norm =
      100.0 * r.norm / J60_10_PEAK;

    output
      << r.height_mm << ","
      << r.duration_s << ","

      << p6_hipx << ","
      << p6_hipy << ","
      << p6_knee << ","
      << p6_norm << ","

      << p10_hipx << ","
      << p10_hipy << ","
      << p10_knee << ","
      << p10_norm
      << "\n";

    if (p6_norm > worst_J60_6)
    {
      worst_J60_6 = p6_norm;
      worst_result_6 = r;
    }

    if (p10_norm > worst_J60_10)
    {
      worst_J60_10 = p10_norm;
      worst_result_10 = r;
    }
  }

  std::cout
    << "\n========================================\n"
    << "ACTUATOR FEASIBILITY SCREENING\n"
    << "========================================\n";

  std::cout
    << "\nJ60-6 peak torque = "
    << J60_6_PEAK
    << " Nm\n";

  std::cout
    << "Worst case:\n"
    << "  Height   = "
    << worst_result_6.height_mm
    << " mm\n"
    << "  Duration = "
    << worst_result_6.duration_s
    << " s\n"
    << "  HipX     = "
    << worst_result_6.hipx
    << " Nm\n"
    << "  HipY     = "
    << worst_result_6.hipy
    << " Nm\n"
    << "  Knee     = "
    << worst_result_6.knee
    << " Nm\n"
    << "  Norm     = "
    << worst_result_6.norm
    << " Nm\n"
    << "  Peak utilization = "
    << worst_J60_6
    << "%\n";

  std::cout
    << "\nJ60-10 peak torque = "
    << J60_10_PEAK
    << " Nm\n";

  std::cout
    << "Worst case:\n"
    << "  Height   = "
    << worst_result_10.height_mm
    << " mm\n"
    << "  Duration = "
    << worst_result_10.duration_s
    << " s\n"
    << "  HipX     = "
    << worst_result_10.hipx
    << " Nm\n"
    << "  HipY     = "
    << worst_result_10.hipy
    << " Nm\n"
    << "  Knee     = "
    << worst_result_10.knee
    << " Nm\n"
    << "  Norm     = "
    << worst_result_10.norm
    << " Nm\n"
    << "  Peak utilization = "
    << worst_J60_10
    << "%\n";

  std::cout
    << "\nOutput:\n"
    << "lite3_actuator_feasibility.csv\n";

  return 0;
}