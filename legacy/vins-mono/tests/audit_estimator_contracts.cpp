// Standalone diagnostic: compile against the current evaluator and math helper.
// A detected defect is printed as CONTRACT_FAIL; it is not a passing regression.
#include <Eigen/Dense>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>

#include "utility/trajectory_evaluator.h"
#include "utility/utility.h"

namespace {
const std::vector<Eigen::Vector3d> kPositions = {
    {0, 0, 0}, {1, 0, 0}, {1, 1, 0}};

void writeGt(const std::filesystem::path& path) {
  std::ofstream f(path);
  f << "#timestamp_ns,px,py,pz,qw,qx,qy,qz\n";
  for (int i = 0; i < 3; ++i) {
    f << static_cast<long long>(i) * 1000000000LL << ','
      << kPositions[i].x() << ',' << kPositions[i].y() << ",0,1,0,0,0\n";
  }
}

void writeEst(const std::filesystem::path& path, bool rotate,
              bool unmatched_prefix, double scale) {
  std::ofstream f(path);
  f << std::setprecision(17);
  if (unmatched_prefix) {
    for (double t : {-100.0, -99.99, -99.98}) {
      f << t << " 0 0 0 0 0 0 1\n";
    }
  }
  for (int i = 0; i < 3; ++i) {
    const double a = rotate ? i * std::acos(-1.0) / 2 : 0;
    f << i << ' ' << scale * kPositions[i].x() << ' '
      << scale * kPositions[i].y() << " 0 0 0 " << std::sin(a / 2)
      << ' ' << std::cos(a / 2) << '\n';
  }
}

utility::TrajectoryEvaluator evaluator(const std::filesystem::path& dir,
                                      const char* name, bool rotate,
                                      bool prefix, double scale) {
  const auto est = dir / (std::string(name) + ".txt");
  writeEst(est, rotate, prefix, scale);
  utility::TrajectoryEvaluator e;
  if (!e.loadVioTrajectory(est.string()) ||
      !e.loadGroundTruth((dir / "gt.csv").string()) ||
      e.associateTrajectories(0.001) != 3 || !e.alignTrajectories()) {
    throw std::runtime_error("Cannot construct the diagnostic fixture");
  }
  return e;
}
}  // namespace

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: audit_estimator_contracts <fixture-output-dir>\n";
    return 2;
  }
  const std::filesystem::path dir(argv[1]);
  std::filesystem::create_directories(dir);
  writeGt(dir / "gt.csv");
  std::cout << std::setprecision(12);

  // Same Eigen comma assignment and aliased column-major input as configure().
  Eigen::Matrix3d R;
  R << -0.9995250378696743, 0.0075019185074052044, -0.02989013031643309,
      0.029615343885863205, -0.03439736061393144, -0.998969345370175,
      -0.008522328211654736, -0.9993800792498829, 0.03415885127385616;
  const double before = (R.transpose() * R - Eigen::Matrix3d::Identity()).norm();
  const double* r_ic = R.data();
  R << r_ic[0], r_ic[1], r_ic[2], r_ic[3], r_ic[4], r_ic[5],
      r_ic[6], r_ic[7], r_ic[8];
  const double after = (R.transpose() * R - Eigen::Matrix3d::Identity()).norm();
  std::cout << (after > 1e-6 ? "CONTRACT_FAIL" : "CONTRACT_PASS")
            << " R_IC_alias orthogonality_before=" << before
            << " after=" << after << " det_after=" << R.determinant() << '\n';

  auto rotation = evaluator(dir, "rotation90", true, false, 1);
  const auto rot = rotation.computeRPE(1);
  const double expected_rot = std::acos(-1.0) / 2;
  std::cout << (std::abs(rot.rmse_rot - expected_rot) > 1e-6 ? "CONTRACT_FAIL" : "CONTRACT_PASS")
            << " rotational_RPE expected_rad=" << expected_rot
            << " observed_rad=" << rot.rmse_rot << " pairs=" << rot.num_pairs << '\n';

  auto prefix = evaluator(dir, "unmatched_prefix", false, true, 1);
  const auto pre = prefix.computeRPE(1);
  std::cout << (pre.num_pairs != 2 ? "CONTRACT_FAIL" : "CONTRACT_PASS")
            << " associated_RPE_timestamps expected_pairs=2 observed_pairs="
            << pre.num_pairs << " matches=" << prefix.getMatchedSize() << '\n';

  auto doubled = evaluator(dir, "double_scale", false, false, 2);
  const auto ate = doubled.computeATE();
  // Best rigid fit preserves scale: centered points give sqrt(4/9)=2/3 m.
  std::cout << "METRIC_LIMIT metric_SE3_oracle_m=" << 2.0 / 3
            << " reported_Sim3_ATE_m=" << ate.rmse << '\n';

  const auto propagated = utility::Utility::deltaQ(Eigen::Vector3d(0, 0, 1)).toRotationMatrix();
  std::cout << "CONTRACT_FAIL propagation_SO3 theta_rad=1 det="
            << propagated.determinant() << " orthogonality="
            << (propagated.transpose() * propagated - Eigen::Matrix3d::Identity()).norm() << '\n';

  std::cout << "DIAGNOSTIC_ONLY no_accuracy_or_production_acceptance\n";
  return 0;
}
