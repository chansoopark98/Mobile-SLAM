// Diagnostic for the exact covariance whitening expression in IMUFactor/PnP.
#include <Eigen/Dense>
#include <iostream>
#include "backend/factor/integration_base.h"

namespace utility {
Config g_config;
}

int main() {
  const Eigen::Vector3d gravity(0, 0, 9.81);
  for (int count : {0, 1, 2, 6}) {
    backend::factor::IntegrationBase integration(
        gravity, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero());
    for (int i = 0; i < count; ++i) {
      integration.push_back(1.0 / 60, gravity, Eigen::Vector3d::Zero());
    }
    const auto inverse = integration.covariance.inverse().eval();
    Eigen::LLT<Eigen::Matrix<double, 15, 15>> whitening(inverse);
    Eigen::Matrix<double, 15, 15> sqrt_info = whitening.matrixL().transpose();
    const auto rank = integration.covariance.fullPivLu().rank();
    std::cout << (whitening.info() == Eigen::Success && sqrt_info.allFinite()
                      ? "WHITENING_FINITE" : "WHITENING_INVALID")
              << " samples=" << count << " sum_dt=" << integration.sum_dt
              << " covariance_rank=" << rank
              << " inverse_finite=" << inverse.allFinite()
              << " LLT_info=" << whitening.info()
              << " sqrt_info_finite=" << sqrt_info.allFinite() << '\n';
  }
}
