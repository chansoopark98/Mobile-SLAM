#ifndef BACKEND__FACTOR__COVARIANCE_WHITENING_H
#define BACKEND__FACTOR__COVARIANCE_WHITENING_H

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <limits>

namespace backend::factor {

// Sigma = L L^T. The triangular solve preserves r^T Sigma^-1 r without
// inverting Sigma or adding information to an unobservable covariance.
inline bool covarianceWhitening(const Eigen::Matrix<double, 15, 15>& covariance,
                                Eigen::Matrix<double, 15, 15>& whitening) {
    if (!covariance.allFinite()) return false;
    const double scale = covariance.cwiseAbs().maxCoeff();
    if (!(scale > 0) ||
        (covariance - covariance.transpose()).cwiseAbs().maxCoeff() > 1e-10 * scale) return false;
    Eigen::LLT<Eigen::Matrix<double, 15, 15>> llt(covariance);
    if (llt.info() != Eigen::Success) return false;
    const Eigen::Matrix<double, 15, 15> lower = llt.matrixL();
    // Eigen can report Success for rank-deficient matrices with rounding-sized
    // positive pivots. Check the pivots as well as the decomposition status.
    const double pivot_floor = 64 * std::numeric_limits<double>::epsilon() * scale;
    for (int i = 0; i < 15; ++i) {
        if (!(lower(i, i) > 0) || lower(i, i) * lower(i, i) <= pivot_floor) return false;
    }
    whitening = lower.triangularView<Eigen::Lower>().solve(Eigen::Matrix<double, 15, 15>::Identity());
    return whitening.allFinite();
}

}  // namespace backend::factor
#endif
