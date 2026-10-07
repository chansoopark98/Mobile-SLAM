#ifndef BACKEND__SOLVER_DIAGNOSTICS_H
#define BACKEND__SOLVER_DIAGNOSTICS_H

#include <limits>
#include <string>

namespace backend {
struct SolverDiagnostics {
    bool usable = false;
    int iterations = 0;
    int terminationType = -1;
    double initialCost = std::numeric_limits<double>::quiet_NaN();
    double finalCost = std::numeric_limits<double>::quiet_NaN();
    int imuFactors = 0;
    int rejectedImuFactors = 0;
    int visualFactors = 0;
    int currentVisualFactors = 0;
    std::string reason = "not_solved";
    std::string qualityReason;
    // Opt-in Ceres iteration/message snapshot; absent when diagnostic capture is off.
    std::string numericalDetails = "null";
};
}  // namespace backend
#endif
