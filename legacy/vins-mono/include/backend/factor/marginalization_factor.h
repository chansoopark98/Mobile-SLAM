#ifndef BACKEND__FACTOR__MARGINALIZATION_FACTOR_H
#define BACKEND__FACTOR__MARGINALIZATION_FACTOR_H

#include <ceres/ceres.h>
#include <cstddef>
#include <string>
#include <unordered_map>

#include "utility/utility.h"

namespace backend {
namespace factor {

inline constexpr int kMarginalizationAssemblyThreads = 1;
inline constexpr std::size_t kMarginalizationDenseWorkspaceCellBudget = 8000000;

struct MarginalizationStatistics {
    std::size_t inputRows = 0;
    std::size_t denseWorkspaceCells = 0;
    bool workspaceEstimateOverflow = false;
    int droppedRank = 0;
    bool timingAvailable = false;
    double elapsedMilliseconds = 0;
};

struct ResidualBlockInfo {
    ResidualBlockInfo(ceres::CostFunction* _cost_function, ceres::LossFunction* _loss_function,
                      std::vector<double*> _parameter_blocks, std::vector<int> _drop_set)
        : cost_function(_cost_function),
          loss_function(_loss_function),
          parameter_blocks(_parameter_blocks),
          drop_set(_drop_set) {}

    bool Evaluate();

    ceres::CostFunction* cost_function;
    ceres::LossFunction* loss_function;
    std::vector<double*> parameter_blocks;
    std::vector<int> drop_set;

    double** raw_jacobians = nullptr;
    std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> jacobians;
    Eigen::VectorXd residuals;

    int localSize(int size) {
        return size == 7 ? 6 : size;
    }
};

class MarginalizationInfo {
public:
    ~MarginalizationInfo();
    int localSize(int size) const;
    int globalSize(int size) const;
    void addResidualBlockInfo(ResidualBlockInfo* residual_block_info);
    void preMarginalize();
    bool marginalize();
    std::vector<double*> getParameterBlocks(std::unordered_map<long, double*>& addr_shift);
    void setDiagnosticCapture(bool enabled);
    const std::string& getMarginalizationDiagnostics() const { return diagnostic_json_; }
    std::string getPriorNormalDiagnostics(const std::vector<double*>& parameters);
    const std::vector<long>& getParameterBlockOrder() const { return parameter_block_order_; }
    const std::string& getFailureReason() const { return failure_reason_; }
    const MarginalizationStatistics& getStatistics() const { return statistics_; }

    std::vector<ResidualBlockInfo*> factors;
    int m = 0, n = 0;
    std::unordered_map<long, int> parameter_block_size;  // global size
    int sum_block_size;
    std::unordered_map<long, int> parameter_block_idx;  // local size
    std::unordered_map<long, double*> parameter_block_data;

    std::vector<int> keep_block_size;  // global size
    std::vector<int> keep_block_idx;   // local size
    std::vector<double*> keep_block_data;

    Eigen::MatrixXd linearized_jacobians;
    Eigen::VectorXd linearized_residuals;

private:
    bool prepareWorkspace();

    // Residual/parameter insertion is semantic; physical addresses are lookup keys only.
    std::vector<long> parameter_block_order_;
    bool diagnostic_capture_ = false;
    std::string diagnostic_json_ = "{\"enabled\":false}";
    std::string failure_reason_;
    MarginalizationStatistics statistics_;
};

class MarginalizationFactor : public ceres::CostFunction {
public:
    MarginalizationFactor(MarginalizationInfo* _marginalization_info);
    virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const;

    MarginalizationInfo* marginalization_info;
};

}  // namespace factor
}  // namespace backend

#endif  // BACKEND__FACTOR__MARGINALIZATION_FACTOR_H
