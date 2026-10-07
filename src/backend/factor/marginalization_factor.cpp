#include "backend/factor/marginalization_factor.h"
#include <Eigen/QR>
#include <chrono>
#include <iomanip>
#include <limits>
#include <memory>
#include <new>
#include <sstream>

namespace backend {
namespace factor {

namespace {
constexpr Eigen::Index kDiagnosticMatrixCells = 65536;
constexpr Eigen::Index kDiagnosticVectorRows = 1000;
bool diagnosticSquareFits(int dimension) {
    const auto size = static_cast<std::size_t>(dimension);
    return !size || size <= static_cast<std::size_t>(kDiagnosticMatrixCells) / size;
}
void diagnosticNumber(std::ostream& out, double value) {
    if (std::isfinite(value)) out << std::setprecision(17) << value;
    else out << "null";
}
void diagnosticMatrix(std::ostream& out, const Eigen::MatrixXd& matrix) {
    const bool truncated = matrix.size() > kDiagnosticMatrixCells;
    out << "{\"rows\":" << matrix.rows() << ",\"cols\":" << matrix.cols()
        << ",\"cellLimit\":" << kDiagnosticMatrixCells << ",\"truncated\":" << (truncated ? "true" : "false")
        << ",\"nonfinite\":" << (matrix.allFinite() ? "false" : "true") << ",\"values\":";
    if (truncated) out << "null";
    else {
        out << '[';
        for (Eigen::Index row = 0; row < matrix.rows(); ++row) for (Eigen::Index col = 0; col < matrix.cols(); ++col) {
            if (row || col) out << ',';
            diagnosticNumber(out, matrix(row, col));
        }
        out << ']';
    }
    out << '}';
}
void diagnosticSpectrum(std::ostream& out, const Eigen::MatrixXd& matrix,
                        const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd>& eigen) {
    const auto& values = eigen.eigenvalues();
    const double scale = values.size() ? values.cwiseAbs().maxCoeff() : 0;
    const double cutoff = matrix.rows() * std::numeric_limits<double>::epsilon() * scale;
    out << "{\"dimension\":" << matrix.rows() << ",\"eigenInfo\":" << int(eigen.info())
        << ",\"rankUsedToFormPrior\":false,\"rankThresholdDiagnosticOnly\":"; diagnosticNumber(out, cutoff);
    out << ",\"frobeniusNorm\":"; diagnosticNumber(out, matrix.norm());
    out << ",\"symmetryDefectNorm\":"; diagnosticNumber(out, (matrix - matrix.transpose()).norm());
    out << ",\"spectralNormFromActualEigenvalues\":"; diagnosticNumber(out, scale);
    out << ",\"dimensionEpsilonScaleNotUsedForRank\":";
    diagnosticNumber(out, matrix.rows() * std::numeric_limits<double>::epsilon() * scale);
    out << ",\"retainedRankDiagnosticOnly\":" << (values.array() > cutoff).count()
        << ",\"negativeEigenvalues\":" << (values.array() < 0).count()
        << ",\"nonfinite\":" << (matrix.allFinite() && values.allFinite() && eigen.eigenvectors().allFinite() ? "false" : "true")
        << ",\"eigenvaluesTruncated\":" << (values.size() > kDiagnosticVectorRows ? "true" : "false")
        << ",\"eigenvalues\":[";
    for (Eigen::Index i = 0; i < std::min(values.size(), kDiagnosticVectorRows); ++i) {
        if (i) out << ',';
        diagnosticNumber(out, values[i]);
    }
    out << "]";
    const bool truncated = matrix.size() > kDiagnosticMatrixCells;
    out << ",\"residualMetricsTruncated\":" << (truncated ? "true" : "false") << ",\"eigenResidualRelative\":";
    if (truncated) out << "null";
    else {
        // Eigen's actual operation uses the lower self-adjoint triangle.
        const Eigen::MatrixXd effective = matrix.selfadjointView<Eigen::Lower>();
        diagnosticNumber(out, (effective * eigen.eigenvectors() - eigen.eigenvectors() * values.asDiagonal()).norm() /
                              std::max(1.0, effective.norm()));
    }
    out << ",\"matrix\":"; diagnosticMatrix(out, matrix); out << '}';
}
}  // namespace

void MarginalizationInfo::setDiagnosticCapture(bool enabled) {
    diagnostic_capture_ = enabled;
    if (!enabled) diagnostic_json_ = "{\"enabled\":false}";
}

std::string MarginalizationInfo::getPriorNormalDiagnostics(const std::vector<double*>& parameters) {
    if (!diagnostic_capture_) return "{\"enabled\":false}";
    if (linearized_jacobians.size() > kDiagnosticMatrixCells || parameters.size() != keep_block_size.size())
        return "{\"enabled\":true,\"truncated\":true,\"evaluationAvailable\":false}";
    Eigen::VectorXd current_residual(n);
    MarginalizationFactor prior(this);
    const bool evaluated = prior.Evaluate(parameters.data(), current_residual.data(), nullptr);
    const Eigen::MatrixXd normal = linearized_jacobians.transpose() * linearized_jacobians;
    const Eigen::VectorXd current_gradient = linearized_jacobians.transpose() * current_residual;
    const Eigen::VectorXd reference_gradient = linearized_jacobians.transpose() * linearized_residuals;
    std::ostringstream out;
    out << "{\"enabled\":true,\"truncated\":false,\"evaluationAvailable\":" << (evaluated ? "true" : "false")
        << ",\"nonfinite\":" << (normal.allFinite() && current_gradient.allFinite() && reference_gradient.allFinite() ? "false" : "true")
        << ",\"currentParameterResidualNorm\":"; diagnosticNumber(out, current_residual.norm());
    out << ",\"normal\":"; diagnosticMatrix(out, normal);
    out << ",\"currentParameterGradient\":"; diagnosticMatrix(out, current_gradient);
    out << ",\"linearizationReferenceGradient\":"; diagnosticMatrix(out, reference_gradient);
    out << '}';
    return out.str();
}

bool ResidualBlockInfo::Evaluate() {
    residuals.resize(cost_function->num_residuals());

    std::vector<int> block_sizes = cost_function->parameter_block_sizes();
    delete[] raw_jacobians;
    raw_jacobians = nullptr;
    raw_jacobians = new double*[block_sizes.size()];
    jacobians.resize(block_sizes.size());

    for (int i = 0; i < static_cast<int>(block_sizes.size()); i++) {
        jacobians[i].resize(cost_function->num_residuals(), block_sizes[i]);
        raw_jacobians[i] = jacobians[i].data();
        // dim += block_sizes[i] == 7 ? 6 : block_sizes[i];
    }
    if (!cost_function->Evaluate(parameter_blocks.data(), residuals.data(), raw_jacobians)) return false;

    if (loss_function) {
        double residual_scaling_, alpha_sq_norm_;

        double sq_norm, rho[3];

        sq_norm = residuals.squaredNorm();
        loss_function->Evaluate(sq_norm, rho);
        // printf("sq_norm: %f, rho[0]: %f, rho[1]: %f, rho[2]: %f\n", sq_norm,
        // rho[0], rho[1], rho[2]);

        double sqrt_rho1_ = sqrt(rho[1]);

        if ((sq_norm == 0.0) || (rho[2] <= 0.0)) {
            residual_scaling_ = sqrt_rho1_;
            alpha_sq_norm_ = 0.0;
        } else {
            const double D = 1.0 + 2.0 * sq_norm * rho[2] / rho[1];
            const double alpha = 1.0 - sqrt(D);
            residual_scaling_ = sqrt_rho1_ / (1 - alpha);
            alpha_sq_norm_ = alpha / sq_norm;
        }

        for (int i = 0; i < static_cast<int>(parameter_blocks.size()); i++) {
            jacobians[i] =
                sqrt_rho1_ * (jacobians[i] - alpha_sq_norm_ * residuals * (residuals.transpose() * jacobians[i]));
        }

        residuals *= residual_scaling_;
    }
    return true;
}

MarginalizationInfo::~MarginalizationInfo() {
    for (auto it = parameter_block_data.begin(); it != parameter_block_data.end(); ++it)
        delete[] it->second;

    for (int i = 0; i < (int)factors.size(); i++) {
        delete[] factors[i]->raw_jacobians;

        delete factors[i]->cost_function;

        delete factors[i];
    }
}

void MarginalizationInfo::addResidualBlockInfo(ResidualBlockInfo* residual_block_info) {
    factors.emplace_back(residual_block_info);

    std::vector<double*>& parameter_blocks = residual_block_info->parameter_blocks;
    std::vector<int> parameter_block_sizes = residual_block_info->cost_function->parameter_block_sizes();

    for (int i = 0; i < static_cast<int>(residual_block_info->parameter_blocks.size()); i++) {
        double* addr = parameter_blocks[i];
        int size = parameter_block_sizes[i];
        if (parameter_block_size.find(reinterpret_cast<long>(addr)) == parameter_block_size.end())
            parameter_block_order_.push_back(reinterpret_cast<long>(addr));
        parameter_block_size[reinterpret_cast<long>(addr)] = size;
    }

    for (int i = 0; i < static_cast<int>(residual_block_info->drop_set.size()); i++) {
        double* addr = parameter_blocks[residual_block_info->drop_set[i]];
        parameter_block_idx[reinterpret_cast<long>(addr)] = 0;
    }
}

void MarginalizationInfo::preMarginalize() {
    statistics_ = MarginalizationStatistics{};
    failure_reason_.clear();
    linearized_jacobians.resize(0, 0);
    linearized_residuals.resize(0);
    try {
        if (!prepareWorkspace()) return;
        for (auto* factor : factors) {
            if (!factor->Evaluate()) {
                failure_reason_ = "marginalization_factor_evaluation_failed";
                return;
            }
            const auto& block_sizes = factor->cost_function->parameter_block_sizes();
            for (std::size_t i = 0; i < block_sizes.size(); ++i) {
                const long address = reinterpret_cast<long>(factor->parameter_blocks[i]);
                const int size = block_sizes[i];
                if (parameter_block_data.find(address) == parameter_block_data.end()) {
                    auto data = std::make_unique<double[]>(size);
                    std::memcpy(data.get(), factor->parameter_blocks[i], sizeof(double) * size);
                    parameter_block_data.emplace(address, data.get());
                    (void)data.release();
                }
            }
        }
    } catch (const std::bad_alloc&) {
        failure_reason_ = "marginalization_allocation_failed";
    }
}

int MarginalizationInfo::localSize(int size) const {
    return size == 7 ? 6 : size;
}

int MarginalizationInfo::globalSize(int size) const {
    return size == 6 ? 7 : size;
}

bool MarginalizationInfo::prepareWorkspace() {
    auto reject = [&](const char* reason) {
        failure_reason_ = reason;
        return false;
    };
    m = n = 0;
    parameter_block_idx.clear();
    for (const auto* factor : factors) for (int index : factor->drop_set)
        parameter_block_idx[reinterpret_cast<long>(factor->parameter_blocks[index])] = 0;
    int pos = 0;
    for (long address : parameter_block_order_) {
        auto dropped = parameter_block_idx.find(address);
        if (dropped == parameter_block_idx.end()) continue;
        const int size = localSize(parameter_block_size.at(address));
        if (size <= 0 || size > std::numeric_limits<int>::max() - pos)
            return reject("marginalization_dimension_overflow");
        dropped->second = pos;
        pos += size;
    }
    m = pos;
    for (long address : parameter_block_order_) {
        if (parameter_block_idx.find(address) != parameter_block_idx.end()) continue;
        const int size = localSize(parameter_block_size.at(address));
        if (size <= 0 || size > std::numeric_limits<int>::max() - pos)
            return reject("marginalization_dimension_overflow");
        parameter_block_idx[address] = pos;
        pos += size;
    }
    n = pos - m;
    for (const auto* factor : factors) {
        const auto rows = static_cast<std::size_t>(factor->cost_function->num_residuals());
        if (rows > static_cast<std::size_t>(std::numeric_limits<Eigen::Index>::max()) - statistics_.inputRows)
            return reject("marginalization_dimension_overflow");
        statistics_.inputRows += rows;
    }
    if (!n || !statistics_.inputRows) return reject("marginalization_empty_system");

    // Combined bound includes evaluated ambient factor J/r and saved parameters, then
    // D/CPQR, augmented kept/transform/reduced/kept-QR, outputs and scratch. No dense Q.
    auto add_cells = [&](std::size_t rows, std::size_t cols, std::size_t copies) {
        const auto limit = std::numeric_limits<std::size_t>::max();
        if ((cols && rows > limit / cols) || (rows * cols && copies > limit / (rows * cols)) ||
            rows * cols * copies > limit - statistics_.denseWorkspaceCells) {
            statistics_.workspaceEstimateOverflow = true;
            return false;
        }
        statistics_.denseWorkspaceCells += rows * cols * copies;
        return true;
    };
    for (const auto* factor : factors) {
        const std::size_t rows = static_cast<std::size_t>(factor->cost_function->num_residuals());
        if (!add_cells(rows, 1, 1)) return reject("marginalization_dimension_overflow");
        for (int size : factor->cost_function->parameter_block_sizes())
            if (!add_cells(rows, size, 1)) return reject("marginalization_dimension_overflow");
    }
    for (long address : parameter_block_order_)
        if (!add_cells(parameter_block_size.at(address), 1, 1))
            return reject("marginalization_dimension_overflow");
    if (!add_cells(statistics_.inputRows, m, 2) || !add_cells(statistics_.inputRows, std::size_t(n) + 1, 3) ||
        !add_cells(n, n, 4) || !add_cells(statistics_.inputRows, 1, 16) ||
        !add_cells(m, 1, 16) || !add_cells(std::size_t(n) + 1, 1, 16))
        return reject("marginalization_dimension_overflow");
    if (diagnostic_capture_ &&
        ((diagnosticSquareFits(m) && !add_cells(m, m, 8)) ||
         (diagnosticSquareFits(n) && !add_cells(n, n, 8))))
        return reject("marginalization_dimension_overflow");
    if (statistics_.denseWorkspaceCells > kMarginalizationDenseWorkspaceCellBudget)
        return reject("marginalization_dense_budget_exceeded");

    return true;
}

bool MarginalizationInfo::marginalize() {
    linearized_jacobians.resize(0, 0);
    linearized_residuals.resize(0);
    const auto started = diagnostic_capture_ ? std::chrono::steady_clock::now() : std::chrono::steady_clock::time_point{};
    Eigen::MatrixXd diagnostic_amm;
    auto finish = [&](bool success) {
        if (!diagnostic_capture_) return success;
        statistics_.timingAvailable = true;
        statistics_.elapsedMilliseconds = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - started).count();
        std::ostringstream out;
        out << "{\"enabled\":true,\"method\":\"square_root_qr\",\"success\":" << (success ? "true" : "false")
            << ",\"failureReason\":\"" << failure_reason_ << "\",\"assemblyThreads\":" << kMarginalizationAssemblyThreads
            << ",\"inputRows\":" << statistics_.inputRows << ",\"droppedDimension\":" << m
            << ",\"keptDimension\":" << n << ",\"droppedRank\":" << statistics_.droppedRank
            << ",\"denseWorkspaceCellBudget\":" << kMarginalizationDenseWorkspaceCellBudget
            << ",\"workspaceEstimateOverflow\":" << (statistics_.workspaceEstimateOverflow ? "true" : "false")
            << ",\"denseWorkspaceCells\":";
        if (statistics_.workspaceEstimateOverflow) out << "null";
        else out << statistics_.denseWorkspaceCells;
        out << ",\"timingAvailable\":true,\"assemblyAndQrMilliseconds\":";
        diagnosticNumber(out, statistics_.elapsedMilliseconds);
        out << ",\"normalMatricesDiagnosticOnly\":true,\"AmmDiagnosticTruncated\":"
            << (!diagnosticSquareFits(m) ? "true" : "false")
            << ",\"SchurDiagnosticTruncated\":" << (linearized_jacobians.size() > kDiagnosticMatrixCells ? "true" : "false")
            << ",\"nonfinite\":" << (failure_reason_ == "marginalization_nonfinite_input" ||
                                      failure_reason_ == "marginalization_nonfinite_prior" ? "true" : "false")
            << ",\"Amm\":";
        if (!success || diagnostic_amm.rows() == 0) out << "null";
        else {
            Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen(diagnostic_amm);
            diagnosticSpectrum(out, diagnostic_amm, eigen);
        }
        out << ",\"Schur\":";
        if (!success || linearized_jacobians.size() > kDiagnosticMatrixCells) out << "null";
        else {
            const Eigen::MatrixXd normal = linearized_jacobians.transpose() * linearized_jacobians;
            Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen(normal);
            diagnosticSpectrum(out, normal, eigen);
        }
        out << ",\"SchurGradient\":";
        if (!success || linearized_jacobians.size() > kDiagnosticMatrixCells) out << "null";
        else diagnosticMatrix(out, linearized_jacobians.transpose() * linearized_residuals);
        out << '}';
        diagnostic_json_ = out.str();
        return success;
    };
    auto fail = [&](const char* reason) {
        failure_reason_ = reason;
        linearized_jacobians.resize(0, 0);
        linearized_residuals.resize(0);
        return finish(false);
    };

    if (!failure_reason_.empty()) return finish(false);
    if (!n || !statistics_.inputRows) return fail("marginalization_empty_system");

    try {
        const Eigen::Index rows = static_cast<Eigen::Index>(statistics_.inputRows);
        Eigen::MatrixXd dropped = Eigen::MatrixXd::Zero(rows, m);
        Eigen::MatrixXd kept = Eigen::MatrixXd::Zero(rows, Eigen::Index(n) + 1);
        Eigen::Index row = 0;
        for (const auto* factor : factors) {
            const Eigen::Index count = factor->residuals.size();
            kept.block(row, n, count, 1) = factor->residuals;
            for (std::size_t i = 0; i < factor->parameter_blocks.size(); ++i) {
                const long address = reinterpret_cast<long>(factor->parameter_blocks[i]);
                const int column = parameter_block_idx.at(address);
                const int size = localSize(parameter_block_size.at(address));
                if (column < m) dropped.block(row, column, count, size) += factor->jacobians[i].leftCols(size);
                else kept.block(row, column - m, count, size) += factor->jacobians[i].leftCols(size);
            }
            row += count;
        }
        if (!dropped.allFinite() || !kept.allFinite()) return fail("marginalization_nonfinite_input");
        if (diagnostic_capture_ && m && diagnosticSquareFits(m))
            diagnostic_amm = dropped.transpose() * dropped;
        if (m) {
            Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(dropped);
            // Eigen3.4 default: diagonalSize*epsilon, relative to the largest pivot.
            // Only dropped-column rank is determined; kept information is never thresholded.
            statistics_.droppedRank = static_cast<int>(qr.rank());
            kept = (qr.householderQ().adjoint() * kept).eval();
        }
        dropped.resize(0, 0);
        Eigen::MatrixXd reduced = kept.bottomRows(rows - statistics_.droppedRank);
        kept.resize(0, 0);
        linearized_jacobians = Eigen::MatrixXd::Zero(n, n);
        linearized_residuals = Eigen::VectorXd::Zero(n);
        if (reduced.rows()) {
            // Unpivoted kept QR preserves column semantics and all weak observable modes.
            Eigen::HouseholderQR<Eigen::MatrixXd> qr(reduced.leftCols(n));
            const Eigen::VectorXd residual = qr.householderQ().adjoint() * reduced.col(n);
            const Eigen::Index retained_rows = std::min(reduced.rows(), Eigen::Index(n));
            linearized_jacobians.topRows(retained_rows) =
                qr.matrixQR().topRows(retained_rows).template triangularView<Eigen::Upper>();
            linearized_residuals.head(retained_rows) = residual.head(retained_rows);
        }
        if (!linearized_jacobians.allFinite() || !linearized_residuals.allFinite())
            return fail("marginalization_nonfinite_prior");
        return finish(true);
    } catch (const std::bad_alloc&) {
        return fail("marginalization_allocation_failed");
    }
}

std::vector<double*> MarginalizationInfo::getParameterBlocks(std::unordered_map<long, double*>& addr_shift) {
    std::vector<double*> keep_block_addr;
    keep_block_size.clear();
    keep_block_idx.clear();
    keep_block_data.clear();

    for (long address : parameter_block_order_) {
        if (parameter_block_idx.at(address) >= m) {
            keep_block_size.push_back(parameter_block_size.at(address));
            keep_block_idx.push_back(parameter_block_idx.at(address));
            keep_block_data.push_back(parameter_block_data.at(address));
            keep_block_addr.push_back(addr_shift.at(address));
        }
    }
    sum_block_size = std::accumulate(std::begin(keep_block_size), std::end(keep_block_size), 0);

    return keep_block_addr;
}

MarginalizationFactor::MarginalizationFactor(MarginalizationInfo* _marginalization_info)
    : marginalization_info(_marginalization_info) {
    int cnt = 0;
    for (auto it : marginalization_info->keep_block_size) {
        mutable_parameter_block_sizes()->push_back(it);
        cnt += it;
    }
    // printf("residual size: %d, %d\n", cnt, n);
    set_num_residuals(marginalization_info->n);
};

bool MarginalizationFactor::Evaluate(double const* const* parameters, double* residuals, double** jacobians) const {
    int n = marginalization_info->n;
    int m = marginalization_info->m;
    Eigen::VectorXd dx(n);
    for (int i = 0; i < static_cast<int>(marginalization_info->keep_block_size.size()); i++) {
        int size = marginalization_info->keep_block_size[i];
        int idx = marginalization_info->keep_block_idx[i] - m;
        Eigen::VectorXd x = Eigen::Map<const Eigen::VectorXd>(parameters[i], size);
        Eigen::VectorXd x0 = Eigen::Map<const Eigen::VectorXd>(marginalization_info->keep_block_data[i], size);
        if (size != 7)
            dx.segment(idx, size) = x - x0;
        else {
            dx.segment<3>(idx + 0) = x.head<3>() - x0.head<3>();
            dx.segment<3>(idx + 3) = 2.0 * Utility::positify(Eigen::Quaterniond(x0(6), x0(3), x0(4), x0(5)).inverse() *
                                                             Eigen::Quaterniond(x(6), x(3), x(4), x(5)))
                                               .vec();
            if (!((Eigen::Quaterniond(x0(6), x0(3), x0(4), x0(5)).inverse() *
                   Eigen::Quaterniond(x(6), x(3), x(4), x(5)))
                      .w() >= 0)) {
                dx.segment<3>(idx + 3) =
                    2.0 * -Utility::positify(Eigen::Quaterniond(x0(6), x0(3), x0(4), x0(5)).inverse() *
                                             Eigen::Quaterniond(x(6), x(3), x(4), x(5)))
                               .vec();
            }
        }
    }
    Eigen::Map<Eigen::VectorXd>(residuals, n) =
        marginalization_info->linearized_residuals + marginalization_info->linearized_jacobians * dx;
    if (jacobians) {
        for (int i = 0; i < static_cast<int>(marginalization_info->keep_block_size.size()); i++) {
            if (jacobians[i]) {
                int size = marginalization_info->keep_block_size[i], local_size = marginalization_info->localSize(size);
                int idx = marginalization_info->keep_block_idx[i] - m;
                Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> jacobian(
                    jacobians[i], n, size);
                jacobian.setZero();
                jacobian.leftCols(local_size) = marginalization_info->linearized_jacobians.middleCols(idx, local_size);
            }
        }
    }
    return true;
}

}  // namespace factor
}  // namespace backend
