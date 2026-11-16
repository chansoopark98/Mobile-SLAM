#ifndef UTILITY_INERTIAL_PREDICTION_H
#define UTILITY_INERTIAL_PREDICTION_H

#include "common/common_types.h"
#include "utility/so3.h"
#include <cmath>
#include <cstdint>
#include <limits>

namespace utility {
class InertialPrediction {
public:
    static constexpr double max_horizon_seconds = .5;
    static constexpr double max_raw_gap_seconds = .1;
    static constexpr double time_tolerance = 1e-9;
    bool seed(const common::VINSResult& state, const Eigen::Vector3d& gravity,
              const Eigen::Matrix3d& ric, const Eigen::Vector3d& tic,
              const Eigen::Vector3d& acc, const Eigen::Vector3d& gyro, uint64_t epoch) {
        invalidate();
        if (!std::isfinite(state.timestamp) || state.timestamp < 0 || !state.P.allFinite() ||
            !state.V.allFinite() || !state.Ba.allFinite() || !state.Bg.allFinite() ||
            !so3::isRotation(state.R) || !so3::isRotation(ric) || !tic.allFinite() ||
            !gravity.allFinite() || !std::isfinite(gravity.stableNorm()) || gravity.stableNorm() <= 0 ||
            !acc.allFinite() || !gyro.allFinite()) return false;
        state_=state;gravity_=gravity;ric_=ric;tic_=tic;acc_=acc;gyro_=gyro;
        anchor_position_=state.P;anchor_timestamp_=state.timestamp;epoch_=epoch;valid_=true;
        return true;
    }
    void advance(double timestamp, const Eigen::Vector3d& acc, const Eigen::Vector3d& gyro, uint64_t epoch) {
        if (!valid_) return;
        const double dt=timestamp-state_.timestamp;
        if (!std::isfinite(timestamp) || epoch!=epoch_ || dt<0 ||
            dt>max_raw_gap_seconds+time_tolerance ||
            timestamp-anchor_timestamp_>max_horizon_seconds+time_tolerance ||
            !acc.allFinite() || !gyro.allFinite()) { invalidate();return; }
        if (dt==0) { acc_=acc;gyro_=gyro;return; }
        const Eigen::Vector3d a0=state_.R*(acc_-state_.Ba)-gravity_;
        const Eigen::Vector3d omega=.5*(gyro_+gyro)-state_.Bg;
        const Eigen::Matrix3d rotation=state_.R*so3::exp(omega*dt).toRotationMatrix();
        const Eigen::Vector3d a1=rotation*(acc-state_.Ba)-gravity_;
        const Eigen::Vector3d position=state_.P+dt*state_.V+dt*dt*(a0/3.0+a1/6.0);
        const Eigen::Vector3d velocity=state_.V+.5*dt*(a0+a1);
        if (!so3::isRotation(rotation) || !position.allFinite() || !velocity.allFinite() ||
            (position-anchor_position_).stableNorm()>5.0) { invalidate();return; }
        state_.R=rotation;state_.P=position;state_.V=velocity;state_.timestamp=timestamp;
        acc_=acc;gyro_=gyro;
    }
    void invalidate() { valid_=false; }
    bool valid() const { return valid_; }
    double timestamp() const { return valid_ ? state_.timestamp : -1; }
    double anchorTimestamp() const { return valid_ ? anchor_timestamp_ : -1; }
    uint64_t epoch() const { return epoch_; }
    Eigen::Matrix4d cameraPose() const {
        if (!valid_) return Eigen::Matrix4d::Constant(std::numeric_limits<double>::quiet_NaN());
        Eigen::Matrix4d pose=Eigen::Matrix4d::Identity();
        pose.block<3,3>(0,0)=state_.R*ric_;
        pose.block<3,1>(0,3)=state_.P+state_.R*tic_;
        return pose;
    }
private:
    bool valid_=false;
    common::VINSResult state_{};
    Eigen::Vector3d gravity_,tic_,acc_,gyro_,anchor_position_;
    Eigen::Matrix3d ric_;
    double anchor_timestamp_=-1;
    uint64_t epoch_=0;
};
}
#endif
