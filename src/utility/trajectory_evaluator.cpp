#include "utility/trajectory_evaluator.h"
#include "utility/logging.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <sstream>

namespace utility {
namespace {
bool validPose(TimestampedPose& pose) {
    if (!std::isfinite(pose.timestamp) || !pose.position.allFinite() ||
        !pose.orientation.coeffs().allFinite() || pose.orientation.norm() < 1e-12) return false;
    pose.orientation.normalize();
    return true;
}
}

bool TrajectoryEvaluator::loadVioTrajectory(const std::string& filepath) {
    vio_trajectory_.clear(); matched_vio_.clear(); matched_gt_.clear(); aligned_vio_.clear();
    std::ifstream input(filepath);
    if (!input) return false;
    std::string line;
    while (std::getline(input,line)) {
        if (line.empty() || line[0]=='#') continue;
        TimestampedPose pose; double qx,qy,qz,qw;
        std::istringstream row(line);
        if (!(row >> pose.timestamp >> pose.position.x() >> pose.position.y() >> pose.position.z() >> qx >> qy >> qz >> qw)) {vio_trajectory_.clear();return false;}
        pose.orientation=Eigen::Quaterniond(qw,qx,qy,qz);
        if (!validPose(pose) || (!vio_trajectory_.empty() && pose.timestamp<=vio_trajectory_.back().timestamp)) {vio_trajectory_.clear();return false;}
        vio_trajectory_.push_back(pose);
    }
    return !vio_trajectory_.empty();
}
bool TrajectoryEvaluator::loadGroundTruth(const std::string& filepath) {
    gt_trajectory_.clear(); matched_vio_.clear(); matched_gt_.clear(); aligned_vio_.clear();
    std::ifstream input(filepath);
    if (!input) return false;
    std::string line;
    while (std::getline(input,line)) {
        if (line.empty() || line[0]=='#') continue;
        std::replace(line.begin(),line.end(),',',' ');
        TimestampedPose pose; double qw,qx,qy,qz;
        std::istringstream row(line);
        if (!(row>>pose.timestamp>>pose.position.x()>>pose.position.y()>>pose.position.z()>>qw>>qx>>qy>>qz)) {gt_trajectory_.clear();return false;}
        pose.timestamp*=1e-9;
        pose.orientation=Eigen::Quaterniond(qw,qx,qy,qz);
        if (!validPose(pose) || (!gt_trajectory_.empty() && pose.timestamp<=gt_trajectory_.back().timestamp)) {gt_trajectory_.clear();return false;}
        gt_trajectory_.push_back(pose);
    }
    return !gt_trajectory_.empty();
}
void TrajectoryEvaluator::transformVioToBodyFrame(const Eigen::Matrix3d& r_ic,const Eigen::Vector3d& t_ic) {
    matched_vio_.clear();matched_gt_.clear();aligned_vio_.clear();
    if (!r_ic.allFinite() || !t_ic.allFinite() || (r_ic.transpose()*r_ic-Eigen::Matrix3d::Identity()).norm()>1e-6 || std::abs(r_ic.determinant()-1)>1e-6) {vio_trajectory_.clear();return;}
    for (auto& pose:vio_trajectory_) {
        const Eigen::Matrix3d rotation=pose.orientation.toRotationMatrix()*r_ic.transpose();
        pose.position-=rotation*t_ic;
        pose.orientation=Eigen::Quaterniond(rotation);
    }
}
int TrajectoryEvaluator::associateTrajectories(double max_dt) {
    matched_vio_.clear();matched_gt_.clear();aligned_vio_.clear();
    if (!std::isfinite(max_dt) || max_dt<0) return 0;
    size_t next=0;
    for (const auto& pose:vio_trajectory_) {
        auto first=gt_trajectory_.begin()+next;
        auto upper=std::lower_bound(first,gt_trajectory_.end(),pose.timestamp,[](const auto& p,double t){return p.timestamp<t;});
        auto best=gt_trajectory_.end();double error=max_dt;
        for (auto candidate:{upper,upper==first?gt_trajectory_.end():upper-1}) {
            if (candidate==gt_trajectory_.end()) continue;
            const double dt=std::abs(candidate->timestamp-pose.timestamp);
            if (dt<=error) {best=candidate;error=dt;}
        }
        if (best!=gt_trajectory_.end()) {
            matched_vio_.push_back(pose);matched_gt_.push_back(*best);
            next=static_cast<size_t>(best-gt_trajectory_.begin())+1;
        }
    }
    return getMatchedSize();
}
bool TrajectoryEvaluator::alignTrajectories(bool diagnostic_sim3) {
    aligned_vio_.clear();alignment_scale_=1;
    const int n=getMatchedSize();if(n<3)return false;
    Eigen::Matrix<double,3,Eigen::Dynamic> src(3,n),dst(3,n);
    for(int i=0;i<n;++i){src.col(i)=matched_vio_[i].position;dst.col(i)=matched_gt_[i].position;}
    if(diagnostic_sim3 && (src.colwise()-src.rowwise().mean()).squaredNorm()<1e-12)return false;
    const Eigen::Matrix4d transform=Eigen::umeyama(src,dst,diagnostic_sim3);
    if(!transform.allFinite())return false;
    alignment_scale_=std::cbrt(transform.block<3,3>(0,0).determinant());
    if(!(alignment_scale_>0))return false;
    for(const auto& pose:matched_vio_)aligned_vio_.push_back(transform.block<3,3>(0,0)*pose.position+transform.block<3,1>(0,3));
    return true;
}
AteResult TrajectoryEvaluator::computeATE() const {
    AteResult result;if(aligned_vio_.empty() || aligned_vio_.size()!=matched_gt_.size())return result;
    std::vector<double> errors;
    for(size_t i=0;i<aligned_vio_.size();++i)errors.push_back((aligned_vio_[i]-matched_gt_[i].position).norm());
    double sum=0,squared=0;for(double e:errors){if(!std::isfinite(e))return result;sum+=e;squared+=e*e;}
    result.num_pairs=errors.size();result.valid=true;result.alignment_scale=alignment_scale_;result.rmse=std::sqrt(squared/errors.size());result.mean=sum/errors.size();
    std::sort(errors.begin(),errors.end());const size_t n=errors.size();result.median=n%2?errors[n/2]:(errors[n/2-1]+errors[n/2])/2;result.min=errors.front();result.max=errors.back();
    double variance=0;for(double e:errors)variance+=(e-result.mean)*(e-result.mean);result.std_dev=std::sqrt(variance/n);return result;
}
RpeResult TrajectoryEvaluator::computeRPE(double delta) const {
    RpeResult result;if(!std::isfinite(delta)||delta<=0||matched_vio_.size()<2)return result;
    double trans_squared=0,rot_squared=0;
    const double tolerance=std::min(0.05,delta*0.1);
    for(size_t i=0;i+1<matched_vio_.size();++i){
        const double target=matched_vio_[i].timestamp+delta;
        auto first=matched_vio_.begin()+i+1;
        auto upper=std::lower_bound(first,matched_vio_.end(),target,[](const auto& p,double t){return p.timestamp<t;});
        auto best=matched_vio_.end();double diff=tolerance;
        for(auto candidate:{upper,upper==first?matched_vio_.end():upper-1}){if(candidate==matched_vio_.end())continue;const double d=std::abs(candidate->timestamp-target);if(d<=diff){best=candidate;diff=d;}}
        if(best==matched_vio_.end())continue;
        const size_t j=best-matched_vio_.begin();
        const auto& ei=matched_vio_[i];const auto& ej=matched_vio_[j];const auto& gi=matched_gt_[i];const auto& gj=matched_gt_[j];
        const Eigen::Quaterniond er=ei.orientation.conjugate()*ej.orientation;
        const Eigen::Quaterniond gr=gi.orientation.conjugate()*gj.orientation;
        const Eigen::Vector3d et=ei.orientation.conjugate()*(ej.position-ei.position);
        const Eigen::Vector3d gt=gi.orientation.conjugate()*(gj.position-gi.position);
        const Eigen::Vector3d error_t=gr.conjugate()*(et-gt);
        const Eigen::Quaterniond error_q=(gr.conjugate()*er).normalized();
        const double angle=2*std::atan2(error_q.vec().norm(),std::abs(error_q.w()));
        trans_squared+=error_t.squaredNorm();rot_squared+=angle*angle;++result.num_pairs;
    }
    if(result.num_pairs){result.valid=true;result.rmse_trans=std::sqrt(trans_squared/result.num_pairs);result.rmse_rot=std::sqrt(rot_squared/result.num_pairs);}return result;
}
void TrajectoryEvaluator::printResults(const AteResult& ate, const RpeResult& rpe) const {
    std::cerr << "\n========== Trajectory Evaluation Results ==========" << std::endl;
    std::cerr << "ATE (Absolute Trajectory Error):" << std::endl;
    std::cerr << "  RMSE:    " << std::fixed << std::setprecision(4) << ate.rmse << " m" << std::endl;
    std::cerr << "  Mean:    " << ate.mean << " m" << std::endl;
    std::cerr << "  Median:  " << ate.median << " m" << std::endl;
    std::cerr << "  Std Dev: " << ate.std_dev << " m" << std::endl;
    std::cerr << "  Min:     " << ate.min << " m" << std::endl;
    std::cerr << "  Max:     " << ate.max << " m" << std::endl;
    std::cerr << "  Pairs:   " << ate.num_pairs << std::endl;
    std::cerr << std::endl;
    std::cerr << "RPE (Relative Pose Error, delta=1.0s):" << std::endl;
    std::cerr << "  RMSE Trans: " << std::fixed << std::setprecision(4) << rpe.rmse_trans << " m" << std::endl;
    std::cerr << "  RMSE Rot:   " << rpe.rmse_rot << " rad" << std::endl;
    std::cerr << "  Pairs:      " << rpe.num_pairs << std::endl;
    std::cerr << "==================================================" << std::endl;
}

bool TrajectoryEvaluator::saveResults(const std::string& filepath, const AteResult& ate, const RpeResult& rpe) const {
    if (!ate.valid || !rpe.valid) return false;
    std::ofstream file(filepath);
    if (!file.is_open()) {
        LOG_ERROR("Cannot open evaluation output file: " << filepath);
        return false;
    }

    file << "Trajectory Evaluation Results" << std::endl;
    file << "=============================" << std::endl;
    file << std::endl;
    file << "ATE (Absolute Trajectory Error):" << std::endl;
    file << std::fixed << std::setprecision(6);
    file << "  RMSE:    " << ate.rmse << " m" << std::endl;
    file << "  Mean:    " << ate.mean << " m" << std::endl;
    file << "  Median:  " << ate.median << " m" << std::endl;
    file << "  Std Dev: " << ate.std_dev << " m" << std::endl;
    file << "  Min:     " << ate.min << " m" << std::endl;
    file << "  Max:     " << ate.max << " m" << std::endl;
    file << "  Pairs:   " << ate.num_pairs << std::endl;
    file << std::endl;
    file << "RPE (Relative Pose Error, delta=1.0s):" << std::endl;
    file << "  RMSE Trans: " << rpe.rmse_trans << " m" << std::endl;
    file << "  RMSE Rot:   " << rpe.rmse_rot << " rad" << std::endl;
    file << "  Pairs:      " << rpe.num_pairs << std::endl;

    file.close();
    LOG_INFO("Evaluation results saved to " << filepath);
    return true;
}

}  // namespace utility
