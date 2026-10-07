#include "vio_engine.h"
#include "common/common_types.h"
#include "backend/factor/projection_factor.h"
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <iomanip>

namespace {
bool isRotation(const Eigen::Matrix3d& rotation) {
    return rotation.allFinite() &&
        (rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).norm() <= 1e-6 &&
        std::abs(rotation.determinant() - 1.0) <= 1e-6;
}
Eigen::Vector3d acceleration(const IMUReading& value) {
    return {value.acc_x,value.acc_y,value.acc_z};
}
Eigen::Vector3d angularVelocity(const IMUReading& value) {
    return {value.gyro_x,value.gyro_y,value.gyro_z};
}
void jsonNumber(std::ostream& output,double value) {
    if(std::isfinite(value)) output<<std::setprecision(17)<<value;
    else output<<"null";
}
}

VIOEngine::VIOEngine()
    : configured_(false), current_time_(-1.0), prev_image_timestamp_(-1.0),
      prev_acc_(Eigen::Vector3d::Zero()), prev_gyro_(Eigen::Vector3d::Zero()),
      img_cnt_(0), pnp_initialized_(false), latest_position_(Eigen::Vector3d::Zero()),
      latest_rotation_(Eigen::Matrix3d::Identity()), has_valid_pose_(false), init_start_time_(-1.0) {}
VIOEngine::~VIOEngine() {
    // The explicit profile owns the process-global OpenCV scheduler for this synchronous engine.
    // OpenCV 4.5.4/TBB retains allocations unless its scheduler is released with thread count zero.
    if(execution_configured_) cv::setNumThreads(0);
}

bool VIOEngine::configure(int width, int height,
                          double fx, double fy, double cx, double cy, int model_type,
                          double k2, double k3, double k4, double k5,
                          const double* r_ic, const double* t_ic,
                          double acc_n, double acc_w, double gyr_n, double gyr_w, double g_norm) {
    // Copy every input before writing g_config: callers may point into its Eigen storage.
    if (!r_ic || !t_ic) { invalidatePose("invalid_calibration_pointer"); return false; }
    const Eigen::Matrix3d rotation = Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor>>(r_ic);
    const Eigen::Vector3d translation = Eigen::Map<const Eigen::Vector3d>(t_ic);
    const double values[] = {fx,fy,cx,cy,k2,k3,k4,k5,acc_n,acc_w,gyr_n,gyr_w,g_norm};
    const bool finite = std::all_of(std::begin(values),std::end(values),[](double x){ return std::isfinite(x); });
    if (width < 8 || height < 8 || width > 8192 || height > 8192 ||
        static_cast<int64_t>(width)*height > 16777216 || !finite || fx <= 0 || fy <= 0 ||
        !isRotation(rotation) || !translation.allFinite() ||
        acc_n <= 0 || acc_w <= 0 || gyr_n <= 0 || gyr_w <= 0 || g_norm <= 0 ||
        (model_type != common::camera_models::Camera::KANNALA_BRANDT &&
         model_type != common::camera_models::Camera::PINHOLE)) {
        invalidatePose("invalid_calibration"); return false;
    }
    auto cfg = utility::g_config;
    cfg.camera.col = width; cfg.camera.row = height;
    cfg.camera.fx = fx; cfg.camera.fy = fy; cfg.camera.cx = cx; cfg.camera.cy = cy;
    cfg.camera.focal_length = (fx+fy)/2;
    cfg.camera.r_ic = rotation; cfg.camera.t_ic = translation;
    cfg.estimator.acc_n = acc_n; cfg.estimator.acc_w = acc_w;
    cfg.estimator.gyr_n = gyr_n; cfg.estimator.gyr_w = gyr_w;
    cfg.estimator.g = Eigen::Vector3d(0,0,g_norm);
    cfg.feature_tracker.show_track = 0;
    cfg.feature_tracker.max_cnt = 120; cfg.feature_tracker.min_dist = 20;
    cfg.feature_tracker.equalize = 1;
    cfg.feature_tracker.fisheye = model_type == common::camera_models::Camera::KANNALA_BRANDT;
    cfg.feature_tracker.f_threshold = 1.0; cfg.feature_tracker.window_size = 20;
    cfg.estimator.solver_time = .06; cfg.estimator.num_iterations = 10;
    cfg.estimator.min_parallax = 10; cfg.estimator.init_depth = 2;
    auto tracker = std::make_unique<frontend::FeatureTracker>();
    if (!tracker->setIntrinsicParameter(model_type,width,height,fx,fy,cx,cy,k2,k3,k4,k5)) {
        invalidatePose("unsupported_camera_model"); return false;
    }
    utility::g_config = cfg;
    configured_width_ = width; configured_height_ = height;
    feature_tracker_ = std::move(tracker);
    feature_tracker_->setDiagnosticCapture(diagnostic_capture_);
    configured_ = true;
    resetState("configured");
    return true;
}

#ifndef __EMSCRIPTEN__
bool VIOEngine::configureFromConfig(const utility::Config& config, const std::string& camera_file) {
    const utility::Config owned = config;
    if (!std::isfinite(owned.camera.col) || !std::isfinite(owned.camera.row) ||
        std::floor(owned.camera.col) != owned.camera.col || std::floor(owned.camera.row) != owned.camera.row ||
        owned.camera.col < 8 || owned.camera.row < 8 || owned.camera.col > 8192 || owned.camera.row > 8192) {
        invalidatePose("invalid_calibration"); return false;
    }
    const auto camera = common::camera_models::CameraFactory::instance()->generateCameraFromYamlFile(camera_file);
    if (!camera) { invalidatePose("invalid_camera_file"); return false; }
    std::vector<double> parameters;
    camera->writeParameters(parameters);
    if (parameters.size() != 8 || camera->imageWidth()!=owned.camera.col || camera->imageHeight()!=owned.camera.row) {
        invalidatePose("unsupported_camera_model"); return false;
    }
    Eigen::Matrix<double,3,3,Eigen::RowMajor> rotation = owned.camera.r_ic;
    if (!configure(static_cast<int>(owned.camera.col),static_cast<int>(owned.camera.row),
                   parameters[4],parameters[5],parameters[6],parameters[7],camera->modelType(),
                   parameters[0],parameters[1],parameters[2],parameters[3],rotation.data(),owned.camera.t_ic.data(),
                   owned.estimator.acc_n,owned.estimator.acc_w,owned.estimator.gyr_n,owned.estimator.gyr_w,
                   owned.estimator.g.norm())) return false;
    // Keep native YAML tuning; camera factory and direct configure now use identical model/layout.
    utility::g_config = owned;
    feature_tracker_->m_camera = camera;
    estimator_->setParameter();
    return true;
}
#endif

void VIOEngine::invalidatePose(const std::string& reason) {
    has_valid_pose_ = false; pose_timestamp_ = -1.0;
    latest_position_.setZero(); latest_rotation_.setIdentity();
    last_reason_ = reason;
}

void VIOEngine::resetState(const std::string& reason) {
    ++epoch_;
    if(execution_configured_) {
        cv::setRNGSeed(execution_seed_);
        cv::setNumThreads(execution_threads_);
    }
    estimator_ = configured_ ? std::make_unique<backend::Estimator>() : nullptr;
    if (estimator_) {
        estimator_->setParameter();
        estimator_->setDiagnosticCapture(diagnostic_capture_);
        estimator_->setBenchmarkSolverProfile(benchmark_solver_profile_);
        estimator_generation_ = estimator_->getResetGeneration();
    }
    if (feature_tracker_) feature_tracker_->reset();
    current_time_ = prev_image_timestamp_ = last_imu_timestamp_ = -1.0;
    frame_timestamp_ = -1.0;
    prev_acc_.setZero(); prev_gyro_.setZero(); pending_imu_.clear();
    imu_samples_since_image_ = 0;
    pnp_frontend_.reset(); pnp_initialized_ = false; img_cnt_ = 0;
    cached_solved_features_.clear(); solved_feature_map_.clear();
    last_solver_iterations_ = 0; last_solver_termination_ = "not_run";
    last_solver_quality_reason_.clear();
    init_start_time_ = -1.0;
    invalidatePose(reason);
}

bool VIOEngine::feedIMU(double timestamp, const Eigen::Vector3d& acc, const Eigen::Vector3d& gyro) {
    const double dt = current_time_ < 0 ? 0.0 : timestamp-current_time_;
    if (dt > 0 && imu_samples_since_image_ >= kMaxIMUReadings) {
        resetState("imu_history_overflow"); return false;
    }
    estimator_->processIMU(dt,acc,gyro);
    if (pnp_initialized_ && pnp_frontend_) pnp_frontend_->processIMU(dt,acc,gyro);
    current_time_ = timestamp; prev_acc_ = acc; prev_gyro_ = gyro;
    if (dt > 0) ++imu_samples_since_image_;
    return true;
}

bool VIOEngine::processIMUData(const IMUReading* readings,int count,double image_timestamp) {
    double previous = last_imu_timestamp_;
    for (int i=0;i<count;++i) {
        const auto& value=readings[i];
        if (!std::isfinite(value.timestamp) || value.timestamp < 0 ||
            !acceleration(value).allFinite() || !angularVelocity(value).allFinite()) {
            last_reason_="invalid_imu"; return false;
        }
        if (value.timestamp <= previous || (current_time_ >= 0 && value.timestamp <= current_time_)) {
            last_reason_="imu_out_of_order"; return false;
        }
        previous=value.timestamp;
    }
    if (pending_imu_.size()+static_cast<size_t>(count)>kMaxIMUReadings) {
        resetState("imu_overflow"); return false;
    }
    for(int i=0;i<count;++i) pending_imu_.push_back(readings[i]);
    if(count>0) last_imu_timestamp_=previous;
    if(current_time_<0) {
        if(pending_imu_.empty() || pending_imu_.front().timestamp>image_timestamp) {
            last_reason_="imu_missing_past"; return false;
        }
        const auto seed=pending_imu_.front(); pending_imu_.pop_front();
        if(!feedIMU(seed.timestamp,acceleration(seed),angularVelocity(seed))) return false;
    }
    while(!pending_imu_.empty() && pending_imu_.front().timestamp<=image_timestamp) {
        const auto value=pending_imu_.front();
        if(value.timestamp-current_time_>kMaxSensorGapSeconds) {
            resetState("imu_gap"); return false;
        }
        pending_imu_.pop_front();
        if(!feedIMU(value.timestamp,acceleration(value),angularVelocity(value))) return false;
    }
    if(current_time_==image_timestamp) return true;
    if(pending_imu_.empty()) { last_reason_="imu_missing_future"; return false; }
    const auto& right=pending_imu_.front();
    const double span=right.timestamp-current_time_;
    if(span>kMaxSensorGapSeconds) { resetState("imu_gap"); return false; }
    const double weight=(image_timestamp-current_time_)/span;
    // Preserve the interpolated endpoint and retain the original right bracket once.
    const Eigen::Vector3d acc=(1-weight)*prev_acc_+weight*acceleration(right);
    const Eigen::Vector3d gyro=(1-weight)*prev_gyro_+weight*angularVelocity(right);
    return feedIMU(image_timestamp,acc,gyro);
}

void VIOEngine::writePoseOutput(double* output) const {
    if(!output) return;
    for(int r=0;r<3;++r) {
        for(int c=0;c<3;++c) output[4*r+c]=latest_rotation_(r,c);
        output[4*r+3]=latest_position_(r);
    }
    output[12]=output[13]=output[14]=0; output[15]=1;
}

bool VIOEngine::publishPose(const Eigen::Vector3d& position,const Eigen::Matrix3d& rotation,
                            double timestamp,double* output) {
    if(!position.allFinite() || position.norm()>1e6 || !isRotation(rotation)) {
        resetState("invalid_pose"); frame_timestamp_=timestamp; return false;
    }
    latest_position_=position+rotation*estimator_->t_ic_;
    latest_rotation_=rotation*estimator_->r_ic_;
    has_valid_pose_=true; pose_timestamp_=timestamp; last_reason_="tracking";
    if(!last_solver_quality_reason_.empty()) last_reason_ += ":"+last_solver_quality_reason_;
    writePoseOutput(output); return true;
}

std::vector<common::SolvedFeature> VIOEngine::matchFeaturesForPnP() const {
    std::vector<common::SolvedFeature> matched;
    if(!feature_tracker_) return matched;
    for(size_t j=0;j<feature_tracker_->ids.size();++j) {
        if(feature_tracker_->track_cnt[j]<=1) continue;
        const auto it=solved_feature_map_.find(feature_tracker_->ids[j]);
        if(it==solved_feature_map_.end()) continue;
        auto value=cached_solved_features_[it->second];
        const auto& point=feature_tracker_->cur_undistorted_pts[j];
        value.observation=Eigen::Vector2d(point.x,point.y);
        if(value.observation.allFinite()) matched.push_back(value);
    }
    return matched;
}

bool VIOEngine::processFrame(const uint8_t* gray_image,int width,int height,
                             const IMUReading* imu_readings,int imu_count,
                             double image_timestamp,double* output) {
    if(output) std::fill(output,output+16,std::numeric_limits<double>::quiet_NaN());
    invalidatePose("not_configured");
    last_solver_iterations_ = 0; last_solver_termination_ = "not_run";
    last_solver_quality_reason_.clear();
    frame_timestamp_=std::isfinite(image_timestamp) ? image_timestamp : -1.0;
    if(!configured_ || !estimator_ || !feature_tracker_) return false;
    if(!gray_image || !output || width!=configured_width_ || height!=configured_height_ ||
       !std::isfinite(image_timestamp) || image_timestamp<0 || imu_count<0 || imu_count>kMaxIMUReadings ||
       (imu_count>0 && !imu_readings)) { last_reason_="invalid_frame"; return false; }
    if(estimator_->getResetGeneration()!=estimator_generation_) {
        resetState("estimator_reset"); frame_timestamp_=image_timestamp; return false;
    }
    if(prev_image_timestamp_>=0 && image_timestamp<=prev_image_timestamp_) {
        last_reason_="frame_out_of_order"; return false;
    }
    if(prev_image_timestamp_>=0 && image_timestamp-prev_image_timestamp_>kMaxSensorGapSeconds) {
        resetState("frame_gap"); frame_timestamp_=image_timestamp; return false;
    }
    const bool imu_covered=processIMUData(imu_readings,imu_count,image_timestamp);
    frame_timestamp_=image_timestamp;
    if(!imu_covered && last_reason_!="imu_missing_past" && last_reason_!="imu_missing_future") return false;
    const std::string imu_reason=last_reason_;
    if(!isInitialized()) {
        if(init_start_time_<0) init_start_time_=image_timestamp;
        else if(image_timestamp-init_start_time_>kInitTimeoutSeconds) {
            resetState("initialization_timeout"); frame_timestamp_=image_timestamp; return false;
        }
    } else init_start_time_=-1.0;
    const bool pnp_active=pnp_initialized_ && utility::g_config.pnp.enable_pnp;
    const bool backend_frame=!pnp_active || img_cnt_==0;
    try {
        const cv::Mat image(height,width,CV_8UC1,const_cast<uint8_t*>(gray_image));
        feature_tracker_->detectAndTrack(image,image_timestamp,backend_frame);
        prev_image_timestamp_=image_timestamp;
        if(!imu_covered) { last_reason_=imu_reason; return false; }
        if(pnp_active && !backend_frame) {
            const auto matched=matchFeaturesForPnP();
            img_cnt_=(img_cnt_+1)%utility::g_config.pnp.freq;
            const bool usable=pnp_frontend_->processImage(matched,image_timestamp);
            recordSolver(pnp_frontend_->getLastSolverDiagnostics());
            if(!usable || !pnp_frontend_->hasUsableLatestImageUpdate()) {
                last_reason_=pnp_frontend_->getLastSolverDiagnostics().reason; return false;
            }
            return publishPose(pnp_frontend_->getPosition(),pnp_frontend_->getRotation(),image_timestamp,output);
        }
        for(unsigned int i=0;feature_tracker_->updateID(i);++i) {}
        common::ImageData image_data;
        for(size_t j=0;j<feature_tracker_->ids.size();++j) {
            if(feature_tracker_->track_cnt[j]<=1) continue;
            const auto& ray=feature_tracker_->cur_undistorted_pts[j];
            const auto& pixel=feature_tracker_->cur_pts[j];
            const auto& velocity=feature_tracker_->pts_velocity[j];
            Eigen::Matrix<double,7,1> value;
            value << ray.x,ray.y,1.0,pixel.x,pixel.y,velocity.x,velocity.y;
            if(value.allFinite()) image_data[feature_tracker_->ids[j]]=value;
        }
        if(image_data.empty()) { last_reason_="empty_features"; return false; }
        estimator_->processImage(image_data,image_timestamp);
        imu_samples_since_image_=0;
        recordSolver(estimator_->getLastSolverDiagnostics());
        if(estimator_->getResetGeneration()!=estimator_generation_) {
            resetState("estimator_reset"); frame_timestamp_=image_timestamp; return false;
        }
        if(!isInitialized()) { last_reason_="initializing"; return false; }
        if(!estimator_->hasUsableLatestImageUpdate()) {
            last_reason_=estimator_->getLastSolverDiagnostics().reason; return false;
        }
        const auto position=estimator_->sliding_window_[utility::WINDOW_SIZE].P;
        const auto rotation=estimator_->sliding_window_[utility::WINDOW_SIZE].R;
        if(!position.allFinite() || position.norm()>1e6 || !isRotation(rotation)) {
            resetState("invalid_pose"); frame_timestamp_=image_timestamp; return false;
        }
        if(!pnp_initialized_ && utility::g_config.pnp.enable_pnp) {
            pnp_frontend_=std::make_unique<frontend::PnPFrontend>();
            pnp_frontend_->setExtrinsicParameters(estimator_->r_ic_,estimator_->t_ic_);
            pnp_frontend_->setIMUModel();
            pnp_frontend_->initializeState(estimator_->getLatestVINSResult());
            pnp_initialized_=true; img_cnt_=0;
            // PnP starts at this exact backend endpoint; the next positive dt uses it.
            pnp_frontend_->processIMU(0.0,prev_acc_,prev_gyro_);
        }
        if(pnp_initialized_ && pnp_frontend_) {
            cached_solved_features_=estimator_->getSolvedFeatures();
            solved_feature_map_.clear();
            for(size_t i=0;i<cached_solved_features_.size();++i) solved_feature_map_[cached_solved_features_[i].id]=i;
            pnp_frontend_->processImage(matchFeaturesForPnP(),image_timestamp);
            pnp_frontend_->setBackendSolution(estimator_->getLatestVINSResult());
            img_cnt_=(img_cnt_+1)%utility::g_config.pnp.freq;
        }
        return publishPose(position,rotation,image_timestamp,output);
    } catch(const std::exception&) {
        resetState("processing_exception"); frame_timestamp_=image_timestamp; return false;
    }
}

bool VIOEngine::isInitialized() const {
    return estimator_ && estimator_->solver_flag_==common::SolverFlag::NON_LINEAR;
}
int VIOEngine::getFeaturePointCount() const {
    if(!feature_tracker_) return 0;
    return std::count_if(feature_tracker_->track_cnt.begin(),feature_tracker_->track_cnt.end(),[](int count){return count>1;});
}
int VIOEngine::getMapPoints(double* output,int max_count) const {
    if(!estimator_ || !has_valid_pose_ || !output || max_count<=0) return 0;
    const auto points=estimator_->getSlidingWindowMapPoints();
    int count=0;
    for(const auto& point:points) {
        if(!point.allFinite()) continue;
        if(count==max_count) break;
        for(int j=0;j<3;++j) output[count*3+j]=point(j);
        ++count;
    }
    return count;
}
int VIOEngine::getStatusCode() const {
    if(!configured_ || !estimator_) return static_cast<int>(VIOStatus::NOT_CONFIGURED);
    if(isInitialized()) return static_cast<int>(has_valid_pose_ ? VIOStatus::TRACKING : VIOStatus::LOST);
    return static_cast<int>(VIOStatus::INITIALIZING);
}
int VIOEngine::getLastSolverIterations() const {
    return last_solver_iterations_;
}
std::string VIOEngine::getLastSolverTermination() const {
    return last_solver_termination_;
}
void VIOEngine::recordSolver(const backend::SolverDiagnostics& diagnostics) {
    last_solver_iterations_=diagnostics.iterations;
    last_solver_quality_reason_=diagnostics.qualityReason;
    switch(diagnostics.terminationType) {
        case 0: last_solver_termination_="CONVERGENCE"; break;
        case 1: last_solver_termination_="NO_CONVERGENCE"; break;
        case 2: last_solver_termination_="FAILURE"; break;
        case 3: last_solver_termination_="USER_SUCCESS"; break;
        case 4: last_solver_termination_="USER_FAILURE"; break;
        default: last_solver_termination_="not_run";
    }
}
void VIOEngine::setExecutionParams(int seed,int cv_threads) {
    if(seed<0 || cv_threads<1 || cv_threads>4) return;
    execution_seed_=seed; execution_threads_=cv_threads; execution_configured_=true;
    cv::setRNGSeed(seed);
    cv::setNumThreads(cv_threads);
}
int VIOEngine::getCVThreadCount() const { return cv::getNumThreads(); }
void VIOEngine::setDiagnosticCapture(bool enabled) {
    diagnostic_capture_=enabled;
    if(feature_tracker_) feature_tracker_->setDiagnosticCapture(enabled);
    if(estimator_) estimator_->setDiagnosticCapture(enabled);
}
void VIOEngine::setBenchmarkSolverProfile(bool enabled) {
    benchmark_solver_profile_ = enabled;
    if (estimator_) estimator_->setBenchmarkSolverProfile(enabled);
}
std::string VIOEngine::getFeatureDiagnostics() const {
    if(!diagnostic_capture_ || !feature_tracker_) return "{\"enabled\":false}";
    std::ostringstream output;
    output<<"{\"enabled\":true,\"epoch\":"<<epoch_<<",\"frameTimestamp\":";
    jsonNumber(output,frame_timestamp_);
    output<<",\"opencvVersion\":\""<<CV_VERSION<<"\",\"executionSeed\":"<<getExecutionSeed()
          <<",\"cvThreads\":"<<getCVThreadCount()<<",\"tracker\":";
    const auto& tracker=feature_tracker_->getFeatureDiagnostics();
    output<<(tracker.empty()?"null":tracker)<<",\"backend\":";
    if(estimator_) {
        const auto& backend=estimator_->getBackendDiagnostics();
        output<<(backend.empty()?"null":backend);
    } else output<<"null";
    output<<",\"params\":{";
    bool first=true;
    const auto property=[&](const char* name,double value) {
        if(!first)output<<',';first=false;
        output<<'"'<<name<<"\":";jsonNumber(output,value);
    };
    const auto& config=utility::g_config;
    property("camera_model",feature_tracker_->m_camera?int(feature_tracker_->m_camera->modelType()):-1);
    property("width",config.camera.col);property("height",config.camera.row);
    property("fx",config.camera.fx);property("fy",config.camera.fy);
    property("cx",config.camera.cx);property("cy",config.camera.cy);property("focal_length",config.camera.focal_length);
    const auto& ft=config.feature_tracker;
    property("max_cnt",ft.max_cnt);property("min_dist",ft.min_dist);property("window_size",ft.window_size);
    property("f_threshold",ft.f_threshold);property("show_track",ft.show_track);property("equalize",ft.equalize);
    property("fisheye",ft.fisheye);property("lk_window_size",ft.lk_window_size);property("lk_pyramid_levels",ft.lk_pyramid_levels);
    property("lk_iterations",ft.lk_iterations);property("lk_eps",ft.lk_eps);property("f_threshold_edge_factor",ft.f_threshold_edge_factor);
    const auto& estimator=config.estimator;
    property("window_size_backend",estimator.window_size);property("num_iterations",estimator.num_iterations);
    property("marginalization_assembly_threads",backend::factor::kMarginalizationAssemblyThreads);
    property("solver_time",estimator.solver_time);property("min_parallax",estimator.min_parallax);
    property("init_depth",estimator.init_depth);property("num_of_features",estimator.num_of_features);
    property("acc_n",estimator.acc_n);property("acc_w",estimator.acc_w);property("gyr_n",estimator.gyr_n);property("gyr_w",estimator.gyr_w);
    const auto& pnp=config.pnp;
    property("pnp_size",pnp.pnp_size);property("pnp_freq",pnp.freq);property("pnp_max_iterations",pnp.pnp_max_iterations);
    property("pnp_solver_time",pnp.pnp_solver_time);property("pnp_enable",pnp.enable_pnp?1:0);
    property("base_solver_time",pnp.base_solver_time);property("min_solver_time",pnp.min_solver_time);property("enable_adaptive",pnp.enable_adaptive?1:0);
    for(int row=0;row<3;++row) {
        const std::string t="t_ic_"+std::to_string(row),g="g_"+std::to_string(row);
        property(t.c_str(),config.camera.t_ic(row));property(g.c_str(),estimator.g(row));
        for(int col=0;col<3;++col) {const std::string key="r_ic_"+std::to_string(row)+std::to_string(col);property(key.c_str(),config.camera.r_ic(row,col));}
    }
    for(int row=0;row<2;++row)for(int col=0;col<2;++col) {
        const std::string key="projection_sqrt_info_"+std::to_string(row)+std::to_string(col);
        property(key.c_str(),backend::factor::ProjectionFactor::sqrt_info(row,col));
    }
    if(feature_tracker_->m_camera) {
        std::vector<double> intrinsics;feature_tracker_->m_camera->writeParameters(intrinsics);
        for(size_t i=0;i<std::min(intrinsics.size(),size_t(16));++i) {
            const std::string key="camera_parameter_"+std::to_string(i);property(key.c_str(),intrinsics[i]);
        }
    }
    output<<"}}";
    return output.str();
}

void VIOEngine::setMobileParams(double solver_time, int num_iterations, int max_features) {
    if (!std::isfinite(solver_time) || solver_time <= 0 || solver_time > 10 ||
        num_iterations <= 0 || num_iterations > 100 || max_features < 8 || max_features > utility::NUM_OF_FEATURES) return;
    auto& cfg = utility::g_config;
    cfg.estimator.solver_time = solver_time;
    cfg.estimator.num_iterations = num_iterations;
    cfg.feature_tracker.max_cnt = max_features;
}

void VIOEngine::setFThreshold(double f_threshold) {
    if (!std::isfinite(f_threshold) || f_threshold <= 0) return;
    utility::g_config.feature_tracker.f_threshold = f_threshold;
    std::cout << "[VIOEngine] f_threshold set to " << f_threshold << std::endl;
}

void VIOEngine::setTrackingParams(int lk_window, int lk_pyramid, int min_dist, double f_edge_factor) {
    auto& cfg = utility::g_config.feature_tracker;
    if (lk_window > 0 && (lk_window % 2 == 1)) {
        cfg.lk_window_size = lk_window;
    }
    if (lk_pyramid > 0 && lk_pyramid <= 5) {
        cfg.lk_pyramid_levels = lk_pyramid;
    }
    if (min_dist > 0) {
        cfg.min_dist = min_dist;
    }
    if (std::isfinite(f_edge_factor) && f_edge_factor >= 0.0) {
        cfg.f_threshold_edge_factor = f_edge_factor;
    }
    // Mobile-optimized TermCriteria: faster convergence at low resolution (240x180).
    // Native TUM VI (512x512) uses defaults (30/0.01) set via config struct.
    cfg.lk_iterations = 20;
    cfg.lk_eps = 0.03;
    std::cout << "[VIOEngine] Tracking params: lk_window=" << cfg.lk_window_size
              << " lk_pyramid=" << cfg.lk_pyramid_levels
              << " min_dist=" << cfg.min_dist
              << " f_edge_factor=" << cfg.f_threshold_edge_factor
              << " lk_criteria=" << cfg.lk_iterations << "/" << cfg.lk_eps << std::endl;
}

void VIOEngine::setPnPParams(bool enable_pnp, int freq) {
    auto& cfg = utility::g_config.pnp;
    cfg.enable_pnp = enable_pnp;
    if (freq >= 1 && freq <= 10) {
        cfg.freq = freq;
    }
    // If disabling PnP, reset PnP state so all frames go through backend
    if (!enable_pnp) {
        pnp_initialized_ = false;
        pnp_frontend_.reset();
        img_cnt_ = 0;
        cached_solved_features_.clear();
        solved_feature_map_.clear();
    }
    std::cout << "[VIOEngine] PnP params: enable=" << enable_pnp
              << " freq=" << cfg.freq << std::endl;
}

void VIOEngine::reset() { resetState("external_reset"); }
