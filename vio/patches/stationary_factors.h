// Local integration patch, GPL-3.0, like the surrounding VINS-Fusion code.
#pragma once
#include <ceres/ceres.h>
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <deque>
#include "temporal_stereo_window.h"
#include "imu_refresh_health.h"

// Raw IMU and tracked stereo image evidence only. No navigation, vehicle
// landed/armed flag or simulator truth is accepted by this detector.
struct StationaryConfig
{
    double imu_window_s = 1.0;
    double continuous_support_s = 1.0;
    int minimum_imu_samples = 100;
    int minimum_near_stereo_tracks = 20;
    double maximum_depth_m = 8.0;
    double maximum_flow_p90_px_s = 0.5;
    double maximum_flow_median_px_s = 0.25;
    double maximum_instant_flow_p90_px_s = 2.0;
    double maximum_instant_flow_median_px_s = 0.75;
    double maximum_fit_jitter_p90_px = 0.06; // actual ground_tracks_01 maximum: .03050 px
    double maximum_gyro_mean_rad_s = 0.01;
    double maximum_gyro_std_rad_s = 0.008;
    double maximum_acc_std_m_s2 = 0.07; // original IMU sigma: ~0.029/axis
    double maximum_gravity_norm_error_m_s2 = 0.3;
};

struct StationaryEvidence
{
    int imu_samples = 0;
    double imu_span_s = 0;
    double imu_age_s = 1e9;
    double gyro_mean_norm = 1e9;
    double gyro_std_norm = 1e9;
    double acc_std_norm = 1e9;
    double gravity_norm_error = 1e9;
    double imu_first_source_stamp_s = -1, imu_last_source_stamp_s = -1;
    Eigen::Vector3d acc_mean = Eigen::Vector3d::Zero(), gyro_mean = Eigen::Vector3d::Zero();
    Eigen::Vector3d acc_std = Eigen::Vector3d::Zero(), gyro_std = Eigen::Vector3d::Zero();
    int near_stereo_tracks = 0;
    double flow_p90_px_s = 1e9;
    double flow_median_px_s = 1e9;
    int window_supported_tracks = 0;
    double window_span_s = 0;
    double instant_flow_p90_px_s = 1e9;
    double instant_flow_median_px_s = 1e9;
    double fit_jitter_p90_px = 1e9;
};

struct StationaryImuSample
{
    double t;
    Eigen::Vector3d acc, gyro;
};

class SensorStationaryDetector
{
public:
    StationaryConfig config;
    std::deque<StationaryImuSample> imu;
    double supported_since = -1;
    double previous_image_t = -1;
    bool instant_gate = false;
    bool active = false;

    void clear()
    {
        imu.clear(); supported_since = -1; previous_image_t = -1; instant_gate = false; active = false;
    }
    void addIMU(double t, const Eigen::Vector3d &acc, const Eigen::Vector3d &gyro)
    {
        if (!std::isfinite(t) || !acc.allFinite() || !gyro.allFinite())
        { clear(); return; }
        if (!imu.empty() && t <= imu.back().t) return;
        imu.push_back({t, acc, gyro});
        while (!imu.empty() && t - imu.front().t > config.imu_window_s + 0.01) imu.pop_front();
    }
    StationaryEvidence imuEvidence(double image_t, double gravity_norm) const
    {
        StationaryEvidence e;
        Eigen::Vector3d acc_sum = Eigen::Vector3d::Zero(), gyro_sum = Eigen::Vector3d::Zero();
        double first = -1, last = -1;
        for (const auto &s : imu)
        {
            if (s.t < image_t - config.imu_window_s || s.t > image_t + 0.005) continue;
            if (first < 0) first = s.t;
            last = s.t; ++e.imu_samples; acc_sum += s.acc; gyro_sum += s.gyro;
        }
        if (!e.imu_samples) return e;
        const Eigen::Vector3d acc_mean = acc_sum / e.imu_samples, gyro_mean = gyro_sum / e.imu_samples;
        double acc_squared = 0, gyro_squared = 0;
        Eigen::Vector3d acc_axis_squared = Eigen::Vector3d::Zero(), gyro_axis_squared = Eigen::Vector3d::Zero();
        for (const auto &s : imu)
        {
            if (s.t < image_t - config.imu_window_s || s.t > image_t + 0.005) continue;
            acc_squared += (s.acc - acc_mean).squaredNorm();
            gyro_squared += (s.gyro - gyro_mean).squaredNorm();
            acc_axis_squared += (s.acc - acc_mean).cwiseProduct(s.acc - acc_mean);
            gyro_axis_squared += (s.gyro - gyro_mean).cwiseProduct(s.gyro - gyro_mean);
        }
        e.imu_span_s = last - first; e.imu_age_s = image_t - last;
        e.imu_first_source_stamp_s = first; e.imu_last_source_stamp_s = last;
        e.acc_mean = acc_mean; e.gyro_mean = gyro_mean;
        e.acc_std = (acc_axis_squared / e.imu_samples).cwiseSqrt();
        e.gyro_std = (gyro_axis_squared / e.imu_samples).cwiseSqrt();
        e.gyro_mean_norm = gyro_mean.norm();
        e.gyro_std_norm = std::sqrt(gyro_squared / e.imu_samples);
        e.acc_std_norm = std::sqrt(acc_squared / e.imu_samples);
        e.gravity_norm_error = std::abs(acc_mean.norm() - gravity_norm);
        return e;
    }
    bool update(double image_t, const StationaryEvidence &e)
    {
        if (previous_image_t >= 0 && (image_t <= previous_image_t || image_t - previous_image_t > 0.2))
            supported_since = -1;
        previous_image_t = image_t;
        instant_gate = std::isfinite(image_t) && e.imu_samples >= config.minimum_imu_samples
            && e.imu_span_s >= config.imu_window_s - 0.02 && e.imu_age_s >= -0.005 && e.imu_age_s <= 0.02
            && e.gyro_mean_norm <= config.maximum_gyro_mean_rad_s
            && e.gyro_std_norm <= config.maximum_gyro_std_rad_s
            && e.acc_std_norm <= config.maximum_acc_std_m_s2
            && e.gravity_norm_error <= config.maximum_gravity_norm_error_m_s2
            && e.near_stereo_tracks >= config.minimum_near_stereo_tracks
            && e.flow_p90_px_s <= config.maximum_flow_p90_px_s
            && e.flow_median_px_s <= config.maximum_flow_median_px_s
            && e.window_supported_tracks >= config.minimum_near_stereo_tracks
            && e.window_span_s >= 0.98
            && e.instant_flow_p90_px_s <= config.maximum_instant_flow_p90_px_s
            && e.instant_flow_median_px_s <= config.maximum_instant_flow_median_px_s
            && e.fit_jitter_p90_px <= config.maximum_fit_jitter_p90_px;
        if (!instant_gate) { supported_since = -1; active = false; return false; }
        if (supported_since < 0 || image_t < supported_since) supported_since = image_t;
        active = image_t - supported_since >= config.continuous_support_s;
        return active;
    }
};

// Calibrated, positive disparity and real temporal tracks in both eyes exclude
// far-scene/constant-velocity ambiguity and zero flow on newly created tracks.
inline bool nearStereoStationarySupport(const Eigen::Matrix<double, 7, 1> &left,
        const Eigen::Matrix<double, 7, 1> &right, bool previously_tracked_both_eyes,
        bool rectified, double baseline_x_m, double fx, double maximum_depth_m,
        double *flow_px_s)
{
    if (!previously_tracked_both_eyes || !rectified || !left.allFinite() || !right.allFinite()
        || baseline_x_m <= 0 || fx <= 0) return false;
    const double disparity = left[3] - right[3];
    if (disparity <= 0 || std::abs(left[4] - right[4]) > 1.5) return false;
    const double depth = fx * baseline_x_m / disparity;
    if (!std::isfinite(depth) || depth <= 0 || depth > maximum_depth_m) return false;
    *flow_px_s = fx * std::max(left.tail<2>().norm(), right.tail<2>().norm());
    return std::isfinite(*flow_px_s);
}

class ZeroVelocityFactor : public ceres::SizedCostFunction<3, 9>
{
public:
    explicit ZeroVelocityFactor(double sigma) : weight_(1.0 / sigma) {}
    bool Evaluate(double const* const* parameters, double *residuals, double **jacobians) const override
    {
        for (int i = 0; i < 3; ++i)
        {
            residuals[i] = weight_ * parameters[0][i];
            if (jacobians && jacobians[0])
                for (int j = 0; j < 9; ++j) jacobians[0][i * 9 + j] = i == j ? weight_ : 0;
        }
        return true;
    }
private:
    double weight_;
};

// Soft relative-position information for two independently confirmed stationary
// frames. Neither frame receives an absolute world position prior.
class StationaryRelativePositionFactor : public ceres::SizedCostFunction<3, 7, 7>
{
public:
    explicit StationaryRelativePositionFactor(double sigma) : weight_(1.0 / sigma) {}
    bool Evaluate(double const* const* parameters, double *residuals, double **jacobians) const override
    {
        for (int i = 0; i < 3; ++i)
        {
            residuals[i] = (parameters[1][i] - parameters[0][i]) * weight_;
            if (jacobians)
                for (int block = 0; block < 2; ++block)
                    if (jacobians[block]) for (int j = 0; j < 7; ++j)
                        jacobians[block][i * 7 + j] = i == j ? (block ? weight_ : -weight_) : 0;
        }
        return true;
    }
private:
    double weight_;
};

struct StationaryFactorCounts { int zero_velocity = 0; int relative_position = 0; };

// The interval array is separate from endpoint flags. Dropping a moving or
// unconfirmed frame must not turn stationary endpoints into a stationary path.
inline void slideStationaryEvidenceSecondNew(bool *frames, bool *intervals, int last)
{
    const bool merged_interval = intervals[last - 1] && intervals[last]
        && frames[last - 2] && frames[last - 1] && frames[last];
    frames[last - 1] = frames[last];
    intervals[last - 1] = merged_interval;
    frames[last] = false; intervals[last] = false;
}

inline void slideStationaryEvidenceOld(bool *frames, bool *intervals, int last)
{
    for (int i = 0; i < last; ++i)
    { frames[i] = frames[i + 1]; intervals[i] = intervals[i + 1]; }
    intervals[0] = false;
    frames[last] = false; intervals[last] = false;
}

inline StationaryFactorCounts addStationaryFactors(ceres::Problem &problem,
        const bool *stationary_frames, const bool *stationary_intervals,
        double (*pose)[7], double (*speed_bias)[9],
        int frame_count, double velocity_sigma, double position_sigma)
{
    StationaryFactorCounts counts;
    for (int i = 0; i <= frame_count; ++i)
    {
        if (stationary_frames[i])
        {
            problem.AddResidualBlock(new ZeroVelocityFactor(velocity_sigma), NULL, speed_bias[i]);
            ++counts.zero_velocity;
        }
        if (i > 0 && stationary_frames[i - 1] && stationary_frames[i] && stationary_intervals[i])
        {
            problem.AddResidualBlock(new StationaryRelativePositionFactor(position_sigma), NULL, pose[i - 1], pose[i]);
            ++counts.relative_position;
        }
    }
    return counts;
}

inline bool usableOptimizedKinematics(double dt, const Eigen::Vector3d &delta_position,
        const Eigen::Vector3d &previous_velocity, const Eigen::Vector3d &velocity,
        double orientation_delta_rad)
{
    if (!std::isfinite(dt) || dt <= 0 || !delta_position.allFinite()
        || !previous_velocity.allFinite() || !velocity.allFinite() || !std::isfinite(orientation_delta_rad)) return false;
    // Preserve the independent adapter's original jump criteria.
    if (delta_position.norm() > std::max(0.35, 4.0 * dt)
        || orientation_delta_rad > std::max(0.25, 6.0 * dt)) return false;
    const Eigen::Vector3d integrated = (previous_velocity + velocity) * (0.5 * dt);
    return (delta_position - integrated).norm() <= 0.35 + 3.0 * dt * dt;
}
