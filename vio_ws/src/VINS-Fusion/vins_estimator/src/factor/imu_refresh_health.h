// Local native-VINS health patch, GPL-3.0 like the surrounding estimator.
#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <algorithm>

struct NativeImuPrediction
{
    double stamp = 0;
    Eigen::Vector3d p = Eigen::Vector3d::Zero(), v = Eigen::Vector3d::Zero();
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
    Eigen::Vector3d ba = Eigen::Vector3d::Zero(), bg = Eigen::Vector3d::Zero();
    Eigen::Vector3d acc = Eigen::Vector3d::Zero(), gyro = Eigen::Vector3d::Zero();
};

// Same midpoint formula and small-angle deltaQ as upstream fastPredictIMU.
// This runs on a candidate copy, never overwriting the published trajectory.
inline bool predictNativeImu(NativeImuPrediction &state, double stamp,
        const Eigen::Vector3d &acc, const Eigen::Vector3d &gyro, const Eigen::Vector3d &gravity)
{
    if (!std::isfinite(stamp) || stamp < state.stamp || !acc.allFinite() || !gyro.allFinite()) return false;
    const double dt = stamp - state.stamp;
    const Eigen::Vector3d previous_acc = state.q * (state.acc - state.ba) - gravity;
    const Eigen::Vector3d delta_angle = (0.5 * (state.gyro + gyro) - state.bg) * dt;
    state.q = state.q * Eigen::Quaterniond(1, delta_angle.x() * .5, delta_angle.y() * .5, delta_angle.z() * .5);
    const Eigen::Vector3d current_acc = state.q * (acc - state.ba) - gravity;
    const Eigen::Vector3d mean_acc = 0.5 * (previous_acc + current_acc);
    state.p += dt * state.v + 0.5 * dt * dt * mean_acc;
    state.v += dt * mean_acc; state.stamp = stamp; state.acc = acc; state.gyro = gyro;
    return state.p.allFinite() && state.v.allFinite() && state.q.coeffs().allFinite();
}

// The earlier published IMU callback may be waiting for mPropagate. Its
// pending actual sample must be close enough to represent real source replay,
// rather than integrating an unobserved long gap as one midpoint step.
inline bool advancePendingPublishedImu(NativeImuPrediction &state, double stamp,
        const Eigen::Vector3d &acc, const Eigen::Vector3d &gyro, const Eigen::Vector3d &gravity)
{
    if (stamp - state.stamp > .008 + 1e-6) return false; // nominal 250 Hz, at most two samples
    return predictNativeImu(state, stamp, acc, gyro, gravity);
}

struct NativeRefreshEvidence
{
    double comparison_stamp_sim_s = 0;
    double previous_published_stamp_sim_s = 0;
    double candidate_base_stamp_sim_s = 0;
    double pending_old_forward_dt_s = 0;
    double imu_position_delta_m = 1e9;
    double body_position_delta_m = 1e9;
    double orientation_delta_rad = 1e9;
};

inline bool nativeRefreshConsistent(const NativeImuPrediction &published,
        const NativeImuPrediction &candidate, const Eigen::Vector3d &imu_to_body_flu,
        NativeRefreshEvidence *evidence)
{
    if (!std::isfinite(published.stamp) || !std::isfinite(candidate.stamp)
        || std::abs(published.stamp - candidate.stamp) > 1e-6
        || !published.p.allFinite() || !candidate.p.allFinite()
        || !published.q.coeffs().allFinite() || !candidate.q.coeffs().allFinite()
        || published.q.norm() < 1e-12 || candidate.q.norm() < 1e-12
        || !imu_to_body_flu.allFinite()) return false;
    evidence->comparison_stamp_sim_s = candidate.stamp;
    const auto q0 = published.q.normalized(), q1 = candidate.q.normalized();
    evidence->imu_position_delta_m = (candidate.p - published.p).norm();
    evidence->body_position_delta_m = (candidate.p + q1 * imu_to_body_flu - published.p - q0 * imu_to_body_flu).norm();
    evidence->orientation_delta_rad = 2 * std::acos(std::min(1.0, std::abs(q0.dot(q1))));
    // Same physical time: the existing jump floor applies, with only 1 um of
    // numerical tolerance. Check actual configured body lever arm as well as
    // the native IMU reference point; no pose clipping/freezing/reanchoring.
    return evidence->imu_position_delta_m <= .35 + 1e-6
        && evidence->body_position_delta_m <= .35 + 1e-6
        && evidence->orientation_delta_rad <= .25 + 1e-6;
}
