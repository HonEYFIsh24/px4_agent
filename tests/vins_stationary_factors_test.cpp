#include "stationary_factors.h"
#include <cassert>
#include <iostream>
#include <limits>

StationaryEvidence evidence()
{
    StationaryEvidence e;
    e.imu_samples = 251; e.imu_span_s = 1.0; e.imu_age_s = 0;
    e.gyro_mean_norm = .002; e.gyro_std_norm = .001;
    e.acc_std_norm = .052; e.gravity_norm_error = .1;
    e.near_stereo_tracks = 25; e.flow_p90_px_s = .3; e.flow_median_px_s = .1;
    e.window_supported_tracks = 25; e.window_span_s = 1.0; e.fit_jitter_p90_px = .02;
    e.instant_flow_p90_px_s = .8; e.instant_flow_median_px_s = .2;
    return e;
}

void test_detector()
{
    SensorStationaryDetector detector;
    for (int i = 0; i <= 250; ++i)
    {
        const double sign = i % 2 ? 1 : -1;
        detector.addIMU(i * .004, Eigen::Vector3d(.03 * sign, -.03 * sign, 9.91 + .03 * sign),
            Eigen::Vector3d(0, 0, .002 + sign * .001));
    }
    auto e = detector.imuEvidence(1.0, 9.81);
    assert(e.imu_samples == 251 && e.imu_span_s > .999 && std::abs(e.imu_age_s) < 1e-10);
    assert(e.imu_first_source_stamp_s == 0 && e.imu_last_source_stamp_s == 1);
    assert(std::abs(e.gyro_mean.z()-(.002-.001/251)) < 1e-12);
    assert(std::abs(e.acc_mean.z()-(9.91-.03/251)) < 1e-12);
    assert(std::abs(e.acc_std.norm()-e.acc_std_norm) < 1e-12);
    assert(e.acc_std_norm > .05 && e.acc_std_norm < .053);
    e.near_stereo_tracks = 25; e.flow_p90_px_s = .3; e.flow_median_px_s = .1;
    e.window_supported_tracks = 25; e.window_span_s = 1.0; e.fit_jitter_p90_px = .02;
    e.instant_flow_p90_px_s = .8; e.instant_flow_median_px_s = .2;
    assert(!detector.update(1.0, e));
    for (int i = 1; i <= 10; ++i) detector.update(1.0 + .1 * i, e);
    assert(detector.active);
    e.flow_p90_px_s = .501; assert(!detector.update(2.1, e));
    assert(!detector.active && detector.supported_since == -1);
    e = evidence(); detector.update(2.2, e); detector.update(3.3, e);
    assert(!detector.active); // a long image gap cannot supply continuous support
    for (int i = 1; i <= 10; ++i) detector.update(3.3 + .1 * i, e);
    assert(detector.active);
    e.near_stereo_tracks = 19; assert(!detector.update(4.4, e));
    e = evidence(); e.imu_span_s = .97; assert(!detector.update(4.5, e));
    e = evidence(); e.imu_age_s = .021; assert(!detector.update(4.6, e));
    e = evidence(); e.gyro_mean_norm = .011; assert(!detector.update(4.7, e));
    e = evidence(); e.acc_std_norm = .071; assert(!detector.update(4.8, e));
    e = evidence(); e.gravity_norm_error = .31; assert(!detector.update(4.9, e));
    e = evidence(); e.gyro_std_norm = std::numeric_limits<double>::quiet_NaN();
    assert(!detector.update(5.0, e));
    detector.addIMU(1.1, Eigen::Vector3d::Zero(), Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN()));
    assert(detector.imu.empty() && !detector.active);
}

void test_stereo()
{
    Eigen::Matrix<double, 7, 1> left, right;
    left << .1, .1, 1, 360, 250, .0005, 0;
    right << .1, .1, 1, 350, 250, .001, 0;
    double flow = -1;
    assert(nearStereoStationarySupport(left, right, true, true, .12, 320, 8, &flow));
    assert(std::abs(flow - .32) < 1e-10); // uses faster eye, not left eye alone
    assert(!nearStereoStationarySupport(left, right, false, true, .12, 320, 8, &flow));
    assert(!nearStereoStationarySupport(left, right, true, false, .12, 320, 8, &flow));
    right[3] = 357; // real 12.8 m depth: insufficient nearby support
    assert(!nearStereoStationarySupport(left, right, true, true, .12, 320, 8, &flow));
    right[3] = 360; assert(!nearStereoStationarySupport(left, right, true, true, .12, 320, 8, &flow));
    right[3] = 361; assert(!nearStereoStationarySupport(left, right, true, true, .12, 320, 8, &flow));
    right[3] = 350; right[4] = 252;
    assert(!nearStereoStationarySupport(left, right, true, true, .12, 320, 8, &flow));
}

void test_jacobians()
{
    ZeroVelocityFactor velocity(.03);
    double v[9] = {.06, -.03, .09, 0, 0, 0, 0, 0, 0}, residual[3], jac[27];
    const double *blocks[] = {v}; double *j[] = {jac};
    assert(velocity.Evaluate(blocks, residual, j));
    assert(std::abs(residual[0] - 2) < 1e-12 && std::abs(residual[1] + 1) < 1e-12);
    for (int row = 0; row < 3; ++row) for (int col = 0; col < 9; ++col)
        assert(std::abs(jac[row * 9 + col] - (row == col ? 1 / .03 : 0)) < 1e-12);
    StationaryRelativePositionFactor position(.02);
    double p0[7] = {1, 2, 3, 0, 0, 0, 1}, p1[7] = {1.04, 1.98, 3.06, 0, 0, 0, 1};
    const double *poses[] = {p0, p1}; double j0[21], j1[21]; double *jp[] = {j0, j1};
    assert(position.Evaluate(poses, residual, jp));
    assert(std::abs(residual[0] - 2) < 1e-10);
    for (int row = 0; row < 3; ++row) for (int col = 0; col < 7; ++col)
    {
        assert(std::abs(j0[row * 7 + col] - (row == col ? -50 : 0)) < 1e-12);
        assert(std::abs(j1[row * 7 + col] - (row == col ? 50 : 0)) < 1e-12);
    }
    // A common global translation cannot create an absolute-position prior.
    for (int k = 0; k < 3; ++k) { p0[k] += 100; p1[k] += 100; }
    assert(position.Evaluate(poses, residual, NULL)); assert(std::abs(residual[0] - 2) < 1e-9);
}

void test_real_solver_and_history()
{
    double p[3][7] = {{1, 2, 3, 0, 0, 0, 1}, {1.1, 2.1, 3.1, 0, 0, 0, 1}, {7, 8, 9, 0, 0, 0, 1}};
    double v[3][9] = {{.1, .2, .3, 0, 0, 0, 0, 0, 0}, {.1, .2, .3, 0, 0, 0, 0, 0, 0}, {1, 2, 3, 0, 0, 0, 0, 0, 0}};
    bool flags[3] = {true, true, false};
    bool intervals[3] = {false, true, false};
    ceres::Problem problem;
    auto count = addStationaryFactors(problem, flags, intervals, p, v, 2, .03, .02);
    assert(count.zero_velocity == 2 && count.relative_position == 1);
    assert(!problem.HasParameterBlock(v[2]) && !problem.HasParameterBlock(p[2]));
    problem.SetParameterBlockConstant(p[0]);
    problem.SetParameterization(p[1], new ceres::SubsetParameterization(7, {3, 4, 5, 6}));
    for (int i = 0; i < 2; ++i)
        problem.SetParameterization(v[i], new ceres::SubsetParameterization(9, {3, 4, 5, 6, 7, 8}));
    ceres::Solver::Options options; options.linear_solver_type = ceres::DENSE_QR;
    ceres::Solver::Summary summary; ceres::Solve(options, &problem, &summary);
    assert(summary.IsSolutionUsable() && summary.final_cost < 1e-12);
    for (int k = 0; k < 3; ++k) { assert(std::abs(v[0][k]) < 1e-8); assert(std::abs(v[1][k]) < 1e-8); assert(std::abs(p[1][k] - p[0][k]) < 1e-8); }
    assert(v[2][0] == 1 && p[2][0] == 7); // moving history/current states remain untouched
    bool released[3] = {true, false, true}; ceres::Problem second;
    count = addStationaryFactors(second, released, intervals, p, v, 2, .03, .02);
    assert(count.zero_velocity == 2 && count.relative_position == 0); // no interval across motion
    assert(!second.HasParameterBlock(v[1]));
}

void test_actual_sliding_window_continuity()
{
    // Exercise the same helper called by Estimator::slideWindow SECOND_NEW:
    // stationary A, moving/unconfirmed B, stationary C. Discarding B keeps
    // C's valid zero-velocity evidence but may never imply A -> C stationary.
    bool frames[3] = {true, false, true};
    bool intervals[3] = {false, false, false};
    double p[3][7] = {{1, 0, 0, 0, 0, 0, 1}, {4, 0, 0, 0, 0, 0, 1}, {7, 0, 0, 0, 0, 0, 1}};
    double v[3][9] = {{.1, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    slideStationaryEvidenceSecondNew(frames, intervals, 2);
    assert(frames[0] && frames[1] && !frames[2]);
    assert(!intervals[1]);
    ceres::Problem problem;
    auto counts = addStationaryFactors(problem, frames, intervals, p, v, 1, .03, .02);
    assert(counts.zero_velocity == 2 && counts.relative_position == 0);
    assert(!problem.HasParameterBlock(p[0]) && !problem.HasParameterBlock(p[1]));
    // Repeated static non-keyframes cannot erase the earlier unsupported path.
    frames[2] = true; intervals[2] = true;
    slideStationaryEvidenceSecondNew(frames, intervals, 2);
    assert(frames[1] && !intervals[1]);
    // Two independently supported intervals do preserve genuine continuity.
    bool contiguous[3] = {true, true, true}; bool links[3] = {false, true, true};
    slideStationaryEvidenceSecondNew(contiguous, links, 2);
    assert(contiguous[1] && links[1] && !contiguous[2] && !links[2]);
    // OLD shifts links with their endpoint states and discards the prior edge.
    bool older[4] = {false, true, true, true}; bool older_links[4] = {false, false, true, false};
    slideStationaryEvidenceOld(older, older_links, 3);
    assert(older[0] && older[1] && older[2] && !older[3]);
    assert(!older_links[0] && older_links[1] && !older_links[2] && !older_links[3]);
}

std::vector<StereoWindowTrack> stereo_frame(int index, double velocity, double jitter = .04)
{
    std::vector<StereoWindowTrack> tracks;
    const double time = index * .1;
    const double noise = index % 2 ? jitter : -jitter;
    for (int id = 0; id < 30; ++id)
        tracks.push_back({id, Eigen::Vector2d(350 + id + velocity * time + noise, 200),
            Eigen::Vector2d(340 + id + velocity * time - noise, 200)});
    return tracks;
}

void test_temporal_visual_stationarity_and_motion()
{
    TemporalStereoWindow window; SensorStationaryDetector detector;
    auto e = evidence(); StereoWindowEvidence fit;
    for (int i = 0; i <= 30; ++i)
    {
        fit = window.addFrame(i * .1, stereo_frame(i, 0));
        e.window_supported_tracks = fit.supported_tracks; e.window_span_s = fit.minimum_span_s;
        e.flow_p90_px_s = fit.flow_p90_px_s; e.flow_median_px_s = fit.flow_median_px_s; e.fit_jitter_p90_px = fit.fit_jitter_p90_px;
        detector.update(i * .1, e);
    }
    assert(detector.active && fit.flow_p90_px_s < .05); // .8 px/s short-step jitter is harmless after fitting
    e.instant_flow_p90_px_s = 2.001; assert(!detector.update(3.1, e)); // obvious motion releases at this same frame
    e = evidence(); e.instant_flow_median_px_s = .751; assert(!detector.update(3.2, e));
    window.clear(); detector.clear();
    for (int i = 0; i <= 30; ++i)
    {
        fit = window.addFrame(i * .1, stereo_frame(i, 1));
        e = evidence(); e.window_supported_tracks = fit.supported_tracks; e.window_span_s = fit.minimum_span_s;
        e.flow_p90_px_s = fit.flow_p90_px_s; e.flow_median_px_s = fit.flow_median_px_s; e.fit_jitter_p90_px = fit.fit_jitter_p90_px;
        assert(!detector.update(i * .1, e)); // constant speed, quiet IMU must never become stationary
    }
    assert(fit.flow_p90_px_s > .95);
    window.clear();
    for (int i = 0; i <= 20; ++i)
    {
        auto f = stereo_frame(i, 0, 0); for (auto &track : f) track.right_uv.x() += 2 * i * .1;
        fit = window.addFrame(i * .1, f);
    }
    assert(fit.flow_p90_px_s > 1.99); // faster right eye cannot be hidden
    fit = window.addFrame(2.1, {}); assert(fit.supported_tracks == 0);
    fit = window.addFrame(2.2, stereo_frame(22, 0)); assert(fit.supported_tracks == 0);
    for (int i = 23; i <= 31; ++i) fit = window.addFrame(i * .1, stereo_frame(i, 0));
    assert(fit.supported_tracks == 0);
    fit = window.addFrame(3.2, stereo_frame(32, 0)); assert(fit.supported_tracks == 30);
    fit = window.addFrame(4.3, stereo_frame(43, 0)); assert(fit.supported_tracks == 0);
    fit = window.addFrame(4.2, stereo_frame(42, 0)); assert(fit.supported_tracks == 0);
    e = evidence(); e.fit_jitter_p90_px = .061; assert(!detector.update(4.4, e));
}

void test_same_stamp_propagation_refresh()
{
    NativeImuPrediction old, current;
    old.stamp = 48.272; old.p = Eigen::Vector3d(.039321672998, -.896660961413, -.107238883435);
    current.stamp = 48.272; current.p = Eigen::Vector3d(.124021, -.55966, -.06043);
    NativeRefreshEvidence e;
    assert(!nativeRefreshConsistent(old, current, Eigen::Vector3d(0, 0, -.02), &e));
    current.p = old.p + Eigen::Vector3d(.1, 0, 0);
    assert(nativeRefreshConsistent(old, current, Eigen::Vector3d(0, 0, -.02), &e));
    assert(e.comparison_stamp_sim_s == 48.272);
    current.stamp += .004; assert(!nativeRefreshConsistent(old, current, Eigen::Vector3d::Zero(), &e));
    // Actual lever-arm orientation can cross the body jump boundary even when
    // the native IMU-position correction alone is below it.
    old = NativeImuPrediction(); current = old;
    current.p.x() = .3499; current.q = Eigen::Quaterniond(Eigen::AngleAxisd(-.02, Eigen::Vector3d::UnitY()));
    assert(!nativeRefreshConsistent(old, current, Eigen::Vector3d(0, 0, -.02), &e));
    assert(e.imu_position_delta_m < .35 && e.body_position_delta_m > .35);
    // Test the actual candidate propagation formula with a real measurement
    // clock and gravity removal, without using a replacement sample stamp.
    NativeImuPrediction prediction; prediction.acc = Eigen::Vector3d(1, 0, 9.8066);
    assert(predictNativeImu(prediction, .1, prediction.acc, Eigen::Vector3d::Zero(), Eigen::Vector3d(0, 0, 9.8066)));
    assert(std::abs(prediction.p.x() - .005) < 1e-12 && std::abs(prediction.v.x() - .1) < 1e-12);
    assert(!predictNativeImu(prediction, .05, prediction.acc, Eigen::Vector3d::Zero(), Eigen::Vector3d(0, 0, 9.8066)));
    assert(!advancePendingPublishedImu(prediction, .112, prediction.acc, Eigen::Vector3d::Zero(), Eigen::Vector3d(0, 0, 9.8066)));
    assert(prediction.stamp == .1); // rejected long gap leaves the old state untouched
    assert(advancePendingPublishedImu(prediction, .104, prediction.acc, Eigen::Vector3d::Zero(), Eigen::Vector3d(0, 0, 9.8066)));
}

void test_real_failure_values()
{
    Eigen::Vector3d dp(-.220759, -.328156, -.120976), v(-.40798, -.00539, -.10777);
    assert(!usableOptimizedKinematics(.1, dp, v, v, .01));
    assert(!usableOptimizedKinematics(.1, Eigen::Vector3d(.35, 0, 0), Eigen::Vector3d(-.4, 0, 0), Eigen::Vector3d(-.4, 0, 0), .01));
    assert(usableOptimizedKinematics(.1, Eigen::Vector3d(.1, 0, 0), Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(1, 0, 0), .01));
    assert(usableOptimizedKinematics(.1, Eigen::Vector3d(.2, .1, 0), Eigen::Vector3d(.1, 0, 0), Eigen::Vector3d(.1, 0, 0), .1));
    assert(!usableOptimizedKinematics(.1, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), .61));
    assert(!usableOptimizedKinematics(0, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 0));
}

int main()
{
    test_detector(); test_stereo(); test_jacobians(); test_real_solver_and_history(); test_actual_sliding_window_continuity(); test_temporal_visual_stationarity_and_motion(); test_same_stamp_propagation_refresh(); test_real_failure_values();
    std::cout << "PASS: raw IMU/stereo gates, analytical factors, Ceres solve/history release, actual failure kinematics" << std::endl;
}
