// Local integration patch, GPL-3.0, like the surrounding VINS-Fusion code.
#pragma once
#include <cmath>
#include <vector>

// Decide before optimization and before SECOND_NEW merges two actual IMU
// intervals. OLD retains both measured intervals, then moves the standard
// sliding window; it never shortens, clips or fabricates any measurement dt.
inline bool forceKeyframeForImuSpan(bool use_imu, bool full_window,
        double previous_interval_s, double current_interval_s, double bound_s)
{
    return use_imu && full_window && std::isfinite(bound_s) && bound_s > 0
        && std::isfinite(previous_interval_s) && previous_interval_s >= 0
        && std::isfinite(current_interval_s) && current_interval_s >= 0
        && previous_interval_s + current_interval_s >= bound_s - 1e-9;
}

// A diagnostic of the direct IMU/bias transmission path, not a claim about
// connectivity/rank of the complete nonlinear visual factor graph. An edge
// denotes an IMUFactor actually included in this optimization. Roots denote
// SpeedBias blocks present in the existing marginalization prior, or the
// single initial sensor-bias prior when no marginalization prior exists.
inline bool priorReachesLatestViaImu(const std::vector<bool>& prior_roots,
        const std::vector<bool>& included_edges)
{
    if (prior_roots.empty() || included_edges.size() + 1 != prior_roots.size()) return false;
    size_t first = prior_roots.size() - 1;
    while (first > 0 && included_edges[first - 1]) --first;
    for (size_t i = first; i < prior_roots.size(); ++i)
        if (prior_roots[i]) return true;
    return false;
}
