// Local VINS simulation patch, GPL-3.0 like the surrounding estimator.
#pragma once
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <deque>
#include <map>
#include <vector>

struct StereoWindowTrack
{
    int id;
    Eigen::Vector2d left_uv, right_uv;
};

struct StereoWindowSample
{
    double t;
    Eigen::Vector2d left_uv, right_uv;
};

struct StereoWindowEvidence
{
    int supported_tracks = 0;
    double minimum_span_s = 0;
    double flow_median_px_s = 1e9;
    double flow_p90_px_s = 1e9;
    double fit_jitter_p90_px = 1e9;
};

inline double stationaryQuantile(std::vector<double> values, double fraction)
{
    if (values.empty()) return 1e9;
    std::sort(values.begin(), values.end());
    const size_t i = fraction == 0.5 ? (values.size() - 1) / 2
        : static_cast<size_t>(std::ceil(fraction * values.size())) - 1;
    return values[std::min(i, values.size() - 1)];
}

// Only real measurement stamps and raw image feature pixels enter this class.
// The caller supplies continuously seen, positive-disparity nearby stereo IDs.
// A missing ID is erased immediately and cannot regain its old history.
class TemporalStereoWindow
{
public:
    double window_s = 1.0;
    double minimum_span_s = 0.98;
    double maximum_image_gap_s = 0.2;
    int minimum_samples = 8;
    std::map<int, std::deque<StereoWindowSample>> tracks;
    double previous_frame_t = -1;

    void clear() { tracks.clear(); previous_frame_t = -1; }

    StereoWindowEvidence addFrame(double stamp, const std::vector<StereoWindowTrack> &current)
    {
        StereoWindowEvidence result;
        if (!std::isfinite(stamp)) { clear(); return result; }
        if (previous_frame_t >= 0 && (stamp <= previous_frame_t
                || stamp - previous_frame_t > maximum_image_gap_s + 1e-6)) tracks.clear();
        previous_frame_t = stamp;
        std::map<int, std::deque<StereoWindowSample>> next;
        std::vector<double> flows, jitters, spans;
        for (const auto &track : current)
        {
            if (!track.left_uv.allFinite() || !track.right_uv.allFinite()) continue;
            auto previous = tracks.find(track.id);
            auto &samples = next[track.id];
            if (previous != tracks.end()) samples.swap(previous->second);
            samples.push_back({stamp, track.left_uv, track.right_uv});
            while (!samples.empty() && samples.front().t < stamp - window_s - 1e-6) samples.pop_front();
            if (samples.size() < static_cast<size_t>(minimum_samples)
                || stamp - samples.front().t < minimum_span_s) continue;
            double time_mean = 0;
            Eigen::Vector2d left_mean = Eigen::Vector2d::Zero(), right_mean = Eigen::Vector2d::Zero();
            for (const auto &s : samples)
            { time_mean += s.t - stamp; left_mean += s.left_uv; right_mean += s.right_uv; }
            time_mean /= samples.size(); left_mean /= samples.size(); right_mean /= samples.size();
            double time_squared = 0;
            Eigen::Vector2d left_cov = Eigen::Vector2d::Zero(), right_cov = Eigen::Vector2d::Zero();
            for (const auto &s : samples)
            {
                const double dt = s.t - stamp - time_mean;
                time_squared += dt * dt;
                left_cov += dt * (s.left_uv - left_mean); right_cov += dt * (s.right_uv - right_mean);
            }
            if (time_squared <= 0 || !std::isfinite(time_squared)) continue;
            const Eigen::Vector2d left_velocity = left_cov / time_squared, right_velocity = right_cov / time_squared;
            double left_residual = 0, right_residual = 0;
            for (const auto &s : samples)
            {
                const double dt = s.t - stamp - time_mean;
                left_residual += (s.left_uv - left_mean - dt * left_velocity).squaredNorm();
                right_residual += (s.right_uv - right_mean - dt * right_velocity).squaredNorm();
            }
            const double speed = std::max(left_velocity.norm(), right_velocity.norm());
            const double jitter = std::sqrt(std::max(left_residual, right_residual) / samples.size());
            if (!std::isfinite(speed) || !std::isfinite(jitter)) continue;
            flows.push_back(speed); jitters.push_back(jitter); spans.push_back(stamp - samples.front().t);
        }
        tracks.swap(next);
        result.supported_tracks = static_cast<int>(flows.size());
        if (!flows.empty())
        {
            result.minimum_span_s = *std::min_element(spans.begin(), spans.end());
            result.flow_median_px_s = stationaryQuantile(flows, 0.5);
            result.flow_p90_px_s = stationaryQuantile(flows, 0.9);
            result.fit_jitter_p90_px = stationaryQuantile(jitters, 0.9);
        }
        return result;
    }
};
