#pragma once
// PX4_AGENT_STATIONARY_INITIALIZATION_V9: source-only startup validation.
// Not a new factor or measurement. Actual native code calls these validations.
#include <cmath>
#include <string>
#include <vector>
#include <algorithm>

inline bool validateInitialHeaderEdges(const std::vector<double> &headers,
    const std::vector<double> &map_headers, const std::vector<double> &window_spans,
    const std::vector<double> &image_edge_spans, std::string &reason)
{
    if (headers.empty() || headers.size() != map_headers.size()
        || headers.size() != window_spans.size() || headers.size() != image_edge_spans.size())
    { reason = "initial_window_map_size_mismatch"; return false; }
    for (std::size_t i = 0; i < headers.size(); ++i)
    {
        if (!std::isfinite(headers[i]) || (i && headers[i] <= headers[i-1])
            || headers[i] != map_headers[i])
        { reason = "initial_window_header_mapping_invalid"; return false; }
        if (i && (!std::isfinite(window_spans[i]) || !std::isfinite(image_edge_spans[i])
            || window_spans[i] <= 0 || image_edge_spans[i] <= 0
            || std::abs(window_spans[i] - (headers[i]-headers[i-1])) > 1e-5
            || std::abs(image_edge_spans[i] - (headers[i]-headers[i-1])) > 1e-5))
        { reason = "initial_window_raw_imu_edge_mismatch"; return false; }
    }
    reason.clear(); return true;
}

inline bool confirmedStationaryInitialWindow(const bool *frames, const bool *intervals,
    int last, bool current_source_gate, bool current_confirmed, std::string &reason)
{
    if (last < 1 || !current_source_gate || !current_confirmed)
    { reason = "current_stationary_source_not_confirmed"; return false; }
    for (int i = 0; i <= last; ++i)
    {
        if (!frames[i]) { reason = "window_frame_not_stationary_confirmed"; return false; }
        if (i && !intervals[i]) { reason = "window_interval_not_stationary_continuous"; return false; }
    }
    reason.clear(); return true;
}
