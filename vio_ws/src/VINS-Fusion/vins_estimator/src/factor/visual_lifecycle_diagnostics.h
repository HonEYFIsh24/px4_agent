// Read-only, bounded snapshots. Never changes FeatureManager::used_num or the
// optimizer, outlier rule, tracker IDs, source timestamps or window contents.
#pragma once
#include <cmath>
#include <limits>
#include <set>
#include <sstream>
#include <string>
#include <vector>

struct NativeVisualFeatureRow {
    int id, start_frame, observations, solve_flag;
    double depth;
};
struct NativeVisualSnapshot {
    std::string phase;
    int total = 0, eligible = 0;
    std::vector<NativeVisualFeatureRow> rows;
};
struct NativeVisualReprojection {
    int id, start_frame, observations, error_terms;
    double depth, mean_normalized, mean_virtual_px;
    bool removed;
};
struct NativeVisualLifecycle {
    enum { FEATURE_LIMIT = 128, REMOVED_ID_LIMIT = 256 };
    bool sampled = false, outliers_evaluated = false;
    double source_stamp = -1;
    int solve_sequence = 0, reprojection_total = 0;
    std::string outcome = "not_sampled";
    std::vector<double> headers, imu_spans;
    NativeVisualSnapshot before, solved, after;
    std::vector<NativeVisualReprojection> reprojections;
    int removed_ids_total = 0;
    std::vector<int> removed_ids;

    template <typename Manager>
    static NativeVisualSnapshot snapshot(const Manager &manager, const std::string &phase) {
        NativeVisualSnapshot result; result.phase = phase;
        for (const auto &feature : manager.feature) {
            ++result.total;
            const int n = static_cast<int>(feature.feature_per_frame.size());
            if (n >= 4) ++result.eligible; // same eligibility as getFeatureCount
            if (result.rows.size() < FEATURE_LIMIT)
                result.rows.push_back({feature.feature_id, feature.start_frame,
                                       n, feature.solve_flag, feature.estimated_depth});
        }
        return result;
    }
    void addReprojection(int id, int start, int n, double depth, int terms,
                         double mean, double focal, bool removed) {
        ++reprojection_total;
        if (reprojections.size() < FEATURE_LIMIT)
            reprojections.push_back({id,start,n,terms,depth,mean,mean*focal,removed});
    }
    static void number(std::ostringstream &out, double value) {
        if (std::isfinite(value)) out << value; else out << "null";
    }
    static void writeSnapshot(std::ostringstream &out, const NativeVisualSnapshot &snapshot) {
        out << "{\"phase\":\"" << snapshot.phase << "\",\"total_features\":" << snapshot.total
            << ",\"eligible_features\":" << snapshot.eligible << ",\"recorded_features\":" << snapshot.rows.size()
            << ",\"omitted_features\":" << snapshot.total-snapshot.rows.size()
            << ",\"truncated\":" << (snapshot.total > static_cast<int>(snapshot.rows.size()) ? "true":"false")
            << ",\"features\":[";
        bool first = true;
        for (const auto &row : snapshot.rows) {
            if (!first) out << ","; first = false;
            out << "{\"id\":" << row.id << ",\"start_frame\":" << row.start_frame
                << ",\"observation_count\":" << row.observations << ",\"solve_flag\":" << row.solve_flag
                << ",\"estimated_depth_m\":"; number(out,row.depth); out << "}";
        }
        out << "]}";
    }
    std::string json(bool enabled, double visual_gap_start = -1) const {
        if (!enabled) return "{\"enabled\":false}";
        std::ostringstream out; out.precision(12);
        out << "{\"enabled\":true,\"schema_version\":1,\"sampled\":" << (sampled?"true":"false")
            << ",\"solve_sequence\":" << solve_sequence << ",\"source_stamp_sim_s\":"; number(out,source_stamp);
        out << ",\"visual_gap_since_source_stamp_sim_s\":"; number(out,visual_gap_start);
        out << ",\"visual_gap_elapsed_sim_s\":"; number(out,visual_gap_start < 0 ? 0 : source_stamp-visual_gap_start);
        out << ",\"snapshot_scope\":\"actual_solve_before_window_slide\",\"eligibility_rule\":\"observation_count_ge_4_only\""
            << ",\"feature_limit_per_snapshot\":" << FEATURE_LIMIT << ",\"removed_id_limit\":" << REMOVED_ID_LIMIT
            << ",\"outcome\":\"" << outcome << "\",\"window_headers_sim_s\":[";
        for (std::size_t i=0;i<headers.size();++i) { if(i)out<<","; number(out,headers[i]); }
        out << "],\"original_imu_spans_s\":[";
        for (std::size_t i=0;i<imu_spans.size();++i) { if(i)out<<","; number(out,imu_spans[i]); }
        out << "],\"before_solve\":"; writeSnapshot(out,before);
        out << ",\"post_solve_pre_outlier\":"; writeSnapshot(out,solved);
        out << ",\"post_outlier_before_slide\":"; writeSnapshot(out,after);
        out << ",\"outliers_evaluated\":" << (outliers_evaluated?"true":"false")
            << ",\"reprojection_focal_length_virtual_px\":460,\"outlier_threshold_virtual_px\":3"
            << ",\"reprojection_mean_semantics\":\"existing_outlier_loop_mean_of_norms_no_extra_evaluation\""
            << ",\"evaluated_feature_count\":" << reprojection_total
            << ",\"omitted_reprojection_rows\":" << reprojection_total-reprojections.size()
            << ",\"reprojection_rows_truncated\":" << (reprojection_total>static_cast<int>(reprojections.size())?"true":"false")
            << ",\"reprojections\":[";
        bool first=true;
        for (const auto &row:reprojections) {
            if(!first)out<<",";first=false;
            out << "{\"id\":" << row.id << ",\"start_frame\":" << row.start_frame
                << ",\"observation_count\":" << row.observations << ",\"error_terms\":" << row.error_terms
                << ",\"estimated_depth_m\":"; number(out,row.depth);
            out << ",\"average_normalized_error\":"; number(out,row.mean_normalized);
            out << ",\"average_virtual_px_error\":"; number(out,row.mean_virtual_px);
            out << ",\"removed\":" << (row.removed?"true":"false") << "}";
        }
        out << "],\"removed_ids_total\":" << removed_ids_total
            << ",\"removed_ids_truncated\":" << (removed_ids_total>REMOVED_ID_LIMIT?"true":"false")
            << ",\"removed_ids\":[";
        int recorded=0;
        for(int id:removed_ids) { if(recorded==REMOVED_ID_LIMIT)break; if(recorded++)out<<",";out<<id; }
        out << "],\"removed_ids_omitted\":" << removed_ids_total-recorded << "}";
        return out.str();
    }
};
