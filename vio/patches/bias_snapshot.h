// Diagnostic serialization only, GPL-3.0 like the surrounding native source.
#pragma once
#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include <string>
#include <ceres/ceres.h>
#include <fstream>
#include <iomanip>
#include <sstream>

struct BiasSnapshotBlock
{
    std::string label;
    std::string matrix_local_basis;
    int column = 0, global_size = 0, local_size = 0;
    int actual_optimizer_local_size = -1;
    Eigen::MatrixXd optimizer_to_matrix_local;
    Eigen::MatrixXd physical_pose_tangent_from_optimizer;
    bool constant_in_actual_problem = false, present_in_actual_problem = false;
    double source_stamp = -1;
    std::vector<double> values, linearization_values;
};

inline std::string biasJsonQuote(const std::string& s)
{
    std::ostringstream o; o << '"';
    for (unsigned char c : s)
        if (c == '"' || c == '\\') o << '\\' << c;
        else if (c < 32) o << "\\u" << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(c) << std::dec;
        else o << c;
    o << '"'; return o.str();
}

template<class T> inline void biasWriteVector(std::ostream& o, const T& v)
{
    o << '[';
    for (int i = 0; i < static_cast<int>(v.size()); ++i)
    { if (i) o << ','; if (std::isfinite(static_cast<double>(v[i]))) o << v[i]; else o << "null"; }
    o << ']';
}

inline void biasWriteMatrix(std::ostream& o, const Eigen::MatrixXd& m)
{
    o << '[';
    for (int i = 0; i < m.rows(); ++i)
    { if (i) o << ','; biasWriteVector(o, Eigen::VectorXd(m.row(i).transpose())); }
    o << ']';
}

inline void biasWriteBlocks(std::ostream& o, const std::vector<BiasSnapshotBlock>& blocks)
{
    o << '[';
    for (int i = 0; i < static_cast<int>(blocks.size()); ++i)
    {
        if (i) o << ','; const auto& b = blocks[i];
        o << "{\"label\":" << biasJsonQuote(b.label) << ",\"column\":" << b.column
          << ",\"global_size\":" << b.global_size << ",\"local_size\":" << b.local_size
          << ",\"matrix_local_basis\":" << biasJsonQuote(b.matrix_local_basis)
          << ",\"actual_optimizer_local_size\":" << b.actual_optimizer_local_size
          << ",\"optimizer_to_matrix_local\":";biasWriteMatrix(o,b.optimizer_to_matrix_local);
        o << ",\"physical_pose_tangent_from_optimizer\":";biasWriteMatrix(o,b.physical_pose_tangent_from_optimizer);
        o
          << ",\"present_in_actual_problem\":" << (b.present_in_actual_problem ? "true" : "false")
          << ",\"constant_in_actual_problem\":" << (b.constant_in_actual_problem ? "true" : "false")
          << ",\"source_stamp_sim_s\":" << b.source_stamp << ",\"global_values\":";
        biasWriteVector(o,b.values);o << ",\"linearization_global_values\":";
        biasWriteVector(o,b.linearization_values);o << '}';
    }
    o << ']';
}

inline std::string biasSnapshotPath(const std::string& directory, int sequence,
    double stamp, const std::string& phase)
{
    std::ostringstream path;path << directory << "/bias_" << sequence << '_'
        << static_cast<long long>(std::llround(stamp*1e9)) << '_' << phase << ".json";
    return path.str();
}

inline void biasWriteSnapshotPrefix(std::ostream& o, double stamp, const std::string& phase,
    const std::string& context, const std::vector<BiasSnapshotBlock>& blocks,
    const Eigen::Matrix<double,6,1>& sigmas, const std::string& matrix_kind)
{
    o << std::setprecision(17) << "{\"schema_version\":3,\"diagnostic_only\":true,\"source_stamp_sim_s\":" << stamp
      << ",\"snapshot_phase\":" << biasJsonQuote(phase) << ",\"context\":" << context
      << ",\"matrix_kind\":" << biasJsonQuote(matrix_kind)
      << ",\"column_basis\":\"per_parameter_block_matrix_local_basis\""
      << ",\"bias_coordinate_order\":[\"Ba_x\",\"Ba_y\",\"Ba_z\",\"Bg_x\",\"Bg_y\",\"Bg_z\"]"
      << ",\"initial_factory_bias_sigmas\":";
    biasWriteVector(o,sigmas); o << ",\"parameter_blocks\":";biasWriteBlocks(o,blocks);
}

inline bool biasWriteDenseSnapshot(const std::string& path, double stamp, const std::string& phase,
    const std::string& context, const Eigen::MatrixXd& j, const Eigen::VectorXd& r,
    const Eigen::VectorXd& stored_r, const std::vector<BiasSnapshotBlock>& blocks,
    const Eigen::Matrix<double,6,1>& sigmas, const Eigen::MatrixXd* stored_jacobian = nullptr)
{
    if (std::ifstream(path).good() || !j.allFinite() || !r.allFinite()) return false;
    std::ofstream o(path);if (!o) return false;
    biasWriteSnapshotPrefix(o,stamp,phase,context,blocks,sigmas,"dense_native_marginalization");
    o << ",\"jacobian\":";biasWriteMatrix(o,j);o << ",\"residual\":";biasWriteVector(o,r);
    o << ",\"stored_linearized_residual\":";biasWriteVector(o,stored_r);
    if (stored_jacobian) { o << ",\"stored_frozen_prior_jacobian\":";biasWriteMatrix(o,*stored_jacobian); }
    o << ",\"robust_loss_provenance\":\"native_ResidualBlockInfo_loss_scaling_already_applied\"";
    o << '}';return o.good();
}

inline bool biasWriteCrsSnapshot(const std::string& path, double stamp, const std::string& phase,
    const std::string& context, const ceres::CRSMatrix& j, const std::vector<double>& r,
    const std::vector<double>& gradient, const std::vector<BiasSnapshotBlock>& blocks,
    const Eigen::Matrix<double,6,1>& sigmas, double actual_cost)
{
    if (std::ifstream(path).good()) return false;
    std::ofstream o(path);if (!o) return false;
    biasWriteSnapshotPrefix(o,stamp,phase,context,blocks,sigmas,"ceres_actual_variable_only_crs");
    o << ",\"jacobian_crs\":{\"num_rows\":" << j.num_rows << ",\"num_cols\":" << j.num_cols << ",\"rows\":";
    biasWriteVector(o,j.rows);o << ",\"cols\":";biasWriteVector(o,j.cols);o << ",\"values\":";biasWriteVector(o,j.values);
    o << "},\"residual\":";biasWriteVector(o,r);o << ",\"ceres_evaluate_gradient\":";biasWriteVector(o,gradient);
    o << ",\"ceres_evaluate_cost\":" << actual_cost
      << ",\"ceres_apply_loss_function\":true,\"fixed_parameter_blocks_excluded\":true}";
    return o.good();
}
