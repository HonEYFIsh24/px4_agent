// Diagnostic only, GPL-3.0 like VINS-Fusion. No factor or estimator state changes.
#pragma once
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

struct BiasInformationResult
{
    bool valid = false;
    std::string reason;
    Eigen::MatrixXd projected_jacobian;
    Eigen::VectorXd projected_residual;
    Eigen::Matrix<double,6,6> information = Eigen::Matrix<double,6,6>::Zero();
    Eigen::Matrix<double,6,6> factory_scaled_information = Eigen::Matrix<double,6,6>::Zero();
    Eigen::Matrix<double,6,1> gradient = Eigen::Matrix<double,6,1>::Zero();
    Eigen::Matrix<double,6,1> gauss_newton_delta = Eigen::Matrix<double,6,1>::Zero();
    Eigen::Matrix<double,6,1> factory_scaled_singular_values = Eigen::Matrix<double,6,1>::Zero();
    Eigen::VectorXd nuisance_singular_values, nuisance_column_norms;
    int nuisance_rank = 0, bias_rank = 0;
    double rank_rtol = 1e-8, rank_atol = 1e-12;
    double nuisance_threshold = 0, bias_threshold = 0;
    double condition_number = std::numeric_limits<double>::infinity();
    double projected_quadratic_cost = 0;
    bool nuisance_columns_normalized = true;
};

inline BiasInformationResult effectiveBiasInformation(
    const Eigen::MatrixXd& jacobian, const Eigen::VectorXd& residual,
    const std::vector<int>& bias_columns, const std::vector<int>& nuisance_columns,
    const Eigen::Matrix<double,6,1>& factory_sigmas,
    double rank_rtol = 1e-8, double rank_atol = 1e-12, bool normalize_nuisance_columns = true)
{
    BiasInformationResult out;
    out.rank_rtol = rank_rtol; out.rank_atol = rank_atol;
    out.nuisance_columns_normalized = normalize_nuisance_columns;
    if (jacobian.rows() == 0 || residual.size() != jacobian.rows()
        || bias_columns.size() != 6 || !jacobian.allFinite() || !residual.allFinite()
        || !factory_sigmas.allFinite() || (factory_sigmas.array() <= 0).any()
        || !std::isfinite(rank_rtol) || rank_rtol <= 0 || !std::isfinite(rank_atol) || rank_atol <= 0)
    { out.reason = "invalid_dimensions_values_or_threshold"; return out; }
    std::vector<bool> used(jacobian.cols(), false);
    for (int col : bias_columns)
    {
        if (col < 0 || col >= jacobian.cols() || used[col])
        { out.reason = "invalid_or_duplicate_bias_column"; return out; }
        used[col] = true;
    }
    for (int col : nuisance_columns)
    {
        if (col < 0 || col >= jacobian.cols() || used[col])
        { out.reason = "invalid_or_overlapping_nuisance_column"; return out; }
        used[col] = true;
    }
    out.projected_jacobian.resize(jacobian.rows(), 6);
    for (int i = 0; i < 6; ++i) out.projected_jacobian.col(i) = jacobian.col(bias_columns[i]);
    out.projected_residual = residual;
    if (!nuisance_columns.empty())
    {
        Eigen::MatrixXd nuisance(jacobian.rows(), nuisance_columns.size());
        out.nuisance_column_norms.resize(nuisance_columns.size());
        for (int i = 0; i < static_cast<int>(nuisance_columns.size()); ++i)
        {
            nuisance.col(i) = jacobian.col(nuisance_columns[i]);
            out.nuisance_column_norms[i] = nuisance.col(i).norm();
            // Column scaling preserves the nuisance subspace. Zero columns
            // remain zero; the actual normalization and threshold are exposed.
            if (normalize_nuisance_columns && out.nuisance_column_norms[i] > 0)
                nuisance.col(i) /= out.nuisance_column_norms[i];
        }
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(nuisance, Eigen::ComputeThinU | Eigen::ComputeThinV);
        out.nuisance_singular_values = svd.singularValues();
        out.nuisance_threshold = std::max(rank_atol,
            rank_rtol * (out.nuisance_singular_values.size() ? out.nuisance_singular_values[0] : 0));
        for (int i = 0; i < out.nuisance_singular_values.size(); ++i)
            if (out.nuisance_singular_values[i] > out.nuisance_threshold) ++out.nuisance_rank;
        if (out.nuisance_rank)
        {
            const Eigen::MatrixXd basis = svd.matrixU().leftCols(out.nuisance_rank);
            out.projected_jacobian -= basis * (basis.transpose() * out.projected_jacobian).eval();
            out.projected_residual -= basis * (basis.transpose() * out.projected_residual).eval();
        }
    }
    const Eigen::MatrixXd scaled = out.projected_jacobian * factory_sigmas.asDiagonal();
    Eigen::JacobiSVD<Eigen::MatrixXd> bias_svd(scaled, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const Eigen::VectorXd singular = bias_svd.singularValues();
    for (int i = 0; i < singular.size(); ++i) out.factory_scaled_singular_values[i] = singular[i];
    out.bias_threshold = std::max(rank_atol, rank_rtol * out.factory_scaled_singular_values[0]);
    Eigen::VectorXd inverse = Eigen::VectorXd::Zero(singular.size());
    for (int i = 0; i < singular.size(); ++i)
        if (singular[i] > out.bias_threshold) { ++out.bias_rank; inverse[i] = 1/singular[i]; }
    // Forming a 6x6 Gram matrix only after orthogonal elimination avoids the
    // Arr-Arm*Amm^-1*Amr subtraction that the diagnostic is intended to audit.
    out.information = out.projected_jacobian.transpose() * out.projected_jacobian;
    out.factory_scaled_information = scaled.transpose() * scaled;
    out.gradient = out.projected_jacobian.transpose() * out.projected_residual;
    out.gauss_newton_delta = factory_sigmas.asDiagonal() *
        (-bias_svd.matrixV() * inverse.asDiagonal() * bias_svd.matrixU().transpose() * out.projected_residual);
    if (out.bias_rank == 6)
        out.condition_number = std::pow(out.factory_scaled_singular_values[0]/out.factory_scaled_singular_values[5],2);
    out.projected_quadratic_cost = .5*out.projected_residual.squaredNorm();
    out.valid = out.information.allFinite() && out.factory_scaled_information.allFinite()
        && out.gradient.allFinite() && out.gauss_newton_delta.allFinite();
    if (!out.valid) out.reason = "nonfinite_projected_result";
    return out;
}
