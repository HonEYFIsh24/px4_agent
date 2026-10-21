// Opt-in evidence only. No residual, solver option, health gate or state changes.
#pragma once
#include "bias_snapshot.h"
#include "fixed_yaw_pose_parameterization.h"
#include <typeinfo>

// Visual-only native Ceres already fixes pose0 completely; the physical
// four-DOF IMU gauge quotient applies only to an unanchored IMU problem.
inline int biasFreeGlobalCoordinateGaugeDimensions(bool use_imu,bool fixed_xyz_yaw)
{ return !use_imu||fixed_xyz_yaw ? 0 : 4; }
inline bool biasPhysicalGlobalGaugeQuotientApplicable(bool use_imu,bool fixed_xyz_yaw)
{ return use_imu&&!fixed_xyz_yaw; }

inline std::string biasResidualSegment(int row, int count, const ceres::CostFunction* cost,
    const ceres::LossFunction* loss)
{
    std::ostringstream o;o<<"{\"row\":"<<row<<",\"num_residuals\":"<<count
      <<",\"cost_function_rtti\":"<<biasJsonQuote(typeid(*cost).name())
      <<",\"has_loss\":"<<(loss?"true":"false")<<",\"loss_function_rtti\":"
      <<biasJsonQuote(loss?typeid(*loss).name():"")<<'}';return o.str();
}

template<class E> inline BiasSnapshotBlock biasDescribeBlock(E& e, ceres::Problem& p,
    double* values, int size, int col, int local, const double* linearization = nullptr)
{
    BiasSnapshotBlock b;b.label="unknown";b.global_size=size;b.local_size=local;b.column=col;
    b.present_in_actual_problem=p.HasParameterBlock(values);
    b.constant_in_actual_problem=b.present_in_actual_problem && p.IsParameterBlockConstant(values);
    b.actual_optimizer_local_size=b.present_in_actual_problem ? p.ParameterBlockLocalSize(values) : -1;
    b.matrix_local_basis=size==7 ? "native_translation_right_body_rotation_6" : "native_euclidean";
    if(size==7 && local==2)b.matrix_local_basis="first_pose_fixed_xyz_yaw_euler_roll_pitch_milliradians_2";
    if(b.actual_optimizer_local_size>=0)
    {
        b.optimizer_to_matrix_local=Eigen::MatrixXd::Identity(local,b.actual_optimizer_local_size);
        if(size==7 && local==6 && b.actual_optimizer_local_size==2)
            b.optimizer_to_matrix_local=fixedYawPoseBodyTangentMap(values)*FIXED_YAW_RP_RADIANS_PER_LOCAL_UNIT;
        if(size==7)
            b.physical_pose_tangent_from_optimizer=b.actual_optimizer_local_size==2
                ? Eigen::MatrixXd(fixedYawPoseBodyTangentMap(values)*FIXED_YAW_RP_RADIANS_PER_LOCAL_UNIT) : Eigen::MatrixXd::Identity(6,b.actual_optimizer_local_size);
    }
    for (int i=0;i<=WINDOW_SIZE;++i)
    {
        if (values==e.para_Pose[i]) { b.label="pose_"+std::to_string(i);b.source_stamp=e.Headers[i]; }
        if (values==e.para_SpeedBias[i]) { b.label="speed_bias_"+std::to_string(i);b.source_stamp=e.Headers[i]; }
    }
    for (int i=0;i<NUM_OF_CAM;++i) if (values==e.para_Ex_Pose[i]) b.label="extrinsic_"+std::to_string(i);
    if (values==e.para_Td[0]) b.label="time_offset";
    for (int i=0;i<NUM_OF_F;++i) if (values==e.para_Feature[i]) b.label="feature_inverse_depth_"+std::to_string(i);
    b.values.assign(values,values+size);
    if (linearization) b.linearization_values.assign(linearization,linearization+size);
    return b;
}

template<class E> inline std::string biasDiagnosticContext(E& e, const std::string& extra="")
{
    std::ostringstream o;o<<std::setprecision(17)<<"{\"optimizer_window\":"<<e.native_imu_optimizer_window_json
        <<",\"native_gravity_world_m_s2\":";biasWriteVector(o,e.g);
    o<<",\"imu_noise_native_discrete_sigmas\":["<<ACC_N<<','<<GYR_N<<','<<ACC_W<<','<<GYR_W<<']'
     <<",\"latest_speedbias_label\":\"speed_bias_"<<e.frame_count<<"\""
     <<",\"configured_marginalization_pose_jacobian_mode\":"
     <<biasJsonQuote(EXACT_MARGINALIZATION_POSE_JACOBIAN ? "exact_residual_chain_rule" : "legacy_frozen_local")
     <<",\"first_pose_xyz_yaw_gauge_enabled\":"<<(USE_IMU&&FIX_FIRST_POSE_XYZ_YAW ? "true" : "false")
     <<",\"first_pose_rp_radians_per_optimizer_local_unit\":"<<(USE_IMU&&FIX_FIRST_POSE_XYZ_YAW ? FIXED_YAW_RP_RADIANS_PER_LOCAL_UNIT : 1.)
     <<",\"physical_marginalizer_pose_local_size\":6"
     <<",\"no_ground_truth_input\":true,\"native_residual_model_unchanged\":true,\"native_algorithm_unchanged\":"
     <<(EXACT_MARGINALIZATION_POSE_JACOBIAN||FIX_FIRST_POSE_XYZ_YAW ? "false" : "true")<<extra<<'}';return o.str();
}

template<class E> inline void biasRecordSnapshot(E& e, const std::string& phase,
    const std::string& path, bool success)
{
    std::ostringstream item;item<<"{\"phase\":"<<biasJsonQuote(phase)<<",\"path\":"<<biasJsonQuote(path)
        <<",\"written\":"<<(success?"true":"false")<<'}';e.bias_diagnostic_file_entries.push_back(item.str());
    std::ostringstream o;o<<std::setprecision(17)<<"{\"enabled\":true,\"sample_sequence\":"<<e.bias_diagnostic_sequence
        <<",\"source_stamp_sim_s\":"<<e.Headers[e.frame_count]<<",\"files\":[";
    for (int i=0;i<static_cast<int>(e.bias_diagnostic_file_entries.size());++i)
    { if (i) o<<',';o<<e.bias_diagnostic_file_entries[i]; }o<<"]}";e.native_bias_diagnostics_json=o.str();
}

inline Eigen::Matrix<double,6,1> biasFactorySigmas()
{
    Eigen::Matrix<double,6,1> s;s<<ACC_BIAS_PRIOR_SIGMA,ACC_BIAS_PRIOR_SIGMA,ACC_BIAS_PRIOR_SIGMA,
        GYR_BIAS_PRIOR_SIGMA,GYR_BIAS_PRIOR_SIGMA,GYR_BIAS_PRIOR_SIGMA;return s;
}

template<class E> inline void biasDumpProblem(E& e, ceres::Problem& problem, const std::string& phase)
{
    ceres::Problem::EvaluateOptions options;options.apply_loss_function=true;
    problem.GetResidualBlocks(&options.residual_blocks);
    std::ostringstream segments;segments<<",\"residual_segments\":[";int residual_row=0;
    for (int i=0;i<static_cast<int>(options.residual_blocks.size());++i)
    {
        const auto id=options.residual_blocks[i];const auto* cost=problem.GetCostFunctionForResidualBlock(id);
        if (i) segments<<',';segments<<biasResidualSegment(residual_row,cost->num_residuals(),cost,
            problem.GetLossFunctionForResidualBlock(id));residual_row+=cost->num_residuals();
    }
    segments<<"],\"matrix_scope\":\"actual_ceres_optimizer_variable_basis\",\"free_global_coordinate_gauge_dimensions\":"
        <<biasFreeGlobalCoordinateGaugeDimensions(USE_IMU,FIX_FIRST_POSE_XYZ_YAW)
        <<",\"physical_global_gauge_quotient_applicable\":"<<(biasPhysicalGlobalGaugeQuotientApplicable(USE_IMU,FIX_FIRST_POSE_XYZ_YAW)?"true":"false");
    std::vector<double*> all;problem.GetParameterBlocks(&all);
    std::vector<BiasSnapshotBlock> blocks;int col=0;
    for (double* b:all) if (!problem.IsParameterBlockConstant(b))
    {
        const int size=problem.ParameterBlockSize(b), local=problem.ParameterBlockLocalSize(b);
        options.parameter_blocks.push_back(b);blocks.push_back(biasDescribeBlock(e,problem,b,size,col,local));col+=local;
    }
    double cost=-1;std::vector<double> residual,gradient;ceres::CRSMatrix j;
    const bool evaluated=problem.Evaluate(options,&cost,&residual,&gradient,&j)
        && j.num_cols==col && static_cast<int>(residual.size())==j.num_rows && std::isfinite(cost);
    const std::string path=biasSnapshotPath(BIAS_DIAGNOSTICS_DIR,e.bias_diagnostic_sequence,e.Headers[e.frame_count],phase);
    const bool written=evaluated && biasWriteCrsSnapshot(path,e.Headers[e.frame_count],phase,
        biasDiagnosticContext(e,segments.str()),j,residual,gradient,blocks,biasFactorySigmas(),cost);
    biasRecordSnapshot(e,phase,path,written);
}

template<class E> inline void biasDumpCurrentPrior(E& e, ceres::Problem& problem,
    MarginalizationInfo* info, const std::vector<double*>& addresses, const std::string& phase)
{
    if (!info || !info->valid || addresses.size()!=info->keep_block_size.size()) return;
    std::vector<BiasSnapshotBlock> blocks;
    for (int i=0;i<static_cast<int>(addresses.size());++i)
        blocks.push_back(biasDescribeBlock(e,problem,addresses[i],info->keep_block_size[i],
            info->keep_block_idx[i]-info->m,info->localSize(info->keep_block_size[i]),info->keep_block_data[i]));
    Eigen::VectorXd residual(info->n);
    Eigen::MatrixXd current_jacobian=Eigen::MatrixXd::Zero(info->n,info->n);
    std::vector<std::vector<double>> jacobian_buffers(addresses.size());
    std::vector<double*> raw_jacobians(addresses.size());
    for (int i=0;i<static_cast<int>(addresses.size());++i)
    { jacobian_buffers[i].resize(info->n*info->keep_block_size[i]);raw_jacobians[i]=jacobian_buffers[i].data(); }
    MarginalizationFactor factor(info);
    if (!factor.Evaluate(addresses.data(),residual.data(),raw_jacobians.data())) return;
    for (int i=0;i<static_cast<int>(addresses.size());++i)
    {
        const int size=info->keep_block_size[i],local=info->localSize(size);
        Eigen::Map<Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>> j(raw_jacobians[i],info->n,size);
        current_jacobian.middleCols(info->keep_block_idx[i]-info->m,local)=j.leftCols(local);
    }
    const std::string path=biasSnapshotPath(BIAS_DIAGNOSTICS_DIR,e.bias_diagnostic_sequence,e.Headers[e.frame_count],phase);
    const bool written=biasWriteDenseSnapshot(path,e.Headers[e.frame_count],phase,
        biasDiagnosticContext(e,",\"prior_pose_jacobian_mode\":"+biasJsonQuote(info->exact_pose_jacobian ?
            "exact_residual_chain_rule" : "legacy_frozen_local")+",\"jacobian_evaluated_at_current_state\":true,\"matrix_scope\":\"physical_prior_tangent_before_optimizer_projection\""),
        current_jacobian,residual,info->linearized_residuals,
        blocks,biasFactorySigmas(),&info->linearized_jacobians);biasRecordSnapshot(e,phase,path,written);
}

template<class E> inline void biasDumpMarginalization(E& e, ceres::Problem& problem,
    MarginalizationInfo* info, const std::string& prefix)
{
    if (!info || !info->valid) return;
    std::vector<BiasSnapshotBlock> incoming_blocks,outgoing_blocks;
    for (const auto& item:info->parameter_block_idx)
    {
        double* p=reinterpret_cast<double*>(item.first);const int size=info->parameter_block_size.at(item.first);
        const int local=info->localSize(size);
        incoming_blocks.push_back(biasDescribeBlock(e,problem,p,size,item.second,local,info->parameter_block_data.at(item.first)));
        if (item.second>=info->m)
            outgoing_blocks.push_back(biasDescribeBlock(e,problem,p,size,item.second-info->m,local,info->parameter_block_data.at(item.first)));
    }
    int rows=0;for (auto* f:info->factors) rows+=f->residuals.size();
    Eigen::MatrixXd incoming=Eigen::MatrixXd::Zero(rows,info->m+info->n);Eigen::VectorXd residual(rows);
    int row=0;
    for (auto* f:info->factors)
    {
        residual.segment(row,f->residuals.size())=f->residuals;
        for (int i=0;i<static_cast<int>(f->parameter_blocks.size());++i)
        {
            const long key=reinterpret_cast<long>(f->parameter_blocks[i]);
            incoming.block(row,info->parameter_block_idx.at(key),f->residuals.size(),
                info->localSize(info->parameter_block_size.at(key)))=f->jacobians[i].leftCols(info->localSize(info->parameter_block_size.at(key)));
        }
        row+=f->residuals.size();
    }
    std::ostringstream extra;extra<<",\"matrix_scope\":\"physical_marginalizer_tangent_not_optimizer_basis\",\"marginalized_local_column_count\":"<<info->m
        <<",\"native_normal_equation_eigenvalue_eps\":"<<info->eps<<",\"native_Amm_eigenvalues\":";
    biasWriteVector(extra,info->diagnostic_amm_eigenvalues);extra<<",\"native_Schur_before_clip_eigenvalues\":";
    biasWriteVector(extra,info->diagnostic_schur_eigenvalues);
    extra<<",\"incoming_residual_segments\":[";int segment_row=0;
    for (int i=0;i<static_cast<int>(info->factors.size());++i)
    {
        const auto* f=info->factors[i];if (i) extra<<',';
        extra<<biasResidualSegment(segment_row,f->residuals.size(),f->cost_function,f->loss_function);
        segment_row+=f->residuals.size();
    }
    extra<<']';
    const std::string context=biasDiagnosticContext(e,extra.str());
    for (int output=0;output<2;++output)
    {
        const std::string phase=prefix+(output?"_outgoing_prior":"_incoming_factor_stack");
        const std::string path=biasSnapshotPath(BIAS_DIAGNOSTICS_DIR,e.bias_diagnostic_sequence,e.Headers[e.frame_count],phase);
        const bool written=biasWriteDenseSnapshot(path,e.Headers[e.frame_count],phase,context,
            output?info->linearized_jacobians:incoming,output?info->linearized_residuals:residual,
            info->linearized_residuals,output?outgoing_blocks:incoming_blocks,biasFactorySigmas());
        biasRecordSnapshot(e,phase,path,written);
    }
}
