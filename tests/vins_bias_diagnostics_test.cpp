// Actual native IMU + two free pose/velocity/bias states. No truth or new factor.
#include "bias_information.h"
#include "bias_snapshot.h"
#include "imu_factor.h"
#include "imu_bias_prior.h"
#include "marginalization_factor.h"
#include "stationary_factors.h"
#include <cassert>
#include <iostream>

double ACC_N=.029409182239566, GYR_N=.0029511956263521;
double ACC_W=.094867697353033, GYR_W=.00061324346876081;
Eigen::Vector3d G(0,0,9.81);

int main(int argc,char** argv)
{
    Eigen::Matrix<double,6,1> sigma;sigma<<.196,.196,.196,.0087,.0087,.0087;
    double pose0[7]={0,0,0,0,0,0,1},pose1[7]={0,0,0,0,0,0,1};
    double sb0[9]={},sb1[9]={};
    // Nonzero initial residual checks effective gradient/mean, not only rank.
    for(int i=0;i<6;++i)sb0[i+3]=sb1[i+3]=sigma[i]*.25;
    IntegrationBase integration(Eigen::Vector3d(0,0,9.81),Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero());
    for(int i=0;i<25;++i)integration.push_back(.004,Eigen::Vector3d(0,0,9.81),Eigen::Vector3d::Zero());
    MarginalizationInfo info;info.diagnostics_enabled=true;
    info.addResidualBlockInfo(new ResidualBlockInfo(new ImuBiasPrior(.196,.0087),nullptr,
        {sb0},{0}));
    info.addResidualBlockInfo(new ResidualBlockInfo(new ZeroVelocityFactor(.03),nullptr,{sb0},{0}));
    info.addResidualBlockInfo(new ResidualBlockInfo(new StationaryRelativePositionFactor(.02),nullptr,
        {pose0,pose1},{0}));
    info.addResidualBlockInfo(new ResidualBlockInfo(new IMUFactor(&integration),nullptr,
        {pose0,sb0,pose1,sb1},{0,1}));
    info.preMarginalize();info.marginalize();assert(info.valid);
    int rows=0;for(auto* f:info.factors)rows+=f->residuals.size();
    Eigen::MatrixXd j=Eigen::MatrixXd::Zero(rows,info.m+info.n);Eigen::VectorXd r(rows);
    int row=0;for(auto* f:info.factors){r.segment(row,f->residuals.size())=f->residuals;
        for(int i=0;i<(int)f->parameter_blocks.size();++i){long key=(long)f->parameter_blocks[i];
            j.block(row,info.parameter_block_idx.at(key),f->residuals.size(),
                info.localSize(info.parameter_block_size.at(key)))=f->jacobians[i].leftCols(
                    info.localSize(info.parameter_block_size.at(key)));}row+=f->residuals.size();}
    std::vector<int> target,nuisance;
    const int base=info.parameter_block_idx.at((long)sb1)+3;
    for(int i=0;i<6;++i)target.push_back(base+i);
    for(int i=0;i<j.cols();++i)if(i<base||i>=base+6)nuisance.push_back(i);
    auto incoming=effectiveBiasInformation(j,r,target,nuisance,sigma,1e-6,1e-12,false);
    assert(incoming.valid&&incoming.bias_rank==6);
    std::vector<int> out_target,out_nuisance;
    for(int i:target)out_target.push_back(i-info.m);
    for(int i=0;i<info.n;++i)if(i<base-info.m||i>=base-info.m+6)out_nuisance.push_back(i);
    auto outgoing=effectiveBiasInformation(info.linearized_jacobians,info.linearized_residuals,
        out_target,out_nuisance,sigma,1e-6,1e-12,false);
    assert(outgoing.valid&&outgoing.bias_rank==6);
    assert((incoming.factory_scaled_information-outgoing.factory_scaled_information).norm()<.01);
    assert((incoming.gauss_newton_delta-outgoing.gauss_newton_delta).norm()<.002);
    assert((incoming.gauss_newton_delta+Eigen::Map<Eigen::Matrix<double,6,1>>(sb1+3)).norm()<.002);
    // Actual calibrated blocks are constants: allowing a synthetic fixed
    // calibration column to vary would falsely erase one bias direction.
    Eigen::MatrixXd fixed=Eigen::MatrixXd::Zero(7,7);fixed.topLeftCorner(6,6)=sigma.cwiseInverse().asDiagonal();
    fixed.col(6)=fixed.col(0);Eigen::VectorXd zero=Eigen::VectorXd::Zero(7);
    auto fixed_actual=effectiveBiasInformation(fixed,zero,{0,1,2,3,4,5},{},sigma);
    auto fixed_unrestricted=effectiveBiasInformation(fixed,zero,{0,1,2,3,4,5},{6},sigma);
    assert(fixed_actual.bias_rank==6&&fixed_unrestricted.bias_rank==5);
    // Rank thresholds and normalization are explicit, independently auditable.
    auto normalized=effectiveBiasInformation(j,r,target,nuisance,sigma,1e-8,1e-12,true);
    assert(normalized.valid&&normalized.nuisance_columns_normalized);
    assert(!incoming.nuisance_columns_normalized);
    if(argc>1){std::vector<BiasSnapshotBlock> blocks;
        assert(biasWriteDenseSnapshot(argv[1],.1,"regression_incoming","{}",j,r,
            info.linearized_residuals,blocks,sigma));
        assert(!biasWriteDenseSnapshot(argv[1],.1,"regression_incoming","{}",j,r,
            info.linearized_residuals,blocks,sigma));}
    std::cout<<"PASS: actual full IMU/pose nuisance projection, one prior, OLD, gradient, constants, serialization\n";
}
