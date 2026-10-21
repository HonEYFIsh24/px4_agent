// Existing native prior residual + actual native PosePlus; no new constraint.
#include "marginalization_factor.h"
#include "pose_local_parameterization.h"
#include "fixed_yaw_pose_parameterization.h"
#include <cassert>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>

struct Fixture {
    MarginalizationInfo info;
    std::vector<std::vector<double>> values;
    std::vector<std::string> labels;
    std::vector<bool> constants;
    Eigen::VectorXd expected;
    explicit Fixture(const std::string& path,bool exact):info(exact) {
        std::ifstream input(path);assert(input);int rows,cols,count;input>>rows>>cols>>count;assert(rows==cols);
        info.m=0;info.n=cols;values.resize(count);labels.resize(count);constants.resize(count);
        for(int i=0;i<count;++i){int size,col,constant;input>>size>>col>>constant>>labels[i];constants[i]=constant;
            info.keep_block_size.push_back(size);info.keep_block_idx.push_back(col);
            double* x0=new double[size];for(int k=0;k<size;++k)input>>x0[k];
            info.keep_block_data.push_back(x0);info.parameter_block_data[(long)x0]=x0;
            values[i].resize(size);for(double& x:values[i])input>>x;
        }
        info.linearized_jacobians.resize(rows,cols);
        for(int i=0;i<rows;++i)for(int k=0;k<cols;++k)input>>info.linearized_jacobians(i,k);
        info.linearized_residuals.resize(rows);expected.resize(rows);
        for(int k=0;k<rows;++k)input>>info.linearized_residuals[k];for(int k=0;k<rows;++k)input>>expected[k];assert(input);
    }
    std::vector<double*> addresses(){std::vector<double*> result;for(auto& x:values)result.push_back(x.data());return result;}
    Eigen::VectorXd residual(){auto p=addresses();Eigen::VectorXd r(info.n);MarginalizationFactor f(&info);
        assert(f.Evaluate(p.data(),r.data(),nullptr));return r;}
    Eigen::MatrixXd localJacobian(){auto p=addresses();std::vector<std::vector<double>> storage(values.size());
        std::vector<double*> j(values.size());for(int i=0;i<(int)values.size();++i){storage[i].resize(info.n*values[i].size());j[i]=storage[i].data();}
        Eigen::VectorXd r(info.n);MarginalizationFactor f(&info);assert(f.Evaluate(p.data(),r.data(),j.data()));
        Eigen::MatrixXd result=Eigen::MatrixXd::Zero(info.n,info.n);
        for(int i=0;i<(int)values.size();++i){int size=values[i].size(),local=info.localSize(size);
            Eigen::Map<Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>> block(j[i],info.n,size);
            result.middleCols(info.keep_block_idx[i],local)=block.leftCols(local);
            if(size==7)assert(block.col(6).norm()==0.);}
        return result;}
    Eigen::MatrixXd finiteDifference(double h){Eigen::MatrixXd result=Eigen::MatrixXd::Zero(info.n,info.n);
        PoseLocalParameterization parameterization;const ceres::LocalParameterization& plus=parameterization;
        for(int i=0;i<(int)values.size();++i){int size=values[i].size(),local=info.localSize(size),col=info.keep_block_idx[i];auto original=values[i];
            for(int axis=0;axis<local;++axis){Eigen::VectorXd delta=Eigen::VectorXd::Zero(local);delta[axis]=h;
                if(size==7)plus.Plus(original.data(),delta.data(),values[i].data());else values[i][axis]=original[axis]+h;
                Eigen::VectorXd up=residual();values[i]=original;delta[axis]=-h;
                if(size==7)plus.Plus(original.data(),delta.data(),values[i].data());else values[i][axis]=original[axis]-h;
                Eigen::VectorXd down=residual();result.col(col+axis)=(up-down)/(2*h);values[i]=original;}}
        return result;}
    Eigen::VectorXd yawDirection(){Eigen::VectorXd result=Eigen::VectorXd::Zero(info.n);Eigen::Vector3d z=Eigen::Vector3d::UnitZ();
        for(int i=0;i<(int)values.size();++i){if(constants[i])continue;int c=info.keep_block_idx[i];
            if(labels[i].find("pose_")==0){Eigen::Map<Eigen::Vector3d> p(values[i].data());Eigen::Map<Eigen::Quaterniond> q(values[i].data()+3);
                result.segment<3>(c)=z.cross(p);result.segment<3>(c+3)=q.inverse()*z;}
            if(labels[i].find("speed_bias_")==0)result.segment<3>(c)=z.cross(Eigen::Map<Eigen::Vector3d>(values[i].data()));}
        return result;}
    double yawCost(double angle){auto original=values;Eigen::Quaterniond dq(Eigen::AngleAxisd(angle,Eigen::Vector3d::UnitZ()));
        for(int i=0;i<(int)values.size();++i){if(constants[i])continue;
            if(labels[i].find("pose_")==0){Eigen::Map<Eigen::Vector3d> p(values[i].data());Eigen::Map<Eigen::Quaterniond> q(values[i].data()+3);p=dq*p;q=(dq*q).normalized();}
            if(labels[i].find("speed_bias_")==0){Eigen::Map<Eigen::Vector3d> v(values[i].data());v=dq*v;}}
        double cost=.5*residual().squaredNorm();values=original;return cost;}
};

void captured(const std::string& file){Fixture legacy(file,false),exact(file,true);
    const Eigen::MatrixXd frozen=exact.info.linearized_jacobians;
    assert((exact.residual()-legacy.residual()).norm()==0.);
    assert((exact.residual()-exact.expected).norm()<1e-8);
    assert((legacy.localJacobian()-legacy.info.linearized_jacobians).norm()==0.);
    const auto j=exact.localJacobian();double maximum=0;
    for(double h:{1e-5,1e-6,1e-7}){auto fd=exact.finiteDifference(h);double err=(j-fd).norm()/std::max(1.,j.norm());maximum=std::max(maximum,err);assert(err<2e-7);}
    assert((exact.info.linearized_jacobians-frozen).norm()==0.);
    // The actual prior, via custom Euler RP Plus, uses Jphysical6 * B6x2.
    for(int i=0;i<(int)exact.values.size();++i)if(exact.labels[i]=="pose_0"){
        auto original=exact.values[i];FixedYawPoseParameterization chart(original.data());
        const Eigen::MatrixXd expected=j.middleCols(exact.info.keep_block_idx[i],6)*fixedYawPoseBodyTangentMap(original.data())*FIXED_YAW_RP_RADIANS_PER_LOCAL_UNIT;
        for(int axis=0;axis<2;++axis){double delta[2]={};delta[axis]=1e-3;
            assert(chart.Plus(original.data(),delta,exact.values[i].data()));Eigen::VectorXd up=exact.residual();
            delta[axis]=-1e-3;assert(chart.Plus(original.data(),delta,exact.values[i].data()));Eigen::VectorXd down=exact.residual();
            exact.values[i]=original;assert(((up-down)/2e-3-expected.col(axis)).norm()/std::max(1.,expected.col(axis).norm())<2e-7);}}
    // Exact derivative is only an invertible change of pose-nuisance basis at
    // this point. Bias columns stay identical, so it cannot add bias information.
    Eigen::MatrixXd transform=Eigen::MatrixXd::Identity(exact.info.n,exact.info.n);
    for(int i=0;i<(int)exact.values.size();++i)if(exact.values[i].size()==7){
        Eigen::Map<const Eigen::Quaterniond> q0(exact.info.keep_block_data[i]+3),q(exact.values[i].data()+3);
        Eigen::Quaterniond relative=q0.inverse()*q;double sign=relative.w()>=0?1.:-1.;
        Eigen::Matrix3d d=sign*(relative.w()*Eigen::Matrix3d::Identity()+Utility::skewSymmetric(relative.vec()));
        assert(std::abs(d.determinant())>1e-6);transform.block<3,3>(exact.info.keep_block_idx[i]+3,exact.info.keep_block_idx[i]+3)=d;}
    assert((j*transform.inverse()-legacy.localJacobian()).norm()/std::max(1.,j.norm())<1e-10);
    const auto r=exact.residual(),direction=exact.yawDirection();double declared=(j*direction).dot(r);
    double fd=(exact.yawCost(1e-6)-exact.yawCost(-1e-6))/(2e-6);
    assert(std::abs(fd-declared)<1e-5*std::max(1.,std::abs(fd)));
    double old=(legacy.localJacobian()*direction).dot(r);
    std::cout<<std::setprecision(17)<<"captured exact_gradient="<<declared<<" FD="<<fd<<" legacy_gradient="<<old<<" relative_J_FD_error="<<maximum<<"\n";
    // All translations and non-pose blocks retain identical derivatives.
    for(int i=0;i<(int)exact.values.size();++i){int col=exact.info.keep_block_idx[i];int size=exact.values[i].size();
        int unchanged=size==7?3:size;assert((j.middleCols(col,unchanged)-legacy.localJacobian().middleCols(col,unchanged)).norm()==0.);}
    // Hemisphere-equivalent quaternion representations give identical r and J.
    for(int i=0;i<(int)exact.values.size();++i)if(exact.values[i].size()==7)
        for(int k=3;k<7;++k)exact.values[i][k]*=-1;
    assert((exact.residual()-r).norm()<1e-9);assert((exact.localJacobian()-j).norm()<1e-9);
    // Near the ±180° chart cut, use increments safely inside one hemisphere.
    for(double degrees:{0.,16.,-16.,179.,-179.}){int i=0;while(exact.values[i].size()!=7)++i;
        Eigen::Map<const Eigen::Quaterniond> q0(exact.info.keep_block_data[i]+3);Eigen::Map<Eigen::Quaterniond> q(exact.values[i].data()+3);
        q=(q0*Eigen::Quaterniond(Eigen::AngleAxisd(degrees*M_PI/180.,Eigen::Vector3d(1,2,3).normalized()))).normalized();
        auto a=exact.localJacobian(),b=exact.finiteDifference(1e-6);assert((a-b).norm()/std::max(1.,a.norm())<2e-7);}
    int cut=0;while(exact.values[cut].size()!=7)++cut;
    Eigen::Map<const Eigen::Quaterniond> cut_q0(exact.info.keep_block_data[cut]+3);
    Eigen::Map<Eigen::Quaterniond> cut_q(exact.values[cut].data()+3);
    cut_q=(cut_q0*Eigen::Quaterniond(Eigen::AngleAxisd(M_PI-1e-4,Eigen::Vector3d::UnitX()))).normalized();
    Eigen::VectorXd one_side=exact.residual();
    cut_q=(cut_q0*Eigen::Quaterniond(Eigen::AngleAxisd(M_PI+1e-4,Eigen::Vector3d::UnitX()))).normalized();
    assert((one_side-exact.residual()).norm()>3.99*frozen.col(exact.info.keep_block_idx[cut]+3).norm());
    std::cout<<"PASS: exact native prior FD, legacy identity, unchanged residual/prior, hemisphere/near-cut\n";
}
int main(int argc,char** argv){assert(argc>1);for(int i=1;i<argc;++i)captured(argv[i]);
    std::cout<<"180-degree hemisphere cut is mathematically nondifferentiable; no FD across/smoothing tested\n";}
