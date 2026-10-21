// Regression based on the independent constant-measurement negative case.
// Actual factors/free poses, known synthetic stationary support; no ROS/truth feed.
#include "integration_base.h"
#include "imu_factor.h"
#include "imu_bias_prior.h"
#include "marginalization_factor.h"
#include "pose_local_parameterization.h"
#include "fixed_yaw_pose_parameterization.h"
#include "stationary_factors.h"
#include "projectionTwoFrameOneCamFactor.h"
#include "projectionTwoFrameTwoCamFactor.h"
#include "projectionOneFrameTwoCamFactor.h"
#include <cassert>
#include <fstream>
#include <iomanip>
#include <memory>

constexpr int N = 11, FEATURES = 8;
const Vector3d measured_acc(-.173961716717,-.15441814047,9.92683909375);
const Vector3d measured_gyro(-.00102485092165,.00560275018514,.0000643927487078);
const Vector2d zero2=Vector2d::Zero();

struct Experiment {
    double pose[N][7]{}, sb[N][9]{}, ex[2][7]{}, inv_depth[FEATURES]{}, td[1]{};
    Vector3d left[FEATURES], right[FEATURES];
    std::unique_ptr<IntegrationBase> imu[N];
    MarginalizationInfo* prior=nullptr;
    std::vector<double*> prior_blocks;
    bool stereo;
    std::string name;

    Experiment(std::string name_,bool wrong,bool stereo_):stereo(stereo_),name(name_) {
        Matrix3d calibration;
        calibration << 0,-sin(.35),cos(.35), -1,0,0, 0,-cos(.35),-sin(.35);
        Quaterniond qe(calibration);
        for(int c=0;c<2;c++) {
            ex[c][0]=.12; ex[c][1]=c ? -.06 : .06; ex[c][2]=.16;
            ex[c][3]=qe.x(); ex[c][4]=qe.y(); ex[c][5]=qe.z(); ex[c][6]=qe.w();
        }
        Quaterniond q=wrong ? Quaterniond(.995003663463,-.0330686333961,.0941509366215,.00312670964989)
            : Quaterniond::FromTwoVectors(measured_acc,G);
        q.normalize();
        Vector3d ba=Vector3d::Zero(); if(wrong)ba=measured_acc-q.inverse()*G;
        for(int i=0;i<N;i++) {
            pose[i][3]=q.x(); pose[i][4]=q.y(); pose[i][5]=q.z(); pose[i][6]=q.w();
            Eigen::Map<Vector3d>(sb[i]+3)=ba;
            Eigen::Map<Vector3d>(sb[i]+6)=measured_gyro;
            if(i) buildImu(i,ba,measured_gyro);
        }
        for(int f=0;f<FEATURES;f++) {
            double depth=.8+.45*f;
            left[f]=Vector3d(-.3+.09*f,-.2+.07*(f%4),1);
            Vector3d xyz=calibration.transpose()*(calibration*(depth*left[f])+Vector3d(0,.12,0));
            right[f]=xyz/xyz.z(); inv_depth[f]=1/depth;
        }
    }
    ~Experiment(){delete prior;}
    void buildImu(int i,const Vector3d& ba,const Vector3d& bg) {
        imu[i].reset(new IntegrationBase(measured_acc,measured_gyro,ba,bg));
        for(int k=0;k<25;k++) imu[i]->push_back(.004,measured_acc,measured_gyro);
    }
    void visualProblem(ceres::Problem& p) {
        if(!stereo) return;
        for(int c=0;c<2;c++) {
            p.AddParameterBlock(ex[c],7,new PoseLocalParameterization());
            p.SetParameterBlockConstant(ex[c]);
        }
        p.AddParameterBlock(td,1); p.SetParameterBlockConstant(td);
        auto* loss=new ceres::HuberLoss(1.0);
        for(int f=0;f<FEATURES;f++) {
            p.AddResidualBlock(new ProjectionOneFrameTwoCamFactor(left[f],right[f],zero2,zero2,0,0),
                loss,ex[0],ex[1],inv_depth+f,td);
            for(int j=1;j<N;j++) {
                p.AddResidualBlock(new ProjectionTwoFrameOneCamFactor(left[f],left[f],zero2,zero2,0,0),
                    loss,pose[0],pose[j],ex[0],inv_depth+f,td);
                p.AddResidualBlock(new ProjectionTwoFrameTwoCamFactor(left[f],right[f],zero2,zero2,0,0),
                    loss,pose[0],pose[j],ex[0],ex[1],inv_depth+f,td);
            }
        }
    }
    void addProblem(ceres::Problem& p) {
        for(int i=0;i<N;i++) {
            p.AddParameterBlock(pose[i],7,i==0 ? static_cast<ceres::LocalParameterization*>(new FixedYawPoseParameterization(pose[i])) : static_cast<ceres::LocalParameterization*>(new PoseLocalParameterization()));
            p.AddParameterBlock(sb[i],9);
            p.AddResidualBlock(new ZeroVelocityFactor(.03),nullptr,sb[i]);
            if(i) {
                p.AddResidualBlock(new StationaryRelativePositionFactor(.02),nullptr,pose[i-1],pose[i]);
                p.AddResidualBlock(new IMUFactor(imu[i].get()),nullptr,pose[i-1],sb[i-1],pose[i],sb[i]);
            }
        }
        visualProblem(p);
        if(prior) p.AddResidualBlock(new MarginalizationFactor(prior),nullptr,prior_blocks);
        else p.AddResidualBlock(new ImuBiasPrior(.196,.0087),nullptr,sb[0]);
    }
    double tilt()const {
        Quaterniond q(pose[N-1][6],pose[N-1][3],pose[N-1][4],pose[N-1][5]);
        return acos(std::max(-1.,std::min(1.,(q*Vector3d::UnitZ()).z())))*180/M_PI;
    }
    void solve(int step) {
        ceres::Problem p; addProblem(p);
        assert(p.ParameterBlockLocalSize(pose[0])==2);
        for(int i=1;i<N;++i)assert(p.ParameterBlockLocalSize(pose[i])==6);
        if(step==0 || step==1){
            ceres::Problem::EvaluateOptions evaluate;evaluate.apply_loss_function=false;
            evaluate.parameter_blocks={pose[0]};p.GetResidualBlocks(&evaluate.residual_blocks);
            std::vector<double> r;ceres::CRSMatrix sparse;double cost;
            assert(p.Evaluate(evaluate,&cost,&r,nullptr,&sparse));assert(sparse.num_cols==2);
            MatrixXd actual=MatrixXd::Zero(sparse.num_rows,2);
            for(int row=0;row<sparse.num_rows;++row)for(int k=sparse.rows[row];k<sparse.rows[row+1];++k)actual(row,sparse.cols[k])=sparse.values[k];
            double original[7];std::copy(pose[0],pose[0]+7,original);FixedYawPoseParameterization chart(original);
            for(int axis=0;axis<2;++axis){double delta[2]={};delta[axis]=1e-3;assert(chart.Plus(original,delta,pose[0]));
                std::vector<double> up,down;assert(p.Evaluate(evaluate,nullptr,&up,nullptr,nullptr));delta[axis]=-1e-3;
                assert(chart.Plus(original,delta,pose[0]));assert(p.Evaluate(evaluate,nullptr,&down,nullptr,nullptr));std::copy(original,original+7,pose[0]);
                VectorXd fd(sparse.num_rows);for(int row=0;row<sparse.num_rows;++row)fd[row]=(up[row]-down[row])/2e-3;
                assert((fd-actual.col(axis)).norm()/std::max(1.,actual.col(axis).norm())<2e-7);}
        }

        ceres::Solver::Options opts;
        opts.linear_solver_type=ceres::DENSE_SCHUR;
        opts.trust_region_strategy_type=ceres::DOGLEG;
        opts.max_num_iterations=100; opts.max_solver_time_in_seconds=5;
        opts.function_tolerance=1e-12; opts.gradient_tolerance=1e-10; opts.parameter_tolerance=1e-12;
        std::vector<double> probe_gradient;ceres::Problem::EvaluateOptions probe;probe.parameter_blocks={pose[0]};
        assert(p.Evaluate(probe,nullptr,nullptr,&probe_gradient,nullptr));
        std::cerr<<"initial RP gradient step="<<step<<" "<<probe_gradient[0]<<" "<<probe_gradient[1]<<"\n";
        ceres::Solver::Summary result; ceres::Solve(opts,&p,&result);
        if(!result.IsSolutionUsable())throw std::runtime_error("unusable solve");
    }
    void oldMarginalizeAndShift(int step) {
        auto* next=new MarginalizationInfo(true);
        if(prior) {
            std::vector<int> drop;
            for(int i=0;i<(int)prior_blocks.size();i++)if(prior_blocks[i]==pose[0]||prior_blocks[i]==sb[0])drop.push_back(i);
            next->addResidualBlockInfo(new ResidualBlockInfo(new MarginalizationFactor(prior),nullptr,prior_blocks,drop));
        } else next->addResidualBlockInfo(new ResidualBlockInfo(new ImuBiasPrior(.196,.0087),nullptr,{sb[0]},{0}));
        next->addResidualBlockInfo(new ResidualBlockInfo(new ZeroVelocityFactor(.03),nullptr,{sb[0]},{0}));
        next->addResidualBlockInfo(new ResidualBlockInfo(new StationaryRelativePositionFactor(.02),nullptr,{pose[0],pose[1]},{0}));
        next->addResidualBlockInfo(new ResidualBlockInfo(new IMUFactor(imu[1].get()),nullptr,{pose[0],sb[0],pose[1],sb[1]},{0,1}));
        if(stereo) {
            auto* loss=new ceres::HuberLoss(1.0);
            for(int f=0;f<FEATURES;f++) {
                next->addResidualBlockInfo(new ResidualBlockInfo(new ProjectionOneFrameTwoCamFactor(left[f],right[f],zero2,zero2,0,0),loss,{ex[0],ex[1],inv_depth+f,td},{2}));
                for(int j=1;j<N;j++) {
                    next->addResidualBlockInfo(new ResidualBlockInfo(new ProjectionTwoFrameOneCamFactor(left[f],left[f],zero2,zero2,0,0),loss,{pose[0],pose[j],ex[0],inv_depth+f,td},{0,3}));
                    next->addResidualBlockInfo(new ResidualBlockInfo(new ProjectionTwoFrameTwoCamFactor(left[f],right[f],zero2,zero2,0,0),loss,{pose[0],pose[j],ex[0],ex[1],inv_depth+f,td},{0,4}));
                }
            }
        }
        next->preMarginalize();
        next->marginalize();
        if(!next->valid||!next->linearized_jacobians.allFinite()||!next->linearized_residuals.allFinite())
            throw std::runtime_error("invalid marginalization");
        std::unordered_map<long,double*> shift;
        for(int i=1;i<N;i++) {shift[(long)pose[i]]=pose[i-1];shift[(long)sb[i]]=sb[i-1];}
        for(int c=0;c<2;c++)shift[(long)ex[c]]=ex[c];shift[(long)td]=td;
        prior_blocks=next->getParameterBlocks(shift);
        delete prior;prior=next;
        for(int i=0;i<N-1;i++) {
            std::copy(pose[i+1],pose[i+1]+7,pose[i]);
            std::copy(sb[i+1],sb[i+1]+9,sb[i]);
            if(i>0)imu[i]=std::move(imu[i+1]);
        }
        std::copy(pose[N-2],pose[N-2]+7,pose[N-1]);
        std::copy(sb[N-2],sb[N-2]+9,sb[N-1]);
        buildImu(N-1,Vector3d(sb[N-1][3],sb[N-1][4],sb[N-1][5]),Vector3d(sb[N-1][6],sb[N-1][7],sb[N-1][8]));
    }
};


void relative_heading_test(bool imu_predicted_initialization) {
    double pose[2][7]{},sb[2][9]{},ex[2][7]{},inverse_depth[8]{},td[1]{};
    Matrix3d calibration;calibration<<0,-sin(.35),cos(.35),-1,0,0,0,-cos(.35),-sin(.35);
    Quaterniond qe(calibration),initial=Quaterniond::FromTwoVectors(measured_acc,G);
    // Synthetic physical movement: constant world acceleration and constant
    // right-body angular rate. All IMU samples are generated at their real dt.
    const Vector3d omega(.05,-.04,.2),acceleration(.2,-.1,.08);
    const Vector3d true_ba(.02,-.01,.03),true_bg=measured_gyro;
    const Vector3d target_position=.5*acceleration;
    auto orientation=[&](double t){return Quaterniond(initial*Quaterniond(Eigen::AngleAxisd(t*omega.norm(),omega.normalized())));};
    Quaterniond target=orientation(1.);
    for(int i=0;i<2;++i){pose[i][3]=initial.x();pose[i][4]=initial.y();pose[i][5]=initial.z();pose[i][6]=initial.w();
        Eigen::Map<Vector3d>(sb[i]+6)=measured_gyro;ex[i][0]=.12;ex[i][1]=i?-.06:.06;ex[i][2]=.16;
        ex[i][3]=qe.x();ex[i][4]=qe.y();ex[i][5]=qe.z();ex[i][6]=qe.w();}
    IntegrationBase imu(initial.inverse()*(G+acceleration)+true_ba,true_bg+omega,Vector3d::Zero(),true_bg);
    for(int k=1;k<=250;++k)imu.push_back(.004,orientation(k*.004).inverse()*(G+acceleration)+true_ba,true_bg+omega);
    // Initialize the new frame by genuine measured IMU prediction, matching
    // VINS' normal initialization; starting it frozen with a 1-second turn
    // caused Ceres' raw-gradient chart probe to fail (preserved negative case).
    const Vector3d predicted_position=initial*imu.delta_p-.5*G;
    const Vector3d predicted_velocity=initial*imu.delta_v-G;
    Quaterniond predicted_orientation=initial*imu.delta_q;
    if(imu_predicted_initialization){
        Eigen::Map<Vector3d> seeded_position(pose[1]);seeded_position=predicted_position;
        Eigen::Map<Quaterniond>(pose[1]+3)=predicted_orientation;
        Eigen::Map<Vector3d> seeded_velocity(sb[1]);seeded_velocity=predicted_velocity;}
    const Vector3d initial_position(pose[1][0],pose[1][1],pose[1][2]);
    ceres::Problem problem;
    for(int i=0;i<2;++i){problem.AddParameterBlock(pose[i],7,i==0 ? static_cast<ceres::LocalParameterization*>(new FixedYawPoseParameterization(pose[i])) : static_cast<ceres::LocalParameterization*>(new PoseLocalParameterization()));problem.AddParameterBlock(sb[i],9);
        problem.AddParameterBlock(ex[i],7,new PoseLocalParameterization());problem.SetParameterBlockConstant(ex[i]);}
    problem.AddParameterBlock(td,1);problem.SetParameterBlockConstant(td);
    problem.AddResidualBlock(new ImuBiasPrior(.196,.0087),nullptr,sb[0]);
    problem.AddResidualBlock(new IMUFactor(&imu),nullptr,pose[0],sb[0],pose[1],sb[1]);
    for(int f=0;f<8;++f){double depth=.8+.45*f;Vector3d first(-.3+.09*f,-.2+.07*(f%4),1);
        Vector3d world=initial*(calibration*(depth*first)+Vector3d(ex[0][0],ex[0][1],ex[0][2]));
        auto pixel=[&](const Quaterniond& body,const Vector3d& position,int camera){Vector3d p=calibration.transpose()*(body.inverse()*(world-position)-
            Vector3d(ex[camera][0],ex[camera][1],ex[camera][2]));return Vector3d(p/p.z());};
        Vector3d right0=pixel(initial,Vector3d::Zero(),1),left1=pixel(target,target_position,0),right1=pixel(target,target_position,1);inverse_depth[f]=1/depth;
        problem.AddResidualBlock(new ProjectionOneFrameTwoCamFactor(first,right0,zero2,zero2,0,0),nullptr,ex[0],ex[1],inverse_depth+f,td);
        problem.AddResidualBlock(new ProjectionTwoFrameOneCamFactor(first,left1,zero2,zero2,0,0),nullptr,pose[0],pose[1],ex[0],inverse_depth+f,td);
        problem.AddResidualBlock(new ProjectionTwoFrameTwoCamFactor(first,right1,zero2,zero2,0,0),nullptr,pose[0],pose[1],ex[0],ex[1],inverse_depth+f,td);
    }
    // Only first XYZ/yaw are gauge-fixed; its RP2 and later pose6 are free. No
    // absolute roll/pitch/yaw/position factor is added. Relative pixels + IMU
    // observe relative motion; the chosen initial global yaw only defines coordinates.
    for(int i=0;i<2;++i)assert(problem.ParameterBlockLocalSize(pose[i])==(i==0?2:6)&&!problem.IsParameterBlockConstant(pose[i]));
    ceres::Solver::Options options;options.linear_solver_type=ceres::DENSE_SCHUR;options.max_num_iterations=100;options.max_solver_time_in_seconds=5;
    std::vector<double> gradient;ceres::Problem::EvaluateOptions evaluation;evaluation.parameter_blocks={pose[0]};
    assert(problem.Evaluate(evaluation,nullptr,nullptr,&gradient,nullptr));std::cerr<<"motion initial RP local gradient "<<gradient[0]<<" "<<gradient[1]<<"\n";
    ceres::Solver::Summary result;ceres::Solve(options,&problem,&result);assert(result.IsSolutionUsable());
    Quaterniond q0(pose[0][6],pose[0][3],pose[0][4],pose[0][5]),q1(pose[1][6],pose[1][3],pose[1][4],pose[1][5]);
    double relative_yaw=Utility::R2ypr(q1.toRotationMatrix()).x()-Utility::R2ypr(q0.toRotationMatrix()).x();
    relative_yaw=std::atan2(std::sin(relative_yaw*M_PI/180),std::cos(relative_yaw*M_PI/180));
    const double expected_yaw=(Utility::R2ypr(target.toRotationMatrix()).x()-Utility::R2ypr(initial.toRotationMatrix()).x())*M_PI/180;
    assert(std::abs(relative_yaw-expected_yaw)<.003);
    assert((q0.inverse()*q1).angularDistance(initial.inverse()*target)<.003);
    const Vector3d observed_translation=q0.inverse()*(Vector3d(pose[1][0],pose[1][1],pose[1][2])-Vector3d(pose[0][0],pose[0][1],pose[0][2]));
    assert((observed_translation-initial.inverse()*target_position).norm()<.005);
    assert(observed_translation.norm()>.1);
    assert((Vector3d(pose[1][0],pose[1][1],pose[1][2])-initial_position).norm()>.003);
    assert((q0.inverse()*q1).vec().head<2>().norm()>.02);
    assert(Vector3d(pose[0][0],pose[0][1],pose[0][2]).norm()<1e-12);
    std::cout<<"PASS: relative heading motion observed with first RP2 and subsequent6 free poses: "<<(imu_predicted_initialization?"IMU predicted seed ":"V7 original frozen seed ")<<relative_yaw<<" rad, translation "<<observed_translation.transpose()<<" m, relative RP free\n";
}
int main() {
    ACC_N=.029409182239566;GYR_N=.0029511956263521;ACC_W=.094867697353033;GYR_W=.00061324346876081;G=Vector3d(0,0,9.8066);
    ProjectionTwoFrameOneCamFactor::sqrt_info=Matrix2d::Identity()*(460./1.5);
    ProjectionTwoFrameTwoCamFactor::sqrt_info=Matrix2d::Identity()*(460./1.5);
    ProjectionOneFrameTwoCamFactor::sqrt_info=Matrix2d::Identity()*(460./1.5);
    Experiment e("wrong_tilt_exact",true,true);double before=e.tilt();e.solve(0);
    assert(before>10&&e.tilt()<2);
    for(int step=1;step<=10;++step){e.oldMarginalizeAndShift(step);assert(e.prior->exact_pose_jacobian);e.solve(step);assert(e.tilt()<2);}
    std::cout<<"PASS: wrong tilt "<<before<<" -> "<<e.tilt()<<" deg with free RP and once-prior through ten OLD\n";
    relative_heading_test(false);relative_heading_test(true);
}
