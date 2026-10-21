#include "fixed_yaw_pose_parameterization.h"
#include "../estimator/parameters.h"
#include "marginalization_factor.h"
#include "pose_local_parameterization.h"
#include "bias_native_diagnostics.h"
#include <cassert>
#include <iostream>
#include <limits>

struct SnapshotWindow {
    double para_Pose[WINDOW_SIZE+1][7]{},para_SpeedBias[WINDOW_SIZE+1][9]{},para_Ex_Pose[2][7]{},para_Td[1][1]{},para_Feature[NUM_OF_F][1]{};
    double Headers[WINDOW_SIZE+1]{};
};

void math_case(double yaw,double pitch,double roll,double sign)
{
    Eigen::Quaterniond q=Eigen::Quaterniond(Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ()))
        *Eigen::Quaterniond(Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitY()))
        *Eigen::Quaterniond(Eigen::AngleAxisd(roll,Eigen::Vector3d::UnitX()));q.coeffs()*=sign;
    double pose[7]={1,2,3,q.x(),q.y(),q.z(),q.w()},out[7];
    double expected_pitch=pitch,expected_roll=roll;
    FixedYawPoseParameterization chart(pose);double zero[2]={};assert(chart.Plus(pose,zero,out));
    for(int i=0;i<7;++i)assert(std::abs(pose[i]-out[i])<1e-12);
    Eigen::Matrix<double,7,2,Eigen::RowMajor> declared;assert(chart.ComputeJacobian(pose,declared.data()));
    assert(declared.topRows<3>().norm()==0&&declared.row(6).norm()==0);
    const Eigen::Matrix<double,6,2> body=fixedYawPoseBodyTangentMap(pose)*FIXED_YAW_RP_RADIANS_PER_LOCAL_UNIT;assert((declared.topRows<6>()-body).norm()==0);
    for(int axis=0;axis<2;++axis){double delta[2]={},up[7],down[7];double h=1e-3; // 1e-6 physical rad in milliradian local units
        delta[axis]=h;
        assert(chart.Plus(pose,delta,up));delta[axis]=-h;assert(chart.Plus(pose,delta,down));
        Eigen::Quaterniond uq(up[6],up[3],up[4],up[5]),dq(down[6],down[3],down[4],down[5]);
        Eigen::Vector3d right=(2*(q.inverse()*uq).vec()-2*(q.inverse()*dq).vec())/(2*h);
        assert((right-body.block<3,1>(3,axis)).norm()<2e-9);}
    for(int i=0;i<100;++i){double delta[2]={.2,-.1};
        expected_pitch+=fixedYawPitchRetractionStep(expected_pitch,delta[1]);expected_roll+=fixedYawRollRetractionStep(delta[0]);
        assert(chart.Plus(pose,delta,out));
        std::copy(out,out+7,pose);double actual_yaw,actual_pitch,actual_roll;assert(fixedYawPoseZyx(pose,actual_yaw,actual_pitch,actual_roll));
        assert(pose[0]==1&&pose[1]==2&&pose[2]==3);
        assert(std::abs(std::atan2(std::sin(actual_yaw-yaw),std::cos(actual_yaw-yaw)))<1e-12);}
    double actual_yaw,actual_pitch,actual_roll;assert(fixedYawPoseZyx(pose,actual_yaw,actual_pitch,actual_roll));
    assert(std::abs(actual_pitch-expected_pitch)<1e-10);assert(std::abs(actual_roll-expected_roll)<1e-10);
    assert(std::abs(actual_pitch-pitch)>.009);assert(std::abs(actual_roll-roll)>.019);
    // Out-of-manifold input must fail instead of silently projecting XYZ/yaw.
    pose[0]+=.001;assert(!chart.Plus(pose,zero,out));pose[0]-=.001;
    Eigen::Map<Eigen::Quaterniond> current(pose+3);current=Eigen::Quaterniond(Eigen::AngleAxisd(.001,Eigen::Vector3d::UnitZ()))*current;
    assert(!chart.Plus(pose,zero,out));
}

void huge_probe_case(double yaw,double pitch,double roll,double sign)
{
    Eigen::Quaterniond q=Eigen::Quaterniond(Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ()))
        *Eigen::Quaterniond(Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitY()))
        *Eigen::Quaterniond(Eigen::AngleAxisd(roll,Eigen::Vector3d::UnitX()));q.coeffs()*=sign;
    double pose[7]={1,2,3,q.x(),q.y(),q.z(),q.w()},out[7];FixedYawPoseParameterization chart(pose);
    double a,b,c;assert(fixedYawPoseZyx(pose,a,b,c));
    for(double dr:{-std::numeric_limits<double>::max(),-1e308,0.,2*M_PI/FIXED_YAW_RP_RADIANS_PER_LOCAL_UNIT,1e308,std::numeric_limits<double>::max()})
        for(double dp:{-std::numeric_limits<double>::max(),-1e308,0.,1e308,std::numeric_limits<double>::max()}){
            double delta[2]={dr,dp};assert(chart.Plus(pose,delta,out));
            for(double v:out)assert(std::isfinite(v));assert(fixedYawPoseZyx(out,a,b,c));
            assert(std::abs(b)<FIXED_YAW_PITCH_CHART_LIMIT_RAD);
            assert(out[0]==1&&out[1]==2&&out[2]==3);
            assert(std::abs(std::atan2(std::sin(a-yaw),std::cos(a-yaw)))<1e-10);
            Eigen::Quaterniond nq(out[6],out[3],out[4],out[5]);assert(std::abs(nq.norm()-1)<1e-14);
            if(dp==0 && dr!=0)assert(q.angularDistance(nq)>.01); // no 2pi identity probe
        }
    // Nonfinite input delta remains an explicit failure; never mask bad source.
    double delta[2]={std::numeric_limits<double>::infinity(),0};assert(!chart.Plus(pose,delta,out));
    delta[0]=std::numeric_limits<double>::quiet_NaN();assert(!chart.Plus(pose,delta,out));
}

int main(int argc,char** argv){
    for(bool imu:{false,true})for(bool fixed:{false,true}){
        assert(biasFreeGlobalCoordinateGaugeDimensions(imu,fixed)==(imu&&!fixed?4:0));
        assert(biasPhysicalGlobalGaugeQuotientApplicable(imu,fixed)==(imu&&!fixed));}

    for(double sign:{1.,-1.}){math_case(.7,-.35,.4,sign);math_case(179*M_PI/180.,.6,-.5,sign);math_case(-179*M_PI/180.,-1.1,.2,sign);}
    Eigen::Quaterniond singular(Eigen::AngleAxisd(89.5*M_PI/180.,Eigen::Vector3d::UnitY()));
    double pose[7]={0,0,0,singular.x(),singular.y(),singular.z(),singular.w()},out[7],zero[2]={};
    FixedYawPoseParameterization chart(pose);assert(!chart.Plus(pose,zero,out));Eigen::Matrix<double,7,2,Eigen::RowMajor> j;assert(!chart.ComputeJacobian(pose,j.data()));
    // No clamping: failed Plus leaves output untouched.
    std::fill(out,out+7,42);assert(!chart.Plus(pose,zero,out));for(double x:out)assert(x==42);
    for(double sign:{-1.,1.})for(double yaw:{179*M_PI/180.,-179*M_PI/180.})
        for(double pitch:{0.,.8,-.8,FIXED_YAW_PITCH_CHART_LIMIT_RAD-1e-8,
            -FIXED_YAW_PITCH_CHART_LIMIT_RAD+1e-8,FIXED_YAW_PITCH_CHART_LIMIT_RAD-1e-12,
            -FIXED_YAW_PITCH_CHART_LIMIT_RAD+1e-12})huge_probe_case(yaw,pitch,.4,sign);
    double ordinary[7]={0,0,0,0,0,0,1},large_delta[2]={0,2000};
    FixedYawPoseParameterization ordinary_chart(ordinary);assert(ordinary_chart.Plus(ordinary,large_delta,out));
    double yaw,pitch,roll;assert(fixedYawPoseZyx(out,yaw,pitch,roll));
    assert(pitch>0&&pitch<FIXED_YAW_PITCH_CHART_LIMIT_RAD/8);
    if(argc>1){
        SnapshotWindow window;window.para_Pose[0][6]=window.para_Pose[1][6]=window.para_Ex_Pose[0][6]=1;NUM_OF_CAM=1;
        ceres::Problem problem;
        problem.AddParameterBlock(window.para_Pose[0],7,new FixedYawPoseParameterization(window.para_Pose[0]));
        problem.AddParameterBlock(window.para_Pose[1],7,new PoseLocalParameterization());
        problem.AddParameterBlock(window.para_Ex_Pose[0],7,new PoseLocalParameterization());problem.SetParameterBlockConstant(window.para_Ex_Pose[0]);
        // Actual optimizer CRS description is local2; prior/margin description
        // remains physical6 and must carry the explicit scaled map.
        auto optimizer=biasDescribeBlock(window,problem,window.para_Pose[0],7,0,2);
        assert(optimizer.matrix_local_basis.find("milliradians")!=std::string::npos);
        assert(optimizer.optimizer_to_matrix_local.isIdentity());
        auto b=biasDescribeBlock(window,problem,window.para_Pose[0],7,0,6);
        assert(b.actual_optimizer_local_size==2&&b.local_size==6);
        assert((b.optimizer_to_matrix_local-fixedYawPoseBodyTangentMap(window.para_Pose[0])*FIXED_YAW_RP_RADIANS_PER_LOCAL_UNIT).norm()==0);
        assert((optimizer.physical_pose_tangent_from_optimizer-b.optimizer_to_matrix_local).norm()==0);
        auto later=biasDescribeBlock(window,problem,window.para_Pose[1],7,6,6);
        assert(later.actual_optimizer_local_size==6&&later.optimizer_to_matrix_local.isIdentity());
        auto calibration=biasDescribeBlock(window,problem,window.para_Ex_Pose[0],7,12,6);
        assert(calibration.constant_in_actual_problem&&calibration.actual_optimizer_local_size==6);
        Eigen::Matrix<double,6,1> sigma=Eigen::Matrix<double,6,1>::Ones();
        assert(biasWriteDenseSnapshot(argv[1],1,"physical_margin","{}",Eigen::MatrixXd::Identity(6,6),
            Eigen::VectorXd::Zero(6),Eigen::VectorXd::Zero(6),{b},sigma));}
    std::cout<<"PASS: fixed XYZ/yaw, free RP, signed/wrapped quaternion, body basis FD, singular/domain rejection, schema3 mapping, bounded finite DBLmax probes\n";
}
