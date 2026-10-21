// Coordinate gauge only. Native manual Jacobians use body-tangent columns.
#pragma once
#include <ceres/ceres.h>
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>

// Local coordinates are milliradians, a fixed units conversion. Bounded
// smooth retractions handle arbitrary finite Ceres projected-gradient probes
// within the supported Euler chart; no state/output clipping is performed.
constexpr double FIXED_YAW_RP_RADIANS_PER_LOCAL_UNIT = .001;
constexpr double FIXED_YAW_PITCH_CHART_LIMIT_RAD = 89.*M_PI/180.;

inline double fixedYawPitchRetractionStep(double pitch,double local_delta)
{
    const double t=FIXED_YAW_PITCH_CHART_LIMIT_RAD;
    // Stable smooth quadratic: h <= distance-to-boundary/4. The finite
    // ratio may overflow to infinity; tanh(infinity) is its valid limit.
    const double h=(t-pitch)*(t+pitch)/(8*t);
    return h*std::tanh((FIXED_YAW_RP_RADIANS_PER_LOCAL_UNIT*local_delta)/h);
}
inline double fixedYawRollRetractionStep(double local_delta)
{
    // No full-turn identity alias in Ceres' projected-gradient norm.
    // d/d(delta) at zero is s, so existing Jphysical * sB is unchanged.
    return 2*std::atan(.5*FIXED_YAW_RP_RADIANS_PER_LOCAL_UNIT*local_delta);
}

inline bool fixedYawPoseZyx(const double* x, double& yaw, double& pitch, double& roll)
{
    for(int i=0;i<7;++i)if(!std::isfinite(x[i]))return false;
    Eigen::Quaterniond q(x[6],x[3],x[4],x[5]);
    if(std::abs(q.norm()-1.)>1e-6)return false;
    const Eigen::Matrix3d r=q.toRotationMatrix();
    const double sine=-r(2,0);
    if(sine < -1.-1e-12 || sine > 1.+1e-12)return false;
    pitch=std::asin(std::max(-1.,std::min(1.,sine)));
    yaw=std::atan2(r(1,0),r(0,0));roll=std::atan2(r(2,1),r(2,2));
    // The Euler chart is undefined at +/-90 degrees; reject the chart/step,
    // never clamp the physical state or silently switch gauge conventions.
    return std::abs(pitch)<FIXED_YAW_PITCH_CHART_LIMIT_RAD;
}

inline Eigen::Matrix<double,6,2> fixedYawPoseBodyTangentMap(const double* x)
{
    Eigen::Matrix<double,6,2> b=Eigen::Matrix<double,6,2>::Zero();
    double yaw,pitch,roll;if(!fixedYawPoseZyx(x,yaw,pitch,roll))return b;
    b(3,0)=1.;b(4,1)=std::cos(roll);b(5,1)=-std::sin(roll);return b;
}

class FixedYawPoseParameterization : public ceres::LocalParameterization
{
public:
    explicit FixedYawPoseParameterization(const double* anchor)
        : anchor_position_(anchor[0],anchor[1],anchor[2]), anchor_yaw_(0), valid_anchor_(false)
    { double pitch,roll;valid_anchor_=fixedYawPoseZyx(anchor,anchor_yaw_,pitch,roll); }
    bool Plus(const double* x,const double* delta,double* out) const override
    {
        double yaw,pitch,roll;
        if(!valid_anchor_ || !std::isfinite(delta[0]) || !std::isfinite(delta[1])
            || !fixedYawPoseZyx(x,yaw,pitch,roll) || !onAnchor(x,yaw))return false;
        if(delta[0]==0 && delta[1]==0){std::copy(x,x+7,out);return true;}
        pitch+=fixedYawPitchRetractionStep(pitch,delta[1]);
        roll+=fixedYawRollRetractionStep(delta[0]);
        if(!std::isfinite(pitch)||!std::isfinite(roll)||std::abs(pitch)>=FIXED_YAW_PITCH_CHART_LIMIT_RAD)return false;
        Eigen::Map<Eigen::Vector3d> p(out);p=anchor_position_;
        Eigen::Quaterniond q=Eigen::Quaterniond(Eigen::AngleAxisd(anchor_yaw_,Eigen::Vector3d::UnitZ()))
            *Eigen::Quaterniond(Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitY()))
            *Eigen::Quaterniond(Eigen::AngleAxisd(roll,Eigen::Vector3d::UnitX()));
        const Eigen::Quaterniond current(x[6],x[3],x[4],x[5]);
        if(q.coeffs().dot(current.coeffs())<0)q.coeffs()*=-1;
        Eigen::Map<Eigen::Quaterniond> output(out+3);output=q.normalized();return true;
    }
    bool ComputeJacobian(const double* x,double* jacobian) const override
    {
        double yaw,pitch,roll;if(!valid_anchor_||!fixedYawPoseZyx(x,yaw,pitch,roll)||!onAnchor(x,yaw))return false;
        Eigen::Map<Eigen::Matrix<double,7,2,Eigen::RowMajor>> j(jacobian);
        j.setZero();j.topRows<6>()=fixedYawPoseBodyTangentMap(x)*FIXED_YAW_RP_RADIANS_PER_LOCAL_UNIT;return true;
    }
    int GlobalSize() const override{return 7;}
    int LocalSize() const override{return 2;}
private:
    bool onAnchor(const double* x,double yaw) const
    {
        const Eigen::Vector3d p(x[0],x[1],x[2]);
        const double angle=std::atan2(std::sin(yaw-anchor_yaw_),std::cos(yaw-anchor_yaw_));
        return (p-anchor_position_).norm()<=1e-8 && std::abs(angle)<=1e-10;
    }
    const Eigen::Vector3d anchor_position_;
    double anchor_yaw_;
    bool valid_anchor_;
};
