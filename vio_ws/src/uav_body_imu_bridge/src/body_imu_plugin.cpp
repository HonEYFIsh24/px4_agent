#include <cmath>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include "Imu.pb.h"

namespace gazebo
{
// Subscribe in-process: PX4's lockstep world callback must receive every
// original IMU sample before the next physics tick. A second transport client
// can stall that delivery path; this plugin creates no replacement sensor.
class UavBodyImuPlugin : public ModelPlugin
{
public:
    void Load(physics::ModelPtr model, sdf::ElementPtr) override
    {
        if (!ros::isInitialized())
        {
            gzerr << "UavBodyImuPlugin requires gazebo_ros_api_plugin\n";
            return;
        }
        ros_node_.reset(new ros::NodeHandle());
        publisher_ = ros_node_->advertise<sensor_msgs::Imu>("/uav/imu/data_raw", 1000);
        gazebo_node_.reset(new transport::Node());
        gazebo_node_->Init();
        subscriber_ = gazebo_node_->Subscribe("~/" + model->GetName() + "/imu",
                                              &UavBodyImuPlugin::OnImu, this);
        ROS_INFO("Original Iris IMU forwarded in-process to /uav/imu/data_raw; no attitude/EKF input");
    }

private:
    void OnImu(const boost::shared_ptr<const sensor_msgs::msgs::Imu>& input)
    {
        const uint64_t stamp = input->time_usec();
        if (stamp <= previous_stamp_) return;
        previous_stamp_ = stamp;
        sensor_msgs::Imu output;
        output.header.stamp.fromNSec(stamp * 1000);
        output.header.frame_id = "uav_imu_link";
        output.angular_velocity.x = input->angular_velocity().x();
        output.angular_velocity.y = input->angular_velocity().y();
        output.angular_velocity.z = input->angular_velocity().z();
        output.linear_acceleration.x = input->linear_acceleration().x();
        output.linear_acceleration.y = input->linear_acceleration().y();
        output.linear_acceleration.z = input->linear_acceleration().z();
        if (!std::isfinite(output.angular_velocity.x) || !std::isfinite(output.angular_velocity.y)
            || !std::isfinite(output.angular_velocity.z) || !std::isfinite(output.linear_acceleration.x)
            || !std::isfinite(output.linear_acceleration.y) || !std::isfinite(output.linear_acceleration.z)) return;
        output.orientation.w = 1;
        output.orientation_covariance[0] = -1;
        for (int i = 0; i < 3; ++i)
        {
            output.angular_velocity_covariance[i * 4] = 0.00018665 * 0.00018665 / 0.004;
            output.linear_acceleration_covariance[i * 4] = 0.00186 * 0.00186 / 0.004;
        }
        publisher_.publish(output);
    }

    uint64_t previous_stamp_ = 0;
    std::unique_ptr<ros::NodeHandle> ros_node_;
    ros::Publisher publisher_;
    transport::NodePtr gazebo_node_;
    transport::SubscriberPtr subscriber_;
};
GZ_REGISTER_MODEL_PLUGIN(UavBodyImuPlugin)
}
