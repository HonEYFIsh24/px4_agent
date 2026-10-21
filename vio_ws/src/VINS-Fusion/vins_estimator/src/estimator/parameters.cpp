/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "parameters.h"
#include <cmath>
#include <stdexcept>

double INIT_DEPTH;
double MIN_PARALLAX;
double ACC_N, ACC_W;
double ACC_BIAS_PRIOR_SIGMA = 0, GYR_BIAS_PRIOR_SIGMA = 0;
int ENABLE_ZUPT = 0;
// PX4_AGENT_VISUAL_LIFECYCLE_DIAGNOSTICS_V1: read-only opt-in, no physical change.
int VISUAL_LIFECYCLE_DIAGNOSTICS = 0;
int STATIONARY_INITIALIZATION = 0; // no delayed initialization unless explicitly enabled
double MAX_IMU_PREINTEGRATION_SPAN_S = 0; // opt-in; unchanged upstream keyframe policy by default
int FIX_FIRST_POSE_XYZ_YAW = 0; // coordinate gauge experiment, upstream disabled
int EXACT_MARGINALIZATION_POSE_JACOBIAN = 0; // upstream legacy unless explicit opt-in
std::string BIAS_DIAGNOSTICS_DIR; // empty disables all matrix snapshots
double BIAS_DIAGNOSTICS_PERIOD_S = 1.0, BIAS_DIAGNOSTICS_STARTUP_DENSE_S = 3.0;
Eigen::Vector3d HEALTH_IMU_TO_BODY_FLU = Eigen::Vector3d::Zero();
double ZUPT_VELOCITY_SIGMA = 0.03, ZUPT_POSITION_SIGMA = 0.02, ZUPT_CAMERA_FX = 320;
double GYR_N, GYR_W;

std::vector<Eigen::Matrix3d> RIC;
std::vector<Eigen::Vector3d> TIC;

Eigen::Vector3d G{0.0, 0.0, 9.8};

double BIAS_ACC_THRESHOLD;
double BIAS_GYR_THRESHOLD;
double SOLVER_TIME;
int NUM_ITERATIONS;
int ESTIMATE_EXTRINSIC;
int ESTIMATE_TD;
int ROLLING_SHUTTER;
std::string EX_CALIB_RESULT_PATH;
std::string VINS_RESULT_PATH;
std::string OUTPUT_FOLDER;
std::string IMU_TOPIC;
int ROW, COL;
double TD;
int NUM_OF_CAM;
int STEREO;
int USE_IMU;
int MULTIPLE_THREAD;
map<int, Eigen::Vector3d> pts_gt;
std::string IMAGE0_TOPIC, IMAGE1_TOPIC;
std::string FISHEYE_MASK;
std::vector<std::string> CAM_NAMES;
int MAX_CNT;
int MIN_DIST;
double F_THRESHOLD;
int SHOW_TRACK;
int FLOW_BACK;


template <typename T>
T readParam(ros::NodeHandle &n, std::string name)
{
    T ans;
    if (n.getParam(name, ans))
    {
        ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load " << name);
        n.shutdown();
    }
    return ans;
}

void readParameters(std::string config_file)
{
    FILE *fh = fopen(config_file.c_str(),"r");
    if(fh == NULL){
        ROS_WARN("config_file dosen't exist; wrong config_file path");
        ROS_BREAK();
        return;          
    }
    fclose(fh);

    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    fsSettings["image0_topic"] >> IMAGE0_TOPIC;
    fsSettings["image1_topic"] >> IMAGE1_TOPIC;
    MAX_CNT = fsSettings["max_cnt"];
    MIN_DIST = fsSettings["min_dist"];
    F_THRESHOLD = fsSettings["F_threshold"];
    SHOW_TRACK = fsSettings["show_track"];
    FLOW_BACK = fsSettings["flow_back"];

    MULTIPLE_THREAD = fsSettings["multiple_thread"];

    USE_IMU = fsSettings["imu"];
    printf("USE_IMU: %d\n", USE_IMU);
    if(USE_IMU)
    {
        fsSettings["imu_topic"] >> IMU_TOPIC;
        printf("IMU_TOPIC: %s\n", IMU_TOPIC.c_str());
        ACC_N = fsSettings["acc_n"];
        ACC_W = fsSettings["acc_w"];
        GYR_N = fsSettings["gyr_n"];
        GYR_W = fsSettings["gyr_w"];
        G.z() = fsSettings["g_norm"];
    }

    SOLVER_TIME = fsSettings["max_solver_time"];
    NUM_ITERATIONS = fsSettings["max_num_iterations"];
    MIN_PARALLAX = fsSettings["keyframe_parallax"];
    MIN_PARALLAX = MIN_PARALLAX / FOCAL_LENGTH;

    fsSettings["output_path"] >> OUTPUT_FOLDER;
    VINS_RESULT_PATH = OUTPUT_FOLDER + "/vio.csv";
    std::cout << "result path " << VINS_RESULT_PATH << std::endl;
    std::ofstream fout(VINS_RESULT_PATH, std::ios::out);
    fout.close();

    ESTIMATE_EXTRINSIC = fsSettings["estimate_extrinsic"];
    if (ESTIMATE_EXTRINSIC == 2)
    {
        ROS_WARN("have no prior about extrinsic param, calibrate extrinsic param");
        RIC.push_back(Eigen::Matrix3d::Identity());
        TIC.push_back(Eigen::Vector3d::Zero());
        EX_CALIB_RESULT_PATH = OUTPUT_FOLDER + "/extrinsic_parameter.csv";
    }
    else 
    {
        if ( ESTIMATE_EXTRINSIC == 1)
        {
            ROS_WARN(" Optimize extrinsic param around initial guess!");
            EX_CALIB_RESULT_PATH = OUTPUT_FOLDER + "/extrinsic_parameter.csv";
        }
        if (ESTIMATE_EXTRINSIC == 0)
            ROS_WARN(" fix extrinsic param ");

        cv::Mat cv_T;
        fsSettings["body_T_cam0"] >> cv_T;
        Eigen::Matrix4d T;
        cv::cv2eigen(cv_T, T);
        RIC.push_back(T.block<3, 3>(0, 0));
        TIC.push_back(T.block<3, 1>(0, 3));
    } 
    
    NUM_OF_CAM = fsSettings["num_of_cam"];
    printf("camera number %d\n", NUM_OF_CAM);

    if(NUM_OF_CAM != 1 && NUM_OF_CAM != 2)
    {
        printf("num_of_cam should be 1 or 2\n");
        assert(0);
    }


    int pn = config_file.find_last_of('/');
    std::string configPath = config_file.substr(0, pn);
    
    std::string cam0Calib;
    fsSettings["cam0_calib"] >> cam0Calib;
    std::string cam0Path = configPath + "/" + cam0Calib;
    CAM_NAMES.push_back(cam0Path);

    if(NUM_OF_CAM == 2)
    {
        STEREO = 1;
        std::string cam1Calib;
        fsSettings["cam1_calib"] >> cam1Calib;
        std::string cam1Path = configPath + "/" + cam1Calib; 
        //printf("%s cam1 path\n", cam1Path.c_str() );
        CAM_NAMES.push_back(cam1Path);
        
        cv::Mat cv_T;
        fsSettings["body_T_cam1"] >> cv_T;
        Eigen::Matrix4d T;
        cv::cv2eigen(cv_T, T);
        RIC.push_back(T.block<3, 3>(0, 0));
        TIC.push_back(T.block<3, 1>(0, 3));
    }

    if (!fsSettings["acc_bias_prior_sigma"].empty()) fsSettings["acc_bias_prior_sigma"] >> ACC_BIAS_PRIOR_SIGMA;
    if (!fsSettings["gyr_bias_prior_sigma"].empty()) fsSettings["gyr_bias_prior_sigma"] >> GYR_BIAS_PRIOR_SIGMA;
    ROS_INFO("Initial native-IMU bias prior sigmas: acceleration=%f gyro=%f", ACC_BIAS_PRIOR_SIGMA, GYR_BIAS_PRIOR_SIGMA);
    // PX4_AGENT_STATIONARY_HEALTH_V9: optional sensor-supported ZUPT and matrix evidence.
    if (!fsSettings["fix_first_pose_xyz_yaw"].empty())
        fsSettings["fix_first_pose_xyz_yaw"] >> FIX_FIRST_POSE_XYZ_YAW;
    if (FIX_FIRST_POSE_XYZ_YAW != 0 && FIX_FIRST_POSE_XYZ_YAW != 1)
        throw std::runtime_error("fix_first_pose_xyz_yaw must be 0 or 1");
    ROS_INFO("First-pose coordinate gauge: %s", FIX_FIRST_POSE_XYZ_YAW ? "fixed XYZ/yaw, free roll/pitch" : "legacy free 6D pose");

    if (!fsSettings["exact_marginalization_pose_jacobian"].empty())
        fsSettings["exact_marginalization_pose_jacobian"] >> EXACT_MARGINALIZATION_POSE_JACOBIAN;
    if (EXACT_MARGINALIZATION_POSE_JACOBIAN != 0 && EXACT_MARGINALIZATION_POSE_JACOBIAN != 1)
        throw std::runtime_error("exact_marginalization_pose_jacobian must be 0 or 1");
    ROS_INFO("Marginalized prior pose derivative: %s", EXACT_MARGINALIZATION_POSE_JACOBIAN ? "exact existing residual chart" : "legacy frozen local Jacobian");

    if (!fsSettings["bias_information_diagnostics_dir"].empty())
        fsSettings["bias_information_diagnostics_dir"] >> BIAS_DIAGNOSTICS_DIR;
    if (!fsSettings["bias_information_period_s"].empty())
        fsSettings["bias_information_period_s"] >> BIAS_DIAGNOSTICS_PERIOD_S;
    if (!fsSettings["bias_information_startup_dense_s"].empty())
        fsSettings["bias_information_startup_dense_s"] >> BIAS_DIAGNOSTICS_STARTUP_DENSE_S;
    if (!std::isfinite(BIAS_DIAGNOSTICS_PERIOD_S) || BIAS_DIAGNOSTICS_PERIOD_S < 1.0
        || !std::isfinite(BIAS_DIAGNOSTICS_STARTUP_DENSE_S) || BIAS_DIAGNOSTICS_STARTUP_DENSE_S < 0
        || BIAS_DIAGNOSTICS_STARTUP_DENSE_S > 5.0)
        throw std::runtime_error("invalid opt-in matrix diagnostic timing");
    ROS_INFO("Bias matrix diagnostics: %s (empty disables), period=%.3fs startup_dense=%.3fs",
        BIAS_DIAGNOSTICS_DIR.c_str(), BIAS_DIAGNOSTICS_PERIOD_S, BIAS_DIAGNOSTICS_STARTUP_DENSE_S);
    if (!fsSettings["health_imu_to_body_flu"].empty())
    {
        const cv::FileNode offset = fsSettings["health_imu_to_body_flu"];
        if (!offset.isSeq() || offset.size() != 3) throw std::runtime_error("invalid native health IMU/body lever arm");
        for (int axis = 0; axis < 3; ++axis) HEALTH_IMU_TO_BODY_FLU[axis] = static_cast<double>(offset[axis]);
        if (!HEALTH_IMU_TO_BODY_FLU.allFinite()) throw std::runtime_error("nonfinite native health IMU/body lever arm");
    }
    if (!fsSettings["enable_zupt"].empty()) fsSettings["enable_zupt"] >> ENABLE_ZUPT;
    VISUAL_LIFECYCLE_DIAGNOSTICS = 0;
    if (!fsSettings["visual_lifecycle_diagnostics"].empty()) fsSettings["visual_lifecycle_diagnostics"] >> VISUAL_LIFECYCLE_DIAGNOSTICS;
    if (VISUAL_LIFECYCLE_DIAGNOSTICS != 0 && VISUAL_LIFECYCLE_DIAGNOSTICS != 1)
        throw std::runtime_error("visual_lifecycle_diagnostics must be 0 or 1");
    ROS_INFO("Native bounded visual lifecycle diagnostics: %s", VISUAL_LIFECYCLE_DIAGNOSTICS ? "enabled" : "disabled");
    if (!fsSettings["stationary_initialization"].empty()) fsSettings["stationary_initialization"] >> STATIONARY_INITIALIZATION;
    if ((STATIONARY_INITIALIZATION != 0 && STATIONARY_INITIALIZATION != 1)
        || (STATIONARY_INITIALIZATION && (!STEREO || !USE_IMU || !ENABLE_ZUPT)))
        throw std::runtime_error("stationary_initialization requires stereo, IMU, sensor ZUPT and flag 0/1");
    ROS_INFO("Sensor-supported entire-window initialization: %s; native_health.jsonl %s",
        STATIONARY_INITIALIZATION ? "enabled" : "disabled", STATIONARY_INITIALIZATION ? "enabled" : "disabled");
    if (!fsSettings["max_imu_preintegration_span_s"].empty())
        fsSettings["max_imu_preintegration_span_s"] >> MAX_IMU_PREINTEGRATION_SPAN_S;
    if (!std::isfinite(MAX_IMU_PREINTEGRATION_SPAN_S) || MAX_IMU_PREINTEGRATION_SPAN_S < 0
        || (MAX_IMU_PREINTEGRATION_SPAN_S > 0
            && (MAX_IMU_PREINTEGRATION_SPAN_S < 0.2 || MAX_IMU_PREINTEGRATION_SPAN_S > 5.0)))
        throw std::runtime_error("max_imu_preintegration_span_s must be 0 (disabled) or 0.2..5 seconds");
    ROS_INFO("Measured IMU-span keyframe limit: %.3f s (0 disables)", MAX_IMU_PREINTEGRATION_SPAN_S);
    if (!fsSettings["zupt_velocity_sigma"].empty()) fsSettings["zupt_velocity_sigma"] >> ZUPT_VELOCITY_SIGMA;
    if (!fsSettings["zupt_position_sigma"].empty()) fsSettings["zupt_position_sigma"] >> ZUPT_POSITION_SIGMA;
    if (!fsSettings["zupt_camera_fx"].empty()) fsSettings["zupt_camera_fx"] >> ZUPT_CAMERA_FX;
    if ((ENABLE_ZUPT != 0 && ENABLE_ZUPT != 1) || !std::isfinite(ZUPT_VELOCITY_SIGMA)
        || !std::isfinite(ZUPT_POSITION_SIGMA) || !std::isfinite(ZUPT_CAMERA_FX)
        || ZUPT_VELOCITY_SIGMA < 0.01 || ZUPT_POSITION_SIGMA < 0.01 || ZUPT_CAMERA_FX <= 0)
        throw std::runtime_error("invalid optional ZUPT configuration");
    ROS_INFO("Sensor ZUPT enabled=%d velocity_sigma=%.3f position_sigma=%.3f fx=%.1f",
             ENABLE_ZUPT, ZUPT_VELOCITY_SIGMA, ZUPT_POSITION_SIGMA, ZUPT_CAMERA_FX);

    INIT_DEPTH = 5.0;
    BIAS_ACC_THRESHOLD = 0.1;
    BIAS_GYR_THRESHOLD = 0.1;

    TD = fsSettings["td"];
    ESTIMATE_TD = fsSettings["estimate_td"];
    if (ESTIMATE_TD)
        ROS_INFO_STREAM("Unsynchronized sensors, online estimate time offset, initial td: " << TD);
    else
        ROS_INFO_STREAM("Synchronized sensors, fix time offset: " << TD);

    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];
    ROS_INFO("ROW: %d COL: %d ", ROW, COL);

    if(!USE_IMU)
    {
        ESTIMATE_EXTRINSIC = 0;
        ESTIMATE_TD = 0;
        printf("no imu, fix extrinsic param; no time offset calibration\n");
    }

    fsSettings.release();
}
