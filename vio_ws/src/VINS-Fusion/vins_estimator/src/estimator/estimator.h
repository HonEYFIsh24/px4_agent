/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once
 
#include <thread>
#include <mutex>
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <ceres/ceres.h>
#include <unordered_map>
#include <queue>
#include <opencv2/core/eigen.hpp>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include "parameters.h"
#include "feature_manager.h"
#include "../utility/utility.h"
#include "../utility/tic_toc.h"
#include "../initial/solve_5pts.h"
#include "../initial/initial_sfm.h"
#include "../initial/initial_alignment.h"
#include "../initial/initial_ex_rotation.h"
#include "../factor/imu_factor.h"
#include "../factor/pose_local_parameterization.h"
#include "../factor/marginalization_factor.h"
#include "../factor/projectionTwoFrameOneCamFactor.h"
#include "../factor/projectionTwoFrameTwoCamFactor.h"
#include "../factor/projectionOneFrameTwoCamFactor.h"
// PX4_AGENT_STATIONARY_HEALTH_V9: sensor health and opt-in matrix evidence.
#include "../factor/stationary_factors.h"
#include "../factor/imu_chain_health.h"
#include <set>
#include <string>
#include <atomic>
#include <fstream>
// PX4_AGENT_VISUAL_LIFECYCLE_DIAGNOSTICS_V1: read-only opt-in, no physical change.
#include "../factor/visual_lifecycle_diagnostics.h"
#include "../factor/stationary_initialization.h"
#include "../featureTracker/feature_tracker.h"


class Estimator
{
  public:
    Estimator();
    ~Estimator();
    void setParameter();

    // interface
    void initFirstPose(Eigen::Vector3d p, Eigen::Matrix3d r);
    void inputIMU(double t, const Vector3d &linearAcceleration, const Vector3d &angularVelocity);
    void inputFeature(double t, const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &featureFrame);
    void inputImage(double t, const cv::Mat &_img, const cv::Mat &_img1 = cv::Mat());
    void processIMU(double t, double dt, const Vector3d &linear_acceleration, const Vector3d &angular_velocity);
    void processImage(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, const double header);
    void processMeasurements();
    void changeSensorType(int use_imu, int use_stereo);

    // internal
    void clearState();
    bool initialStructure();
    bool visualInitialAlign();
    bool relativePose(Matrix3d &relative_R, Vector3d &relative_T, int &l);
    void slideWindow();
    void slideWindowNew();
    void slideWindowOld();
    void optimization();
    void vector2double();
    void double2vector();
    bool failureDetection();
    bool getIMUInterval(double t0, double t1, vector<pair<double, Eigen::Vector3d>> &accVector, 
                                              vector<pair<double, Eigen::Vector3d>> &gyrVector);
    void getPoseInWorldFrame(Eigen::Matrix4d &T);
    void getPoseInWorldFrame(int index, Eigen::Matrix4d &T);
    void predictPtsInNextFrame();
    void outliersRejection(set<int> &removeIndex);
    void beginVisualDiagnostics();
    void finishVisualOutlierDiagnostics(const set<int> &removeIndex);
    double reprojectionError(Matrix3d &Ri, Vector3d &Pi, Matrix3d &rici, Vector3d &tici,
                                     Matrix3d &Rj, Vector3d &Pj, Matrix3d &ricj, Vector3d &ticj, 
                                     double depth, Vector3d &uvi, Vector3d &uvj);
    bool updateLatestStates();
    void fastPredictIMU(double t, Eigen::Vector3d linear_acceleration, Eigen::Vector3d angular_velocity);
    void updateStationaryEvidence(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, double stamp);
    bool validateOptimizedSolution();
    std::string nativeHealthJson(double stamp, const std::string &state_phase = "after_process_image_and_slide") const;
    bool prepareStationaryInitialization();
    bool validateInitialWindowMapping(std::string &reason) const;
    void snapshotInitializationEvidence(const std::string &phase, const std::string &reason);
    void appendNativeHealthJson(const std::string &data);
    bool IMUAvailable(double t);
    void initFirstIMUPose(vector<pair<double, Eigen::Vector3d>> &accVector);

    enum SolverFlag
    {
        INITIAL,
        NON_LINEAR
    };

    enum MarginalizationFlag
    {
        MARGIN_OLD = 0,
        MARGIN_SECOND_NEW = 1
    };

    std::mutex mProcess;
    std::mutex mBuf;
    std::mutex mPropagate;
    queue<pair<double, Eigen::Vector3d>> accBuf;
    queue<pair<double, Eigen::Vector3d>> gyrBuf;
    queue<pair<double, map<int, vector<pair<int, Eigen::Matrix<double, 7, 1> > > > > > featureBuf;
    double prevTime, curTime;
    bool openExEstimation;

    std::thread trackThread;
    std::thread processThread;

    FeatureTracker featureTracker;

    std::atomic<SolverFlag> solver_flag;
    MarginalizationFlag  marginalization_flag;
    Vector3d g;

    Matrix3d ric[2];
    Vector3d tic[2];

    Vector3d        Ps[(WINDOW_SIZE + 1)];
    Vector3d        Vs[(WINDOW_SIZE + 1)];
    Matrix3d        Rs[(WINDOW_SIZE + 1)];
    Vector3d        Bas[(WINDOW_SIZE + 1)];
    Vector3d        Bgs[(WINDOW_SIZE + 1)];
    double td;

    Matrix3d back_R0, last_R, last_R0;
    Vector3d back_P0, last_P, last_P0;
    double Headers[(WINDOW_SIZE + 1)];

    IntegrationBase *pre_integrations[(WINDOW_SIZE + 1)] = {};
    Vector3d acc_0, gyr_0;

    vector<double> dt_buf[(WINDOW_SIZE + 1)];
    vector<Vector3d> linear_acceleration_buf[(WINDOW_SIZE + 1)];
    vector<Vector3d> angular_velocity_buf[(WINDOW_SIZE + 1)];

    int frame_count;
    int sum_of_outlier, sum_of_back, sum_of_front, sum_of_invalid;
    int inputImageCnt;

    FeatureManager f_manager;
    MotionEstimator m_estimator;
    InitialEXRotation initial_ex_rotation;

    bool first_imu;
    bool is_valid, is_key;
    bool failure_occur;

    vector<Vector3d> point_cloud;
    vector<Vector3d> margin_cloud;
    vector<Vector3d> key_poses;
    double initial_timestamp;


    double para_Pose[WINDOW_SIZE + 1][SIZE_POSE];
    double para_SpeedBias[WINDOW_SIZE + 1][SIZE_SPEEDBIAS];
    double para_Feature[NUM_OF_F][SIZE_FEATURE];
    double para_Ex_Pose[2][SIZE_POSE];
    double para_Retrive_Pose[SIZE_POSE];
    double para_Td[1][1];
    double para_Tr[1][1];

    int loop_window_index;

    MarginalizationInfo *last_marginalization_info = nullptr;
    vector<double *> last_marginalization_parameter_blocks;

    map<double, ImageFrame> all_image_frame;
    IntegrationBase *tmp_pre_integration = nullptr;

    Eigen::Vector3d initP;
    Eigen::Matrix3d initR;

    double latest_time;
    Eigen::Vector3d latest_P, latest_V, latest_Ba, latest_Bg, latest_acc_0, latest_gyr_0;
    Eigen::Quaterniond latest_Q;

    double visual_gap_since;
    SensorStationaryDetector stationary_detector;
    StationaryEvidence stationary_evidence;
    TemporalStereoWindow stationary_stereo_window;
    bool propagation_refresh_checked;
    NativeRefreshEvidence propagation_refresh_evidence;
    bool stationary_frames[WINDOW_SIZE + 1];
    bool stationary_intervals[WINDOW_SIZE + 1]; // [i] supports interval i-1 -> i
    std::set<int> previous_stereo_ids;
    bool stationary_initialization_seeded;
    int stationary_initialization_seed_count, stationary_initialization_wait_count;
    int initial_bias_prior_insertion_count;
    std::string native_initialization_evidence_json;
    std::ofstream native_health_trace;
    bool native_health_trace_error = false;
    std::size_t native_health_trace_records = 0;
    double stationary_initialization_seed_stamp;
    Eigen::Vector3d stationary_initialization_seed_acc, stationary_initialization_seed_gyro;
    double stationary_initialization_seed_first_imu_stamp, stationary_initialization_seed_last_imu_stamp;
    int stationary_initialization_seed_imu_samples;
    NativeVisualLifecycle native_visual_lifecycle;
    int native_visual_solve_sequence = 0;
    std::string native_stereo_tracks_json;
    std::string native_imu_optimizer_window_json;
    std::string native_bias_diagnostics_json;
    std::string native_gauge_evidence_json;
    std::vector<std::string> bias_diagnostic_file_entries;
    double bias_diagnostic_first_stamp, bias_diagnostic_last_stamp;
    int bias_diagnostic_sequence;
    bool keyframe_visual_requested, keyframe_forced_by_imu_span;
    double keyframe_candidate_merged_imu_span_s;
    int solver_iterations, solver_successful_steps, solver_unsuccessful_steps, solver_termination_type;
    double solver_wall_time_s, solver_budget_s, solver_last_gradient_max_norm;
    bool solver_solution_usable;
    double solver_initial_cost, solver_final_cost, solver_residual_rms;
    std::string solver_health_reason;
    bool have_validated_optimized;
    double last_validated_optimized_t;
    Eigen::Vector3d last_validated_optimized_p, last_validated_optimized_v;
    Eigen::Matrix3d last_validated_optimized_r;
    bool initFirstPoseFlag;
    bool initThreadFlag;
};
