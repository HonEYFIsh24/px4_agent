/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "estimator.h"
#include "../utility/visualization.h"
#include "../factor/imu_bias_prior.h"
#include "../factor/bias_native_diagnostics.h"
#include "../factor/fixed_yaw_pose_parameterization.h"
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <sstream>
// PX4_AGENT_STATIONARY_HEALTH_V9: sensor health and opt-in source matrix snapshots.
// PX4_AGENT_INITIAL_BIAS_PRIOR: native IMU statistics, added once then marginalized.
// PX4_AGENT_VIO_PATCH: health, guarded reset, and clean shutdown only.

Estimator::Estimator(): f_manager{Rs}
{
    ROS_INFO("init begins");
    initThreadFlag = false;
    clearState();
}

Estimator::~Estimator()
{
    if (MULTIPLE_THREAD)
    {
        processThread.join();
        printf("join thread \n");
    }
}

void Estimator::clearState()
{
    mProcess.lock();
    {
        std::lock_guard<std::mutex> buffer_lock(mBuf);
        while(!accBuf.empty()) accBuf.pop();
        while(!gyrBuf.empty()) gyrBuf.pop();
        while(!featureBuf.empty()) featureBuf.pop();
    }

    visual_gap_since = -1;
    stationary_detector.clear();
    stationary_evidence = StationaryEvidence();
    stationary_stereo_window.clear();
    propagation_refresh_checked = false;
    propagation_refresh_evidence = NativeRefreshEvidence();
    previous_stereo_ids.clear();
// PX4_AGENT_VISUAL_LIFECYCLE_DIAGNOSTICS_V1: read-only opt-in, no physical change.
    native_visual_lifecycle = NativeVisualLifecycle();
    native_visual_solve_sequence = 0;
    native_stereo_tracks_json = "[]";
    stationary_initialization_seeded = false;
    stationary_initialization_seed_count = stationary_initialization_wait_count = 0;
    initial_bias_prior_insertion_count = 0;
    stationary_initialization_seed_stamp = -1;
    stationary_initialization_seed_acc.setZero(); stationary_initialization_seed_gyro.setZero();
    stationary_initialization_seed_first_imu_stamp = stationary_initialization_seed_last_imu_stamp = -1;
    stationary_initialization_seed_imu_samples = 0;
    native_initialization_evidence_json = "{\"phase\":\"startup\"}";
    solver_solution_usable = true;
    solver_initial_cost = solver_final_cost = solver_residual_rms = 0;
    solver_health_reason.clear();
    native_imu_optimizer_window_json = "{}";
    native_bias_diagnostics_json = "{\"enabled\":false}";
    native_gauge_evidence_json = "{\"sampled\":false}";
    bias_diagnostic_first_stamp = bias_diagnostic_last_stamp = -1;
    bias_diagnostic_sequence = 0;
    bias_diagnostic_file_entries.clear();
    keyframe_visual_requested = keyframe_forced_by_imu_span = false;
    keyframe_candidate_merged_imu_span_s = 0;
    solver_iterations = solver_successful_steps = solver_unsuccessful_steps = 0;
    solver_termination_type = -1;
    solver_wall_time_s = solver_budget_s = solver_last_gradient_max_norm = 0;
    have_validated_optimized = false;
    last_validated_optimized_t = -1;
    prevTime = -1;
    curTime = 0;
    openExEstimation = 0;
    initP = Eigen::Vector3d(0, 0, 0);
    initR = Eigen::Matrix3d::Identity();
    inputImageCnt = 0;
    initFirstPoseFlag = false;

    for (int i = 0; i < WINDOW_SIZE + 1; i++)
    {
        Headers[i] = 0;
        Rs[i].setIdentity();
        Ps[i].setZero();
        Vs[i].setZero();
        Bas[i].setZero();
        Bgs[i].setZero();
        stationary_frames[i] = false;
        stationary_intervals[i] = false;
        dt_buf[i].clear();
        linear_acceleration_buf[i].clear();
        angular_velocity_buf[i].clear();

        if (pre_integrations[i] != nullptr)
        {
            delete pre_integrations[i];
        }
        pre_integrations[i] = nullptr;
    }

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic[i] = Vector3d::Zero();
        ric[i] = Matrix3d::Identity();
    }

    first_imu = false,
    sum_of_back = 0;
    sum_of_front = 0;
    frame_count = 0;
    solver_flag = INITIAL;
    initial_timestamp = 0;
    // ImageFrame edges own distinct preintegrations, not window/tmp pointers.
    for (auto &entry : all_image_frame)
    { delete entry.second.pre_integration; entry.second.pre_integration = nullptr; }
    all_image_frame.clear();

    if (tmp_pre_integration != nullptr)
        delete tmp_pre_integration;
    if (last_marginalization_info != nullptr)
        delete last_marginalization_info;

    tmp_pre_integration = nullptr;
    last_marginalization_info = nullptr;
    last_marginalization_parameter_blocks.clear();

    f_manager.clearState();

    failure_occur = 0;

    mProcess.unlock();
}

void Estimator::setParameter()
{
    mProcess.lock();
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic[i] = TIC[i];
        ric[i] = RIC[i];
        cout << " exitrinsic cam " << i << endl  << ric[i] << endl << tic[i].transpose() << endl;
    }
    f_manager.setRic(ric);
    ProjectionTwoFrameOneCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    ProjectionTwoFrameTwoCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    ProjectionOneFrameTwoCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    td = TD;
    g = G;
    cout << "set g " << g.transpose() << endl;
    featureTracker.readIntrinsicParameter(CAM_NAMES);

    std::cout << "MULTIPLE_THREAD is " << MULTIPLE_THREAD << '\n';
    if (MULTIPLE_THREAD && !initThreadFlag)
    {
        initThreadFlag = true;
        processThread = std::thread(&Estimator::processMeasurements, this);
    }
    mProcess.unlock();
}

void Estimator::changeSensorType(int use_imu, int use_stereo)
{
    bool restart = false;
    mProcess.lock();
    if(!use_imu && !use_stereo)
        printf("at least use two sensors! \n");
    else
    {
        if(USE_IMU != use_imu)
        {
            USE_IMU = use_imu;
            if(USE_IMU)
            {
                // reuse imu; restart system
                restart = true;
            }
            else
            {
                if (last_marginalization_info != nullptr)
                    delete last_marginalization_info;

                tmp_pre_integration = nullptr;
                last_marginalization_info = nullptr;
                last_marginalization_parameter_blocks.clear();
            }
        }
        
        STEREO = use_stereo;
        printf("use imu %d use stereo %d\n", USE_IMU, STEREO);
    }
    mProcess.unlock();
    if(restart)
    {
        clearState();
        setParameter();
    }
}

void Estimator::inputImage(double t, const cv::Mat &_img, const cv::Mat &_img1)
{
    inputImageCnt++;
    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
    TicToc featureTrackerTime;

    if(_img1.empty())
        featureFrame = featureTracker.trackImage(t, _img);
    else
        featureFrame = featureTracker.trackImage(t, _img, _img1);
    //printf("featureTracker time: %f\n", featureTrackerTime.toc());

    if (SHOW_TRACK)
    {
        cv::Mat imgTrack = featureTracker.getTrackImage();
        pubTrackImage(imgTrack, t);
    }
    
    if(MULTIPLE_THREAD)  
    {     
        if(inputImageCnt % 2 == 0)
        {
            mBuf.lock();
            featureBuf.push(make_pair(t, featureFrame));
            mBuf.unlock();
        }
    }
    else
    {
        mBuf.lock();
        featureBuf.push(make_pair(t, featureFrame));
        mBuf.unlock();
        TicToc processTime;
        processMeasurements();
        printf("process time: %f\n", processTime.toc());
    }
    
}

void Estimator::inputIMU(double t, const Vector3d &linearAcceleration, const Vector3d &angularVelocity)
{
    mBuf.lock();
    accBuf.push(make_pair(t, linearAcceleration));
    gyrBuf.push(make_pair(t, angularVelocity));
    //printf("input imu with time %f \n", t);
    mBuf.unlock();

    if (solver_flag == NON_LINEAR)
    {
        std::lock_guard<std::mutex> propagation_lock(mPropagate);
        // A failure may have been latched while this callback waited for the
        // refresh lock. A successful refresh may already contain this queued
        // sample; 0 dt is its first genuine publication, an older stamp is not.
        if (solver_flag == NON_LINEAR && t >= latest_time)
        {
            fastPredictIMU(t, linearAcceleration, angularVelocity);
            pubLatestOdometry(latest_P, latest_Q, latest_V, t);
        }
    }
}

void Estimator::inputFeature(double t, const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &featureFrame)
{
    mBuf.lock();
    featureBuf.push(make_pair(t, featureFrame));
    mBuf.unlock();

    if(!MULTIPLE_THREAD)
        processMeasurements();
}


bool Estimator::getIMUInterval(double t0, double t1, vector<pair<double, Eigen::Vector3d>> &accVector, 
                                vector<pair<double, Eigen::Vector3d>> &gyrVector)
{
    if(accBuf.empty())
    {
        printf("not receive imu\n");
        return false;
    }
    //printf("get imu from %f %f\n", t0, t1);
    //printf("imu fornt time %f   imu end time %f\n", accBuf.front().first, accBuf.back().first);
    if(t1 <= accBuf.back().first)
    {
        while (!accBuf.empty() && accBuf.front().first <= t0)
        {
            accBuf.pop();
            gyrBuf.pop();
        }
        if (accBuf.empty() || gyrBuf.empty()) return false;
        while (accBuf.front().first < t1)
        {
            accVector.push_back(accBuf.front());
            accBuf.pop();
            gyrVector.push_back(gyrBuf.front());
            gyrBuf.pop();
        }
        accVector.push_back(accBuf.front());
        gyrVector.push_back(gyrBuf.front());
    }
    else
    {
        printf("wait for imu\n");
        return false;
    }
    return true;
}

bool Estimator::IMUAvailable(double t)
{
    std::lock_guard<std::mutex> buffer_lock(mBuf);
    if(!accBuf.empty() && t <= accBuf.back().first)
        return true;
    else
        return false;
}

void Estimator::processMeasurements()
{
    ros::NodeHandle health_node;
    ros::Publisher bias_pub = health_node.advertise<std_msgs::Float64MultiArray>("/vins_estimator/imu_bias", 1);
    ros::Publisher native_health_pub = health_node.advertise<std_msgs::String>("/vins_estimator/native_health", 1, true);
    ros::Publisher health_pub = health_node.advertise<std_msgs::Bool>("/vins_estimator/tracking_ok", 1, true);
    while (ros::ok())
    {
        //printf("process measurments\n");
        pair<double, map<int, vector<pair<int, Eigen::Matrix<double, 7, 1> > > > > feature;
        vector<pair<double, Eigen::Vector3d>> accVector, gyrVector;
        bool feature_available = false;
        {
            std::lock_guard<std::mutex> buffer_lock(mBuf);
            if (!featureBuf.empty()) { feature = featureBuf.front(); feature_available = true; }
        }
        if(feature_available)
        {
            double required_imu_time = feature.first;
            int use_imu_for_wait = 0;
            {
                std::lock_guard<std::mutex> process_lock(mProcess);
                required_imu_time += td; use_imu_for_wait = USE_IMU;
            }
            while(ros::ok())
            {
                if ((!use_imu_for_wait || IMUAvailable(required_imu_time)))
                    break;
                else
                {
                    printf("wait for imu ... \n");
                    if (! MULTIPLE_THREAD)
                        return;
                    std::chrono::milliseconds dura(5);
                    std::this_thread::sleep_for(dura);
                }
            }
            if (!ros::ok()) return;
            // Serialize native IMU integration and detector/preintegration state
            // with external restart, not only the later image optimization.
            mProcess.lock();
            mBuf.lock();
            if (featureBuf.empty() || featureBuf.front().first != feature.first)
            { mBuf.unlock(); mProcess.unlock(); continue; } // reset discarded this snapshot
            curTime = feature.first + td;
            if (USE_IMU && !getIMUInterval(prevTime, curTime, accVector, gyrVector))
            { mBuf.unlock(); mProcess.unlock(); continue; }
            featureBuf.pop();
            mBuf.unlock();

            if(USE_IMU)
            {
                if(!initFirstPoseFlag)
                    initFirstIMUPose(accVector);
                for(size_t i = 0; i < accVector.size(); i++)
                {
                    double dt;
                    if(i == 0)
                        dt = accVector[i].first - prevTime;
                    else if (i == accVector.size() - 1)
                        dt = curTime - accVector[i - 1].first;
                    else
                        dt = accVector[i].first - accVector[i - 1].first;
                    processIMU(accVector[i].first, dt, accVector[i].second, gyrVector[i].second);
                }
            }
            processImage(feature.second, feature.first);
            std_msgs::String native_health;
            native_health.data = nativeHealthJson(feature.first);
            appendNativeHealthJson(native_health.data);
            native_health_pub.publish(native_health);
            std_msgs::Bool tracking_status;
            tracking_status.data = solver_flag == NON_LINEAR && !failure_occur;
            health_pub.publish(tracking_status);
            std_msgs::Float64MultiArray bias_status;
            for (int axis = 0; axis < 3; ++axis) bias_status.data.push_back(Bas[frame_count][axis]);
            for (int axis = 0; axis < 3; ++axis) bias_status.data.push_back(Bgs[frame_count][axis]);
            bias_pub.publish(bias_status);
            if (failure_occur)
            {
                // clearState/setParameter acquire mProcess themselves.
                mProcess.unlock();
                clearState();
                setParameter();
                ROS_WARN("VIO reset: waiting for stereo/IMU initialization");
                continue;
            }
            prevTime = curTime;

            printStatistics(*this, 0);

            std_msgs::Header header;
            header.frame_id = "world";
            header.stamp = ros::Time(feature.first);

            pubOdometry(*this, header);
            pubKeyPoses(*this, header);
            pubCameraPose(*this, header);
            pubPointCloud(*this, header);
            pubKeyframe(*this);
            pubTF(*this, header);
            mProcess.unlock();
        }

        if (! MULTIPLE_THREAD)
            break;

        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}


void Estimator::initFirstIMUPose(vector<pair<double, Eigen::Vector3d>> &accVector)
{
    printf("init first imu pose\n");
    initFirstPoseFlag = true;
    //return;
    Eigen::Vector3d averAcc(0, 0, 0);
    int n = (int)accVector.size();
    for(size_t i = 0; i < accVector.size(); i++)
    {
        averAcc = averAcc + accVector[i].second;
    }
    averAcc = averAcc / n;
    printf("averge acc %f %f %f\n", averAcc.x(), averAcc.y(), averAcc.z());
    Matrix3d R0 = Utility::g2R(averAcc);
    double yaw = Utility::R2ypr(R0).x();
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
    Rs[0] = R0;
    cout << "init R0 " << endl << Rs[0] << endl;
    //Vs[0] = Vector3d(5, 0, 0);
}

void Estimator::initFirstPose(Eigen::Vector3d p, Eigen::Matrix3d r)
{
    Ps[0] = p;
    Rs[0] = r;
    initP = p;
    initR = r;
}


void Estimator::processIMU(double t, double dt, const Vector3d &linear_acceleration, const Vector3d &angular_velocity)
{
    stationary_detector.addIMU(t, linear_acceleration, angular_velocity);
    if (!first_imu)
    {
        first_imu = true;
        acc_0 = linear_acceleration;
        gyr_0 = angular_velocity;
    }

    if (!pre_integrations[frame_count])
    {
        pre_integrations[frame_count] = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};
    }
    if (frame_count != 0)
    {
        pre_integrations[frame_count]->push_back(dt, linear_acceleration, angular_velocity);
        //if(solver_flag != NON_LINEAR)
            tmp_pre_integration->push_back(dt, linear_acceleration, angular_velocity);

        dt_buf[frame_count].push_back(dt);
        linear_acceleration_buf[frame_count].push_back(linear_acceleration);
        angular_velocity_buf[frame_count].push_back(angular_velocity);

        int j = frame_count;         
        Vector3d un_acc_0 = Rs[j] * (acc_0 - Bas[j]) - g;
        Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - Bgs[j];
        Rs[j] *= Utility::deltaQ(un_gyr * dt).toRotationMatrix();
        Vector3d un_acc_1 = Rs[j] * (linear_acceleration - Bas[j]) - g;
        Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
        Ps[j] += dt * Vs[j] + 0.5 * dt * dt * un_acc;
        Vs[j] += dt * un_acc;
    }
    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity; 
}

void Estimator::processImage(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, const double header)
{
    // Refuse invalid source time before transferring the temporary IMU edge.
    if (!std::isfinite(header) || (!all_image_frame.empty() && header <= all_image_frame.rbegin()->first))
    {
        solver_solution_usable = false; failure_occur = true; solver_flag = INITIAL;
        solver_health_reason = "invalid_or_duplicate_image_source_stamp";
        if (STATIONARY_INITIALIZATION) snapshotInitializationEvidence("rejected", solver_health_reason);
        return;
    }
    updateStationaryEvidence(image, header);
    ROS_DEBUG("new image coming ------------------------------------------");
    ROS_DEBUG("Adding feature points %lu", image.size());
    keyframe_visual_requested = f_manager.addFeatureCheckParallax(frame_count, image, td);
    keyframe_candidate_merged_imu_span_s = 0;
    keyframe_forced_by_imu_span = false;
    if (USE_IMU && frame_count == WINDOW_SIZE && pre_integrations[frame_count-1] && pre_integrations[frame_count])
    {
        keyframe_candidate_merged_imu_span_s = pre_integrations[frame_count-1]->sum_dt
            + pre_integrations[frame_count]->sum_dt;
        keyframe_forced_by_imu_span = forceKeyframeForImuSpan(USE_IMU, true,
            pre_integrations[frame_count-1]->sum_dt, pre_integrations[frame_count]->sum_dt,
            MAX_IMU_PREINTEGRATION_SPAN_S);
    }
    // Preserve the existing parallax decision; additionally retain a keyframe
    // before SECOND_NEW would discard an IMU bridge after prolonged stillness.
    if (keyframe_visual_requested || keyframe_forced_by_imu_span)
    {
        marginalization_flag = MARGIN_OLD;
        //printf("keyframe\n");
    }
    else
    {
        marginalization_flag = MARGIN_SECOND_NEW;
        //printf("non-keyframe\n");
    }

    ROS_DEBUG("%s", marginalization_flag ? "Non-keyframe" : "Keyframe");
    ROS_DEBUG("Solving %d", frame_count);
    ROS_DEBUG("number of feature: %d", f_manager.getFeatureCount());
    Headers[frame_count] = header;

    ImageFrame imageframe(image, header);
    imageframe.pre_integration = tmp_pre_integration;
    all_image_frame.insert(make_pair(header, imageframe));
    tmp_pre_integration = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};

    if(ESTIMATE_EXTRINSIC == 2)
    {
        ROS_INFO("calibrating extrinsic param, rotation movement is needed");
        if (frame_count != 0)
        {
            vector<pair<Vector3d, Vector3d>> corres = f_manager.getCorresponding(frame_count - 1, frame_count);
            Matrix3d calib_ric;
            if (initial_ex_rotation.CalibrationExRotation(corres, pre_integrations[frame_count]->delta_q, calib_ric))
            {
                ROS_WARN("initial extrinsic rotation calib success");
                ROS_WARN_STREAM("initial extrinsic rotation: " << endl << calib_ric);
                ric[0] = calib_ric;
                RIC[0] = calib_ric;
                ESTIMATE_EXTRINSIC = 1;
            }
        }
    }

    if (solver_flag == INITIAL)
    {
        // monocular + IMU initilization
        if (!STEREO && USE_IMU)
        {
            if (frame_count == WINDOW_SIZE)
            {
                bool result = false;
                if(ESTIMATE_EXTRINSIC != 2 && (header - initial_timestamp) > 0.1)
                {
                    result = initialStructure();
                    initial_timestamp = header;   
                }
                if(result)
                {
                    optimization();
                    if (!solver_solution_usable) { failure_occur = true; solver_flag = INITIAL; return; }
                    if (!updateLatestStates()) { failure_occur = true; solver_flag = INITIAL; return; }
                    solver_flag = NON_LINEAR;
                    slideWindow();
                    ROS_INFO("Initialization finish!");
                }
                else
                    slideWindow();
            }
        }

        // stereo + IMU initilization
        if(STEREO && USE_IMU)
        {
            f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic, ric);
            f_manager.triangulate(frame_count, Ps, Rs, tic, ric);
            if (STATIONARY_INITIALIZATION && frame_count < WINDOW_SIZE)
                snapshotInitializationEvidence("waiting", "partial_camera_window");
            if (frame_count == WINDOW_SIZE)
            {
                if (STATIONARY_INITIALIZATION)
                {
                    if (!prepareStationaryInitialization())
                    {
                        if (!failure_occur)
                        {
                            marginalization_flag = MARGIN_OLD;
                            slideWindow(); // preserve every real adjacent IMU edge while waiting
                        }
                        return; // INITIAL: no solve, no marginalized prior, no VIO output
                    }
                    marginalization_flag = MARGIN_OLD; // first usable solve carries the once-only prior
                }
                else
                {
                    // Immediate upstream initialization has exactly the current full window.
                    std::string mapping_reason;
                    if (!validateInitialWindowMapping(mapping_reason))
                    { solver_solution_usable = false; failure_occur = true; solver_health_reason = mapping_reason; return; }
                    for (int i = 0; i <= WINDOW_SIZE; ++i)
                    { all_image_frame.at(Headers[i]).R = Rs[i]; all_image_frame.at(Headers[i]).T = Ps[i]; }
                    solveGyroscopeBias(all_image_frame, Bgs);
                    for (int i = 0; i <= WINDOW_SIZE; i++)
                        pre_integrations[i]->repropagate(Vector3d::Zero(), Bgs[i]);
                }
                optimization();
                if (STATIONARY_INITIALIZATION)
                    snapshotInitializationEvidence("first_solve_complete", solver_solution_usable ? "usable_joint_solve" : "unusable_joint_solve");
                if (!solver_solution_usable) { failure_occur = true; solver_flag = INITIAL; return; }
                if (!updateLatestStates()) { failure_occur = true; solver_flag = INITIAL; return; }
                solver_flag = NON_LINEAR;
                slideWindow();
                ROS_INFO("Initialization finish!");
            }
        }

        // stereo only initilization
        if(STEREO && !USE_IMU)
        {
            f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic, ric);
            f_manager.triangulate(frame_count, Ps, Rs, tic, ric);
            optimization();
            if (!solver_solution_usable) { failure_occur = true; solver_flag = INITIAL; return; }

            if(frame_count == WINDOW_SIZE)
            {
                optimization();
                if (!solver_solution_usable) { failure_occur = true; solver_flag = INITIAL; return; }
                if (!updateLatestStates()) { failure_occur = true; solver_flag = INITIAL; return; }
                solver_flag = NON_LINEAR;
                slideWindow();
                ROS_INFO("Initialization finish!");
            }
        }

        if(frame_count < WINDOW_SIZE)
        {
            frame_count++;
            int prev_frame = frame_count - 1;
            Ps[frame_count] = Ps[prev_frame];
            Vs[frame_count] = Vs[prev_frame];
            Rs[frame_count] = Rs[prev_frame];
            Bas[frame_count] = Bas[prev_frame];
            Bgs[frame_count] = Bgs[prev_frame];
        }

    }
    else
    {
        TicToc t_solve;
        if(!USE_IMU)
            f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic, ric);
        f_manager.triangulate(frame_count, Ps, Rs, tic, ric);
        optimization();
        if (!solver_solution_usable) { failure_occur = true; solver_flag = INITIAL; return; }
        set<int> removeIndex;
        outliersRejection(removeIndex);
        f_manager.removeOutlier(removeIndex);
        if (VISUAL_LIFECYCLE_DIAGNOSTICS) finishVisualOutlierDiagnostics(removeIndex);
        if (! MULTIPLE_THREAD)
        {
            featureTracker.removeOutliers(removeIndex);
            predictPtsInNextFrame();
        }
            
        ROS_DEBUG("solver costs: %fms", t_solve.toc());

        if (failureDetection())
        {
            ROS_WARN("failure detection!");
            failure_occur = 1;
            solver_flag = INITIAL;
            return;
        }

        slideWindow();
        f_manager.removeFailures();
        // prepare output of VINS
        key_poses.clear();
        for (int i = 0; i <= WINDOW_SIZE; i++)
            key_poses.push_back(Ps[i]);

        last_R = Rs[WINDOW_SIZE];
        last_P = Ps[WINDOW_SIZE];
        last_R0 = Rs[0];
        last_P0 = Ps[0];
        if (!updateLatestStates()) { failure_occur = true; solver_flag = INITIAL; return; }
    }  
}

bool Estimator::initialStructure()
{
    TicToc t_sfm;
    //check imu observibility
    {
        map<double, ImageFrame>::iterator frame_it;
        Vector3d sum_g;
        for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++)
        {
            double dt = frame_it->second.pre_integration->sum_dt;
            Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
            sum_g += tmp_g;
        }
        Vector3d aver_g;
        aver_g = sum_g * 1.0 / ((int)all_image_frame.size() - 1);
        double var = 0;
        for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++)
        {
            double dt = frame_it->second.pre_integration->sum_dt;
            Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
            var += (tmp_g - aver_g).transpose() * (tmp_g - aver_g);
            //cout << "frame g " << tmp_g.transpose() << endl;
        }
        var = sqrt(var / ((int)all_image_frame.size() - 1));
        //ROS_WARN("IMU variation %f!", var);
        if(var < 0.25)
        {
            ROS_INFO("IMU excitation not enouth!");
            //return false;
        }
    }
    // global sfm
    Quaterniond Q[frame_count + 1];
    Vector3d T[frame_count + 1];
    map<int, Vector3d> sfm_tracked_points;
    vector<SFMFeature> sfm_f;
    for (auto &it_per_id : f_manager.feature)
    {
        int imu_j = it_per_id.start_frame - 1;
        SFMFeature tmp_feature;
        tmp_feature.state = false;
        tmp_feature.id = it_per_id.feature_id;
        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            Vector3d pts_j = it_per_frame.point;
            tmp_feature.observation.push_back(make_pair(imu_j, Eigen::Vector2d{pts_j.x(), pts_j.y()}));
        }
        sfm_f.push_back(tmp_feature);
    } 
    Matrix3d relative_R;
    Vector3d relative_T;
    int l;
    if (!relativePose(relative_R, relative_T, l))
    {
        ROS_INFO("Not enough features or parallax; Move device around");
        return false;
    }
    GlobalSFM sfm;
    if(!sfm.construct(frame_count + 1, Q, T, l,
              relative_R, relative_T,
              sfm_f, sfm_tracked_points))
    {
        ROS_DEBUG("global SFM failed!");
        marginalization_flag = MARGIN_OLD;
        return false;
    }

    //solve pnp for all frame
    map<double, ImageFrame>::iterator frame_it;
    map<int, Vector3d>::iterator it;
    frame_it = all_image_frame.begin( );
    for (int i = 0; frame_it != all_image_frame.end( ); frame_it++)
    {
        // provide initial guess
        cv::Mat r, rvec, t, D, tmp_r;
        if((frame_it->first) == Headers[i])
        {
            frame_it->second.is_key_frame = true;
            frame_it->second.R = Q[i].toRotationMatrix() * RIC[0].transpose();
            frame_it->second.T = T[i];
            i++;
            continue;
        }
        if((frame_it->first) > Headers[i])
        {
            i++;
        }
        Matrix3d R_inital = (Q[i].inverse()).toRotationMatrix();
        Vector3d P_inital = - R_inital * T[i];
        cv::eigen2cv(R_inital, tmp_r);
        cv::Rodrigues(tmp_r, rvec);
        cv::eigen2cv(P_inital, t);

        frame_it->second.is_key_frame = false;
        vector<cv::Point3f> pts_3_vector;
        vector<cv::Point2f> pts_2_vector;
        for (auto &id_pts : frame_it->second.points)
        {
            int feature_id = id_pts.first;
            for (auto &i_p : id_pts.second)
            {
                it = sfm_tracked_points.find(feature_id);
                if(it != sfm_tracked_points.end())
                {
                    Vector3d world_pts = it->second;
                    cv::Point3f pts_3(world_pts(0), world_pts(1), world_pts(2));
                    pts_3_vector.push_back(pts_3);
                    Vector2d img_pts = i_p.second.head<2>();
                    cv::Point2f pts_2(img_pts(0), img_pts(1));
                    pts_2_vector.push_back(pts_2);
                }
            }
        }
        cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);     
        if(pts_3_vector.size() < 6)
        {
            cout << "pts_3_vector size " << pts_3_vector.size() << endl;
            ROS_DEBUG("Not enough points for solve pnp !");
            return false;
        }
        if (! cv::solvePnP(pts_3_vector, pts_2_vector, K, D, rvec, t, 1))
        {
            ROS_DEBUG("solve pnp fail!");
            return false;
        }
        cv::Rodrigues(rvec, r);
        MatrixXd R_pnp,tmp_R_pnp;
        cv::cv2eigen(r, tmp_R_pnp);
        R_pnp = tmp_R_pnp.transpose();
        MatrixXd T_pnp;
        cv::cv2eigen(t, T_pnp);
        T_pnp = R_pnp * (-T_pnp);
        frame_it->second.R = R_pnp * RIC[0].transpose();
        frame_it->second.T = T_pnp;
    }
    if (visualInitialAlign())
        return true;
    else
    {
        ROS_INFO("misalign visual structure with IMU");
        return false;
    }

}

bool Estimator::visualInitialAlign()
{
    TicToc t_g;
    VectorXd x;
    //solve scale
    bool result = VisualIMUAlignment(all_image_frame, Bgs, g, x);
    if(!result)
    {
        ROS_DEBUG("solve g failed!");
        return false;
    }

    // change state
    for (int i = 0; i <= frame_count; i++)
    {
        Matrix3d Ri = all_image_frame[Headers[i]].R;
        Vector3d Pi = all_image_frame[Headers[i]].T;
        Ps[i] = Pi;
        Rs[i] = Ri;
        all_image_frame[Headers[i]].is_key_frame = true;
    }

    double s = (x.tail<1>())(0);
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        pre_integrations[i]->repropagate(Vector3d::Zero(), Bgs[i]);
    }
    for (int i = frame_count; i >= 0; i--)
        Ps[i] = s * Ps[i] - Rs[i] * TIC[0] - (s * Ps[0] - Rs[0] * TIC[0]);
    int kv = -1;
    map<double, ImageFrame>::iterator frame_i;
    for (frame_i = all_image_frame.begin(); frame_i != all_image_frame.end(); frame_i++)
    {
        if(frame_i->second.is_key_frame)
        {
            kv++;
            Vs[kv] = frame_i->second.R * x.segment<3>(kv * 3);
        }
    }

    Matrix3d R0 = Utility::g2R(g);
    double yaw = Utility::R2ypr(R0 * Rs[0]).x();
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
    g = R0 * g;
    //Matrix3d rot_diff = R0 * Rs[0].transpose();
    Matrix3d rot_diff = R0;
    for (int i = 0; i <= frame_count; i++)
    {
        Ps[i] = rot_diff * Ps[i];
        Rs[i] = rot_diff * Rs[i];
        Vs[i] = rot_diff * Vs[i];
    }
    ROS_DEBUG_STREAM("g0     " << g.transpose());
    ROS_DEBUG_STREAM("my R0  " << Utility::R2ypr(Rs[0]).transpose()); 

    f_manager.clearDepth();
    f_manager.triangulate(frame_count, Ps, Rs, tic, ric);

    return true;
}

bool Estimator::relativePose(Matrix3d &relative_R, Vector3d &relative_T, int &l)
{
    // find previous frame which contians enough correspondance and parallex with newest frame
    for (int i = 0; i < WINDOW_SIZE; i++)
    {
        vector<pair<Vector3d, Vector3d>> corres;
        corres = f_manager.getCorresponding(i, WINDOW_SIZE);
        if (corres.size() > 20)
        {
            double sum_parallax = 0;
            double average_parallax;
            for (int j = 0; j < int(corres.size()); j++)
            {
                Vector2d pts_0(corres[j].first(0), corres[j].first(1));
                Vector2d pts_1(corres[j].second(0), corres[j].second(1));
                double parallax = (pts_0 - pts_1).norm();
                sum_parallax = sum_parallax + parallax;

            }
            average_parallax = 1.0 * sum_parallax / int(corres.size());
            if(average_parallax * 460 > 30 && m_estimator.solveRelativeRT(corres, relative_R, relative_T))
            {
                l = i;
                ROS_DEBUG("average_parallax %f choose l %d and newest frame to triangulate the whole structure", average_parallax * 460, l);
                return true;
            }
        }
    }
    return false;
}

void Estimator::vector2double()
{
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        para_Pose[i][0] = Ps[i].x();
        para_Pose[i][1] = Ps[i].y();
        para_Pose[i][2] = Ps[i].z();
        Quaterniond q{Rs[i]};
        para_Pose[i][3] = q.x();
        para_Pose[i][4] = q.y();
        para_Pose[i][5] = q.z();
        para_Pose[i][6] = q.w();

        if(USE_IMU)
        {
            para_SpeedBias[i][0] = Vs[i].x();
            para_SpeedBias[i][1] = Vs[i].y();
            para_SpeedBias[i][2] = Vs[i].z();

            para_SpeedBias[i][3] = Bas[i].x();
            para_SpeedBias[i][4] = Bas[i].y();
            para_SpeedBias[i][5] = Bas[i].z();

            para_SpeedBias[i][6] = Bgs[i].x();
            para_SpeedBias[i][7] = Bgs[i].y();
            para_SpeedBias[i][8] = Bgs[i].z();
        }
    }

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        para_Ex_Pose[i][0] = tic[i].x();
        para_Ex_Pose[i][1] = tic[i].y();
        para_Ex_Pose[i][2] = tic[i].z();
        Quaterniond q{ric[i]};
        para_Ex_Pose[i][3] = q.x();
        para_Ex_Pose[i][4] = q.y();
        para_Ex_Pose[i][5] = q.z();
        para_Ex_Pose[i][6] = q.w();
    }


    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < f_manager.getFeatureCount(); i++)
        para_Feature[i][0] = dep(i);

    para_Td[0][0] = td;
}

void Estimator::double2vector()
{
    Vector3d origin_R0 = Utility::R2ypr(Rs[0]);
    Vector3d origin_P0 = Ps[0];

    if (failure_occur)
    {
        origin_R0 = Utility::R2ypr(last_R0);
        origin_P0 = last_P0;
        failure_occur = 0;
    }

    if(USE_IMU)
    {
        Vector3d origin_R00 = Utility::R2ypr(Quaterniond(para_Pose[0][6],
                                                          para_Pose[0][3],
                                                          para_Pose[0][4],
                                                          para_Pose[0][5]).toRotationMatrix());
        double y_diff = origin_R0.x() - origin_R00.x();
        //TODO
        Matrix3d rot_diff = Utility::ypr2R(Vector3d(y_diff, 0, 0));
        if (abs(abs(origin_R0.y()) - 90) < 1.0 || abs(abs(origin_R00.y()) - 90) < 1.0)
        {
            ROS_DEBUG("euler singular point!");
            rot_diff = Rs[0] * Quaterniond(para_Pose[0][6],
                                           para_Pose[0][3],
                                           para_Pose[0][4],
                                           para_Pose[0][5]).toRotationMatrix().transpose();
        }

        // Measure the actual native restoration, including legacy mode.
        const Vector3d optimized_p0(para_Pose[0][0],para_Pose[0][1],para_Pose[0][2]);
        const Vector3d applied_translation = origin_P0 - rot_diff * optimized_p0;
        std::ostringstream gauge;gauge << std::setprecision(12)
            << "{\"sampled\":true,\"source_stamp_sim_s\":" << Headers[frame_count]
            << ",\"snapshot_phase\":\"after_solve_before_slide_native_gauge_restoration\""
            << ",\"first_pose_xyz_yaw_fixed\":" << (FIX_FIRST_POSE_XYZ_YAW ? "true" : "false")
            << ",\"retraction\":\"" << (FIX_FIRST_POSE_XYZ_YAW ? "bounded_h_tanh_pitch_cayley_roll" : "legacy_free_pose") << "\""
            << ",\"first_pose_pitch_before_solve_rad\":" << origin_R0.y() * M_PI / 180.
            << ",\"first_pose_pitch_after_solve_rad\":" << origin_R00.y() * M_PI / 180.
            << ",\"first_pose_roll_before_solve_rad\":" << origin_R0.z() * M_PI / 180.
            << ",\"first_pose_roll_after_solve_rad\":" << origin_R00.z() * M_PI / 180.
            << ",\"rp_radians_per_optimizer_local_unit\":" << (FIX_FIRST_POSE_XYZ_YAW ? FIXED_YAW_RP_RADIANS_PER_LOCAL_UNIT : 1.)
            << ",\"first_pose_optimizer_local_size\":" << (FIX_FIRST_POSE_XYZ_YAW ? 2 : 6)
            << ",\"first_pose_yaw_before_solve_rad\":" << origin_R0.x() * M_PI / 180.
            << ",\"first_pose_yaw_after_solve_rad\":" << origin_R00.x() * M_PI / 180.
            << ",\"actual_yaw_restoration_rad\":" << std::atan2(rot_diff(1,0),rot_diff(0,0))
            << ",\"actual_rotation_restoration_angle_rad\":" << Eigen::AngleAxisd(rot_diff).angle()
            << ",\"first_pose_position_displacement_m\":" << (optimized_p0-origin_P0).norm()
            << ",\"actual_translation_restoration_norm_m\":" << applied_translation.norm() << '}';
        native_gauge_evidence_json = gauge.str();

        for (int i = 0; i <= WINDOW_SIZE; i++)
        {

            Rs[i] = rot_diff * Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]).normalized().toRotationMatrix();
            
            Ps[i] = rot_diff * Vector3d(para_Pose[i][0] - para_Pose[0][0],
                                    para_Pose[i][1] - para_Pose[0][1],
                                    para_Pose[i][2] - para_Pose[0][2]) + origin_P0;


                Vs[i] = rot_diff * Vector3d(para_SpeedBias[i][0],
                                            para_SpeedBias[i][1],
                                            para_SpeedBias[i][2]);

                Bas[i] = Vector3d(para_SpeedBias[i][3],
                                  para_SpeedBias[i][4],
                                  para_SpeedBias[i][5]);

                Bgs[i] = Vector3d(para_SpeedBias[i][6],
                                  para_SpeedBias[i][7],
                                  para_SpeedBias[i][8]);
            
        }
    }
    else
    {
        for (int i = 0; i <= WINDOW_SIZE; i++)
        {
            Rs[i] = Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]).normalized().toRotationMatrix();
            
            Ps[i] = Vector3d(para_Pose[i][0], para_Pose[i][1], para_Pose[i][2]);
        }
    }

    if(USE_IMU)
    {
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            tic[i] = Vector3d(para_Ex_Pose[i][0],
                              para_Ex_Pose[i][1],
                              para_Ex_Pose[i][2]);
            ric[i] = Quaterniond(para_Ex_Pose[i][6],
                                 para_Ex_Pose[i][3],
                                 para_Ex_Pose[i][4],
                                 para_Ex_Pose[i][5]).normalized().toRotationMatrix();
        }
    }

    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < f_manager.getFeatureCount(); i++)
        dep(i) = para_Feature[i][0];
    f_manager.setDepth(dep);

    if(USE_IMU)
        td = para_Td[0][0];

}

void Estimator::updateStationaryEvidence(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, double stamp)
{
    stationary_evidence = stationary_detector.imuEvidence(stamp + td, G.norm());
    std::set<int> current_stereo_ids;
    std::vector<double> speeds;
    std::vector<StereoWindowTrack> window_tracks;
    std::ostringstream raw_tracks;
    raw_tracks.precision(12); raw_tracks << "[";
    bool first_track = true;
    const Eigen::Vector3d baseline = ric[0].transpose() * (tic[1] - tic[0]);
    const bool rectified = STEREO && (ric[0].transpose() * ric[1] - Eigen::Matrix3d::Identity()).norm() < 1e-3
        && baseline.x() > 0 && std::abs(baseline.y()) < 1e-3 && std::abs(baseline.z()) < 1e-3;
    for (const auto &entry : image)
    {
        const Eigen::Matrix<double, 7, 1> *left = NULL, *right = NULL;
        for (const auto &camera : entry.second)
        {
            if (camera.first == 0) left = &camera.second;
            if (camera.first == 1) right = &camera.second;
        }
        if (!left || !right || !left->allFinite() || !right->allFinite()) continue;
        current_stereo_ids.insert(entry.first);
        double speed = 0;
        if (nearStereoStationarySupport(*left, *right, previous_stereo_ids.count(entry.first), rectified,
                baseline.x(), ZUPT_CAMERA_FX, stationary_detector.config.maximum_depth_m, &speed))
        {
            speeds.push_back(speed);
            window_tracks.push_back({entry.first, Eigen::Vector2d((*left)[3], (*left)[4]),
                Eigen::Vector2d((*right)[3], (*right)[4])});
            if (!first_track) raw_tracks << ",";
            first_track = false;
            raw_tracks << "{\"id\":" << entry.first << ",\"left_uv\":[" << (*left)[3] << "," << (*left)[4]
                << "],\"right_uv\":[" << (*right)[3] << "," << (*right)[4]
                << "],\"left_velocity_normalized\":[" << (*left)[5] << "," << (*left)[6]
                << "],\"right_velocity_normalized\":[" << (*right)[5] << "," << (*right)[6]
                << "],\"depth_m\":" << ZUPT_CAMERA_FX * baseline.x() / ((*left)[3] - (*right)[3]) << "}";
        }
    }
    raw_tracks << "]"; native_stereo_tracks_json = raw_tracks.str();
    previous_stereo_ids.swap(current_stereo_ids);
    stationary_evidence.near_stereo_tracks = static_cast<int>(speeds.size());
    if (!speeds.empty())
    {
        std::sort(speeds.begin(), speeds.end());
        stationary_evidence.instant_flow_median_px_s = speeds[(speeds.size() - 1) / 2];
        stationary_evidence.instant_flow_p90_px_s = speeds[static_cast<size_t>(std::ceil(0.9 * speeds.size())) - 1];
    }
    const StereoWindowEvidence visual_window = stationary_stereo_window.addFrame(stamp, window_tracks);
    stationary_evidence.window_supported_tracks = visual_window.supported_tracks;
    stationary_evidence.window_span_s = visual_window.minimum_span_s;
    stationary_evidence.flow_median_px_s = visual_window.flow_median_px_s;
    stationary_evidence.flow_p90_px_s = visual_window.flow_p90_px_s;
    stationary_evidence.fit_jitter_p90_px = visual_window.fit_jitter_p90_px;
    const bool sensor_stationary = stationary_detector.update(stamp, stationary_evidence);
    stationary_frames[frame_count] = ENABLE_ZUPT && USE_IMU && sensor_stationary;
    stationary_intervals[frame_count] = frame_count > 0 && stationary_frames[frame_count - 1]
        && stationary_frames[frame_count];
}

bool Estimator::validateOptimizedSolution()
{
    const int i = frame_count;
    if (!Ps[i].allFinite() || !Vs[i].allFinite() || !Rs[i].allFinite()
        || !Bas[i].allFinite() || !Bgs[i].allFinite())
    { solver_health_reason = "nonfinite_optimized_state"; return false; }
    if (have_validated_optimized && Headers[i] > last_validated_optimized_t)
    {
        const double dt = Headers[i] - last_validated_optimized_t;
        const Eigen::Vector3d dp = Ps[i] - last_validated_optimized_p;
        const double angle = Eigen::AngleAxisd(last_validated_optimized_r.transpose() * Rs[i]).angle();
        if (!usableOptimizedKinematics(dt, dp, last_validated_optimized_v, Vs[i], angle))
        {
            solver_health_reason = "optimized_pose_velocity_inconsistent";
            ROS_WARN("VIO optimized kinematics invalid: t=%.6f dt=%.6f dp=%.6f integrated_error=%.6f angle=%.6f",
                Headers[i], dt, dp.norm(), (dp - (last_validated_optimized_v + Vs[i]) * (0.5 * dt)).norm(), angle);
            return false;
        }
    }
    else if (have_validated_optimized && Headers[i] < last_validated_optimized_t)
    { solver_health_reason = "optimized_stamp_regressed"; return false; }
    have_validated_optimized = true;
    last_validated_optimized_t = Headers[i];
    last_validated_optimized_p = Ps[i]; last_validated_optimized_v = Vs[i]; last_validated_optimized_r = Rs[i];
    return true;
}

bool Estimator::validateInitialWindowMapping(std::string &reason) const
{
    if (frame_count != WINDOW_SIZE)
    { reason = "partial_camera_window"; return false; }
    std::vector<double> headers, keys, window_spans, image_spans;
    for (const auto &entry : all_image_frame) keys.push_back(entry.first);
    for (int i = 0; i <= WINDOW_SIZE; ++i)
    {
        headers.push_back(Headers[i]);
        window_spans.push_back(pre_integrations[i] ? pre_integrations[i]->sum_dt : -1);
        const auto found = all_image_frame.find(Headers[i]);
        image_spans.push_back(found != all_image_frame.end() && found->second.pre_integration
            ? found->second.pre_integration->sum_dt : -1);
    }
    return validateInitialHeaderEdges(headers, keys, window_spans, image_spans, reason);
}

void Estimator::snapshotInitializationEvidence(const std::string &phase, const std::string &reason)
{
    std::ostringstream o; o.precision(12);
    o << "{\"phase\":\"" << phase << "\",\"reason\":\"" << reason
      << "\",\"snapshot_phase\":\"before_slide\",\"source_stamp_sim_s\":" << Headers[frame_count]
      << ",\"frame_count\":" << frame_count << ",\"seeded\":" << (stationary_initialization_seeded ? "true" : "false")
      << ",\"seed_count\":" << stationary_initialization_seed_count << ",\"wait_count\":" << stationary_initialization_wait_count
      << ",\"initial_bias_prior_insertion_count\":" << initial_bias_prior_insertion_count
      << ",\"existing_valid_marginal_prior\":" << (last_marginalization_info && last_marginalization_info->valid ? "true" : "false")
      << ",\"current_source_gate\":" << (stationary_detector.instant_gate ? "true" : "false")
      << ",\"current_stationary_confirmed\":" << (stationary_detector.active ? "true" : "false")
      << ",\"headers_sim_s\":[";
    for (int i=0; i<=frame_count; ++i) { if(i) o << ","; o << Headers[i]; }
    o << "],\"stationary_frames\":[";
    for (int i=0; i<=frame_count; ++i) { if(i) o << ","; o << (stationary_frames[i] ? "true" : "false"); }
    o << "],\"stationary_intervals\":[";
    for (int i=0; i<=frame_count; ++i) { if(i) o << ","; o << (stationary_intervals[i] ? "true" : "false"); }
    o << "],\"image_map_keys_sim_s\":["; bool comma=false;
    for (const auto &entry : all_image_frame) { if(comma) o << ","; comma=true; o << entry.first; }
    o << "],\"map_size\":" << all_image_frame.size()
      << ",\"seed_prior_was_present\":false"
      << ",\"seed_source_stamp_sim_s\":" << stationary_initialization_seed_stamp
      << ",\"seed_imu_first_source_stamp_sim_s\":" << stationary_initialization_seed_first_imu_stamp
      << ",\"seed_imu_last_source_stamp_sim_s\":" << stationary_initialization_seed_last_imu_stamp
      << ",\"seed_imu_samples\":" << stationary_initialization_seed_imu_samples
      << ",\"seed_acc_mean_m_s2\":[" << stationary_initialization_seed_acc.x() << "," << stationary_initialization_seed_acc.y() << "," << stationary_initialization_seed_acc.z()
      << "],\"seed_gyro_mean_rad_s\":[" << stationary_initialization_seed_gyro.x() << "," << stationary_initialization_seed_gyro.y() << "," << stationary_initialization_seed_gyro.z()
      << "],\"seed_semantics\":\"once_only_sensor_supported_initial_guess_no_new_factor_or_constant_rp\"}";
    native_initialization_evidence_json = o.str();
}

bool Estimator::prepareStationaryInitialization()
{
    std::string reason;
    if (!validateInitialWindowMapping(reason))
    {
        solver_solution_usable = false; failure_occur = true; solver_health_reason = reason;
        snapshotInitializationEvidence("rejected", reason); return false;
    }
    if (stationary_initialization_seeded || (last_marginalization_info && last_marginalization_info->valid))
    {
        solver_solution_usable = false; failure_occur = true;
        solver_health_reason = "initial_seed_would_overwrite_existing_prior_or_seed";
        snapshotInitializationEvidence("rejected", solver_health_reason); return false;
    }
    if (!confirmedStationaryInitialWindow(stationary_frames, stationary_intervals, WINDOW_SIZE,
            stationary_detector.instant_gate, stationary_detector.active, reason))
    {
        ++stationary_initialization_wait_count;
        snapshotInitializationEvidence("waiting", reason); return false;
    }
    // Select actual raw samples spanning exactly this source camera window.
    // The existing detector already confirmed all frames and intervals using
    // near positive-disparity stereo plus low-variance original IMU.
    const double first = Headers[0] + td, last = Headers[WINDOW_SIZE] + td;
    Eigen::Vector3d acc_mean = Eigen::Vector3d::Zero(), gyro_mean = Eigen::Vector3d::Zero();
    double first_sample=-1, last_sample=-1, previous=-1; int count=0; bool gap=false;
    for (const auto &sample : stationary_detector.imu)
    {
        if (sample.t < first - 1e-6 || sample.t > last + 0.005) continue;
        if (!std::isfinite(sample.t) || !sample.acc.allFinite() || !sample.gyro.allFinite()) { gap=true; break; }
        if (previous >= 0 && (sample.t <= previous || sample.t - previous > 0.012)) gap=true;
        if (!count) first_sample=sample.t;
        last_sample=sample.t; previous=sample.t; ++count; acc_mean+=sample.acc; gyro_mean+=sample.gyro;
    }
    if (gap || count < stationary_detector.config.minimum_imu_samples || first_sample > first+0.012
        || last_sample < last-0.012 || last_sample-first_sample < last-first-0.012)
    {
        ++stationary_initialization_wait_count;
        snapshotInitializationEvidence("waiting", "raw_imu_does_not_cover_complete_camera_window"); return false;
    }
    acc_mean /= count; gyro_mean /= count;
    if (!acc_mean.allFinite() || !gyro_mean.allFinite() || acc_mean.norm() < 1)
    {
        solver_solution_usable=false; failure_occur=true; solver_health_reason="invalid_stationary_seed_source";
        snapshotInitializationEvidence("rejected", solver_health_reason); return false;
    }
    // A common world rotation preserves every relative visual geometry. Yaw
    // and translation choose coordinates; all RP variables remain Ceres-free.
    const Eigen::Matrix3d from_acc = Utility::g2R(acc_mean);
    const double previous_yaw = Utility::R2ypr(Rs[0]).x();
    const double acc_yaw = Utility::R2ypr(from_acc).x();
    const Eigen::Matrix3d desired = Utility::ypr2R(Eigen::Vector3d(previous_yaw-acc_yaw,0,0))*from_acc;
    const Eigen::Matrix3d common_rotation=desired*Rs[0].transpose();
    const Eigen::Vector3d origin=Ps[0];
    for (int i=0; i<=WINDOW_SIZE; ++i)
    {
        Rs[i]=common_rotation*Rs[i]; Ps[i]=origin+common_rotation*(Ps[i]-origin);
        Vs[i].setZero(); Bas[i].setZero(); Bgs[i]=gyro_mean;
        ImageFrame &frame=all_image_frame.at(Headers[i]); frame.R=Rs[i]; frame.T=Ps[i];
        if (pre_integrations[i]) pre_integrations[i]->repropagate(Bas[i], Bgs[i]);
        if (frame.pre_integration) frame.pre_integration->repropagate(Bas[i], Bgs[i]);
    }
    if (tmp_pre_integration) tmp_pre_integration->repropagate(Vector3d::Zero(), gyro_mean);
    stationary_initialization_seed_acc=acc_mean; stationary_initialization_seed_gyro=gyro_mean;
    stationary_initialization_seed_first_imu_stamp=first_sample; stationary_initialization_seed_last_imu_stamp=last_sample;
    stationary_initialization_seed_imu_samples=count; stationary_initialization_seed_stamp=Headers[WINDOW_SIZE];
    stationary_initialization_seeded=true; ++stationary_initialization_seed_count;
    snapshotInitializationEvidence("seeded_before_first_solve", "complete_source_stationary_window");
    // Preserve the actual source-supported seed while prior is still absent.
    // File evidence only: INITIAL remains false and no ROS/EV pose is emitted.
    appendNativeHealthJson(nativeHealthJson(Headers[WINDOW_SIZE], "seed_before_first_solve_and_slide"));
    ROS_INFO("V9 stationary INITIAL seed once at %.6f: source IMU [%.6f,%.6f] n=%d; no prior yet",
        Headers[WINDOW_SIZE], first_sample, last_sample, count);
    return true;
}

void Estimator::appendNativeHealthJson(const std::string &data)
{
    if (!STATIONARY_INITIALIZATION) return;
    if (!native_health_trace.is_open())
        native_health_trace.open(OUTPUT_FOLDER + "/native_health.jsonl", std::ios::out | std::ios::app);
    if (native_health_trace && data.size() <= 512*1024)
    {
        native_health_trace << data << '\n'; native_health_trace.flush();
        if (native_health_trace) { ++native_health_trace_records; return; }
    }
    if (!native_health_trace_error) ROS_ERROR("Native startup health trace cannot be written to %s", OUTPUT_FOLDER.c_str());
    native_health_trace_error=true; // never declare a complete initialization trace on failed IO
}

std::string Estimator::nativeHealthJson(double stamp, const std::string &state_phase) const
{
    std::ostringstream o;
    o.precision(12);
    const auto vector_json = [](const Eigen::Vector3d& vector) {
        std::ostringstream a; a.precision(12); a << "[";
        for (int axis = 0; axis < 3; ++axis)
        { if (axis) a << ","; if (std::isfinite(vector[axis])) a << vector[axis]; else a << "null"; }
        a << "]"; return a.str();
    };
    const Eigen::Quaterniond q(Rs[frame_count]);
    std::ostringstream quaternion_json; quaternion_json.precision(12); quaternion_json << "[";
    const double quaternion_values[4] = {q.x(), q.y(), q.z(), q.w()};
    for (int axis = 0; axis < 4; ++axis)
    { if (axis) quaternion_json << ","; if (std::isfinite(quaternion_values[axis])) quaternion_json << quaternion_values[axis]; else quaternion_json << "null"; }
    quaternion_json << "]";
    // Sentinel values are finite, keeping diagnostics valid JSON even before
    // the first IMU window or stereo pair has arrived.
    o << "{\"implementation_revision\":\"stationary_health_v9\",\"marginalization_pose_jacobian_mode\":\"" << (EXACT_MARGINALIZATION_POSE_JACOBIAN ? "exact_residual_chain_rule" : "legacy_frozen_local") << "\",\"active_prior_pose_jacobian_mode\":\"" << (last_marginalization_info && last_marginalization_info->valid ? (last_marginalization_info->exact_pose_jacobian ? "exact_residual_chain_rule" : "legacy_frozen_local") : "none") << "\",\"stamp\":" << stamp << ",\"tracking_ok\":" << ((solver_flag == NON_LINEAR && !failure_occur) ? "true" : "false")
      << ",\"solution_usable\":" << (solver_solution_usable ? "true" : "false")
      << ",\"reason\":\"" << solver_health_reason << "\",\"initial_cost\":" << solver_initial_cost
      << ",\"final_cost\":" << solver_final_cost << ",\"residual_rms\":" << solver_residual_rms
      << ",\"solver_iterations\":" << solver_iterations << ",\"solver_successful_steps\":" << solver_successful_steps
      << ",\"solver_unsuccessful_steps\":" << solver_unsuccessful_steps << ",\"solver_termination_type\":" << solver_termination_type
      << ",\"solver_wall_time_s\":" << solver_wall_time_s << ",\"solver_budget_s\":" << solver_budget_s
      << ",\"solver_last_gradient_max_norm\":" << solver_last_gradient_max_norm
      << ",\"solver_gradient_norm_semantics\":\"ceres_projected_ambient_step_norm_not_physical_rp_gradient\""
      << ",\"current_state_stamp_sim_s\":" << Headers[frame_count]
      << ",\"current_state_snapshot_phase\":\"" << state_phase << "\""
      << ",\"current_position_world_m\":" << vector_json(Ps[frame_count])
      << ",\"current_velocity_world_m_s\":" << vector_json(Vs[frame_count])
      << ",\"current_orientation_xyzw\":" << quaternion_json.str()
      << ",\"current_acc_bias_m_s2\":" << vector_json(Bas[frame_count])
      << ",\"current_gyro_bias_rad_s\":" << vector_json(Bgs[frame_count])
      << ",\"estimator_gravity_world_m_s2\":" << vector_json(g)
      << ",\"stationary_initialization_enabled\":" << (STATIONARY_INITIALIZATION ? "true" : "false")
      << ",\"initialization\":" << native_initialization_evidence_json
      << ",\"native_health_trace_records_before_current\":" << native_health_trace_records
      << ",\"native_health_trace_error\":" << (native_health_trace_error ? "true" : "false")
      << ",\"imu_optimizer_window\":" << native_imu_optimizer_window_json
      << ",\"bias_information_diagnostics\":" << native_bias_diagnostics_json
      << ",\"native_coordinate_gauge\":" << native_gauge_evidence_json
      << ",\"zupt_enabled\":" << (ENABLE_ZUPT ? "true" : "false")
      << ",\"stationary_source_gate\":" << (stationary_detector.instant_gate ? "true" : "false")
      << ",\"stationary_confirmed\":" << ((ENABLE_ZUPT && USE_IMU && stationary_detector.active) ? "true" : "false")
      << ",\"stationary_support_s\":" << (stationary_detector.supported_since < 0 ? 0 : stamp - stationary_detector.supported_since)
      << ",\"imu_samples\":" << stationary_evidence.imu_samples << ",\"imu_span_s\":" << stationary_evidence.imu_span_s
      << ",\"imu_window_first_source_stamp_sim_s\":" << stationary_evidence.imu_first_source_stamp_s
      << ",\"imu_window_last_source_stamp_sim_s\":" << stationary_evidence.imu_last_source_stamp_s
      << ",\"imu_window_acc_mean_m_s2\":" << vector_json(stationary_evidence.acc_mean)
      << ",\"imu_window_gyro_mean_rad_s\":" << vector_json(stationary_evidence.gyro_mean)
      << ",\"imu_window_acc_std_m_s2\":" << vector_json(stationary_evidence.acc_std)
      << ",\"imu_window_gyro_std_rad_s\":" << vector_json(stationary_evidence.gyro_std)
      << ",\"imu_window_gyro_mean_minus_estimated_bias_rad_s\":" << vector_json(stationary_evidence.gyro_mean - Bgs[frame_count])
      << ",\"imu_age_s\":" << stationary_evidence.imu_age_s << ",\"gyro_mean_norm\":" << stationary_evidence.gyro_mean_norm
      << ",\"gyro_std_norm\":" << stationary_evidence.gyro_std_norm << ",\"acc_std_norm\":" << stationary_evidence.acc_std_norm
      << ",\"gravity_norm_error\":" << stationary_evidence.gravity_norm_error << ",\"near_stereo_tracks\":" << stationary_evidence.near_stereo_tracks
      << ",\"flow_p90_px_s\":" << stationary_evidence.instant_flow_p90_px_s << ",\"flow_median_px_s\":" << stationary_evidence.instant_flow_median_px_s
      << ",\"window_supported_tracks\":" << stationary_evidence.window_supported_tracks << ",\"window_span_s\":" << stationary_evidence.window_span_s
      << ",\"window_flow_p90_px_s\":" << stationary_evidence.flow_p90_px_s << ",\"window_flow_median_px_s\":" << stationary_evidence.flow_median_px_s
      << ",\"window_fit_jitter_p90_px\":" << stationary_evidence.fit_jitter_p90_px
      << ",\"propagation_refresh_checked\":" << (propagation_refresh_checked ? "true" : "false")
      << ",\"refresh_imu_delta_m\":" << propagation_refresh_evidence.imu_position_delta_m
      << ",\"refresh_body_delta_m\":" << propagation_refresh_evidence.body_position_delta_m
      << ",\"refresh_angle_rad\":" << propagation_refresh_evidence.orientation_delta_rad
      << ",\"refresh_comparison_stamp_sim_s\":" << propagation_refresh_evidence.comparison_stamp_sim_s
      << ",\"refresh_previous_published_stamp_sim_s\":" << propagation_refresh_evidence.previous_published_stamp_sim_s
      << ",\"refresh_candidate_base_stamp_sim_s\":" << propagation_refresh_evidence.candidate_base_stamp_sim_s
      << ",\"refresh_pending_old_forward_dt_s\":" << propagation_refresh_evidence.pending_old_forward_dt_s
      << ",\"visual_lifecycle_diagnostics\":" << native_visual_lifecycle.json(VISUAL_LIFECYCLE_DIAGNOSTICS != 0, visual_gap_since)
      << ",\"stereo_tracks\":" << native_stereo_tracks_json;
    if (!stationary_detector.imu.empty())
        o << ",\"current_raw_imu_source_stamp_sim_s\":" << stationary_detector.imu.back().t
          << ",\"current_raw_acc_m_s2\":" << vector_json(stationary_detector.imu.back().acc)
          << ",\"current_raw_gyro_rad_s\":" << vector_json(stationary_detector.imu.back().gyro);
    o << "}";
    return o.str();
}

bool Estimator::failureDetection()
{
    // Limits suit the agent's 1 m/s, 30 m radius simulation missions.
    if (f_manager.getFeatureCount() < 10)
    {
        if (visual_gap_since < 0) visual_gap_since = Headers[WINDOW_SIZE];
    }
    else visual_gap_since = -1;
    bool visual_lost = visual_gap_since >= 0 && Headers[WINDOW_SIZE] - visual_gap_since > 1.5;
    if (!Ps[WINDOW_SIZE].allFinite() || !Vs[WINDOW_SIZE].allFinite()
        || !Rs[WINDOW_SIZE].allFinite() || !Bas[WINDOW_SIZE].allFinite()
        || !Bgs[WINDOW_SIZE].allFinite() || Bas[WINDOW_SIZE].norm() > 2.5
        || Bgs[WINDOW_SIZE].norm() > 1.0 || Vs[WINDOW_SIZE].norm() > 3.0
        || Ps[WINDOW_SIZE].norm() > 100.0 || visual_lost)
    {
        solver_health_reason = "native_limits_or_visual_tracking_lost";
        ROS_WARN("VIO invalid: features=%d, speed=%.3f, acc_bias=%.3f, gyro_bias=%.3f",
            f_manager.getFeatureCount(), Vs[WINDOW_SIZE].norm(),
            Bas[WINDOW_SIZE].norm(), Bgs[WINDOW_SIZE].norm());
        return true;
    }
    return false;
}

void Estimator::optimization()
{
    TicToc t_whole, t_prepare;
    if (VISUAL_LIFECYCLE_DIAGNOSTICS) beginVisualDiagnostics();
    solver_solution_usable = true;
    solver_health_reason.clear();
    vector2double();
    if (USE_IMU && FIX_FIRST_POSE_XYZ_YAW)
    {
        double yaw,pitch,roll;
        if (!fixedYawPoseZyx(para_Pose[0],yaw,pitch,roll))
        {
            solver_solution_usable = false;
            solver_health_reason = "first_pose_fixed_yaw_chart_invalid_or_singular";
            return;
        }
    }


    ceres::Problem problem;
    const bool add_initial_bias_prior = USE_IMU && ACC_BIAS_PRIOR_SIGMA > 0 && GYR_BIAS_PRIOR_SIGMA > 0
        && (!last_marginalization_info || !last_marginalization_info->valid);
    // Capture the exact topology and real integration durations supplied to
    // this solve, before OLD/SECOND_NEW changes any window slot. Membership
    // of a prior SpeedBias block is not a proof of bias-information rank.
    std::vector<bool> prior_roots(frame_count + 1, false), included_edges;
    if (add_initial_bias_prior) prior_roots[0] = true;
    if (last_marginalization_info && last_marginalization_info->valid)
        for (double* block : last_marginalization_parameter_blocks)
            for (int i = 0; i <= frame_count; ++i)
                if (block == para_SpeedBias[i]) prior_roots[i] = true;
    std::ostringstream chain; chain.precision(12);
    chain << "{\"snapshot_phase\":\"before_solve_and_slide\",\"solve_image_stamp_sim_s\":" << Headers[frame_count]
          << ",\"marginalization_flag\":" << static_cast<int>(marginalization_flag)
          << ",\"visual_keyframe_requested\":" << (keyframe_visual_requested ? "true" : "false")
          << ",\"keyframe_forced_by_imu_span\":" << (keyframe_forced_by_imu_span ? "true" : "false")
          << ",\"candidate_merged_imu_span_s\":" << keyframe_candidate_merged_imu_span_s
          << ",\"configured_span_bound_s\":" << MAX_IMU_PREINTEGRATION_SPAN_S
          << ",\"initial_bias_prior_added_this_solve\":" << (add_initial_bias_prior ? "true" : "false")
          << ",\"marginalization_prior_valid_before_solve\":" << ((last_marginalization_info && last_marginalization_info->valid) ? "true" : "false")
          << ",\"prior_information_rank_inferred\":false,\"prior_speedbias_parameter_indices\":[";
    bool first_prior = true;
    for (int i = 0; i <= frame_count; ++i) if (prior_roots[i])
    { if (!first_prior) chain << ","; chain << i; first_prior = false; }
    chain << "],\"camera_state_stamps_sim_s\":[";
    for (int i = 0; i <= frame_count; ++i) { if (i) chain << ","; chain << Headers[i]; }
    chain << "],\"preintegration_intervals\":[";
    int included_imu_count = 0, skipped_imu_count = 0;
    for (int j = 1; j <= frame_count && USE_IMU; ++j)
    {
        const bool included = pre_integrations[j]->sum_dt <= 10.0;
        included_edges.push_back(included);
        included ? ++included_imu_count : ++skipped_imu_count;
        if (j > 1) chain << ",";
        chain << "{\"from_camera_stamp_sim_s\":" << Headers[j-1]
              << ",\"to_camera_stamp_sim_s\":" << Headers[j]
              << ",\"actual_integrated_imu_span_s\":" << pre_integrations[j]->sum_dt
              << ",\"original_sample_step_count\":" << pre_integrations[j]->dt_buf.size()
              << ",\"optimization_factor_included\":" << (included ? "true" : "false") << "}";
    }
    const bool old_bridge_included = USE_IMU && marginalization_flag == MARGIN_OLD
        && frame_count == WINDOW_SIZE && pre_integrations[1]->sum_dt < 10.0;
    chain << "],\"included_imu_factor_count\":" << included_imu_count << ",\"skipped_imu_factor_count\":" << skipped_imu_count
          << ",\"prior_speedbias_topology_reaches_latest_via_imu\":" << (priorReachesLatestViaImu(prior_roots, included_edges) ? "true" : "false")
          << ",\"old_marginalization_imu_bridge_included\":" << (old_bridge_included ? "true" : "false")
          << ",\"old_marginalization_bridge_skipped_due_span\":" << ((USE_IMU && marginalization_flag == MARGIN_OLD
              && frame_count == WINDOW_SIZE && pre_integrations[1]->sum_dt >= 10.0) ? "true" : "false") << "}";
    native_imu_optimizer_window_json = chain.str();
    ceres::LossFunction *loss_function;
    //loss_function = NULL;
    loss_function = new ceres::HuberLoss(1.0);
    //loss_function = new ceres::CauchyLoss(1.0 / FOCAL_LENGTH);
    //ceres::LossFunction* loss_function = new ceres::HuberLoss(1.0);
    for (int i = 0; i < frame_count + 1; i++)
    {
        ceres::LocalParameterization *local_parameterization = USE_IMU && FIX_FIRST_POSE_XYZ_YAW && i == 0
            ? static_cast<ceres::LocalParameterization *>(new FixedYawPoseParameterization(para_Pose[0]))
            : static_cast<ceres::LocalParameterization *>(new PoseLocalParameterization());
        problem.AddParameterBlock(para_Pose[i], SIZE_POSE, local_parameterization);
        if(USE_IMU)
            problem.AddParameterBlock(para_SpeedBias[i], SIZE_SPEEDBIAS);
    }
    if(!USE_IMU)
        problem.SetParameterBlockConstant(para_Pose[0]);
    if (add_initial_bias_prior)
    {
        ++initial_bias_prior_insertion_count;
        problem.AddResidualBlock(new ImuBiasPrior(ACC_BIAS_PRIOR_SIGMA, GYR_BIAS_PRIOR_SIGMA),
                                 NULL, para_SpeedBias[0]);
    }

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Ex_Pose[i], SIZE_POSE, local_parameterization);
        if ((ESTIMATE_EXTRINSIC && frame_count == WINDOW_SIZE && Vs[0].norm() > 0.2) || openExEstimation)
        {
            //ROS_INFO("estimate extinsic param");
            openExEstimation = 1;
        }
        else
        {
            //ROS_INFO("fix extinsic param");
            problem.SetParameterBlockConstant(para_Ex_Pose[i]);
        }
    }
    problem.AddParameterBlock(para_Td[0], 1);

    if (!ESTIMATE_TD || Vs[0].norm() < 0.2)
        problem.SetParameterBlockConstant(para_Td[0]);

    if (last_marginalization_info && last_marginalization_info->valid)
    {
        // construct new marginlization_factor
        MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
        problem.AddResidualBlock(marginalization_factor, NULL,
                                 last_marginalization_parameter_blocks);
    }
    if(USE_IMU)
    {
        for (int i = 0; i < frame_count; i++)
        {
            int j = i + 1;
            if (pre_integrations[j]->sum_dt > 10.0)
                continue;
            IMUFactor* imu_factor = new IMUFactor(pre_integrations[j]);
            problem.AddResidualBlock(imu_factor, NULL, para_Pose[i], para_SpeedBias[i], para_Pose[j], para_SpeedBias[j]);
        }
    }

    // Each window state retains its own sensor evidence; current stationarity
    // never imposes a zero-velocity observation on earlier moving states.
    if (ENABLE_ZUPT && USE_IMU)
    {
        const StationaryFactorCounts counts = addStationaryFactors(problem, stationary_frames, stationary_intervals,
            para_Pose, para_SpeedBias, frame_count, ZUPT_VELOCITY_SIGMA, ZUPT_POSITION_SIGMA);
        ROS_DEBUG("Sensor-supported stationary factors: velocity=%d relative_position=%d",
            counts.zero_velocity, counts.relative_position);
    }

    int f_m_cnt = 0;
    int feature_index = -1;
    for (auto &it_per_id : f_manager.feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (it_per_id.used_num < 4)
            continue;
 
        ++feature_index;

        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
        
        Vector3d pts_i = it_per_id.feature_per_frame[0].point;

        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            if (imu_i != imu_j)
            {
                Vector3d pts_j = it_per_frame.point;
                ProjectionTwoFrameOneCamFactor *f_td = new ProjectionTwoFrameOneCamFactor(pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                                                                 it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                problem.AddResidualBlock(f_td, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index], para_Td[0]);
            }

            if(STEREO && it_per_frame.is_stereo)
            {                
                Vector3d pts_j_right = it_per_frame.pointRight;
                if(imu_i != imu_j)
                {
                    ProjectionTwoFrameTwoCamFactor *f = new ProjectionTwoFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                 it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                    problem.AddResidualBlock(f, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]);
                }
                else
                {
                    ProjectionOneFrameTwoCamFactor *f = new ProjectionOneFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                 it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                    problem.AddResidualBlock(f, loss_function, para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]);
                }
               
            }
            f_m_cnt++;
        }
    }

    ROS_DEBUG("visual measurement count: %d", f_m_cnt);
    //printf("prepare for ceres: %f \n", t_prepare.toc());

    bool diagnostic_sample = false;
    if (!BIAS_DIAGNOSTICS_DIR.empty() && USE_IMU && frame_count == WINDOW_SIZE)
    {
        const double stamp = Headers[frame_count];
        if (bias_diagnostic_first_stamp < 0) bias_diagnostic_first_stamp = stamp;
        diagnostic_sample = stamp - bias_diagnostic_first_stamp <= BIAS_DIAGNOSTICS_STARTUP_DENSE_S + 1e-9
            || bias_diagnostic_last_stamp < 0 || stamp - bias_diagnostic_last_stamp >= BIAS_DIAGNOSTICS_PERIOD_S - 1e-9;
        if (diagnostic_sample)
        {
            ++bias_diagnostic_sequence; bias_diagnostic_last_stamp = stamp;
            bias_diagnostic_file_entries.clear();
            biasDumpCurrentPrior(*this,problem,last_marginalization_info,last_marginalization_parameter_blocks,"prior_before_solve");
            biasDumpProblem(*this,problem,"full_problem_before_solve");
        }
    }

    ceres::Solver::Options options;

    options.linear_solver_type = ceres::DENSE_SCHUR;
    //options.num_threads = 2;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_num_iterations = NUM_ITERATIONS;
    //options.use_explicit_schur_complement = true;
    //options.minimizer_progress_to_stdout = true;
    //options.use_nonmonotonic_steps = true;
    if (marginalization_flag == MARGIN_OLD)
        options.max_solver_time_in_seconds = SOLVER_TIME * 4.0 / 5.0;
    else
        options.max_solver_time_in_seconds = SOLVER_TIME;
    TicToc t_solver;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    if (diagnostic_sample) biasDumpProblem(*this,problem,"full_problem_after_solve_before_gauge");
    solver_iterations = static_cast<int>(summary.iterations.size());
    solver_successful_steps = summary.num_successful_steps;
    solver_unsuccessful_steps = summary.num_unsuccessful_steps;
    solver_termination_type = static_cast<int>(summary.termination_type);
    solver_wall_time_s = std::isfinite(summary.total_time_in_seconds) ? summary.total_time_in_seconds : -1;
    solver_budget_s = options.max_solver_time_in_seconds;
    solver_last_gradient_max_norm = summary.iterations.empty() ? -1 : summary.iterations.back().gradient_max_norm;
    if (!std::isfinite(solver_last_gradient_max_norm)) solver_last_gradient_max_norm = -1;
    //cout << summary.BriefReport() << endl;
    ROS_DEBUG("Iterations : %d", static_cast<int>(summary.iterations.size()));
    //printf("solver costs: %f \n", t_solver.toc());

    solver_initial_cost = std::isfinite(summary.initial_cost) ? summary.initial_cost : -1;
    solver_final_cost = std::isfinite(summary.final_cost) ? summary.final_cost : -1;
    solver_solution_usable = summary.IsSolutionUsable() && summary.num_residuals > 0
        && std::isfinite(summary.initial_cost) && summary.initial_cost >= 0
        && std::isfinite(summary.final_cost) && summary.final_cost >= 0
        && summary.final_cost <= summary.initial_cost + 1e-6 * (1.0 + summary.initial_cost);
    ceres::Problem::EvaluateOptions evaluation_options;
    std::vector<double> residuals;
    ceres::CRSMatrix jacobian;
    double evaluated_cost = 0;
    if (solver_solution_usable)
    {
        solver_solution_usable = problem.Evaluate(evaluation_options, &evaluated_cost, &residuals, NULL, &jacobian)
            && std::isfinite(evaluated_cost) && evaluated_cost >= 0 && !residuals.empty();
        for (double r : residuals) solver_solution_usable = solver_solution_usable && std::isfinite(r);
        for (double j : jacobian.values) solver_solution_usable = solver_solution_usable && std::isfinite(j);
    }
    solver_residual_rms = solver_solution_usable ? std::sqrt(2.0 * evaluated_cost / residuals.size()) : -1;
    if (!solver_solution_usable)
    {
        solver_health_reason = "ceres_unusable_solution_or_residual";
        ROS_WARN("VIO unusable optimization: %s", summary.BriefReport().c_str());
        return;
    }
    double2vector();
    solver_solution_usable = validateOptimizedSolution();
    if (!solver_solution_usable) return;
    if (VISUAL_LIFECYCLE_DIAGNOSTICS)
    {
        native_visual_lifecycle.solved = NativeVisualLifecycle::snapshot(f_manager,"post_solve_pre_outlier");
        native_visual_lifecycle.outcome = solver_flag == INITIAL ? "initial_solve_without_outlier_stage" : "awaiting_outlier_stage";
    }
    //printf("frame_count: %d \n", frame_count);

    if(frame_count < WINDOW_SIZE)
        return;
    
    TicToc t_whole_marginalization;
    if (marginalization_flag == MARGIN_OLD)
    {
        MarginalizationInfo *marginalization_info = new MarginalizationInfo(EXACT_MARGINALIZATION_POSE_JACOBIAN != 0);
        marginalization_info->diagnostics_enabled = diagnostic_sample;
        vector2double();
        if (diagnostic_sample) biasDumpProblem(*this,problem,"full_problem_after_native_yaw_gauge");
        if (add_initial_bias_prior)
        {
            marginalization_info->addResidualBlockInfo(new ResidualBlockInfo(
                new ImuBiasPrior(ACC_BIAS_PRIOR_SIGMA, GYR_BIAS_PRIOR_SIGMA), NULL,
                vector<double *>{para_SpeedBias[0]}, vector<int>{0}));
        }

        if (last_marginalization_info && last_marginalization_info->valid)
        {
            vector<int> drop_set;
            for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
            {
                if (last_marginalization_parameter_blocks[i] == para_Pose[0] ||
                    last_marginalization_parameter_blocks[i] == para_SpeedBias[0])
                    drop_set.push_back(i);
            }
            // construct new marginlization_factor
            MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                           last_marginalization_parameter_blocks,
                                                                           drop_set);
            marginalization_info->addResidualBlockInfo(residual_block_info);
        }

        if (ENABLE_ZUPT && USE_IMU && stationary_frames[0])
        {
            marginalization_info->addResidualBlockInfo(new ResidualBlockInfo(
                new ZeroVelocityFactor(ZUPT_VELOCITY_SIGMA), NULL,
                vector<double *>{para_SpeedBias[0]}, vector<int>{0}));
            if (stationary_frames[1] && stationary_intervals[1])
                marginalization_info->addResidualBlockInfo(new ResidualBlockInfo(
                    new StationaryRelativePositionFactor(ZUPT_POSITION_SIGMA), NULL,
                    vector<double *>{para_Pose[0], para_Pose[1]}, vector<int>{0}));
        }

        if(USE_IMU)
        {
            if (pre_integrations[1]->sum_dt < 10.0)
            {
                IMUFactor* imu_factor = new IMUFactor(pre_integrations[1]);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(imu_factor, NULL,
                                                                           vector<double *>{para_Pose[0], para_SpeedBias[0], para_Pose[1], para_SpeedBias[1]},
                                                                           vector<int>{0, 1});
                marginalization_info->addResidualBlockInfo(residual_block_info);
            }
        }

        {
            int feature_index = -1;
            for (auto &it_per_id : f_manager.feature)
            {
                it_per_id.used_num = it_per_id.feature_per_frame.size();
                if (it_per_id.used_num < 4)
                    continue;

                ++feature_index;

                int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
                if (imu_i != 0)
                    continue;

                Vector3d pts_i = it_per_id.feature_per_frame[0].point;

                for (auto &it_per_frame : it_per_id.feature_per_frame)
                {
                    imu_j++;
                    if(imu_i != imu_j)
                    {
                        Vector3d pts_j = it_per_frame.point;
                        ProjectionTwoFrameOneCamFactor *f_td = new ProjectionTwoFrameOneCamFactor(pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                                                                          it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f_td, loss_function,
                                                                                        vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index], para_Td[0]},
                                                                                        vector<int>{0, 3});
                        marginalization_info->addResidualBlockInfo(residual_block_info);
                    }
                    if(STEREO && it_per_frame.is_stereo)
                    {
                        Vector3d pts_j_right = it_per_frame.pointRight;
                        if(imu_i != imu_j)
                        {
                            ProjectionTwoFrameTwoCamFactor *f = new ProjectionTwoFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                          it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                                                                                           vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]},
                                                                                           vector<int>{0, 4});
                            marginalization_info->addResidualBlockInfo(residual_block_info);
                        }
                        else
                        {
                            ProjectionOneFrameTwoCamFactor *f = new ProjectionOneFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                          it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                                                                                           vector<double *>{para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]},
                                                                                           vector<int>{2});
                            marginalization_info->addResidualBlockInfo(residual_block_info);
                        }
                    }
                }
            }
        }

        TicToc t_pre_margin;
        marginalization_info->preMarginalize();
        ROS_DEBUG("pre marginalization %f ms", t_pre_margin.toc());
        
        TicToc t_margin;
        marginalization_info->marginalize();
        if (!marginalization_info->valid || !marginalization_info->linearized_jacobians.allFinite()
            || !marginalization_info->linearized_residuals.allFinite())
        {
            delete marginalization_info;
            solver_solution_usable = false;
            solver_health_reason = "unusable_marginalization_prior";
            ROS_WARN("VIO marginalization produced an unusable prior");
            return;
        }
        ROS_DEBUG("marginalization %f ms", t_margin.toc());
        if (diagnostic_sample) biasDumpMarginalization(*this,problem,marginalization_info,"old");

        std::unordered_map<long, double *> addr_shift;
        for (int i = 1; i <= WINDOW_SIZE; i++)
        {
            addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
            if(USE_IMU)
                addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
        }
        for (int i = 0; i < NUM_OF_CAM; i++)
            addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];

        addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];

        vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);

        if (last_marginalization_info)
            delete last_marginalization_info;
        last_marginalization_info = marginalization_info;
        last_marginalization_parameter_blocks = parameter_blocks;
        
    }
    else
    {
        if (last_marginalization_info &&
            std::count(std::begin(last_marginalization_parameter_blocks), std::end(last_marginalization_parameter_blocks), para_Pose[WINDOW_SIZE - 1]))
        {

            MarginalizationInfo *marginalization_info = new MarginalizationInfo(EXACT_MARGINALIZATION_POSE_JACOBIAN != 0);
            marginalization_info->diagnostics_enabled = diagnostic_sample;
            vector2double();
            if (diagnostic_sample) biasDumpProblem(*this,problem,"full_problem_after_native_yaw_gauge");
            if (last_marginalization_info && last_marginalization_info->valid)
            {
                vector<int> drop_set;
                for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
                {
                    ROS_ASSERT(last_marginalization_parameter_blocks[i] != para_SpeedBias[WINDOW_SIZE - 1]);
                    if (last_marginalization_parameter_blocks[i] == para_Pose[WINDOW_SIZE - 1])
                        drop_set.push_back(i);
                }
                // construct new marginlization_factor
                MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                               last_marginalization_parameter_blocks,
                                                                               drop_set);

                marginalization_info->addResidualBlockInfo(residual_block_info);
            }

            TicToc t_pre_margin;
            ROS_DEBUG("begin marginalization");
            marginalization_info->preMarginalize();
            ROS_DEBUG("end pre marginalization, %f ms", t_pre_margin.toc());

            TicToc t_margin;
            ROS_DEBUG("begin marginalization");
            marginalization_info->marginalize();
        if (!marginalization_info->valid || !marginalization_info->linearized_jacobians.allFinite()
            || !marginalization_info->linearized_residuals.allFinite())
        {
            delete marginalization_info;
            solver_solution_usable = false;
            solver_health_reason = "unusable_marginalization_prior";
            ROS_WARN("VIO marginalization produced an unusable prior");
            return;
        }
            ROS_DEBUG("end marginalization, %f ms", t_margin.toc());
            if (diagnostic_sample) biasDumpMarginalization(*this,problem,marginalization_info,"second_new");
            
            std::unordered_map<long, double *> addr_shift;
            for (int i = 0; i <= WINDOW_SIZE; i++)
            {
                if (i == WINDOW_SIZE - 1)
                    continue;
                else if (i == WINDOW_SIZE)
                {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
                    if(USE_IMU)
                        addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
                }
                else
                {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i];
                    if(USE_IMU)
                        addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i];
                }
            }
            for (int i = 0; i < NUM_OF_CAM; i++)
                addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];

            addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];

            
            vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);
            if (last_marginalization_info)
                delete last_marginalization_info;
            last_marginalization_info = marginalization_info;
            last_marginalization_parameter_blocks = parameter_blocks;
            
        }
    }
    //printf("whole marginalization costs: %f \n", t_whole_marginalization.toc());
    //printf("whole time for ceres: %f \n", t_whole.toc());
}

void Estimator::slideWindow()
{
    TicToc t_margin;
    if (marginalization_flag == MARGIN_OLD)
    {
        double t_0 = Headers[0];
        back_R0 = Rs[0];
        back_P0 = Ps[0];
        if (frame_count == WINDOW_SIZE)
        {
            slideStationaryEvidenceOld(stationary_frames, stationary_intervals, WINDOW_SIZE);
            for (int i = 0; i < WINDOW_SIZE; i++)
            {
                Headers[i] = Headers[i + 1];
                Rs[i].swap(Rs[i + 1]);
                Ps[i].swap(Ps[i + 1]);
                if(USE_IMU)
                {
                    std::swap(pre_integrations[i], pre_integrations[i + 1]);

                    dt_buf[i].swap(dt_buf[i + 1]);
                    linear_acceleration_buf[i].swap(linear_acceleration_buf[i + 1]);
                    angular_velocity_buf[i].swap(angular_velocity_buf[i + 1]);

                    Vs[i].swap(Vs[i + 1]);
                    Bas[i].swap(Bas[i + 1]);
                    Bgs[i].swap(Bgs[i + 1]);
                }
            }
            Headers[WINDOW_SIZE] = Headers[WINDOW_SIZE - 1];
            Ps[WINDOW_SIZE] = Ps[WINDOW_SIZE - 1];
            Rs[WINDOW_SIZE] = Rs[WINDOW_SIZE - 1];

            if(USE_IMU)
            {
                Vs[WINDOW_SIZE] = Vs[WINDOW_SIZE - 1];
                Bas[WINDOW_SIZE] = Bas[WINDOW_SIZE - 1];
                Bgs[WINDOW_SIZE] = Bgs[WINDOW_SIZE - 1];

                delete pre_integrations[WINDOW_SIZE];
                pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

                dt_buf[WINDOW_SIZE].clear();
                linear_acceleration_buf[WINDOW_SIZE].clear();
                angular_velocity_buf[WINDOW_SIZE].clear();
            }

            if (true || solver_flag == INITIAL)
            {
                // Delayed stereo INITIAL must own exactly the retained Header keys.
                // The original code left t_0 and its freed edge in the map.
                const auto end = STATIONARY_INITIALIZATION && STEREO && USE_IMU
                    ? all_image_frame.upper_bound(t_0) : all_image_frame.lower_bound(t_0);
                for (auto it = all_image_frame.begin(); it != end; ++it)
                { delete it->second.pre_integration; it->second.pre_integration = nullptr; }
                all_image_frame.erase(all_image_frame.begin(), end);
                if (!(STATIONARY_INITIALIZATION && STEREO && USE_IMU))
                {
                    const auto retained_old = all_image_frame.find(t_0);
                    if (retained_old != all_image_frame.end())
                    { delete retained_old->second.pre_integration; retained_old->second.pre_integration = nullptr; }
                }
            }
            slideWindowOld();
        }
    }
    else
    {
        if (frame_count == WINDOW_SIZE)
        {
            Headers[frame_count - 1] = Headers[frame_count];
            slideStationaryEvidenceSecondNew(stationary_frames, stationary_intervals, frame_count);
            Ps[frame_count - 1] = Ps[frame_count];
            Rs[frame_count - 1] = Rs[frame_count];

            if(USE_IMU)
            {
                for (unsigned int i = 0; i < dt_buf[frame_count].size(); i++)
                {
                    double tmp_dt = dt_buf[frame_count][i];
                    Vector3d tmp_linear_acceleration = linear_acceleration_buf[frame_count][i];
                    Vector3d tmp_angular_velocity = angular_velocity_buf[frame_count][i];

                    pre_integrations[frame_count - 1]->push_back(tmp_dt, tmp_linear_acceleration, tmp_angular_velocity);

                    dt_buf[frame_count - 1].push_back(tmp_dt);
                    linear_acceleration_buf[frame_count - 1].push_back(tmp_linear_acceleration);
                    angular_velocity_buf[frame_count - 1].push_back(tmp_angular_velocity);
                }

                Vs[frame_count - 1] = Vs[frame_count];
                Bas[frame_count - 1] = Bas[frame_count];
                Bgs[frame_count - 1] = Bgs[frame_count];

                delete pre_integrations[WINDOW_SIZE];
                pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

                dt_buf[WINDOW_SIZE].clear();
                linear_acceleration_buf[WINDOW_SIZE].clear();
                angular_velocity_buf[WINDOW_SIZE].clear();
            }
            slideWindowNew();
        }
    }
}

void Estimator::slideWindowNew()
{
    sum_of_front++;
    f_manager.removeFront(frame_count);
}

void Estimator::slideWindowOld()
{
    sum_of_back++;

    bool shift_depth = solver_flag == NON_LINEAR;
    if (shift_depth)
    {
        Matrix3d R0, R1;
        Vector3d P0, P1;
        R0 = back_R0 * ric[0];
        R1 = Rs[0] * ric[0];
        P0 = back_P0 + back_R0 * tic[0];
        P1 = Ps[0] + Rs[0] * tic[0];
        f_manager.removeBackShiftDepth(R0, P0, R1, P1);
    }
    else
    {
        f_manager.removeBack();
        if (STATIONARY_INITIALIZATION && STEREO && USE_IMU)
        {
            // Waiting INITIAL has no optimized inter-frame transform. The
            // removed reference changes each inverse depth's camera anchor;
            // re-triangulate the retained actual stereo observations instead
            // of carrying old depth through unoptimized pose predictions.
            f_manager.clearDepth();
            f_manager.triangulate(WINDOW_SIZE - 1, Ps, Rs, tic, ric);
        }
    }
}


void Estimator::getPoseInWorldFrame(Eigen::Matrix4d &T)
{
    T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = Rs[frame_count];
    T.block<3, 1>(0, 3) = Ps[frame_count];
}

void Estimator::getPoseInWorldFrame(int index, Eigen::Matrix4d &T)
{
    T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = Rs[index];
    T.block<3, 1>(0, 3) = Ps[index];
}

void Estimator::predictPtsInNextFrame()
{
    //printf("predict pts in next frame\n");
    if(frame_count < 2)
        return;
    // predict next pose. Assume constant velocity motion
    Eigen::Matrix4d curT, prevT, nextT;
    getPoseInWorldFrame(curT);
    getPoseInWorldFrame(frame_count - 1, prevT);
    nextT = curT * (prevT.inverse() * curT);
    map<int, Eigen::Vector3d> predictPts;

    for (auto &it_per_id : f_manager.feature)
    {
        if(it_per_id.estimated_depth > 0)
        {
            int firstIndex = it_per_id.start_frame;
            int lastIndex = it_per_id.start_frame + it_per_id.feature_per_frame.size() - 1;
            //printf("cur frame index  %d last frame index %d\n", frame_count, lastIndex);
            if((int)it_per_id.feature_per_frame.size() >= 2 && lastIndex == frame_count)
            {
                double depth = it_per_id.estimated_depth;
                Vector3d pts_j = ric[0] * (depth * it_per_id.feature_per_frame[0].point) + tic[0];
                Vector3d pts_w = Rs[firstIndex] * pts_j + Ps[firstIndex];
                Vector3d pts_local = nextT.block<3, 3>(0, 0).transpose() * (pts_w - nextT.block<3, 1>(0, 3));
                Vector3d pts_cam = ric[0].transpose() * (pts_local - tic[0]);
                int ptsIndex = it_per_id.feature_id;
                predictPts[ptsIndex] = pts_cam;
            }
        }
    }
    featureTracker.setPrediction(predictPts);
    //printf("estimator output %d predict pts\n",(int)predictPts.size());
}

double Estimator::reprojectionError(Matrix3d &Ri, Vector3d &Pi, Matrix3d &rici, Vector3d &tici,
                                 Matrix3d &Rj, Vector3d &Pj, Matrix3d &ricj, Vector3d &ticj, 
                                 double depth, Vector3d &uvi, Vector3d &uvj)
{
    Vector3d pts_w = Ri * (rici * (depth * uvi) + tici) + Pi;
    Vector3d pts_cj = ricj.transpose() * (Rj.transpose() * (pts_w - Pj) - ticj);
    Vector2d residual = (pts_cj / pts_cj.z()).head<2>() - uvj.head<2>();
    double rx = residual.x();
    double ry = residual.y();
    return sqrt(rx * rx + ry * ry);
}

void Estimator::beginVisualDiagnostics()
{
    if (!VISUAL_LIFECYCLE_DIAGNOSTICS) return;
    const int sequence = ++native_visual_solve_sequence;
    native_visual_lifecycle = NativeVisualLifecycle();
    native_visual_lifecycle.sampled = true;
    native_visual_lifecycle.solve_sequence = sequence;
    native_visual_lifecycle.source_stamp = Headers[frame_count];
    native_visual_lifecycle.outcome = "solve_started_no_accepted_candidate_yet";
    native_visual_lifecycle.before = NativeVisualLifecycle::snapshot(f_manager,"before_solve");
    for (int i=0;i<=frame_count;++i) native_visual_lifecycle.headers.push_back(Headers[i]);
    if (USE_IMU) for (int i=1;i<=frame_count;++i)
        native_visual_lifecycle.imu_spans.push_back(pre_integrations[i] ? pre_integrations[i]->sum_dt : std::numeric_limits<double>::quiet_NaN());
}

void Estimator::finishVisualOutlierDiagnostics(const set<int> &removeIndex)
{
    if (!VISUAL_LIFECYCLE_DIAGNOSTICS) return;
    native_visual_lifecycle.removed_ids_total = static_cast<int>(removeIndex.size());
    native_visual_lifecycle.removed_ids.clear();
    for (int id:removeIndex) {
        if (native_visual_lifecycle.removed_ids.size()==NativeVisualLifecycle::REMOVED_ID_LIMIT) break;
        native_visual_lifecycle.removed_ids.push_back(id);
    }
    native_visual_lifecycle.after = NativeVisualLifecycle::snapshot(f_manager,"post_outlier_before_slide");
    native_visual_lifecycle.outcome = "outlier_stage_completed_before_failure_guard_and_slide";
}

void Estimator::outliersRejection(set<int> &removeIndex)
{
    if (VISUAL_LIFECYCLE_DIAGNOSTICS) native_visual_lifecycle.outliers_evaluated = true;
    //return;
    int feature_index = -1;
    for (auto &it_per_id : f_manager.feature)
    {
        double err = 0;
        int errCnt = 0;
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (it_per_id.used_num < 4)
            continue;
        feature_index ++;
        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
        Vector3d pts_i = it_per_id.feature_per_frame[0].point;
        double depth = it_per_id.estimated_depth;
        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            if (imu_i != imu_j)
            {
                Vector3d pts_j = it_per_frame.point;             
                double tmp_error = reprojectionError(Rs[imu_i], Ps[imu_i], ric[0], tic[0], 
                                                    Rs[imu_j], Ps[imu_j], ric[0], tic[0],
                                                    depth, pts_i, pts_j);
                err += tmp_error;
                errCnt++;
                //printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
            }
            // need to rewrite projecton factor.........
            if(STEREO && it_per_frame.is_stereo)
            {
                
                Vector3d pts_j_right = it_per_frame.pointRight;
                if(imu_i != imu_j)
                {            
                    double tmp_error = reprojectionError(Rs[imu_i], Ps[imu_i], ric[0], tic[0], 
                                                        Rs[imu_j], Ps[imu_j], ric[1], tic[1],
                                                        depth, pts_i, pts_j_right);
                    err += tmp_error;
                    errCnt++;
                    //printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
                }
                else
                {
                    double tmp_error = reprojectionError(Rs[imu_i], Ps[imu_i], ric[0], tic[0], 
                                                        Rs[imu_j], Ps[imu_j], ric[1], tic[1],
                                                        depth, pts_i, pts_j_right);
                    err += tmp_error;
                    errCnt++;
                    //printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
                }       
            }
        }
        double ave_err = err / errCnt;
        if(ave_err * FOCAL_LENGTH > 3)
            removeIndex.insert(it_per_id.feature_id);
        if (VISUAL_LIFECYCLE_DIAGNOSTICS)
            native_visual_lifecycle.addReprojection(it_per_id.feature_id, it_per_id.start_frame,
                it_per_id.used_num, depth, errCnt, ave_err, FOCAL_LENGTH,
                removeIndex.count(it_per_id.feature_id) != 0);

    }
}

void Estimator::fastPredictIMU(double t, Eigen::Vector3d linear_acceleration, Eigen::Vector3d angular_velocity)
{
    double dt = t - latest_time;
    latest_time = t;
    Eigen::Vector3d un_acc_0 = latest_Q * (latest_acc_0 - latest_Ba) - g;
    Eigen::Vector3d un_gyr = 0.5 * (latest_gyr_0 + angular_velocity) - latest_Bg;
    latest_Q = latest_Q * Utility::deltaQ(un_gyr * dt);
    Eigen::Vector3d un_acc_1 = latest_Q * (linear_acceleration - latest_Ba) - g;
    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
    latest_P = latest_P + dt * latest_V + 0.5 * dt * dt * un_acc;
    latest_V = latest_V + dt * un_acc;
    latest_acc_0 = linear_acceleration;
    latest_gyr_0 = angular_velocity;
}

bool Estimator::updateLatestStates()
{
    std::unique_lock<std::mutex> propagation_lock(mPropagate);
    NativeImuPrediction candidate;
    candidate.stamp = Headers[frame_count] + td;
    candidate.p = Ps[frame_count]; candidate.q = Eigen::Quaterniond(Rs[frame_count]);
    candidate.v = Vs[frame_count]; candidate.ba = Bas[frame_count]; candidate.bg = Bgs[frame_count];
    candidate.acc = acc_0; candidate.gyro = gyr_0;
    NativeImuPrediction published;
    const bool previously_published = USE_IMU && solver_flag == NON_LINEAR;
    if (previously_published)
    {
        published.stamp = latest_time; published.p = latest_P; published.q = latest_Q; published.v = latest_V;
        published.ba = latest_Ba; published.bg = latest_Bg; published.acc = latest_acc_0; published.gyro = latest_gyr_0;
    }
    propagation_refresh_checked = false;
    propagation_refresh_evidence = NativeRefreshEvidence();
    propagation_refresh_evidence.candidate_base_stamp_sim_s = candidate.stamp;
    propagation_refresh_evidence.previous_published_stamp_sim_s = previously_published ? published.stamp : 0;
    mBuf.lock();
    queue<pair<double, Eigen::Vector3d>> tmp_accBuf = accBuf;
    queue<pair<double, Eigen::Vector3d>> tmp_gyrBuf = gyrBuf;
    mBuf.unlock();
    bool matched_published_stamp = previously_published && std::abs(candidate.stamp - published.stamp) <= 1e-6;
    NativeImuPrediction at_published_stamp = candidate;
    while (!tmp_accBuf.empty() && !tmp_gyrBuf.empty())
    {
        const double stamp = tmp_accBuf.front().first;
        if (!predictNativeImu(candidate, stamp, tmp_accBuf.front().second, tmp_gyrBuf.front().second, g))
        {
            solver_solution_usable = false; failure_occur = true; solver_flag = INITIAL;
            solver_health_reason = "invalid_candidate_imu_propagation";
            return false;
        }
        if (previously_published && !matched_published_stamp)
        {
            if (std::abs(stamp - published.stamp) <= 1e-6)
            { at_published_stamp = candidate; matched_published_stamp = true; }
            else if (stamp > published.stamp)
            {
                // An IMU callback can have queued its sample while waiting for
                // mPropagate. Advance the old trajectory with that same actual
                // sample, then compare both predictions at its true stamp.
                propagation_refresh_evidence.pending_old_forward_dt_s = stamp - published.stamp;
                if (!advancePendingPublishedImu(published, stamp, tmp_accBuf.front().second, tmp_gyrBuf.front().second, g))
                {
                    solver_solution_usable = false; failure_occur = true; solver_flag = INITIAL;
                    solver_health_reason = "propagation_refresh_unobserved_imu_gap_or_invalid_previous_state";
                    return false;
                }
                at_published_stamp = candidate; matched_published_stamp = true;
            }
        }
        tmp_accBuf.pop(); tmp_gyrBuf.pop();
    }
    if (previously_published)
    {
        propagation_refresh_checked = true;
        if (!matched_published_stamp)
        {
            solver_solution_usable = false; failure_occur = true; solver_flag = INITIAL;
            solver_health_reason = "propagation_refresh_same_time_source_missing";
            return false;
        }
        if (!nativeRefreshConsistent(published, at_published_stamp, HEALTH_IMU_TO_BODY_FLU, &propagation_refresh_evidence))
        {
            solver_solution_usable = false; failure_occur = true; solver_flag = INITIAL;
            solver_health_reason = "optimized_propagation_refresh_inconsistent";
            ROS_WARN("VIO propagation refresh rejected at %.6f: imu_dp=%.6f body_dp=%.6f angle=%.6f",
                published.stamp, propagation_refresh_evidence.imu_position_delta_m,
                propagation_refresh_evidence.body_position_delta_m, propagation_refresh_evidence.orientation_delta_rad);
            return false;
        }
    }
    // Commit only after the candidate agrees at the same actual source time.
    latest_time = candidate.stamp; latest_P = candidate.p; latest_Q = candidate.q; latest_V = candidate.v;
    latest_Ba = candidate.ba; latest_Bg = candidate.bg; latest_acc_0 = candidate.acc; latest_gyr_0 = candidate.gyro;
    return true;
}
