"""Reproducible ROS Noetic and simulation robustness patches for official VINS."""
from pathlib import Path
import subprocess


def patch_estimator(root):
    path = root / 'vins_estimator/src/estimator/estimator.cpp'
    content = path.read_text()
    if '// PX4_AGENT_VIO_PATCH' in content:
        return
    content = content.replace('#include "../utility/visualization.h"', '#include "../utility/visualization.h"\n// PX4_AGENT_VIO_PATCH: health, guarded reset, and clean shutdown only.')
    content = content.replace('    prevTime = -1;', '    visual_gap_since = -1;\n    prevTime = -1;')
    content = content.replace('    while (1)\n', '    ros::NodeHandle health_node;\n    ros::Publisher health_pub = health_node.advertise<std_msgs::Bool>("/vins_estimator/tracking_ok", 1, true);\n    while (ros::ok())\n', 1)
    content = content.replace('            while(1)\n', '            while(ros::ok())\n', 1)
    content = content.replace('            mBuf.lock();\n            if(USE_IMU)', '            if (!ros::ok()) return;\n            mBuf.lock();\n            if(USE_IMU)', 1)
    content = content.replace('            processImage(feature.second, feature.first);\n            prevTime = curTime;', '''            processImage(feature.second, feature.first);
            std_msgs::Bool tracking_status;
            tracking_status.data = solver_flag == NON_LINEAR && !failure_occur;
            health_pub.publish(tracking_status);
            if (failure_occur)
            {
                // clearState/setParameter acquire mProcess themselves.
                mProcess.unlock();
                clearState();
                setParameter();
                ROS_WARN("VIO reset: waiting for stereo/IMU initialization");
                continue;
            }
            prevTime = curTime;''')
    content = content.replace('            clearState();\n            setParameter();\n            ROS_WARN("system reboot!");', '            solver_flag = INITIAL;')
    start = content.index('bool Estimator::failureDetection()')
    end = content.index('\nvoid Estimator::optimization()', start)
    content = content[:start] + '''bool Estimator::failureDetection()
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
        ROS_WARN("VIO invalid: features=%d, speed=%.3f, acc_bias=%.3f, gyro_bias=%.3f",
            f_manager.getFeatureCount(), Vs[WINDOW_SIZE].norm(),
            Bas[WINDOW_SIZE].norm(), Bgs[WINDOW_SIZE].norm());
        return true;
    }
    return false;
}
''' + content[end:]
    path.write_text(content)
    header = root / 'vins_estimator/src/estimator/estimator.h'
    header.write_text(header.read_text().replace('    bool initFirstPoseFlag;', '    double visual_gap_since;\n    bool initFirstPoseFlag;'))
    node = root / 'vins_estimator/src/rosNodeTest.cpp'
    content = node.read_text().replace('    while(1)', '    while(ros::ok())').replace('    ros::spin();', '    ros::spin();\n    sync_thread.join();')
    for camera in ('img0', 'img1'):
        signature = 'void ' + camera + '_callback(const sensor_msgs::ImageConstPtr &img_msg)\n{'
        content = content.replace(signature, signature + '\n    if ((ros::Time::now() - img_msg->header.stamp).toSec() > 0.5) return;')
    content = content.replace('    double t = imu_msg->header.stamp.toSec();', '''    double t = imu_msg->header.stamp.toSec();
    static double previous_imu_stamp = -1;
    if (t <= previous_imu_stamp || (ros::Time::now() - imu_msg->header.stamp).toSec() > 0.5) return;
    previous_imu_stamp = t;''', 1)
    node.write_text(content)


def patch_bias_prior(root):
    factor = root / 'vins_estimator/src/factor/imu_bias_prior.h'
    factor.write_text((Path(__file__).resolve().parent / 'patches/imu_bias_prior.h').read_text())
    path = root / 'vins_estimator/src/estimator/estimator.cpp'
    content = path.read_text()
    if '// PX4_AGENT_INITIAL_BIAS_PRIOR' in content:
        return
    content = content.replace('#include "../utility/visualization.h"', '''#include "../utility/visualization.h"
#include "../factor/imu_bias_prior.h"
#include <std_msgs/Float64MultiArray.h>
// PX4_AGENT_INITIAL_BIAS_PRIOR: native IMU statistics, added once then marginalized.''')
    content = content.replace('    ros::Publisher health_pub =', '    ros::Publisher bias_pub = health_node.advertise<std_msgs::Float64MultiArray>("/vins_estimator/imu_bias", 1);\n    ros::Publisher health_pub =', 1)
    content = content.replace('            health_pub.publish(tracking_status);', '''            health_pub.publish(tracking_status);
            std_msgs::Float64MultiArray bias_status;
            for (int axis = 0; axis < 3; ++axis) bias_status.data.push_back(Bas[frame_count][axis]);
            for (int axis = 0; axis < 3; ++axis) bias_status.data.push_back(Bgs[frame_count][axis]);
            bias_pub.publish(bias_status);''', 1)
    content = content.replace('    ceres::Problem problem;', '''    ceres::Problem problem;
    const bool add_initial_bias_prior = USE_IMU && ACC_BIAS_PRIOR_SIGMA > 0 && GYR_BIAS_PRIOR_SIGMA > 0
        && (!last_marginalization_info || !last_marginalization_info->valid);''', 1)
    content = content.replace('    if(!USE_IMU)\n        problem.SetParameterBlockConstant(para_Pose[0]);', '''    if(!USE_IMU)
        problem.SetParameterBlockConstant(para_Pose[0]);
    if (add_initial_bias_prior)
        problem.AddResidualBlock(new ImuBiasPrior(ACC_BIAS_PRIOR_SIGMA, GYR_BIAS_PRIOR_SIGMA),
                                 NULL, para_SpeedBias[0]);''', 1)
    content = content.replace('        MarginalizationInfo *marginalization_info = new MarginalizationInfo();\n        vector2double();', '''        MarginalizationInfo *marginalization_info = new MarginalizationInfo();
        vector2double();
        if (add_initial_bias_prior)
        {
            marginalization_info->addResidualBlockInfo(new ResidualBlockInfo(
                new ImuBiasPrior(ACC_BIAS_PRIOR_SIGMA, GYR_BIAS_PRIOR_SIGMA), NULL,
                vector<double *>{para_SpeedBias[0]}, vector<int>{0}));
        }''', 1)
    path.write_text(content)
    header = root / 'vins_estimator/src/estimator/parameters.h'
    header.write_text(header.read_text().replace('extern double ACC_N, ACC_W;', 'extern double ACC_N, ACC_W;\nextern double ACC_BIAS_PRIOR_SIGMA, GYR_BIAS_PRIOR_SIGMA;'))
    parameters = root / 'vins_estimator/src/estimator/parameters.cpp'
    content = parameters.read_text().replace('double ACC_N, ACC_W;', 'double ACC_N, ACC_W;\ndouble ACC_BIAS_PRIOR_SIGMA = 0, GYR_BIAS_PRIOR_SIGMA = 0;')
    content = content.replace('    INIT_DEPTH = 5.0;', '''    if (!fsSettings["acc_bias_prior_sigma"].empty()) fsSettings["acc_bias_prior_sigma"] >> ACC_BIAS_PRIOR_SIGMA;
    if (!fsSettings["gyr_bias_prior_sigma"].empty()) fsSettings["gyr_bias_prior_sigma"] >> GYR_BIAS_PRIOR_SIGMA;
    ROS_INFO("Initial native-IMU bias prior sigmas: acceleration=%f gyro=%f", ACC_BIAS_PRIOR_SIGMA, GYR_BIAS_PRIOR_SIGMA);

    INIT_DEPTH = 5.0;''')
    parameters.write_text(content)


def patch_stereo_monotonic(root):
    path = root / 'vins_estimator/src/rosNodeTest.cpp'
    content = path.read_text()
    if '// PX4_AGENT_MONOTONIC_STEREO' in content:
        return
    replacements = {
        'void sync_process()\n{': '''void sync_process()
{
    // PX4_AGENT_MONOTONIC_STEREO: duplicate stamps make feature velocities divide by zero.
    double last_accepted_stereo_stamp = -1;''',
        '''            if(!image0.empty())
                estimator.inputImage(time, image0, image1);''': '''            if(!image0.empty() && time > last_accepted_stereo_stamp)
            {
                last_accepted_stereo_stamp = time;
                estimator.inputImage(time, image0, image1);
            }''',
    }
    for old, new in replacements.items():
        if content.count(old) != 1:
            raise RuntimeError('Cannot locate unique stereo monotonic patch target: ' + old)
        content = content.replace(old, new, 1)
    path.write_text(content)


def patch_opencv_threads(root):
    path = root / 'vins_estimator/src/rosNodeTest.cpp'
    content = path.read_text()
    if '// PX4_AGENT_OPENCV_THREADS' in content:
        old_limit = 'cv::setNumThreads(1);'
        if old_limit in content:
            if content.count(old_limit) != 1:
                raise RuntimeError('Cannot locate unique previous OpenCV thread limit')
            path.write_text(content.replace(old_limit, 'cv::setNumThreads(2);', 1))
        return
    old = '    string config_file = argv[1];'
    if content.count(old) != 1:
        raise RuntimeError('Cannot locate unique OpenCV thread patch target')
    content = content.replace(old, '''    // PX4_AGENT_OPENCV_THREADS: avoid TBB oversubscription alongside Gazebo GUI.
    cv::setNumThreads(2);
    ROS_INFO("OpenCV worker threads: %d", cv::getNumThreads());

    string config_file = argv[1];''', 1)
    path.write_text(content)


def patch_stationary_health(root):
    """Apply native sensor/optimizer health, or upgrade a consistent old version."""
    marker = 'PX4_AGENT_STATIONARY_HEALTH_V9'
    directory = Path(__file__).resolve().parent / 'patches'
    files = [root / 'vins_estimator/src/estimator' / name for name in
             ['estimator.cpp', 'estimator.h', 'parameters.cpp', 'parameters.h']]
    visual_marker = 'PX4_AGENT_VISUAL_LIFECYCLE_DIAGNOSTICS_V1'
    visual_marked = [visual_marker in path.read_text() for path in files]
    if any(visual_marked) and not all(visual_marked):
        raise RuntimeError('Partial native visual-lifecycle diagnostic patch; restore consistent source')
    versions = []
    for version in ('V9', 'V8', 'V7', 'V6', 'V5', 'V4', 'V3', 'V2', 'V1'):
        marked = ['PX4_AGENT_STATIONARY_HEALTH_' + version in path.read_text() for path in files]
        if any(marked) and not all(marked):
            raise RuntimeError('Partial native stationary-health patch; restore consistent source before building')
        if all(marked): versions.append(version)
    if len(versions) > 1:
        raise RuntimeError('Mixed native stationary-health versions')
    current = versions[0] if versions else None
    migrations = ['stationary_health_v%d_to_v%d.patch' % (version, version+1)
                  for version in range(1, 9)]
    names = migrations[int(current[1:])-1:] if current else ['stationary_health.patch']
    for name in names:
        command = ['patch', '--batch', '--forward', '--fuzz=0', '-p1',
                   '--directory', str(root), '--input', str(directory / name)]
        checked = subprocess.run(command + ['--dry-run'], stdout=subprocess.PIPE,
                                 stderr=subprocess.STDOUT, text=True)
        if checked.returncode:
            raise RuntimeError('Native stationary-health patch differs from verified source: ' + checked.stdout)
        applied = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        if applied.returncode:
            raise RuntimeError('Native stationary-health patch failed: ' + applied.stdout)
    if not all(marker in path.read_text() for path in files):
        raise RuntimeError('Native stationary-health patch did not update every source file')
    if not all(visual_marker in path.read_text() for path in files):
        command = ['patch', '--batch', '--forward', '--fuzz=0', '-p1',
                   '--directory', str(root), '--input', str(directory/'stationary_health_v9_visual_diagnostics.patch')]
        checked = subprocess.run(command + ['--dry-run'], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        if checked.returncode:
            raise RuntimeError('Native visual-lifecycle patch differs from verified V9 source: '+checked.stdout)
        applied = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        if applied.returncode or not all(visual_marker in path.read_text() for path in files):
            raise RuntimeError('Native visual-lifecycle patch did not update all source files: '+applied.stdout)
    for name, required in [('marginalization_factor.cpp', 'PX4_AGENT_MARG_ARRAY_DELETE'),
                           ('marginalization_factor.h', 'PX4_AGENT_BIAS_DIAGNOSTICS_V1'),
                           ('marginalization_factor.h', 'PX4_AGENT_EXACT_PRIOR_POSE_JACOBIAN')]:
        if required not in (root/'vins_estimator/src/factor'/name).read_text():
            raise RuntimeError('Partial native marginalization diagnostic patch')
    for name in ('stationary_factors.h', 'temporal_stereo_window.h', 'imu_refresh_health.h', 'imu_chain_health.h',
                 'bias_information.h', 'bias_snapshot.h', 'bias_native_diagnostics.h', 'fixed_yaw_pose_parameterization.h',
                 'stationary_initialization.h', 'visual_lifecycle_diagnostics.h'):
        (root / 'vins_estimator/src/factor' / name).write_text((directory / name).read_text())


def main():
    root = Path(__file__).resolve().parent.parent / 'vio_ws' / 'src' / 'VINS-Fusion'
    names = {
        'CV_GRAY2BGR': 'cv::COLOR_GRAY2BGR', 'CV_BGR2GRAY': 'cv::COLOR_BGR2GRAY',
        'CV_GRAY2RGB': 'cv::COLOR_GRAY2RGB', 'CV_AA': 'cv::LINE_AA',
        'CV_LOAD_IMAGE_GRAYSCALE': 'cv::IMREAD_GRAYSCALE',
        'CV_TERMCRIT_EPS': 'cv::TermCriteria::EPS', 'CV_TERMCRIT_ITER': 'cv::TermCriteria::COUNT',
        'CV_SHAPE_CROSS': 'cv::MORPH_CROSS', 'CV_SHAPE_RECT': 'cv::MORPH_RECT',
        'cv::SHAPE_CROSS': 'cv::MORPH_CROSS', 'cv::SHAPE_RECT': 'cv::MORPH_RECT',
    }
    for name in ('CALIB_CB_ADAPTIVE_THRESH', 'CALIB_CB_NORMALIZE_IMAGE', 'CALIB_CB_FILTER_QUADS',
                 'CALIB_CB_FAST_CHECK', 'ADAPTIVE_THRESH_MEAN_C', 'THRESH_BINARY_INV',
                 'THRESH_BINARY', 'RETR_CCOMP', 'CHAIN_APPROX_SIMPLE'):
        names['CV_' + name] = 'cv::' + name
    changed = []
    for path in root.rglob('*'):
        if path.suffix not in ('.cpp', '.cc', '.h'):
            continue
        original = content = path.read_text()
        for old, new in sorted(names.items(), key=lambda item: -len(item[0])):
            content = content.replace(old, new)
        if path.name == 'visualization.cpp':
            content = content.replace('odometry.child_frame_id = "world";', 'odometry.child_frame_id = "body";')
        if content != original:
            path.write_text(content)
            changed.append(str(path.relative_to(root)))
    for package in ('camera_models', 'vins_estimator'):
        path = root / package / 'CMakeLists.txt'
        content = path.read_text().replace('-O3 -Wall -g', '-O2 -Wall').replace('-O3 -fPIC', '-O2 -fPIC')
        path.write_text(content)
    patch_estimator(root)
    patch_bias_prior(root)
    patch_stereo_monotonic(root)
    patch_opencv_threads(root)
    patch_stationary_health(root)
    print('Applied OpenCV 4, body frame, shutdown, failure handling patches:', ', '.join(changed))


if __name__ == '__main__':
    main()
