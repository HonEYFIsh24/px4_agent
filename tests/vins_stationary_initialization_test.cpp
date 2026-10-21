// Production Estimator initialization regression. Synthetic sensor/feature
// inputs only: no ROS master, vehicle status, model requests or control calls.
// Link to actual vins_lib; argv[1] is the read-only actual camera calibration.
#include "estimator.h"
#include "fixed_yaw_pose_parameterization.h"
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <set>
#include <sstream>
#include <stdexcept>

namespace {
using FeatureImage = std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>>;
using Tree = boost::property_tree::ptree;
const Eigen::Vector3d acceleration(-.173961716717, -.15441814047, 9.92683909375);
const Eigen::Vector3d gyro(-.00102485092165, .00560275018514, .0000643927487078);
constexpr double image_dt = .1, imu_dt = .004, camera_fx = 320.;
constexpr int feature_count = 40, moving_frames = 61;

void require(bool okay, const std::string &reason) {
    if (!okay) throw std::runtime_error(reason);
}

Tree parse(const std::string &value) {
    Tree result; std::istringstream stream(value);
    boost::property_tree::read_json(stream, result);
    return result;
}

void configure(const std::string &camera, const std::string &output) {
    std::ifstream readable(camera);
    require(readable.good(), "actual camera YAML must be readable");
    // readParameters also opens the production YAML's output_path. Hand-set
    // globals instead; setParameter still loads the actual camera YAML.
    NUM_OF_CAM = 2; STEREO = USE_IMU = 1; MULTIPLE_THREAD = 0;
    STATIONARY_INITIALIZATION = ENABLE_ZUPT = 1;
    EXACT_MARGINALIZATION_POSE_JACOBIAN = FIX_FIRST_POSE_XYZ_YAW = 1;
    ESTIMATE_EXTRINSIC = ESTIMATE_TD = 0; TD = 0.; ROW = 480; COL = 640;
    INIT_DEPTH = 5.; MIN_PARALLAX = 10. / FOCAL_LENGTH;
    MAX_CNT = 180; MIN_DIST = 20; F_THRESHOLD = 1.; SHOW_TRACK = FLOW_BACK = 0;
    G = Eigen::Vector3d(0., 0., 9.8066);
    ACC_N = .029409182239566; GYR_N = .0029511956263521;
    ACC_W = .094867697353033; GYR_W = .00061324346876081;
    ACC_BIAS_PRIOR_SIGMA = .196; GYR_BIAS_PRIOR_SIGMA = .0087;
    BIAS_ACC_THRESHOLD = .1; BIAS_GYR_THRESHOLD = .1;
    SOLVER_TIME = .12; NUM_ITERATIONS = 12;
    MAX_IMU_PREINTEGRATION_SPAN_S = 1.;
    ZUPT_CAMERA_FX = camera_fx; ZUPT_VELOCITY_SIGMA = .03; ZUPT_POSITION_SIGMA = .02;
    HEALTH_IMU_TO_BODY_FLU = Eigen::Vector3d(0., 0., -.02);
    // Preserve production derivative/gauge diagnostics, while avoiding large
    // information-matrix dump experiments in this bounded integration test.
    BIAS_DIAGNOSTICS_DIR.clear();
    OUTPUT_FOLDER = output; VINS_RESULT_PATH = output + "/vio.csv";
    EX_CALIB_RESULT_PATH = output + "/extrinsic.csv";
    CAM_NAMES = {camera, camera};
    Eigen::Matrix3d mounting;
    mounting << 0., -std::sin(.35), std::cos(.35),
               -1., 0., 0., 0., -std::cos(.35), -std::sin(.35);
    RIC = {mounting, mounting};
    TIC = {Eigen::Vector3d(.12, .06, .16), Eigen::Vector3d(.12, -.06, .16)};
}

FeatureImage features(double shift_px, double flow_px_s) {
    FeatureImage result;
    for (int id = 0; id < feature_count; ++id) {
        const double depth = .9 + .035 * id;
        const double u = 220. + 28. * (id % 8) + shift_px;
        const double v = 175. + 28. * (id / 8);
        Eigen::Matrix<double, 7, 1> left, right;
        left << (u-320.)/camera_fx, (v-240.)/camera_fx, 1., u, v, flow_px_s/camera_fx, 0.;
        right = left;
        right[0] -= .12/depth; right[3] -= camera_fx*.12/depth;
        result[id] = {{0, left}, {1, right}};
    }
    return result;
}

void imu_interval(Estimator &estimator, double stamp) {
    // Actual production midpoint integration, with original 250 Hz sample dt.
    for (int step = 1; step <= 25; ++step) {
        const double source_stamp=stamp-image_dt+step*imu_dt;
        // processIMU is the window-integration path; inputIMU normally also
        // advances the independently published source trajectory. Execute its
        // actual fastPredict segment with these same raw samples, omitting
        // only ROS publication, so same-time refresh has a real source match.
        if (estimator.solver_flag==Estimator::NON_LINEAR)
            estimator.fastPredictIMU(source_stamp, acceleration, gyro);
        estimator.processIMU(source_stamp, imu_dt, acceleration, gyro);
    }
}

void check_pending_window(const Estimator &estimator, int image_index) {
    require(estimator.frame_count <= WINDOW_SIZE, "delayed INIT exceeded the real state-array boundary");
    require(estimator.last_marginalization_info == nullptr, "waiting INIT created an optimization prior");
    require(estimator.stationary_initialization_seed_count == 0
            && !estimator.stationary_initialization_seeded, "unsupported inputs initialized a stationary seed");
    require(estimator.initial_bias_prior_insertion_count == 0, "unsupported inputs inserted a bias prior");
    require(estimator.solver_iterations == 0, "waiting INIT performed an optimizer solve");
    if (image_index < WINDOW_SIZE) return;
    require(estimator.marginalization_flag == Estimator::MARGIN_OLD,
            "full waiting window must drop OLD even when parallax is small");
    // OLD leaves a copied prediction slot at index 10. It is not a second
    // captured frame: the exact cache keys equal the DISTINCT current Headers.
    std::set<double> expected;
    for (int index = 0; index <= WINDOW_SIZE; ++index) expected.insert(estimator.Headers[index]);
    require(expected.size() == WINDOW_SIZE, "after-slide prediction slot does not duplicate latest source header");
    require(estimator.all_image_frame.size() == expected.size(), "image cache retained an OLD head or lost an active header");
    std::set<const IntegrationBase *> image_edges;
    for (const auto &frame : estimator.all_image_frame) {
        require(expected.count(frame.first) == 1, "cached image header has no current pose slot");
        const IntegrationBase *edge = frame.second.pre_integration;
        require(edge != nullptr, "retained delayed-window frame lost its original IMU interval");
        require(edge != estimator.tmp_pre_integration && image_edges.insert(edge).second,
                "image-edge ownership aliases tmp or another frame");
        for (int index = 0; index <= WINDOW_SIZE; ++index)
            require(edge != estimator.pre_integrations[index], "image IMU edge aliases mutable optimizer interval");
    }
    require(estimator.tmp_pre_integration && estimator.tmp_pre_integration->sum_dt == 0.,
            "new tmp interval already contains reused samples");
    require(estimator.pre_integrations[WINDOW_SIZE]
            && estimator.pre_integrations[WINDOW_SIZE]->sum_dt == 0., "new prediction-slot interval was not cleared");
    for (int index = 1; index < WINDOW_SIZE; ++index) {
        const double span = estimator.Headers[index]-estimator.Headers[index-1];
        require(span > 0. && std::abs(span-estimator.pre_integrations[index]->sum_dt) < 1e-9,
                "optimizer IMU span differs from adjacent captured headers");
    }
}

double maximum_stereo_error_px(const Estimator &estimator) {
    double maximum = 0.; int checked = 0;
    for (const auto &feature : estimator.f_manager.feature) {
        if (feature.estimated_depth <= 0. || feature.feature_per_frame.empty()) continue;
        const auto &first = feature.feature_per_frame.front();
        if (!first.is_stereo) continue;
        const Eigen::Vector3d point = estimator.ric[1].transpose()
            *(estimator.ric[0]*(feature.estimated_depth*first.point)+estimator.tic[0]-estimator.tic[1]);
        require(point.allFinite() && point.z() > 0., "accepted stereo depth is not finite/front-facing");
        maximum = std::max(maximum, camera_fx*(point.head<2>()/point.z()-first.pointRight.head<2>()).norm());
        ++checked;
    }
    require(checked >= 20, "actual optimizer lacks the required near stereo geometry support");
    return maximum;
}

void check_actual_rp_freedom(const Estimator &estimator) {
    double pose[7], changed[7], yaw, pitch, roll, changed_yaw, changed_pitch, changed_roll;
    const Eigen::Quaterniond q(estimator.Rs[0]);
    for (int axis=0; axis<3; ++axis) pose[axis]=estimator.Ps[0][axis];
    pose[3]=q.x(); pose[4]=q.y(); pose[5]=q.z(); pose[6]=q.w();
    require(fixedYawPoseZyx(pose,yaw,pitch,roll), "actual first solved pose is outside the active RP chart");
    FixedYawPoseParameterization chart(pose);
    const double delta[2] = {1., -.7};
    require(chart.LocalSize()==2 && chart.Plus(pose,delta,changed), "actual first-pose gauge removed RP freedom");
    require(fixedYawPoseZyx(changed,changed_yaw,changed_pitch,changed_roll), "RP perturbation lost valid orientation");
    for(int axis=0;axis<3;++axis) require(changed[axis]==pose[axis], "RP gauge changed first XYZ");
    require(std::abs(std::atan2(std::sin(changed_yaw-yaw),std::cos(changed_yaw-yaw)))<1e-9,
            "RP gauge changed the fixed coordinate yaw");
    require(std::abs(changed_pitch-pitch)>1e-5 && std::abs(changed_roll-roll)>1e-5,
            "first pose roll or pitch was frozen");
}
} // namespace

int main(int argc, char **argv) {
    try {
        require(argc==2 || (argc==3 && std::string(argv[2])=="--visual-diagnostics"), "usage: vins_stationary_initialization_test READ_ONLY_CAMERA_YAML [--visual-diagnostics]");
        char output_template[] = "/tmp/vins_stationary_initialization_XXXXXX";
        const char *created = ::mkdtemp(output_template);
        require(created!=nullptr, "exclusive temporary evidence directory could not be created");
        const std::string output(created);
        configure(argv[1],output);
        VISUAL_LIFECYCLE_DIAGNOSTICS = argc==3 ? 1 : 0;
        // Mirrors the production zero-initialized global instance; no NodeHandle,
        // subscribers, processMeasurements thread, ROS master or FCU needed.
        static Estimator estimator;
        estimator.setParameter();
        std::vector<std::pair<double,Eigen::Vector3d>> initial_acc;
        for(int index=0;index<25;++index) initial_acc.push_back({.9+index*imu_dt,acceleration});
        estimator.initFirstIMUPose(initial_acc);
        std::ofstream evidence(output+"/actual_estimator_health.jsonl");
        require(evidence.good(), "temporary source health log could not be opened");
        int prior_insertions=0, total_frames=0;
        double last_stamp=0., shift=0., stereo_before=0., stereo_after=0.;
        auto process = [&](int index,double flow) {
            const double stamp=1.+index*image_dt;
            imu_interval(estimator,stamp);
            estimator.processImage(features(shift,flow),stamp);
            const std::string health=estimator.nativeHealthJson(stamp);
            estimator.appendNativeHealthJson(health);
            require(!estimator.native_health_trace_error, "production initialization evidence writer failed");
            evidence<<health<<"\n"; evidence.flush();
            require(!estimator.failure_occur, "actual Estimator marked failure: "+estimator.solver_health_reason);
            const Tree audit=parse(health);
            if(audit.get<bool>("imu_optimizer_window.initial_bias_prior_added_this_solve",false)) ++prior_insertions;
            last_stamp=stamp; ++total_frames;
            return audit;
        };
        // Constant-speed features distinguish motion from quiet IMU: duration
        // exceeds five simulation seconds and the original 11-slot window.
        for(int index=0;index<moving_frames;++index) {
            shift=.6*index*image_dt;
            const Tree audit=process(index,.6);
            require(estimator.solver_flag==Estimator::INITIAL && !audit.get<bool>("tracking_ok"),
                    "quiet IMU plus sustained feature motion incorrectly enabled tracking");
            check_pending_window(estimator,index);
            require(prior_insertions==0, "waiting INIT inserted an initial-bias prior");
        }
        require(last_stamp-1.>5. && estimator.stationary_initialization_wait_count>0,
                "test did not exercise a genuinely delayed full-window initialization");
        stereo_before=maximum_stereo_error_px(estimator);
        require(stereo_before<.05, "synthetic negative case lacks consistent actual stereo geometry");
        int seeded_frame=-1;
        for(int index=moving_frames;index<moving_frames+80;++index) {
            // The visual motion stops; no supplied velocity, ground state,
            // orientation truth, manually injected pose or prior is used.
            const Tree audit=process(index,0.);
            if(estimator.solver_flag==Estimator::NON_LINEAR) {
                seeded_frame=index;
                require(audit.get<bool>("tracking_ok") && estimator.solver_solution_usable,
                        "tracking preceded a usable actual solve/propagation refresh");
                break;
            }
            check_pending_window(estimator,index);
        }
        require(seeded_frame>=moving_frames+WINDOW_SIZE, "init did not wait for a complete supported static window");
        require(estimator.stationary_initialization_seeded && estimator.stationary_initialization_seed_count==1,
                "sensor-supported window did not seed exactly once");
        require((estimator.stationary_initialization_seed_acc-acceleration).norm()<1e-10
                && (estimator.stationary_initialization_seed_gyro-gyro).norm()<1e-10,
                "initial seed means do not come from the actually supplied original IMU samples");
        require(estimator.stationary_initialization_seed_imu_samples>=100
                && estimator.stationary_initialization_seed_last_imu_stamp-estimator.stationary_initialization_seed_first_imu_stamp>=1.-.012
                && std::abs(estimator.stationary_initialization_seed_stamp-last_stamp)<1e-9,
                "seed source evidence does not cover the complete current camera window");
        const Tree initialization=parse(estimator.native_initialization_evidence_json);
        for(const auto &frame : initialization.get_child("stationary_frames"))
            require(frame.second.get_value<bool>(), "first solve was admitted with a nonstationary source frame");
        require(prior_insertions==1 && estimator.initial_bias_prior_insertion_count==1
                && estimator.last_marginalization_info && estimator.last_marginalization_info->valid,
                "first actual solve did not insert and marginalize one valid initial bias prior");
        require(estimator.marginalization_flag==Estimator::MARGIN_OLD, "first solve failed to build an OLD prior");
        stereo_after=maximum_stereo_error_px(estimator);
        require(stereo_after<.1 && std::abs(stereo_after-stereo_before)<.1,
                "stationary seed/actual solve corrupted calibrated stereo geometry");
        check_actual_rp_freedom(estimator);
        for(int index=seeded_frame+1;index<=seeded_frame+5;++index) {
            const Tree audit=process(index,0.);
            require(estimator.solver_flag==Estimator::NON_LINEAR && audit.get<bool>("tracking_ok"),
                    "supported initialization did not remain tracking in subsequent actual solves");
            require(estimator.stationary_initialization_seed_count==1 && prior_insertions==1
                    && estimator.initial_bias_prior_insertion_count==1,
                    "subsequent solve reused the initialization seed or duplicated the initial bias prior");
        }
        require(estimator.Bas[0].allFinite() && estimator.Bgs[0].allFinite(), "actual solved bias is nonfinite");
        const int completed_seed_count=estimator.stationary_initialization_seed_count;
        // Exercise production ownership cleanup after the retained delayed
        // cache and actual marginalization prior have both been populated.
        estimator.clearState();
        require(estimator.all_image_frame.empty() && estimator.tmp_pre_integration==nullptr
                && estimator.last_marginalization_info==nullptr && estimator.stationary_initialization_seed_count==0
                && !estimator.stationary_initialization_seeded && estimator.initial_bias_prior_insertion_count==0,
                "production epoch reset retained initialization ownership/state");
        for(int index=0;index<=WINDOW_SIZE;++index)
            require(estimator.pre_integrations[index]==nullptr, "production reset retained a window IMU object");
        std::ofstream summary(output+"/summary.json");
        summary<<std::setprecision(12)<<"{\"passed\":true,\"scope\":\"offline_actual_Estimator\","
               <<"\"actual_ros_master\":false,\"actual_flight\":false,\"ground_truth_feedback\":false,"
               <<"\"moving_negative_duration_sim_s\":"<<(moving_frames-1)*image_dt
               <<",\"moving_flow_px_s\":0.6,\"seed_count\":"<<completed_seed_count
               <<",\"initial_prior_insertions\":"<<prior_insertions<<",\"seeded_frame\":"<<seeded_frame
               <<",\"total_actual_process_image_calls\":"<<total_frames
               <<",\"maximum_stereo_error_before_px\":"<<stereo_before
               <<",\"maximum_stereo_error_after_px\":"<<stereo_after
               <<",\"first_pose_roll_pitch_free\":true,\"output_directory\":\""<<output<<"\"}\n";
        std::cout<<"PASS: actual Estimator delayed INIT, quiet-IMU moving-stereo negative, mapped OLD windows, "
                 <<"single supported seed/prior, actual subsequent solves and RP freedom; evidence="<<output<<"\n";
        return 0;
    } catch(const std::exception &error) {
        std::cerr<<"FAIL: "<<error.what()<<"\n";
        return 1;
    }
}
