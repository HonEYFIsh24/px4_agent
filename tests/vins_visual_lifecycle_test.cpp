// Actual native backend deletion pipeline, no ROS master or estimator inputs
// from ground truth. A separate integration test checks automatic solve stages.
#include "estimator.h"
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace {
using Tree = boost::property_tree::ptree;
static Estimator estimator; // same zero-initialized lifetime as the native node
void require(bool condition,const std::string &message) {
    if (!condition) throw std::runtime_error(message);
}
Tree parse(const std::string &text) {
    Tree value;std::istringstream stream(text);
    boost::property_tree::read_json(stream,value);return value;
}
void configure(int diagnostic) {
    estimator.clearState();
    VISUAL_LIFECYCLE_DIAGNOSTICS=diagnostic;
    USE_IMU=0;STEREO=1;MULTIPLE_THREAD=0;
    estimator.frame_count=3;
    for(int i=0;i<=WINDOW_SIZE;++i) {
        estimator.Rs[i].setIdentity();estimator.Ps[i].setZero();
        estimator.Headers[i]=.1*i;
    }
    estimator.ric[0].setIdentity();estimator.ric[1].setIdentity();
    estimator.tic[0].setZero();estimator.tic[1]=Eigen::Vector3d(.12,0,0);
}
void feature(int id,int count,bool inconsistent) {
    FeaturePerId value(id,0);value.estimated_depth=2.;
    // solve_flag=0 deliberately: eligibility is observation count alone.
    for(int frame=0;frame<count;++frame) {
        Eigen::Matrix<double,7,1> left,right;
        const double x=.1+(inconsistent && frame>0 ? .1:0.);
        left<<x,.02,1.,320*x+320,246.4,0.,0.;right=left;
        right[0]-=.12/2.;right[3]-=320*.12/2.;
        FeaturePerFrame observation(left,0.);observation.rightObservation(right);
        value.feature_per_frame.push_back(observation);
    }
    estimator.f_manager.feature.push_back(value);
}
std::set<int> delete_actual_outliers() {
    estimator.beginVisualDiagnostics();
    if(VISUAL_LIFECYCLE_DIAGNOSTICS)
        estimator.native_visual_lifecycle.solved=NativeVisualLifecycle::snapshot(estimator.f_manager,"post_solve_pre_outlier");
    std::set<int> removed;
    estimator.outliersRejection(removed);
    estimator.f_manager.removeOutlier(removed);
    estimator.finishVisualOutlierDiagnostics(removed);
    return removed;
}
std::vector<int> retained_ids() {
    std::vector<int> ids;for(const auto &f:estimator.f_manager.feature)ids.push_back(f.feature_id);return ids;
}
}
int main() {
    try {
        configure(1);feature(10,4,false);feature(20,4,true);feature(30,3,true);
        const auto removed=delete_actual_outliers();const auto retained=retained_ids();
        require(removed==std::set<int>{20},"actual deletion did not distinguish good/bad/young features");
        const Tree evidence=parse(estimator.native_visual_lifecycle.json(true));
        require(evidence.get<int>("before_solve.total_features")==3 && evidence.get<int>("before_solve.eligible_features")==2,"before snapshot does not reflect the actual FM");
        require(evidence.get<int>("post_solve_pre_outlier.eligible_features")==2,"pre-removal snapshot lost maturity evidence");
        require(evidence.get<int>("post_outlier_before_slide.total_features")==2 && evidence.get<int>("post_outlier_before_slide.eligible_features")==1,"after snapshot was captured before actual deletion");
        require(evidence.get<int>("removed_ids_total")==1 && evidence.get<bool>("outliers_evaluated"),"actual deleted ID not recorded");
        require(evidence.get<int>("evaluated_feature_count")==2,"young tracks were falsely called reprojection-evaluated");
        bool actual_error=false;
        for(const auto &item:evidence.get_child("reprojections")) {
            const Tree &row=item.second;
            if(row.get<int>("id")==20) {
                actual_error=true;
                require(row.get<int>("error_terms")==7,"stereo/temporal terms differ from actual original loop");
                require(std::abs(row.get<double>("average_normalized_error")-6.*.1/7.)<1e-11,"diagnostic is not the actual outlier mean");
                require(row.get<bool>("removed") && row.get<double>("average_virtual_px_error")>3.,"original deletion decision misreported");
            }
        }
        require(actual_error,"bad ID actual average absent");
        // Repeat exactly the same native deletion inputs with diagnostics OFF.
        configure(0);feature(10,4,false);feature(20,4,true);feature(30,3,true);
        const auto disabled_removed=delete_actual_outliers();
        require(disabled_removed==removed && retained_ids()==retained,"diagnostic flag changes physical deletion");
        require(!estimator.native_visual_lifecycle.sampled && estimator.native_visual_solve_sequence==0,"disabled diagnostic traversed/stored a snapshot");
        require(estimator.native_visual_lifecycle.json(false)=="{\"enabled\":false}","disabled health is not minimal");
        // All full counts remain actual even when bounded rows/IDs are omitted.
        configure(1);
        for(int id=0;id<300;++id)feature(id,4,true);
        const auto many_removed=delete_actual_outliers();
        const Tree capped=parse(estimator.native_visual_lifecycle.json(true));
        require(many_removed.size()==300 && estimator.f_manager.feature.empty(),"output limits changed deletion of features beyond a cap");
        require(capped.get<int>("before_solve.total_features")==300 && capped.get<int>("before_solve.eligible_features")==300,"capped count hides actual maturity");
        require(capped.get<int>("before_solve.recorded_features")==128 && capped.get<int>("before_solve.omitted_features")==172 && capped.get<bool>("before_solve.truncated"),"feature cap lacks explicit omission evidence");
        require(capped.get<int>("evaluated_feature_count")==300 && capped.get<int>("omitted_reprojection_rows")==172 && capped.get<bool>("reprojection_rows_truncated"),"reprojection cap changes total evaluation count");
        require(capped.get<int>("removed_ids_total")==300 && capped.get_child("removed_ids").size()==256 && capped.get<int>("removed_ids_omitted")==44 && capped.get<bool>("removed_ids_truncated"),"removed-ID cap conceals actual deletion");
        require(capped.get<int>("post_outlier_before_slide.eligible_features")==0,"empty backend incorrectly called visually supported");
        const auto used=estimator.native_visual_lifecycle.before.rows;
        require(used.front().solve_flag==0,"eligibility secretly requires a solved flag");
        // Nonfinite diagnostic numbers are null, not invalid JSON or replacements.
        estimator.native_visual_lifecycle.before.rows[0].depth=std::numeric_limits<double>::quiet_NaN();
        const Tree finite_json=parse(estimator.native_visual_lifecycle.json(true));
        require(finite_json.get_child("before_solve.features").front().second.get<std::string>("estimated_depth_m")=="null","nonfinite depth not reported honestly");
        estimator.clearState();
        std::cout<<"PASS: actual visual lifecycle good/bad/young, before/after deletion, disabled-identical physics, 128/256 caps and nonfinite null\n";
        return 0;
    } catch(const std::exception &error) {
        std::cerr<<"FAIL: "<<error.what()<<"\n";return 1;
    }
}
