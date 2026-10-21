#include "imu_chain_health.h"
#include "imu_bias_prior.h"
#include "marginalization_factor.h"
#include <cassert>
#include <iostream>
#include <numeric>

class BiasRandomWalk : public ceres::SizedCostFunction<6, 9, 9>
{
public:
    explicit BiasRandomWalk(double seconds): acc_sigma_(.006 * std::sqrt(seconds)),
        gyro_sigma_(.000038785 * std::sqrt(seconds)) {}
    bool Evaluate(double const* const* p, double* r, double** j) const override
    {
        for (int axis = 0; axis < 6; ++axis)
        {
            const double weight = 1 / (axis < 3 ? acc_sigma_ : gyro_sigma_);
            r[axis] = (p[1][axis+3]-p[0][axis+3]) * weight;
            if (j) for (int block = 0; block < 2; ++block) if (j[block])
                for (int col = 0; col < 9; ++col)
                    j[block][axis*9+col] = col == axis+3 ? (block ? weight : -weight) : 0;
        }
        return true;
    }
private:
    double acc_sigma_, gyro_sigma_;
};

void test_real_marginalized_bias_information()
{
    // Use actual native MarginalizationInfo/Factor, including its Eigen Schur
    // elimination and epsilon handling, to retain the one initial bias prior.
    // This linear bias subproblem does not assert full nonlinear observability.
    MarginalizationInfo* previous = nullptr;
    std::vector<double*> prior_blocks;
    double old_state[9] = {}, new_state[9] = {};
    for (int step = 1; step <= 30; ++step)
    {
        std::fill(new_state, new_state+9, 0.);
        for (int axis = 0; axis < 6; ++axis) new_state[axis+3] = .01 * (axis+1);
        MarginalizationInfo* next = new MarginalizationInfo();
        if (previous)
            next->addResidualBlockInfo(new ResidualBlockInfo(new MarginalizationFactor(previous), nullptr,
                prior_blocks, std::vector<int>{0}));
        else
            next->addResidualBlockInfo(new ResidualBlockInfo(new ImuBiasPrior(.196, .0087), nullptr,
                std::vector<double*>{old_state}, std::vector<int>{0}));
        next->addResidualBlockInfo(new ResidualBlockInfo(new BiasRandomWalk(1.), nullptr,
            std::vector<double*>{old_state, new_state}, std::vector<int>{0}));
        next->preMarginalize(); next->marginalize();
        assert(next->valid && next->n == 9 && next->linearized_jacobians.allFinite());
        std::unordered_map<long, double*> shifts;
        shifts[reinterpret_cast<long>(new_state)] = old_state;
        std::vector<double*> new_blocks = next->getParameterBlocks(shifts);
        assert(new_blocks.size() == 1 && new_blocks[0] == old_state);
        const Eigen::MatrixXd information = next->linearized_jacobians.transpose() * next->linearized_jacobians;
        for (int axis = 0; axis < 6; ++axis)
        {
            const double initial_variance = axis < 3 ? .196*.196 : .0087*.0087;
            const double process_variance = axis < 3 ? .006*.006 : .000038785*.000038785;
            const double expected_information = 1 / (initial_variance + step*process_variance);
            assert(std::abs(information(axis+3, axis+3)-expected_information) < expected_information*1e-7);
        }
        delete previous;
        previous = next; prior_blocks = new_blocks;
        std::copy(new_state, new_state+9, old_state);
    }
    // The retained marginal factor alone recovers the single zero-centered
    // statistical prior; no fresh per-window bias prior is inserted.
    ceres::Problem problem;
    problem.AddResidualBlock(new MarginalizationFactor(previous), nullptr, prior_blocks);
    problem.SetParameterization(old_state, new ceres::SubsetParameterization(9, {0, 1, 2}));
    ceres::Solver::Options options; options.linear_solver_type = ceres::DENSE_QR;
    ceres::Solver::Summary result; ceres::Solve(options, &problem, &result);
    assert(result.IsSolutionUsable());
    for (int axis = 3; axis < 9; ++axis) assert(std::abs(old_state[axis]) < 1e-6);
    delete previous;
}

struct WindowResult { int forced_old = 0, skipped = 0; double max_span = 0; bool chain_ever_broken = false; };

WindowResult simulate_thirty_seconds(double bound, bool periodically_visual)
{
    // Pure deterministic replay of native OLD/SECOND_NEW slot updates, with
    // genuine 4ms measurement-step IDs. This is a regression, not runtime proof.
    std::vector<std::vector<int>> links(11);
    std::vector<int> retired;
    for (int j = 1; j <= 10; ++j) for (int sample = 0; sample < 25; ++sample)
        links[j].push_back((j-1)*25+sample);
    int next_sample = 250;
    WindowResult result;
    for (int image = 0; image < 300; ++image)
    {
        const double previous_span = links[9].size()*.004, current_span = links[10].size()*.004;
        const bool forced = forceKeyframeForImuSpan(true, true, previous_span, current_span, bound);
        const bool visual = periodically_visual && image%4 == 0;
        if (forced) ++result.forced_old;
        std::vector<bool> roots(11, false), included;
        roots[0] = true;
        for (int j = 1; j <= 10; ++j)
        {
            const double span = links[j].size()*.004;
            result.max_span = std::max(result.max_span, span);
            included.push_back(span <= 10.);
            if (span > 10.) ++result.skipped;
        }
        if (!priorReachesLatestViaImu(roots, included)) result.chain_ever_broken = true;
        if (forced || visual)
        {
            // Standard OLD retires link1, carries its prior through that bridge,
            // then shifts each surviving raw interval. No state/dt clamping.
            retired.insert(retired.end(), links[1].begin(), links[1].end());
            for (int j = 1; j < 10; ++j) links[j] = links[j+1];
        }
        else
            links[9].insert(links[9].end(), links[10].begin(), links[10].end());
        links[10].clear();
        if (bound > 0) for (int j = 1; j <= 10; ++j)
            assert(links[j].size()*.004 <= bound+1e-12);
        // Every source interval ever received appears exactly once in either
        // retired (marginalized) history or the current active preintegrations.
        std::vector<int> all = retired;
        for (int j = 1; j <= 10; ++j) all.insert(all.end(), links[j].begin(), links[j].end());
        assert(static_cast<int>(all.size()) == next_sample);
        for (int i = 0; i < next_sample; ++i) assert(all[i] == i);
        for (int sample = 0; sample < 25; ++sample) links[10].push_back(next_sample++);
    }
    return result;
}

int main()
{
    assert(!forceKeyframeForImuSpan(true, true, .9, .1, 0));
    assert(!forceKeyframeForImuSpan(false, true, .9, .1, 1));
    assert(!forceKeyframeForImuSpan(true, false, .9, .1, 1));
    assert(forceKeyframeForImuSpan(true, true, .9, .1, 1));
    assert(!forceKeyframeForImuSpan(true, true, .8, .1, 1));
    // A real source gap is preserved and visible; no invented small dt.
    assert(forceKeyframeForImuSpan(true, true, .1, 2., 1));
    assert(!forceKeyframeForImuSpan(true, true, NAN, .1, 1));
    const auto old_policy = simulate_thirty_seconds(0., false);
    const auto bounded = simulate_thirty_seconds(1., false);
    const auto visual = simulate_thirty_seconds(1., true);
    assert(old_policy.max_span > 29. && old_policy.skipped > 0 && old_policy.chain_ever_broken);
    assert(bounded.forced_old >= 30 && bounded.skipped == 0 && !bounded.chain_ever_broken && bounded.max_span <= 1.);
    assert(visual.skipped == 0 && !visual.chain_ever_broken);
    assert(!priorReachesLatestViaImu({true, false, false}, {true, false}));
    assert(priorReachesLatestViaImu({false, true, false}, {false, true}));
    test_real_marginalized_bias_information();
    std::cout << "PASS: real 30s sliding-window span regression, raw IMU history, and actual Schur bias prior\n";
}
