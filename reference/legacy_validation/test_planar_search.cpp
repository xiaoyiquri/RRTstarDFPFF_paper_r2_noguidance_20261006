#include "core/rrtstar.h"
#include "systems/dyno_model.h"
#include "systems/linear_model.h"
#include "tests/test_support.h"
#include <functional>
#include <memory>

using namespace rrtpff;

// Test-only boundary spy. It does NOT implement or certify real DynoBench
// dynamics. The actual six-environment validator remains RealDynoAdapter.
struct BoundarySpy {
    DynoProblemData data;
    std::function<bool(const Eigen::VectorXd &)> free = [](const auto &) { return true; };
    std::shared_ptr<std::size_t> dense_calls = std::make_shared<std::size_t>(0);
    const DynoProblemData &problem() const { return data; }
    bool stateValid(const Eigen::VectorXd &x) const { return x.allFinite(); }
    bool controlValid(const Eigen::VectorXd &u) const { return u.allFinite(); }
    bool collisionFree(const Eigen::VectorXd &x) const { return free(x); }
    DynoValidation checkTrajectory(const DynoSamples &s, const Eigen::VectorXd &,
                                   const Eigen::VectorXd &, DynoTolerances, bool) const {
        ++*dense_calls;
        for (const auto &x : s.states)
            if (!free(x))
                return {false, 0, 0, 0, s.states.size(), "FCL collision"};
        return {true, 0, 0, 0, s.states.size(), {}};
    }
};
BoundarySpy boundary(int dim) {
    BoundarySpy a;
    a.data.start = Eigen::VectorXd::Zero(dim);
    a.data.goal = Eigen::VectorXd::Zero(dim);
    a.data.start.head<2>() << 1, 1;
    a.data.goal.head<2>() << 4, 1;
    a.data.lower_position = Eigen::Vector2d(0, 0);
    a.data.upper_position = Eigen::Vector2d(6, 6);
    a.data.reference_dt = .01;
    return a;
}

class BudgetModel : public LinearModel {
  public:
    BudgetModel(Config c, double minimum_duration, bool never_valid = false)
        : LinearModel(c), minimum_duration_(minimum_duration), never_(never_valid) {
        goal_.setDerivative(0, c.goal);
    }
    TerminalConstraint<2, 2> goalConstraint() const {
        return TerminalConstraint<2, 2>::full(goal_);
    }
    bool isGoal(const FlatState &s, double tol) const {
        return (s.values - goal_.values).norm() <= tol;
    }
    bool precheckTrajectory(const FlatTrajectory &t, PlanResult &) const {
        return t.T >= minimum_duration_;
    }
    bool trajectoryValidAfterPrecheck(const FlatTrajectory &t, PlanResult &stats) const {
        ++stats.dyno_validation_count;
        return !never_ && t.T >= minimum_duration_;
    }
    bool trajectoryValid(const FlatTrajectory &t, PlanResult &stats) const {
        return trajectoryValidAfterPrecheck(t, stats);
    }

  private:
    FlatState goal_;
    double minimum_duration_;
    bool never_;
};

class TimingModel : public BudgetModel {
  public:
    explicit TimingModel(Config config) : BudgetModel(config, 0) {}
    std::vector<double> additionalCandidateTimes(const FlatState &,
                                                 const TerminalConstraint<2, 2> &) const {
        return {-1, 100, std::numeric_limits<double>::quiet_NaN(), 4.123, 4.123};
    }
    bool precheckTrajectory(const FlatTrajectory &t, PlanResult &) const {
        return std::abs(t.T - 4.123) < 1e-10;
    }
};

int main() {
    return runTest([] {
        LinearModel::Config mc;
        mc.start.values << 1, 1, 0, 0;
        mc.goal << 2, 1;
        mc.upper << 6, 6;
        PlannerConfig pc;
        pc.timeout = 10;
        pc.max_iterations = 1;
        pc.goal_bias = 1;
        pc.explicit_goal_connection = true;
        pc.goal_tolerance = 1e-7;
        pc.simd_screen = false; // Isolate budget/cache policy from LinearModel's unrelated limits.
        pc.rewire_before_first_solution = false;
        PFFConfig times;
        times.min_time = .5;
        times.max_time = 5;
        times.additional_time_samples = 12;
        using Solver = PFFSolver<2, 2>;
        Solver solver(Solver::Weight::Identity(), times);
        for (bool simd : {false, true}) {
            pc.simd_enabled = simd;
            pc.candidate_precheck = false;
            RRTstar<BudgetModel> old_budget(BudgetModel(mc, 4), pc, solver);
            require(!old_budget.solve().success,
                    "old Top-K can exhaust budget on cheap-reject curves");
            pc.candidate_precheck = true;
            RRTstar<BudgetModel> new_budget(BudgetModel(mc, 4), pc, solver);
            auto success = new_budget.solve();
            require(success.success && success.dyno_validation_count == 1 &&
                        success.candidate_precheck_rejected > 0,
                    "precheck rejects do not consume the expensive validation budget");

            // With only one full-validation slot, remaining goal times must be
            // revisited on later iterations. Only validated rejections are cached.
            pc.cache_goal_connections = true;
            pc.max_iterations = 40;
            RRTstar<BudgetModel> cached(BudgetModel(mc, 0, true), pc, solver);
            const auto rejected = cached.solve();
            const auto curves =
                solver.candidates(mc.start, BudgetModel(mc, 0).goalConstraint()).size();
            require(!rejected.success && rejected.pff_query_count == 1 &&
                        rejected.dyno_validation_count == curves && rejected.goal_cache_hits > 0 &&
                        rejected.goal_cache_exhausted == 1,
                    "goal cache preserves unchecked times; simd=" + std::to_string(simd) +
                        " pff=" + std::to_string(rejected.pff_query_count) +
                        " validations=" + std::to_string(rejected.dyno_validation_count) +
                        " curves=" + std::to_string(curves) +
                        " hits=" + std::to_string(rejected.goal_cache_hits) +
                        " exhausted=" + std::to_string(rejected.goal_cache_exhausted));
            const auto reset = cached.solve();
            require(reset.pff_query_count == 1 && reset.dyno_validation_count == curves,
                    "goal cache is reset for every solve");
            RRTstar<BudgetModel> one_goal(BudgetModel(mc, 0), pc, solver);
            const auto once = one_goal.solve();
            require(once.success && once.node_count == 2 && once.pff_query_count == 1,
                    "cached success does not generate a chain of identical goal nodes");
            pc.cache_goal_connections = false;
            pc.max_iterations = 1;
            RRTstar<TimingModel> missing_time(TimingModel(mc), pc, solver);
            require(!missing_time.solve().success, "generic time hints are opt-in");
            pc.model_time_hints = true;
            RRTstar<TimingModel> hinted(TimingModel(mc), pc, solver);
            const auto with_hint = hinted.solve();
            require(with_hint.success && with_hint.pff_eval_count == curves + 1,
                    "time hints use fixedTime once, filtering duplicates and invalid times");
            close(hinted.path().back().T, 4.123, 1e-12, "model time is preserved");
            auto fixed_config = times;
            fixed_config.free_terminal_time = false;
            fixed_config.fixed_time = 2;
            RRTstar<TimingModel> fixed(TimingModel(mc), pc,
                                       Solver(Solver::Weight::Identity(), fixed_config));
            require(!fixed.solve().success, "fixed final time cannot be replaced by model hints");
            pc.model_time_hints = false;
        }

        using Uni = DynoSystemModel<UnicycleDF, BoundarySpy>;
        RobotValidationConfig validation;
        validation.collision_precheck_samples = 9;
        UnicycleDF::Config uc;
        uc.velocity_lower = -.5;
        uc.velocity_upper = .5;
        uc.omega_lower = -.5;
        uc.omega_upper = .5;
        auto spy = boundary(3);
        // Reject only at the terminal point: the sparse screen must stop before
        // allocating/reconstructing a dense PhysicalTrajectory.
        spy.free = [](const auto &x) { return x(0) < 1.29; };
        Uni uni(spy, UnicycleLift{.3}, UnicycleLift{.3}, uc, validation, .3);
        using USolver = PFFSolver<2, 2, UnicycleTrajectoryMetadata>;
        auto end = uni.start();
        end.setDerivative(0, Eigen::Vector2d(1.3, 1));
        auto edge = USolver{}.fixedTime(uni.start(), USolver::Constraint::full(end), 1);
        require(edge.success &&
                    uni.prepareEdge(uni.start(), uni.initialMetadata(), edge.trajectory),
                "straight motion edge");
        PlanResult counters;
        require(!uni.precheckTrajectory(edge.trajectory, counters) &&
                    counters.collision_precheck_rejected == 1 &&
                    counters.dense_reconstructions == 0 && *spy.dense_calls == 0,
                "colliding endpoint rejects before dense reconstruction");

        // An obstacle between all sparse sample positions must still be caught
        // by the dense validator after a successful precheck.
        auto narrow = boundary(3);
        narrow.free = [](const auto &x) { return std::abs(x(0) - 1.019) > .002; };
        Uni fine(narrow, UnicycleLift{.3}, UnicycleLift{.3}, uc, validation, .3);
        PlanResult full;
        require(fine.precheckTrajectory(edge.trajectory, full),
                "sparse screen may miss a tiny obstacle");
        require(!fine.trajectoryValidAfterPrecheck(edge.trajectory, full) &&
                    *narrow.dense_calls == 1,
                "full collision validation cannot be bypassed");

        PlanarSearchConfig turning;
        turning.turn_limited_sampling = true;
        turning.terminal_speed = .15;
        turning.duration_factors = {.9, 1, 1.1};
        auto open = boundary(3);
        Uni turning_model(open, UnicycleLift{.3}, UnicycleLift{.3}, uc, validation, .3);
        turning_model.setPlanarSearch(turning);
        const auto target =
            turning_model.limitExtension(turning_model.start(), Eigen::Vector2d(1, 5), .35);
        auto hints = turning_model.additionalExpansionConstraints(
            turning_model.start(), turning_model.initialMetadata(), target);
        require(target.x() > 1 && target.y() > 1 && hints.size() == 1,
                "turn-limited flat-output proposal follows initial velocity");
        bool found = false;
        PFFConfig utime;
        utime.min_time = .1;
        utime.max_time = 8;
        utime.additional_time_samples = 12;
        for (auto candidate : USolver(USolver::Weight::Identity(), utime)
                                  .candidates(turning_model.start(), hints[0])) {
            turning_model.prepareEdge(turning_model.start(), turning_model.initialMetadata(),
                                      candidate.trajectory);
            found |= UnicycleDF::feasible(candidate.trajectory, uc);
        }
        require(found, "turn boundary hint admits a physically valid nonzero-speed PFF curve");
        auto reverse_spy = boundary(3);
        reverse_spy.data.start(2) = reverse_spy.data.goal(2) = std::acos(-1.);
        auto reverse_config = uc;
        reverse_config.allow_reverse = true;
        Uni reverse(reverse_spy, UnicycleLift{-.3}, UnicycleLift{-.3}, reverse_config, validation,
                    -.3);
        reverse.setPlanarSearch(turning);
        const auto reverse_target =
            reverse.limitExtension(reverse.start(), Eigen::Vector2d(1, 5), .35);
        const auto reverse_hints = reverse.additionalExpansionConstraints(
            reverse.start(), reverse.initialMetadata(), reverse_target);
        close((target - reverse_target).norm(), 0, 1e-12,
              "turn proposals use world velocity consistently in reverse");
        require(reverse_hints.size() == 1 &&
                    (reverse_hints[0].target.values - hints[0].target.values).norm() < 1e-12 &&
                    reverse.initialMetadata().branch == VelocityBranch::Reverse,
                "additional terminal constraints preserve reverse branch metadata");
        auto wall_config = uc;
        wall_config.velocity_lower = .25;
        Uni wall(open, UnicycleLift{.3}, UnicycleLift{.3}, wall_config, validation, .3);
        bool invalid_hint = false;
        try {
            wall.setPlanarSearch(turning);
        } catch (const std::invalid_argument &) {
            invalid_hint = true;
        }
        require(invalid_hint, "wall speed lower bound cannot be bypassed by hints");
        turning.terminal_speed = .3;
        wall.setPlanarSearch(turning);
        wall_config.omega_lower = -.25;
        Uni asymmetric(open, UnicycleLift{.3}, UnicycleLift{.3}, wall_config, validation, .3);
        asymmetric.setPlanarSearch(turning);
        for (const auto &sample : {Eigen::Vector2d(1, 5), Eigen::Vector2d(1, -2)}) {
            const auto proposal = asymmetric.limitExtension(asymmetric.start(), sample, .5);
            const auto constraints = asymmetric.additionalExpansionConstraints(
                asymmetric.start(), asymmetric.initialMetadata(), proposal);
            bool feasible = false;
            for (double time :
                 asymmetric.additionalCandidateTimes(asymmetric.start(), constraints[0])) {
                auto candidate = USolver{}.fixedTime(asymmetric.start(), constraints[0], time);
                asymmetric.prepareEdge(asymmetric.start(), asymmetric.initialMetadata(),
                                       candidate.trajectory);
                feasible |=
                    candidate.success && UnicycleDF::feasible(candidate.trajectory, wall_config);
            }
            require(feasible,
                    "speed-informed times retain feasible turns under asymmetric Wall controls");
        }
        PlanResult invalid_stats;
        require(
            !uni.precheckTrajectory(UnicycleDF::FlatTrajectory{}, invalid_stats) &&
                invalid_stats.df_failures[static_cast<std::size_t>(FeasibilityFailure::Numeric)] ==
                    1,
            "invalid precheck input fails explicitly before trajectory evaluation");

        using Quad = DynoSystemModel<Quad2DDF, BoundarySpy>;
        Quad2DLift lift;
        lift.acceleration.setZero();
        lift.jerk.setZero();
        Quad quad(boundary(6), lift, lift, {}, validation);
        PlanarSearchConfig qs;
        qs.rest_hint = true;
        qs.tilt_hints.push_back({{Eigen::Vector2d(2.98, 2.8), Eigen::Vector2d(3.02, 3.3)},
                                 Eigen::Vector2d(0, -2),
                                 .85,
                                 .35});
        quad.setPlanarSearch(qs);
        const auto qc =
            quad.additionalExpansionConstraints(quad.start(), {}, Eigen::Vector2d(3, 3));
        require(qc.size() == 2, "hover and tilt are supplemental explicit boundary candidates");
        Quad2DDF::PhysicalState state;
        Quad2DDF::PhysicalControl control;
        Quad2DDF::FlatControl snap;
        require(Quad2DDF::sampleFeasible(qc[1].target, snap, {}, state, control),
                "tilt hint actuator feasibility");
        close(state(2), .85, 1e-12, "acceleration hint reconstructs requested physical attitude");
        require(.6 * std::abs(std::cos(state(2))) + .2 * std::abs(std::sin(state(2))) < .6,
                "tilted box can have positive projected clearance in the 0.6-wide opening");
        require(
            quad.additionalExpansionConstraints(quad.start(), {}, Eigen::Vector2d(2, 4)).size() ==
                1,
            "tilt hints are restricted to their configured flat-output region");
        require(quad.additionalExpansionConstraints(quad.start(), {}, quad.goalOutput()).empty(),
                "hint policy never changes actual goal derivatives");
    });
}
