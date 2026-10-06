#ifdef RRTPFF_QUAD3D_REFERENCE
#include "reference/search_fix_20260920/core/rrtstar.h"
#include "reference/search_fix_20260920/systems/quad3d_model.h"
#else
#include "core/rrtstar.h"
#include "systems/quad3d_model.h"
#endif
#include "tests/quad3d_trace.h"
#include "tests/test_support.h"
#include <fstream>
#include <iomanip>
#include <sstream>

using namespace rrtpff;

// Deterministic iteration-limited regression of the shared core at Quad3D's
// true flat dimensions/order with DF feasibility. It is NOT an FCL benchmark.
class FrozenQuad3Model : public Quad3DModel {
  public:
    FrozenQuad3Model(FlatState start, FlatOutput goal, FlatOutput lower, FlatOutput upper)
        : Quad3DModel(start, goal, lower, upper) {
        target_.setDerivative(0, goal);
    }
    TerminalConstraint<4, 4> goalConstraint() const {
        return TerminalConstraint<4, 4>::full(target_);
    }
    TerminalConstraint<4, 4> expansionConstraint(const FlatOutput &output) const {
        return (output - goalOutput()).norm() < 1e-10 ? goalConstraint()
                                                      : TerminalConstraint<4, 4>::output(output);
    }
    bool isGoal(const FlatState &state, double tolerance) const {
        return (state.values - target_.values).norm() <= tolerance;
    }

  private:
    FlatState target_;
};

std::string snapshot(bool simd) {
    FrozenQuad3Model::FlatState start;
    start.setDerivative(0, Eigen::Vector4d(1, 1, 1, 0));
    FrozenQuad3Model model(start, {2, 2, 1.5, 0}, {.3, .3, .3, 0}, {3, 3, 3, 0});
    PlannerConfig config;
    config.timeout = 60;
    config.max_iterations = 96;
    config.goal_bias = .2;
    config.goal_tolerance = 1e-6;
    config.range = .7;
    config.explicit_goal_connection = true;
    config.rewire_before_first_solution = false;
    config.simd_enabled = simd;
    config.seed = 40;
    PFFConfig pff;
    pff.min_time = .1;
    pff.max_time = 6;
    pff.additional_time_samples = 12;
    RRTstar<FrozenQuad3Model> planner(model, config,
                                      PFFSolver<4, 4>(Eigen::Matrix4d::Identity(), pff));
    const auto result = planner.solve();
    require(result.iterations == config.max_iterations,
            "baseline must end by iterations, not wall clock");
    std::ostringstream out;
    out << std::setprecision(17);
    out << result.success << ' ' << result.node_count << ' ' << result.pff_eval_count << ' '
        << result.rewire_count << ' ' << result.goal_connection_valid << '\n';
    for (const auto &node : planner.tree().nodes()) {
        out << node->id << ' ' << (node->parent ? static_cast<long long>(node->parent->id) : -1)
            << ' ' << node->cost;
        for (double value : node->state.values)
            out << ' ' << value;
        out << ' ' << node->incoming.T;
        for (Eigen::Index i = 0; i < node->incoming.coefficients.size(); ++i)
            out << ' ' << node->incoming.coefficients.data()[i];
        out << '\n';
    }
    return out.str();
}

int main(int argc, char **argv) {
    return runTest([&] {
        quad3d_test::checkComparator();
#ifdef RRTPFF_QUAD3D_REFERENCE
        require(argc == 2, "usage: quad3d_reference OUTPUT_DIRECTORY");
#else
        require(argc == 4, "usage: test_quad3d_baseline REFERENCE_DIRECTORY ACTUAL_DIRECTORY "
                           "HISTORICAL_DIRECTORY");
#endif
        for (bool simd : {false, true}) {
            const std::string suffix = simd ? "/quad3d_simd.txt" : "/quad3d_scalar.txt";
            const std::string mode = simd ? "SIMD" : "scalar";
            const auto actual = snapshot(simd);
#ifdef RRTPFF_QUAD3D_REFERENCE
            // Only this separately built, frozen OLD implementation may create
            // the current-toolchain reference. The new planner never blesses itself.
            quad3d_test::write(std::string(argv[1]) + suffix, actual);
            std::cout << mode << ": frozen previous core captured\n";
#else
            quad3d_test::write(std::string(argv[2]) + suffix, actual);
            const auto current = quad3d_test::parse(actual);
            quad3d_test::compare(
                quad3d_test::parse(quad3d_test::read(std::string(argv[1]) + suffix)), current,
                mode);
            std::cout << mode << ": same-toolchain previous/current core MATCH\n";
            const auto historical =
                quad3d_test::parse(quad3d_test::read(std::string(argv[3]) + suffix));
            try {
                quad3d_test::compare(historical, current, mode);
                std::cout << mode << ": historical Windows trace also matches\n";
            } catch (const std::runtime_error &difference) {
                std::cout << "Historical Windows trace differs (diagnostic only): "
                          << difference.what() << '\n';
            }
#endif
        }
    });
}
