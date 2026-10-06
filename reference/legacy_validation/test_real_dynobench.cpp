#include "systems/robot_factory.h"
#include "tests/test_support.h"
#include <filesystem>
#include <iostream>

using namespace rrtpff;

template <class Model> void checkModel(const Model &model) {
    require(model.initialStateValid(), "real start state and FCL collision");
    require(model.goalOutputValid(), "real goal state and FCL collision");
    const auto goal_constraint = model.expansionConstraint(model.goalOutput());
    for (bool fixed : goal_constraint.fixed)
        require(fixed, "physical goal fixes every flat derivative");
    auto nearby = model.start().output();
    nearby(0) += .01;
    const auto expansion = model.expansionConstraint(nearby);
    for (int i = 0; i < Model::FlatDim * Model::FlatOrder; ++i)
        require(expansion.fixed[i] == (i < Model::FlatDim),
                "ordinary expansion leaves derivatives free");

    using Solver = PFFSolver<Model::FlatDim, Model::FlatOrder, typename Model::Metadata>;
    using Constraint = typename Solver::Constraint;
    auto end = model.start();
    auto position = end.output();
    double duration = 1;
    if constexpr (Model::FlatOrder == 2) {
        duration = .2;
        position += duration * end.derivative(1);
    } else {
        position(0) += .01;
    }
    end.setDerivative(0, position);
    auto solved = Solver{}.fixedTime(model.start(), Constraint::full(end), duration);
    require(solved.success, "real robot fixed-time optimal local trajectory");
    require(model.prepareEdge(model.start(), model.initialMetadata(), solved.trajectory),
            "edge metadata");
    PlanResult stats;
    require(model.trajectoryValid(solved.trajectory, stats),
            "PFF -> DF -> real step/FCL validation");
    require(stats.dyno_validation_count > 0 && stats.collision_check_count > 0,
            "real backend reached");
    if constexpr (Model::FlatDim == 2) {
        PlanResult early;
        require(model.precheckTrajectory(solved.trajectory, early),
                "real sparse physical precheck");
        require(early.collision_check_count > 0 && early.dense_reconstructions == 0,
                "sparse FCL checks run before dense reconstruction");
        require(model.trajectoryValidAfterPrecheck(solved.trajectory, early),
                "sparse pass still receives full real step/FCL validation");
    }
    DynoSamples samples;
    require(model.reconstruct(solved.trajectory, samples, stats), "physical reconstruction");
    auto report = model.adapter().checkTrajectory(samples, samples.states.front(),
                                                  samples.states.back(), model.tolerances(), true);
    require(report.success, "upstream DynoBench Trajectory::check with corrected residual");
    // This must fail even if the installed upstream version has a verbose=false
    // max_jump bug. Keep expected endpoints unchanged and perturb an interior point.
    samples.states[samples.states.size() / 2](0) += .02;
    report = model.adapter().checkTrajectory(samples, samples.states.front(), samples.states.back(),
                                             model.tolerances(), false);
    require(!report.success && report.reason == "dynamics residual", "corrupted dynamics rejected");
}

int main(int argc, char **argv) {
    return runTest([&] {
        require(argc == 3, "usage: test_real_dynobench DYNO_SOURCE PLANNER_SOURCE");
        const std::filesystem::path source = argv[1], project = argv[2];
        for (const char *name : {"uni_bugtrap", "uni_wall", "quad2d_bugtrap", "quad2d_hole",
                                 "quad3d_block", "quad3d_window"}) {
            const auto yaml = YAML::LoadFile(
                (project / "benchmark/config" / (std::string(name) + ".yaml")).string());
            const auto env =
                source / "envs" / yaml["dynobench"]["environment_file"].as<std::string>();
            const auto models = project / "benchmark/models/paper";
            RealDynoAdapter adapter(env.string(), models.string());
            const auto &data = adapter.problem();
            if (data.robot_type == "quad3d_v0") {
                require(!data.motor_control, "paper Quad3D wrench mode");
                close(data.mass, 1, 1e-12, "paper Quad3D mass");
                require((data.inertia_3d - Eigen::Vector3d(.1, .1, .2)).norm() < 1e-12,
                        "paper Quad3D inertia");
                const Eigen::Vector4d upper = data.control_to_wrench * data.upper_control;
                require((upper - Eigen::Vector4d(1.5 * data.mass * data.gravity, 2, 2, 2)).norm() <
                            1e-10,
                        "actual DynoBench normalization produces paper wrench bounds");
                require(
                    (data.control_to_wrench * data.wrench_to_control - Eigen::Matrix4d::Identity())
                            .norm() < 1e-12,
                    "actual allocation inverse");
                Eigen::VectorXd state = data.start, next;
                state.segment<4>(3) << 0, 0, 0, 1;
                state.tail<6>().setZero();
                Eigen::Vector4d u(1, .05, 0, 0);
                require(adapter.step(state, u, .0001, next), "actual wrench-mode step");
                close((next(10) - state(10)) / .0001, 10, 1e-6,
                      "1 N m / 0.1 kg m2 produces 10 rad/s2");
            } else if (data.robot_type.find("unicycle1_") == 0) {
                close(data.upper_control(0), 1, 1e-12, "paper Unicycle speed");
                close(data.upper_control(1), 1.5, 1e-12, "paper Unicycle angular speed");
            } else {
                close(data.mass, .034, 1e-12, "paper Quad2D mass");
                close(data.inertia_2d, 1e-4, 1e-12, "paper Quad2D inertia");
                close(data.arm_length, .1, 1e-12, "paper Quad2D arm");
                close(data.upper_control(0), 1.3, 1e-12, "paper normalized motor limit");
            }
            RobotValidationConfig validation;
            validation.max_step = .001;
            validation.collision_screen = true;
            const auto system = yaml["system"].as<std::string>();
            if (system != "quad3d")
                validation.collision_precheck_samples = 9;
            if (system == "unicycle")
                checkModel(makeUnicycle(adapter, yaml, validation));
            else if (system == "quad2d")
                checkModel(makeQuad2D(adapter, yaml, validation));
            else
                checkModel(makeQuad3D(adapter, yaml, validation));

            const auto environment = YAML::LoadFile(env.string());
            const auto center = environment["environment"]["obstacles"][0]["center"];
            Eigen::VectorXd colliding = adapter.problem().start;
            for (std::size_t i = 0; i < center.size(); ++i)
                colliding(i) = center[i].as<double>();
            require(adapter.stateValid(colliding) && !adapter.collisionFree(colliding),
                    "actual FCL obstacle rejection");
            Eigen::VectorXd invalid = adapter.problem().upper_control;
            invalid(0) += 1;
            require(!adapter.controlValid(invalid), "actual actuator bound rejection");
            require(data.collision_inner_radius > 0 && !data.collision_boxes.empty(),
                    "actual centered geometry available for conservative rejection");
            // Verify rejection-only geometry against actual FCL at deterministic poses.
            for (int trial = 0; trial < 160; ++trial) {
                Eigen::VectorXd pose = data.start;
                const int dim = static_cast<int>(data.lower_position.size());
                for (int a = 0; a < dim; ++a) {
                    const double fraction = std::fmod((trial + 1) * (0.61803398875 + .13 * a), 1.0);
                    pose(a) = data.lower_position(a) +
                              fraction * (data.upper_position(a) - data.lower_position(a));
                }
                bool hit = false;
                for (const auto &box : data.collision_boxes) {
                    unsigned char mask = 0;
                    sphereAABBBatch(pose.data(), box.lower.data(), box.upper.data(), dim,
                                    data.collision_inner_radius - 1e-9, 1, &mask,
                                    SIMDBackend::Scalar);
                    hit = hit || mask;
                }
                if (hit)
                    require(!adapter.collisionFree(pose),
                            "SIMD inner-sphere rejection implies actual FCL collision");
            }
            std::cout << name << ": real local dynamics, collision, endpoint contracts PASS\n";
        }
    });
}
