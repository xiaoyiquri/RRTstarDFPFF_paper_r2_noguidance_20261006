#include "dynobench/real_adapter.h"
#include <Eigen/LU>
#include <algorithm>
#include <cmath>
#include <dynobench/motions.hpp>
#include <dynobench/planar_rotor.hpp>
#include <dynobench/quadrotor.hpp>
#include <dynobench/robot_models.hpp>
#include <dynobench/unicycle1.hpp>
#include <fcl/geometry/shape/box.h>
#include <fcl/geometry/shape/sphere.h>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <stdexcept>

namespace rrtpff {
struct RealDynoAdapter::Impl {
    dynobench::Problem environment;
    std::shared_ptr<dynobench::Model_robot> robot;
    DynoProblemData data;
};

RealDynoAdapter::RealDynoAdapter(const std::string &environment_file,
                                 const std::string &models_directory)
    : impl_(std::make_shared<Impl>()) {
    auto &p = impl_->environment;
    p.read_from_yaml(environment_file.c_str());
    if (p.robotTypes.size() != 1)
        throw std::invalid_argument("Exactly one DynoBench robot is required");
    const auto model_file = std::filesystem::path(models_directory) / (p.robotType + ".yaml");
    impl_->robot = dynobench::robot_factory(model_file.string().c_str(), p.p_lb, p.p_ub);
    dynobench::load_env(*impl_->robot, p);
    auto &d = impl_->data;
    auto &robot = *impl_->robot;
    d.robot_type = p.robotType;
    d.start = p.start;
    d.goal = p.goal;
    d.lower_position = p.p_lb;
    d.upper_position = p.p_ub;
    d.lower_state = robot.get_x_lb();
    d.upper_state = robot.get_x_ub();
    d.lower_control = robot.get_u_lb();
    d.upper_control = robot.get_u_ub();
    d.reference_dt = robot.ref_dt;
    if (!std::isfinite(d.reference_dt) || d.reference_dt <= 0)
        throw std::invalid_argument("Invalid DynoBench reference dt");
    if (dynamic_cast<dynobench::Model_unicycle1 *>(&robot)) {
        d.max_velocity = std::max(std::abs(d.lower_control(0)), std::abs(d.upper_control(0)));
        d.max_omega = std::max(std::abs(d.lower_control(1)), std::abs(d.upper_control(1)));
    } else if (auto *quad = dynamic_cast<dynobench::Model_quad2d *>(&robot)) {
        d.mass = quad->params.m;
        d.gravity = quad->g;
        d.inertia_2d = quad->params.I;
        d.arm_length = quad->params.l;
        d.max_motor = quad->params.max_f;
        d.max_velocity = quad->params.max_vel;
        d.max_omega = quad->params.max_angular_vel;
        d.drag = quad->params.drag_against_vel;
    } else if (auto *quad = dynamic_cast<dynobench::Model_quad3d *>(&robot)) {
        d.mass = quad->params.m;
        d.gravity = quad->g;
        d.inertia_3d = quad->params.J_v;
        d.arm_length = quad->params.arm_length;
        d.thrust_to_torque = quad->params.t2t;
        d.max_motor = quad->params.max_f;
        d.motor_control = quad->params.motor_control;
        d.max_velocity = quad->params.max_vel;
        d.max_omega = quad->params.max_angular_vel;
        d.control_to_wrench = quad->B0;
        const auto factor = d.control_to_wrench.fullPivLu();
        if (!factor.isInvertible())
            throw std::invalid_argument("Singular DynoBench Quad3D actuation matrix");
        d.wrench_to_control = factor.inverse();
    } else {
        throw std::invalid_argument("Unsupported real DynoBench model: " + p.robotType);
    }
    if (d.drag)
        throw std::invalid_argument("DF bridge requires no drag");
    // Only known, centered single-body geometries are used for rejection.
    // Unknown geometry leaves the screen disabled; FCL still validates it.
    if (robot.collision_geometries.size() == 1) {
        const auto *geometry = robot.collision_geometries.front().get();
        if (const auto *sphere = dynamic_cast<const fcl::Sphered *>(geometry))
            d.collision_inner_radius = sphere->radius;
        else if (const auto *box = dynamic_cast<const fcl::Boxd *>(geometry))
            d.collision_inner_radius = 0.5 * box->side.minCoeff();
    }
    for (const auto &obstacle : p.obstacles) {
        if (obstacle.type != "box" || obstacle.center.size() != d.lower_position.size() ||
            obstacle.size.size() != obstacle.center.size())
            continue;
        CollisionAABB box;
        for (Eigen::Index a = 0; a < obstacle.center.size(); ++a) {
            box.lower[a] = obstacle.center(a) - 0.5 * obstacle.size(a);
            box.upper[a] = obstacle.center(a) + 0.5 * obstacle.size(a);
        }
        d.collision_boxes.push_back(box);
    }
}

const DynoProblemData &RealDynoAdapter::problem() const { return impl_->data; }
bool RealDynoAdapter::stateValid(const Eigen::VectorXd &x) const {
    return x.size() == impl_->robot->nx && x.allFinite() && impl_->robot->is_state_valid(x);
}
bool RealDynoAdapter::controlValid(const Eigen::VectorXd &u) const {
    return u.size() == impl_->robot->nu && u.allFinite() && impl_->robot->is_control_valid(u);
}
bool RealDynoAdapter::collisionFree(const Eigen::VectorXd &x) const {
    return stateValid(x) && impl_->robot->collision_check(x);
}
double RealDynoAdapter::distance(const Eigen::VectorXd &a, const Eigen::VectorXd &b) const {
    if (a.size() != impl_->robot->nx || b.size() != impl_->robot->nx || !a.allFinite() ||
        !b.allFinite())
        return std::numeric_limits<double>::infinity();
    return impl_->robot->distance(a, b);
}
bool RealDynoAdapter::step(const Eigen::VectorXd &x, const Eigen::VectorXd &u, double dt,
                           Eigen::VectorXd &next) const {
    if (!stateValid(x) || !controlValid(u) || !std::isfinite(dt) || dt <= 0)
        return false;
    next.resize(impl_->robot->nx);
    impl_->robot->step(next, x, u, dt);
    return next.allFinite();
}

DynoValidation RealDynoAdapter::checkTrajectory(const DynoSamples &s,
                                                const Eigen::VectorXd &expected_start,
                                                const Eigen::VectorXd &expected_goal,
                                                DynoTolerances tolerance,
                                                bool upstream_check) const {
    DynoValidation result;
    auto fail = [&](const char *reason) {
        result.reason = reason;
        return result;
    };
    if (s.states.size() < 2 || s.controls.size() + 1 != s.states.size() ||
        s.times.size() != s.states.size())
        return fail("sample dimensions");
    if (!std::isfinite(tolerance.dynamics) || tolerance.dynamics <= 0 ||
        !std::isfinite(tolerance.endpoint) || tolerance.endpoint <= 0)
        return fail("invalid tolerances");
    if (!std::isfinite(s.times.front()) || s.times.front() != 0)
        return fail("initial time");
    for (std::size_t i = 0; i < s.states.size(); ++i) {
        if (!stateValid(s.states[i]))
            return fail("state bounds");
        ++result.collision_checks;
        if (!collisionFree(s.states[i]))
            return fail("FCL collision");
        if (i == s.controls.size())
            break;
        if (!controlValid(s.controls[i]))
            return fail("control bounds");
        const double dt = s.times[i + 1] - s.times[i];
        Eigen::VectorXd next;
        if (!step(s.states[i], s.controls[i], dt, next))
            return fail("DynoBench step");
        const double jump = distance(next, s.states[i + 1]);
        if (!std::isfinite(jump))
            return fail("nonfinite dynamics residual");
        result.max_jump = std::max(result.max_jump, jump);
        if (jump >= tolerance.dynamics)
            return fail("dynamics residual");
    }
    result.start_error = distance(s.states.front(), expected_start);
    result.goal_error = distance(s.states.back(), expected_goal);
    if (!(result.start_error < tolerance.endpoint && result.goal_error < tolerance.endpoint))
        return fail("physical endpoint");
    if (upstream_check) {
        dynobench::Trajectory trajectory;
        trajectory.states = s.states;
        trajectory.actions = s.controls;
        trajectory.times = Eigen::Map<const Eigen::VectorXd>(s.times.data(), s.times.size());
        trajectory.start = expected_start;
        trajectory.goal = expected_goal;
        trajectory.check(impl_->robot, false);
        // Some upstream versions only accumulate max_jump when verbose=true.
        // Always supply the independent residual from real step()/distance().
        trajectory.max_jump = std::max(trajectory.max_jump, result.max_jump);
        dynobench::Feasibility_thresholds thresholds;
        thresholds.traj_tol = tolerance.dynamics;
        thresholds.goal_tol = tolerance.endpoint;
        thresholds.col_tol = 1e-9;
        thresholds.x_bound_tol = 1e-9;
        thresholds.u_bound_tol = 1e-9;
        trajectory.update_feasibility(thresholds, false);
        if (!trajectory.feasible)
            return fail("DynoBench Trajectory::check");
    }
    result.success = true;
    return result;
}

void RealDynoAdapter::writeTrajectory(const DynoSamples &s, const std::string &file, double cost,
                                      DynoTolerances tolerance) const {
    const auto result = checkTrajectory(s, problem().start, problem().goal, tolerance, true);
    if (!result.success)
        throw std::invalid_argument("Refusing to export unvalidated trajectory: " + result.reason);
    // Upstream to_yaml_format uses an Eigen IOFormat with six significant
    // digits. Preserve the validated samples and dt with round-trip precision.
    std::ofstream out(file);
    if (!out)
        throw std::runtime_error("Cannot create trajectory YAML: " + file);
    out << std::setprecision(std::numeric_limits<double>::max_digits10);
    auto vector = [&](const Eigen::VectorXd &v) {
        out << '[';
        for (Eigen::Index i = 0; i < v.size(); ++i) {
            if (i)
                out << ", ";
            out << v(i);
        }
        out << "]\n";
    };
    out << "cost: " << cost << "\nfeasible: true\nmax_jump: " << result.max_jump
        << "\nstart_distance: " << result.start_error << "\ngoal_distance: " << result.goal_error
        << "\nstart: ";
    vector(problem().start);
    out << "goal: ";
    vector(problem().goal);
    out << "states:\n";
    for (const auto &x : s.states) {
        out << "  - ";
        vector(x);
    }
    out << "actions:\n";
    for (const auto &u : s.controls) {
        out << "  - ";
        vector(u);
    }
    out << "times:\n";
    for (double t : s.times)
        out << "  - " << t << '\n';
    out.flush();
    if (!out)
        throw std::runtime_error("Failed to write trajectory YAML: " + file);
}
} // namespace rrtpff
