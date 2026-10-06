#pragma once
#include "common/collision_geometry.h"
#include <Eigen/Core>
#include <memory>
#include <string>
#include <vector>

namespace rrtpff {
// This boundary deliberately contains no DynoBench or FCL headers. Physical
// vectors are dynamic here because the external library uses VectorXd; models
// and the planner retain their own compile-time dimensions.
struct DynoProblemData {
    std::string robot_type;
    Eigen::VectorXd start, goal, lower_position, upper_position;
    Eigen::VectorXd lower_state, upper_state, lower_control, upper_control;
    double reference_dt{0};
    // Read from the actual instantiated model, not guessed from file names.
    double mass{0}, gravity{0}, inertia_2d{0}, arm_length{0}, thrust_to_torque{0};
    Eigen::Vector3d inertia_3d = Eigen::Vector3d::Zero();
    double max_velocity{0}, max_omega{0}, max_motor{0};
    bool drag{false}, motor_control{true};
    Eigen::Matrix4d control_to_wrench = Eigen::Matrix4d::Zero();
    Eigen::Matrix4d wrench_to_control = Eigen::Matrix4d::Zero();
    double collision_inner_radius{0};
    std::vector<CollisionAABB> collision_boxes;
};

struct DynoSamples {
    std::vector<Eigen::VectorXd> states;
    std::vector<Eigen::VectorXd> controls;
    std::vector<double> times;
};

struct DynoValidation {
    bool success{false};
    double max_jump{0}, start_error{0}, goal_error{0};
    std::size_t collision_checks{0};
    std::string reason;
};

struct DynoTolerances {
    double dynamics{1e-3};
    double endpoint{1e-3};
};

class RealDynoAdapter {
  public:
    RealDynoAdapter(const std::string &environment_file, const std::string &models_directory);
    const DynoProblemData &problem() const;
    bool stateValid(const Eigen::VectorXd &state) const;
    bool controlValid(const Eigen::VectorXd &control) const;
    bool collisionFree(const Eigen::VectorXd &state) const;
    double distance(const Eigen::VectorXd &a, const Eigen::VectorXd &b) const;
    bool step(const Eigen::VectorXd &state, const Eigen::VectorXd &control, double dt,
              Eigen::VectorXd &next) const;
    DynoValidation checkTrajectory(const DynoSamples &samples,
                                   const Eigen::VectorXd &expected_start,
                                   const Eigen::VectorXd &expected_goal, DynoTolerances tolerance,
                                   bool upstream_check = true) const;
    void writeTrajectory(const DynoSamples &samples, const std::string &filename,
                         double planning_cost, DynoTolerances tolerance) const;

  private:
    struct Impl;
    std::shared_ptr<Impl> impl_;
};
} // namespace rrtpff
