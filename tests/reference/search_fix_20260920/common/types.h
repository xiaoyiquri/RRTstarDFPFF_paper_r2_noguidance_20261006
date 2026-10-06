#pragma once
#include <array>
#include <cstddef>
#include <limits>
#include <string>

namespace rrtpff {
using Scalar = double;
enum class SystemType { Unicycle, Quad2D, Quad3D };
enum class EnvironmentType { Bugtrap, Wall, Hole, Block, Window };
enum class FeasibilityFailure : std::size_t {
    None,
    Numeric,
    Branch,
    SpeedZero,
    SpeedMin,
    SpeedMax,
    Omega,
    Acceleration,
    Jerk,
    Thrust,
    Motor,
    AngularAcceleration,
    Singularity,
    SampleLimit,
    Count
};
inline constexpr std::array<const char *, static_cast<std::size_t>(FeasibilityFailure::Count)>
    failureNames{"none",        "numeric",     "branch", "speed_zero",
                 "speed_min",   "speed_max",   "omega",  "acceleration",
                 "jerk",        "thrust",      "motor",  "angular_acceleration",
                 "singularity", "sample_limit"};
inline bool failFeasibility(FeasibilityFailure *reason, FeasibilityFailure value) {
    if (reason)
        *reason = value;
    return false;
}
struct PlanResult {
    bool success{false};
    double cost{std::numeric_limits<double>::infinity()};
    double duration{0};
    // trajectory duration, not planning wall time
    double first_solution_time{std::numeric_limits<double>::infinity()};
    std::size_t node_count{0}, pff_eval_count{0};
    std::size_t dyno_validation_count{0}, collision_check_count{0};
    std::size_t simd_eval_count{0}, rewire_count{0}, iterations{0};
    std::size_t rejected_candidates{0}, pff_rejected{0}, df_rejected{0}, flat_bound_rejected{0},
        state_bound_rejected{0}, control_bound_rejected{0}, collision_rejected{0};
    std::size_t simd_pff_lanes{0}, simd_flat_samples{0}, simd_nn_lanes{0};
    std::size_t screen_rejected{0}, pff_query_count{0};
    bool simd_active{false};
    double nn_time{0}, screen_time{0}, validation_time{0};
    double planning_time{0}, path_length{0};
    double df_time{0}, pff_time{0}, simd_time{0}, dyno_validation_time{0};
    std::string failure_reason;
    std::array<std::size_t, static_cast<std::size_t>(FeasibilityFailure::Count)> df_failures{};
    std::size_t dynamics_rejected{0}, goal_connection_queries{0}, goal_connection_valid{0};
    double closest_goal_distance{std::numeric_limits<double>::infinity()};
};
} // namespace rrtpff
