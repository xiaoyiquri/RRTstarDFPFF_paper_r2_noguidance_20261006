#pragma once
#include <cmath>
#include <cstddef>
#include <stdexcept>
namespace rrtpff {
struct PlannerConfig {
    double timeout{10}, goal_bias{.1}, range{4}, rewire_factor{3};
    double goal_tolerance{.05};
    std::size_t max_iterations{10000}, max_nodes{4001}, top_k{32};
    unsigned seed{40};
    bool simd_enabled{false};
    bool simd_pff{true}, simd_nn{true}, simd_screen{true};
    bool interleaved_samples{true};
    std::size_t collision_samples{33};
    bool prune{false}, stop_at_first_solution{false};
    bool rewire_before_first_solution{true};
    bool explicit_goal_connection{false};
    std::size_t goal_parent_limit{16};
    void validate() const {
        if (!std::isfinite(timeout) || timeout <= 0 || !std::isfinite(range) || range <= 0 ||
            !std::isfinite(goal_bias) || goal_bias < 0 || goal_bias > 1 ||
            !std::isfinite(rewire_factor) || rewire_factor <= 0 || !std::isfinite(goal_tolerance) ||
            goal_tolerance < 0 || !max_iterations || max_nodes < 2 || !top_k ||
            !goal_parent_limit || collision_samples < 2 || collision_samples > 1000000)
            throw std::invalid_argument("Invalid planner configuration");
    }
};
} // namespace rrtpff
