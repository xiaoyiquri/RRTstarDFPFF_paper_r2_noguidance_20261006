#pragma once

#include <array>
#include <cstddef>
#include <cstdio>
#include <limits>

#include <ompl/base/Cost.h>

namespace ompl::control::detail
{
    enum class EdgeMode : unsigned char
    {
        FreeVel,
        FixedVel
    };

    struct DIState
    {
        double x{0.0};
        double y{0.0};
        double vx{0.0};
        double vy{0.0};
    };

    template <std::size_t W>
    struct DIStateBlock
    {
        // Temporary float SoA. For W=8, one AVX2 register holds x(t_i..t_{i+7})
        // (and likewise y, vx, and vy) for one trajectory or candidate block.
        std::array<float, W> x{};
        std::array<float, W> y{};
        std::array<float, W> vx{};
        std::array<float, W> vy{};
    };

    struct DIEdgePrimitive
    {
        EdgeMode mode{EdgeMode::FreeVel};
        double x0{0.0};
        double y0{0.0};
        double vx0{0.0};
        double vy0{0.0};
        double tOpt{0.0};
        // x(t) = x0 + vx0*t + x2*t^2 + x3*t^3 (same for y).
        // Both free-terminal-velocity and fixed-terminal-velocity edges use
        // this canonical representation.
        double x2{0.0};
        double y2{0.0};
        double x3{0.0};
        double y3{0.0};
    };

    struct EdgeEval
    {
        bool ok{false};
        double t_opt{0.0};
        ompl::base::Cost incCost{
            std::numeric_limits<double>::infinity()};
        DIState terminal{};
        DIEdgePrimitive primitive{};
    };
}  // namespace ompl::control::detail
