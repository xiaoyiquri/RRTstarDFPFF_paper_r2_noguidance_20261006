#pragma once

#include <array>
#include <cstddef>

#include "di_primitive_types.h"

namespace ompl::control::detail
{
    template <std::size_t W>
    struct DICanonicalStateBlock
    {
        std::array<double, W> x{};
        std::array<double, W> y{};
        std::array<double, W> vx{};
        std::array<double, W> vy{};
    };

    template <std::size_t W>
    struct EdgeEvalBlock
    {
        std::size_t count{0};
        std::array<bool, W> ok{};
        std::array<double, W> tOpt{};
        std::array<double, W> incCost{};
        // These values can become tree-node state/cache data, so they remain double.
        DICanonicalStateBlock<W> terminal{};
        std::array<DIEdgePrimitive, W> primitive{};
    };
}  // namespace ompl::control::detail
