#pragma once

#include <algorithm>
#include <array>
#include <cmath>

#include "di_simd_backend.h"

auto ompl::control::RRTstarDI::evaluateEdgeState(
    const DIEdgePrimitive &primitive,
    double t) const -> DIState
{
    DIState state;
    evaluateEdgeState(primitive, t, state.x, state.y, state.vx, state.vy);
    return state;
}

auto ompl::control::RRTstarDI::evaluateEdgeStateByMode(
    const DIEdgePrimitive &primitive,
    double t) const -> DIState
{
    // The primitive is canonical for both models; mode is kept explicit so
    // validation cannot silently use the wrong terminal-constraint model.
    switch (primitive.mode)
    {
    case EdgeMode::FreeVel:
    case EdgeMode::FixedVel:
        return evaluateEdgeState(primitive, t);
    }
    return DIState{};
}

void ompl::control::RRTstarDI::evaluateEdgeStateBlock(
    const DIEdgePrimitive &primitive,
    const std::array<double, PrimitiveBatchWidth> &times,
    unsigned int count,
    PrimitiveStateBlock &block) const
{
    if (count == PrimitiveBatchWidth)
    {
        detail::evaluateEdgeStateBlock8Simd(primitive, times, block);
        return;
    }

    for (unsigned int i = 0; i < count; ++i)
    {
        const DIState state = evaluateEdgeState(primitive, times[i]);
        block.x[i] = static_cast<float>(state.x);
        block.y[i] = static_cast<float>(state.y);
        block.vx[i] = static_cast<float>(state.vx);
        block.vy[i] = static_cast<float>(state.vy);
    }
}

void ompl::control::RRTstarDI::evaluateEdgeStateBlockByMode(
    const DIEdgePrimitive &primitive,
    const std::array<double, PrimitiveBatchWidth> &times,
    unsigned int count,
    PrimitiveStateBlock &block) const
{
    switch (primitive.mode)
    {
    case EdgeMode::FreeVel:
    case EdgeMode::FixedVel:
        evaluateEdgeStateBlock(primitive, times, count, block);
        return;
    }
}

auto ompl::control::RRTstarDI::toDIState(const base::State *state) const -> DIState
{
    DIState diState;
    extractStateComponents(state, diState.x, diState.y, diState.vx, diState.vy);
    return diState;
}

void ompl::control::RRTstarDI::fromDIState(base::State *state, const DIState &diState) const
{
    setStateComponents(state, diState.x, diState.y, diState.vx, diState.vy);
}

auto ompl::control::RRTstarDI::isFiniteState(const DIState &state) const -> bool
{
    return std::isfinite(state.x) && std::isfinite(state.y) &&
           std::isfinite(state.vx) && std::isfinite(state.vy);
}

auto ompl::control::RRTstarDI::checkPrimitiveState(
    const DIState &state,
    base::State *fallbackScratch) const -> bool
{
    if (!isFiniteState(state))
        return false;

    if (!fastEnvironment_.empty())
        return fastEnvironment_.checkState(state);

    if (fastValidityChecker_)
        return fastValidityChecker_(state.x, state.y, state.vx, state.vy);

    if (fastBatchValidityChecker_)
    {
        const double xs[1] = {state.x};
        const double ys[1] = {state.y};
        const double vxs[1] = {state.vx};
        const double vys[1] = {state.vy};
        return fastBatchValidityChecker_(xs, ys, vxs, vys, 1);
    }

    if (!fallbackScratch)
        return false;

    fromDIState(fallbackScratch, state);
    return si_->isValid(fallbackScratch);
}

auto ompl::control::RRTstarDI::checkPrimitiveStateBlock(
    const PrimitiveStateBlock &block,
    unsigned int count,
    base::State *fallbackScratch) const -> bool
{
    for (unsigned int i = 0; i < count; ++i)
    {
        if (!std::isfinite(block.x[i]) || !std::isfinite(block.y[i]) ||
            !std::isfinite(block.vx[i]) || !std::isfinite(block.vy[i]))
            return false;
    }

    if (!fastEnvironment_.empty())
        return fastEnvironment_.checkStateBlock(block, count);

    if (fastBatchValidityChecker_)
    {
        std::array<double, PrimitiveBatchWidth> xs{};
        std::array<double, PrimitiveBatchWidth> ys{};
        std::array<double, PrimitiveBatchWidth> vxs{};
        std::array<double, PrimitiveBatchWidth> vys{};
        for (unsigned int i = 0; i < count; ++i)
        {
            xs[i] = static_cast<double>(block.x[i]);
            ys[i] = static_cast<double>(block.y[i]);
            vxs[i] = static_cast<double>(block.vx[i]);
            vys[i] = static_cast<double>(block.vy[i]);
        }
        return fastBatchValidityChecker_(xs.data(), ys.data(), vxs.data(), vys.data(), count);
    }

    for (unsigned int i = 0; i < count; ++i)
    {
        if (!checkPrimitiveState(
                DIState{block.x[i], block.y[i], block.vx[i], block.vy[i]},
                fallbackScratch))
            return false;
    }

    return true;
}

auto ompl::control::RRTstarDI::validatePreparedMotionPrimitive(
    const DIEdgePrimitive &primitive,
    const DIState &start,
    const DIState &goal,
    base::State *fallbackScratch) const -> bool
{
    const double t_opt = primitive.tOpt;
    if (!(t_opt > 0.0) || !std::isfinite(t_opt))
        return false;

    if (t_opt < minTrajectoryTime_ || t_opt > maxTrajectoryTime_)
        return false;

    if (!checkPrimitiveState(start, fallbackScratch) || !checkPrimitiveState(goal, fallbackScratch))
        return false;

    if (numCollisionCheckPoints_ < 2)
        return true;

    // Rake layout: lanes contain consecutive samples from one trajectory.
    // For example, samples 1..16 are checked as [1..8], then [9..16].
    // This is deliberately different from assigning distant trajectory
    // intervals to the lanes and walking those intervals in lockstep.
    const unsigned int firstInteriorSample = 1u;
    const unsigned int lastInteriorSample = numCollisionCheckPoints_ - 1u;

    for (unsigned int firstSample = firstInteriorSample;
         firstSample <= lastInteriorSample;
         firstSample += static_cast<unsigned int>(RakeBatchWidth))
    {
        std::array<double, PrimitiveBatchWidth> times{};
        const unsigned int remaining = lastInteriorSample - firstSample + 1u;
        const unsigned int count = std::min<unsigned int>(
            static_cast<unsigned int>(RakeBatchWidth), remaining);

        for (unsigned int lane = 0; lane < count; ++lane)
        {
            const unsigned int sampleIndex = firstSample + lane;
            times[lane] = t_opt * static_cast<double>(sampleIndex) /
                          static_cast<double>(numCollisionCheckPoints_);
        }

        PrimitiveStateBlock block;
        evaluateEdgeStateBlockByMode(primitive, times, count, block);
        if (!checkPrimitiveStateBlock(block, count, fallbackScratch))
            return false;
    }

    return true;
}
