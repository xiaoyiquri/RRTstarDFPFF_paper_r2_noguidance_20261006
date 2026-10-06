#pragma once

#include <cstddef>
#include <cstdio>
#include <vector>

#include <ompl/base/Cost.h>
#include <ompl/base/State.h>

namespace di_cost
{
    // 与 RRTstarDI 里的 DI_costFreeVel 一致
    double costFreeVel(
        double t_s,
        double x01, double x02, double x03, double x04,
        double x11, double x12);

    // 与 RRTstarDI 里的 DI_timeFreeVel 一致
    std::vector<double> timeFreeVel(
        double x01, double x02, double x03, double x04,
        double x11, double x12);

    // 与 RRTstarDI 里的 solveQuartic 一致
    std::vector<double> solveQuartic(const std::vector<double> &coeffs);

    // 与 RRTstarDI 里的 computeOptimalTimeFreeVel 一致
    bool computeOptimalTimeFreeVel(
        const ompl::base::State *s1,
        const ompl::base::State *s2,
        double minTrajectoryTime,
        double maxTrajectoryTime,
        double &t_opt);

    // 统一的状态到状态边代价接口
    ompl::base::Cost motionCost(
        const ompl::base::State *s1,
        const ompl::base::State *s2,
        double minTrajectoryTime,
        double maxTrajectoryTime);
}
