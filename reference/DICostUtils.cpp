#include "DICostUtils.h"

#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <array>
#include <algorithm>
#include <cmath>
#include <complex>
#include <limits>
#include <vector>

namespace ob = ompl::base;

namespace
{
    using QuarticCoefficients = std::array<double, 5>;

    struct QuarticRoots
    {
        std::array<double, 4> values{};
        std::size_t count{0};
    };

    std::array<std::complex<double>, 4> solveMonicQuarticDurandKerner(
        double b3, double b2, double b1, double b0)
    {
        using Complex = std::complex<double>;

        const double radius = 1.0 + std::max(
            std::max(std::abs(b3), std::abs(b2)),
            std::max(std::abs(b1), std::abs(b0)));

        std::array<Complex, 4> roots{
            Complex(radius, 0.0),
            Complex(0.0, radius),
            Complex(-radius, 0.0),
            Complex(0.0, -radius)};

        auto eval = [=](Complex z)
        {
            return (((z + b3) * z + b2) * z + b1) * z + b0;
        };

        for (int iter = 0; iter < 80; ++iter)
        {
            double maxDelta = 0.0;
            for (std::size_t i = 0; i < roots.size(); ++i)
            {
                Complex denom(1.0, 0.0);
                for (std::size_t j = 0; j < roots.size(); ++j)
                {
                    if (i != j)
                        denom *= roots[i] - roots[j];
                }

                if (std::abs(denom) < 1e-18)
                    denom = Complex(1e-18, 0.0);

                const Complex delta = eval(roots[i]) / denom;
                roots[i] -= delta;
                maxDelta = std::max(maxDelta, std::abs(delta));
            }

            if (maxDelta < 1e-12)
                break;
        }

        return roots;
    }

    QuarticCoefficients makeTimeFreeVelCoefficients(
        double x01, double x02, double x03, double x04,
        double x11, double x12)
    {
        return QuarticCoefficients{
            1.0,
            0.0,
            -3.0 * x03 * x03 - 3.0 * x04 * x04,
            12.0 * x03 * x11 - 12.0 * x02 * x04 - 12.0 * x01 * x03 + 12.0 * x04 * x12,
            -9.0 * x01 * x01 + 18.0 * x01 * x11 - 9.0 * x02 * x02 + 18.0 * x02 * x12
                - 9.0 * x11 * x11 - 9.0 * x12 * x12};
    }

    QuarticRoots solveQuarticFixed(const QuarticCoefficients &coeffs)
    {
        QuarticRoots roots;

        const double a4 = coeffs[0];
        if (std::abs(a4) < 1e-12)
            return roots;

        const double b3 = coeffs[1] / a4;
        const double b2 = coeffs[2] / a4;
        const double b1 = coeffs[3] / a4;
        const double b0 = coeffs[4] / a4;

        const auto evals = solveMonicQuarticDurandKerner(b3, b2, b1, b0);
        const double imagTol = 1e-8;

        for (std::size_t i = 0; i < evals.size(); ++i)
        {
            const std::complex<double> z = evals[i];
            if (std::abs(z.imag()) <= imagTol)
                roots.values[roots.count++] = z.real();
        }

        return roots;
    }

    inline void extractStateComponents(
        const ob::State *s,
        double &x, double &y,
        double &vx, double &vy)
    {
        const auto *sv = s->as<ob::RealVectorStateSpace::StateType>();
        x  = sv->values[0];
        y  = sv->values[1];
        vx = sv->values[2];
        vy = sv->values[3];
    }
}

double di_cost::costFreeVel(
    double t_s,
    double x01, double x02, double x03, double x04,
    double x11, double x12)
{
    const double cost =
        t_s
        + (3.0 * x03 * x03 + 3.0 * x04 * x04) / t_s
        + (3.0 * x01 * x01 - 6.0 * x01 * x11 + 3.0 * x02 * x02 - 6.0 * x02 * x12
           + 3.0 * x11 * x11 + 3.0 * x12 * x12) / (t_s * t_s * t_s)
        + (6.0 * x01 * x03 + 6.0 * x02 * x04 - 6.0 * x03 * x11 - 6.0 * x04 * x12) / (t_s * t_s);

    return cost;
}

std::vector<double> di_cost::timeFreeVel(
    double x01, double x02, double x03, double x04,
    double x11, double x12)
{
    const auto coeffs = makeTimeFreeVelCoefficients(x01, x02, x03, x04, x11, x12);
    return std::vector<double>(coeffs.begin(), coeffs.end());
}

std::vector<double> di_cost::solveQuartic(const std::vector<double> &coeffs)
{
    if (coeffs.size() != 5)
        return {};

    const QuarticCoefficients fixedCoeffs{
        coeffs[0], coeffs[1], coeffs[2], coeffs[3], coeffs[4]};
    const QuarticRoots fixedRoots = solveQuarticFixed(fixedCoeffs);

    std::vector<double> roots;
    roots.reserve(fixedRoots.count);

    for (std::size_t i = 0; i < fixedRoots.count; ++i)
        roots.push_back(fixedRoots.values[i]);

    return roots;
}

bool di_cost::computeOptimalTimeFreeVel(
    const ob::State *s1,
    const ob::State *s2,
    double minTrajectoryTime,
    double maxTrajectoryTime,
    double &t_opt)
{
    double x01, x02, x03, x04;
    extractStateComponents(s1, x01, x02, x03, x04);

    const auto *s2v = s2->as<ob::RealVectorStateSpace::StateType>();
    const double x11 = s2v->values[0];
    const double x12 = s2v->values[1];

    const QuarticCoefficients coeffs =
        makeTimeFreeVelCoefficients(x01, x02, x03, x04, x11, x12);
    const QuarticRoots roots = solveQuarticFixed(coeffs);

    double bestCost = std::numeric_limits<double>::infinity();
    double bestTime = -1.0;

    auto considerCandidate = [&](double t)
    {
        if (!std::isfinite(t))
            return;

        const double c = costFreeVel(t, x01, x02, x03, x04, x11, x12);
        if (c < bestCost)
        {
            bestCost = c;
            bestTime = t;
        }
    };

    for (std::size_t i = 0; i < roots.count; ++i)
    {
        const double r = roots.values[i];
        if (std::isfinite(r) && r > 1e-9 &&
            r >= minTrajectoryTime && r <= maxTrajectoryTime)
            considerCandidate(r);
    }

    considerCandidate(minTrajectoryTime);
    considerCandidate(maxTrajectoryTime);

    if (!std::isfinite(bestTime))
        return false;

    t_opt = bestTime;
    return true;
}

ob::Cost di_cost::motionCost(
    const ob::State *s1,
    const ob::State *s2,
    double minTrajectoryTime,
    double maxTrajectoryTime)
{
    double t_opt = 0.0;
    if (!computeOptimalTimeFreeVel(s1, s2, minTrajectoryTime, maxTrajectoryTime, t_opt))
        return ob::Cost(std::numeric_limits<double>::infinity());

    double x01, x02, x03, x04;
    extractStateComponents(s1, x01, x02, x03, x04);

    const auto *s2v = s2->as<ob::RealVectorStateSpace::StateType>();
    const double x11 = s2v->values[0];
    const double x12 = s2v->values[1];

    const double c = costFreeVel(t_opt, x01, x02, x03, x04, x11, x12);
    return ob::Cost(c);
}
