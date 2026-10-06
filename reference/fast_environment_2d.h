#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#if defined(__AVX2__)
#include <immintrin.h>
#elif defined(__ARM_NEON) || defined(__ARM_NEON__)
#include <arm_neon.h>
#endif

#include "di_primitive_types.h"

namespace ompl::control::detail
{
    struct CirclePrimitive
    {
        double x{0.0};
        double y{0.0};
        double r2{0.0};
    };

    struct RectPrimitive
    {
        double xmin{0.0};
        double xmax{0.0};
        double ymin{0.0};
        double ymax{0.0};
    };

    class FastEnvironment2D
    {
    public:
        void clear()
        {
            hasPositionBounds_ = false;
            hasVelocityBounds_ = false;
            circles_.clear();
            rects_.clear();
        }

        void setPositionBounds(double xmin, double xmax, double ymin, double ymax)
        {
            hasPositionBounds_ = true;
            xmin_ = xmin;
            xmax_ = xmax;
            ymin_ = ymin;
            ymax_ = ymax;
        }

        void clearPositionBounds()
        {
            hasPositionBounds_ = false;
        }

        void setVelocityBounds(double vxmin, double vxmax, double vymin, double vymax)
        {
            hasVelocityBounds_ = true;
            vxmin_ = vxmin;
            vxmax_ = vxmax;
            vymin_ = vymin;
            vymax_ = vymax;
        }

        void clearVelocityBounds()
        {
            hasVelocityBounds_ = false;
        }

        void addCircle(double x, double y, double radius)
        {
            constexpr double kSafetyMargin = 0.1;
            const double safeRadius = radius + kSafetyMargin;
            circles_.push_back(CirclePrimitive{x, y, safeRadius * safeRadius});
        }

        void addRect(double xmin, double xmax, double ymin, double ymax)
        {
            constexpr double kSafetyMargin = 0.1;
            rects_.push_back(RectPrimitive{xmin - kSafetyMargin, xmax + kSafetyMargin,
                                           ymin - kSafetyMargin, ymax + kSafetyMargin});
        }

        auto empty() const -> bool
        {
            return !hasPositionBounds_ && !hasVelocityBounds_ &&
                   circles_.empty() && rects_.empty();
        }

        auto checkState(const DIState &state) const -> bool
        {
            if (hasPositionBounds_)
            {
                if (state.x < xmin_ || state.x > xmax_ ||
                    state.y < ymin_ || state.y > ymax_)
                    return false;
            }

            if (hasVelocityBounds_)
            {
                if (state.vx < vxmin_ || state.vx > vxmax_ ||
                    state.vy < vymin_ || state.vy > vymax_)
                    return false;
            }

            for (const auto &circle : circles_)
            {
                const double dx = state.x - circle.x;
                const double dy = state.y - circle.y;
                if (dx * dx + dy * dy <= circle.r2)
                    return false;
            }

            for (const auto &rect : rects_)
            {
                if (state.x >= rect.xmin && state.x <= rect.xmax &&
                    state.y >= rect.ymin && state.y <= rect.ymax)
                    return false;
            }

            return true;
        }

        template <std::size_t W>
        auto checkStateBlock(const DIStateBlock<W> &block, unsigned int count) const -> bool
        {
            if constexpr (W == 8)
            {
                if (count == 8)
                    return checkStateBlock8(block);
            }

            for (unsigned int i = 0; i < count; ++i)
            {
                if (!checkState(DIState{block.x[i], block.y[i], block.vx[i], block.vy[i]}))
                    return false;
            }

            return true;
        }

    private:
        bool hasPositionBounds_{false};
        bool hasVelocityBounds_{false};
        double xmin_{0.0};
        double xmax_{0.0};
        double ymin_{0.0};
        double ymax_{0.0};
        double vxmin_{0.0};
        double vxmax_{0.0};
        double vymin_{0.0};
        double vymax_{0.0};
        std::vector<CirclePrimitive> circles_;
        std::vector<RectPrimitive> rects_;

        auto checkStateBlock8Scalar(const DIStateBlock<8> &block) const -> bool
        {
            for (std::size_t i = 0; i < 8; ++i)
            {
                if (!checkState(DIState{block.x[i], block.y[i], block.vx[i], block.vy[i]}))
                    return false;
            }

            return true;
        }

#if defined(__AVX2__)
        auto checkStateBlock8Avx2(const DIStateBlock<8> &block) const -> bool
        {
            const __m256 xs = _mm256_loadu_ps(block.x.data());
            const __m256 ys = _mm256_loadu_ps(block.y.data());
            const __m256 vxs = _mm256_loadu_ps(block.vx.data());
            const __m256 vys = _mm256_loadu_ps(block.vy.data());

            unsigned int invalidMask = 0u;

            auto anyMask = [](const __m256 mask) -> unsigned int
            {
                return static_cast<unsigned int>(_mm256_movemask_ps(mask));
            };

            if (hasPositionBounds_)
            {
                invalidMask |= anyMask(_mm256_cmp_ps(xs, _mm256_set1_ps(static_cast<float>(xmin_)), _CMP_LT_OQ));
                invalidMask |= anyMask(_mm256_cmp_ps(xs, _mm256_set1_ps(static_cast<float>(xmax_)), _CMP_GT_OQ));
                invalidMask |= anyMask(_mm256_cmp_ps(ys, _mm256_set1_ps(static_cast<float>(ymin_)), _CMP_LT_OQ));
                invalidMask |= anyMask(_mm256_cmp_ps(ys, _mm256_set1_ps(static_cast<float>(ymax_)), _CMP_GT_OQ));
            }

            if (hasVelocityBounds_)
            {
                invalidMask |= anyMask(_mm256_cmp_ps(vxs, _mm256_set1_ps(static_cast<float>(vxmin_)), _CMP_LT_OQ));
                invalidMask |= anyMask(_mm256_cmp_ps(vxs, _mm256_set1_ps(static_cast<float>(vxmax_)), _CMP_GT_OQ));
                invalidMask |= anyMask(_mm256_cmp_ps(vys, _mm256_set1_ps(static_cast<float>(vymin_)), _CMP_LT_OQ));
                invalidMask |= anyMask(_mm256_cmp_ps(vys, _mm256_set1_ps(static_cast<float>(vymax_)), _CMP_GT_OQ));
            }

            for (const auto &circle : circles_)
            {
                const __m256 dx = _mm256_sub_ps(xs, _mm256_set1_ps(static_cast<float>(circle.x)));
                const __m256 dy = _mm256_sub_ps(ys, _mm256_set1_ps(static_cast<float>(circle.y)));
                const __m256 dist2 = _mm256_add_ps(_mm256_mul_ps(dx, dx), _mm256_mul_ps(dy, dy));
                invalidMask |= anyMask(_mm256_cmp_ps(
                    dist2, _mm256_set1_ps(static_cast<float>(circle.r2)), _CMP_LE_OQ));
            }

            for (const auto &rect : rects_)
            {
                const __m256 xGeMin = _mm256_cmp_ps(xs, _mm256_set1_ps(static_cast<float>(rect.xmin)), _CMP_GE_OQ);
                const __m256 xLeMax = _mm256_cmp_ps(xs, _mm256_set1_ps(static_cast<float>(rect.xmax)), _CMP_LE_OQ);
                const __m256 yGeMin = _mm256_cmp_ps(ys, _mm256_set1_ps(static_cast<float>(rect.ymin)), _CMP_GE_OQ);
                const __m256 yLeMax = _mm256_cmp_ps(ys, _mm256_set1_ps(static_cast<float>(rect.ymax)), _CMP_LE_OQ);
                const __m256 inside =
                    _mm256_and_ps(_mm256_and_ps(xGeMin, xLeMax), _mm256_and_ps(yGeMin, yLeMax));
                invalidMask |= anyMask(inside);
            }

            return invalidMask == 0u;
        }
#endif

#if defined(__aarch64__) && (defined(__ARM_NEON) || defined(__ARM_NEON__))
        auto checkStateBlock8Neon(const DIStateBlock<8> &block) const -> bool
        {
            return checkStateBlock8Scalar(block);
        }
#endif

        auto checkStateBlock8(const DIStateBlock<8> &block) const -> bool
        {
#if defined(__AVX2__)
            return checkStateBlock8Avx2(block);
#elif defined(__aarch64__) && (defined(__ARM_NEON) || defined(__ARM_NEON__))
            return checkStateBlock8Neon(block);
#else
            return checkStateBlock8Scalar(block);
#endif
        }
    };
}  // namespace ompl::control::detail
