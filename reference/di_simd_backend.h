#pragma once

#include <array>
#include <cstddef>

#if defined(__AVX2__)
#include <immintrin.h>
#elif defined(__ARM_NEON) || defined(__ARM_NEON__)
#include <arm_neon.h>
#endif

#include "di_primitive_types.h"

namespace ompl::control::detail
{
    inline constexpr bool kHasAvx2Backend =
#if defined(__AVX2__)
        true;
#else
        false;
#endif

    inline constexpr bool kHasNeonBackend =
#if defined(__aarch64__) && (defined(__ARM_NEON) || defined(__ARM_NEON__))
        true;
#else
        false;
#endif

    inline void evaluateEdgeStateBlock8Scalar(
        const DIEdgePrimitive &primitive,
        const std::array<double, 8> &times,
        DIStateBlock<8> &out)
    {
        for (std::size_t i = 0; i < 8; ++i)
        {
            const float t = static_cast<float>(times[i]);
            const float t2 = t * t;
            const float t3 = t2 * t;
            const float x2 = static_cast<float>(primitive.x2);
            const float y2 = static_cast<float>(primitive.y2);
            const float x3 = static_cast<float>(primitive.x3);
            const float y3 = static_cast<float>(primitive.y3);

            out.x[i] = static_cast<float>(primitive.x0) + t * static_cast<float>(primitive.vx0) +
                       t2 * x2 + t3 * x3;
            out.y[i] = static_cast<float>(primitive.y0) + t * static_cast<float>(primitive.vy0) +
                       t2 * y2 + t3 * y3;
            out.vx[i] = static_cast<float>(primitive.vx0) +
                        2.0f * t * x2 + 3.0f * t2 * x3;
            out.vy[i] = static_cast<float>(primitive.vy0) +
                        2.0f * t * y2 + 3.0f * t2 * y3;
        }
    }

#if defined(__AVX2__)
    inline void evaluateEdgeStateBlock8Avx2(
        const DIEdgePrimitive &primitive,
        const std::array<double, 8> &times,
        DIStateBlock<8> &out)
    {
        alignas(32) float timeValues[8];
        for (std::size_t i = 0; i < 8; ++i)
            timeValues[i] = static_cast<float>(times[i]);

        const __m256 t = _mm256_load_ps(timeValues);
        const __m256 t2 = _mm256_mul_ps(t, t);
        const __m256 t3 = _mm256_mul_ps(t2, t);
        const __m256 x2 = _mm256_set1_ps(static_cast<float>(primitive.x2));
        const __m256 y2 = _mm256_set1_ps(static_cast<float>(primitive.y2));
        const __m256 x3 = _mm256_set1_ps(static_cast<float>(primitive.x3));
        const __m256 y3 = _mm256_set1_ps(static_cast<float>(primitive.y3));
        const __m256 x0 = _mm256_set1_ps(static_cast<float>(primitive.x0));
        const __m256 y0 = _mm256_set1_ps(static_cast<float>(primitive.y0));
        const __m256 vx0 = _mm256_set1_ps(static_cast<float>(primitive.vx0));
        const __m256 vy0 = _mm256_set1_ps(static_cast<float>(primitive.vy0));
        const __m256 two = _mm256_set1_ps(2.0f);
        const __m256 three = _mm256_set1_ps(3.0f);

        const __m256 x = _mm256_add_ps(
            _mm256_add_ps(_mm256_add_ps(x0, _mm256_mul_ps(t, vx0)), _mm256_mul_ps(t2, x2)),
            _mm256_mul_ps(t3, x3));
        const __m256 y = _mm256_add_ps(
            _mm256_add_ps(_mm256_add_ps(y0, _mm256_mul_ps(t, vy0)), _mm256_mul_ps(t2, y2)),
            _mm256_mul_ps(t3, y3));
        const __m256 vx = _mm256_add_ps(
            _mm256_add_ps(vx0, _mm256_mul_ps(_mm256_mul_ps(two, t), x2)),
            _mm256_mul_ps(_mm256_mul_ps(three, t2), x3));
        const __m256 vy = _mm256_add_ps(
            _mm256_add_ps(vy0, _mm256_mul_ps(_mm256_mul_ps(two, t), y2)),
            _mm256_mul_ps(_mm256_mul_ps(three, t2), y3));

        _mm256_storeu_ps(out.x.data(), x);
        _mm256_storeu_ps(out.y.data(), y);
        _mm256_storeu_ps(out.vx.data(), vx);
        _mm256_storeu_ps(out.vy.data(), vy);
    }
#endif

#if defined(__aarch64__) && (defined(__ARM_NEON) || defined(__ARM_NEON__))
    inline void evaluateEdgeStateBlock8Neon(
        const DIEdgePrimitive &primitive,
        const std::array<double, 8> &times,
        DIStateBlock<8> &out)
    {
        evaluateEdgeStateBlock8Scalar(primitive, times, out);
    }
#endif

    inline void evaluateEdgeStateBlock8Simd(
        const DIEdgePrimitive &primitive,
        const std::array<double, 8> &times,
        DIStateBlock<8> &out)
    {
#if defined(__AVX2__)
        evaluateEdgeStateBlock8Avx2(primitive, times, out);
#elif defined(__aarch64__) && (defined(__ARM_NEON) || defined(__ARM_NEON__))
        evaluateEdgeStateBlock8Neon(primitive, times, out);
#else
        evaluateEdgeStateBlock8Scalar(primitive, times, out);
#endif
    }
}  // namespace ompl::control::detail
