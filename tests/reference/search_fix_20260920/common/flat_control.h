#pragma once

#include "common/types.h"
#include <Eigen/Core>

namespace rrtpff {

template<int Dim>
struct FlatControl {
    static_assert(Dim > 0, "Positive flat-control dimension required");

    using Vector = Eigen::Matrix<Scalar, Dim, 1>;

    Vector values = Vector::Zero();

    bool finite() const { return values.allFinite(); }

    Scalar& operator()(int index) { return values(index); }
    const Scalar& operator()(int index) const { return values(index); }

    operator const Vector&() const { return values; }
    operator Vector&() { return values; }
};

}  // namespace rrtpff
