#pragma once
#include "common/flat_state.h"
#include "common/flat_control.h"
#include "common/math_utils.h"
#include <array>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <variant>

namespace rrtpff {
template<int Dim, int Order, typename Metadata = std::monostate>
struct FlatTrajectory {
    using State=FlatState<Dim,Order>;
    using Control=FlatControl<Dim>;
    using Output=typename State::Output;
    using MetadataType=Metadata;
    static constexpr int Degree=2*Order-1;
    double T{0}, cost{0};
    Eigen::Matrix<double,Dim,2*Order> coefficients=Eigen::Matrix<double,Dim,2*Order>::Zero();
    State start{}, end{};
    Metadata metadata{};
    Output derivative(double t,int d) const {
        if(!std::isfinite(t)||t < -1e-10||t > T+1e-10 || d<0 || d>Degree)
            throw std::out_of_range("Trajectory evaluation outside its domain");
        t=std::clamp(t,0.0,T);
        Output result=Output::Zero();
        for(int k=Degree;k>=d;--k)
            result=result*t+coefficients.col(k)*(factorial(k)/factorial(k-d));
        return result;
    }
    State evaluate(double t) const {
        State result;
        for(int d=0;d<Order;++d) {
            result.setDerivative(d, derivative(t, d));
        }
        return result;
    }

    State evaluateState(double t) const { return evaluate(t); }

    Control evaluateControl(double t) const {
        Control result;
        result.values = derivative(t, Order);
        return result;
    }

    Output control(double t) const {return derivative(t,Order);
}
    Output velocity(double t) const {return derivative(t,1);
}
    Output acceleration(double t) const {return derivative(t,2);
}
    Output jerk(double t) const {return derivative(t,3);
}
    Output snap(double t) const {return derivative(t,4);
}
    template<std::size_t Width>
    std::array<State,Width> evaluateBatch(const std::array<double,Width>& times) const {
        std::array<State,Width> out{};
        for(std::size_t i=0;i<Width;++i) out[i]=evaluate(times[i]);
        return out;
 // scalar reference; SIMD is a separate later layer
    }
};
}
