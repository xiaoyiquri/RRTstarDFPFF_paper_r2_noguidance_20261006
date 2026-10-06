# Fast validity integration

This copy uses the AVX2 rake layout for one trajectory. The temporary SoA
state block uses `float` and the AVX2 backend uses `__m256`, so the SIMD width
is `k = 8`.

For `num_collision_check_points = N`, the endpoints `t_0` and `t_N` are
checked separately. Interior samples are grouped consecutively:

```text
[t_1, ..., t_8], [t_9, ..., t_16], ...
```

Each full group is evaluated as one `DIStateBlock<8>` in float SoA form:
`x[8]`, `y[8]`, `vx[8]`, and `vy[8]`. A final group with fewer than eight
samples uses the scalar tail path and checks only its valid lanes.

The built-in `FastEnvironment2D` AVX2 path checks position bounds, velocity
bounds, circles, and rectangles with lane masks. A set bit in any obstacle or
constraint mask rejects the whole trajectory edge.

`MyRRTstar` now supports two optional fast validity hooks:

```cpp
planner->setFastValidityChecker(...);
planner->setFastBatchValidityChecker(...);
```

Use at least the scalar hook when an external checker is required. The built-in
environment path already performs the 8-sample AVX2 validity pass.

In `test.cpp`, keep the OMPL checker but factor out the numeric checker:

```cpp
bool isStateValidFast(double x, double y, double vx, double vy)
{
    if (x < kXMin || x > kXMax ||
        y < kYMin || y > kYMax ||
        vx < kVMin || vx > kVMax ||
        vy < kVMin || vy > kVMax)
        return false;

    for (const auto &c : g_planCircles)
    {
        const double dx = x - c.x;
        const double dy = y - c.y;
        const double safeRadius = c.radius + 0.1;
        if (dx * dx + dy * dy <= safeRadius * safeRadius)
            return false;
    }

    for (const auto &r : g_planRects)
    {
        if (x >= r.x_min - 0.1 && x <= r.x_max + 0.1 &&
            y >= r.y_min - 0.1 && y <= r.y_max + 0.1)
            return false;
    }

    return true;
}

bool isStateValid(const ob::State *state)
{
    const auto *s = state->as<ob::RealVectorStateSpace::StateType>();
    return isStateValidFast(s->values[0], s->values[1], s->values[2], s->values[3]);
}
```

After creating the planner:

```cpp
planner->setFastValidityChecker(
    [](double x, double y, double vx, double vy)
    {
        return isStateValidFast(x, y, vx, vy);
    });

planner->setFastBatchValidityChecker(
    [](const double *x,
       const double *y,
       const double *vx,
       const double *vy,
       unsigned int count)
    {
        for (unsigned int i = 0; i < count; ++i)
        {
            if (!isStateValidFast(x[i], y[i], vx[i], vy[i]))
                return false;
        }
        return true;
    });
```

The batch hook remains available for external validity models. It is not used
when the built-in `FastEnvironment2D` has been configured, so the planner's
standard 2D test path uses the rake-aware AVX2 circle and rectangle checks.
