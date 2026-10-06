#include <cstddef>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <complex>
#include <array>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/PlannerData.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/util/RandomNumbers.h>

#include <ompl/control/SimpleSetup.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/PathControl.h>
#include <ompl/control/StatePropagator.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/util/Console.h>

#include <opencv2/opencv.hpp>
#include "MyRRTstar.h"
#include "di_simd_backend.h"

namespace ob = ompl::base;
namespace oc = ompl::control;

static constexpr double kPi    = 3.14159265358979323846;
static constexpr double kTwoPi = 2.0 * kPi;
static constexpr double kWorldXMin = 0.0;
static constexpr double kWorldXMax = 20.0;
static constexpr double kWorldYMin = 0.0;
static constexpr double kWorldYMax = 20.0;
static constexpr double kVelMin = -10.0;
static constexpr double kVelMax = 10.0;
static constexpr double kStartX = 2.0;
static constexpr double kStartY = 2.0;
static constexpr double kGoalX = 18.0;
static constexpr double kGoalY = 18.0;
static constexpr double kGoalTolerance = 0.05;

// ============================================================================
// 障碍�?
// ============================================================================
struct CircleObstacle
{
    double x, y, radius;
    CircleObstacle(double x_, double y_, double r_) : x(x_), y(y_), radius(r_) {}
};

struct RectangleObstacle
{
    double x_min, x_max, y_min, y_max;
    RectangleObstacle(double xmin, double xmax, double ymin, double ymax)
        : x_min(xmin), x_max(xmax), y_min(ymin), y_max(ymax) {}
};

std::vector<CircleObstacle>    g_circles;
std::vector<RectangleObstacle> g_rects;

// ============================================================================
// 只约束位置目�?
// ============================================================================
class PositionOnlyGoal : public ob::GoalSampleableRegion
{
public:
    PositionOnlyGoal(const ob::SpaceInformationPtr &si, double gx, double gy, double tol)
        : ob::GoalSampleableRegion(si), gx_(gx), gy_(gy), rng_()
    {
        setThreshold(tol);
    }

    double distanceGoal(const ob::State *st) const override
    {
        const auto *s = st->as<ob::RealVectorStateSpace::StateType>();
        const double dx = s->values[0] - gx_;
        const double dy = s->values[1] - gy_;
        return std::sqrt(dx * dx + dy * dy);
    }

    void sampleGoal(ob::State *st) const override
    {
        auto *s = st->as<ob::RealVectorStateSpace::StateType>();

        const double r = threshold_ * std::sqrt(rng_.uniform01());
        const double a = rng_.uniformReal(0.0, kTwoPi);

        s->values[0] = gx_ + r * std::cos(a);
        s->values[1] = gy_ + r * std::sin(a);
        s->values[2] = 0.0;
        s->values[3] = 0.0;
    }

    bool canSample() const  { return true; }
    unsigned int maxSampleCount() const  { return 1000000u; }

private:
    double gx_, gy_;
    mutable ompl::RNG rng_;
};

// ============================================================================
// 双积分器传播�?
// ============================================================================
class DoubleIntegratorPropagator : public oc::StatePropagator
{
public:
    explicit DoubleIntegratorPropagator(const oc::SpaceInformationPtr &si)
        : oc::StatePropagator(si)
    {
    }

    void propagate(const ob::State *state,
                   const oc::Control *control,
                   const double duration,
                   ob::State *result) const override
    {
        const auto *s = state->as<ob::RealVectorStateSpace::StateType>();
        const auto *u = control->as<oc::RealVectorControlSpace::ControlType>();
        auto *r = result->as<ob::RealVectorStateSpace::StateType>();

        const double x  = s->values[0];
        const double y  = s->values[1];
        const double vx = s->values[2];
        const double vy = s->values[3];

        const double ax = u->values[0];
        const double ay = u->values[1];

        r->values[0] = x  + vx * duration + 0.5 * ax * duration * duration;
        r->values[1] = y  + vy * duration + 0.5 * ay * duration * duration;
        r->values[2] = vx + ax * duration;
        r->values[3] = vy + ay * duration;
    }

    bool canPropagateBackward() const override
    {
        return false;
    }
};

// ============================================================================
// 状态有效性检�?
// ============================================================================
bool isStateValid(const ob::State *state)
{
    const auto *s = state->as<ob::RealVectorStateSpace::StateType>();
    const double x  = s->values[0];
    const double y  = s->values[1];
    const double vx = s->values[2];
    const double vy = s->values[3];

    if (!std::isfinite(x) || !std::isfinite(y) ||
        !std::isfinite(vx) || !std::isfinite(vy))
        return false;

    if (x < kWorldXMin || x > kWorldXMax ||
        y < kWorldYMin || y > kWorldYMax)
        return false;

    if (vx < kVelMin || vx > kVelMax ||
        vy < kVelMin || vy > kVelMax)
        return false;

    for (const auto &c : g_circles)
    {
        const double dx = x - c.x;
        const double dy = y - c.y;
        if (dx * dx + dy * dy <= c.radius * c.radius)
            return false;
    }

    for (const auto &r : g_rects)
    {
        if (x >= r.x_min && x <= r.x_max &&
            y >= r.y_min && y <= r.y_max)
            return false;
    }

    return true;
}

void configureFastEnvironment(const std::shared_ptr<oc::RRTstarDI> &planner)
{
    planner->clearFastEnvironment();
    planner->setFastPositionBounds(kWorldXMin, kWorldXMax, kWorldYMin, kWorldYMax);
    planner->setFastVelocityBounds(kVelMin, kVelMax, kVelMin, kVelMax);

    for (const auto &c : g_circles)
        planner->addFastCircleObstacle(c.x, c.y, c.radius);

    for (const auto &r : g_rects)
        planner->addFastRectObstacle(r.x_min, r.x_max, r.y_min, r.y_max);
}

// ============================================================================
// 导出�?
// ============================================================================
void exportTreeToText(const ob::PlannerData &data, const std::string &filename)
{
    std::ofstream file(filename);
    file << std::fixed << std::setprecision(6);

    file << "# RRTstarDI Tree Structure\n";
    file << "# Total Vertices: " << data.numVertices() << "\n";
    file << "# Total Edges:    " << data.numEdges()    << "\n\n";

    file << "=== NODES ===\n";
    for (unsigned int i = 0; i < data.numVertices(); ++i)
    {
        const ob::State *state = data.getVertex(i).getState();
        const auto *s = state->as<ob::RealVectorStateSpace::StateType>();

        file << "Node " << i << ": "
             << s->values[0] << " " << s->values[1] << " "
             << s->values[2] << " " << s->values[3] << "\n";
    }

    file << "\n=== EDGES ===\n";
    for (unsigned int i = 0; i < data.numVertices(); ++i)
    {
        std::vector<unsigned int> edgeList;
        data.getEdges(i, edgeList);
        for (auto j : edgeList)
            file << "Edge: " << i << " -> " << j << "\n";
    }

    file.close();
    std::cout << "Tree exported to: " << filename << std::endl;
}

// ============================================================================
// 导出 PathControl 状态序�?
// ============================================================================
void exportOptimalPath(const oc::PathControl &path, const std::string &filename)
{
    std::ofstream file(filename);
    file << std::fixed << std::setprecision(6);

    file << "# Optimal Solution Path (States)\n";
    file << "# Total states: " << path.getStateCount() << "\n";
    file << "# Format: x y vx vy\n\n";

    for (std::size_t i = 0; i < path.getStateCount(); ++i)
    {
        const auto *s = path.getState(i)->as<ob::RealVectorStateSpace::StateType>();
        file << s->values[0] << " " << s->values[1] << " "
             << s->values[2] << " " << s->values[3] << "\n";
    }

    file.close();
    std::cout << "Optimal path exported to: " << filename << std::endl;
}

// ============================================================================
// 导出控制重放轨迹
// ============================================================================
void exportDynamicTrajectory(const oc::PathControl &path,
                             const std::string &filename)
{
    std::ofstream file(filename);
    file << std::fixed << std::setprecision(6);

    file << "# Detailed trajectory replayed from PathControl\n";
    file << "# Format: segment_idx t x y vx vy ax ay duration\n\n";

    if (path.getStateCount() < 1 || path.getControlCount() < 1)
    {
        file << "# Empty PathControl.\n";
        return;
    }

    for (std::size_t i = 0; i < path.getControlCount(); ++i)
    {
        const ob::State *s = path.getState(i);
        const oc::Control *c = path.getControl(i);
        const double dur = path.getControlDuration(i);

        if (!s || !c || !(dur > 0.0))
            continue;

        const auto *sv = s->as<ob::RealVectorStateSpace::StateType>();
        const auto *cv = c->as<oc::RealVectorControlSpace::ControlType>();

        const double x0  = sv->values[0];
        const double y0  = sv->values[1];
        const double vx0 = sv->values[2];
        const double vy0 = sv->values[3];

        const double ax = cv->values[0];
        const double ay = cv->values[1];

        const unsigned int numSamples = 50;

        for (unsigned int k = 0; k <= numSamples; ++k)
        {
            const double t = dur * static_cast<double>(k) / static_cast<double>(numSamples);

            const double x  = x0 + vx0 * t + 0.5 * ax * t * t;
            const double y  = y0 + vy0 * t + 0.5 * ay * t * t;
            const double vx = vx0 + ax * t;
            const double vy = vy0 + ay * t;

            file << i   << " "
                 << t   << " "
                 << x   << " "
                 << y   << " "
                 << vx  << " "
                 << vy  << " "
                 << ax  << " "
                 << ay  << " "
                 << dur << "\n";
        }
    }

    file.close();
    std::cout << "Dynamic trajectory exported to: " << filename << std::endl;
}

// ============================================================================
// 坐标映射
// ============================================================================
cv::Point2i worldToPixel(double x, double y,
                         double xmin, double xmax,
                         double ymin, double ymax,
                         int imgW, int imgH,
                         int margin = 0)
{
    const double sx = (imgW - 2.0 * margin) / (xmax - xmin);
    const double sy = (imgH - 2.0 * margin) / (ymax - ymin);

    const double px = margin + (x - xmin) * sx;
    const double py = imgH - (margin + (y - ymin) * sy);

    return cv::Point2i((int)std::round(px), (int)std::round(py));
}

// ============================================================================
// 覆盖�?
// ============================================================================
void drawOverlayAxesTicksStartGoal(cv::Mat &img,
                                   double xmin, double xmax,
                                   double ymin, double ymax,
                                   double sx, double sy,
                                   double gx, double gy)
{
    const int W = img.cols;
    const int H = img.rows;

    const cv::Scalar kAxisColor(0, 0, 0);
    cv::rectangle(img, cv::Rect(0, 0, W, H), kAxisColor, 1, cv::LINE_AA);

    cv::Point2i pS = worldToPixel(sx, sy, xmin, xmax, ymin, ymax, W, H, 0);
    cv::Point2i pG = worldToPixel(gx, gy, xmin, xmax, ymin, ymax, W, H, 0);

    cv::circle(img, pS, 6, cv::Scalar(0, 255, 0), -1, cv::LINE_AA);
    cv::putText(img, "Start", pS + cv::Point(8, -8), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(0, 128, 0), 1, cv::LINE_AA);

    cv::circle(img, pG, 6, cv::Scalar(0, 0, 255), -1, cv::LINE_AA);
    cv::putText(img, "Goal", pG + cv::Point(8, -8), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(0, 0, 180), 1, cv::LINE_AA);
}

// ============================================================================
// FixedVel 绘图辅助：对�?MATLAB DI_time / DI_state�?
// 这些函数只用于显示，不参与规划或碰撞检测�?
// ============================================================================
struct PlotDIState { double x, y, vx, vy; };

std::array<double, 5> fixedVelTimeCoefficients(const PlotDIState &a, const PlotDIState &b)
{
    return {{1.0, 0.0,
        -4.0*a.vx*a.vx - 4.0*a.vx*b.vx - 4.0*a.vy*a.vy - 4.0*a.vy*b.vy -
        4.0*b.vx*b.vx - 4.0*b.vy*b.vy,
        24.0*a.vx*b.x - 24.0*a.y*a.vy - 24.0*a.x*b.vx - 24.0*a.x*a.vx -
        24.0*a.y*b.vy + 24.0*a.vy*b.y + 24.0*b.x*b.vx + 24.0*b.y*b.vy,
        -36.0*a.x*a.x + 72.0*a.x*b.x - 36.0*a.y*a.y + 72.0*a.y*b.y -
        36.0*b.x*b.x - 36.0*b.y*b.y}};
}

bool fixedVelTime(const PlotDIState &a, const PlotDIState &b, double &tf)
{
    const auto c = fixedVelTimeCoefficients(a, b);
    double scale = 0.0;
    for (double v : c) scale = std::max(scale, std::abs(v));
    if (!(scale > 0.0) || !std::isfinite(scale)) return false;
    using C = std::complex<double>;
    constexpr double pi = 3.14159265358979323846;
    std::array<C, 4> roots{};
    for (unsigned int i = 0; i < 4; ++i)
        roots[i] = std::polar(1.0 + 0.4 * scale, 2.0 * pi * i / 4.0);
    auto eval = [&](const C &z) { C v(c[0]); for (unsigned int i=1;i<5;++i) v=v*z+c[i]; return v; };
    for (unsigned int iter = 0; iter < 200; ++iter)
    {
        double maxDelta = 0.0;
        for (unsigned int i = 0; i < 4; ++i)
        {
            C denom(1.0, 0.0);
            for (unsigned int j = 0; j < 4; ++j) if (i != j) denom *= roots[i] - roots[j];
            if (std::abs(denom) < 1e-18) continue;
            const C delta = eval(roots[i]) / denom;
            roots[i] -= delta;
            maxDelta = std::max(maxDelta, std::abs(delta));
        }
        if (maxDelta < 1e-11 * (1.0 + scale)) break;
    }
    tf = std::numeric_limits<double>::infinity();
    for (const C &r : roots)
    {
        if (std::abs(r.imag()) > 1e-7 * (1.0 + std::abs(r.real()))) continue;
        if (r.real() >= -1e-9 && std::isfinite(r.real())) tf = std::min(tf, std::max(0.0, r.real()));
    }
    return std::isfinite(tf) && tf > 1e-9;
}

PlotDIState fixedVelState(double t, double ts, const PlotDIState &a, const PlotDIState &b)
{
    const double dt=t-ts, ts2=ts*ts, ts3=ts2*ts;
    return {
        b.x+b.vx*dt + dt*dt*dt*(2*a.x-2*b.x+ts*a.vx+ts*b.vx)/ts3 + dt*dt*(3*a.x-3*b.x+ts*a.vx+2*ts*b.vx)/ts2,
        b.y+b.vy*dt + dt*dt*dt*(2*a.y-2*b.y+ts*a.vy+ts*b.vy)/ts3 + dt*dt*(3*a.y-3*b.y+ts*a.vy+2*ts*b.vy)/ts2,
        b.vx + 2*dt*(3*a.x-3*b.x+ts*a.vx+2*ts*b.vx)/ts2 + 3*dt*dt*(2*a.x-2*b.x+ts*a.vx+ts*b.vx)/ts3,
        b.vy + 2*dt*(3*a.y-3*b.y+ts*a.vy+2*ts*b.vy)/ts2 + 3*dt*dt*(2*a.y-2*b.y+ts*a.vy+ts*b.vy)/ts3};
}

bool readPlotDIState(const ob::State *state, PlotDIState &out)
{
    if (!state) return false;
    const auto *s=state->as<ob::RealVectorStateSpace::StateType>();
    if (!s) return false;
    out={s->values[0],s->values[1],s->values[2],s->values[3]};
    return std::isfinite(out.x)&&std::isfinite(out.y)&&std::isfinite(out.vx)&&std::isfinite(out.vy);
}

template <typename Consumer>
bool drawFixedVelSegment(const PlotDIState &a, const PlotDIState &b, double xmin, double xmax,
                         double ymin, double ymax, int width, int height, Consumer &&consume)
{
    double tf=0.0; if (!fixedVelTime(a,b,tf)) return false;
    const unsigned int n=std::max(1u,static_cast<unsigned int>(std::ceil(tf/0.2)));
    std::vector<cv::Point2i> points;
    points.reserve(n + 1);
    for (unsigned int k=0;k<=n;++k)
    {
        const PlotDIState s=fixedVelState(tf*static_cast<double>(k)/n,tf,a,b);
        if (!std::isfinite(s.x)||!std::isfinite(s.y)||s.x<xmin||s.x>xmax||s.y<ymin||s.y>ymax) return false;
        points.push_back(worldToPixel(s.x,s.y,xmin,xmax,ymin,ymax,width,height,0));
    }
    for (const auto &p : points) consume(p);
    return true;
}

// ============================================================================
// 画整棵树
// ============================================================================
void drawWholeTree(const ob::PlannerData &data,
                   oc::RRTstarDI *,
                   cv::Mat &img,
                   double xmin, double xmax,
                   double ymin, double ymax)
{
    const int W = img.cols;
    const int H = img.rows;

    for (unsigned int i = 0; i < data.numVertices(); ++i)
    {
        const ob::State *s1 = data.getVertex(i).getState();

        std::vector<unsigned int> edgeList;
        data.getEdges(i, edgeList);

        for (auto j : edgeList)
        {
            const ob::State *s2 = data.getVertex(j).getState();

            PlotDIState a{}, b{};
            if (!readPlotDIState(s1, a) || !readPlotDIState(s2, b)) continue;
            std::vector<cv::Point2i> curve;
            if (!drawFixedVelSegment(a, b, xmin, xmax, ymin, ymax, W, H,
                                     [&](const cv::Point2i &p) { curve.push_back(p); }))
                continue;

            for (std::size_t k = 1; k < curve.size(); ++k)
                cv::line(img, curve[k - 1], curve[k], cv::Scalar(180, 180, 180), 1, cv::LINE_AA);
        }
    }

    for (unsigned int i = 0; i < data.numVertices(); ++i)
    {
        const auto *s = data.getVertex(i).getState()->as<ob::RealVectorStateSpace::StateType>();
        cv::Point2i p = worldToPixel(s->values[0], s->values[1], xmin, xmax, ymin, ymax, img.cols, img.rows, 0);
        cv::circle(img, p, 2, cv::Scalar(120, 120, 120), -1, cv::LINE_AA);
    }
}

// ============================================================================
// 画最优轨迹：按相邻完整状态使�?FixedVel 解析轨迹重建
// ============================================================================
void drawBestTrajectory(const oc::PathControl &path,
                        cv::Mat &img,
                        double xmin, double xmax,
                        double ymin, double ymax)
{
    if (path.getStateCount() < 2)
        return;

    const int W = img.cols;
    const int H = img.rows;

    cv::Point2i lastPt;
    bool hasLast = false;

    for (std::size_t i = 0; i + 1 < path.getStateCount(); ++i)
    {
        const ob::State *s = path.getState(i);
        const ob::State *sNext = path.getState(i + 1);
        PlotDIState a{}, b{};
        if (!readPlotDIState(s, a) || !readPlotDIState(sNext, b)) continue;
        drawFixedVelSegment(a, b, xmin, xmax, ymin, ymax, W, H,
                            [&](const cv::Point2i &pt)
                            {
                                if (hasLast)
                                    cv::line(img, lastPt, pt, cv::Scalar(0, 0, 255), 3, cv::LINE_AA);
                                lastPt = pt;
                                hasLast = true;
                            });
    }
}

// ============================================================================
// 画树和最优轨�?
// ============================================================================
void drawTreeAndTrajectory(const ob::PlannerData &data,
                           const oc::PathControl &path,
                           oc::RRTstarDI *planner,
                           const std::string &imgFile,
                           double xmin, double xmax,
                           double ymin, double ymax,
                           double sx, double sy,
                           double gx, double gy)
{
    const int W = 800;
    const int H = 800;
    cv::Mat img(H, W, CV_8UC3, cv::Scalar(255, 255, 255));

    const double sxPix = static_cast<double>(W) / (xmax - xmin);
    const double syPix = static_cast<double>(H) / (ymax - ymin);
    const double sPix  = std::min(sxPix, syPix);

    // 1) 障碍�?
    for (const auto &c : g_circles)
    {
        cv::Point2i pc = worldToPixel(c.x, c.y, xmin, xmax, ymin, ymax, W, H, 0);
        const int pr = static_cast<int>(std::round(c.radius * sPix));
        cv::circle(img, pc, pr, cv::Scalar(100, 100, 100), -1, cv::LINE_AA);
    }

    for (const auto &r : g_rects)
    {
        cv::Point2i p1 = worldToPixel(r.x_min, r.y_min, xmin, xmax, ymin, ymax, W, H, 0);
        cv::Point2i p2 = worldToPixel(r.x_max, r.y_max, xmin, xmax, ymin, ymax, W, H, 0);

        cv::Rect rect(std::min(p1.x, p2.x),
                      std::min(p1.y, p2.y),
                      std::abs(p1.x - p2.x),
                      std::abs(p1.y - p2.y));

        cv::rectangle(img, rect, cv::Scalar(100, 100, 100), -1, cv::LINE_AA);
    }

    // 2) 整棵�?
    drawWholeTree(data, planner, img, xmin, xmax, ymin, ymax);

    // 3) 最优轨�?
    drawBestTrajectory(path, img, xmin, xmax, ymin, ymax);

    // 4) 覆盖�?
    drawOverlayAxesTicksStartGoal(img, xmin, xmax, ymin, ymax, sx, sy, gx, gy);

    cv::imshow("RRTstarDI Tree + Best Trajectory", img);
    cv::imwrite(imgFile, img);
    std::cout << "Saved image to: " << imgFile << std::endl;
    cv::waitKey(0);
}

// ============================================================================
// 主函�?
// ============================================================================
struct RunOptions
{
    bool benchmark{false};
    bool help{false};
    unsigned int runs{50};
    unsigned int warmupRuns{5};
    unsigned int maxSamples{4000};
    unsigned int seed{40};
    double timeoutSeconds{30.0};
};

struct PlanningInstance
{
    std::unique_ptr<oc::SimpleSetup> setup;
    std::shared_ptr<oc::RRTstarDI> planner;
};

void printUsage(const char *program)
{
    std::cout
        << "Usage:\n"
        << "  " << program << "\n"
        << "  " << program << " --benchmark [options]\n\n"
        << "Benchmark options:\n"
        << "  --runs N       Measured runs (default: 50)\n"
        << "  --warmup N     Warmup runs (default: 5)\n"
        << "  --samples N    Accepted samples per run (default: 4000)\n"
        << "  --seed N       OMPL base seed (default: 40)\n"
        << "  --timeout SEC  Per-run timeout (default: 30)\n"
        << "  --help          Show this message\n";
}

RunOptions parseOptions(int argc, char **argv)
{
    RunOptions options;

    auto requireValue = [&](int &index, const std::string &option) -> std::string
    {
        if (index + 1 >= argc)
            throw std::invalid_argument("Missing value for " + option);
        return argv[++index];
    };

    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        if (arg == "--benchmark")
            options.benchmark = true;
        else if (arg == "--runs")
            options.runs = static_cast<unsigned int>(std::stoul(requireValue(i, arg)));
        else if (arg == "--warmup")
            options.warmupRuns = static_cast<unsigned int>(std::stoul(requireValue(i, arg)));
        else if (arg == "--samples")
            options.maxSamples = static_cast<unsigned int>(std::stoul(requireValue(i, arg)));
        else if (arg == "--seed")
            options.seed = static_cast<unsigned int>(std::stoul(requireValue(i, arg)));
        else if (arg == "--timeout")
            options.timeoutSeconds = std::stod(requireValue(i, arg));
        else if (arg == "--help" || arg == "-h")
            options.help = true;
        else
            throw std::invalid_argument("Unknown option: " + arg);
    }

    if (options.runs == 0u)
        throw std::invalid_argument("--runs must be greater than zero");
    if (options.maxSamples == 0u)
        throw std::invalid_argument("--samples must be greater than zero");
    if (!(options.timeoutSeconds > 0.0) || !std::isfinite(options.timeoutSeconds))
        throw std::invalid_argument("--timeout must be a finite positive number");

    return options;
}

PlanningInstance createPlanningInstance(unsigned int maxSamples)
{
    PlanningInstance instance;

    auto stateSpace = std::make_shared<ob::RealVectorStateSpace>(4);

    ob::RealVectorBounds stateBounds(4);
    stateBounds.setLow(0, kWorldXMin); stateBounds.setHigh(0, kWorldXMax);
    stateBounds.setLow(1, kWorldYMin); stateBounds.setHigh(1, kWorldYMax);
    stateBounds.setLow(2, kVelMin);    stateBounds.setHigh(2, kVelMax);
    stateBounds.setLow(3, kVelMin);    stateBounds.setHigh(3, kVelMax);
    stateSpace->setBounds(stateBounds);

    auto controlSpace = std::make_shared<oc::RealVectorControlSpace>(stateSpace, 2);

    ob::RealVectorBounds controlBounds(2);
    controlBounds.setLow(0, -8.0); controlBounds.setHigh(0, 8.0);
    controlBounds.setLow(1, -8.0); controlBounds.setHigh(1, 8.0);
    controlSpace->setBounds(controlBounds);

    instance.setup = std::make_unique<oc::SimpleSetup>(controlSpace);
    auto &ss = *instance.setup;
    ss.setStateValidityChecker(isStateValid);

    ss.getSpaceInformation()->setStatePropagator(
        std::make_shared<DoubleIntegratorPropagator>(ss.getSpaceInformation()));
    ss.getSpaceInformation()->setPropagationStepSize(0.05);
    ss.getSpaceInformation()->setMinMaxControlDuration(1, 20);

    g_circles.clear();
    g_rects.clear();

    g_circles.emplace_back( 6.0,  6.0, 3.0);
    g_circles.emplace_back(15.0, 14.0, 3.0);
    g_circles.emplace_back(10.0, 11.0, 2.0);
    g_circles.emplace_back( 5.0, 15.0, 3.5);
    g_circles.emplace_back(15.0,  5.0, 3.0);

    ob::ScopedState<> start(stateSpace);
    start[0] = kStartX; start[1] = kStartY; start[2] = 0.0; start[3] = 0.0;

    ss.setStartState(start);
    auto posGoal = std::make_shared<PositionOnlyGoal>(
        ss.getSpaceInformation(), kGoalX, kGoalY, kGoalTolerance);
    ss.setGoal(posGoal);

    ss.setOptimizationObjective(
        std::make_shared<ob::PathLengthOptimizationObjective>(ss.getSpaceInformation()));

    instance.planner = std::make_shared<oc::RRTstarDI>(ss.getSpaceInformation());
    auto &planner = instance.planner;
    planner->setKNearest(false);
    planner->setRewireFactor(3.0);
    planner->setRange(4.0);
    planner->setGoalBias(0.0);

    planner->setNumCollisionCheckPoints(50);
    planner->setTimeSolverTolerance(1e-5);
    planner->setTimeSolverWork(24, 6, 8);
    planner->setMinTrajectoryTime(0.001);
    planner->setMaxTrajectoryTime(5.0);
    planner->setControlDiscretization(20);
    planner->setMaxSampleCount(maxSamples);
    configureFastEnvironment(planner);

    ss.setPlanner(planner);
    ss.setup();

    return instance;
}

double percentile(const std::vector<double> &sortedValues, double fraction)
{
    const double rank = std::ceil(fraction * static_cast<double>(sortedValues.size()));
    const std::size_t index = static_cast<std::size_t>(std::max(1.0, rank)) - 1u;
    return sortedValues[std::min(index, sortedValues.size() - 1u)];
}

int runBenchmark(const RunOptions &options)
{
    cv::setNumThreads(1);
    ompl::msg::setLogLevel(ompl::msg::LOG_ERROR);

    std::cout << "Benchmark mode: planning only (no export, drawing, imwrite, imshow, or waitKey)\n";
    std::cout << "Primitive backend: "
              << (oc::detail::kHasAvx2Backend ? "AVX2 float8" : "scalar float8") << "\n";
    std::cout << "Runs=" << options.runs
              << ", warmup=" << options.warmupRuns
              << ", samples=" << options.maxSamples
              << ", seed=" << options.seed
              << ", timeout=" << options.timeoutSeconds << " s\n";

    std::vector<double> times;
    times.reserve(options.runs);
    unsigned int solvedRuns = 0u;
    unsigned int minAcceptedSamples = std::numeric_limits<unsigned int>::max();
    unsigned int maxAcceptedSamples = 0u;

    const unsigned int totalRuns = options.warmupRuns + options.runs;
    for (unsigned int run = 0; run < totalRuns; ++run)
    {
        PlanningInstance instance = createPlanningInstance(options.maxSamples);

        const auto startTime = std::chrono::steady_clock::now();
        const ob::PlannerStatus solved = instance.setup->solve(options.timeoutSeconds);
        const auto endTime = std::chrono::steady_clock::now();

        if (run < options.warmupRuns)
            continue;

        const double seconds = std::chrono::duration<double>(endTime - startTime).count();
        times.push_back(seconds);
        if (solved)
            ++solvedRuns;

        const unsigned int acceptedSamples = instance.planner->getSampleCount();
        minAcceptedSamples = std::min(minAcceptedSamples, acceptedSamples);
        maxAcceptedSamples = std::max(maxAcceptedSamples, acceptedSamples);
    }

    std::vector<double> sortedTimes = times;
    std::sort(sortedTimes.begin(), sortedTimes.end());
    const double totalSeconds = std::accumulate(times.begin(), times.end(), 0.0);
    const double meanSeconds = totalSeconds / static_cast<double>(times.size());

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "Solved: " << solvedRuns << "/" << options.runs << "\n";
    std::cout << "Accepted samples min/max: "
              << minAcceptedSamples << "/" << maxAcceptedSamples << "\n";
    std::cout << "Planning seconds mean:  " << meanSeconds << "\n";
    std::cout << "Planning seconds median:" << percentile(sortedTimes, 0.50) << "\n";
    std::cout << "Planning seconds p95:   " << percentile(sortedTimes, 0.95) << "\n";
    std::cout << "Planning seconds min:   " << sortedTimes.front() << "\n";
    std::cout << "Planning seconds max:   " << sortedTimes.back() << "\n";
    std::cout << "Measured planning total:" << totalSeconds << "\n";

    if (minAcceptedSamples != options.maxSamples || maxAcceptedSamples != options.maxSamples)
    {
        std::cerr << "Benchmark incomplete: at least one run did not reach --samples.\n";
        return 2;
    }

    return solvedRuns == options.runs ? 0 : 3;
}

int runNormal(const RunOptions &options)
{
    PlanningInstance instance = createPlanningInstance(options.maxSamples);
    auto &ss = *instance.setup;
    auto &planner = instance.planner;

    std::cout << "Planning..." << std::endl;

    auto t0 = std::chrono::steady_clock::now();
    ob::PlannerStatus solved = ss.solve(options.timeoutSeconds);
    auto t1 = std::chrono::steady_clock::now();

    const double seconds = std::chrono::duration<double>(t1 - t0).count();
    std::cout << "Planning elapsed time: " << seconds << " s\n";

    if (!solved)
    {
        std::cout << "No solution found.\n";
        return 0;
    }

    std::cout << "Found solution.\n";

    auto &pathBase = ss.getSolutionPath();
    auto *path = dynamic_cast<oc::PathControl *>(&pathBase);
    if (!path)
    {
        std::cerr << "Solution is not a PathControl.\n";
        return 1;
    }

    std::cout << "State count:   " << path->getStateCount() << "\n";
    std::cout << "Control count: " << path->getControlCount() << "\n";

    ob::PlannerData data(ss.getSpaceInformation());
    ss.getPlanner()->getPlannerData(data);

    // exportTreeToText(data, "tree.txt");
    // exportOptimalPath(*path, "optimal_path.txt");
    // exportDynamicTrajectory(*path, "detailed_trajectory.txt");

    drawTreeAndTrajectory(data, *path, planner.get(), "tree.png",
                          kWorldXMin, kWorldXMax, kWorldYMin, kWorldYMax,
                          kStartX, kStartY,
                          kGoalX, kGoalY);

    return 0;
}

int main(int argc, char **argv)
{
    try
    {
        const RunOptions options = parseOptions(argc, argv);
        if (options.help)
        {
            printUsage(argv[0]);
            return 0;
        }

        ompl::RNG::setSeed(options.seed);
        return options.benchmark ? runBenchmark(options) : runNormal(options);
    }
    catch (const std::exception &error)
    {
        std::cerr << "Error: " << error.what() << "\n";
        printUsage(argv[0]);
        return 1;
    }
}
