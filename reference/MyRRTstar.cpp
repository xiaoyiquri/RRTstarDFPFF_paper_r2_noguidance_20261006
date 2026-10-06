/* Authors: Alejandro Perez, Sertac Karaman, Ryan Luna, Luis G. Torres, Ioan Sucan, Javier V Gomez, Jonathan Gammell */

#include "MyRRTstar.h"
#include <algorithm>
#include <boost/math/constants/constants.hpp>
#include <array>
#include <cstddef>
#include <limits>
#include <vector>
#include <chrono>
#if defined(__AVX2__)
#include <immintrin.h>
#endif
#include "ompl/base/Goal.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/base/goals/GoalState.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/base/samplers/InformedStateSampler.h"
#include "ompl/base/samplers/informed/RejectionInfSampler.h"
#include "ompl/base/samplers/informed/OrderedInfSampler.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/util/GeometricEquations.h"
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <cmath>
#include "DICostUtils.h"
#include "di_primitive_helpers.h"

//淇敼涓�?
ompl::control::RRTstarDI::RRTstarDI(const SpaceInformationPtr &si)
  : base::Planner(si, "RRTstarDI"), siC_(si)
{
	//淇敼浜岋細 
	if (!siC_)
    throw ompl::Exception("RRTstarDI requires control::SpaceInformation");
    
    specs_.approximateSolutions = true;//鏀寔杩戜技�?    specs_.optimizingPaths = true;//鍙紭鍖栬矾�?    specs_.canReportIntermediateSolutions = true;//鑳芥姤鍛婁腑闂磋В

      //澹版槑涓€绯诲垪鍙厤缃弬鏁帮紝骞剁粦�?getter/setter锛屼究浜庡閮紙閰嶇疆鏂囦�?鍛戒护琛?UI锛変慨鏀?planner 琛屼负銆?    //Planner::declareParam<double>("range", this, &RRTstarDI::setRange, &RRTstarDI::getRange, "0.:1.:10000.");
    Planner::declareParam<double>("range", this, &RRTstarDI::setRange, &RRTstarDI::getRange, "0.:1.:10000.");
    Planner::declareParam<double>("goal_bias", this, &RRTstarDI::setGoalBias, &RRTstarDI::getGoalBias, "0.:.05:1.");
    Planner::declareParam<double>("rewire_factor", this, &RRTstarDI::setRewireFactor, &RRTstarDI::getRewireFactor,
                                  "1.0:0.01:2.0");
    Planner::declareParam<bool>("use_k_nearest", this, &RRTstarDI::setKNearest, &RRTstarDI::getKNearest, "0,1");
    Planner::declareParam<bool>("delay_collision_checking", this, &RRTstarDI::setDelayCC, &RRTstarDI::getDelayCC, "0,1");
    Planner::declareParam<bool>("tree_pruning", this, &RRTstarDI::setTreePruning, &RRTstarDI::getTreePruning, "0,1");
    Planner::declareParam<double>("prune_threshold", this, &RRTstarDI::setPruneThreshold, &RRTstarDI::getPruneThreshold,
                                  "0.:.01:1.");
    Planner::declareParam<bool>("pruned_measure", this, &RRTstarDI::setPrunedMeasure, &RRTstarDI::getPrunedMeasure, "0,1");
    Planner::declareParam<bool>("informed_sampling", this, &RRTstarDI::setInformedSampling, &RRTstarDI::getInformedSampling,
                                "0,1");
    Planner::declareParam<bool>("sample_rejection", this, &RRTstarDI::setSampleRejection, &RRTstarDI::getSampleRejection,
                                "0,1");
    Planner::declareParam<bool>("new_state_rejection", this, &RRTstarDI::setNewStateRejection,
                                &RRTstarDI::getNewStateRejection, "0,1");
    Planner::declareParam<bool>("use_admissible_heuristic", this, &RRTstarDI::setAdmissibleCostToCome,
                                &RRTstarDI::getAdmissibleCostToCome, "0,1");
    Planner::declareParam<bool>("ordered_sampling", this, &RRTstarDI::setOrderedSampling, &RRTstarDI::getOrderedSampling,
                                "0,1");
    Planner::declareParam<unsigned int>("ordering_batch_size", this, &RRTstarDI::setBatchSize, &RRTstarDI::getBatchSize,
                                        "1:100:1000000");
    Planner::declareParam<bool>("focus_search", this, &RRTstarDI::setFocusSearch, &RRTstarDI::getFocusSearch, "0,1");
    Planner::declareParam<unsigned int>("number_sampling_attempts", this, &RRTstarDI::setNumSamplingAttempts,
                                        &RRTstarDI::getNumSamplingAttempts, "10:10:100000");
    Planner::declareParam<unsigned int>("num_collision_check_points", this, &RRTstarDI::setNumCollisionCheckPoints, &RRTstarDI::getNumCollisionCheckPoints, "2:1:1000");
	Planner::declareParam<double>("min_trajectory_time", this, &RRTstarDI::setMinTrajectoryTime, &RRTstarDI::getMinTrajectoryTime, "0.001:0.001:100");
	Planner::declareParam<double>("max_trajectory_time", this, &RRTstarDI::setMaxTrajectoryTime, &RRTstarDI::getMaxTrajectoryTime, "0.01:0.01:1000");
	Planner::declareParam<unsigned int>("control_discretization", this, &RRTstarDI::setControlDiscretization, &RRTstarDI::getControlDiscretization, "1:1:1000");

    addPlannerProgressProperty("iterations INTEGER", [this] { return numIterationsProperty(); });
    addPlannerProgressProperty("best cost REAL", [this] { return bestCostProperty(); });
}

//淇敼浜岋細 
ompl::control::RRTstarDI::~RRTstarDI()
{
    freeMemory();
}
void ompl::control::RRTstarDI::setup()
{
    Planner::setup();

    auto *rcs = dynamic_cast<ompl::control::RealVectorControlSpace *>(siC_->getControlSpace().get());
    if (!rcs || rcs->getDimension() != 2)
    {
        throw ompl::Exception("RRTstarDI requires a 2D RealVectorControlSpace");
    }

    auto *rss = dynamic_cast<ompl::base::RealVectorStateSpace *>(si_->getStateSpace().get());
    if (!rss || rss->getDimension() != 4)
    {
        throw ompl::Exception("RRTstarDI requires a 4D RealVectorStateSpace [x,y,vx,vy]");
    }

    if (!siC_->getStatePropagator())
    {
        throw ompl::Exception("RRTstarDI requires a state propagator");
    }

    const char *primitiveBackend =
        detail::kHasAvx2Backend ? "AVX2 float8" : "scalar float8";
    OMPL_INFORM("%s: Primitive SIMD backend: %s", getName().c_str(), primitiveBackend);
    OMPL_INFORM("%s: XY nearest-neighbor backend: SoA %s",
                getName().c_str(), detail::kHasAvx2Backend ? "AVX2 (8 floats)" : "scalar float");

    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);

    if (!si_->getStateSpace()->hasSymmetricDistance() || !si_->getStateSpace()->hasSymmetricInterpolate())
    {
        OMPL_WARN("%s requires a state space with symmetric distance and symmetric interpolation.", getName().c_str());
    }

    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
        // =========================銆愪慨鏀?b�?========================
        //淇敼鍑芥暟璋冪�?
    nn_->setDistanceFunction([this](const Motion *a, const Motion *b) { return motionDistanceXY(a, b); });

    if (pdef_)
	{
	    if (!pdef_->hasOptimizationObjective())
	        throw ompl::Exception("RRTstarDI requires an external OptimizationObjective.");
	
	    opt_ = pdef_->getOptimizationObjective();
	    bestCost_ = opt_->infiniteCost();
	    prunedCost_ = opt_->infiniteCost();
	}
	else
	{
	    OMPL_INFORM("%s: problem definition is not set, deferring setup completion...", getName().c_str());
	    setup_ = false;
	}

//===================================================銆愮粓鐐圭洿杩炴鏌ャ€?========================================
     prunedMeasure_ = getXYMeasure();
    //prunedMeasure_ = si_->getSpaceMeasure();
    //===================================================銆愮粓鐐圭洿杩炴鏌ャ€?========================================
    calculateRewiringLowerBounds();
}

void ompl::control::RRTstarDI::clear()
{
    setup_ = false;
    Planner::clear();
    sampler_.reset();
    infSampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    motionCount_ = 0;

    bestGoalMotion_ = nullptr;
    goalMotions_.clear();
    improvingGoalCount_ = 0;
    startMotions_.clear();

    iterations_ = 0;
    bestCost_ = base::Cost(std::numeric_limits<double>::quiet_NaN());
    prunedCost_ = base::Cost(std::numeric_limits<double>::quiet_NaN());
    prunedMeasure_ = 0.0;
}

//PFF cost���㹫ʽ 
double ompl::control::RRTstarDI::DI_costFreeVel(
    double t_s,
    double x01, double x02, double x03, double x04,
    double x11, double x12) const
{
    return t_s
        + (3*x03*x03 + 3*x04*x04) / t_s
        + (3*x01*x01 - 6*x01*x11 + 3*x02*x02 - 6*x02*x12
           + 3*x11*x11 + 3*x12*x12) / (t_s*t_s*t_s)
        + (6*x01*x03 + 6*x02*x04 - 6*x03*x11 - 6*x04*x12) / (t_s*t_s);
}

//ֱ�����ĳһʱ�̵� (x,y,vx,vy)
void ompl::control::RRTstarDI::DI_stateFreeVel_out(
    double t, double t_s,
    double x01, double x02, double x03, double x04,
    double x11, double x12,
    double &x, double &y, double &vx, double &vy) const
{
    DIEdgePrimitive primitive;
    prepareEdgePrimitive(t_s, x01, x02, x03, x04, x11, x12, primitive);
    evaluateEdgeState(primitive, t, x, y, vx, vy);
}
//��������ʱ�� t_opt ����ֹ״̬��λ�á��ٶȣ�Ԥ����һ�����ζ���ʽ�켣�ġ�ԭʼ���ݡ�
//��DIEdgePrimitive �ṹ�壩
//����״̬����Ϳ��Ƽ��㶼������Щ���ݣ������ظ�����
void ompl::control::RRTstarDI::prepareEdgePrimitive(
    double t_opt,
    double x01, double x02, double x03, double x04,
    double x11, double x12,
    DIEdgePrimitive &primitive) const
{
    primitive.mode = EdgeMode::FreeVel;
    primitive.x0 = x01;
    primitive.y0 = x02;
    primitive.vx0 = x03;
    primitive.vy0 = x04;
    primitive.tOpt = t_opt;
    const double invTwoTs3 = 0.5 / (t_opt * t_opt * t_opt);
    primitive.x3 = (x01 - x11 + t_opt * x03) * invTwoTs3;
    primitive.y3 = (x02 - x12 + t_opt * x04) * invTwoTs3;
    primitive.x2 = -3.0 * t_opt * primitive.x3;
    primitive.y2 = -3.0 * t_opt * primitive.y3;
}

void ompl::control::RRTstarDI::prepareEdgePrimitiveFixedVel(
    double t_opt,
    const DIState &fromState,
    const DIState &targetState,
    DIEdgePrimitive &primitive) const
{
    primitive.mode = EdgeMode::FixedVel;
    primitive.x0 = fromState.x;
    primitive.y0 = fromState.y;
    primitive.vx0 = fromState.vx;
    primitive.vy0 = fromState.vy;
    primitive.tOpt = t_opt;

    const double invT = 1.0 / t_opt;
    const double invT2 = invT * invT;
    const double invT3 = invT2 * invT;
    const double dx = targetState.x - fromState.x;
    const double dy = targetState.y - fromState.y;

    primitive.x2 = 3.0 * dx * invT2 -
                   (2.0 * fromState.vx + targetState.vx) * invT;
    primitive.y2 = 3.0 * dy * invT2 -
                   (2.0 * fromState.vy + targetState.vy) * invT;
    primitive.x3 = -2.0 * dx * invT3 +
                   (fromState.vx + targetState.vx) * invT2;
    primitive.y3 = -2.0 * dy * invT3 +
                   (fromState.vy + targetState.vy) * invT2;
}

//����켣��ʱ�� t ��λ�ú��ٶ� 
void ompl::control::RRTstarDI::evaluateEdgeState(
    const DIEdgePrimitive &primitive,
    double t,
    double &x, double &y,
    double &vx, double &vy) const
{
    const double t2 = t * t;
    const double t3 = t2 * t;
    x = primitive.x0 + primitive.vx0 * t + primitive.x2 * t2 + primitive.x3 * t3;
    y = primitive.y0 + primitive.vy0 * t + primitive.y2 * t2 + primitive.y3 * t3;
    vx = primitive.vx0 + 2.0 * primitive.x2 * t + 3.0 * primitive.x3 * t2;
    vy = primitive.vy0 + 2.0 * primitive.y2 * t + 3.0 * primitive.y3 * t2;
}

//����ʱ�� t �Ŀ�������Ҳ���Ǽ��ٶ�
void ompl::control::RRTstarDI::evaluateEdgeControl(
    const DIEdgePrimitive &primitive,
    double t,
    double &ux,
    double &uy) const
{
    ux = 2.0 * primitive.x2 + 6.0 * primitive.x3 * t;
    uy = 2.0 * primitive.y2 + 6.0 * primitive.y3 * t;
}
// ============================================================================
// ============================================================================
// �ȼ��ʱ�䷶Χ��Ȼ�����validateMotionPrimitive
// ============================================================================
bool ompl::control::RRTstarDI::checkDynamicMotionWithTimeFast(
    const base::State *s1,
    const base::State *s2,
    double t_opt,
    base::State *fallbackScratch) const
{
    const DIState start = toDIState(s1);
    const DIState goal = toDIState(s2);
    DIEdgePrimitive primitive;
    prepareEdgePrimitive(t_opt, start.x, start.y, start.vx, start.vy,
                         goal.x, goal.y, primitive);
    return checkFreeVelMotionWithTime(primitive, start,
                                      evaluateEdgeState(primitive, t_opt),
                                      fallbackScratch);
}

bool ompl::control::RRTstarDI::checkFreeVelMotionWithTime(
    const DIEdgePrimitive &primitive,
    const DIState &start,
    const DIState &goal,
    base::State *fallbackScratch) const
{
    return primitive.mode == EdgeMode::FreeVel &&
           validatePreparedMotionPrimitive(primitive, start, goal, fallbackScratch);
}

bool ompl::control::RRTstarDI::checkFixedVelMotionWithTime(
    const DIEdgePrimitive &primitive,
    const DIState &start,
    const DIState &goal,
    base::State *fallbackScratch) const
{
    return primitive.mode == EdgeMode::FixedVel &&
           validatePreparedMotionPrimitive(primitive, start, goal, fallbackScratch);
}

std::vector<double> ompl::control::RRTstarDI::DI_stateFreeVel(
    double t, double t_s,
    double x01, double x02, double x03, double x04,
    double x11, double x12) const
{
    std::vector<double> states(4);

    double x, y, vx, vy;
    DI_stateFreeVel_out(t, t_s, x01, x02, x03, x04, x11, x12, x, y, vx, vy);

    states[0] = x;
    states[1] = y;
    states[2] = vx;
    states[3] = vy;
    return states;
}

// ============================================================================
// ���� DI_costFreeVel ��ʱ�� T �ĵ���������һ���Ĵζ���ʽ��ϵ�� 
// ============================================================================
auto ompl::control::RRTstarDI::makeTimeFreeVelCoefficients(
    double x01, double x02, double x03, double x04,
    double x11, double x12) const -> QuarticCoefficients
{
    return QuarticCoefficients{
        1.0,
        0.0,
        -3.0 * x03 * x03 - 3.0 * x04 * x04,
        12.0 * x03 * x11 - 12.0 * x02 * x04 - 12.0 * x01 * x03 + 12.0 * x04 * x12,
        -9.0 * x01 * x01 + 18.0 * x01 * x11 - 9.0 * x02 * x02 + 18.0 * x02 * x12 -
            9.0 * x11 * x11 - 9.0 * x12 * x12};
}

auto ompl::control::RRTstarDI::makeTimeFixedVelCoefficients(
    const DIState &fromState,
    const DIState &targetState) const -> QuarticCoefficients
{
    const double x01 = fromState.x;
    const double x02 = fromState.y;
    const double x03 = fromState.vx;
    const double x04 = fromState.vy;
    const double x11 = targetState.x;
    const double x12 = targetState.y;
    const double x13 = targetState.vx;
    const double x14 = targetState.vy;

    return QuarticCoefficients{
        1.0,
        0.0,
        -4.0 * x03 * x03 - 4.0 * x03 * x13 - 4.0 * x04 * x04 -
            4.0 * x04 * x14 - 4.0 * x13 * x13 - 4.0 * x14 * x14,
        24.0 * x03 * x11 - 24.0 * x02 * x04 - 24.0 * x01 * x13 -
            24.0 * x01 * x03 - 24.0 * x02 * x14 + 24.0 * x04 * x12 +
            24.0 * x11 * x13 + 24.0 * x12 * x14,
        -36.0 * x01 * x01 + 72.0 * x01 * x11 - 36.0 * x02 * x02 +
            72.0 * x02 * x12 - 36.0 * x11 * x11 - 36.0 * x12 * x12};
}

double ompl::control::RRTstarDI::DI_costFixedVel(
    double t,
    const DIState &fromState,
    const DIState &targetState) const
{
    const double invT = 1.0 / t;
    const double invT2 = invT * invT;
    const double invT3 = invT2 * invT;
    const double x01 = fromState.x;
    const double x02 = fromState.y;
    const double x03 = fromState.vx;
    const double x04 = fromState.vy;
    const double x11 = targetState.x;
    const double x12 = targetState.y;
    const double x13 = targetState.vx;
    const double x14 = targetState.vy;

    return t +
           4.0 * (x03 * x03 + x04 * x04 + x13 * x13 + x14 * x14) * invT +
           12.0 * (x01 * x01 + x02 * x02 + x11 * x11 + x12 * x12) * invT3 +
           12.0 * (x01 * x03 + x02 * x04 + x01 * x13 + x02 * x14 -
                   x03 * x11 - x04 * x12 - x11 * x13 - x12 * x14) * invT2 +
           4.0 * (x03 * x13 + x04 * x14) * invT -
           24.0 * (x01 * x11 + x02 * x12) * invT3;
}

// 銆愭柊澧炲嚱�?4 
// ============================================================================
void ompl::control::RRTstarDI::extractStateComponents(
    const base::State *s,
    double &x, double &y,
    double &vx, double &vy) const
{
    const auto *sv = s->as<base::RealVectorStateSpace::StateType>();
    x  = sv->values[0];
    y  = sv->values[1];
    vx = sv->values[2];
    vy = sv->values[3];
}

// ============================================================================
// 銆愭柊澧炲嚱�?5 
// ============================================================================
void ompl::control::RRTstarDI::setStateComponents(
    base::State *s,
    double x, double y,
    double vx, double vy) const
{
    auto *sv = s->as<base::RealVectorStateSpace::StateType>();
    sv->values[0] = x;
    sv->values[1] = y;
    sv->values[2] = vx;
    sv->values[3] = vy;
}


//���ǵ�ǰ�����������ĺ���ʱ���Ż����� 
/*
computeBestTimeFreeVelFromDIStates
    ������ makeTimeFreeVelCoefficients
    ������ ���쵼������ʽ derivativePolynomial(T)
    ������ �����Сʱ�� lo �����ʱ�� hi
    ������ �� lo��hi ��Ϊ��ѡʱ��
    ������ ��ʱ��������ɨ��
    ������ ���ֵ������ű仯
    ��      ������ Newton ���� + ���ַ� refineRoot
    ������ �Ժ�ѡʱ����� DI_costFreeVel
    ������ ѡ�������С�� tOpt �� bestCost
*/ 
bool ompl::control::RRTstarDI::computeBestTimeFromQuartic(
    const QuarticCoefficients &coeffs,
    const std::function<double(double)> &costAtTime,
    double &tOpt,
    double &bestCost) const
{
    const double lo = std::max(minTrajectoryTime_, 1e-9);
    const double hi = maxTrajectoryTime_;
    if (!(lo > 0.0) || !(hi >= lo) || !std::isfinite(lo) || !std::isfinite(hi))
        return false;

    auto derivativePolynomial = [&](double t)
    {
        return (((coeffs[0] * t + coeffs[1]) * t + coeffs[2]) * t + coeffs[3]) * t + coeffs[4];
    };

    auto derivativePolynomialPrime = [&](double t)
    {
        return (4.0 * coeffs[0] * t * t * t) +
               (3.0 * coeffs[1] * t * t) +
               (2.0 * coeffs[2] * t) + coeffs[3];
    };

    tOpt = -1.0;
    bestCost = std::numeric_limits<double>::infinity();

    auto considerCandidate = [&](double t)
    {
        if (!std::isfinite(t) || t < lo || t > hi)
            return;

        const double cost = costAtTime(t);
        if (std::isfinite(cost) && cost < bestCost)
        {
            bestCost = cost;
            tOpt = t;
        }
    };

    considerCandidate(lo);
    considerCandidate(hi);

    const double tol = std::max(1e-12, timeSolverTolerance_);
    auto refineRoot = [&](double left, double right, double fLeft, unsigned int maxBisectionIters)
    {
        double guess = 0.5 * (left + right);
        for (unsigned int iter = 0; iter < rootNewtonIterations_; ++iter)
        {
            const double fGuess = derivativePolynomial(guess);
            const double dfGuess = derivativePolynomialPrime(guess);
            const double mid = 0.5 * (left + right);
            double candidate = mid;
            if (std::isfinite(fGuess) && std::isfinite(dfGuess) && std::abs(dfGuess) > 1e-12)
            {
                const double newton = guess - fGuess / dfGuess;
                if (newton > left && newton < right)
                    candidate = newton;
            }

            const double fCandidate = derivativePolynomial(candidate);
            if (!std::isfinite(fCandidate))
                break;

            if ((fLeft < 0.0 && fCandidate > 0.0) ||
                (fLeft > 0.0 && fCandidate < 0.0))
            {
                right = candidate;
            }
            else
            {
                left = candidate;
                fLeft = fCandidate;
            }
            guess = candidate;
        }

        unsigned int bisectionIters = 0;
        if ((right - left) > tol)
        {
            bisectionIters = static_cast<unsigned int>(
                std::ceil(std::log2((right - left) / tol)));
            bisectionIters = std::min(maxBisectionIters, bisectionIters);
        }

        for (unsigned int iter = 0; iter < bisectionIters && (right - left) > tol; ++iter)
        {
            const double mid = 0.5 * (left + right);
            const double fMid = derivativePolynomial(mid);
            if (!std::isfinite(fMid))
                break;

            if ((fLeft < 0.0 && fMid > 0.0) ||
                (fLeft > 0.0 && fMid < 0.0))
            {
                right = mid;
            }
            else
            {
                left = mid;
                fLeft = fMid;
            }
        }

        return 0.5 * (left + right);
    };

    const double qLo = derivativePolynomial(lo);

    const unsigned int rootScanIntervals = std::max(4u, rootScanIntervals_);
    const bool useLogSpacing = std::isfinite(hi / lo) && (hi / lo > 4.0);
    const double ratio = useLogSpacing
                             ? std::exp(std::log(hi / lo) / static_cast<double>(rootScanIntervals))
                             : 1.0;

    double prevT = lo;
    double prevQ = qLo;
    double currT = lo;

    if (std::isfinite(prevQ) && std::abs(prevQ) <= 1e-12)
        considerCandidate(prevT);

    for (unsigned int i = 1; i <= rootScanIntervals; ++i)
    {
        currT = useLogSpacing
                    ? (i == rootScanIntervals ? hi : currT * ratio)
                    : lo + (hi - lo) * static_cast<double>(i) /
                               static_cast<double>(rootScanIntervals);

        const double currQ = derivativePolynomial(currT);
        if (!std::isfinite(prevQ) || !std::isfinite(currQ))
        {
            prevT = currT;
            prevQ = currQ;
            continue;
        }

        if (std::abs(currQ) <= 1e-12)
        {
            considerCandidate(currT);
        }
        else if ((prevQ < 0.0 && currQ > 0.0) ||
                 (prevQ > 0.0 && currQ < 0.0))
        {
            considerCandidate(refineRoot(prevT, currT, prevQ, rootBisectionIterations_));
        }

        prevT = currT;
        prevQ = currQ;
    }

    return std::isfinite(tOpt) && std::isfinite(bestCost);
}

bool ompl::control::RRTstarDI::computeBestTimeFreeVelFromDIStates(
    const DIState &fromState,
    const DIState &targetState,
    double &tOpt,
    double &bestCost) const
{
    const QuarticCoefficients coeffs = makeTimeFreeVelCoefficients(
        fromState.x, fromState.y, fromState.vx, fromState.vy,
        targetState.x, targetState.y);
    return computeBestTimeFromQuartic(
        coeffs,
        [&](double t)
        {
            return DI_costFreeVel(
                t, fromState.x, fromState.y, fromState.vx, fromState.vy,
                targetState.x, targetState.y);
        },
        tOpt, bestCost);
}

bool ompl::control::RRTstarDI::computeBestTimeFixedVelFromDIStates(
    const DIState &fromState,
    const DIState &targetState,
    double &tOpt,
    double &bestCost) const
{
    const QuarticCoefficients coeffs =
        makeTimeFixedVelCoefficients(fromState, targetState);
    return computeBestTimeFromQuartic(
        coeffs,
        [&](double t) { return DI_costFixedVel(t, fromState, targetState); },
        tOpt, bestCost);
}

//�����ǰ� OMPL �� base::State תΪ DIState 
bool ompl::control::RRTstarDI::computeOptimalTimeFreeVel(
    const base::State *s1,
    const base::State *s2,
    double &t_opt) const
{
    double bestCost = std::numeric_limits<double>::infinity();
    if (!computeBestTimeFreeVelFromDIStates(toDIState(s1), toDIState(s2), t_opt, bestCost))
        return false;

    return true;
}

bool ompl::control::RRTstarDI::isControlSequenceWithinBounds(
    const std::vector<ControlSegment> &segments) const
{
    const auto *rcs =
        dynamic_cast<const ompl::control::RealVectorControlSpace *>(siC_->getControlSpace().get());
    if (!rcs)
        return false;

    const auto &cb = rcs->getBounds();
    const unsigned int dim = rcs->getDimension();

    for (const auto &seg : segments)
    {
        if (!seg.control)
            return false;
        if (!(seg.duration > 0.0) || !std::isfinite(seg.duration))
            return false;

        const auto *u = seg.control->as<ompl::control::RealVectorControlSpace::ControlType>();
        for (unsigned int k = 0; k < dim; ++k)
        {
            if (!std::isfinite(u->values[k]))
                return false;
            if (u->values[k] < cb.low[k] || u->values[k] > cb.high[k])
                return false;
        }
    }

    return true;
}

bool ompl::control::RRTstarDI::isEdgeControlWithinBounds(
    const DIEdgePrimitive &primitive) const
{
    if (!(primitive.tOpt > 0.0) || !std::isfinite(primitive.tOpt))
        return false;

    const auto *rcs =
        dynamic_cast<const ompl::control::RealVectorControlSpace *>(siC_->getControlSpace().get());
    if (!rcs || rcs->getDimension() != 2)
        return false;

    const auto &cb = rcs->getBounds();

    auto checkControlAt = [&](double t)
    {
        double ux = 0.0;
        double uy = 0.0;
        evaluateEdgeControl(primitive, t, ux, uy);

        if (!std::isfinite(ux) || !std::isfinite(uy))
            return false;
        if (ux < cb.low[0] || ux > cb.high[0] ||
            uy < cb.low[1] || uy > cb.high[1])
            return false;

        return true;
    };

    return checkControlAt(0.0) && checkControlAt(primitive.tOpt);
}


// ============================================================================
// �����״̬ s1 ��״̬ s2 ����С��̬����
// ============================================================================
ompl::base::Cost ompl::control::RRTstarDI::dynamicMotionCost(
    const base::State *s1,
    const base::State *s2) const
{
    EdgeEval eval;
    if (!solveFreeVelEdge(s1, s2, eval, nullptr))
        return base::Cost(std::numeric_limits<double>::infinity());
    return eval.incCost;
}


// ============================================================================
// �����ġ�����ʱ�� + �Ϸ��ԡ��ӿ�
// ============================================================================
bool ompl::control::RRTstarDI::isDynamicMotionValid(
    const base::State *s1,
    const base::State *s2) const
{
    EdgeEval eval;
    if (!solveFreeVelEdge(s1, s2, eval, nullptr))
        return false;

    base::State *scratch = si_->allocState();
    const bool ok = checkFreeVelMotionWithTime(eval.primitive,
                                               toDIState(s1), eval.terminal, scratch);
    si_->freeState(scratch);
    return ok;
}
//���� RRT* ���ӱ�ʱ����Ҫ�ӿ�   
	// computeOptimalTimeFreeVel + DI_stateFreeVel_out + dynamicMotionCost
bool ompl::control::RRTstarDI::solveFreeVelEdge(
    const base::State *from,
    const base::State *targetRef,
    EdgeEval &eval,
    base::State *outState) const
{
    const DIState fromState = toDIState(from);
    const DIState targetState = toDIState(targetRef);
    return solveFreeVelEdgeFromDIStates(fromState, targetState, eval, outState);
}

/*
�����ͬʱ�õ���
1. ����ʱ�䣻
2. �����ն�״̬��
3. ���������ۣ�
4. �켣ԭ�
*/
bool ompl::control::RRTstarDI::solveFreeVelEdgeFromDIStates(
    const DIState &fromState,
    const DIState &targetState,
    EdgeEval &eval,
    base::State *outState) const
{
    eval = EdgeEval{};

    double t_opt = -1.0;
    double bestCost = std::numeric_limits<double>::infinity();
    if (!computeBestTimeFreeVelFromDIStates(fromState, targetState, t_opt, bestCost))
        return false;

    prepareEdgePrimitive(t_opt, fromState.x, fromState.y, fromState.vx, fromState.vy, targetState.x, targetState.y, eval.primitive);
    eval.terminal = evaluateEdgeState(eval.primitive, t_opt);
    if (outState)
        fromDIState(outState, eval.terminal);

    eval.ok = true;
    eval.t_opt = t_opt;
    eval.incCost = base::Cost(bestCost);

    return true;
}

bool ompl::control::RRTstarDI::solveFixedVelEdgeFromDIStates(
    const DIState &fromState,
    const DIState &targetState,
    EdgeEval &eval,
    base::State *outState) const
{
    eval = EdgeEval{};

    double tOpt = -1.0;
    double bestCost = std::numeric_limits<double>::infinity();
    if (!computeBestTimeFixedVelFromDIStates(fromState, targetState, tOpt, bestCost))
        return false;

    prepareEdgePrimitiveFixedVel(tOpt, fromState, targetState, eval.primitive);
    eval.terminal = targetState;
    if (outState)
        fromDIState(outState, targetState);
    eval.ok = true;
    eval.t_opt = tOpt;
    eval.incCost = base::Cost(bestCost);
    return true;
}

void ompl::control::RRTstarDI::evaluateRewireCandidateBlock(
    const base::State *from,
    Motion *const *targets,
    std::size_t count,
    EdgeEvalBlock &block) const
{
    block.count = std::min<std::size_t>(count, PrimitiveBatchWidth);
    const DIState fromState = toDIState(from);

    for (std::size_t lane = 0; lane < block.count; ++lane)
    {
        block.ok[lane] = false;
        EdgeEval eval;
        if (!solveFixedVelEdgeFromDIStates(
                fromState, targets[lane]->cache, eval, nullptr))
            continue;

        block.ok[lane] = true;
        block.tOpt[lane] = eval.t_opt;
        block.incCost[lane] = eval.incCost.value();
        block.terminal.x[lane] = eval.terminal.x;
        block.terminal.y[lane] = eval.terminal.y;
        block.terminal.vx[lane] = eval.terminal.vx;
        block.terminal.vy[lane] = eval.terminal.vy;
        block.primitive[lane] = eval.primitive;
    }
}

//���ڸ��ڵ��ѡ�� rewiring ��ѡ�� 8 ·��������
void ompl::control::RRTstarDI::evaluateEdgeBasicBlock8(
    const std::array<DIState, PrimitiveBatchWidth> &fromStates,
    const std::array<DIState, PrimitiveBatchWidth> &targetStates,
    std::size_t count,
    EdgeEvalBlock &block,
    std::array<base::State *, PrimitiveBatchWidth> &outStates) const
{
    block.count = std::min<std::size_t>(count, PrimitiveBatchWidth);
    for (std::size_t i = 0; i < block.count; ++i)
        block.ok[i] = false;

    if (block.count == 0)
        return;

#if defined(__AVX2__)
    if (block.count == PrimitiveBatchWidth)
    {
        auto set8 = [](double v0, double v1, double v2, double v3,
                       double v4, double v5, double v6, double v7)
        {
            return _mm256_set_ps(static_cast<float>(v7), static_cast<float>(v6),
                                 static_cast<float>(v5), static_cast<float>(v4),
                                 static_cast<float>(v3), static_cast<float>(v2),
                                 static_cast<float>(v1), static_cast<float>(v0));
        };

        const __m256 x0 = set8(fromStates[0].x, fromStates[1].x, fromStates[2].x, fromStates[3].x,
                               fromStates[4].x, fromStates[5].x, fromStates[6].x, fromStates[7].x);
        const __m256 y0 = set8(fromStates[0].y, fromStates[1].y, fromStates[2].y, fromStates[3].y,
                               fromStates[4].y, fromStates[5].y, fromStates[6].y, fromStates[7].y);
        const __m256 vx0 = set8(fromStates[0].vx, fromStates[1].vx, fromStates[2].vx, fromStates[3].vx,
                                fromStates[4].vx, fromStates[5].vx, fromStates[6].vx, fromStates[7].vx);
        const __m256 vy0 = set8(fromStates[0].vy, fromStates[1].vy, fromStates[2].vy, fromStates[3].vy,
                                fromStates[4].vy, fromStates[5].vy, fromStates[6].vy, fromStates[7].vy);
        const __m256 gx = set8(targetStates[0].x, targetStates[1].x, targetStates[2].x, targetStates[3].x,
                               targetStates[4].x, targetStates[5].x, targetStates[6].x, targetStates[7].x);
        const __m256 gy = set8(targetStates[0].y, targetStates[1].y, targetStates[2].y, targetStates[3].y,
                               targetStates[4].y, targetStates[5].y, targetStates[6].y, targetStates[7].y);

        const __m256 dx = _mm256_sub_ps(x0, gx);
        const __m256 dy = _mm256_sub_ps(y0, gy);
        const __m256 v2 = _mm256_add_ps(_mm256_mul_ps(vx0, vx0), _mm256_mul_ps(vy0, vy0));
        const __m256 dist2 = _mm256_add_ps(_mm256_mul_ps(dx, dx), _mm256_mul_ps(dy, dy));
        const __m256 dot = _mm256_add_ps(_mm256_mul_ps(vx0, dx), _mm256_mul_ps(vy0, dy));

        const __m256 a2 = _mm256_mul_ps(_mm256_set1_ps(-3.0f), v2);
        const __m256 a1 = _mm256_mul_ps(_mm256_set1_ps(-12.0f), dot);
        const __m256 a0 = _mm256_mul_ps(_mm256_set1_ps(-9.0f), dist2);

        const __m256 costA = _mm256_mul_ps(_mm256_set1_ps(3.0f), v2);
        const __m256 costB = _mm256_mul_ps(_mm256_set1_ps(3.0f), dist2);
        const __m256 costC = _mm256_mul_ps(_mm256_set1_ps(6.0f), dot);

        const __m256 zero = _mm256_setzero_ps();
        const __m256 one = _mm256_set1_ps(1.0f);
        const __m256 allMask = _mm256_castsi256_ps(_mm256_set1_epi32(-1));
        const __m256 absMask = _mm256_castsi256_ps(_mm256_set1_epi32(0x7fffffff));
        const __m256 zeroTol = _mm256_set1_ps(1e-6f);
        const __m256 two = _mm256_set1_ps(2.0f);
        const __m256 four = _mm256_set1_ps(4.0f);
        const __m256 twoA2 = _mm256_add_ps(a2, a2);

        auto derivativeAt = [&](const __m256 t)
        {
            const __m256 t2 = _mm256_mul_ps(t, t);
            const __m256 t4 = _mm256_mul_ps(t2, t2);
#if defined(__FMA__)
            const __m256 high = _mm256_fmadd_ps(a2, t2, t4);
            return _mm256_fmadd_ps(a1, t, _mm256_add_ps(high, a0));
#else
            return _mm256_add_ps(
                _mm256_add_ps(t4, _mm256_mul_ps(a2, t2)),
                _mm256_add_ps(_mm256_mul_ps(a1, t), a0));
#endif
        };

        auto derivativePrimeAt = [&](const __m256 t)
        {
            const __m256 t2 = _mm256_mul_ps(t, t);
#if defined(__FMA__)
            const __m256 cubic = _mm256_mul_ps(t2, t);
            return _mm256_fmadd_ps(four, cubic, _mm256_fmadd_ps(twoA2, t, a1));
#else
            return _mm256_add_ps(
                _mm256_mul_ps(four, _mm256_mul_ps(t2, t)),
                _mm256_add_ps(_mm256_mul_ps(two, _mm256_mul_ps(a2, t)), a1));
#endif
        };

        auto costAt = [&](const __m256 t)
        {
            const __m256 invT = _mm256_div_ps(one, t);
            const __m256 invT2 = _mm256_mul_ps(invT, invT);
            const __m256 invT3 = _mm256_mul_ps(invT2, invT);
#if defined(__FMA__)
            const __m256 costWithB = _mm256_fmadd_ps(costB, invT3, t);
            const __m256 costWithC = _mm256_fmadd_ps(costC, invT2, costWithB);
            return _mm256_fmadd_ps(costA, invT, costWithC);
#else
            return _mm256_add_ps(
                _mm256_add_ps(t, _mm256_mul_ps(costA, invT)),
                _mm256_add_ps(_mm256_mul_ps(costC, invT2), _mm256_mul_ps(costB, invT3)));
#endif
        };

        __m256 bestCostVec = _mm256_set1_ps(std::numeric_limits<float>::infinity());
        __m256 bestTimeVec = _mm256_set1_ps(-1.0f);
        const float rootTol = std::max(1e-6f, static_cast<float>(timeSolverTolerance_));

        auto considerCandidate = [&](const __m256 t, const __m256 mask)
        {
            const __m256 cost = costAt(t);
            const __m256 better = _mm256_cmp_ps(cost, bestCostVec, _CMP_LT_OQ);
            const __m256 updateMask = _mm256_and_ps(mask, better);
            bestCostVec = _mm256_blendv_ps(bestCostVec, cost, updateMask);
            bestTimeVec = _mm256_blendv_ps(bestTimeVec, t, updateMask);
        };

        auto refineRoot = [&](const __m256 left0,
                              const __m256 right0,
                              const __m256 fLeft0,
                              const __m256 activeMask,
                              double intervalWidth,
                              unsigned int maxBisectionIters)
        {
            __m256 left = left0;
            __m256 right = right0;
            __m256 fLeft = fLeft0;
            __m256 guess = _mm256_mul_ps(_mm256_add_ps(left, right), _mm256_set1_ps(0.5f));

            for (unsigned int iter = 0; iter < rootNewtonIterations_; ++iter)
            {
                const __m256 fGuess = derivativeAt(guess);
                const __m256 dfGuess = derivativePrimeAt(guess);
                const __m256 absDf = _mm256_and_ps(dfGuess, absMask);
                const __m256 newton =
                    _mm256_sub_ps(guess, _mm256_div_ps(fGuess, dfGuess));
                const __m256 mid = _mm256_mul_ps(_mm256_add_ps(left, right), _mm256_set1_ps(0.5f));
                const __m256 validNewton = _mm256_and_ps(
                    _mm256_and_ps(_mm256_cmp_ps(absDf, zeroTol, _CMP_GT_OQ),
                                  _mm256_cmp_ps(newton, left, _CMP_GT_OQ)),
                    _mm256_cmp_ps(newton, right, _CMP_LT_OQ));
                const __m256 candidate = _mm256_blendv_ps(mid, newton, validNewton);
                const __m256 fCandidate = derivativeAt(candidate);

                const __m256 leftNeg = _mm256_cmp_ps(fLeft, zero, _CMP_LT_OQ);
                const __m256 leftPos = _mm256_cmp_ps(fLeft, zero, _CMP_GT_OQ);
                const __m256 midNeg = _mm256_cmp_ps(fCandidate, zero, _CMP_LT_OQ);
                const __m256 midPos = _mm256_cmp_ps(fCandidate, zero, _CMP_GT_OQ);
                const __m256 rootInLeftHalf = _mm256_or_ps(
                    _mm256_and_ps(leftNeg, midPos),
                    _mm256_and_ps(leftPos, midNeg));

                right = _mm256_blendv_ps(right, candidate, rootInLeftHalf);
                left = _mm256_blendv_ps(candidate, left, rootInLeftHalf);
                fLeft = _mm256_blendv_ps(fCandidate, fLeft, rootInLeftHalf);
                guess = candidate;
            }

            unsigned int bisectionIters = 0;
            if (intervalWidth > rootTol)
            {
                bisectionIters = static_cast<unsigned int>(
                    std::ceil(std::log2(intervalWidth / rootTol)));
                bisectionIters = std::min(maxBisectionIters, bisectionIters);
            }

            const __m256 tolVec = _mm256_set1_ps(rootTol);
            for (unsigned int iter = 0; iter < bisectionIters; ++iter)
            {
                const __m256 width = _mm256_sub_ps(right, left);
                const __m256 stillWide = _mm256_cmp_ps(width, tolVec, _CMP_GT_OQ);
                if ((_mm256_movemask_ps(_mm256_and_ps(stillWide, activeMask))) == 0)
                    break;

                const __m256 mid = _mm256_mul_ps(_mm256_add_ps(left, right), _mm256_set1_ps(0.5f));
                const __m256 fMid = derivativeAt(mid);

                const __m256 leftNeg = _mm256_cmp_ps(fLeft, zero, _CMP_LT_OQ);
                const __m256 leftPos = _mm256_cmp_ps(fLeft, zero, _CMP_GT_OQ);
                const __m256 midNeg = _mm256_cmp_ps(fMid, zero, _CMP_LT_OQ);
                const __m256 midPos = _mm256_cmp_ps(fMid, zero, _CMP_GT_OQ);
                const __m256 rootInLeftHalf = _mm256_or_ps(
                    _mm256_and_ps(leftNeg, midPos),
                    _mm256_and_ps(leftPos, midNeg));

                right = _mm256_blendv_ps(right, mid, rootInLeftHalf);
                left = _mm256_blendv_ps(mid, left, rootInLeftHalf);
                fLeft = _mm256_blendv_ps(fMid, fLeft, rootInLeftHalf);
            }

            return _mm256_mul_ps(_mm256_add_ps(left, right), _mm256_set1_ps(0.5f));
        };

        const double lo = std::max(minTrajectoryTime_, 1e-9);
        const double hi = maxTrajectoryTime_;
        if (!(lo > 0.0) || !(hi >= lo) || !std::isfinite(lo) || !std::isfinite(hi))
            return;

        considerCandidate(_mm256_set1_ps(lo), allMask);
        considerCandidate(_mm256_set1_ps(hi), allMask);

        const unsigned int rootScanIntervals = std::max(4u, rootScanIntervals_);
        const bool useLogSpacing = std::isfinite(hi / lo) && (hi / lo > 4.0);
        const double ratio = useLogSpacing
                                 ? std::exp(std::log(hi / lo) / static_cast<double>(rootScanIntervals))
                                 : 1.0;

        double prevT = lo;
        double currT = lo;
        __m256 prevQ = derivativeAt(_mm256_set1_ps(prevT));

        for (unsigned int i = 1; i <= rootScanIntervals; ++i)
        {
            currT = useLogSpacing
                        ? (i == rootScanIntervals ? hi : currT * ratio)
                        : lo + (hi - lo) * static_cast<double>(i) /
                                   static_cast<double>(rootScanIntervals);

            const __m256 currTVec = _mm256_set1_ps(currT);
            const __m256 currQ = derivativeAt(currTVec);

            const __m256 currZeroMask =
                _mm256_cmp_ps(_mm256_and_ps(currQ, absMask), zeroTol, _CMP_LE_OQ);
            considerCandidate(currTVec, currZeroMask);

            const __m256 prevNeg = _mm256_cmp_ps(prevQ, zero, _CMP_LT_OQ);
            const __m256 prevPos = _mm256_cmp_ps(prevQ, zero, _CMP_GT_OQ);
            const __m256 currNeg = _mm256_cmp_ps(currQ, zero, _CMP_LT_OQ);
            const __m256 currPos = _mm256_cmp_ps(currQ, zero, _CMP_GT_OQ);
            const __m256 signChangeMask = _mm256_or_ps(
                _mm256_and_ps(prevNeg, currPos),
                _mm256_and_ps(prevPos, currNeg));

            if (_mm256_movemask_ps(signChangeMask) != 0)
            {
                const __m256 root = refineRoot(
                    _mm256_set1_ps(prevT), currTVec, prevQ, signChangeMask,
                    currT - prevT, rootBisectionIterations_);
                considerCandidate(root, signChangeMask);
            }

            prevT = currT;
            prevQ = currQ;
        }

        alignas(32) float tValues[PrimitiveBatchWidth];
        alignas(32) float costValues[PrimitiveBatchWidth];
        _mm256_store_ps(tValues, bestTimeVec);
        _mm256_store_ps(costValues, bestCostVec);

        for (std::size_t lane = 0; lane < PrimitiveBatchWidth; ++lane)
        {
            const double t = tValues[lane];
            const double cost = costValues[lane];
            if (!std::isfinite(t) || !std::isfinite(cost) || t < lo || t > hi)
                continue;

            prepareEdgePrimitive(t,
                                 fromStates[lane].x, fromStates[lane].y,
                                 fromStates[lane].vx, fromStates[lane].vy,
                                 targetStates[lane].x, targetStates[lane].y,
                                 block.primitive[lane]);
            const DIState terminal = evaluateEdgeState(block.primitive[lane], t);
            fromDIState(outStates[lane], terminal);

            block.ok[lane] = true;
            block.tOpt[lane] = t;
            block.incCost[lane] = cost;
            block.terminal.x[lane] = terminal.x;
            block.terminal.y[lane] = terminal.y;
            block.terminal.vx[lane] = terminal.vx;
            block.terminal.vy[lane] = terminal.vy;
        }

        return;
    }
#endif

    for (std::size_t lane = 0; lane < block.count; ++lane)
    {
        EdgeEval eval;
        if (!solveFreeVelEdgeFromDIStates(fromStates[lane], targetStates[lane], eval, outStates[lane]))
            continue;

        block.ok[lane] = true;
        block.tOpt[lane] = eval.t_opt;
        block.incCost[lane] = eval.incCost.value();
        block.terminal.x[lane] = eval.terminal.x;
        block.terminal.y[lane] = eval.terminal.y;
        block.terminal.vx[lane] = eval.terminal.vx;
        block.terminal.vy[lane] = eval.terminal.vy;
        block.primitive[lane] = eval.primitive;
    }
}
// =========================銆愪慨鏀?a�? 4D------------>2D=========================
double ompl::control::RRTstarDI::motionDistanceXY(const Motion *a,
                                                   const Motion *b) const
{
    const double dx = a->cache.x - b->cache.x;
    const double dy = a->cache.y - b->cache.y;
    return std::sqrt(dx * dx + dy * dy);
}

double ompl::control::RRTstarDI::motionDistanceXYSquared(const Motion *a,
                                                         const Motion *b) const
{
    const double dx = a->cache.x - b->cache.x;
    const double dy = a->cache.y - b->cache.y;
    return dx * dx + dy * dy;
}

void ompl::control::RRTstarDI::interpolateXY(const base::State *from,
                                             const base::State *to,
                                             double maxDist,
                                             base::State *out) const
{
    double x0, y0, vx0, vy0;
    double x1, y1, vx1, vy1;
    extractStateComponents(from, x0, y0, vx0, vy0);
    extractStateComponents(to,   x1, y1, vx1, vy1);

    const double dx = x1 - x0;
    const double dy = y1 - y0;
    const double d  = std::sqrt(dx * dx + dy * dy);

    if (d <= 1e-12 || d <= maxDist)
    {
        // 浣嶇疆鐩存帴鍒扮洰鏍囷紱閫熷害鍙綔涓哄崰浣嶏紝涓嶄綔涓虹粓绔害鏉?        setStateComponents(out, x1, y1, 0.0, 0.0);
        return;
    }

    const double r = maxDist / d;
    const double nx = x0 + r * dx;
    const double ny = y0 + r * dy;

    // 娉ㄦ剰锛氳繖�?dstate 鍙槸鈥滀綅缃弬鑰冪偣鈥濓紝鍚庨�?DI 杩炶竟鍙細�?x,y
    setStateComponents(out, nx, ny, 0.0, 0.0);
}

double ompl::control::RRTstarDI::getXYMeasure() const
{
    const auto *rss =
        si_->getStateSpace()->as<ompl::base::RealVectorStateSpace>();

    const auto &b = rss->getBounds();
    return (b.high[0] - b.low[0]) * (b.high[1] - b.low[1]);
}
// =========================銆愪慨鏀?a�?========================


ompl::base::PlannerStatus ompl::control::RRTstarDI::solve(
    const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    const auto solveStartTime = std::chrono::steady_clock::now();
    auto secondsSinceSolveStart = [&solveStartTime]() -> double
    {
        return std::chrono::duration<double>(
                   std::chrono::steady_clock::now() - solveStartTime)
            .count();
    };

    base::Goal *goal = pdef_->getGoal().get();
    auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);

    bool symCost = opt_->isSymmetric();
    (void)symCost;

    if (pis_.haveMoreStartStates())
    {
        while (const base::State *st = pis_.nextStart())
        {
            auto *motion = new Motion(siC_);
            si_->copyState(motion->state, st);
            motion->cache = toDIState(st);
            motion->cost = opt_->identityCost();
            motion->incCost = opt_->identityCost();
            motion->parent = nullptr;
            addMotionToNearestNeighbors(motion);
            startMotions_.push_back(motion);

            double distTmp = std::numeric_limits<double>::infinity();
            if (goal->isSatisfied(motion->state, &distTmp))
            {
                motion->inGoal = true;
                goalMotions_.push_back(motion);

                if (!bestGoalMotion_ || opt_->isCostBetterThan(motion->cost, bestCost_))
                {
                    bestGoalMotion_ = motion;
                    bestCost_ = motion->cost;
                    ++improvingGoalCount_;
                }
            }
        }
        infSampler_.reset();
    }

    if (nearestNeighborCount() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!sampler_ && !infSampler_)
        allocSampler();

    const std::size_t nnCountAtStart = nearestNeighborCount();
    const std::size_t expectedMotionCount =
        nnCountAtStart + static_cast<std::size_t>(maxSampleCount_) + 1u;
    soaMotions_.reserve(expectedMotionCount);
    soaX_.reserve(expectedMotionCount);
    soaY_.reserve(expectedMotionCount);
    soaDistanceScratch_.reserve(expectedMotionCount);
    soaIndexScratch_.reserve(expectedMotionCount);
    OMPL_INFORM("%s: Started planning with %u states. Seeking a solution better than %.5f.",
                getName().c_str(), static_cast<unsigned int>(nnCountAtStart), opt_->getCostThreshold().value());

    if ((useTreePruning_ || useRejectionSampling_ || useInformedSampling_ ||
         useNewStateRejection_) &&
        !si_->getStateSpace()->isMetricSpace())
    {
        OMPL_WARN("%s: The state space (%s) is not metric and as a result the "
                  "optimization objective may not satisfy the triangle inequality. "
                  "You may need to disable pruning or rejection.",
                  getName().c_str(), si_->getStateSpace()->getName().c_str());
    }

    const base::ReportIntermediateSolutionFn intermediateSolutionCallback =
        pdef_->getIntermediateSolutionCallback();

    Motion *approxGoalMotion = nullptr;
    double approxDist = std::numeric_limits<double>::infinity();

    auto *rmotion = new Motion(siC_);
    base::State *rstate = rmotion->state;

    base::State *xstate = si_->allocState();
    base::State *scratch = si_->allocState();
    base::State *candState = si_->allocState();
    base::State *bestState = si_->allocState();
    base::State *goalRefState = si_->allocState();
    base::State *goalCandState = si_->allocState();

    std::array<base::State *, PrimitiveBatchWidth> parentBlockStates{};
    for (std::size_t i = 0; i < PrimitiveBatchWidth; ++i)
    {
        parentBlockStates[i] = si_->allocState();
    }
    std::vector<Motion *> nbh;

    std::vector<base::Cost> costs;
    std::vector<base::Cost> incCosts;
    std::vector<std::size_t> sortedCostIndices;
    std::vector<double> tCache;
    std::vector<unsigned char> tOk;

    // =========================銆愪慨鏀?1�?========================
    // delayCC 绗竴杞凡缁忕畻鍑虹殑瑙ｆ瀽缁堢鐘舵€佺紦瀛樹笅鏉ワ紝
    // 绗簩杞悎娉曟€ф鏌ユ椂鐩存帴澶嶇敤锛岄伩鍏嶅啀�?DI_stateFreeVel_out(...)
    std::vector<double> fxCache;
    std::vector<double> fyCache;
    std::vector<double> fvxCache;
    std::vector<double> fvyCache;
    std::vector<DIEdgePrimitive> primitiveCache;
    // ==========================================================

    unsigned int rewireTest = 0;
    unsigned int statesGenerated = 0;
    const double maxDistanceSq = maxDistance_ * maxDistance_;
    std::size_t parentFullBlocks = 0;
    std::size_t parentTailBlocks = 0;
    std::size_t parentBlockCandidates = 0;
    std::size_t rewireFullBlocks = 0;
    std::size_t rewireTailBlocks = 0;
    std::size_t rewireBlockCandidates = 0;

   
   
    // ==========================================================

    if (bestGoalMotion_)
    {
        OMPL_INFORM("%s: Starting planning with existing solution of cost %.5f (first solution time %.6f seconds)",
                    getName().c_str(), bestCost_.value(), secondsSinceSolveStart());
    }

    if (useKNearest_)
    {
        OMPL_INFORM("%s: Initial k-nearest value of %u",
                    getName().c_str(),
                    (unsigned int)std::ceil(k_rrt_ * log((double)(nearestNeighborCount() + 1u))));
    }
    else
    {
        OMPL_INFORM("%s: Initial rewiring radius of %.2f", getName().c_str(),
                    std::min(maxDistance_,
                             r_rrt_ * std::pow(log((double)(nearestNeighborCount() + 1u)) /
                                                   ((double)(nearestNeighborCount() + 1u)),
                                               1.0 / (double)(si_->getStateDimension()))));
    }

    CostIndexCompare compareFn(costs, *opt_);

    sampleCount_ = 0;
    const bool forceSamples = (maxSampleCount_ > 0);

    while (ptc == false)
    {
        iterations_++;

        // ------------------------------------------------------------------
        // 1) 閲囨�?
        // ------------------------------------------------------------------
        if (goal_s && goalMotions_.size() < goal_s->maxSampleCount() &&
            rng_.uniform01() < goalBias_ && goal_s->canSample())
        {
            goal_s->sampleGoal(rstate);
        }
        else
        {
            if (!sampleUniform(rstate))
                continue;
        }
        rmotion->cache = toDIState(rstate);

        // ------------------------------------------------------------------
        // 2) 鏈€杩戦�?
        // ------------------------------------------------------------------
        Motion *nmotion = nearestNeighborSoA(rmotion);
        if (!nmotion)
            continue;

        if (intermediateSolutionCallback && si_->equalStates(nmotion->state, rstate))
            continue;

        base::State *dstate = rstate;

        // ------------------------------------------------------------------
        // 3) 鍑犱綍姝ラ暱鎺у埗
        // ------------------------------------------------------------------
        // ========================淇敼a 琛ュ伩鎺у�?===================================== 
        double d = motionDistanceXY(nmotion, rmotion);
		if (d > maxDistance_)
		{
		    interpolateXY(nmotion->state, rstate, maxDistance_, xstate);
		    dstate = xstate;
		}
//        double d = si_->distance(nmotion->state, rstate);
//        if (d > maxDistance_)
//        {
//            si_->getStateSpace()->interpolate(nmotion->state, rstate, maxDistance_ / d, xstate);
//            dstate = xstate;
//        }

        // ------------------------------------------------------------------
        // 4) 鍏堢敓鎴?nmotion -> sampled target 鐨勯粯璁ゅ€欓€?        // ------------------------------------------------------------------
        // =========================銆愪慨鏀?3�?========================
        // 榛樿杈圭洿鎺ヨ�?evalEdge锛岄伩鍏嶏細
        // computeOptimalTimeFreeVel + DI_stateFreeVel_out + dynamicMotionCost
        // 涓夊閲嶅璋冪�?
       EdgeEval evalDefault;
        if (!solveFreeVelEdge(nmotion->state, dstate, evalDefault, candState))
		    continue;
		
		double t_default = evalDefault.t_opt;
		base::Cost defaultIncCost = evalDefault.incCost;
        // ==========================================================

        if (!checkPrimitiveState(evalDefault.terminal, scratch))
            continue;

        if (!isEdgeControlWithinBounds(evalDefault.primitive))
            continue;

        if (!checkFreeVelMotionWithTime(evalDefault.primitive,
                                       nmotion->cache, evalDefault.terminal, scratch))
            continue;

        auto *motion = new Motion(siC_);
        motion->parent = nmotion;
        si_->copyState(motion->state, candState);
        motion->cache = evalDefault.terminal;

        motion->incCost = defaultIncCost;
        motion->cost = opt_->combineCosts(nmotion->cost, motion->incCost);
        motion->duration = t_default;

        Motion *bestParent = motion->parent;
        si_->copyState(bestState, motion->state);
        base::Cost bestIncCost = motion->incCost;
        base::Cost bestTotalCost = motion->cost;
        double bestDuration = motion->duration;

        // ------------------------------------------------------------------
        // 5) 閭诲�?
        // ------------------------------------------------------------------
        nbh.clear();
        getNeighbors(motion, nbh);
        rewireTest += nbh.size();
        ++statesGenerated;

        if (costs.size() < nbh.size())
        {
            costs.resize(nbh.size());
            incCosts.resize(nbh.size());
            sortedCostIndices.resize(nbh.size());
        }
        if (tCache.size() < nbh.size())
        {
            tCache.resize(nbh.size());
            tOk.resize(nbh.size());
        }
        if (fxCache.size() < nbh.size())
        {
            fxCache.resize(nbh.size());
            fyCache.resize(nbh.size());
            fvxCache.resize(nbh.size());
            fvyCache.resize(nbh.size());
            primitiveCache.resize(nbh.size());
        }

        for (std::size_t i = 0; i < nbh.size(); ++i)
        {
            sortedCostIndices[i] = i;
            tOk[i] = 0;
            costs[i] = opt_->infiniteCost();
            incCosts[i] = opt_->infiniteCost();
        }

        // ------------------------------------------------------------------
        // 6) 閫夋嫨鏇翠紭鐖惰妭鐐癸紙candidate -> check -> commit �?best*�?        // ------------------------------------------------------------------
        if (delayCC_)
        {
            EdgeEvalBlock parentBlock;
            std::array<Motion *, PrimitiveBatchWidth> parentPtrs;
            std::array<std::size_t, PrimitiveBatchWidth> parentIdxs;
            std::size_t parentCount = 0;

            auto flushParentCandidates = [&](std::size_t count)
            {
                if (count == 0)
                    return;

                parentBlockCandidates += count;
                if (count == PrimitiveBatchWidth)
                    ++parentFullBlocks;
                else
                    ++parentTailBlocks;

                evaluateParentCandidateBlock(
                    parentPtrs.data(), count, motion->state, parentBlock, parentBlockStates);

                for (std::size_t lane = 0; lane < parentBlock.count; ++lane)
                {
                    const std::size_t idx = parentIdxs[lane];
                    if (!parentBlock.ok[lane])
                        continue;

                    tCache[idx] = parentBlock.tOpt[lane];
                    tOk[idx] = 1;
                    incCosts[idx] = base::Cost(parentBlock.incCost[lane]);
                    costs[idx] = opt_->combineCosts(nbh[idx]->cost, incCosts[idx]);

                    fxCache[idx] = parentBlock.terminal.x[lane];
                    fyCache[idx] = parentBlock.terminal.y[lane];
                    fvxCache[idx] = parentBlock.terminal.vx[lane];
                    fvyCache[idx] = parentBlock.terminal.vy[lane];
                    primitiveCache[idx] = parentBlock.primitive[lane];
                }
            };

            for (std::size_t idx = 0; idx < nbh.size(); ++idx)
            {
                Motion *parentCand = nbh[idx];
                if (!opt_->isCostBetterThan(parentCand->cost, bestTotalCost))
                    continue;

                if (!useKNearest_ &&
                    motionDistanceXYSquared(parentCand, motion) >= maxDistanceSq)
                    continue;

                parentPtrs[parentCount] = parentCand;
                parentIdxs[parentCount] = idx;
                ++parentCount;

                if (parentCount == PrimitiveBatchWidth)
                {
                    flushParentCandidates(parentCount);
                    parentCount = 0;
                }
            }
            flushParentCandidates(parentCount);

            std::sort(sortedCostIndices.begin(),
                      sortedCostIndices.begin() + nbh.size(),
                      compareFn);

            for (auto it = sortedCostIndices.begin();
                 it != sortedCostIndices.begin() + nbh.size(); ++it)
            {
                const std::size_t idx = *it;

                if (!opt_->isCostBetterThan(costs[idx], bestTotalCost))
                    break;

                Motion *parentCand = nbh[idx];

                if (!tOk[idx])
                    continue;

                // =========================銆愪慨鏀?b�? 4D----->2D    =========================
                if (!useKNearest_ &&
                    motionDistanceXYSquared(parentCand, motion) >= maxDistanceSq)
                    continue;

                setStateComponents(candState,
                                   fxCache[idx], fyCache[idx],
                                   fvxCache[idx], fvyCache[idx]);

                if (!checkPrimitiveState(
                        DIState{fxCache[idx], fyCache[idx], fvxCache[idx], fvyCache[idx]},
                        scratch))
                    continue;

                if (!isEdgeControlWithinBounds(primitiveCache[idx]))
                    continue;

                if (!checkFreeVelMotionWithTime(primitiveCache[idx],
                                               parentCand->cache,
                                               DIState{fxCache[idx], fyCache[idx], fvxCache[idx], fvyCache[idx]},
                                               scratch))
                    continue;

                bestParent = parentCand;
                si_->copyState(bestState, candState);
                const DIState bestStateCache{fxCache[idx], fyCache[idx], fvxCache[idx], fvyCache[idx]};
                bestIncCost = incCosts[idx];
                bestTotalCost = costs[idx];
                bestDuration = tCache[idx];
                // ==========================================================

                fromDIState(bestState, bestStateCache);
                break;
            }
        }
        else
        {
            EdgeEvalBlock parentBlock;
            std::array<Motion *, PrimitiveBatchWidth> parentPtrs;
            std::size_t parentCount = 0;

            auto flushParentCandidates = [&](std::size_t count)
            {
                if (count == 0)
                    return;

                parentBlockCandidates += count;
                if (count == PrimitiveBatchWidth)
                    ++parentFullBlocks;
                else
                    ++parentTailBlocks;

                evaluateParentCandidateBlock(
                    parentPtrs.data(), count, motion->state, parentBlock, parentBlockStates);

                for (std::size_t lane = 0; lane < parentBlock.count; ++lane)
                {
                    Motion *parentCand = parentPtrs[lane];

                    if (!parentBlock.ok[lane])
                        continue;

                    if (!useKNearest_ &&
                        motionDistanceXYSquared(parentCand, motion) >= maxDistanceSq)
                        continue;

                    const double t2 = parentBlock.tOpt[lane];
                    const base::Cost inc(parentBlock.incCost[lane]);
                    const base::Cost total = opt_->combineCosts(parentCand->cost, inc);

                    if (!opt_->isCostBetterThan(total, bestTotalCost))
                        continue;

                    const DIState terminal{
                        parentBlock.terminal.x[lane],
                        parentBlock.terminal.y[lane],
                        parentBlock.terminal.vx[lane],
                        parentBlock.terminal.vy[lane]};

                    if (!checkPrimitiveState(terminal, scratch))
                        continue;

                    if (!isEdgeControlWithinBounds(parentBlock.primitive[lane]))
                        continue;

                    base::State *laneState = parentBlockStates[lane];
                    if (!checkFreeVelMotionWithTime(parentBlock.primitive[lane],
                                                   parentCand->cache, terminal, scratch))
                        continue;

                    bestParent = parentCand;
                    si_->copyState(bestState, laneState);
                    bestIncCost = inc;
                    bestTotalCost = total;
                    bestDuration = t2;
                }
            };

            for (std::size_t idx = 0; idx < nbh.size(); ++idx)
            {
                Motion *parentCand = nbh[idx];
                if (parentCand == nmotion)
                    continue;

                if (!opt_->isCostBetterThan(parentCand->cost, bestTotalCost))
                    continue;

                if (!useKNearest_ &&
                    motionDistanceXYSquared(parentCand, motion) >= maxDistanceSq)
                    continue;

                parentPtrs[parentCount] = parentCand;
                ++parentCount;

                if (parentCount == PrimitiveBatchWidth)
                {
                    flushParentCandidates(parentCount);
                    parentCount = 0;
                }
            }
            flushParentCandidates(parentCount);
        }

        // 鎶婃渶浣冲€欓€変竴娆℃€ф彁浜ょ�?motion
        motion->parent = bestParent;
        si_->copyState(motion->state, bestState);
        motion->cache = toDIState(bestState);
        motion->incCost = bestIncCost;
        motion->cost = bestTotalCost;
        motion->duration = bestDuration;

        // ------------------------------------------------------------------
        // 7) 鏂拌妭鐐规嫆�?        // ------------------------------------------------------------------
        bool added = false;

        if (useNewStateRejection_)
        {
            if (opt_->isCostBetterThan(solutionHeuristic(motion), bestCost_))
            {
                addMotionToNearestNeighbors(motion);
                motion->parent->children.push_back(motion);
                added = true;
            }
            else
            {
                freeMotionControls(motion);
                si_->freeState(motion->state);
                delete motion;
                continue;
            }
        }
        else
        {
            addMotionToNearestNeighbors(motion);
            motion->parent->children.push_back(motion);
            added = true;
        }

        if (added)
        {
            ++sampleCount_;
            if (forceSamples && sampleCount_ >= maxSampleCount_)
                break;
        }

        // ------------------------------------------------------------------
        // 8) rewiring
        // ------------------------------------------------------------------
        EdgeEvalBlock rewireBlock;
        std::array<Motion *, PrimitiveBatchWidth> rewirePtrs;
        std::size_t rewireCount = 0;

        auto flushRewireCandidates = [&](std::size_t count)
        {
            if (count == 0)
                return;

            rewireBlockCandidates += count;
            if (count == PrimitiveBatchWidth)
                ++rewireFullBlocks;
            else
                ++rewireTailBlocks;

            evaluateRewireCandidateBlock(
                motion->state, rewirePtrs.data(), count, rewireBlock);

            for (std::size_t lane = 0; lane < rewireBlock.count; ++lane)
            {
                Motion *nbr = rewirePtrs[lane];

                if (!rewireBlock.ok[lane])
                    continue;

                const double t3 = rewireBlock.tOpt[lane];
                const base::Cost incNew(rewireBlock.incCost[lane]);
                const base::Cost costFromMotion = opt_->combineCosts(motion->cost, incNew);

                if (!opt_->isCostBetterThan(costFromMotion, nbr->cost))
                    continue;

                const DIState terminal{
                    rewireBlock.terminal.x[lane],
                    rewireBlock.terminal.y[lane],
                    rewireBlock.terminal.vx[lane],
                    rewireBlock.terminal.vy[lane]};

                if (!checkPrimitiveState(terminal, scratch))
                    continue;

                if (!isEdgeControlWithinBounds(rewireBlock.primitive[lane]))
                    continue;

                if (!checkFixedVelMotionWithTime(
                        rewireBlock.primitive[lane], motion->cache, terminal, scratch))
                    continue;

                removeFromParent(nbr);
                nbr->parent = motion;
                motion->children.push_back(nbr);

                nbr->incCost = incNew;
                nbr->cost = costFromMotion;
                nbr->duration = t3;

                clearControlSequence(nbr->edgeControls);

                updateChildCosts(nbr);
            }
        };

        for (std::size_t idx = 0; idx < nbh.size(); ++idx)
        {
            Motion *nbr = nbh[idx];
            if (nbr == motion->parent)
                continue;

            if (!opt_->isCostBetterThan(motion->cost, nbr->cost))
                continue;

            if (!useKNearest_ &&
                motionDistanceXYSquared(motion, nbr) >= maxDistanceSq)
                continue;

            rewirePtrs[rewireCount] = nbr;
            ++rewireCount;

            if (rewireCount == PrimitiveBatchWidth)
            {
                flushRewireCandidates(rewireCount);
                rewireCount = 0;
            }
        }
        flushRewireCandidates(rewireCount);

        // ------------------------------------------------------------------
        // 9) 鐩爣妫€娴?        // ------------------------------------------------------------------
 //===================================================銆愮粓鐐圭洿杩炴鏌ャ€?========================================       
        // ------------------------------------------------------------------
// 9) ????(???:????? + ???????)
// ------------------------------------------------------------------
bool checkForSolution = false;
double distanceFromGoal = std::numeric_limits<double>::infinity();
Motion *goalCandidateMotion = nullptr;

// 9.1 ?????????? goal ?
if (goal->isSatisfied(motion->state, &distanceFromGoal))
{
    if (!motion->inGoal)
    {
        motion->inGoal = true;
        goalMotions_.push_back(motion);
    }
    goalCandidateMotion = motion;
    checkForSolution = true;
}
// 9.2 MATLAB ?:?????????,???????
else if (goal_s && goal_s->canSample() && distanceFromGoal < maxDistance_)
{
    goal_s->sampleGoal(goalRefState);  // PositionOnlyGoal:??? x,y,??????

    EdgeEval goalEval;
        if (solveFreeVelEdge(motion->state, goalRefState, goalEval, goalCandState) &&
        checkPrimitiveState(goalEval.terminal, scratch) &&
        isEdgeControlWithinBounds(goalEval.primitive) &&
        checkFreeVelMotionWithTime(goalEval.primitive, motion->cache,
                                   goalEval.terminal, scratch))
    {
        if (goal->isSatisfied(goalCandState))
        {
            const base::Cost goalTotalCost = opt_->combineCosts(motion->cost, goalEval.incCost);
            if (!bestGoalMotion_ || opt_->isCostBetterThan(goalTotalCost, bestCost_))
            {
                auto *goalMotion = new Motion(siC_);
                goalMotion->parent = motion;
                motion->children.push_back(goalMotion);

                si_->copyState(goalMotion->state, goalCandState);
                goalMotion->cache = goalEval.terminal;
                goalMotion->incCost = goalEval.incCost;
                goalMotion->cost = goalTotalCost;
                goalMotion->duration = goalEval.t_opt;

                addMotionToNearestNeighbors(goalMotion);
                goalCandidateMotion = goalMotion;
                checkForSolution = true;
            }
        }
    }
}

// 9.3 ???? bestGoalMotion_ / bestCost_
if (checkForSolution && goalCandidateMotion)
{
    if (!goalCandidateMotion->inGoal)
    {
        goalCandidateMotion->inGoal = true;
        goalMotions_.push_back(goalCandidateMotion);
    }

    bool updatedSolution = false;
    for (auto *goalMotion : goalMotions_)
    {
        if (!goalMotion)
            continue;

        if (!bestGoalMotion_ || opt_->isCostBetterThan(goalMotion->cost, bestCost_))
        {
            bestGoalMotion_ = goalMotion;
            bestCost_ = goalMotion->cost;
            updatedSolution = true;
        }
    }

    if (updatedSolution)
    {
        ++improvingGoalCount_;

        if (improvingGoalCount_ == 1)
        {
            OMPL_INFORM("%s: Found an initial solution with a cost of %.2f in %.6f seconds, %u iterations (%u vertices in the graph)",
                        getName().c_str(), bestCost_.value(), secondsSinceSolveStart(), iterations_,
                        static_cast<unsigned int>(nearestNeighborCount()));
        }

        if (useTreePruning_)
            pruneTree(bestCost_);

        if (intermediateSolutionCallback)
        {
            std::vector<const base::State *> spath;
            Motion *intermediate_solution = bestGoalMotion_;

            while (intermediate_solution != nullptr)
            {
                spath.push_back(intermediate_solution->state);
                intermediate_solution = intermediate_solution->parent;
            }

            std::reverse(spath.begin(), spath.end());
            intermediateSolutionCallback(this, spath, bestCost_);
        }
    }
}
//===================================================銆愮粓鐐圭洿杩炴鏌ャ€?========================================
        /*
        bool checkForSolution = false;
        double distanceFromGoal = std::numeric_limits<double>::infinity();

        if (goal->isSatisfied(motion->state, &distanceFromGoal))
        {
            motion->inGoal = true;
            goalMotions_.push_back(motion);
            checkForSolution = true;
        }

        if (checkForSolution)
        {
            bool updatedSolution = false;

            if (!bestGoalMotion_ && !goalMotions_.empty())
            {
                bestGoalMotion_ = goalMotions_.front();
                bestCost_ = bestGoalMotion_->cost;
                updatedSolution = true;

                OMPL_INFORM("%s: Found an initial solution with a cost of %.2f in %u iterations (%u vertices in the graph)",
                            getName().c_str(), bestCost_.value(), iterations_,
                            static_cast<unsigned int>(nearestNeighborCount()));
            }
            else
            {
                for (auto &goalMotion : goalMotions_)
                {
                    if (opt_->isCostBetterThan(goalMotion->cost, bestCost_))
                    {
                        bestGoalMotion_ = goalMotion;
                        bestCost_ = bestGoalMotion_->cost;
                        updatedSolution = true;

                        if (opt_->isSatisfied(bestCost_))
                            break;
                    }
                }
            }

            if (updatedSolution)
            {
                if (useTreePruning_)
                    pruneTree(bestCost_);

                if (intermediateSolutionCallback)
                {
                    std::vector<const base::State *> spath;
                    Motion *intermediate_solution = bestGoalMotion_;

                    while (intermediate_solution != nullptr)
                    {
                        spath.push_back(intermediate_solution->state);
                        intermediate_solution = intermediate_solution->parent;
                    }

                    std::reverse(spath.begin(), spath.end());
                    intermediateSolutionCallback(this, spath, bestCost_);
                }
            }
        }
*/
        // ------------------------------------------------------------------
        // 10) 杩戜技�?        // ------------------------------------------------------------------
        if (!bestGoalMotion_ && distanceFromGoal < approxDist)
        {
            approxGoalMotion = motion;
            approxDist = distanceFromGoal;
        }

        // ------------------------------------------------------------------
        // 11) 鎻愬墠缁堟
        // ------------------------------------------------------------------
        if (!forceSamples)
        {
            if (bestGoalMotion_ && opt_->isSatisfied(bestCost_))
                break;
        }
    }

    // ----------------------------------------------------------------------
    // 12) 鏋勯€犺В璺�?
    // ----------------------------------------------------------------------
    Motion *newSolution = nullptr;

    if (bestGoalMotion_)
        newSolution = bestGoalMotion_;
    else if (approxGoalMotion)
        newSolution = approxGoalMotion;

    if (newSolution)
    {
        if (!forceSamples)
            ptc.terminate();

        std::vector<Motion *> mpath;
        Motion *iterMotion = newSolution;
        while (iterMotion != nullptr)
        {
            mpath.push_back(iterMotion);
            iterMotion = iterMotion->parent;
        }

        std::reverse(mpath.begin(), mpath.end());

        auto path = std::make_shared<ompl::control::PathControl>(siC_);
        path->append(mpath.front()->state);

        base::State *analyticState = si_->allocState();
        std::vector<ControlSegment> pathControls;
        pathControls.reserve(controlDiscretization_);

        for (std::size_t i = 1; i < mpath.size(); ++i)
        {
            Motion *m = mpath[i];
            Motion *p = m->parent;

            buildControlSequenceForEdge(p->state, m->state, m->duration, pathControls);
            DIEdgePrimitive pathPrimitive;
            prepareEdgePrimitiveFixedVel(
                m->duration, p->cache, m->cache, pathPrimitive);

            if (pathControls.empty())
            {
                path->append(m->state);
                continue;
            }

            double elapsed = 0.0;

            for (std::size_t j = 0; j < pathControls.size(); ++j)
            {
                const auto &seg = pathControls[j];
                const bool isLast = (j + 1 == pathControls.size());

                elapsed += seg.duration;

                if (isLast)
                {
                    // 鏈€鍚庝竴娈典粛鐒剁洿鎺ラ敋鍒颁富鑺傜偣锛岄伩鍏嶆暟鍊艰宸?                    path->append(m->state, seg.control, seg.duration);
                }
                else
                {
                    fromDIState(analyticState, evaluateEdgeState(pathPrimitive, elapsed));
                    path->append(analyticState, seg.control, seg.duration);
                }
            }
        }

        clearControlSequence(pathControls);
        si_->freeState(analyticState);

        base::PlannerSolution psol(path);
        psol.setPlannerName(getName());

        if (!bestGoalMotion_)
            psol.setApproximate(approxDist);

        psol.setOptimized(opt_, newSolution->cost, opt_->isSatisfied(bestCost_));
        pdef_->addSolutionPath(psol);
    }

    for (std::size_t i = 0; i < PrimitiveBatchWidth; ++i)
    {
        si_->freeState(parentBlockStates[i]);
    }

    si_->freeState(bestState);
    si_->freeState(candState);
    si_->freeState(scratch);
    si_->freeState(xstate);
    
    //===================================================銆愮粓鐐圭洿杩炴鏌ャ€?========================================
    si_->freeState(goalCandState);
	si_->freeState(goalRefState);
    //===================================================銆愮粓鐐圭洿杩炴鏌ャ€?========================================

    if (rmotion->state)
        si_->freeState(rmotion->state);
    delete rmotion;

    OMPL_INFORM("%s: Created %u new states. Checked %u rewire options. %u goal states in tree (%u improved best cost). Final solution cost %.3f",
                getName().c_str(), statesGenerated, rewireTest,
                static_cast<unsigned int>(goalMotions_.size()), improvingGoalCount_, bestCost_.value());

    const std::size_t fullBlocks = parentFullBlocks + rewireFullBlocks;
    const std::size_t totalBlocks = fullBlocks + parentTailBlocks + rewireTailBlocks;
    const std::size_t totalBlockCandidates = parentBlockCandidates + rewireBlockCandidates;
    const double fullBlockRate = totalBlocks > 0
                                     ? 100.0 * static_cast<double>(fullBlocks) / static_cast<double>(totalBlocks)
                                     : 0.0;
    const double averageLanes = totalBlocks > 0
                                    ? static_cast<double>(totalBlockCandidates) / static_cast<double>(totalBlocks)
                                    : 0.0;

    OMPL_INFORM("%s: Edge blocks parent(SIMD) full/tail/candidates=%llu/%llu/%llu; FixedVel rewire full/tail/candidates=%llu/%llu/%llu; full-block rate %.1f%%, average lanes %.2f/8",
                getName().c_str(),
                static_cast<unsigned long long>(parentFullBlocks),
                static_cast<unsigned long long>(parentTailBlocks),
                static_cast<unsigned long long>(parentBlockCandidates),
                static_cast<unsigned long long>(rewireFullBlocks),
                static_cast<unsigned long long>(rewireTailBlocks),
                static_cast<unsigned long long>(rewireBlockCandidates),
                fullBlockRate, averageLanes);

    return {newSolution != nullptr, bestGoalMotion_ == nullptr};
}

//=====================================銆愪慨鏀?4D---->2D�?==================================== 
std::size_t ompl::control::RRTstarDI::nearestNeighborCount() const
{
    return nn_ ? motionCount_ : 0;
}

void ompl::control::RRTstarDI::registerMotionSoA(Motion *motion)
{
    if (!motion)
        return;

    if (motion->soaIndex < soaMotions_.size() &&
        soaMotions_[motion->soaIndex] == motion)
    {
        updateMotionSoA(motion);
        return;
    }

    motion->soaIndex = soaMotions_.size();
    soaMotions_.push_back(motion);
    soaX_.push_back(static_cast<float>(motion->cache.x));
    soaY_.push_back(static_cast<float>(motion->cache.y));
}

void ompl::control::RRTstarDI::unregisterMotionSoA(Motion *motion)
{
    if (!motion || motion->soaIndex >= soaMotions_.size() ||
        soaMotions_[motion->soaIndex] != motion)
        return;

    const std::size_t removed = motion->soaIndex;
    const std::size_t last = soaMotions_.size() - 1u;
    if (removed != last)
    {
        soaMotions_[removed] = soaMotions_[last];
        soaX_[removed] = soaX_[last];
        soaY_[removed] = soaY_[last];
        soaMotions_[removed]->soaIndex = removed;
    }

    soaMotions_.pop_back();
    soaX_.pop_back();
    soaY_.pop_back();
    motion->soaIndex = std::numeric_limits<std::size_t>::max();
}

void ompl::control::RRTstarDI::updateMotionSoA(Motion *motion)
{
    if (!motion || motion->soaIndex >= soaMotions_.size() ||
        soaMotions_[motion->soaIndex] != motion)
        return;

    soaX_[motion->soaIndex] = static_cast<float>(motion->cache.x);
    soaY_[motion->soaIndex] = static_cast<float>(motion->cache.y);
}

void ompl::control::RRTstarDI::addMotionToNearestNeighbors(Motion *motion)
{
    nn_->add(motion);
    registerMotionSoA(motion);
    ++motionCount_;
}

ompl::control::RRTstarDI::Motion *
ompl::control::RRTstarDI::nearestNeighborSoA(const Motion *query) const
{
    if (!query || soaMotions_.empty())
        return nullptr;

    const float qx = static_cast<float>(query->cache.x);
    const float qy = static_cast<float>(query->cache.y);
    float bestDistance = std::numeric_limits<float>::infinity();
    std::size_t bestIndex = 0u;
    std::size_t i = 0u;

#if defined(__AVX2__)
    const __m256 qx8 = _mm256_set1_ps(qx);
    const __m256 qy8 = _mm256_set1_ps(qy);
    alignas(32) float distance8[8];
    for (; i + 8u <= soaMotions_.size(); i += 8u)
    {
        const __m256 dx = _mm256_sub_ps(_mm256_loadu_ps(soaX_.data() + i), qx8);
        const __m256 dy = _mm256_sub_ps(_mm256_loadu_ps(soaY_.data() + i), qy8);
        const __m256 distance = _mm256_add_ps(
            _mm256_mul_ps(dx, dx), _mm256_mul_ps(dy, dy));
        _mm256_store_ps(distance8, distance);

        for (std::size_t lane = 0; lane < 8u; ++lane)
        {
            if (distance8[lane] < bestDistance)
            {
                bestDistance = distance8[lane];
                bestIndex = i + lane;
            }
        }
    }
#endif

    for (; i < soaMotions_.size(); ++i)
    {
        const float dx = soaX_[i] - qx;
        const float dy = soaY_[i] - qy;
        const float distance = dx * dx + dy * dy;
        if (distance < bestDistance)
        {
            bestDistance = distance;
            bestIndex = i;
        }
    }

    return soaMotions_[bestIndex];
}

void ompl::control::RRTstarDI::getNeighborsSoA(
    Motion *query, double radius, std::vector<Motion *> &nbh) const
{
    nbh.clear();
    if (!query || soaMotions_.empty() || radius < 0.0)
        return;

    const float qx = static_cast<float>(query->cache.x);
    const float qy = static_cast<float>(query->cache.y);
    const float radiusFloat = static_cast<float>(radius);
    const float radiusSquared = radiusFloat * radiusFloat;
    std::size_t i = 0u;

#if defined(__AVX2__)
    const __m256 qx8 = _mm256_set1_ps(qx);
    const __m256 qy8 = _mm256_set1_ps(qy);
    const __m256 radius8 = _mm256_set1_ps(radiusSquared);
    for (; i + 8u <= soaMotions_.size(); i += 8u)
    {
        const __m256 dx = _mm256_sub_ps(_mm256_loadu_ps(soaX_.data() + i), qx8);
        const __m256 dy = _mm256_sub_ps(_mm256_loadu_ps(soaY_.data() + i), qy8);
        const __m256 distance = _mm256_add_ps(
            _mm256_mul_ps(dx, dx), _mm256_mul_ps(dy, dy));
        unsigned int mask = static_cast<unsigned int>(_mm256_movemask_ps(
            _mm256_cmp_ps(distance, radius8, _CMP_LE_OQ)));
        while (mask != 0u)
        {
            unsigned int lane = 0u;
            while (((mask >> lane) & 1u) == 0u)
                ++lane;
            nbh.push_back(soaMotions_[i + lane]);
            mask &= mask - 1u;
        }
    }
#endif

    for (; i < soaMotions_.size(); ++i)
    {
        const float dx = soaX_[i] - qx;
        const float dy = soaY_[i] - qy;
        if (dx * dx + dy * dy <= radiusSquared)
            nbh.push_back(soaMotions_[i]);
    }
}

void ompl::control::RRTstarDI::getNeighbors(Motion *motion, std::vector<Motion *> &nbh) const
{
    auto cardDbl = static_cast<double>(nearestNeighborCount() + 1u);
    if (useKNearest_)
    {
        const std::size_t k = std::min<std::size_t>(
            static_cast<std::size_t>(std::ceil(k_rrt_ * log(cardDbl))),
            soaMotions_.size());
        nbh.clear();
        if (k == 0u)
            return;

        const float qx = static_cast<float>(motion->cache.x);
        const float qy = static_cast<float>(motion->cache.y);
        soaDistanceScratch_.resize(soaMotions_.size());
        soaIndexScratch_.resize(soaMotions_.size());
        auto &distances = soaDistanceScratch_;
        auto &indices = soaIndexScratch_;
        std::size_t i = 0u;

#if defined(__AVX2__)
        const __m256 qx8 = _mm256_set1_ps(qx);
        const __m256 qy8 = _mm256_set1_ps(qy);
        for (; i + 8u <= soaMotions_.size(); i += 8u)
        {
            const __m256 dx = _mm256_sub_ps(_mm256_loadu_ps(soaX_.data() + i), qx8);
            const __m256 dy = _mm256_sub_ps(_mm256_loadu_ps(soaY_.data() + i), qy8);
            const __m256 distance = _mm256_add_ps(
                _mm256_mul_ps(dx, dx), _mm256_mul_ps(dy, dy));
            _mm256_storeu_ps(distances.data() + i, distance);
        }
#endif
        for (; i < soaMotions_.size(); ++i)
        {
            const float dx = soaX_[i] - qx;
            const float dy = soaY_[i] - qy;
            distances[i] = dx * dx + dy * dy;
        }
        for (std::size_t index = 0; index < indices.size(); ++index)
            indices[index] = index;

        auto closer = [&distances](std::size_t lhs, std::size_t rhs)
        {
            if (distances[lhs] != distances[rhs])
                return distances[lhs] < distances[rhs];
            return lhs < rhs;
        };
        if (k < indices.size())
            std::nth_element(indices.begin(), indices.begin() + k, indices.end(), closer);
        std::sort(indices.begin(), indices.begin() + k, closer);

        nbh.reserve(k);
        for (std::size_t rank = 0; rank < k; ++rank)
            nbh.push_back(soaMotions_[indices[rank]]);
    }
    else
    {
    	//=====================================銆愪慨鏀?4D---->2D�?==================================== 
    	double r = std::min(
	    maxDistance_,
	    r_rrt_ * std::pow(log(cardDbl) / cardDbl, 1.0 / 2.0));
//        double r = std::min(
//            maxDistance_, r_rrt_ * std::pow(log(cardDbl) / cardDbl, 1 / static_cast<double>(si_->getStateDimension())));
		//=====================================銆愪慨鏀?4D---->2D�?==================================== 
        getNeighborsSoA(motion, r, nbh);
    }
}

void ompl::control::RRTstarDI::removeFromParent(Motion *m)
{
    for (auto it = m->parent->children.begin(); it != m->parent->children.end(); ++it)
    {
        if (*it == m)
        {
            m->parent->children.erase(it);
            break;
        }
    }
}
void ompl::control::RRTstarDI::updateChildCosts(Motion *m)
{
    for (Motion *ch : m->children)
    {
        ch->cost = opt_->combineCosts(m->cost, ch->incCost);
        updateChildCosts(ch);
    }
}

void ompl::control::RRTstarDI::freeMemory()
{
    if (nn_)
    {
        std::vector<Motion *> motions;
        nn_->list(motions);
        for (auto &motion : motions)
        {
            freeMotionControls(motion);

            if (motion->state)
                si_->freeState(motion->state);

            delete motion;
        }
    }
    soaMotions_.clear();
    soaX_.clear();
    soaY_.clear();
    soaDistanceScratch_.clear();
    soaIndexScratch_.clear();
    motionCount_ = 0;
    improvingGoalCount_ = 0;
}

void ompl::control::RRTstarDI::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion *> motions;
    if (nn_)
        nn_->list(motions);

    if (bestGoalMotion_)
        data.addGoalVertex(base::PlannerDataVertex(bestGoalMotion_->state));

    for (auto &motion : motions)
    {
        if (motion->parent == nullptr)
            data.addStartVertex(base::PlannerDataVertex(motion->state));
        else
            data.addEdge(base::PlannerDataVertex(motion->parent->state), base::PlannerDataVertex(motion->state));
    }
}

int ompl::control::RRTstarDI::pruneTree(const base::Cost &pruneTreeCost)
{
    // Variable
    // The percent improvement (expressed as a [0,1] fraction) in cost
    double fracBetter;
    // The number pruned
    int numPruned = 0;

    if (opt_->isFinite(prunedCost_))
    {
        fracBetter = std::abs((pruneTreeCost.value() - prunedCost_.value()) / prunedCost_.value());
    }
    else
    {
        fracBetter = 1.0;
    }

    if (fracBetter > pruneThreshold_)
    {
        // We are only pruning motions if they, AND all descendents, have a estimated cost greater than pruneTreeCost
        // The easiest way to do this is to find leaves that should be pruned and ascend up their ancestry until a
        // motion is found that is kept.
        // To avoid making an intermediate copy of the NN structure, we process the tree by descending down from the
        // start(s).
        // In the first pass, all Motions with a cost below pruneTreeCost, or Motion's with children with costs below
        // pruneTreeCost are added to the replacement NN structure,
        // while all other Motions are stored as either a 'leaf' or 'chain' Motion. After all the leaves are
        // disconnected and deleted, we check
        // if any of the the chain Motions are now leaves, and repeat that process until done.
        // This avoids (1) copying the NN structure into an intermediate variable and (2) the use of the expensive
        // NN::remove() method.

        // Variable
        // The queue of Motions to process:
        std::queue<Motion *, std::deque<Motion *>> motionQueue;
        // The list of leaves to prune
        std::queue<Motion *, std::deque<Motion *>> leavesToPrune;
        // The list of chain vertices to recheck after pruning
        std::list<Motion *> chainsToRecheck;

        // Clear the NN structure:
        nn_->clear();
        motionCount_ = 0;

        // Put all the starts into the NN structure and their children into the queue:
        // We do this so that start states are never pruned.
        for (auto &startMotion : startMotions_)
        {
            // Add to the NN
            addMotionToNearestNeighbors(startMotion);

            // Add their children to the queue:
            addChildrenToList(&motionQueue, startMotion);
        }

        while (motionQueue.empty() == false)
        {
            // Test, can the current motion ever provide a better solution?
            if (keepCondition(motionQueue.front(), pruneTreeCost))
            {
                // Yes it can, so it definitely won't be pruned
                // Add it back into the NN structure
                addMotionToNearestNeighbors(motionQueue.front());

                // Add it's children to the queue
                addChildrenToList(&motionQueue, motionQueue.front());
            }
            else
            {
                // No it can't, but does it have children?
                if (motionQueue.front()->children.empty() == false)
                {
                    // Yes it does.
                    // We can minimize the number of intermediate chain motions if we check their children
                    // If any of them won't be pruned, then this motion won't either. This intuitively seems
                    // like a nice balance between following the descendents forever.

                    // Variable
                    // Whether the children are definitely to be kept.
                    bool keepAChild = false;

                    // Find if any child is definitely not being pruned.
                    for (unsigned int i = 0u; keepAChild == false && i < motionQueue.front()->children.size(); ++i)
                    {
                        // Test if the child can ever provide a better solution
                        keepAChild = keepCondition(motionQueue.front()->children.at(i), pruneTreeCost);
                    }

                    // Are we *definitely* keeping any of the children?
                    if (keepAChild)
                    {
                        // Yes, we are, so we are not pruning this motion
                        // Add it back into the NN structure.
                        addMotionToNearestNeighbors(motionQueue.front());
                    }
                    else
                    {
                        // No, we aren't. This doesn't mean we won't though
                        // Move this Motion to the temporary list
                        chainsToRecheck.push_back(motionQueue.front());
                    }

                    // Either way. add it's children to the queue
                    addChildrenToList(&motionQueue, motionQueue.front());
                }
                else
                {
                    // No, so we will be pruning this motion:
                    leavesToPrune.push(motionQueue.front());
                }
            }

            // Pop the iterator, std::list::erase returns the next iterator
            motionQueue.pop();
        }

        // We now have a list of Motions to definitely remove, and a list of Motions to recheck
        // Iteratively check the two lists until there is nothing to to remove
        while (leavesToPrune.empty() == false)
        {
            // First empty the current leaves-to-prune
            while (leavesToPrune.empty() == false)
            {
                // If this leaf is a goal, remove it from the goal set
                if (leavesToPrune.front()->inGoal == true)
                {
                    // Warn if pruning the _best_ goal
                    if (leavesToPrune.front() == bestGoalMotion_)
                    {
                        OMPL_ERROR("%s: Pruning the best goal.", getName().c_str());
                    }
                    // Remove it
                    goalMotions_.erase(std::remove(goalMotions_.begin(), goalMotions_.end(), leavesToPrune.front()),
                                       goalMotions_.end());
                }

                // Remove the leaf from its parent
                removeFromParent(leavesToPrune.front());
                unregisterMotionSoA(leavesToPrune.front());
                
                //淇�?
				freeMotionControls(leavesToPrune.front()); 

                // Erase the actual motion
                // First free the state
                si_->freeState(leavesToPrune.front()->state);

                // then delete the pointer
                delete leavesToPrune.front();

                // And finally remove it from the list, erase returns the next iterator
                leavesToPrune.pop();

                // Update our counter
                ++numPruned;
            }

            // Now, we need to go through the list of chain vertices and see if any are now leaves
            auto mIter = chainsToRecheck.begin();
            while (mIter != chainsToRecheck.end())
            {
                // Is the Motion a leaf?
                if ((*mIter)->children.empty() == true)
                {
                    // It is, add to the removal queue
                    leavesToPrune.push(*mIter);

                    // Remove from this queue, getting the next
                    mIter = chainsToRecheck.erase(mIter);
                }
                else
                {
                    // Is isn't, skip to the next
                    ++mIter;
                }
            }
        }

        // Now finally add back any vertices left in chainsToReheck.
        // These are chain vertices that have descendents that we want to keep
        for (const auto &r : chainsToRecheck)
        {
            // Add the motion back to the NN struct:
            addMotionToNearestNeighbors(r);
        }

        // All done pruning.
        // Update the cost at which we've pruned:
        prunedCost_ = pruneTreeCost;

        // And if we're using the pruned measure, the measure to which we've pruned
        if (usePrunedMeasure_)
        {
            prunedMeasure_ = infSampler_->getInformedMeasure(prunedCost_);

            if (useKNearest_ == false)
            {
                calculateRewiringLowerBounds();
            }
        }
        // No else, prunedMeasure_ is the si_ measure by default.
    }

    return numPruned;
}

void ompl::control::RRTstarDI::addChildrenToList(std::queue<Motion *, std::deque<Motion *>> *motionList, Motion *motion)
{
    for (auto &child : motion->children)
    {
        motionList->push(child);
    }
}

bool ompl::control::RRTstarDI::keepCondition(const Motion *motion, const base::Cost &threshold) const
{
    // We keep if the cost-to-come-heuristic of motion is <= threshold, by checking
    // if !(threshold < heuristic), as if b is not better than a, then a is better than, or equal to, b
    if (bestGoalMotion_ && motion == bestGoalMotion_)
    {
        // If the threshold is the theoretical minimum, the bestGoalMotion_ will sometimes fail the test due to floating point precision. Avoid that.
        return true;
    }

    return !opt_->isCostBetterThan(threshold, solutionHeuristic(motion));
}

ompl::base::Cost ompl::control::RRTstarDI::solutionHeuristic(const Motion *motion) const
{
    base::Cost costToCome;
    if (useAdmissibleCostToCome_)
    {
        // Start with infinite cost
        costToCome = opt_->infiniteCost();

        // Find the min from each start
        for (auto &startMotion : startMotions_)
        {
            costToCome = opt_->betterCost(
                costToCome, opt_->motionCost(startMotion->state,
                                             motion->state));  // lower-bounding cost from the start to the state
        }
    }
    else
    {
        costToCome = motion->cost;  // current cost from the state to the goal
    }

    const base::Cost costToGo =
        opt_->costToGo(motion->state, pdef_->getGoal().get());  // lower-bounding cost from the state to the goal
    return opt_->combineCosts(costToCome, costToGo);            // add the two costs
}

void ompl::control::RRTstarDI::setTreePruning(const bool prune)
{
    if (static_cast<bool>(opt_) == true)
    {
        if (opt_->hasCostToGoHeuristic() == false)
        {
            OMPL_INFORM("%s: No cost-to-go heuristic set. Informed techniques will not work well.", getName().c_str());
        }
    }

    // If we just disabled tree pruning, but we wee using prunedMeasure, we need to disable that as it required myself
    if (prune == false && getPrunedMeasure() == true)
    {
        setPrunedMeasure(false);
    }

    // Store
    useTreePruning_ = prune;
}

void ompl::control::RRTstarDI::setPrunedMeasure(bool informedMeasure)
{
    if (static_cast<bool>(opt_) == true)
    {
        if (opt_->hasCostToGoHeuristic() == false)
        {
            OMPL_INFORM("%s: No cost-to-go heuristic set. Informed techniques will not work well.", getName().c_str());
        }
    }

    // This option only works with informed sampling
    if (informedMeasure == true && (useInformedSampling_ == false || useTreePruning_ == false))
    {
        OMPL_ERROR("%s: InformedMeasure requires InformedSampling and TreePruning.", getName().c_str());
    }

    // Check if we're changed and update parameters if we have:
    if (informedMeasure != usePrunedMeasure_)
    {
        // Store the setting
        usePrunedMeasure_ = informedMeasure;

        // Update the prunedMeasure_ appropriately, if it has been configured.
        if (setup_ == true)
        {
            if (usePrunedMeasure_)
            {
                prunedMeasure_ = infSampler_->getInformedMeasure(prunedCost_);
            }
            else
            {
            	//=========================�?D------------->2d�?================== 
            	prunedMeasure_ = si_->getSpaceMeasure();
                //prunedMeasure_ = si_->getSpaceMeasure();
                //=========================�?D------------->2d�?================== 
            }
        }

        // And either way, update the rewiring radius if necessary
        if (useKNearest_ == false)
        {
            calculateRewiringLowerBounds();
        }
    }
}

void ompl::control::RRTstarDI::setInformedSampling(bool informedSampling)
{
    if (static_cast<bool>(opt_) == true)
    {
        if (opt_->hasCostToGoHeuristic() == false)
        {
            OMPL_INFORM("%s: No cost-to-go heuristic set. Informed techniques will not work well.", getName().c_str());
        }
    }

    // This option is mutually exclusive with setSampleRejection, assert that:
    if (informedSampling == true && useRejectionSampling_ == true)
    {
        OMPL_ERROR("%s: InformedSampling and SampleRejection are mutually exclusive options.", getName().c_str());
    }

    // If we just disabled tree pruning, but we are using prunedMeasure, we need to disable that as it required myself
    if (informedSampling == false && getPrunedMeasure() == true)
    {
        setPrunedMeasure(false);
    }

    // Check if we're changing the setting of informed sampling. If we are, we will need to create a new sampler, which
    // we only want to do if one is already allocated.
    if (informedSampling != useInformedSampling_)
    {
        // If we're disabled informedSampling, and prunedMeasure is enabled, we need to disable that
        if (informedSampling == false && usePrunedMeasure_ == true)
        {
            setPrunedMeasure(false);
        }

        // Store the value
        useInformedSampling_ = informedSampling;

        // If we currently have a sampler, we need to make a new one
        if (sampler_ || infSampler_)
        {
            // Reset the samplers
            sampler_.reset();
            infSampler_.reset();

            // Create the sampler
            allocSampler();
        }
    }
}

void ompl::control::RRTstarDI::setSampleRejection(const bool reject)
{
    if (static_cast<bool>(opt_) == true)
    {
        if (opt_->hasCostToGoHeuristic() == false)
        {
            OMPL_INFORM("%s: No cost-to-go heuristic set. Informed techniques will not work well.", getName().c_str());
        }
    }

    // This option is mutually exclusive with setInformedSampling, assert that:
    if (reject == true && useInformedSampling_ == true)
    {
        OMPL_ERROR("%s: InformedSampling and SampleRejection are mutually exclusive options.", getName().c_str());
    }

    // Check if we're changing the setting of rejection sampling. If we are, we will need to create a new sampler, which
    // we only want to do if one is already allocated.
    if (reject != useRejectionSampling_)
    {
        // Store the setting
        useRejectionSampling_ = reject;

        // If we currently have a sampler, we need to make a new one
        if (sampler_ || infSampler_)
        {
            // Reset the samplers
            sampler_.reset();
            infSampler_.reset();

            // Create the sampler
            allocSampler();
        }
    }
}

void ompl::control::RRTstarDI::setOrderedSampling(bool orderSamples)
{
    // Make sure we're using some type of informed sampling
    if (useInformedSampling_ == false && useRejectionSampling_ == false)
    {
        OMPL_ERROR("%s: OrderedSampling requires either informed sampling or rejection sampling.", getName().c_str());
    }

    // Check if we're changing the setting. If we are, we will need to create a new sampler, which we only want to do if
    // one is already allocated.
    if (orderSamples != useOrderedSampling_)
    {
        // Store the setting
        useOrderedSampling_ = orderSamples;

        // If we currently have a sampler, we need to make a new one
        if (sampler_ || infSampler_)
        {
            // Reset the samplers
            sampler_.reset();
            infSampler_.reset();

            // Create the sampler
            allocSampler();
        }
    }
}

void ompl::control::RRTstarDI::allocSampler()
{
    // Allocate the appropriate type of sampler.
    if (useInformedSampling_)
    {
        // We are using informed sampling, this can end-up reverting to rejection sampling in some cases
        OMPL_INFORM("%s: Using informed sampling.", getName().c_str());
        infSampler_ = opt_->allocInformedStateSampler(pdef_, numSampleAttempts_);
    }
    else if (useRejectionSampling_)
    {
        // We are explicitly using rejection sampling.
        OMPL_INFORM("%s: Using rejection sampling.", getName().c_str());
        infSampler_ = std::make_shared<base::RejectionInfSampler>(pdef_, numSampleAttempts_);
    }
    else
    {
        // We are using a regular sampler
        sampler_ = si_->allocStateSampler();
    }

    // Wrap into a sorted sampler
    if (useOrderedSampling_ == true)
    {
        infSampler_ = std::make_shared<base::OrderedInfSampler>(infSampler_, batchSize_);
    }
    // No else
}
//=================================================================�?D---------------------�?2D�?
bool ompl::control::RRTstarDI::sampleUniform(base::State *statePtr)
{
    bool ok = false;

    // 鍏堟寜鍘熼€昏緫閲囨�?
    if (useInformedSampling_ || useRejectionSampling_)
    {
        ok = infSampler_->sampleUniform(statePtr, bestCost_);
    }
    else
    {
        sampler_->sampleUniform(statePtr);
        ok = true;
    }

    if (!ok || !statePtr)
        return false;

    // 鍏抽敭淇敼�?    // 鍙繚鐣欎綅缃噰鏍锋剰涔夛紝鎶婇€熷害閲囨牱缁熶竴娓呴�?
    auto *rv = statePtr->as<ompl::base::RealVectorStateSpace::StateType>();
    rv->values[2] = 0.0;
    rv->values[3] = 0.0;

    return true;
}


void ompl::control::RRTstarDI::calculateRewiringLowerBounds()
{
    const double dimDbl = 2.0;
    const double measure = getXYMeasure();

    k_rrt_ = rewireFactor_ *
             (std::pow(2.0, dimDbl + 1.0) *
              boost::math::constants::e<double>() *
              (1.0 + 1.0 / dimDbl));

    r_rrt_ = rewireFactor_ *
             std::pow(2.0 * (1.0 + 1.0 / dimDbl) *
                      (measure / ompl::unitNBallMeasure(2)), 1.0 / dimDbl);
}

void ompl::control::RRTstarDI::clearControlSequence(std::vector<ControlSegment> &segments) const
{
    for (auto &seg : segments)
    {
        if (seg.control != nullptr)
            siC_->freeControl(seg.control);

        seg.control = nullptr;
        seg.duration = 0.0;
    }

    segments.clear();
}

void ompl::control::RRTstarDI::freeMotionControls(Motion *m) const
{
    if (!m)
        return;

    clearControlSequence(m->edgeControls);
}
//�����������ƺ�����ɢ�ɶ����ֵ���ƶΣ�����д�� PathControl
void ompl::control::RRTstarDI::buildControlSequenceForEdge(
    const base::State *s1,
    const base::State *s2,
    double t_opt,
    std::vector<ControlSegment> &segments) const
{
    clearControlSequence(segments);

    if (!(t_opt > 0.0) || !std::isfinite(t_opt))
        return;

    DIEdgePrimitive primitive;
    prepareEdgePrimitiveFixedVel(t_opt, toDIState(s1), toDIState(s2), primitive);

    const unsigned int N = std::max(1u, controlDiscretization_);
    const double dt = t_opt / static_cast<double>(N);
    segments.reserve(N);

    for (unsigned int i = 0; i < N; ++i)
    {
        const double tm = (static_cast<double>(i) + 0.5) * dt;
        double ux = 0.0;
        double uy = 0.0;
        evaluateEdgeControl(primitive, tm, ux, uy);

        auto *ctrl = siC_->allocControl();
        auto *rctrl = ctrl->as<ompl::control::RealVectorControlSpace::ControlType>();
        rctrl->values[0] = ux;
        rctrl->values[1] = uy;

        ControlSegment seg;
        seg.control = ctrl;
        seg.duration = dt;
        segments.push_back(seg);
    }
}
