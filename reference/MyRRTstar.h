/* Authors: Alejandro Perez, Sertac Karaman, Ryan Luna, Luis G. Torres,
   Ioan Sucan, Javier V Gomez, Jonathan Gammell */

#ifndef OMPL_CONTROL_PLANNERS_RRT_RRTSTAR_DI_CONTROL_
#define OMPL_CONTROL_PLANNERS_RRT_RRTSTAR_DI_CONTROL_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <utility>
#include <deque>
#include <functional>
#include <limits>
#include <list>
#include <memory>
#include <queue>
#include <string>
#include <vector>

#include <ompl/base/Planner.h>
#include <ompl/base/PlannerData.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/StateSampler.h>
#include <ompl/base/samplers/InformedStateSampler.h>

#include <ompl/control/SpaceInformation.h>
#include <ompl/control/PathControl.h>
#include <ompl/control/Control.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>

#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/util/RandomNumbers.h>

#include "di_primitive_types.h"
#include "di_edge_eval_block.h"
#include "fast_environment_2d.h"

namespace ompl
{
namespace control
{

class RRTstarDI : public base::Planner
{
public:
    RRTstarDI(const SpaceInformationPtr &si);
    ~RRTstarDI() override;

    void getPlannerData(base::PlannerData &data) const override;
    base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

    void clear() override;
    void setup() override;

    void setGoalBias(double goalBias)
    {
        goalBias_ = goalBias;
    }

    double getGoalBias() const
    {
        return goalBias_;
    }

    void setRange(double distance)
    {
        maxDistance_ = distance;
    }

    double getRange() const
    {
        return maxDistance_;
    }

    void setRewireFactor(double rewireFactor)
    {
        rewireFactor_ = rewireFactor;
        calculateRewiringLowerBounds();
    }

    double getRewireFactor() const
    {
        return rewireFactor_;
    }

    template <template <typename T> class NN>
    void setNearestNeighbors()
    {
        if (nn_ && nearestNeighborCount() != 0)
            OMPL_WARN("Calling setNearestNeighbors will clear all states.");
        clear();
        nn_ = std::make_shared<NN<Motion *>>();
        setup();
    }

    void setDelayCC(bool delayCC)
    {
        delayCC_ = delayCC;
    }

    bool getDelayCC() const
    {
        return delayCC_;
    }

    void setTreePruning(bool prune);
    bool getTreePruning() const
    {
        return useTreePruning_;
    }

    void setPruneThreshold(double pp)
    {
        pruneThreshold_ = pp;
    }

    double getPruneThreshold() const
    {
        return pruneThreshold_;
    }

    void setPrunedMeasure(bool informedMeasure);
    bool getPrunedMeasure() const
    {
        return usePrunedMeasure_;
    }

    void setInformedSampling(bool informedSampling);
    bool getInformedSampling() const
    {
        return useInformedSampling_;
    }

    void setSampleRejection(bool reject);
    bool getSampleRejection() const
    {
        return useRejectionSampling_;
    }

    void setNewStateRejection(bool reject)
    {
        useNewStateRejection_ = reject;
    }

    bool getNewStateRejection() const
    {
        return useNewStateRejection_;
    }

    void setAdmissibleCostToCome(bool admissible)
    {
        useAdmissibleCostToCome_ = admissible;
    }

    bool getAdmissibleCostToCome() const
    {
        return useAdmissibleCostToCome_;
    }

    void setOrderedSampling(bool orderSamples);
    bool getOrderedSampling() const
    {
        return useOrderedSampling_;
    }

    void setBatchSize(unsigned int batchSize)
    {
        batchSize_ = batchSize;
    }

    unsigned int getBatchSize() const
    {
        return batchSize_;
    }

    void setFocusSearch(bool focus)
    {
        setInformedSampling(focus);
        setTreePruning(focus);
        setPrunedMeasure(focus);
        setNewStateRejection(focus);
    }

    bool getFocusSearch() const
    {
        return getInformedSampling() && getPrunedMeasure() &&
               getTreePruning() && getNewStateRejection();
    }

    void setKNearest(bool useKNearest)
    {
        useKNearest_ = useKNearest;
    }

    bool getKNearest() const
    {
        return useKNearest_;
    }

    void setNumSamplingAttempts(unsigned int numAttempts)
    {
        numSampleAttempts_ = numAttempts;
    }

    unsigned int getNumSamplingAttempts() const
    {
        return numSampleAttempts_;
    }

    unsigned int numIterations() const
    {
        return iterations_;
    }

    base::Cost bestCost() const
    {
        return bestCost_;
    }

    void setMaxSampleCount(unsigned int n)
    {
        maxSampleCount_ = n;
    }

    unsigned int getMaxSampleCount() const
    {
        return maxSampleCount_;
    }

    unsigned int getSampleCount() const
    {
        return sampleCount_;
    }

    void setNumCollisionCheckPoints(unsigned int n)
    {
        numCollisionCheckPoints_ = n;
    }

    unsigned int getNumCollisionCheckPoints() const
    {
        return numCollisionCheckPoints_;
    }

    void setTimeSolverTolerance(double tol)
    {
        timeSolverTolerance_ = tol;
    }

    double getTimeSolverTolerance() const
    {
        return timeSolverTolerance_;
    }

    void setTimeSolverWork(unsigned int scanIntervals,
                           unsigned int newtonIterations,
                           unsigned int bisectionIterations)
    {
        rootScanIntervals_ = std::max(4u, scanIntervals);
        rootNewtonIterations_ = newtonIterations;
        rootBisectionIterations_ = bisectionIterations;
    }

    unsigned int getRootScanIntervals() const
    {
        return rootScanIntervals_;
    }

    unsigned int getRootNewtonIterations() const
    {
        return rootNewtonIterations_;
    }

    unsigned int getRootBisectionIterations() const
    {
        return rootBisectionIterations_;
    }

    void setMinTrajectoryTime(double t)
    {
        minTrajectoryTime_ = t;
    }

    void setMaxTrajectoryTime(double t)
    {
        maxTrajectoryTime_ = t;
    }

    double getMinTrajectoryTime() const
    {
        return minTrajectoryTime_;
    }

    double getMaxTrajectoryTime() const
    {
        return maxTrajectoryTime_;
    }

    void setControlDiscretization(unsigned int n)
    {
        controlDiscretization_ = n;
    }

    unsigned int getControlDiscretization() const
    {
        return controlDiscretization_;
    }

    using FastValidityChecker =
        std::function<bool(double x, double y, double vx, double vy)>;

    using FastBatchValidityChecker =
        std::function<bool(const double *x,
                           const double *y,
                           const double *vx,
                           const double *vy,
                           unsigned int count)>;

    void setFastValidityChecker(FastValidityChecker checker)
    {
        fastValidityChecker_ = std::move(checker);
    }

    void setFastBatchValidityChecker(FastBatchValidityChecker checker)
    {
        fastBatchValidityChecker_ = std::move(checker);
    }

    void clearFastEnvironment()
    {
        fastEnvironment_.clear();
    }

    void setFastPositionBounds(double xmin, double xmax, double ymin, double ymax)
    {
        fastEnvironment_.setPositionBounds(xmin, xmax, ymin, ymax);
    }

    void setFastVelocityBounds(double vxmin, double vxmax, double vymin, double vymax)
    {
        fastEnvironment_.setVelocityBounds(vxmin, vxmax, vymin, vymax);
    }

    void addFastCircleObstacle(double x, double y, double radius)
    {
        fastEnvironment_.addCircle(x, y, radius);
    }

    void addFastRectObstacle(double xmin, double xmax, double ymin, double ymax)
    {
        fastEnvironment_.addRect(xmin, xmax, ymin, ymax);
    }

    bool hasFastValidityChecker() const
    {
        return static_cast<bool>(fastValidityChecker_) ||
               static_cast<bool>(fastBatchValidityChecker_) ||
               !fastEnvironment_.empty();
    }

    void DI_stateFreeVel_out(double t, double t_s,
                             double x01, double x02, double x03, double x04,
                             double x11, double x12,
                             double &x, double &y, double &vx, double &vy) const;

    double DI_costFreeVel(double t_s,
                          double x01, double x02, double x03, double x04,
                          double x11, double x12) const;

    std::vector<double> DI_stateFreeVel(double t, double t_s,
                                        double x01, double x02,
                                        double x03, double x04,
                                        double x11, double x12) const;

    bool computeOptimalTimeFreeVel(const base::State *s1,
                                   const base::State *s2,
                                   double &t_opt) const;

    base::Cost dynamicMotionCost(const base::State *s1,
                                 const base::State *s2) const;

    bool checkDynamicMotionWithTimeFast(const base::State *s1,
                                        const base::State *s2,
                                        double t_opt,
                                        base::State *fallbackScratch) const;

    bool isDynamicMotionValid(const base::State *s1,
                              const base::State *s2) const;

    void extractStateComponents(const base::State *state,
                                double &x, double &y,
                                double &vx, double &vy) const;

    void setStateComponents(base::State *state,
                            double x, double y,
                            double vx, double vy) const;
                            
     

protected:
    using DIState = detail::DIState;
    using EdgeMode = detail::EdgeMode;
    template <std::size_t W>
    using DIStateBlock = detail::DIStateBlock<W>;
    using FastEnvironment2D = detail::FastEnvironment2D;
    using QuarticCoefficients = std::array<double, 5>;

    inline static constexpr std::size_t PrimitiveBatchWidth = 8u;
    // AVX2 + float uses eight 32-bit lanes for candidate blocks and one-trajectory rakes.
    inline static constexpr std::size_t RakeBatchWidth = PrimitiveBatchWidth;
    static_assert(PrimitiveBatchWidth == 8u, "The float AVX2 backend requires eight lanes");
    using PrimitiveStateBlock = DIStateBlock<PrimitiveBatchWidth>;

    using DIEdgePrimitive = detail::DIEdgePrimitive;
    auto makeTimeFreeVelCoefficients(double x01, double x02,
                                     double x03, double x04,
                                     double x11, double x12) const
        -> QuarticCoefficients;
    auto makeTimeFixedVelCoefficients(const DIState &fromState,
                                      const DIState &targetState) const
        -> QuarticCoefficients;
    bool computeBestTimeFromQuartic(
        const QuarticCoefficients &coeffs,
        const std::function<double(double)> &costAtTime,
        double &tOpt,
        double &bestCost) const;
    bool computeBestTimeFreeVelFromDIStates(const DIState &fromState,
                                            const DIState &targetState,
                                            double &tOpt,
                                            double &bestCost) const;
    bool computeBestTimeFixedVelFromDIStates(const DIState &fromState,
                                             const DIState &targetState,
                                             double &tOpt,
                                             double &bestCost) const;
    double DI_costFixedVel(double t,
                           const DIState &fromState,
                           const DIState &targetState) const;

    void prepareEdgePrimitive(double t_opt,
                              double x01, double x02, double x03, double x04,
                              double x11, double x12,
                              DIEdgePrimitive &primitive) const;
    void prepareEdgePrimitiveFixedVel(double t_opt,
                                      const DIState &fromState,
                                      const DIState &targetState,
                                      DIEdgePrimitive &primitive) const;
    void evaluateEdgeState(const DIEdgePrimitive &primitive,
                           double t,
                           double &x, double &y,
                           double &vx, double &vy) const;
    auto evaluateEdgeState(const DIEdgePrimitive &primitive,
                           double t) const -> DIState;
    auto evaluateEdgeStateByMode(const DIEdgePrimitive &primitive,
                                 double t) const -> DIState;
    void evaluateEdgeStateBlock(const DIEdgePrimitive &primitive,
                                const std::array<double, PrimitiveBatchWidth> &times,
                                unsigned int count,
                                PrimitiveStateBlock &block) const;
    void evaluateEdgeStateBlockByMode(
        const DIEdgePrimitive &primitive,
        const std::array<double, PrimitiveBatchWidth> &times,
        unsigned int count,
        PrimitiveStateBlock &block) const;
    void evaluateEdgeControl(const DIEdgePrimitive &primitive,
                             double t,
                             double &ux,
                             double &uy) const;
    auto toDIState(const base::State *state) const -> DIState;
    void fromDIState(base::State *state, const DIState &diState) const;
    auto isFiniteState(const DIState &state) const -> bool;
    auto checkPrimitiveState(const DIState &state, base::State *fallbackScratch) const -> bool;
    auto checkPrimitiveStateBlock(
        const PrimitiveStateBlock &block,
        unsigned int count,
        base::State *fallbackScratch) const -> bool;
    auto validatePreparedMotionPrimitive(
        const DIEdgePrimitive &primitive,
        const DIState &start,
        const DIState &goal,
        base::State *fallbackScratch) const -> bool;
    bool checkFreeVelMotionWithTime(const DIEdgePrimitive &primitive,
                                    const DIState &start,
                                    const DIState &goal,
                                    base::State *fallbackScratch) const;
    bool checkFixedVelMotionWithTime(const DIEdgePrimitive &primitive,
                                     const DIState &start,
                                     const DIState &goal,
                                     base::State *fallbackScratch) const;

    struct ControlSegment
    {
        Control *control{nullptr};
        double duration{0.0};
    };

    class DICostObjective : public base::OptimizationObjective
    {
    public:
        explicit DICostObjective(const base::SpaceInformationPtr &si)
            : base::OptimizationObjective(si)
        {
            description_ = "RRTstarDI internal cost objective";
        }

        base::Cost stateCost(const base::State *) const override
        {
            return identityCost();
        }

        base::Cost motionCost(const base::State *, const base::State *) const override
        {
            // 真正边代价由 planner 内部显式计算写入 motion->incCost
            return identityCost();
        }

        base::Cost identityCost() const override
        {
            return base::Cost(0.0);
        }

        base::Cost infiniteCost() const override
        {
            return base::Cost(std::numeric_limits<double>::infinity());
        }

        base::Cost combineCosts(base::Cost c1, base::Cost c2) const override
        {
            return base::Cost(c1.value() + c2.value());
        }

        bool isCostBetterThan(base::Cost c1, base::Cost c2) const override
        {
            return c1.value() < c2.value();
        }

        bool isSatisfied(base::Cost c) const override
        {
            return c.value() <= getCostThreshold().value();
        }
    };

    class Motion
    {
    public:
        explicit Motion(const SpaceInformationPtr &si)
            : state(si->allocState()), parent(nullptr), inGoal(false)
        {
        }

        ~Motion() = default;

        base::State *state{nullptr};
        DIState cache{};
        Motion *parent{nullptr};
        bool inGoal{false};
        base::Cost cost;
        base::Cost incCost;
        std::vector<Motion *> children;

        double duration{0.0};
        std::vector<ControlSegment> edgeControls;
        std::size_t soaIndex{std::numeric_limits<std::size_t>::max()};
    };

    void allocSampler();
    bool sampleUniform(base::State *statePtr);
    void freeMemory();

    bool isControlSequenceWithinBounds(const std::vector<ControlSegment> &segments) const;
    bool isEdgeControlWithinBounds(const DIEdgePrimitive &primitive) const;
    void clearControlSequence(std::vector<ControlSegment> &segments) const;
    void freeMotionControls(Motion *m) const;
    void buildControlSequenceForEdge(const base::State *s1,
                                     const base::State *s2,
                                     double t_opt,
                                     std::vector<ControlSegment> &segments) const;


	
    using EdgeEval = detail::EdgeEval;

	// FreeVel edge solver: time, cost, terminal state and primitive are produced together.
	bool solveFreeVelEdge(const base::State *from,
	                      const base::State *targetRef,
	                      EdgeEval &eval,
	                      base::State *outState) const;
	bool solveFreeVelEdgeFromDIStates(const DIState &fromState,
	                                  const DIState &targetState,
	                                  EdgeEval &eval,
	                                  base::State *outState) const;
	// FixedVel edge solver: both endpoint positions and velocities are constrained.
	bool solveFixedVelEdgeFromDIStates(const DIState &fromState,
	                                   const DIState &targetState,
	                                   EdgeEval &eval,
	                                   base::State *outState) const;

    using EdgeEvalBlock = detail::EdgeEvalBlock<PrimitiveBatchWidth>;

    void evaluateEdgeBasicBlock8(
        const std::array<DIState, PrimitiveBatchWidth> &fromStates,
        const std::array<DIState, PrimitiveBatchWidth> &targetStates,
        std::size_t count,
        EdgeEvalBlock &block,
        std::array<base::State *, PrimitiveBatchWidth> &outStates) const;

    void evaluateParentCandidateBlock(
        Motion *const *parents,
        std::size_t count,
        const base::State *targetRef,
        EdgeEvalBlock &block,
        std::array<base::State *, PrimitiveBatchWidth> &outStates) const
    {
        const std::size_t blockCount = std::min<std::size_t>(count, PrimitiveBatchWidth);
        const DIState targetState = toDIState(targetRef);
        std::array<DIState, PrimitiveBatchWidth> fromStates;
        std::array<DIState, PrimitiveBatchWidth> targetStates;

        for (std::size_t i = 0; i < blockCount; ++i)
        {
            fromStates[i] = parents[i]->cache;
            targetStates[i] = targetState;
        }

        evaluateEdgeBasicBlock8(fromStates, targetStates, blockCount, block, outStates);
    }

    void evaluateRewireCandidateBlock(
        const base::State *from,
        Motion *const *targets,
        std::size_t count,
        EdgeEvalBlock &block) const;
                       
    struct CostIndexCompare
    {
        CostIndexCompare(const std::vector<base::Cost> &costs,
                         const base::OptimizationObjective &opt)
            : costs_(costs), opt_(opt)
        {
        }

        bool operator()(std::size_t i, std::size_t j) const
        {
            return opt_.isCostBetterThan(costs_[i], costs_[j]);
        }

        const std::vector<base::Cost> &costs_;
        const base::OptimizationObjective &opt_;
    };

    double distanceFunction(const Motion *a, const Motion *b) const
    {
        const double dx = a->cache.x - b->cache.x;
        const double dy = a->cache.y - b->cache.y;
        return std::sqrt(dx * dx + dy * dy);
    }
    
    // =========================【修改 1】========================= 
     //4d---->2D                      
    double motionDistanceXY(const Motion *a, const Motion *b) const;
    double motionDistanceXYSquared(const Motion *a, const Motion *b) const;
    
    void interpolateXY(const base::State *from,
                                             const base::State *to,
                                             double maxDist,
                                             base::State *out) const;
    double getXYMeasure() const;
    // =========================【修改 1】=========================

    void getNeighbors(Motion *motion, std::vector<Motion *> &nbh) const;
    void addMotionToNearestNeighbors(Motion *motion);
    void registerMotionSoA(Motion *motion);
    void unregisterMotionSoA(Motion *motion);
    void updateMotionSoA(Motion *motion);
    Motion *nearestNeighborSoA(const Motion *query) const;
    void getNeighborsSoA(Motion *query, double radius, std::vector<Motion *> &nbh) const;
    std::size_t nearestNeighborCount() const;
    void removeFromParent(Motion *m);
    void updateChildCosts(Motion *m);
    int pruneTree(const base::Cost &pruneTreeCost);
    base::Cost solutionHeuristic(const Motion *motion) const;
    void addChildrenToList(std::queue<Motion *, std::deque<Motion *>> *motionList, Motion *motion);
    bool keepCondition(const Motion *motion, const base::Cost &threshold) const;
    void calculateRewiringLowerBounds();

    base::StateSamplerPtr sampler_;
    base::InformedSamplerPtr infSampler_;
    std::shared_ptr<NearestNeighbors<Motion *>> nn_;

    SpaceInformationPtr siC_;

    unsigned int maxSampleCount_{0};
    unsigned int sampleCount_{0};
    std::size_t motionCount_{0};

    unsigned int numCollisionCheckPoints_{30};
    double timeSolverTolerance_{1e-5};
    unsigned int rootScanIntervals_{32};
    unsigned int rootNewtonIterations_{8};
    unsigned int rootBisectionIterations_{12};
    double minTrajectoryTime_{0.1};
    double maxTrajectoryTime_{50.0};

    unsigned int controlDiscretization_{10};
    FastValidityChecker fastValidityChecker_;
    FastBatchValidityChecker fastBatchValidityChecker_;
    FastEnvironment2D fastEnvironment_;

    double goalBias_{0.05};
    double maxDistance_{0.0};
    RNG rng_;
    bool useKNearest_{true};
    double rewireFactor_{1.1};
    double k_rrt_{0.0};
    double r_rrt_{0.0};
    bool delayCC_{true};

    base::OptimizationObjectivePtr opt_;
    Motion *bestGoalMotion_{nullptr};
    std::vector<Motion *> goalMotions_;
    unsigned int improvingGoalCount_{0};

    bool useTreePruning_{false};
    double pruneThreshold_{0.05};
    bool usePrunedMeasure_{false};
    bool useInformedSampling_{false};
    bool useRejectionSampling_{false};
    bool useNewStateRejection_{false};
    bool useAdmissibleCostToCome_{true};

    unsigned int numSampleAttempts_{100u};
    bool useOrderedSampling_{false};
    unsigned int batchSize_{1u};

    std::vector<Motion *> startMotions_;

    // XY-only SoA cache used by the 2D AVX2 nearest and radius scans. The
    // arrays stay compact; deleting a motion swaps the last live slot in.
    std::vector<Motion *> soaMotions_;
    std::vector<float> soaX_;
    std::vector<float> soaY_;
    mutable std::vector<float> soaDistanceScratch_;
    mutable std::vector<std::size_t> soaIndexScratch_;

    base::Cost bestCost_{std::numeric_limits<double>::quiet_NaN()};
    base::Cost prunedCost_{std::numeric_limits<double>::quiet_NaN()};
    double prunedMeasure_{0.0};
    unsigned int iterations_{0u};

    std::string numIterationsProperty() const
    {
        return std::to_string(numIterations());
    }

    std::string bestCostProperty() const
    {
        return std::to_string(bestCost().value());
    }
};

}  // namespace control
}  // namespace ompl

#endif
