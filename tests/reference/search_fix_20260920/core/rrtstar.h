#pragma once
#include "common/planner_config.h"
#include "common/timer.h"
#include "core/nearest_neighbor.h"
#include "core/node.h"
#include "core/rewiring.h"
#include "core/sampler.h"
#include "core/tree.h"
#include "pff/pff.h"
#include "simd/simd_pff.h"
#include <algorithm>
#include <cmath>
#include <type_traits>
#include <utility>

namespace rrtpff {
template <class M, class = void> struct HasExpansionConstraint : std::false_type {};
template <class M>
struct HasExpansionConstraint<M, std::void_t<decltype(std::declval<const M &>().expansionConstraint(
                                     std::declval<const typename M::FlatOutput &>()))>>
    : std::true_type {};
template <class M, class = void> struct HasTrajectoryScreen : std::false_type {};
template <class M, class = void> struct HasGoalConstraint : std::false_type {};
template <class M>
struct HasGoalConstraint<M, std::void_t<decltype(std::declval<const M &>().goalConstraint())>>
    : std::true_type {};
template <class M, class = void> struct HasAlternativeConstraint : std::false_type {};
template <class M>
struct HasAlternativeConstraint<
    M,
    std::void_t<decltype(std::declval<const M &>().alternativeExpansionConstraint(
        std::declval<const typename M::FlatState &>(), std::declval<const typename M::Metadata &>(),
        std::declval<const typename M::FlatOutput &>()))>> : std::true_type {};
template <class M>
struct HasTrajectoryScreen<M, std::void_t<decltype(std::declval<const M &>().screenTrajectory(
                                  std::declval<const typename M::FlatTrajectory &>(), std::size_t{},
                                  bool{}, std::declval<PlanResult &>()))>> : std::true_type {};
template <class SystemModelT> class RRTstar {
  public:
    using Model = SystemModelT;
    using State = typename Model::FlatState;
    using Output = typename Model::FlatOutput;
    using Metadata = typename Model::Metadata;
    using Node = RRTNode<Model::FlatDim, Model::FlatOrder, Metadata>;
    using Solver = PFFSolver<Model::FlatDim, Model::FlatOrder, Metadata>;
    using Constraint = typename Solver::Constraint;
    using Edge = typename Solver::Result;
    using Trajectory = typename Node::Trajectory;
    static constexpr std::size_t BatchWidth = 8;
    using BatchSolver = SIMD_PFF<Model::FlatDim, Model::FlatOrder, Metadata, BatchWidth>;

    RRTstar(Model model, PlannerConfig config, Solver pff = Solver{})
        : model_(std::move(model)), config_(config), batch_pff_(std::move(pff)) {
        config_.validate();
    }

    PlanResult solve() {
        tree_.clear();
        goals_.clear();
        path_.clear();
        stats_ = {};
        stats_.simd_active = config_.simd_enabled && simdRuntimeAVX2();

        Timer wall;

        if (!model_.initialStateValid() || !model_.goalOutputValid()) {
            stats_.failure_reason = "invalid_endpoint";
            stats_.planning_time = wall.seconds();
            return stats_;
        }

        Node *root = tree_.createNode(model_.start(), nullptr, {}, model_.initialMetadata());
        if (model_.isGoal(root->state, config_.goal_tolerance)) {
            goals_.push_back(root);
            stats_.first_solution_time = wall.seconds();
        }

        Sampler<Model> sampler(model_, config_.seed);
        NearestNeighbor<Node, Model> nn(tree_, model_, config_.simd_enabled && config_.simd_nn);
        std::vector<Node *> parents;
        std::vector<Candidate> candidates;

        while (wall.seconds() < config_.timeout && stats_.iterations < config_.max_iterations &&
               tree_.size() < config_.max_nodes) {
            if (config_.stop_at_first_solution && !goals_.empty())
                break;

            ++stats_.iterations;
            bool sampled_goal = false;
            const Output target = sampler.sampleFlatOutput(config_.goal_bias, &sampled_goal);
            const bool goal_connection =
                sampled_goal && config_.explicit_goal_connection && HasGoalConstraint<Model>::value;

            // Select the nearest parent to the sample before limiting extension.
            Timer nn_timer;
            Node *nearest = nn.nearest(target);
            const Output limited =
                goal_connection ? target
                                : model_.limitExtension(nearest->state, target, config_.range);

            parents.clear();
            const double n = static_cast<double>(tree_.size() + 1);
            const double radius = config_.rewire_factor * config_.range *
                                  std::pow(std::log(n) / n, 1.0 / Model::FlatDim);
            nn.near(limited, radius, parents);
            if (std::find(parents.begin(), parents.end(), nearest) == parents.end())
                parents.push_back(nearest);
            if (goal_connection) {
                // Bounded goal-parent set: consider distinct full states, not
                // just repeatedly trying the one closest position.
                parents.clear();
                for (const auto &node : tree_.nodes())
                    parents.push_back(node.get());
                const auto keep = std::min(config_.goal_parent_limit, parents.size());
                std::partial_sort(parents.begin(), parents.begin() + keep, parents.end(),
                                  [&](Node *a, Node *b) {
                                      const double da = model_.distanceToOutput(a->state, target),
                                                   db = model_.distanceToOutput(b->state, target);
                                      return da == db ? a->id < b->id : da < db;
                                  });
                parents.resize(keep);
                stats_.goal_connection_queries += parents.size();
            }

            stats_.nn_time += nn_timer.seconds();
            candidates.clear();
            for (std::size_t offset = 0; offset < parents.size(); offset += BatchWidth) {
                if (wall.seconds() >= config_.timeout)
                    break;
                const auto count = std::min(BatchWidth, parents.size() - offset);
                std::array<Node *, BatchWidth> block{};
                std::array<Constraint, BatchWidth> targets{};
                for (std::size_t i = 0; i < count; ++i) {
                    block[i] = parents[offset + i];
                    if constexpr (HasExpansionConstraint<Model>::value)
                        targets[i] = model_.expansionConstraint(limited);
                    else
                        targets[i] = Constraint::output(limited);
                    if constexpr (HasGoalConstraint<Model>::value)
                        if (goal_connection)
                            targets[i] = model_.goalConstraint();
                }
                appendBatch(block, targets, count, candidates);
                if constexpr (HasAlternativeConstraint<Model>::value) {
                    if (!goal_connection) {
                        std::array<Node *, BatchWidth> alternatives{};
                        std::array<Constraint, BatchWidth> alternative_targets{};
                        std::size_t used = 0;
                        for (std::size_t i = 0; i < count; ++i) {
                            auto alternative = model_.alternativeExpansionConstraint(
                                block[i]->state, block[i]->metadata, limited);
                            if (alternative) {
                                alternatives[used] = block[i];
                                alternative_targets[used++] = *alternative;
                            }
                        }
                        if (used)
                            appendBatch(alternatives, alternative_targets, used, candidates);
                    }
                }
            }
            sortCandidates(candidates);

            Node *added = nullptr;
            const std::size_t count = std::min(config_.top_k, candidates.size());
            for (std::size_t i = 0; i < count && wall.seconds() < config_.timeout; ++i) {
                auto &candidate = candidates[i];
                if (!validateEdge(candidate.edge.trajectory))
                    continue;
                if (goal_connection)
                    ++stats_.goal_connection_valid;
                added = tree_.createNode(candidate.edge.terminal_state, candidate.parent,
                                         candidate.edge.trajectory);
                added->metadata = model_.terminalMetadata(candidate.edge.trajectory);
                break;
            }
            if (!added)
                continue;

            if (model_.isGoal(added->state, config_.goal_tolerance)) {
                goals_.push_back(added);
                if (!std::isfinite(stats_.first_solution_time))
                    stats_.first_solution_time = wall.seconds();
            }

            if (config_.rewire_before_first_solution || !goals_.empty())
                rewire(added, parents, wall);

            if (config_.prune)
                pruneTree();
        }

        Node *best = bestGoal();
        stats_.node_count = tree_.size();
        stats_.simd_nn_lanes = nn.vectorEvaluations();
        stats_.simd_eval_count =
            stats_.simd_pff_lanes + stats_.simd_flat_samples + stats_.simd_nn_lanes;
        stats_.planning_time = wall.seconds();
        for (const auto &node : tree_.nodes())
            stats_.closest_goal_distance =
                std::min(stats_.closest_goal_distance,
                         model_.distanceToOutput(node->state, model_.goalOutput()));
        if (!best) {
            if (stats_.iterations >= config_.max_iterations)
                stats_.failure_reason = "max_iterations_no_goal";
            else if (tree_.size() >= config_.max_nodes)
                stats_.failure_reason = "max_nodes_no_goal";
            else
                stats_.failure_reason = "timeout_no_goal";
        }

        if (best) {
            stats_.success = true;
            stats_.cost = best->cost;
            for (Node *n = best; n->parent; n = n->parent)
                path_.push_back(n->incoming);
            std::reverse(path_.begin(), path_.end());

            for (const auto &edge : path_) {
                stats_.duration += edge.T;
                auto previous = edge.evaluate(0).output();
                for (int i = 1; i <= 100; ++i) {
                    const auto p = edge.evaluate(edge.T * i / 100).output();
                    stats_.path_length += (p - previous).norm();
                    previous = p;
                }
            }
        }
        return stats_;
    }

    const std::vector<Trajectory> &path() const { return path_; }

    const RRTTree<Node> &tree() const { return tree_; }

  private:
    struct Candidate {
        Node *parent;
        Edge edge;
        double total;
        std::size_t sequence;
    };

    void appendBatch(const std::array<Node *, BatchWidth> &parents,
                     const std::array<Constraint, BatchWidth> &targets, std::size_t count,
                     std::vector<Candidate> &out) {
        Timer timer;
        std::array<State, BatchWidth> states{};
        for (std::size_t i = 0; i < count; ++i)
            states[i] = parents[i]->state;
        const auto before = stats_.simd_pff_lanes;
        auto edges = batch_pff_.candidatesBatch(states, targets, count,
                                                config_.simd_enabled && config_.simd_pff,
                                                &stats_.simd_pff_lanes);
        const double elapsed = timer.seconds();
        stats_.pff_time += elapsed;
        if (stats_.simd_pff_lanes > before)
            stats_.simd_time += elapsed;
        stats_.pff_query_count += count;
        for (std::size_t i = 0; i < count; ++i) {
            stats_.pff_eval_count += edges[i].size();
            if (edges[i].empty())
                ++stats_.pff_rejected;
            for (auto &edge : edges[i]) {
                if (!model_.prepareEdge(parents[i]->state, parents[i]->metadata, edge.trajectory)) {
                    ++stats_.rejected_candidates;
                    continue;
                }
                const double total = parents[i]->cost + edge.cost;
                out.push_back({parents[i], std::move(edge), total, out.size()});
            }
        }
    }

    void sortCandidates(std::vector<Candidate> &c) const {
        const auto count = std::min(config_.top_k, c.size());
        std::partial_sort(c.begin(), c.begin() + count, c.end(), [](const auto &a, const auto &b) {
            return a.total == b.total ? a.sequence < b.sequence : a.total < b.total;
        });
    }

    bool validateEdge(const Trajectory &tr) {
        if constexpr (HasTrajectoryScreen<Model>::value) {
            if (stats_.simd_active && config_.simd_screen) {
                Timer timer;
                const bool keep = model_.screenTrajectory(tr, config_.collision_samples,
                                                          config_.interleaved_samples, stats_);
                const double elapsed = timer.seconds();
                stats_.screen_time += elapsed;
                stats_.simd_time += elapsed;
                if (!keep) {
                    ++stats_.screen_rejected;
                    return false;
                }
            }
        }
        Timer timer;
        const bool valid = model_.trajectoryValid(tr, stats_);
        stats_.validation_time += timer.seconds();
        return valid;
    }

    void rewire(Node *added, const std::vector<Node *> &near, const Timer &wall) {
        std::array<Node *, BatchWidth> nodes{};
        std::array<Constraint, BatchWidth> targets{};
        std::size_t count = 0;
        auto flush = [&] {
            if (!count)
                return;
            std::array<State, BatchWidth> states{};
            for (std::size_t i = 0; i < count; ++i)
                states[i] = added->state;
            Timer timer;
            const auto before = stats_.simd_pff_lanes;
            auto results = batch_pff_.candidatesBatch(states, targets, count,
                                                      config_.simd_enabled && config_.simd_pff,
                                                      &stats_.simd_pff_lanes);
            const double elapsed = timer.seconds();
            stats_.pff_time += elapsed;
            if (stats_.simd_pff_lanes > before)
                stats_.simd_time += elapsed;
            stats_.pff_query_count += count;
            for (std::size_t lane = 0; lane < count && wall.seconds() < config_.timeout; ++lane) {
                Node *node = nodes[lane];
                stats_.pff_eval_count += results[lane].size();
                std::size_t tried = 0;
                for (auto &edge : results[lane]) {
                    // Node state and branch never change. Descendant edges are
                    // reused; only cumulative costs propagate after commit.
                    if (added->cost + edge.cost >= node->cost - 1e-10)
                        break;
                    if (!model_.prepareEdge(added->state, added->metadata, edge.trajectory))
                        continue;
                    if (tried++ >= config_.top_k)
                        break;
                    if (validateEdge(edge.trajectory) &&
                        Rewiring<Node>::commit(node, added, edge.trajectory, model_)) {
                        ++stats_.rewire_count;
                        break;
                    }
                }
            }
            count = 0;
        };
        for (Node *node : near) {
            if (wall.seconds() >= config_.timeout)
                return;
            if (!node->parent || node == added || Rewiring<Node>::isAncestor(node, added))
                continue;
            // Every edge has positive time and nonnegative effort. This is an
            // admissible zero edge-cost lower bound, applied BEFORE solving.
            if (added->cost >= node->cost - 1e-10)
                continue;
            nodes[count] = node;
            targets[count] = Constraint::full(node->state);
            if (++count == BatchWidth)
                flush();
        }
        flush();
    }

    Node *bestGoal() const {
        Node *best = nullptr;
        for (Node *n : goals_)
            if (!best || n->cost < best->cost)
                best = n;
        return best;
    }

    void pruneTree() {
        Node *best = bestGoal();
        if (!best)
            return;

        std::vector<Node *> removed;
        for (const auto &owned : tree_.nodes()) {
            Node *n = owned.get();
            if (n == tree_.root() || !n->children.empty() || Rewiring<Node>::isAncestor(n, best))
                continue;
            if (std::find(goals_.begin(), goals_.end(), n) != goals_.end())
                continue;
            // An admissible model bound from START via n to the goal is needed;
            // current tree g(n) is not a valid global lower bound for pruning.
            if (model_.solutionLowerBound(n->state) >= best->cost)
                removed.push_back(n);
        }
        for (Node *n : removed)
            tree_.removeNode(n);
    }

    Model model_;
    PlannerConfig config_;
    BatchSolver batch_pff_;
    RRTTree<Node> tree_;
    std::vector<Node *> goals_;
    std::vector<Trajectory> path_;
    PlanResult stats_{};
};
} // namespace rrtpff
