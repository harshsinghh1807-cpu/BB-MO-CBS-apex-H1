#include "HighLevel/pexHSolver.h"
#include "LowLevel/ApexSearch.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <queue>
#include <set>

extern std::ofstream output;
using namespace pexh;

pexHSolver::pexHSolver(size_t graph_size, int agent_num, int dim, int turn_dim, int turn_cost, int time_limit)
    : HighLevelSolver(graph_size, agent_num, Algorithm::BBMOCBS_PEX, false, dim, turn_dim, turn_cost, time_limit) {
    if (agent_num < 1 || dim < 1 || time_limit < 1 || turn_dim < -1 || turn_dim >= dim || turn_cost < 0)
        throw std::invalid_argument("invalid solver dimensions, turn cost or time limit");
}
void pexHSolver::set_eps(double eps) {
    if (!std::isfinite(eps) || eps < 0) throw std::invalid_argument("epsilon must be finite and nonnegative");
    eps_ = eps;
}
void pexHSolver::set_matching_limit(size_t k) {
    if (!k) throw std::invalid_argument("K must be at least 1");
    k_ = k;
}
void pexHSolver::set_merging_strategy(MergingStrategy strategy) {
    if (strategy != MergingStrategy::MORE_SLACK && strategy != MergingStrategy::CONFLICT_BASED)
        throw std::invalid_argument("pex-H supports MORE_SLACK and CONFLICT_BASED merging");
    strategy_ = strategy;
}
Solution pexHSolver::representative(const SearchNode& node, const JointPathPair& joint) const {
    Solution result{joint.first, CostVector(DIM, 0), {}};
    for (size_t x = 0; x < joint.second.size(); ++x) {
        size_t id = joint.second[x];
        result.cost = sum(result.cost, node.sop_cost[x].at(id));
        result.paths.push_back(node.sop_waypoints[x].at(id));
    }
    return result;
}
void pexHSolver::merge_joint_paths(SearchNode& node) {
    std::vector<JointPathPair> partial{{CostVector(DIM,0), {}}};
    for (size_t x = 0; x < static_cast<size_t>(AGENT_NUM); ++x) {
        std::vector<JointPathPair> next;
        std::vector<size_t> path_ids;
        for (const auto& path : node.sop_apex[x]) path_ids.push_back(path.first);
        std::sort(path_ids.begin(), path_ids.end());
        for (const auto& prefix : partial) for (size_t id : path_ids) {
            check_deadline(deadline_);
            JointPathPair candidate{sum(prefix.first, node.sop_apex[x].at(id)), prefix.second};
            candidate.second.push_back(id);
            const auto candidate_rep = representative(node, candidate);
            bool merged = false;
            for (auto& existing : next) {
                check_deadline(deadline_);
                auto apex = minimum(existing.first, candidate.first);
                const auto existing_rep = representative(node, existing);
                bool keep_existing = covered_by(apex, existing_rep.cost, eps_);
                bool keep_candidate = covered_by(apex, candidate_rep.cost, eps_);
                if (!keep_existing && !keep_candidate) continue;
                bool choose_candidate = !keep_existing;
                if (keep_existing && keep_candidate) {
                    if (strategy_ == MergingStrategy::CONFLICT_BASED)
                        choose_candidate = conflicts(candidate_rep.paths).size() < conflicts(existing_rep.paths).size();
                    else {
                        auto ratio = [&](const CostVector& cost) {
                            long double r = 1;
                            for (size_t o = 0; o < cost.size(); ++o)
                                if (apex[o]) r = std::max(r, static_cast<long double>(cost[o])/apex[o]);
                            return r;
                        };
                        choose_candidate = ratio(candidate_rep.cost) < ratio(existing_rep.cost);
                    }
                }
                if (choose_candidate) existing.second = candidate.second;
                existing.first = std::move(apex);
                merged = true;
                break;
            }
            if (!merged) next.push_back(std::move(candidate));
        }
        partial = std::move(next);
    }
    std::sort(partial.begin(), partial.end());
    node.all_jps.assign(partial.begin(), partial.end());
}
void pexHSolver::prune(SearchNode& node) {
    for (auto it = node.all_jps.begin(); it != node.all_jps.end();) {
        check_deadline(deadline_);
        bool remove = false;
        for (auto& solution : solutions_)
            if (covered_by(it->first, solution.cost, eps_)) {
                solution.apex = minimum(solution.apex, it->first);
                remove = true;
                break;
            }
        if (remove) it = node.all_jps.erase(it);
        else ++it;
    }
}
OutputTuple pexHSolver::run(std::vector<Edge>& edges, std::vector<std::pair<size_t, size_t>>& endpoints,
                           HSolutionID& paths, std::vector<CostVector>& costs, LoggerPtr& logger) {
    stats_ = Statistics{};
    solutions_.clear(); paths.clear(); costs.clear(); completed_ = false;
    auto started = Clock::now();
    deadline_ = started + std::chrono::seconds(TIME_LIMIT);
    double merge_seconds = 0, low_seconds = 0;
    int splits = 0;
    if (endpoints.size() != static_cast<size_t>(AGENT_NUM) || edges.empty())
        throw std::invalid_argument("endpoints must match agent count and graph must contain edges");
    for (const auto& edge : edges) {
        if (edge.source >= GRAPH_SIZE || edge.target >= GRAPH_SIZE || edge.cost.size() != static_cast<size_t>(DIM))
            throw std::invalid_argument("invalid edge or cost dimension");
        for (size_t c : edge.cost) if (c == MAX_COST) throw std::invalid_argument("MAX_COST is reserved for infinity");
    }
    for (const auto& endpoint : endpoints)
        if (endpoint.first >= GRAPH_SIZE || endpoint.second >= GRAPH_SIZE)
            throw std::invalid_argument("agent endpoint is outside the graph");
    AdjacencyMatrix graph(GRAPH_SIZE, edges), inverse(GRAPH_SIZE, edges, true);
    using NodePtr = std::shared_ptr<SearchNode>;
    struct Entry { CostVector key; size_t serial; NodePtr node; };
    struct Later {
        bool operator()(const Entry& a, const Entry& b) const {
            if (a.key != b.key) return a.key > b.key;
            return a.serial > b.serial;
        }
    };
    std::priority_queue<Entry, std::vector<Entry>, Later> open;
    size_t serial = 0;
    auto enqueue = [&](NodePtr node, CostVector key) {
        node->key = std::move(key);
        open.push({node->key, serial++, node});
    };
    auto merge = [&](SearchNode& node) {
        auto t = Clock::now();
        merge_joint_paths(node);
        merge_seconds += std::chrono::duration<double>(Clock::now()-t).count();
    };
    try {
        std::set<size_t> starts, goals;
        bool trivially_infeasible = false;
        for (const auto& endpoint : endpoints) {
            if (!starts.insert(endpoint.first).second || !goals.insert(endpoint.second).second)
                trivially_infeasible = true;
        }
        std::vector<std::vector<CostVector>> distances;
        if (!trivially_infeasible)
            for (const auto& endpoint : endpoints) distances.push_back(reverse_distances(inverse, endpoint.second, deadline_));
        auto low_level = [&](SearchNode& node, size_t x, const std::vector<std::vector<size_t>>& rep) {
            check_deadline(deadline_);
            ++stats_.low_level_searches;
            VertexCAT vertices;
            EdgeCAT edge_cat;
            if (strategy_ == MergingStrategy::CONFLICT_BASED && !rep.empty()) {
                size_t length = 0;
                for (const auto& path : rep) length = std::max(length, path.size());
                for (size_t t = 0; t < length; ++t) {
                    vertices[t].assign(GRAPH_SIZE+1, 0);
                    edge_cat[t].resize(GRAPH_SIZE+1);
                    for (size_t y = 0; y < rep.size(); ++y) if (x != y) {
                        size_t from = rep[y][std::min(t, rep[y].size()-1)];
                        size_t to = rep[y][std::min(t+1, rep[y].size()-1)];
                        ++vertices[t][from];
                        ++edge_cat[t][to][from]; // candidate reverse edge conflicts with this move
                    }
                }
            }
            Heuristic heuristic = [&](size_t vertex, size_t, bool) { return distances.at(x).at(vertex); };
            ::EPS epsilon(DIM, eps_);
            ApexSearch solver(graph, epsilon, epsilon, TURN_DIM, TURN_COST, logger);
            solver.set_merge_strategy(strategy_);
            solver.set_constraint_safe_search(deadline_);
            auto t = Clock::now();
            solver(node.sop_waypoints[x], node.sop_apex[x], node.sop_cost[x],
                   endpoints[x].first, endpoints[x].second, heuristic,
                   node.vertex_constraints[x], node.edge_constraints[x], TIME_LIMIT,
                   vertices, edge_cat, node.conflict_num);
            low_seconds += std::chrono::duration<double>(Clock::now()-t).count();
            check_deadline(deadline_);
        };
        auto root = std::make_shared<SearchNode>(AGENT_NUM);
        bool feasible = !trivially_infeasible;
        for (int x = 0; feasible && x < AGENT_NUM; ++x) {
            low_level(*root, x, {});
            feasible = !root->sop_waypoints[x].empty();
        }
        if (feasible) {
            merge(*root);
            if (!root->all_jps.empty()) enqueue(root, root->all_jps.front().first);
        }
        while (!open.empty()) {
            check_deadline(deadline_);
            auto node = open.top().node; open.pop();
            auto previous = node->all_jps.front();
            prune(*node);
            if (node->all_jps.empty()) continue;
            if (node->all_jps.front() != previous) {
                ++stats_.representative_changes;
                node->analysis.reset();
                enqueue(node, node->all_jps.front().first);
                continue;
            }
            auto rep = representative(*node, node->all_jps.front());
            if (conflicts(rep.paths).empty()) {
                add_solution(std::move(rep), solutions_, eps_);
                node->all_jps.pop_front();
                if (!node->all_jps.empty()) ++stats_.representative_changes;
                node->analysis.reset();
                if (!node->all_jps.empty()) enqueue(node, node->all_jps.front().first);
                continue;
            }
            if (!node->analysis) {
                ++stats_.analyses;
                node->analysis = std::make_shared<Analysis>(analyze(*node, rep.paths, graph, endpoints, k_,
                                                                  TURN_DIM, TURN_COST, deadline_, stats_.scalar_searches));
                stats_.max_matching = std::max(stats_.max_matching, node->analysis->matching.size());
            } else ++stats_.analysis_cache_hits;
            const auto& analysis = *node->analysis;
            if (analysis.nondominated.empty()) { ++stats_.nodes_pruned; continue; }
            auto key = std::max(node->all_jps.front().first, analysis.nondominated.front());
            if (key > node->key) {
                ++stats_.lazy_reinsertions;
                enqueue(node, key); // preserve analysis when only the priority changes
                continue;
            }
            if (bound_covered(analysis.nondominated, solutions_, eps_)) { ++stats_.nodes_pruned; continue; }
            ++splits;
            const auto& conflict = analysis.matching.front().conflict;
            for (int side = 0; side < 2; ++side) {
                check_deadline(deadline_);
                auto bounds = child_bounds(analysis.full, side); // deliberately use Full before global ND
                if (bound_covered(bounds, solutions_, eps_)) { ++stats_.children_pruned; continue; }
                auto child = std::make_shared<SearchNode>(*node);
                child->analysis.reset();
                child->all_jps.clear();
                size_t x = side == 0 ? conflict.a : conflict.b;
                constrain(conflict, side, child->vertex_constraints[x], child->edge_constraints[x]);
                child->sop_waypoints[x].clear(); child->sop_apex[x].clear(); child->sop_cost[x].clear();
                low_level(*child, x, rep.paths);
                if (child->sop_waypoints[x].empty()) continue;
                merge(*child);
                if (!child->all_jps.empty()) enqueue(child, std::max(child->all_jps.front().first, bounds.front()));
            }
        }
        completed_ = true;
    } catch (const SearchTimeout&) {
        // Keep feasible incumbents, but never claim an incomplete front is certified.
        completed_ = false;
    }
    std::sort(solutions_.begin(), solutions_.end(), [](const Solution& a, const Solution& b) { return a.apex < b.apex; });
    for (const auto& solution : solutions_) { paths.push_back(solution.paths); costs.push_back(solution.cost); }
    const double seconds = std::chrono::duration<double>(Clock::now()-started).count();
    output << (completed_ ? (solutions_.empty() ? "INFEASIBLE" : "SUCCESS") : "TIMEOUT") << '\n';
    output << "algorithm: BBMOCBS-pex-H\nepsilon: " << eps_ << "\nmatching_limit: " << k_ << '\n';
    for (const auto& solution : solutions_) output << "apex: " << solution.apex << " cost: " << solution.cost << '\n';
    output << "ConflictAnalyses = " << stats_.analyses << "\nScalarSearches = " << stats_.scalar_searches
           << "\nLazyReinsertions = " << stats_.lazy_reinsertions << "\nNodesPrunedByBounds = " << stats_.nodes_pruned
           << "\nChildrenPrunedByBounds = " << stats_.children_pruned << "\nLowLevelSearches = " << stats_.low_level_searches
           << "\nMaxMatching = " << stats_.max_matching << "\nAnalysisCacheHits = " << stats_.analysis_cache_hits
           << "\nRepresentativeChanges = " << stats_.representative_changes << '\n';
    return std::make_tuple(merge_seconds, low_seconds, seconds, splits, static_cast<int>(solutions_.size()));
}
