#include "HighLevel/pexHSolver.h"
#include "LowLevel/ApexSearch.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <queue>
#include <set>

extern std::ofstream output;
using namespace pexh;
namespace {
struct ScopedSeconds {
    double& seconds;
    Clock::time_point start = Clock::now();
    explicit ScopedSeconds(double& target) : seconds(target) {}
    ~ScopedSeconds() { seconds += std::chrono::duration<double>(Clock::now()-start).count(); }
};
// Sorted, deduplicated constraint tuples make equivalent subproblems share a key.
std::vector<size_t> scalar_key(size_t agent, size_t objective, const VertexConstraint& vertices, const EdgeConstraint& edges) {
    std::vector<std::array<size_t,4>> entries;
    for (const auto& t : vertices) for (size_t v : t.second) entries.push_back({0,t.first,v,0});
    for (const auto& t : edges) for (const auto& v : t.second) for (size_t w : v.second) entries.push_back({1,t.first,v.first,w});
    std::sort(entries.begin(), entries.end());
    entries.erase(std::unique(entries.begin(), entries.end()), entries.end());
    std::vector<size_t> key{agent, objective};
    for (const auto& entry : entries) key.insert(key.end(), entry.begin(), entry.end());
    return key;
}
}

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
    struct Partial { JointPathPair joint; CostVector cost; size_t conflicts = 0; };
    std::vector<Partial> partial{{{CostVector(DIM,0), {}}, CostVector(DIM,0), 0}};
    for (size_t x = 0; x < static_cast<size_t>(AGENT_NUM); ++x) {
        std::vector<Partial> next;
        std::vector<size_t> path_ids;
        for (const auto& path : node.sop_apex[x]) path_ids.push_back(path.first);
        std::sort(path_ids.begin(), path_ids.end());
        for (const auto& prefix : partial) for (size_t id : path_ids) {
            check_deadline(deadline_);
            Partial candidate{{sum(prefix.joint.first, node.sop_apex[x].at(id)), prefix.joint.second},
                              sum(prefix.cost, node.sop_cost[x].at(id)), 0};
            candidate.joint.second.push_back(id);
            if (strategy_ == MergingStrategy::CONFLICT_BASED)
                candidate.conflicts = conflicts(representative(node, candidate.joint).paths).size();
            bool merged = false;
            for (auto& existing : next) {
                check_deadline(deadline_);
                auto apex = minimum(existing.joint.first, candidate.joint.first);
                bool keep_existing = covered_by(apex, existing.cost, eps_);
                bool keep_candidate = covered_by(apex, candidate.cost, eps_);
                if (!keep_existing && !keep_candidate) continue;
                bool choose_candidate = !keep_existing;
                if (keep_existing && keep_candidate) {
                    if (strategy_ == MergingStrategy::CONFLICT_BASED)
                        choose_candidate = candidate.conflicts < existing.conflicts;
                    else {
                        auto ratio = [&](const CostVector& cost) {
                            long double r = 1;
                            for (size_t o = 0; o < cost.size(); ++o)
                                if (apex[o]) r = std::max(r, static_cast<long double>(cost[o])/apex[o]);
                            return r;
                        };
                        choose_candidate = ratio(candidate.cost) < ratio(existing.cost);
                    }
                }
                if (choose_candidate) existing = candidate;
                existing.joint.first = std::move(apex);
                merged = true;
                break;
            }
            if (!merged) next.push_back(std::move(candidate));
        }
        partial = std::move(next);
    }
    std::sort(partial.begin(), partial.end(), [](const Partial& a, const Partial& b) { return a.joint < b.joint; });
    node.all_jps.clear();
    for (auto& entry : partial) node.all_jps.push_back(std::move(entry.joint));
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
        ScopedSeconds timer(merge_seconds);
        merge_joint_paths(node);
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
        std::map<std::vector<size_t>, size_t> scalar_cache;
        struct CachedPaths { PathSet paths; CostSet apexes, costs; std::unordered_map<int,int> conflicts; };
        std::map<std::vector<size_t>, CachedPaths> path_cache;
        size_t cached_waypoints = 0;
        ScalarOracle scalar_oracle = [&](size_t x, size_t objective, const VertexConstraint& vertices, const EdgeConstraint& constraints) {
            check_deadline(deadline_);
            std::vector<size_t> key;
            if (scalar_optimizations_) {
                key = scalar_key(x, objective, vertices, constraints);
                auto it = scalar_cache.find(key);
                if (it != scalar_cache.end()) { ++stats_.scalar_cache_hits; return it->second; }
            }
            ++stats_.scalar_searches;
            ScopedSeconds timer(stats_.scalar_seconds);
            size_t value = exact_shortest_path(graph, endpoints[x].first, endpoints[x].second, objective,
                                              vertices, constraints, TURN_DIM, TURN_COST, deadline_,
                                              scalar_optimizations_ ? &distances[x] : nullptr, &stats_.scalar_expansions);
            if (scalar_optimizations_) {
                // Bound auxiliary cache memory; eviction changes performance only.
                if (scalar_cache.size() >= 100000) scalar_cache.clear();
                scalar_cache.emplace(std::move(key), value);
            }
            return value;
        };
        auto low_level = [&](SearchNode& node, size_t x, const std::vector<std::vector<size_t>>& rep) {
            check_deadline(deadline_);
            ++stats_.low_level_queries;
            // With conflict-based merging the CAT depends on other agents' paths,
            // so constraints alone are not a valid cache key. Cache only MORE_SLACK.
            const bool cacheable = low_level_cache_ && strategy_ == MergingStrategy::MORE_SLACK;
            std::vector<size_t> key;
            if (cacheable) {
                key = scalar_key(x, MAX_COST, node.vertex_constraints[x], node.edge_constraints[x]);
                auto found = path_cache.find(key);
                if (found != path_cache.end()) {
                    ++stats_.low_level_cache_hits;
                    node.sop_waypoints[x] = found->second.paths;
                    node.sop_apex[x] = found->second.apexes;
                    node.sop_cost[x] = found->second.costs;
                    node.conflict_num = found->second.conflicts;
                    return;
                }
            }
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
            ScopedSeconds timer(low_seconds);
            solver(node.sop_waypoints[x], node.sop_apex[x], node.sop_cost[x],
                   endpoints[x].first, endpoints[x].second, heuristic,
                   node.vertex_constraints[x], node.edge_constraints[x], TIME_LIMIT,
                   vertices, edge_cat, node.conflict_num);
            check_deadline(deadline_);
            if (cacheable) {
                size_t waypoints = 1; // count even infeasible results toward the budget
                for (const auto& path : node.sop_waypoints[x]) waypoints += path.second.size();
                if (waypoints <= 1000000) {
                    if (cached_waypoints + waypoints > 1000000 || path_cache.size() >= 10000) {
                        path_cache.clear(); cached_waypoints = 0;
                    }
                    path_cache.emplace(std::move(key), CachedPaths{node.sop_waypoints[x],node.sop_apex[x],node.sop_cost[x],node.conflict_num});
                    cached_waypoints += waypoints;
                }
            }
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
            const auto pair_conflicts = conflicts(rep.paths);
            if (pair_conflicts.empty()) {
                add_solution(std::move(rep), solutions_, eps_);
                node->all_jps.pop_front();
                if (!node->all_jps.empty()) ++stats_.representative_changes;
                node->analysis.reset();
                if (!node->all_jps.empty()) enqueue(node, node->all_jps.front().first);
                continue;
            }
            Conflict conflict = pair_conflicts.front();
            if (use_heuristic_) {
                if (!node->analysis) {
                    ++stats_.analyses;
                    ScopedSeconds timer(stats_.analysis_seconds);
                    node->analysis = std::make_shared<Analysis>(analyze(*node, rep.paths, graph, endpoints, k_,
                        TURN_DIM, TURN_COST, deadline_, stats_.scalar_queries, scalar_oracle));
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
                conflict = analysis.matching.front().conflict;
            }
            ++splits;
            for (int side = 0; side < 2; ++side) {
                check_deadline(deadline_);
                std::vector<CostVector> bounds;
                if (use_heuristic_) {
                    bounds = child_bounds(node->analysis->full, side); // Full, not global ND
                    if (bound_covered(bounds, solutions_, eps_)) { ++stats_.children_pruned; continue; }
                }
                auto child = std::make_shared<SearchNode>(*node);
                child->analysis.reset();
                child->all_jps.clear();
                size_t x = side == 0 ? conflict.a : conflict.b;
                constrain(conflict, side, child->vertex_constraints[x], child->edge_constraints[x]);
                child->sop_waypoints[x].clear(); child->sop_apex[x].clear(); child->sop_cost[x].clear();
                low_level(*child, x, rep.paths);
                if (child->sop_waypoints[x].empty()) continue;
                merge(*child);
                if (!child->all_jps.empty()) enqueue(child, use_heuristic_
                    ? std::max(child->all_jps.front().first, bounds.front()) : child->all_jps.front().first);
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
    output << "algorithm: " << algorithm_name() << "\nepsilon: " << eps_ << "\nmatching_limit: " << k_ << '\n';
    for (const auto& solution : solutions_) output << "apex: " << solution.apex << " cost: " << solution.cost << '\n';
    output << "ConflictAnalyses = " << stats_.analyses << "\nScalarSearches = " << stats_.scalar_searches
           << "\nLazyReinsertions = " << stats_.lazy_reinsertions << "\nNodesPrunedByBounds = " << stats_.nodes_pruned
           << "\nChildrenPrunedByBounds = " << stats_.children_pruned << "\nLowLevelSearches = " << stats_.low_level_searches
           << "\nMaxMatching = " << stats_.max_matching << "\nAnalysisCacheHits = " << stats_.analysis_cache_hits
           << "\nRepresentativeChanges = " << stats_.representative_changes
           << "\nScalarQueries = " << stats_.scalar_queries << "\nScalarCacheHits = " << stats_.scalar_cache_hits
           << "\nScalarExpansions = " << stats_.scalar_expansions << "\nConflictAnalysisTime = " << stats_.analysis_seconds
           << "\nScalarSearchTime = " << stats_.scalar_seconds << '\n';
    output << "LowLevelQueries = " << stats_.low_level_queries << "\nLowLevelCacheHits = " << stats_.low_level_cache_hits << '\n';
    return std::make_tuple(merge_seconds, low_seconds, seconds, splits, static_cast<int>(solutions_.size()));
}
