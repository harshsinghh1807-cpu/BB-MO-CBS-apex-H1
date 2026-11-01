#include "HighLevel/pexHSolver.h"
#include <algorithm>
#include <set>

namespace pexh {
bool covered_by(const CostVector& bound, const CostVector& cost, double eps) {
    if (bound.size() != cost.size()) throw std::invalid_argument("cost dimension mismatch");
    for (size_t o = 0; o < bound.size(); ++o)
        if (static_cast<long double>(cost[o]) > (1.0L + static_cast<long double>(eps))*bound[o]) return false;
    return true;
}
CostVector minimum(const CostVector& a, const CostVector& b) {
    if (a.size() != b.size()) throw std::invalid_argument("cost dimension mismatch");
    CostVector out(a.size());
    for (size_t o = 0; o < a.size(); ++o) out[o] = std::min(a[o], b[o]);
    return out;
}
CostVector sum(const CostVector& a, const CostVector& b) {
    if (a.size() != b.size()) throw std::invalid_argument("cost dimension mismatch");
    CostVector out(a.size());
    for (size_t o = 0; o < a.size(); ++o) out[o] = checked_add(a[o], b[o]);
    return out;
}
std::vector<CostVector> nondominated(std::vector<CostVector> bounds) {
    std::sort(bounds.begin(), bounds.end());
    std::vector<CostVector> result;
    for (auto& w : bounds) {
        // A side with no feasible path has no finite lower-bound obligation.
        if (std::find(w.begin(), w.end(), MAX_COST) != w.end()) continue;
        bool dominated = false;
        for (const auto& existing : result)
            if (covered_by(w, existing, 0)) { dominated = true; break; }
        if (!dominated) result.push_back(std::move(w));
    }
    return result;
}
std::vector<LabeledBound> full_bounds(const CostVector& base, const std::vector<PairBound>& matching) {
    if (matching.empty()) throw std::invalid_argument("matching must include the split pair");
    std::vector<LabeledBound> full{{base, -1}};
    for (size_t p = 0; p < matching.size(); ++p) {
        std::vector<LabeledBound> next;
        for (const auto& w : full)
            for (int side = 0; side < 2; ++side)
                next.push_back({sum(w.cost, matching[p].delta[side]), p == 0 ? side : w.chosen_side});
        full = std::move(next);
    }
    return full;
}
std::vector<CostVector> child_bounds(const std::vector<LabeledBound>& full, int side) {
    std::vector<CostVector> result;
    for (const auto& w : full) if (w.chosen_side == side) result.push_back(w.cost);
    return nondominated(std::move(result));
}
bool bound_covered(const std::vector<CostVector>& bounds, std::vector<Solution>& solutions, double eps) {
    std::vector<size_t> witnesses;
    for (const auto& w : bounds) {
        size_t i = 0;
        while (i < solutions.size() && !covered_by(w, solutions[i].cost, eps)) ++i;
        if (i == solutions.size()) return false;
        witnesses.push_back(i);
    }
    for (size_t i = 0; i < bounds.size(); ++i)
        solutions[witnesses[i]].apex = minimum(solutions[witnesses[i]].apex, bounds[i]);
    return true;
}
void add_solution(Solution solution, std::vector<Solution>& solutions, double eps) {
    for (auto& existing : solutions) {
        auto apex = minimum(existing.apex, solution.apex);
        if (covered_by(apex, existing.cost, eps)) { existing.apex = std::move(apex); return; }
        if (covered_by(apex, solution.cost, eps)) {
            solution.apex = std::move(apex);
            existing = std::move(solution);
            return;
        }
    }
    solutions.push_back(std::move(solution));
}
std::vector<Conflict> conflicts(const std::vector<std::vector<size_t>>& paths) {
    std::vector<Conflict> result;
    for (size_t a = 0; a < paths.size(); ++a) {
        if (paths[a].empty()) throw std::invalid_argument("empty representative path");
        for (size_t b = a+1; b < paths.size(); ++b) {
            if (paths[b].empty()) throw std::invalid_argument("empty representative path");
            auto at = [&](size_t agent, size_t t) { return paths[agent][std::min(t, paths[agent].size()-1)]; };
            size_t length = std::max(paths[a].size(), paths[b].size());
            for (size_t t = 0; t < length; ++t) {
                if (at(a,t) == at(b,t)) {
                    result.push_back({static_cast<int>(a), static_cast<int>(b), t, {at(a,t)}});
                    break;
                }
                if (t+1 < length && at(a,t) == at(b,t+1) && at(b,t) == at(a,t+1)) {
                    result.push_back({static_cast<int>(a), static_cast<int>(b), t, {at(a,t), at(b,t)}});
                    break;
                }
            }
        }
    }
    return result;
}
void constrain(const Conflict& conflict, int side, VertexConstraint& vertices, EdgeConstraint& edges) {
    if (conflict.vertices.size() == 1) vertices[conflict.time].push_back(conflict.vertices[0]);
    else edges[conflict.time][conflict.vertices.at(side)].push_back(conflict.vertices.at(1-side));
}
Analysis analyze(const HighLevelNode& node, const std::vector<std::vector<size_t>>& representative,
                 const AdjacencyMatrix& graph, const std::vector<std::pair<size_t, size_t>>& endpoints,
                 size_t k, int turn_dim, int turn_cost, Deadline deadline, size_t& scalar_searches, const ScalarOracle& oracle) {
    if (k < 1) throw std::invalid_argument("matching limit K must be positive");
    const size_t dim = graph.get_num_of_objectives();
    std::vector<CostVector> ideals(endpoints.size(), CostVector(dim, MAX_COST));
    Analysis result;
    result.base.assign(dim, 0);
    for (size_t x = 0; x < endpoints.size(); ++x) {
        if (node.sop_apex[x].empty()) throw std::logic_error("analysis of an infeasible node");
        for (const auto& path : node.sop_apex[x]) ideals[x] = minimum(ideals[x], path.second);
        result.base = sum(result.base, ideals[x]);
    }
    std::vector<PairBound> pairs;
    for (const auto& conflict : conflicts(representative)) {
        check_deadline(deadline);
        PairBound pair;
        pair.conflict = conflict;
        for (int side = 0; side < 2; ++side) {
            int x = side == 0 ? conflict.a : conflict.b;
            auto vertices = node.vertex_constraints[x];
            auto edges = node.edge_constraints[x];
            constrain(conflict, side, vertices, edges);
            pair.delta[side].assign(dim, 0);
            for (size_t o = 0; o < dim; ++o) {
                ++scalar_searches;
                size_t cost = oracle ? oracle(x, o, vertices, edges) : exact_shortest_path(graph, endpoints[x].first, endpoints[x].second,
                                                 o, vertices, edges, turn_dim, turn_cost, deadline);
                if (cost == MAX_COST) {
                    pair.delta[side].assign(dim, MAX_COST);
                    break;
                }
                pair.delta[side][o] = cost > ideals[x][o] ? cost - ideals[x][o] : 0;
            }
        }
        const CostVector zero(dim, 0);
        pair.cardinal = pair.delta[0] != zero && pair.delta[1] != zero;
        pair.score = std::min(pair.delta[0], pair.delta[1]); // LEX min, not componentwise min
        pairs.push_back(std::move(pair));
    }
    if (pairs.empty()) throw std::logic_error("conflict analysis needs a conflict");
    size_t selected = 0;
    bool found_cardinal = false;
    for (size_t p = 0; p < pairs.size(); ++p)
        if (pairs[p].cardinal && (!found_cardinal || pairs[selected].score < pairs[p].score)) {
            selected = p; found_cardinal = true;
        }
    result.matching.push_back(pairs[selected]);
    std::set<int> used{pairs[selected].conflict.a, pairs[selected].conflict.b};
    std::vector<size_t> order;
    for (size_t p = 0; p < pairs.size(); ++p) if (p != selected && pairs[p].cardinal) order.push_back(p);
    std::stable_sort(order.begin(), order.end(), [&](size_t a, size_t b) { return pairs[a].score > pairs[b].score; });
    for (size_t p : order) {
        if (result.matching.size() == k) break;
        const auto& c = pairs[p].conflict;
        if (used.count(c.a) || used.count(c.b)) continue;
        result.matching.push_back(pairs[p]);
        used.insert(c.a); used.insert(c.b);
    }
    result.full = full_bounds(result.base, result.matching);
    std::vector<CostVector> bounds;
    for (const auto& w : result.full) bounds.push_back(w.cost);
    result.nondominated = nondominated(std::move(bounds));
    check_deadline(deadline);
    return result;
}
}
