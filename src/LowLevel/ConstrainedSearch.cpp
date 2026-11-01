#include "LowLevel/ConstrainedSearch.h"
#include <algorithm>
#include <queue>
#include <tuple>

extern std::unordered_map<size_t, std::vector<int>> id2coord;
namespace pexh {
size_t checked_add(size_t a, size_t b) {
    if (a == MAX_COST || b == MAX_COST) return MAX_COST;
    if (a >= MAX_COST - b) throw std::overflow_error("path cost exceeds size_t range");
    return a + b;
}
bool vertex_blocked(const VertexConstraint& constraints, size_t vertex, size_t time) {
    auto it = constraints.find(time);
    return it != constraints.end() &&
           std::find(it->second.begin(), it->second.end(), vertex) != it->second.end();
}
bool edge_blocked(const EdgeConstraint& constraints, size_t from, size_t to, size_t time) {
    auto it = constraints.find(time);
    if (it == constraints.end()) return false;
    auto jt = it->second.find(from);
    return jt != it->second.end() &&
           std::find(jt->second.begin(), jt->second.end(), to) != jt->second.end();
}
size_t constraint_end(const VertexConstraint& vertices, const EdgeConstraint& edges) {
    size_t last = 0;
    for (const auto& entry : vertices) last = std::max(last, entry.first);
    for (const auto& entry : edges) last = std::max(last, checked_add(entry.first, 1));
    return last;
}
size_t earliest_goal_time(size_t goal, const VertexConstraint& vertices, const EdgeConstraint& edges) {
    size_t earliest = 0;
    for (const auto& entry : vertices)
        if (vertex_blocked(vertices, goal, entry.first))
            earliest = std::max(earliest, checked_add(entry.first, 1));
    for (const auto& entry : edges)
        if (edge_blocked(edges, goal, goal, entry.first))
            earliest = std::max(earliest, checked_add(entry.first, 1));
    return earliest;
}
size_t transition_cost(size_t previous, size_t from, const Edge& edge,
                       size_t objective, int turn_dim, int turn_cost) {
    size_t cost = edge.cost.at(objective);
    if (static_cast<int>(objective) != turn_dim) return cost;
    bool turn;
    if (previous == MAX_COST) turn = edge.target != from;
    else {
        const auto& p = id2coord.at(previous);
        const auto& f = id2coord.at(from);
        const auto& t = id2coord.at(edge.target);
        turn = f.at(0)-p.at(0) != t.at(0)-f.at(0) ||
               f.at(1)-p.at(1) != t.at(1)-f.at(1);
    }
    return turn ? checked_add(cost, static_cast<size_t>(turn_cost)) : cost;
}
size_t exact_shortest_path(const AdjacencyMatrix& graph, size_t source, size_t target,
                           size_t objective, const VertexConstraint& vertices,
                           const EdgeConstraint& edges, int turn_dim, int turn_cost,
                           Deadline deadline) {
    check_deadline(deadline);
    if (vertex_blocked(vertices, source, 0)) return MAX_COST;
    const size_t end = std::max(constraint_end(vertices, edges), earliest_goal_time(target, vertices, edges));
    const size_t goal_time = earliest_goal_time(target, vertices, edges);
    using State = std::tuple<size_t, size_t, size_t>; // vertex, previous vertex, folded time
    using Entry = std::pair<size_t, State>;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
    std::map<State, size_t> distance;
    State root(source, MAX_COST, 0);
    distance[root] = 0;
    open.emplace(0, root);
    while (!open.empty()) {
        check_deadline(deadline);
        auto entry = open.top(); open.pop();
        const auto state = entry.second;
        const size_t cost = entry.first;
        if (distance.at(state) != cost) continue;
        size_t vertex, previous, time;
        std::tie(vertex, previous, time) = state;
        if (vertex == target && time >= goal_time) return cost;
        for (const auto& edge : graph[vertex]) {
            if (vertex_blocked(vertices, edge.target, time+1) ||
                edge_blocked(edges, vertex, edge.target, time)) continue;
            const size_t next_cost = checked_add(cost, transition_cost(previous, vertex, edge, objective, turn_dim, turn_cost));
            State next(edge.target, static_cast<int>(objective) == turn_dim ? vertex : MAX_COST,
                       std::min(time+1, end));
            auto it = distance.find(next);
            if (it == distance.end() || next_cost < it->second) {
                distance[next] = next_cost;
                open.emplace(next_cost, next);
            }
        }
    }
    return MAX_COST;
}
std::vector<CostVector> reverse_distances(const AdjacencyMatrix& inverse, size_t goal, Deadline deadline) {
    size_t dim = inverse.get_num_of_objectives();
    std::vector<CostVector> distance(inverse.size()+1, CostVector(dim, MAX_COST));
    using Entry = std::pair<size_t, size_t>;
    for (size_t o = 0; o < dim; ++o) {
        std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
        distance.at(goal)[o] = 0;
        open.emplace(0, goal);
        while (!open.empty()) {
            check_deadline(deadline);
            auto current = open.top(); open.pop();
            if (distance[current.second][o] != current.first) continue;
            for (const auto& edge : inverse[current.second]) {
                size_t next = checked_add(current.first, edge.cost[o]);
                if (next < distance[edge.target][o]) {
                    distance[edge.target][o] = next;
                    open.emplace(next, edge.target);
                }
            }
        }
    }
    return distance;
}
size_t finite_horizon(const AdjacencyMatrix& graph, const VertexConstraint& vertices,
                      const EdgeConstraint& edges, bool with_orientation) {
    // After the last timed restriction, a nondominated suffix can be chosen simple
    // in (vertex, previous-vertex) space (or just vertex space without turns).
    size_t states = graph.size()+1;
    if (with_orientation)
        for (size_t v = 0; v <= graph.size(); ++v) states = checked_add(states, graph[v].size());
    return checked_add(constraint_end(vertices, edges), states);
}
}
