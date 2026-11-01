#pragma once
#include "Definitions.h"
#include <chrono>
#include <stdexcept>

namespace pexh {
using Clock = std::chrono::steady_clock;
using Deadline = Clock::time_point;
struct SearchTimeout : std::runtime_error {
    SearchTimeout() : std::runtime_error("search deadline reached") {}
};
inline void check_deadline(Deadline deadline) {
    if (Clock::now() >= deadline) throw SearchTimeout();
}
size_t checked_add(size_t a, size_t b);
bool vertex_blocked(const VertexConstraint& constraints, size_t vertex, size_t time);
bool edge_blocked(const EdgeConstraint& constraints, size_t from, size_t to, size_t time);
size_t constraint_end(const VertexConstraint& vertices, const EdgeConstraint& edges);
size_t earliest_goal_time(size_t goal, const VertexConstraint& vertices, const EdgeConstraint& edges);
size_t transition_cost(size_t previous, size_t from, const Edge& edge,
                       size_t objective, int turn_dim, int turn_cost);
// Exact scalar A* (Dijkstra without lower_bounds). Time folds only after constraints expire.
// MAX_COST means proven infeasible; a timeout throws SearchTimeout instead.
size_t exact_shortest_path(const AdjacencyMatrix& graph, size_t source, size_t target,
                           size_t objective, const VertexConstraint& vertices,
                           const EdgeConstraint& edges, int turn_dim, int turn_cost,
                           Deadline deadline, const std::vector<CostVector>* lower_bounds = nullptr,
                           size_t* expansions = nullptr);
// Admissible reverse distances; omitting nonnegative turn penalties remains admissible.
std::vector<CostVector> reverse_distances(const AdjacencyMatrix& inverse, size_t goal,
                                        Deadline deadline);
size_t finite_horizon(const AdjacencyMatrix& graph, const VertexConstraint& vertices,
                      const EdgeConstraint& edges, bool with_orientation);
}
