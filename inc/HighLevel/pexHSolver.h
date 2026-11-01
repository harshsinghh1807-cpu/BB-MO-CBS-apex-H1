#pragma once
#include "HighLevel/HighLevelSolver.h"
#include "LowLevel/ConstrainedSearch.h"
#include <array>

namespace pexh {
struct Conflict {
    int a = -1, b = -1;
    size_t time = 0;
    // One vertex, or the directed edge traversed by a (b traverses its reverse).
    std::vector<size_t> vertices;
};
struct PairBound {
    Conflict conflict;
    std::array<CostVector, 2> delta;
    CostVector score;
    bool cardinal = false;
};
struct LabeledBound {
    CostVector cost;
    int chosen_side; // side of the FIRST pair (the selected split conflict)
};
struct Analysis {
    CostVector base;
    std::vector<PairBound> matching;
    std::vector<LabeledBound> full;
    std::vector<CostVector> nondominated;
};
struct Solution {
    CostVector apex, cost;
    std::vector<std::vector<size_t>> paths;
};
bool covered_by(const CostVector& bound, const CostVector& cost, double eps);
CostVector minimum(const CostVector& a, const CostVector& b);
CostVector sum(const CostVector& a, const CostVector& b);
std::vector<CostVector> nondominated(std::vector<CostVector> bounds);
std::vector<LabeledBound> full_bounds(const CostVector& base, const std::vector<PairBound>& matching);
std::vector<CostVector> child_bounds(const std::vector<LabeledBound>& full, int side);
// Transactional: do not modify any solution apex unless ALL bounds are covered.
bool bound_covered(const std::vector<CostVector>& bounds, std::vector<Solution>& solutions, double eps);
void add_solution(Solution solution, std::vector<Solution>& solutions, double eps);
std::vector<Conflict> conflicts(const std::vector<std::vector<size_t>>& paths);
void constrain(const Conflict& conflict, int side, VertexConstraint& vertices, EdgeConstraint& edges);
Analysis analyze(const HighLevelNode& node, const std::vector<std::vector<size_t>>& representative,
                 const AdjacencyMatrix& graph, const std::vector<std::pair<size_t, size_t>>& endpoints,
                 size_t k, int turn_dim, int turn_cost, Deadline deadline, size_t& scalar_searches);
}

class pexHSolver : public HighLevelSolver {
public:
    struct Statistics {
        size_t analyses = 0, scalar_searches = 0, lazy_reinsertions = 0;
        size_t nodes_pruned = 0, children_pruned = 0, low_level_searches = 0;
        size_t max_matching = 0;
        size_t analysis_cache_hits = 0, representative_changes = 0;
    };
    pexHSolver(size_t graph_size, int agent_num, int dim, int turn_dim, int turn_cost, int time_limit);
    void set_eps(double eps);
    void set_matching_limit(size_t k);
    void set_merging_strategy(MergingStrategy strategy);
    const Statistics& statistics() const { return stats_; }
    const std::vector<pexh::Solution>& solutions() const { return solutions_; }
    bool completed() const { return completed_; }
    OutputTuple run(std::vector<Edge>& edges, std::vector<std::pair<size_t, size_t>>& endpoints,
                    HSolutionID& paths, std::vector<CostVector>& costs, LoggerPtr& logger) override;
private:
    struct SearchNode : HighLevelNode {
        explicit SearchNode(size_t n) : HighLevelNode(n) {}
        CostVector key;
        std::shared_ptr<const pexh::Analysis> analysis;
    };
    double eps_ = 0;
    size_t k_ = 1;
    MergingStrategy strategy_ = MergingStrategy::MORE_SLACK;
    Statistics stats_;
    std::vector<pexh::Solution> solutions_;
    bool completed_ = false;
    pexh::Deadline deadline_;
    void merge_joint_paths(SearchNode& node);
    pexh::Solution representative(const SearchNode& node, const JointPathPair& joint) const;
    void prune(SearchNode& node);
};
