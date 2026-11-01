# Mapping to the supplied BB-MO-CBS-pex-H pseudocode

## Representation

All objectives minimize nonnegative integral costs. `CostVector` uses `size_t`.
`MAX_COST` is reserved for infeasibility. Finite cost overflow raises an error;
infinity is propagated without unsigned wraparound.

`SearchNode` extends the upstream `HighLevelNode`: the per-agent waypoint, apex
and cost maps represent `{Π^i}`; vertex/edge constraints represent Ω; `all_jps`
represents ℙ. Joint paths sort lexicographically by apex, then by path IDs for a
stable tie break. `key` is separate from the representative's apex. Open compares
immutable key snapshots, using an insertion sequence number for equal keys.

A solution stores its apex, actual cost, and representative paths in one object.
For all retained representatives, `c[o] <= (1+epsilon)*A[o]` for every objective.
Comparison with epsilon is componentwise; key ordering, pair scores, lexmin and
lexmax are lexicographic. In particular, lexmax is **not** componentwise maximum.

## Main loop (`src/HighLevel/pexHSolver.cpp`)

1. Initialize each agent's set using upstream ApexSearch in constraint-safe mode;
   an empty root set proves root infeasibility. Merge the sets and enqueue by apex.
2. On extraction, PruneApproxDom updates the covering solution apex and removes
   covered joint representatives. An empty list discards the node.
3. If pruning changes the first representative, reset its analysis cache and
   reinsert with the new representative's apex (pseudocode lines 9–11).
4. A conflict-free representative goes through epsilon-aware AddSolution. Remove
   it, discard the cached analysis, and reinsert any remainder by its new apex.
5. For a conflicting representative, compute ConflictAnalysis once and retain
   it on the node. Build Full, keeping the selected conflict's branch label.
6. Raise priority to `lexmax(representative.apex, lexmin(ND(Full)))`. If this raises
   the stored key, reinsert and reuse the cache on the next extraction.
7. BoundCovered on `ND(Full)` may discard the entire node.
8. For each side of the chosen conflict, derive `W_l` from **Full**, then take ND.
   Only after unsuccessful child-bound pruning, copy the node, add that side's
   constraint, and rerun ApexSearch for the affected agent.
9. Skip an empty low-level result. Merge the new per-agent sets and enqueue with
   `lexmax(new representative apex, lexmin(W_l))`. Children start without a cache.

The eager-solution optimization of upstream pex is not enabled in this variant.

## Supporting functions (`ConflictAnalysis.cpp` and `pexHSolver.cpp`)

MergeJointPaths starts with an empty zero-cost prefix, then folds in each agent's
set. For each pair of candidates, a merged apex is their componentwise minimum.
Either representative can be retained only if its actual cost epsilon-covers
that new apex. The merge preference chooses greater slack, or fewer conflicting
pairs with `--CB true`, when both representatives qualify. This is a deterministic
choice for the pseudocode's “a copy of P or P′”.

AddSolution tries the same apex-minimum merge with an existing solution and stores
all three pieces of the selected solution together. PruneApproxDom transfers the
removed path's apex to a covering solution by componentwise minimum.

BoundCovered first collects one witness solution for **every** bound. If any is
uncovered, it returns false without mutating any solution. Only after complete
coverage does it apply all componentwise-minimum bookkeeping updates. Different
bounds may use different solutions; witnesses need not be unique.

## ConflictAnalysis (`src/HighLevel/ConflictAnalysis.cpp`)

- `Ideal_x[o]` is the componentwise minimum of all stored **apex** vectors for x.
  `A_n` sums those ideals; it is not the selected representative's cost.
- Enumerate each conflicting unordered agent pair once, using its first conflict
  in temporal order. Agents remain at their last vertex after a path ends.
- For each split side and each objective, run exact scalar A* (or reference Dijkstra) under Ω plus
  that side's constraint. This search does not use approximate costs for Δ.
- Compute `delta=max(0, scalar optimum-Ideal_x[o])` with a guarded subtraction.
- A pair is cardinal when both side vectors are nonzero; neither vector needs
  to be positive in every coordinate. The score is the lexicographic minimum
  of its two delta vectors.
- Select the cardinal pair with greatest score. If none exists, select the first
  conflicting pair from deterministic agent-pair order. Ties retain pair order.
- Always include the selected pair, then greedily add other cardinal pairs by
  decreasing score if neither agent is used and the matching has fewer than K pairs.
- Generate both choices per matched pair, adding the chosen deltas to `A_n`.
  The first pair is the selected conflict; each Full entry records that pair's side.

Example: Full may contain `[1,1]` for side 0 and `[2,2]` for side 1. Global ND is
only `[1,1]`, but side 1's child bound must still be `[2,2]`. Filtering global ND
by label would incorrectly turn that child into an empty set and prune it.

If scalar search proves a side infeasible, its whole delta vector becomes infinity.
Finite-bound ND drops such alternatives: they represent no feasible descendants.
An empty child bound set is vacuously covered and can be skipped without a low-level
search. If no finite alternatives remain, the whole node is infeasible. Timeout
raises a separate exception and never creates an infeasible delta.

## Constrained low-level search (`src/LowLevel/ConstrainedSearch.cpp`)

Exact scalar search uses `(vertex, previous vertex, time)` when its objective includes
turn penalties; otherwise the previous vertex is omitted from the state key.
Time is folded after the last timed restriction and the earliest safe final goal
arrival. Since no later constraint distinguishes such states, this creates a finite
exact search without an arbitrary time horizon. Reverse distances are admissible
and consistent (constraints and nonnegative turn costs are omitted), so A* returns
the same optimum as Dijkstra. Its tie breaker prefers greater g at equal f; it never
weights or inflates the heuristic. Successful exact optima, including proven
infeasibility, can be reused for the same agent/objective/canonical constraint set.
Timed-out searches never enter the cache.

ApexSearch's optional constraint-safe mode is enabled by both shared pex variants, but not the legacy solver. It:

- Checks source occupancy at time zero and skips unreachable successors.
- Accepts final goal arrival only after the last constraint forbidding future goal
  occupancy (including an implicit goal self-loop constraint). Earlier visits to
  the goal may expand and leave again.
- Uses a finite horizon equal to the last constraint time plus a bound on the
  number of spatial states (including previous vertices when turns are enabled).
  With nonnegative costs, a nondominated suffix after all timed restrictions can
  be made simple in this state space. This is a completeness-derived bound, not
  a user-chosen search cutoff.
- Checks a shared steady-clock deadline and throws on timeout. The high-level
  solver returns feasible incumbents with `completed=false`.

Both low levels use the same directed departure-time edge constraints and turn
cost convention. Reverse shortest-path vectors ignore turn penalties, making them
admissible with nonnegative turn costs and consistent for ApexSearch. Goal occupancy
conflicts keep **both** positive agent IDs, so the agent that reached its goal first
can also be constrained and replanned.

## Scope of validation

Regression tests compare the new solver against a separate exhaustive joint-state
search on small two- and three-objective graphs, including positive epsilon,
conflict-based merging and a four-agent K=2 case. Separate checks cover bound labels,
bookkeeping, constrained scalar optima, goal revisits, infinity and deadlines.
The CLI validator recomputes costs and collisions without using solver helpers.
Large instances remain subject to exponential worst-case search and Full size.

## Controlled baseline and low-level cache

`BBMOCBS-pex` executes this same main loop with `use_heuristic=false`: it skips
ConflictAnalysis, key raising, whole-node bound pruning and child-bound pruning,
then branches on the first conflict and enqueues by representative apex. All
other code is identical. The upstream implementation remains separately named
`BBMOCBS-pex-legacy`; its output filtering/sorting now moves apex, cost and paths
together rather than using filtered indices into the original costs.

Both shared solvers memoize complete approximate path sets for MORE_SLACK searches
with identical canonical per-agent constraints. Graph, endpoints, epsilon, turn
settings and merge policy are constant within one run. Equivalent constraint
orderings/deduplicated entries have the same feasible paths and deterministic
search behavior. Each cached map is copied back into the node; solution or joint
apex bookkeeping never mutates the cached per-agent data. Infeasibility can be
cached only after normal search completion. The cache is local to a solver run,
bounded by waypoint/entry counts, and disabled for conflict-based merging because
its collision-avoidance table depends on the representative. Eviction affects only
performance. `--low_level_cache false` disables it for comparisons.

Regression checks compare optimized H to reference Dijkstra with both caches off,
and validate the baseline separately against the independent joint-state oracle.
The terminal runner independently recomputes all objective costs from the exported
paths, then compares exact nondominated sets at epsilon zero or mutual coverage at
positive epsilon. Neither timing wins nor matching approximate solution counts are
used as correctness criteria.
