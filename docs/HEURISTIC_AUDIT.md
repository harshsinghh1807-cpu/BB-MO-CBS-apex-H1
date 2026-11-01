# Heuristic audit and three-objective rerun — 2026-11-01

The audit checked the implementation against the supplied BB-MO-CBS-pex-H
pseudocode, verified the changes already present locally, added bounded low-level
memoization, and reran the comparison. This is code review plus bounded empirical
validation, not a formal proof for every instance.

## Findings and corrections

The current H loop implements the supplied control flow: per-agent apex ideals;
exact scalar costs under the candidate split constraint; nonnegative deltas;
cardinality when both side vectors are nonzero; lexicographic pair ranking;
agent-disjoint matching; labeled Full enumeration; lexicographic key raising;
transactional BoundCovered bookkeeping; child bounds taken from Full before ND;
and cache invalidation when the representative changes. No discrepancy was found
in these H-specific rules during this audit. See [ALGORITHM.md](ALGORITHM.md).

The earlier comparison had two separate sources of confusion:

1. The imported pex post-processing could use filtered indices into the original
   cost array and leave paths in their old order. The corrected code filters and
   sorts apex/cost/path triples together. A regression fixture explicitly catches
   the original index mismatch.
2. The imported pex and new H implementations used different merging, goal
   occupancy and conflict-selection code. `BBMOCBS-pex` now uses the same core as H,
   with the H steps disabled, so nonheuristic differences are controlled. The
   imported solver remains `BBMOCBS-pex-legacy`. Consequently the new baseline's
   timings should not be treated as measurements of the unchanged upstream solver.

The validated performance improvements are:

- Exact scalar A* with admissible reverse-distance bounds, plus a bounded cache
  keyed by agent, objective and the complete canonical constraint set. Reference
  Dijkstra remains available. A* does not use an inflated or approximate heuristic.
- Cached partial-joint actual costs and conflict counts during merging, avoiding
  repeated reconstruction of every prefix's paths and costs.
- A bounded cache of complete low-level path sets for identical constraints when
  conflict-based merging is off. It is disabled for CAT-dependent merging. Timeouts
  are never cached as infeasibility. Both pex and H receive the same optimization.
- JSON records retain full floating-point epsilon precision, and path validation
  uses explicit checks even when Python is run with optimization enabled.

## Dataset and settings

Map: `example/random-32-32-20.map`, one bundled scenario
`example/random-32-32-20-random.scen`, taking its first N agents.

Objectives: sum of costs from `random-1.cost`, sum from `random-2.cost`, and
sum of moves/waits from the all-ones `travel-time.cost`. The third objective is
sum of travel times, not makespan. Grid cells corresponding to obstacles are unused.

Approximate experiment: N = 2,4,6,8,10,12,14,16; epsilon = 0.05; K = 1;
CB = false; eager = false; 300-second per-solver limit; Release build on the
local macOS machine. Three sequential repetitions alternated algorithm order.
The reported medians are from those repetitions, with no concurrent build/test
work during the approximate performance run. They are observations on this one
scenario, not a general speedup claim.

## Measured results

All 48 approximate runs completed. The saved [CSV](benchmark-3obj.csv) includes
all median timings, conflict splits, solution counts and cache/pruning counters.
Full representative cost sets and differences are in
[benchmark-3obj-costs.json](benchmark-3obj-costs.json).

Selected medians, pex versus H1:

- N=10: 0.039908 s versus 0.044625 s; 14 versus 14 conflict splits.
- N=12: 0.938777 s versus 0.456458 s; 50 versus 49 splits.
- N=14: 5.132130 s versus 4.634520 s; 1860 versus 1551 splits.
- N=16: 4.958270 s versus 3.491240 s; 1784 versus 1092 splits.

H1 is not faster on every instance. At N=8 and N=10 it performed no child-bound
pruning, and the extra analysis did not reduce split counts. At N=12/14/16 the
bounds pruned 14/20/11 children before low-level search; H also chose different
conflicts and reduced splits. Timing is affected by the difficulty of the individual
searches and cache reuse, not just split counts.

The additional low-level cache preserved every exported apex, cost and path across
all 16 solver/agent-count combinations when compared with the immediately preceding
uncached run. Its benefit is independent of the H logic: it saved 2975 pex low-level
searches and 2411 H1 searches at N=14 in this dataset.

## Cost-vector checks

Every returned path was independently checked for endpoints, legal moves, vertex
collisions, edge swaps and permanent goal occupancy. Every actual cost vector was
recomputed from its path and all three input cost grids. Each cost was also checked
against its stored epsilon-bounded apex. All returned outputs passed.

At epsilon 0.05:

- N=2/4/6/8 returned identical cost sets between pex and H1.
- N=10/12/14/16 returned different sets, with mutual epsilon coverage in every repeat.
- Different approximate representatives and solution counts are allowed by the
  algorithm. The number of representatives alone is not a quality ranking.
- Mutual coverage is a necessary consistency check between these two outputs;
  it cannot prove that neither solver missed the same part of the true front.

At epsilon zero, additional real-map runs completed for N=2/4/6/8/10. Both solvers
returned identical cost vectors: respectively 16/41/57/123/145 vectors. Larger
exact cases N=12/14/16 hit the separate 60-second cutoff for both solvers; incomplete outputs are recorded
as `INCOMPLETE`, never as a passing exact-front comparison. These exact stress runs
are not used for the approximate performance medians. Their [status log](benchmark-exact-3obj.csv) preserves the incomplete runs.

## Regression evidence and reproduction

The C++ suite passed 6362 assertions across 106 independent exhaustive-oracle
instances. It checks baseline and H against the exact tiny-instance oracle,
approximation coverage, path costs/collisions, three objectives including unit
travel time, K=2, future goal constraints, zero costs, branch labels, coverage
bookkeeping and cache reuse. It also verifies that optimized H returns the same
paths and costs as uncached scalar Dijkstra. All three CTest targets pass in Release
and with AddressSanitizer/UndefinedBehaviorSanitizer.

From the repository root:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j 4
ctest --test-dir build --output-on-failure
python3 scripts/compare_pex.py --binary build/bin/bbmocbs_approx \
  --agents 2 4 6 8 10 12 14 16 --epsilon 0.05 --time-limit 300 --repeats 3
```

For completed exact-vector comparisons:

```sh
python3 scripts/compare_pex.py --binary build/bin/bbmocbs_approx \
  --agents 2 4 6 8 10 --epsilon 0 --time-limit 300
```

Each command creates a fresh directory under `results/` containing the printed
comparison log, CSV, per-run logs/reports/paths and cost-set comparisons. Raw logs
are kept locally and ignored by Git; compact measured results are committed above.
