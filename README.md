# BB-MO-CBS-pex-H (BB-MO-CBS-apex-H1)

C++ implementation of **BB-MO-CBS-pex-H**, extending
[FangjiW/BBMOCBS-approx](https://github.com/FangjiW/BBMOCBS-approx) with the supplied
conflict-analysis and lower-bound pruning algorithm. `BB-MO-CBS-apex-H1` is this
repository's name; `BBMOCBS-pex-H` is the canonical solver name. The default
matching limit is **K = 1**; larger K includes additional agent-disjoint cardinal
conflicts.

The complete upstream source, low-level Apex search, original solvers and sample
map are included. The original GPL-3.0 license is retained; see [NOTICE](NOTICE).
Generated build files and the upstream precompiled Linux binary are excluded.

## Build

Requires a C++14 compiler, CMake 3.16 or newer, and Boost headers/program_options.
Python 3 is optional for CLI regression checks and independent output validation.

```sh
# Ubuntu/Debian dependencies:
sudo apt install build-essential cmake libboost-program-options-dev
# On macOS with Homebrew:
# brew install cmake boost

cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure
```

Executable: `build/bin/bbmocbs_approx`. Run it with `--help` for all options.
For a custom Boost installation, pass `-DCMAKE_PREFIX_PATH=/path/to/boost` to CMake.

## Run a small example

Run these commands from the repository root:

```sh
./build/bin/bbmocbs_approx \
  -m example/tiny.map -s example/tiny.scen -n 2 -d 2 \
  --c1 example/tiny-1.cost --c2 example/tiny-2.cost \
  -a BBMOCBS-pex-H -e 0.05 -K 1 -t 30 \
  -o output.txt --paths_file paths.json
```

`-K 1` runs the H1 configuration. `-K 2` allows two disjoint conflict pairs and
up to four labeled bound vectors. In general, Full has `2^|M|` vectors: raising K
can materially increase memory and runtime. `-k` is a different, legacy option
controlling the solution count for `BBMOCBS-k`.

A larger supplied example:

```sh
./build/bin/bbmocbs_approx \
  -m example/random-32-32-20.map \
  -s example/random-32-32-20-random.scen -n 12 -d 2 \
  --c1 example/random-1.cost --c2 example/random-2.cost \
  -a BBMOCBS-pex-H -e 0.05 -K 2 -t 120 \
  -o output.txt --paths_file paths.json
```

Optional `--CB true` prefers representatives with fewer conflicts when merging.
The default is `--CB false`. `--eager false` is required for both shared pex solvers, which follow
the supplied representative-at-a-time algorithm. `BBMOCBS-pex` now runs the same implementation as H with the H steps disabled:
same low-level search, merging, goal occupancy, solution bookkeeping, and deadline.
This makes the comparison measure the heuristic. The imported implementation is
still available as `BBMOCBS-pex-legacy`, with its final cost/path alignment bug fixed.
Legacy eager updates are available only with that legacy name. `BBMOCBS-eps` and
`BBMOCBS-k` remain available. `BBMOCBS-pex-H1` and `BBMOCBS-apex-H1` are aliases for
`BBMOCBS-pex-H`; use `-K 1` for H1.

## Three-objective terminal comparison

The third bundled grid, `example/travel-time.cost`, charges one per move or wait.
The objectives are the two original cost grids plus sum of travel times (not
makespan). All agents stay at their goals after final arrival, without extra cost.

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j 4
ctest --test-dir build --output-on-failure
python3 scripts/compare_pex.py --binary build/bin/bbmocbs_approx \
  --agents 2 4 6 8 10 12 14 16 --epsilon 0.05 --time-limit 300
```

This automatically runs **both** pex and H1, prints screenshot-style result lines,
and saves a timestamped `results/compare-*/` directory containing:

- `comparison.log`: the same summary printed to the terminal;
- `summary.csv`: timing, split counts, cache/pruning counters and validation status;
- `comparisons.json`: full cost sets, set differences, equality and coverage checks;
- per-run `.log`, `.txt`, and `.json` files: progress, solver reports and actual paths;
- `config.json` and `source.patch`: inputs and source-state information.

Pass `--repeats 3` for repeated timings; the solver order alternates between repeats.
Pass `--scenarios path/to/one.scen path/to/two.scen` to compare multiple scenarios for
one map, and `--map`/`--costs c1 c2 c3` for a different dataset. Defaults use the
bundled 32×32 map and its single scenario. Each invocation creates a new output
folder; an explicit `--output` folder must not already exist.

To check **exact Pareto cost equality**, use epsilon zero:

```sh
python3 scripts/compare_pex.py --binary build/bin/bbmocbs_approx \
  --agents 2 4 6 8 --epsilon 0 --time-limit 300
```

For epsilon greater than zero, different representative cost vectors or counts
are permitted. The script validates costs against paths, collision freedom,
apex bounds, and bidirectional epsilon coverage between returned sets. Mutual
coverage is a consistency check, not an independent proof of global completeness.
If either solver times out, front comparison is marked `INCOMPLETE`.
The script exits nonzero on invalid paths/costs or a completed comparison failure.

Exact scalar A* uses admissible reverse distances and a bounded constraint-key
cache. Deterministic low-level path sets are cached under identical constraints
when `--CB false`; CAT-dependent searches with `--CB true` bypass this cache.
For an unoptimized reference run, pass `--scalar-optimizations false
--low-level-cache false` to the comparison script (the executable uses underscore
option names). These options affect work performed, not approximation factors.

## Inputs and output

- MovingAI `.map` files: `.` is traversable, `@` and `T` are blocked.
- MovingAI `.scen` files: use the first `-n` agents. Endpoints must be traversable.
- Each `--c1`, `--c2`, and (for `-d 3`) `--c3` file is a grid of nonnegative integer
  vertex-entry costs with the map's dimensions. Each cell must fit in `INT_MAX`.
- The CLI supports two or three objectives. The new solver's C++ API accepts
  arbitrary positive objective dimensions. Movement is four-connected, with a
  wait action. Moving or waiting pays the entered cell's cost. Agents occupy
  their final goals indefinitely, at no extra cost after their final arrival.
- `--turn_dim` optionally chooses a zero-based objective for `--turn_cost`.
  The upstream convention is preserved: the first non-wait move pays a turn
  penalty, and a change in the immediately preceding displacement pays another.
  A wait has displacement zero, so entering/leaving a wait can incur a penalty.

The text output is appended to `-o`. It contains the status, apex and actual cost
vectors, timing, and counters for analyses, exact scalar searches, cache reuse,
priority updates, node pruning and child pruning.

`--paths_file paths.json` writes machine-readable solutions and completion status.
Both shared solvers support this export. Paths contain vertex IDs assigned to traversable map cells in row-major order.
The JSON keeps apex, actual cost and paths together for each solution.

- `SUCCESS` / `completed: true`: search exhausted Open and returned solutions.
- `INFEASIBLE` / `completed: true`: search exhausted Open without a solution.
- `TIMEOUT` / `completed: false`: the time limit interrupted search. Returned
  incumbents are feasible, but coverage of the entire Pareto front is uncertified.

Exit codes: 0 for completed search (including infeasible), 1 for input/runtime
errors, 2 for a shared pex solver timeout. General unsolvable MAPF instances may reach the
cutoff; the finite single-agent horizon does not impose a joint CBS depth cutoff.

Validate an exported example independently:

```sh
python3 validation/check_output.py \
  --map example/tiny.map --scenario example/tiny.scen \
  --cost example/tiny-1.cost --cost example/tiny-2.cost \
  --result paths.json --agents 2
```

For turn-cost runs, also pass the matching `--turn-dim` and `--turn-cost` options.

## Implementation and verification

See [the algorithm mapping](docs/ALGORITHM.md) for the correspondence to the
supplied pseudocode, cache invalidation, feasibility handling and bound bookkeeping.

`validation/pex_h_regression.cpp` uses an independent exhaustive joint-state solver
to compare exact Pareto fronts at epsilon zero and check epsilon coverage at positive
epsilon. It also checks directed-edge/vertex constraints, future goal occupancy,
K=2 matching, branch labels surviving global dominance, transactional coverage,
priority-cache reuse, and returned path costs/collisions. Checks use explicit
exceptions and remain enabled in Release builds. `validation/smoke_cli.py` checks
input errors, both solver aliases, turn costs, and JSON output.

These are bounded regression checks, not a formal proof of the supplied algorithm
or a performance study. The independent oracle is the correctness reference for the tiny instances.
Optimized H is also checked against uncached scalar Dijkstra with identical
returned paths and costs. See [the audit and measured results](docs/HEURISTIC_AUDIT.md)
for the scope and limitations of the three-objective comparison.
