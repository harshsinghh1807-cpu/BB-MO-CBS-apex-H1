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
The default is `--CB false`. `--eager false` is required for pex-H, which follows
the supplied representative-at-a-time algorithm. The original `BBMOCBS-pex`,
`BBMOCBS-eps` and `BBMOCBS-k` remain selectable with `-a`; they retain their original
algorithm behavior and do not use the new bound analysis. `BBMOCBS-apex-H1` is an
alias for `BBMOCBS-pex-H` (the K option still applies).

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
Paths contain vertex IDs assigned to traversable map cells in row-major order.
The JSON keeps apex, actual cost and paths together for each solution.

- `SUCCESS` / `completed: true`: search exhausted Open and returned solutions.
- `INFEASIBLE` / `completed: true`: search exhausted Open without a solution.
- `TIMEOUT` / `completed: false`: the time limit interrupted search. Returned
  incumbents are feasible, but coverage of the entire Pareto front is uncertified.

Exit codes: 0 for completed search (including infeasible), 1 for input/runtime
errors, 2 for a pex-H timeout. General unsolvable MAPF instances may reach the
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
or a performance study. The retained legacy solver is useful for comparisons, but
the independent oracle is the correctness reference for the tiny instances.
