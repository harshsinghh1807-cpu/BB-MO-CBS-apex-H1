#!/usr/bin/env python3
"""Independently validate pex-H JSON paths against a MovingAI map and cost grids."""
import argparse
import json
import math
from pathlib import Path


def require(condition, message):
    if not condition:
        raise ValueError(message)


def validate(map_path, scenario_path, cost_paths, result_path, agents, turn_dim=-1, turn_cost=5):
    lines = Path(map_path).read_text().splitlines()
    height = int(lines[1].split()[1])
    width = int(lines[2].split()[1])
    cells = [(y, x) for y, row in enumerate(lines[4:4+height])
             for x, char in enumerate(row) if char == '.']
    ids = {cell: i for i, cell in enumerate(cells)}
    endpoints = []
    for line in Path(scenario_path).read_text().splitlines()[1:agents+1]:
        fields = line.split()
        endpoints.append((ids[int(fields[5]), int(fields[4])],
                          ids[int(fields[7]), int(fields[6])]))
    grids = [[list(map(int, line.split())) for line in Path(p).read_text().splitlines()]
             for p in cost_paths]
    result = json.loads(Path(result_path).read_text())
    require(len(endpoints) == agents, 'scenario has fewer agents than requested')
    require(all(len(grid) == height and all(len(row) == width for row in grid) for grid in grids), 'cost grid dimensions differ from map')
    require(isinstance(result['completed'], bool), 'completed must be Boolean')
    require(math.isfinite(result['epsilon']) and result['epsilon'] >= 0, 'invalid epsilon')
    require(result.get('objectives', len(grids)) == len(grids), 'wrong objective dimension')
    require(result['algorithm'] in ('BBMOCBS-pex-H', 'BBMOCBS-pex'), 'unexpected algorithm')
    for solution in result['solutions']:
        paths = solution['paths']
        require(len(paths) == agents, 'wrong agent count')
        require(len(solution['cost']) == len(grids) and len(solution['apex']) == len(grids), 'wrong cost/apex dimension')
        require(all(isinstance(c, int) and c >= 0 for c in solution['cost'] + solution['apex']), 'costs must be nonnegative integers')
        actual = [0] * len(grids)
        for agent, path in enumerate(paths):
            require(path and all(isinstance(v, int) and 0 <= v < len(cells) for v in path), 'invalid path vertex')
            require((path[0], path[-1]) == endpoints[agent], 'path endpoints disagree with scenario')
            previous_direction = (0, 0)
            for t, (u, v) in enumerate(zip(path, path[1:])):
                a, b = cells[u], cells[v]
                direction = b[0]-a[0], b[1]-a[1]
                require(abs(direction[0])+abs(direction[1]) <= 1, 'path makes an illegal move')
                for objective, grid in enumerate(grids):
                    actual[objective] += grid[b[0]][b[1]]
                    if objective == turn_dim and direction != previous_direction:
                        actual[objective] += turn_cost
                previous_direction = direction
        require(actual == solution['cost'], f"cost mismatch: recomputed {actual}, reported {solution['cost']}")
        require(all(c <= (1+result['epsilon'])*a + 1e-9
                    for c, a in zip(actual, solution['apex'])), 'cost is not epsilon-bounded by its apex')
        position = lambda agent, t: paths[agent][min(t, len(paths[agent])-1)]
        for t in range(max(map(len, paths))):
            for a in range(agents):
                for b in range(a+1, agents):
                    require(position(a, t) != position(b, t), f'vertex conflict: agents {a}, {b}, time {t}')
                    require(not (position(a, t) == position(b, t+1) and
                                 position(b, t) == position(a, t+1)), f'edge conflict: agents {a}, {b}, time {t}')
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', required=True)
    parser.add_argument('--scenario', required=True)
    parser.add_argument('--cost', action='append', required=True)
    parser.add_argument('--result', required=True)
    parser.add_argument('--agents', type=int, required=True)
    parser.add_argument('--turn-dim', type=int, default=-1)
    parser.add_argument('--turn-cost', type=int, default=5)
    args = parser.parse_args()
    result = validate(args.map, args.scenario, args.cost, args.result, args.agents,
                      args.turn_dim, args.turn_cost)
    print(f"Validated {len(result['solutions'])} solutions; completed={result['completed']}")
