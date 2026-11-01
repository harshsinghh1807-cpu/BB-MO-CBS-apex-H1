#!/usr/bin/env python3
"""Independently validate pex-H JSON paths against a MovingAI map and cost grids."""
import argparse
import json
from pathlib import Path


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
    assert len(endpoints) == agents
    assert result['algorithm'] == 'BBMOCBS-pex-H'
    for solution in result['solutions']:
        paths = solution['paths']
        assert len(paths) == agents
        actual = [0] * len(grids)
        for agent, path in enumerate(paths):
            assert path and (path[0], path[-1]) == endpoints[agent]
            previous_direction = (0, 0)
            for t, (u, v) in enumerate(zip(path, path[1:])):
                a, b = cells[u], cells[v]
                direction = b[0]-a[0], b[1]-a[1]
                assert abs(direction[0])+abs(direction[1]) <= 1
                for objective, grid in enumerate(grids):
                    actual[objective] += grid[b[0]][b[1]]
                    if objective == turn_dim and direction != previous_direction:
                        actual[objective] += turn_cost
                previous_direction = direction
        assert actual == solution['cost'], (actual, solution['cost'])
        assert all(c <= (1+result['epsilon'])*a + 1e-9
                   for c, a in zip(actual, solution['apex']))
        position = lambda agent, t: paths[agent][min(t, len(paths[agent])-1)]
        for t in range(max(map(len, paths))):
            for a in range(agents):
                for b in range(a+1, agents):
                    assert position(a, t) != position(b, t), ('vertex conflict', a, b, t)
                    assert not (position(a, t) == position(b, t+1) and
                                position(b, t) == position(a, t+1)), ('edge conflict', a, b, t)
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
