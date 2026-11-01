#!/usr/bin/env python3
"""Run a reproducible pex/H1 comparison and independently validate every solution."""
import argparse
import csv
import json
import hashlib
import math
from datetime import datetime
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'validation'))
from check_output import validate


def nondominated(costs):
    result = []
    for candidate in sorted(set(map(tuple, costs))):
        if not any(all(a <= b for a, b in zip(other, candidate)) for other in result):
            result.append(candidate)
    return result


def covers(covering, targets, epsilon):
    return all(any(all(a <= (1+epsilon)*b + 1e-9 for a, b in zip(c, target))
                   for c in covering) for target in targets)


def compare(first, second, epsilon):
    a = sorted(set(tuple(s['cost']) for s in first['solutions']))
    b = sorted(set(tuple(s['cost']) for s in second['solutions']))
    complete = first['completed'] and second['completed']
    same_front = nondominated(a) == nondominated(b)
    mutual = covers(a, b, epsilon) and covers(b, a, epsilon)
    return dict(both_completed=complete, identical_cost_sets=a == b,
                identical_pareto_fronts=same_front, mutual_epsilon_coverage=mutual,
                check='INCOMPLETE' if not complete else 'PASS' if (same_front if epsilon == 0 else mutual) else 'FAIL',
                pex_costs=a, h1_costs=b,
                only_pex=sorted(set(a)-set(b)), only_h1=sorted(set(b)-set(a)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', type=Path, default=ROOT/'build/bin/bbmocbs_approx')
    parser.add_argument('--map', type=Path, default=ROOT/'example/random-32-32-20.map')
    parser.add_argument('--scenarios', type=Path, nargs='+', default=[ROOT/'example/random-32-32-20-random.scen'])
    parser.add_argument('--costs', type=Path, nargs='+', default=[ROOT/'example/random-1.cost', ROOT/'example/random-2.cost', ROOT/'example/travel-time.cost'])
    parser.add_argument('--agents', type=int, nargs='+', default=list(range(2, 17, 2)))
    parser.add_argument('--epsilon', type=float, default=0.05)
    parser.add_argument('--time-limit', type=int, default=300)
    parser.add_argument('--repeats', type=int, default=1)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--cb', choices=['true', 'false'], default='false')
    parser.add_argument('--low-level-cache', choices=['true', 'false'], default='true')
    parser.add_argument('--scalar-optimizations', choices=['true', 'false'], default='true')
    args = parser.parse_args()
    if len(args.costs) not in (2, 3) or not math.isfinite(args.epsilon) or args.epsilon < 0 or min(args.agents+[args.time_limit,args.repeats]) < 1:
        parser.error('require 2 or 3 cost maps, finite epsilon >= 0, positive agents/time/repeats')
    for file in [args.binary,args.map,*args.scenarios,*args.costs]:
        if not file.is_file():
            parser.error(f'file not found: {file}')
    destination = args.output or ROOT/'results'/datetime.now().strftime('compare-%Y%m%d-%H%M%S-%f')
    destination.mkdir(parents=True, exist_ok=False)
    config = vars(args).copy()
    config['solver_names'] = ['BBMOCBS-pex', 'BBMOCBS-pex-H1']
    config['input_sha256'] = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in [args.binary,args.map,*args.scenarios,*args.costs]}
    config['baseline'] = 'Shared pex implementation with the H steps disabled; upstream implementation remains BBMOCBS-pex-legacy.'
    config['git_commit'] = subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()
    config['working_tree_changes'] = subprocess.check_output(['git','status','--short'],cwd=ROOT,text=True)
    (destination/'config.json').write_text(json.dumps(config,default=str,indent=2)+'\n')
    (destination/'source.patch').write_text(subprocess.check_output(['git','diff','HEAD'],cwd=ROOT,text=True))
    rows, comparisons = [], []
    failures = 0
    with (destination/'comparison.log').open('w',buffering=1) as summary:
        def emit(line):
            print(line, flush=True)
            summary.write(line+'\n')
        emit(f'Results: {destination.resolve()}')
        emit(f'Objectives={len(args.costs)} epsilon={args.epsilon} K=1 cutoff={args.time_limit}s; conflicts means resolved conflict splits.')
        emit('Baseline uses the same low-level search, merging and goal semantics as H1, with H disabled.')
        for instance, scenario in enumerate(args.scenarios,1):
            for agents in args.agents:
                for repeat in range(1,args.repeats+1):
                    pair = {}
                    algorithms = [('BBMOCBS-pex','BB-MO-CBS-pex'),('BBMOCBS-pex-H1','BB-MO-CBS-pex-H1')]
                    if repeat % 2 == 0:
                        algorithms.reverse()
                    for algorithm,label in algorithms:
                        stem = destination/f'inst{instance}_N{agents}_r{repeat}_{algorithm}'
                        report, detail, paths = (stem.with_suffix(ext) for ext in ('.txt','.log','.json'))
                        command = [str(args.binary.resolve()),'-m',str(args.map),'-s',str(scenario),'-n',str(agents),'-d',str(len(args.costs)),
                                   '-e',str(args.epsilon),'-a',algorithm,'-K','1','--CB',args.cb,'--eager','false',
                                   '--scalar_optimizations',args.scalar_optimizations,'--low_level_cache',args.low_level_cache,'-t',str(args.time_limit),'-o',str(report),'--paths_file',str(paths)]
                        for objective, cost in enumerate(args.costs,1):
                            command += [f'--c{objective}',str(cost)]
                        started = time.monotonic()
                        with detail.open('w') as stream:
                            try:
                                process = subprocess.run(command,stdout=stream,stderr=subprocess.STDOUT,timeout=args.time_limit+10)
                                code = process.returncode
                            except subprocess.TimeoutExpired:
                                code = None
                        wall = time.monotonic()-started
                        row = dict(instance=instance,scenario=str(scenario),agents=agents,repeat=repeat,algorithm=label,
                                   status='ERROR',solutions=None,conflict_splits=None,total_seconds=wall,path_validation='NOT_CHECKED')
                        if code in (0,2) and paths.exists():
                            try:
                                data = validate(args.map,scenario,args.costs,paths,agents)
                                if data['completed'] != (code == 0):
                                    raise ValueError('completion flag and exit code disagree')
                                pair[algorithm] = data
                                row.update(data['statistics'])
                                row.update(status=('solved' if data['solutions'] else 'INFEASIBLE') if data['completed'] else 'TIMEOUT',
                                           solutions=len(data['solutions']),path_validation='PASS')
                            except (AssertionError,ValueError,KeyError,IndexError) as error:
                                row['path_validation'] = f'FAIL: {error}'
                        elif code is None:
                            row['status'] = 'EXTERNAL_TIMEOUT'
                        if row['status'] == 'ERROR' or row['path_validation'].startswith('FAIL'):
                            failures += 1
                        row['process_wall_seconds'] = wall
                        row['exit_code'] = code
                        rows.append(row)
                        emit(f"inst {instance} N={agents} {label}: {row['status']}, {row['solutions']} solutions, "
                             f"{row['conflict_splits']} conflicts, {row['total_seconds']:.6f}s [paths: {row['path_validation']}]"
                             + (f' repeat={repeat}' if args.repeats>1 else ''))
                    if len(pair)==2:
                        comparison = compare(pair['BBMOCBS-pex'],pair['BBMOCBS-pex-H1'],args.epsilon)
                        comparison.update(instance=instance,agents=agents,repeat=repeat)
                        comparisons.append(comparison)
                        failures += comparison['check']=='FAIL'
                        emit(f"  vectors: {'identical' if comparison['identical_cost_sets'] else 'different'}; "
                             f"{'exact Pareto equality' if args.epsilon==0 else 'mutual epsilon coverage'}: {comparison['check']}")
                    (destination/'comparisons.json').write_text(json.dumps(comparisons,indent=2)+'\n')
                    fields = sorted(set().union(*(r.keys() for r in rows)))
                    with (destination/'summary.csv').open('w',newline='') as stream:
                        writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader();writer.writerows(rows)
        emit('Validation complete.' if not failures else f'Validation failures: {failures}; inspect saved logs.')
        emit('Mutual coverage compares the returned sets; only the small independent oracle tests certify comparison against the exact front.')
    return 1 if failures else 0


if __name__ == '__main__':
    raise SystemExit(main())
