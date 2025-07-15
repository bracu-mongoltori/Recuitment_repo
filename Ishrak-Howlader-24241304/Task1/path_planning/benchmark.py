#!/usr/bin/env python3
import os
import time
import argparse
import pandas as pd

from utils import load_grid, manhattan
from algorithms import dijkstra, astar

def single_run(algo_name, func, grid, start, goal, heuristic=None):
    """Run one invocation, return (runtime_ms, path_len, success_bool)."""
    t0 = time.perf_counter()
    if algo_name == 'dijkstra':
        explored, path = func(grid, start, goal)
    else:
        explored, path = func(grid, start, goal, heuristic)
    t1 = time.perf_counter()

    runtime_ms = (t1 - t0) * 1000
    success    = (len(path) > 0 and path[-1] == goal)
    path_len   = len(path)
    return runtime_ms, path_len, success

def benchmark(maps, start, goal, runs=5):
    records = []
    algos = [
        ('dijkstra', dijkstra,         None),
        ('astar_manhattan', astar,     manhattan)
    ]
    for map_path in maps:
        grid_name = os.path.splitext(os.path.basename(map_path))[0]
        grid = load_grid(map_path)
        for algo_name, func, heuristic in algos:
            for i in range(runs):
                rt, plen, succ = single_run(algo_name, func, grid, start, goal, heuristic)
                records.append({
                    'map':       grid_name,
                    'algorithm': algo_name,
                    'run':       i+1,
                    'runtime_ms': round(rt,2),
                    'path_len':   plen,
                    'success':    succ
                })

    df = pd.DataFrame(records)
    # compute per-algo summary
    summary = df.groupby('algorithm').agg({
        'runtime_ms': 'mean',
        'path_len':   'mean',
        'success':    'mean'
    }).reset_index()
    summary['success_rate_%'] = (summary['success'] * 100).round(1)
    summary = summary.rename(columns={
        'runtime_ms':    'avg_runtime_ms',
        'path_len':      'avg_path_len',
        'success':       'success_frac'
    })[
        ['algorithm','avg_runtime_ms','avg_path_len','success_rate_%']
    ]
    return df, summary

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--maps', nargs='+',
                        default=[
                          '../grids/maps1.csv',
                          '../grids/maps2.csv',
                          '../grids/maps3.csv'
                        ],
                        help='List of grid CSV paths')
    parser.add_argument('--start', nargs=2, type=int, default=(0,0),
                        help='Start cell i j')
    parser.add_argument('--goal',  nargs=2, type=int, default=(9,9),
                        help='Goal cell i j')
    parser.add_argument('--runs', type=int, default=5,
                        help='Repetitions per map & algorithm')
    args = parser.parse_args()

    df, summary = benchmark(
        maps=args.maps,
        start=tuple(args.start),
        goal=tuple(args.goal),
        runs=args.runs
    )

    # save detailed and summary results
    os.makedirs('results', exist_ok=True)
    df.to_csv('results/benchmark_details.csv', index=False)
    summary.to_csv('results/benchmark_summary.csv', index=False)

    print("\n=== Benchmark Summary ===")
    print(summary.to_string(index=False))
