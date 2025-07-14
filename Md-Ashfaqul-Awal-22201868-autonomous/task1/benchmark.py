import time
import numpy as np
from dijkstra import dijkstra
from astar import astar
from utils import load_grid

def benchmark_path_planning(grid_files, start_goal_pairs, algorithms, heuristic='manhattan', runs=5):
    results = []
    for grid_file in grid_files:
        grid = load_grid(grid_file)
        for start, goal in start_goal_pairs[grid_file]:
            for algo in algorithms:
                total_time = 0
                success_count = 0
                path_lengths = []
                for _ in range(runs):
                    t0 = time.time()
                    if algo == 'dijkstra':
                        path, _ = dijkstra(grid, start, goal)
                    else:
                        path, _ = astar(grid, start, goal, heuristic=heuristic)
                    t1 = time.time()
                    elapsed = (t1 - t0) * 1000  # ms
                    total_time += elapsed
                    if path is not None:
                        success_count += 1
                        path_lengths.append(len(path))
                avg_time = total_time / runs
                avg_path_length = np.mean(path_lengths) if path_lengths else 0
                success_rate = (success_count / runs) * 100
                results.append({
                    'map': grid_file,
                    'start': start,
                    'goal': goal,
                    'algorithm': algo,
                    'avg_runtime_ms': avg_time,
                    'avg_path_length': avg_path_length,
                    'success_rate_percent': success_rate
                })
    return results

def print_benchmark_table(results):
    print(f"{'Map':<15} {'Algorithm':<10} {'Avg Time (ms)':<15} {'Avg Path Len':<15} {'Success Rate (%)':<18}")
    print("-" * 75)
    for res in results:
        print(f"{res['map']:<15} {res['algorithm']:<10} {res['avg_runtime_ms']:<15.2f} {res['avg_path_length']:<15.2f} {res['success_rate_percent']:<18.1f}")

if __name__ == "__main__":
    # Example usage
    maps = ['maps/map1.csv', 'maps/map2.csv', 'maps/map3.csv']
    start_goal = {
        'maps/map1.csv': [((0, 0), (9, 9))],
        'maps/map2.csv': [((1, 1), (13, 13))],
        'maps/map3.csv': [((0, 0), (19, 19))]
    }
    algorithms = ['dijkstra', 'astar']
    results = benchmark_path_planning(maps, start_goal, algorithms, heuristic='manhattan', runs=5)
    print_benchmark_table(results)

