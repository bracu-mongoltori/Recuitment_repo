import os
import random
from utils import load_grid_map
from algorithms import dijkstra, astar

# ----------- CONFIGURATION -----------
MAP_DIR = r"C:\Users\USERAS\Desktop\Mongol Tori\Task 1\grid_maps"
maps = ['map1.csv', 'map2.csv', 'map3.csv']
runs_per_map = 5
# -------------------------------------

def sample_free_cells(grid, k=2):
    free = [
        (r, c)
        for r in range(len(grid))
        for c in range(len(grid[0]))
        if grid[r][c] == 0
    ]
    if len(free) < k:
        raise ValueError("Not enough free cells to sample start/goal.")
    return random.sample(free, k)

# Print header
print("2. Benchmark Results")
print("Map\t\tAlgorithm\tAvg Time (ms)\tAvg Path Length\tSuccess Rate (%)")
print("---------------------------------------------------------------")

# Loop over each map file
for map_file in maps:
    map_path = os.path.join(MAP_DIR, map_file)

    for algo in ['dijkstra', 'astar']:
        total_time, total_length, success = 0, 0, 0

        # Run multiple trials for the same algorithm
        for _ in range(runs_per_map):
            grid = load_grid_map(map_path)
            start, goal = sample_free_cells(grid)

            if algo == 'dijkstra':
                path, _, runtime = dijkstra(grid, start, goal)
            else:
                path, _, runtime = astar(grid, start, goal, method='manhattan')

            total_time += runtime
            if path:
                success += 1
                total_length += len(path)

        # Compute metrics
        avg_time = total_time / runs_per_map
        avg_length = (total_length / success) if success > 0 else 0
        success_rate = (success / runs_per_map) * 100

        # Format map name (remove extension, optional: make nicer)
        map_name = os.path.splitext(map_file)[0]

        # Print result in table row
        print(f"{map_name:<10}\t{algo.capitalize():<10}\t{avg_time:.2f}\t\t{avg_length:.2f}\t\t{success_rate:.0f}")
