import time
from plan import load_map, dijkstra, astar, reconstruct_path

map_config = {
    "maps/map1.csv": {"start": (1, 1), "goal": (13, 18)},  # First maze section
    "maps/map2.csv": {"start": (0, 0), "goal": (14, 14)},   # Open grid
    "maps/map3.csv": {"start": (0, 0), "goal": (14,9)}, # Complex corridor
}




# Algorithms to compare
algorithms = {
    "dijkstra": dijkstra,
    "astar": astar
}

results = []

for map_file, config in map_config.items():
    grid = load_map(map_file)
    rows, cols = len(grid), len(grid[0])
    print(f"{map_file} → Grid size: {rows} rows × {cols} columns")


    start = config["start"]
    goal = config["goal"]

    print(f"\nBenchmarking on {map_file} | Start: {start} → Goal: {goal}")

    if grid[start[0]][start[1]] == 1 or grid[goal[0]][goal[1]] == 1:
        print(f"Start or goal is blocked in {map_file}. Skipping.")
        continue

    for algo_name, algo_fn in algorithms.items():
        runtimes = []
        path_lengths = []
        success_count = 0

        for _ in range(5):
            t0 = time.time()
            came_from, _ = algo_fn(grid, start, goal)
            elapsed = (time.time() - t0) * 1000

            path = reconstruct_path(came_from, start, goal)
            if path:
                success_count += 1
                path_lengths.append(len(path))
            else:
                path_lengths.append(0)

            runtimes.append(elapsed)

        avg_time = sum(runtimes) / 5
        avg_length = sum(path_lengths) / success_count if success_count else 0
        success_rate = (success_count / 5) * 100

        results.append((map_file, algo_name, avg_time, avg_length, success_rate))

# Print 
print("\nBenchmark Results:")
print(f"{'Map':<15}{'Algorithm':<10}{'Time(ms)':<12}{'Path Len':<10}{'Success %':<10}")
for row in results:
    print(f"{row[0]:<15}{row[1]:<10}{row[2]:<12.2f}{row[3]:<10.2f}{row[4]:<10.0f}")
