import random
import time
import numpy as np

from utils.grid import load_grid       # Load grid from CSV
from utils.astar import astar          # A* pathfinding algorithm
from utils.dijkstra import dijkstra    # Dijkstra pathfinding algorithm

# -----------------------------------------
# Run a single algorithm and return metrics
# -----------------------------------------
def run_algorithm(algorithm, grid, start, goal):
    start_time = time.perf_counter()  # Start timer

    # Run the selected algorithm and ignore the 'explored' for now
    if algorithm == "astar":
        path, _ = astar(grid, start, goal)
    elif algorithm == "dijkstra":
        path, _ = dijkstra(grid, start, goal)
    else:
        raise ValueError("Unknown algorithm")

    end_time = time.perf_counter()  # Stop timer
    time_taken = (end_time - start_time) * 1000  # Convert to milliseconds

    # Return whether it succeeded, the path length, and how long it took
    if path:
        return True, len(path), time_taken
    else:
        return False, 0, time_taken

# ------------------------------------------------
# Get two random free cells that are far enough apart
# ------------------------------------------------
def get_random_start_goal(grid, min_distance=5):
    empty_cells = [(i, j) for i in range(len(grid)) for j in range(len(grid[0])) if grid[i][j] == 0]

    # Keep trying until you get a pair that is far apart
    while True:
        start, goal = random.sample(empty_cells, 2)
        manhattan_distance = abs(start[0] - goal[0]) + abs(start[1] - goal[1])
        if manhattan_distance >= min_distance:
            return start, goal

# -----------------------------------------
# Run benchmark for both algorithms on a map
# -----------------------------------------
def benchmark(map_path, map_name):
    grid = load_grid(map_path)  # Load the grid from CSV
    results = {"astar": [], "dijkstra": []}  # Store results

    for algo in results.keys():
        for _ in range(5):  # Run 5 trials
            start, goal = get_random_start_goal(grid)
            success, path_len, time_taken = run_algorithm(algo, grid, start, goal)
            results[algo].append({
                "success": success,
                "length": path_len,
                "time": time_taken
            })

    # ----------- Print the summary -----------
    print(f"Running Benchmark: Dijkstra vs A* ({map_name})")
    print("-------------------------------------------------\n")

    for algo in ["dijkstra", "astar"]:
        times = [r["time"] for r in results[algo]]
        lengths = [r["length"] for r in results[algo] if r["success"]]
        success_count = sum(1 for r in results[algo] if r["success"])
        success_rate = success_count / 5 * 100
        avg_time = np.mean(times)
        avg_length = np.mean(lengths) if lengths else 0

        print(f"--- {algo.upper()} ---")
        print(f"Average time       : {avg_time:.2f} ms")
        print(f"Average path length: {avg_length:.2f}")
        print(f"Success rate       : {int(success_rate)}%\n")

# -----------------------------------------
# Run Benchmark on All Three Maps
# -----------------------------------------
benchmark("maps/map1_sparse.csv", "map1_sparse")
benchmark("maps/map2_maze.csv", "map2_maze")
benchmark("maps/map3_dense.csv", "map3_dense")
