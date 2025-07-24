import numpy as np
import time
import pandas as pd
import os


from algorithms.dijkstra import dijkstra
from algorithms.astar import astar, heuristic as manhattan_distance
from utils.visualize import plot_grid
from utils.benchmark import benchmark_run

def calculate_path_length(path):
  
    if not path or len(path) < 2:
        return 0
    return len(path) - 1

def load_grid_from_file(filepath):
 
    try:
        grid = np.loadtxt(filepath, delimiter=',', dtype=int)
        return grid
    except Exception as e:
        print(f"Error loading grid from {filepath}: {e}")
        return None

def main():

    os.makedirs("maps", exist_ok=True)
    os.makedirs("algorithms", exist_ok=True)
    os.makedirs("utils", exist_ok=True)



    grid_configs = [
        {"name": "Map 1 (Example)", "file": "maps/map1.csv", "start": (0, 0), "goal": (19, 19)},
        {"name": "Map 2 (Example)", "file": "maps/map2.csv", "start": (0, 0), "goal": (19, 19)},
        {"name": "Map 3 (Example)", "file": "maps/map3.csv", "start": (0, 0), "goal": (19, 19)}, 
    ]

    
    all_benchmark_results = []


    for config in grid_configs:
        grid_file = config["file"]
        start_coords = config["start"]
        goal_coords = config["goal"]

        grid = load_grid_from_file(grid_file)
        if grid is None:
            print(f"Skipping {config['name']} due to loading error.")
            continue

    
        rows, cols = grid.shape
        if not (0 <= start_coords[0] < rows and 0 <= start_coords[1] < cols and grid[start_coords] == 0):
            print(f"Error: Start coordinates {start_coords} are invalid or an obstacle on {config['name']}.")
            continue
        if not (0 <= goal_coords[0] < rows and 0 <= goal_coords[1] < cols and grid[goal_coords] == 0):
            print(f"Error: Goal coordinates {goal_coords} are invalid or an obstacle on {config['name']}.")
            continue

        print(f"\n--- Running Benchmarks for {config['name']} ---")


        print(f"Running Dijkstra on {config['name']}...")
        dijkstra_metrics = benchmark_run(dijkstra, grid, start_coords, goal_coords)
    
        dijkstra_metrics_summary = {
            "Algorithm": "Dijkstra",
            "Map": config['name'],
            "Avg Runtime (ms)": f"{dijkstra_metrics['avg_time']:.2f}",
            "Avg Path Length (cells)": f"{dijkstra_metrics['avg_length']:.2f}",
            "Success Rate (%)": f"{dijkstra_metrics['success_rate']:.2f}"
        }
        all_benchmark_results.append(dijkstra_metrics_summary)

        
        plot_grid(grid, start_coords, goal_coords, dijkstra_metrics['path'], dijkstra_metrics['explored'], title=f"Dijkstra - {config['name']}")

        
        print(f"Running A* (Manhattan) on {config['name']}...")
        
        astar_metrics = benchmark_run(lambda g, s, go: astar(g, s, go), grid, start_coords, goal_coords)
        
        astar_metrics_summary = {
            "Algorithm": "A* (Manhattan)",
            "Map": config['name'],
            "Avg Runtime (ms)": f"{astar_metrics['avg_time']:.2f}",
            "Avg Path Length (cells)": f"{astar_metrics['avg_length']:.2f}",
            "Success Rate (%)": f"{astar_metrics['success_rate']:.2f}"
        }
        all_benchmark_results.append(astar_metrics_summary)

        plot_grid(grid, start_coords, goal_coords, astar_metrics['path'], astar_metrics['explored'], title=f"A* - {config['name']}")

    
    print(f"\n--- Attempting to generate summary. Total results collected: {len(all_benchmark_results)} ---")
    if all_benchmark_results:
        results_df = pd.DataFrame(all_benchmark_results)
        print("\n--- Benchmarking Summary (Console Output) ---")
        print(results_df.to_markdown(index=False)) 

    
        results_df.to_csv("benchmark_summary.csv", index=False)
        print("\nBenchmark summary saved to benchmark_summary.csv")

    else:
        print("\nNo benchmark results to display. Please check for errors in map loading or coordinate validation.")

if __name__ == "__main__":
    main()
