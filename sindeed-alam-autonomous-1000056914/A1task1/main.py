from utils.grids import load_g,load_config
from algorithms.dijkstra import dijkstra
from algorithms.astar import astar
from utils.visualizer import plot_grid
from Benchmarking.benchmark import benchmark

def run_all():
    maps = ["maps/sparse.csv", "maps/maze.csv", "maps/costfield.csv"]
    results = []

    for map_path in maps:
        grid = load_g(map_path)
        start, goal = load_config()

        print(f"Running on {map_path.split('/')[-1]}")

        for algo_name, algo_func, kwargs in [
            ("Dijkstra", dijkstra, {}),
            ("A*", astar, {"h_type": "manhattan"})
        ]:
            result = benchmark(grid, start, goal, algo_func, **kwargs)
            result['algorithm'] = algo_name
            result['map'] = map_path
            results.append(result)

            path, explored = algo_func(grid, start, goal, **kwargs)
            plot_grid(grid, path, explored, start, goal, title=f"{algo_name} on {map_path}")

    print("\nSummary:")
    print("Algorithm\tMap\t\tAvg Time (ms)\tPath Length\tSuccess Rate (%)")
    for r in results:
        print(f"{r['algorithm']}\t{r['map'].split('/')[-1]}\t{r['avg_time_ms']}\t\t{r['avg_length']}\t\t{r['success_rate']}")

if __name__ == "__main__":
    run_all()
