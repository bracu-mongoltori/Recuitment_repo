#!/usr/bin/env python3
import argparse
import os

from utils import load_grid, manhattan, euclidean, plot_grid
from algorithms import dijkstra, astar

def run_algorithm(algo, grid, start, goal, heuristic=None, name=""):
    if algo == 'dijkstra':
        explored, path = dijkstra(grid, start, goal)
    else:
        explored, path = astar(grid, start, goal, heuristic)
    plot_grid(
        grid, explored, path, start, goal,
        title=f"{algo.upper()} on {name}",
        filename=os.path.join("results", f"{algo}_{name}.png")
    )
    print(f"{algo.upper()} on {name}: Path length: {len(path)}, Explored: {len(explored)}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--grid',   required=True, help='Path to grid CSV')
    parser.add_argument('--start',  nargs=2, type=int, required=True, help='Start cell i j')
    parser.add_argument('--goal',   nargs=2, type=int, required=True, help='Goal cell i j')
    args = parser.parse_args()

    # load data
    grid  = load_grid(args.grid)
    start = tuple(args.start)
    goal  = tuple(args.goal)

    # extract filename without extension, works for Windows & Unix paths
    name = os.path.splitext(os.path.basename(args.grid))[0]

    # ensure results directory exists
    os.makedirs("results", exist_ok=True)

    # run both planners
    run_algorithm('dijkstra', grid, start, goal, name=name)
    run_algorithm('astar',    grid, start, goal, heuristic=manhattan, name=name)
