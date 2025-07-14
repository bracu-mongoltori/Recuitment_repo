import argparse
import time
import numpy as np
import matplotlib.pyplot as plt
from utils import load_grid
from dijkstra import dijkstra
from astar import astar
import os

def plot_grid(grid, path, explored, start, goal, filename):
    plt.figure(figsize=(8,8))
    plt.imshow(grid, cmap='Greys', origin='lower')
    ex_x, ex_y = zip(*explored) if explored else ([], [])
    plt.scatter(ex_y, ex_x, c='lightblue', s=10, label='Explored')
    if path:
        px, py = zip(*path)
        plt.plot(py, px, c='red', linewidth=2, label='Path')
    plt.scatter([start[1], goal[1]], [start[0], goal[0]], c=['green','orange'], s=60, marker='*', label='Start/Goal')
    plt.legend()
    plt.savefig(filename)
    plt.close()

def parse_pos(pos_str):
    x, y = map(int, pos_str.split(','))
    return (x, y)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--map', required=True)
    parser.add_argument('--start', required=True)
    parser.add_argument('--goal', required=True)
    parser.add_argument('--algo', choices=['dijkstra', 'astar'], required=True)
    parser.add_argument('--heuristic', choices=['manhattan', 'euclidean'], default='manhattan')
    args = parser.parse_args()

    grid = load_grid(args.map)
    start = parse_pos(args.start)
    goal = parse_pos(args.goal)

    t0 = time.time()
    if args.algo == 'dijkstra':
        path, explored = dijkstra(grid, start, goal)
    else:
        path, explored = astar(grid, start, goal, heuristic=args.heuristic)
    t1 = time.time()
    runtime = (t1 - t0) * 1000  # ms

    success = path is not None
    path_length = len(path) if success else 0

    print(f"Algorithm: {args.algo}")
    print(f"Runtime: {runtime:.2f} ms")
    print(f"Path length: {path_length}")
    print(f"Success: {success}")

    # Visualization
    results_dir = 'results'
    os.makedirs(results_dir, exist_ok=True)
    filename = os.path.join(results_dir, f"{args.algo}_{os.path.basename(args.map).split('.')[0]}.png")
    plot_grid(grid, path, explored, start, goal, filename)

if __name__ == '__main__':
    main()
