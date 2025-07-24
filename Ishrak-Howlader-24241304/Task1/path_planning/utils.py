import numpy as np
import matplotlib.pyplot as plt

def load_grid(file_path):
    return np.loadtxt(file_path, delimiter=',', dtype=int)

def manhattan(p1, p2):
    return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])

def euclidean(p1, p2):
    return np.hypot(p1[0] - p2[0], p1[1] - p2[1])

def plot_grid(grid, explored, path, start, goal, title, filename):
    cmap = plt.cm.gray_r
    plt.figure(figsize=(6,6))
    plt.imshow(grid, cmap=cmap)

    # Explored cells: faint yellow
    explored_y, explored_x = zip(*explored) if explored else ([], [])
    plt.scatter(explored_x, explored_y, c='red', s=10, label='Explored')

    # Final path: thicker blue line
    if path:
        path_y, path_x = zip(*path)
        plt.plot(path_x, path_y, 'b-', linewidth=2, label='Final Path')

    # Start & goal
    plt.plot(start[1], start[0], 'go', label='Start')
    plt.plot(goal[1], goal[0], 'ro', label='Goal')

    plt.title(title)
    plt.legend(loc='upper right')
    plt.savefig(filename)
    plt.close()

