import matplotlib.pyplot as plt
import numpy as np

def plot_grid(grid, path, explored, start, goal, title="Grid"):
    grid = np.array(grid)
    free_y, free_x = np.where(grid == 1)
    plt.scatter(free_x, free_y, c='lightgray', marker='s')

    if path:
        px, py = zip(*path)
        plt.plot(py, px, 'g-', linewidth=2, label='Path')

    ex, ey = zip(*explored) if explored else ([], [])
    plt.scatter(ey, ex, c='orange', s=10, label='Explored')

    plt.scatter(start[1], start[0], c='blue', s=100, label='Start')
    plt.scatter(goal[1], goal[0], c='red', s=100, label='Goal')
    plt.gca().invert_yaxis()
    plt.title(title)
    plt.grid(True)
    plt.legend()
    plt.show()
