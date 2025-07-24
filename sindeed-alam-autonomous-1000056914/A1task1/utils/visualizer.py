import matplotlib.pyplot as plt
import numpy as np

def plot_grid(grid, path, explored, start, goal, title=""):
    grid_disp = np.copy(grid)
    for cell in explored:
        if cell != start and cell != goal:
            grid_disp[cell] = 0.5
    for cell in path:
        if cell != start and cell != goal:
            grid_disp[cell] = 0.8

    grid_disp[start] = 0.3
    grid_disp[goal] = 0.9

    plt.imshow(grid_disp, cmap='gray_r')
    plt.title(title)
    plt.axis('off')
    plt.show()
