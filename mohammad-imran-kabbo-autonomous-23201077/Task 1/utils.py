# utils.py

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.lines import Line2D

def load_grid_map(path):
    """
    Loads a CSV file into a 2D NumPy array representing the grid.
    Each value is either:
      - 0 = free cell (white)
      - 1 = obstacle (black)
    """
    return np.genfromtxt(path, delimiter=',', dtype=int)


def visualize(grid,
              path,
              explored,
              title,
              save_path):
    """
    grid:     2D numpy array of 0/1
    path:     list of (r,c) cells for your main shortest path
    explored: list of (r,c) cells your algorithm visited
    """
    H, W = grid.shape
    fig, ax = plt.subplots(figsize=(6,6))
    
    # draw occupancy: free=white, occupied=black
    ax.imshow(grid, cmap='Greys', origin='upper', interpolation='none')
    
    # draw grid lines
    ax.set_xticks(np.arange(-.5, W, 1), minor=True)
    ax.set_yticks(np.arange(-.5, H, 1), minor=True)
    ax.grid(which='minor', color='lightgray', linewidth=0.5)
    
    # shade explored cells
    for r, c in explored:
        rect = plt.Rectangle((c-0.5, r-0.5), 1, 1,
                             facecolor='lightblue',
                             edgecolor='none',
                             alpha=0.6)
        ax.add_patch(rect)

    # plot shortest‐path markers (red dots)
    path_handle = None
    if path:
        xs = [c for r, c in path]
        ys = [r for r, c in path]
        path_handle, = ax.plot(xs, ys,
                               linestyle='None',
                               marker='o',
                               markersize=4,
                               markerfacecolor='red',
                               markeredgecolor='red')

    # build legend entries
    legend_handles = []
    legend_handles.append(
        mpatches.Patch(color='lightblue', alpha=0.6, label='Explored cells')
    )
    if path_handle:
        legend_handles.append(
            Line2D([0], [0],
                   marker='o',
                   color='none',
                   markerfacecolor='red',
                   markeredgecolor='red',
                   markersize=6,
                   label='Shortest path')
        )

    ax.legend(handles=legend_handles, loc='lower left', fontsize='small')
    ax.set_title(title)
    ax.set_xlim(-0.5, W-0.5)
    ax.set_ylim(H-0.5, -0.5)
    ax.axis('off')
    plt.tight_layout()
    plt.savefig(save_path, dpi=200)
    plt.close()
