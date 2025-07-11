import csv                          # To read CSV files for the grid
import matplotlib.pyplot as plt    # For plotting the grid, path, and explored cells
import numpy as np                 # For handling the grid as an array

# -----------------------------
# Function to load grid from a CSV file
# -----------------------------
def load_grid(file_path):
    """
    Reads a CSV file and returns a 2D grid as a list of lists.
    Each cell is either 0 (free) or 1 (obstacle).
    """
    with open(file_path, 'r') as f:
        reader = csv.reader(f)  # Create CSV reader object
        return [[int(cell) for cell in row] for row in reader]  # Convert text to integers

# -----------------------------
# Function to visualize grid, path, and explored cells
# -----------------------------
def visualize(grid, path=None, explored=None, save_path=None):
    """
    Draws the grid and overlays the explored cells and path.

    Parameters:
    - grid: 2D list (from CSV)
    - path: list of (x, y) tuples representing the final path
    - explored: list of (x, y) tuples representing visited cells
    - save_path: if provided, the plot will be saved instead of shown
    """
    # Convert grid to NumPy array for plotting
    grid = np.array(grid)

    # Show the grid using grayscale: 0 = white (free), 1 = black (obstacle)
    cmap = plt.cm.gray_r
    plt.imshow(grid, cmap=cmap)

    # ---- Draw explored cells as light blue squares ----
    if explored:
        for (x, y) in explored:
            # Add a blue rectangle centered at (x, y)
            plt.gca().add_patch(
                plt.Rectangle((y - 0.5, x - 0.5), 1, 1, facecolor='lightblue', edgecolor='none', alpha=0.5)
            )

    # ---- Draw final path as red circles ----
    if path:
        for (x, y) in path:
            plt.plot(y, x, 'ro', markersize=3)  # Red dot at each path cell

    # ---- Optional: Add a legend ----
    if explored:
        plt.plot([], [], 's', color='lightblue', label='Explored cells')  # Legend entry
    if path:
        plt.plot([], [], 'ro', label='Final path')  # Legend entry
    plt.legend(loc='lower left')  # Position of legend box

    # ---- Save or display the plot ----
    if save_path:
        plt.axis('off')  # Hide axis ticks
        plt.savefig(save_path, bbox_inches='tight', pad_inches=0)  # Save image to file
    else:
        plt.show()  # Show plot in window

    plt.close()  # Always close the plot to free memory
