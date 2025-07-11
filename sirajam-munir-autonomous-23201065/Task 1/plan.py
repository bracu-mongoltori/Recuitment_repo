

import time  # For measuring how long the algorithm takes
import os    # For creating folders like 'results' if needed

# Importing functions from other files
from utils.grid import load_grid, visualize     # For loading the map and drawing the result
from utils.astar import astar                   # The A* algorithm
from utils.dijkstra import dijkstra             # The Dijkstra algorithm

# ----------- SETUP: Change These Values to Try Different Maps or Algorithms -----------

map_file = "maps/map1_sparse.csv"  # Path to the map file you want to use
start = (2, 5)                     # Starting cell coordinates (row, column)
goal = (45, 30)                    # Goal cell coordinates (row, column)
algorithm = "dijkstra"               # Choose either "astar" or "dijkstra"

# ----------- LOAD THE MAP FROM CSV -----------

# This reads your CSV file and turns it into a grid (2D list of 0s and 1s)
grid = load_grid(map_file)

# ----------- RUN THE SELECTED ALGORITHM -----------

# Start measuring time more accurately (good for small/fast algorithms)
start_time = time.perf_counter()

# Choose the pathfinding algorithm based on the user's input
if algorithm == "astar":
    path,explored = astar(grid, start, goal)  # Run A* algorithm
elif algorithm == "dijkstra":
    path,explored = dijkstra(grid, start, goal)  # Run Dijkstra algorithm
else:
    print("❌ Invalid algorithm! Please use 'astar' or 'dijkstra'.")
    path = None

# Stop the timer
end_time = time.perf_counter()

# Calculate how long it took, in milliseconds
time_taken = (end_time - start_time) * 1000

# ----------- SHOW THE RESULT IN TERMINAL -----------

# If the algorithm found a valid path
if path:
    print("✅ Path found!")
    print("📏 Path length:", len(path))
    print(f"⏱️ Time taken: {time_taken:.2f} ms")
else:
    print("❌ No path could be found from start to goal.")

# ----------- DRAW THE RESULT AND SAVE IMAGE -----------

# Make sure the results folder exists (create if not)
os.makedirs("results", exist_ok=True)

# Generate the image filename based on algorithm and map name
image_name = f"results/{algorithm}_{map_file.split('/')[-1].split('.')[0]}.png"

# Draw the map, mark the path, and save the image
visualize(grid, path=path,explored=explored, save_path=image_name)