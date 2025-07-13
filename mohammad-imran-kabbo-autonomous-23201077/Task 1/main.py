# main.py

import os
from utils import load_grid_map, visualize
from algorithms import dijkstra, astar

def parse_cell(cell_str):
    """
    Converts a string like "3,5" into a tuple (3, 5).
    """
    try:
        r, c = map(int, cell_str.strip().split(','))
        return (r, c)
    except:
        print("❌ Invalid cell format. Use row,col (e.g. 0,0)")
        exit()

print("\n🔷 Welcome to Grid Path Planner (Dijkstra / A*)")
print("--------------------------------------------------")

map_path  = input("🗺️  Enter map path (e.g. grid_maps/map1.csv): ").strip()
start_str = input("🚩 Enter START cell (row,col) e.g. 0,0: ").strip()
goal_str  = input("🏁 Enter GOAL cell (row,col) e.g. 6,6: ").strip()
algo      = input("⚙️  Choose algorithm [dijkstra / astar]: ").strip().lower()

heuristic = "manhattan"
if algo == "astar":
    heuristic = input("📐 Choose heuristic [manhattan / euclidean]: ").strip().lower()

start = parse_cell(start_str)
goal  = parse_cell(goal_str)
grid  = load_grid_map(map_path)

if algo == 'dijkstra':
    path, explored, runtime = dijkstra(grid, start, goal)
    title = "Dijkstra"
elif algo == 'astar':
    path, explored, runtime = astar(grid, start, goal, heuristic)
    title = f"A* ({heuristic})"
else:
    print("❌ Invalid algorithm. Use 'dijkstra' or 'astar'")
    exit()

print("\n✅ Path planning completed.")
print(f"📏 Path length : {len(path)}")
print(f"⏱️  Runtime     : {runtime:.2f} ms")
print(f"🎯 Reached goal: {goal in path}")

outdir = "visuals"
os.makedirs(outdir, exist_ok=True)
fname   = f"{algo}_{os.path.splitext(os.path.basename(map_path))[0]}.png"
outpath = os.path.join(outdir, fname)

visualize(
    grid,
    path=path,
    explored=explored,
    title=title,
    save_path=outpath
)

print(f"🖼️  Saved image to: {outpath}")