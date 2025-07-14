# Task 1: Comparative Grid-Map Path Planning

## Requirements
- Python 3.8 or higher
- `numpy`
- `matplotlib`

## How to Run
Run the following command from the project root:

<pre>python plan.py --map maps/<map_file.csv> --start <x,y> --goal <x,y> --algo <algorithm> [--heuristic <heuristic>]</pre>

### Arguments:

- `--map` : Path to the CSV map file (e.g., `maps/map1.csv`)
- `--start` : Start coordinates in the format `x,y` (e.g., `0,0`)
- `--goal` : Goal coordinates in the format `x,y` (e.g., `9,9`)
- `--algo` : Algorithm to use (`dijkstra` or `astar`)
- `--heuristic` *(optional)* : Heuristic function for A* (e.g., `manhattan`, `euclidean`)

---

This script is part of a larger project comparing classical pathfinding algorithms on occupancy grid maps. It highlights my experience in CLI-based tool development, algorithm integration, and spatial data handling.

