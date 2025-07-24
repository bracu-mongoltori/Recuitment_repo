
## Code Structure

- `astar.py`: Contains A* implementation using a priority queue and Manhattan heuristic.
- `dijkstra.py`: Implements Dijkstra’s algorithm using a priority queue.
- `grid.py`: Handles reading CSV files and visualizing the grid, path, and explored cells.
- `plan.py`: Entry script that loads maps, runs the selected algorithm, times execution, and stores visualization.
- `benchmark.py`: Benchmarks and compares the performance of both algorithms on different 2D grid maps by running each algorithm five times per map with random start-goal pairs, then prints average time, path length, and success rate.

---

## Maps Used

The following 50×50 grid maps were used:
- `map1_sparse.csv` — randomly scattered obstacles (low density)
- `map2_maze.csv` — structured maze with tight corridors
- `map3_dense.csv` — densely packed obstacles

Start position: **(2, 5)**  
Goal position: **(45, 30)**

---

## How to Run

### Run a single algorithm and save visualization:
```bash
python plan.py --map maps/map1_sparse.csv --start 2,5 --goal 45,30 --algo astar
```

### Run benchmark on all maps:
```bash
python benchmark.py
```

---

## Visualization Samples

Each visualization highlights:
- **Explored cells** in light blue
- **Final path** in red

Images were saved for both A* and Dijkstra runs on each map under the `results/` directory.


