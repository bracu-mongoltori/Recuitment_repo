
# 🛣️ Autonomous Navigation Path Planning
This project benchmarks and visualizes two classical graph search algorithms — **Dijkstra’s Algorithm** and **A\*** — for path planning on 2D occupancy grids.

We evaluate and compare their **performance**, **path quality**, and **success rate** on 3 different maps of varying complexity.

---

## 📁 Directory Structure

```
AUTOMATED_PATH/
├── grids/
│   ├── maps1.csv         # Sparse obstacles
│   ├── maps2.csv         # Maze-like
│   └── maps3.csv         # Dense obstacles
└── path_planning/
    ├── algorithms.py
    ├── utils.py
    ├── main.py
    ├── benchmark.py
    ├── benchmark.sh      # (optional: Linux)
    ├── results/
    │   ├── *.png         # Visualization outputs
    │   ├── benchmark_details.csv
    │   └── benchmark_summary.csv
```

---

## ⚙️ Requirements

✅ Python 3.7+  
✅ Install dependencies:
```bash
pip install numpy matplotlib pandas
```

---

## 🚀 Running the Visualizations

Run each algorithm (Dijkstra & A*) on a chosen map and visualize explored & final paths.

### Example:
```bash
cd path_planning

python main.py --grid ../grids/maps1.csv --start 0 0 --goal 9 9
```

This produces two PNG files in `results/`:
- `dijkstra_maps1.png`
- `astar_maps1.png`

Each plot shows:
- Explored cells (yellow)
- Final path (blue)
- Start (green) and Goal (red)

---

## 📊 Running the Benchmark & Comparison

Run **both algorithms 5 times on each of the 3 maps** to measure:
- Average runtime
- Average path length
- Success rate (%)

### Run benchmark:
```bash
cd path_planning

python benchmark.py
```

### Output:
✅ Detailed results:  
```
results/benchmark_details.csv
```

✅ Summary table:  
```
results/benchmark_summary.csv
```

✅ The script also prints the summary to the terminal:
```
=== Benchmark Summary ===
        algorithm  avg_runtime_ms  avg_path_len  success_rate_%
          dijkstra           12.3          18.0           100.0
     astar_manhattan            8.7          18.0           100.0
```

---

## 🧪 Notes
- The heuristic used for A\* is Manhattan distance.
- Both algorithms return optimal paths (if one exists).
- Failures occur only when no path exists between start and goal.

---

## 👨‍💻 Authors
> Autonomous Navigation Evaluation – Path Planning Benchmark
