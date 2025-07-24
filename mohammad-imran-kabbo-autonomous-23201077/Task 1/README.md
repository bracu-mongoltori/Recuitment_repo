# 🧭 Task 1: Grid Map Path Planning – Dijkstra & A*

This project benchmarks two classical pathfinding algorithms (Dijkstra’s and A*) on 2D grid maps representing occupied and free cells.

---

## 🎯 Objective

- Implement and compare **Dijkstra's** and **A*** algorithms on grid maps.
- Evaluate path quality, runtime, and success rate across different maps.
- Visualize explored cells and final paths.

---

## 🗂️ Directory Structure

📦Path_Planning
┣ 📂grid_maps/ # Contains .csv map files (0=free, 1=obstacle)
┣ 📂visuals/ # Output plots (paths & explored cells)
┣ 📄algorithms.py # Contains Dijkstra and A* implementations
┣ 📄benchmark.py # Benchmarks both algorithms on all maps
┣ 📄main.py # CLI entrypoint for single runs
┣ 📄utils.py # Helper functions (loading, visualization)
┣ 📄README.md



---

## 🔧 How to Run

### ▶️ Run a single path planning experiment:
```bash
python main.py --map grid_maps/map1.csv --start 2,5 --goal 45,30 --algo astar


## 📊 Benchmark all maps:

python benchmark.py # This runs Dijkstra and A* on 3 different maps, 5 times each, and prints average results.

##📌 Algorithms Implemented
Dijkstra’s Algorithm: Uniform-cost search, no heuristic.

A*: Informed search using Manhattan or Euclidean heuristic.

## 📁 Sample Map Format CSV file, each cell is:

0 = Free
1 = Obstacle
