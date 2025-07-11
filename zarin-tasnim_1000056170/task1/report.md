Name: Zarin Tasnim
ID: 1000056170
Repo Folder: zarin-tasnim_1000056170/Task1


Implementation Overview
This project implements and benchmarks two classic path-planning algorithms:

Dijkstra’s Algorithm (uninformed, exhaustive search)

A* Algorithm (heuristic-guided search using Manhattan distance)

The algorithms were run on 3 grid maps of size 15×20, using 0 as free cells and 1 as obstacles. Each algorithm searches for a path from a given start to goal cell. The core logic is implemented in a modular plan.py script.

Benchmark SetUp :

1. each alogirthm was run 5 times per map to computre average time, success, and path length

2. Grid sizes 15 x 20 

3. Inputs were validated to avoid blocked or invalid start/goal positions.

| Map File   | Start Position | Goal Position |
| ---------- | -------------- | ------------- |
| `map1.csv` | (1, 1)         | (13, 18)      |
| `map2.csv` | (0, 0)         | (14, 14)      |
| `map3.csv` | (0, 0)         | (14, 9)       |


Benchmark Results:
| Map      | Algorithm | Time (ms) | Path Length | Success Rate (%) |
| -------- | --------- | --------- | ----------- | ---------------- |
| map1.csv | Dijkstra  | 0.20      | 30          | 100              |
| map1.csv | A\*       | 0.21      | 30          | 100              |
| map2.csv | Dijkstra  | 0.51      | 29          | 100              |
| map2.csv | A\*       | 0.38      | 29          | 100              |
| map3.csv | Dijkstra  | 0.31      | 24          | 100              |
| map3.csv | A\*       | 0.43      | 24          | 100              |
  

Which algorithm is fastest on average? 
= Overall, A* is faster on average due to its heuristic-driven search, especially in map2.csv. However, in map3.csv, Dijkstra performed better, likely due to fewer heuristic calculations.


Which consistently finds the shortest path? 
= Both algorithms consistently find the same shortest path, as A* with an admissible heuristic (Manhattan) behaves optimally like Dijkstra.

Do any algorithms ever fail?
= No. Both algorithms succeeded 100% of the time in all valid map configurations. Failures can only occur if the start or goal cell is blocked (i.e., value 1), or if no path exists due to surrounding obstacles.

 Visualization :

Explored cells in light blue
Final path in a red
Obstacles in black
Start Green
Goal Blue   
free space white
