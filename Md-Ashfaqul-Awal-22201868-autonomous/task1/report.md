# Task 1: Pathfinding Algorithm Report

## Overview

This project demonstrates the implementation and benchmarking of two classical pathfinding algorithms—**Dijkstra’s Algorithm** and **A\***—on a 2D occupancy grid. The focus is on algorithmic correctness, performance comparison, and visual clarity.

## Implementation

- Developed modular Python scripts for both Dijkstra and A* algorithms.
- Built utility functions for:
  - Grid parsing and loading
  - Neighbor identification
  - Path reconstruction
- Used Matplotlib to visualize search progress and final paths.

## Benchmarking Methodology

- Each algorithm was executed **5 times per map** to ensure reliable metrics.
- Evaluated on the following criteria:
  - **Runtime** (in milliseconds)
  - **Path length**
  - **Success rate**
- Detailed benchmark results and analysis are provided in the main report.

## Key Observations

- **A\*** consistently outperformed Dijkstra in execution time across all maps.
- Both algorithms identified the optimal path when one existed.
- Failures occurred only in maps with extremely high obstructions or no solvable route.

## Visual Output

- Each run generates a `.png` file that:
  - Highlights all explored nodes
  - Displays the final computed path (if successful)

## How to Use

- See the main `README.md` file for environment setup and execution instructions.

---

