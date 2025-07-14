# Task 1 Report

## Implementation

- Modular Python scripts for Dijkstra and A*.
- Utilities for grid loading, neighbor finding, and path reconstruction.
- Visualization with Matplotlib.

## Benchmarking

- Each algorithm run 5 times per map.
- Metrics: runtime (ms), path length, success rate.
- See main answer for benchmark table and analysis.

## Key Findings

- **A\*** is faster than Dijkstra on all maps.
- Both algorithms find the shortest path when a solution exists.
- Failures occur only in highly obstructed or unsolvable maps.

## Visualization

- Each run produces a PNG image showing explored cells and the final path.

## Usage

- See README for instructions.
