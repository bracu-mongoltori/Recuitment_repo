#!/bin/bash

mkdir -p results

echo "Benchmarking..."

for map in map1 map2 map3
do
    echo "Running on $map"
    python3 main.py --grid grids/${map}.csv --start 0 0 --goal 9 9
done

echo "All benchmarks complete. Results saved in /results."
