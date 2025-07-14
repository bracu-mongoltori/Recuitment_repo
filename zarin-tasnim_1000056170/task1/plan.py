import csv
import matplotlib.pyplot as plt
import numpy as np
import heapq
import argparse
import time

def load_map(maps): #importing the map
    with open(maps, 'r') as f:
        reader =  csv.reader(f)
        grid =[[int(cell) for cell in row] for row in reader]
    return grid

def show_grid(grid): #grid visualization
    plt.imshow(np.array(grid), cmap='gray_r')
    plt.title('Grid Map')
    plt.show()

def dijkstra(grid, start, goal):
    rows, cols = len(grid) , len(grid[0])
    visited = set()
    came_from = {}
    cost_so_far = {start: 0}
    pq = [(0, start)] #priotity queue 

    while pq:
        cost, current = heapq.heappop(pq)

        if current == goal:
            break

        if current in visited:
            continue 
        visited.add(current)

        for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]:
            nx, ny = current[0] + dx, current[1] + dy
            if 0 <= nx < rows and 0 <= ny < cols and grid[nx][ny] == 0:
                next = (nx, ny)
                new_cost = cost_so_far[current] + 1
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    heapq.heappush(pq, (new_cost, next))
                    came_from[next] = current
        
    return came_from, cost_so_far

def reconstruct_path(came_from, start, goal):
    current = goal
    path = [] 
    while current != start:
        path.append(current)
        current = came_from.get(current)
        if current is None:
            return []
    path.append(start)
    path.reverse()
    return path

def plot_path(grid, path, algo_name, explored, start, goal):
    rows, cols = len(grid), len(grid[0])
    img = np.ones((rows, cols, 3))  # Start with white background (free space)

    for i in range(rows):
        for j in range(cols):
            if grid[i][j] == 1:
                img[i][j] = [0, 0, 0]  # Black for obstacles

    for x, y in explored:
        if grid[x][y] == 0:
            img[x][y] = [0.5, 0.8, 1.0]  # Light blue for explored

    for x, y in path:
        img[x][y] = [1.0, 0.2, 0.2]  # Red for path

    sx, sy = start
    img[sx][sy] = [0.0, 1.0, 0.0]  # Green for start

    gx, gy = goal
    img[gx][gy] = [0.0, 0.0, 1.0]  # Blue for goal

    plt.imshow(img)
    plt.title(f"{algo_name.upper()} Path and Explored")
    plt.axis('off')
    plt.show()


def manhattan(a,b):
    return abs(a[0]-b[0]) + abs(a[1] - b[1])

def astar(grid, start, goal):
    rows, cols = len(grid), len(grid[0])
    visited = set()
    came_from = {}
    cost_so_far = {start: 0}
    pq = [(manhattan(start, goal), start)]

    while pq:
        _, current = heapq.heappop(pq)

        if current == goal:
            break
        
        if current in visited:
            continue
        visited.add(current)

        for dx, dy in [(-1,0),(1,0), (0,-1), (0,1)]:
            nx, ny = current[0] + dx, current[1]+ dy
            if 0 <= nx < rows  and 0 <= ny < cols and grid[nx][ny] == 0:
                next = (nx, ny)
                new_cost = cost_so_far[current]+1 
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost + manhattan(next, goal)
                    heapq.heappush(pq, (priority, next))
                    came_from[next] = current
    return came_from, cost_so_far



if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--map', type=str, required=True)
    parser.add_argument('--start', type=str, required=True)
    parser.add_argument('--goal', type=str, required=True)
    parser.add_argument('--algo', type=str, required=True)
    args = parser.parse_args()

    grid = load_map(args.map)
    start = tuple(map(int, args.start.split(',')))
    goal = tuple(map(int, args.goal.split(',')))

    t0 = time.time()

    if args.algo == "dijkstra":
        came_from, cost_so_far = dijkstra(grid, start, goal)
    elif args.algo == "astar":
        came_from, cost_so_far = astar(grid, start, goal)

    else:
        raise ValueError("Invalid algorithm")

    explored = cost_so_far.keys()

    elapsed = (time.time() - t0) * 1000  # milliseconds

    path = reconstruct_path(came_from, start, goal)

    if path:
        print(f"Path found! Length = {len(path)}, Time = {elapsed:.2f} ms")
        plot_path(grid, path, args.algo, explored, start, goal)

    else:
        print("No path found.")
        show_grid(grid)
