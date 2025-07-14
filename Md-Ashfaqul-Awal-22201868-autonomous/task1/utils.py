import numpy as np

def load_grid(map_path):
    return np.loadtxt(map_path, delimiter=',')

def get_neighbors(pos, grid):
    neighbors = []
    directions = [(-1,0), (1,0), (0,-1), (0,1)]  # 4-connectivity
    for dx, dy in directions:
        nx, ny = pos[0] + dx, pos[1] + dy
        if 0 <= nx < grid.shape[0] and 0 <= ny < grid.shape[1]:
            if grid[nx, ny] == 0:
                neighbors.append((nx, ny))
    return neighbors

def reconstruct_path(came_from, start, goal):
    path = []
    node = goal
    while node != start:
        path.append(node)
        node = came_from[node]
    path.append(start)
    path.reverse()
    return path
