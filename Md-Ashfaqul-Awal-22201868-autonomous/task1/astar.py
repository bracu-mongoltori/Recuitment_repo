import heapq
from utils import get_neighbors, reconstruct_path
import math

def manhattan(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def euclidean(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])

def astar(grid, start, goal, heuristic='manhattan'):
    if heuristic == 'manhattan':
        h = manhattan
    else:
        h = euclidean
    visited = set()
    came_from = {}
    cost_so_far = {start: 0}
    queue = [(h(start, goal), 0, start)]
    explored = set()

    while queue:
        _, cost, current = heapq.heappop(queue)
        explored.add(current)
        if current == goal:
            path = reconstruct_path(came_from, start, goal)
            return path, explored
        if current in visited:
            continue
        visited.add(current)
        for neighbor in get_neighbors(current, grid):
            new_cost = cost_so_far[current] + 1
            if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_cost
                priority = new_cost + h(neighbor, goal)
                heapq.heappush(queue, (priority, new_cost, neighbor))
                came_from[neighbor] = current
    return None, explored
