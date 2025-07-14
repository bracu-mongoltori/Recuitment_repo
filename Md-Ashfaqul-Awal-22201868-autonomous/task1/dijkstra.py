import heapq
from utils import get_neighbors, reconstruct_path

def dijkstra(grid, start, goal):
    visited = set()
    came_from = {}
    cost_so_far = {start: 0}
    queue = [(0, start)]
    explored = set()

    while queue:
        cost, current = heapq.heappop(queue)
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
                heapq.heappush(queue, (new_cost, neighbor))
                came_from[neighbor] = current
    return None, explored
