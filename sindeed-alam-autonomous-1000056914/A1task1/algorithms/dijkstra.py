import heapq

def dijkstra(grid, start, end):
    rows, cols = grid.shape
    visited_nodes = set()
    came_from = {}
    cost = {start: 0}
    priority_queue = [(0, start)]
    explored = []

    while priority_queue:
        current_cost, current = heapq.heappop(priority_queue)

        if current in visited_nodes:
            continue

        visited_nodes.add(current)
        explored.append(current)

        if current == end:
            break

        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nx, ny = current[0] + dx, current[1] + dy
            neighbor = (nx, ny)

            if not (0 <= nx < rows and 0<=ny<cols):
                continue
            if grid[nx][ny] == 1:
                continue

            new_cost = cost[current] + 1
            if neighbor not in cost or new_cost<cost[neighbor]:
                cost[neighbor] = new_cost
                came_from[neighbor] = current
                heapq.heappush(priority_queue,(new_cost, neighbor))

    return reconstruct_path(came_from, start, end), explored

def reconstruct_path(came_from, start, goal):
    if goal not in came_from:
        return []
    path = [goal]
    while path[-1] != start:
        path.append(came_from[path[-1]])
    path.reverse()
    return path
