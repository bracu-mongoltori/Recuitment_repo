import heapq

def reconstruct_path(came_from, current):
    path = []
    while current in came_from:
        path.append(current)
        current = came_from[current]
    path.reverse()
    return path

def dijkstra(grid, start, goal):
    M, N = grid.shape
    frontier = []
    heapq.heappush(frontier, (0, start))
    came_from = {}
    cost_so_far = {start: 0}
    explored = set()

    while frontier:
        _, current = heapq.heappop(frontier)
        explored.add(current)

        if current == goal:
            break

        for d in [(-1,0),(1,0),(0,-1),(0,1)]:  # 4-connected
            ni, nj = current[0]+d[0], current[1]+d[1]
            neighbor = (ni, nj)
            if 0 <= ni < M and 0 <= nj < N and grid[ni][nj] == 0:
                new_cost = cost_so_far[current] + 1
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    heapq.heappush(frontier, (new_cost, neighbor))
                    came_from[neighbor] = current
    path = reconstruct_path(came_from, goal)
    return explored, path

def astar(grid, start, goal, heuristic):
    M, N = grid.shape
    frontier = []
    heapq.heappush(frontier, (0, start))
    came_from = {}
    cost_so_far = {start: 0}
    explored = set()

    while frontier:
        _, current = heapq.heappop(frontier)
        explored.add(current)

        if current == goal:
            break

        for d in [(-1,0),(1,0),(0,-1),(0,1)]:
            ni, nj = current[0]+d[0], current[1]+d[1]
            neighbor = (ni, nj)
            if 0 <= ni < M and 0 <= nj < N and grid[ni][nj] == 0:
                new_cost = cost_so_far[current] + 1
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost + heuristic(neighbor, goal)
                    heapq.heappush(frontier, (priority, neighbor))
                    came_from[neighbor] = current
    path = reconstruct_path(came_from, goal)
    return explored, path
