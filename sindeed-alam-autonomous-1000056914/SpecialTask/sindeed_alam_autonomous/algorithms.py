import heapq

def get_neighbors(pos, grid):
    dirs = [(-1,0),(1,0),(0,-1),(0,1)]
    result = []
    for dx, dy in dirs:
        x, y = pos[0]+dx, pos[1]+dy
        if 0 <= x < len(grid) and 0 <= y < len(grid[0]) and grid[x][y] == 1:
            result.append((x, y))
    return result

def reconstruct_path(prev, start, goal):
    path = []
    node = goal
    while node != start:
        path.append(node)
        node = prev.get(node)
        if node is None: return []
    path.append(start)
    return list(reversed(path))

def dijkstra(grid, start, goal):
    dist = {start: 0}
    visited = set()
    prev = {}
    pq = [(0, start)]

    while pq:
        cost, node = heapq.heappop(pq)
        if node in visited: continue
        visited.add(node)
        if node == goal: break

        for nbr in get_neighbors(node, grid):
            new_cost = cost + 1
            if nbr not in dist or new_cost < dist[nbr]:
                dist[nbr] = new_cost
                prev[nbr] = node
                heapq.heappush(pq, (new_cost, nbr))

    return reconstruct_path(prev, start, goal), visited

def astar(grid, start, goal):
    def h(a, b): return abs(a[0]-b[0]) + abs(a[1]-b[1])
    g = {start: 0}
    f = {start: h(start, goal)}
    prev = {}
    visited = set()
    pq = [(f[start], start)]

    while pq:
        _, node = heapq.heappop(pq)
        if node in visited: continue
        visited.add(node)
        if node == goal: break

        for nbr in get_neighbors(node, grid):
            temp_g = g[node] + 1
            if nbr not in g or temp_g < g[nbr]:
                g[nbr] = temp_g
                f[nbr] = temp_g + h(nbr, goal)
                prev[nbr] = node
                heapq.heappush(pq, (f[nbr], nbr))

    return reconstruct_path(prev, start, goal), visited
