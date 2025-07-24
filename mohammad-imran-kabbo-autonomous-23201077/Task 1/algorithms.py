import heapq
import math
import time
from utils import load_grid_map, visualize
from utils import load_grid_map  # adjust import path if needed

def get_neighbors(pos, grid):
    dirs = [(-1,0),(1,0),(0,-1),(0,1)]
    result = []
    r,c = pos
    for dr,dc in dirs:
        nr, nc = r+dr, c+dc
        if 0<=nr<grid.shape[0] and 0<=nc<grid.shape[1] and grid[nr,nc]==0:
            result.append((nr,nc))
    return result

# Dijkstra

def dijkstra(grid, start, goal):
    t0 = time.time()
    dist = {start:0}
    prev = {}
    visited = set()
    pq = [(0,start)]
    explored = []

    while pq:
        cost,u = heapq.heappop(pq)
        if u in visited: continue
        visited.add(u)
        explored.append(u)
        if u==goal: break
        for v in get_neighbors(u,grid):
            nc = cost+1
            if nc < dist.get(v, float('inf')):
                dist[v] = nc
                prev[v] = u
                heapq.heappush(pq, (nc,v))

    # reconstruct
    path=[]
    if goal in prev:
        node=goal
        while node!=start:
            path.append(node)
            node=prev[node]
        path.append(start)
        path.reverse()
    return path, explored, (time.time()-t0)*1000

# A*

def heuristic(a,b,method='manhattan'):
    if method=='euclidean':
        return math.hypot(a[0]-b[0], a[1]-b[1])
    return abs(a[0]-b[0]) + abs(a[1]-b[1])


def astar(grid, start, goal, method='manhattan'):
    t0 = time.time()
    g = {start:0}
    prev = {}
    visited = set()
    pq = [(heuristic(start,goal,method), start)]
    explored = []

    while pq:
        f,u = heapq.heappop(pq)
        if u in visited: continue
        visited.add(u)
        explored.append(u)
        if u==goal: break
        for v in get_neighbors(u,grid):
            ng = g[u] + 1
            if ng < g.get(v,float('inf')):
                g[v] = ng
                prev[v] = u
                heapq.heappush(pq, (ng+heuristic(v,goal,method), v))

    path=[]
    if goal in prev:
        node=goal
        while node!=start:
            path.append(node)
            node=prev[node]
        path.append(start)
        path.reverse()
    return path, explored, (time.time()-t0)*1000