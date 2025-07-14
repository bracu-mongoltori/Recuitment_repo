import heapq
from math import sqrt

def heuristic(a,b,type="manhattan"):
    if type == "manhattan":
        return abs(a[0]-b[0])+abs(a[1]-b[1])
    else:
        return sqrt((a[0]-b[0])**2+(a[1] - b[1]) ** 2)

def astar(grid, start, goal, h_type="manhattan"):
    rows, cols = grid.shape
    open_set = [(0,start)]
    g_score = {start:0}
    came_from = {}
    explored = set()

    while open_set:
        _, current=heapq.heappop(open_set)
        if current==goal:
            break
        if current in explored:
            continue
        explored.add(current)

        for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
            nx, ny = current[0]+dx,current[1]+dy
            neighbor = (nx, ny)

            if not (0<=nx<rows and 0<=ny<cols):
                continue
            if grid[nx,ny]==1:
                continue

            tentative_g = g_score[current] + 1

            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                g_score[neighbor] = tentative_g
                f_score = tentative_g + heuristic(neighbor, goal, h_type)
                heapq.heappush(open_set, (f_score, neighbor))
                came_from[neighbor] = current

    return reconstruct_path(came_from, start, goal), explored

def reconstruct_path(came_from, start, goal):
    if goal not in came_from:
        return []
    path = [goal]
    while path[-1] != start:
        path.append(came_from[path[-1]])
    path.reverse()
    return path
