import pygame
import numpy as np
import heapq
import cv2

# ---------------- CONFIG ----------------
GRID_SIZE = 30  # Adjust based on image resolution
MAP_IMAGE_PATH = 'map.png'  # Replace this with your image file
WINDOW_SIZE = 800
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)

# ---------------------------------------

def load_map(img_path):
    img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
    img = cv2.resize(img, (GRID_SIZE, GRID_SIZE))
    _, binary = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY)
    grid = (binary == 255).astype(int)  # 1 for white (walkable), 0 for black
    return grid

def heuristic(a, b):
    return abs(a[0]-b[0]) + abs(a[1]-b[1])

def a_star(grid, start, goal):
    neighbors = [(0,1),(1,0),(-1,0),(0,-1)]
    close_set = set()
    came_from = {}
    gscore = {start:0}
    fscore = {start:heuristic(start, goal)}
    open_heap = []

    heapq.heappush(open_heap, (fscore[start], start))
    
    while open_heap:
        _, current = heapq.heappop(open_heap)

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path
        
        close_set.add(current)
        for dx, dy in neighbors:
            neighbor = (current[0]+dx, current[1]+dy)
            tentative_g_score = gscore[current] + 1
            if 0 <= neighbor[0] < grid.shape[0]:
                if 0 <= neighbor[1] < grid.shape[1]:
                    if grid[neighbor[0]][neighbor[1]] == 0:
                        continue
                else:
                    continue
            else:
                continue
            
            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue
            
            if tentative_g_score < gscore.get(neighbor, float('inf')) or neighbor not in [i[1] for i in open_heap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(open_heap, (fscore[neighbor], neighbor))
    return False

def draw_grid(screen, grid, path):
    rows, cols = grid.shape
    cell_width = WINDOW_SIZE // cols
    cell_height = WINDOW_SIZE // rows

    for x in range(rows):
        for y in range(cols):
            color = WHITE if grid[x][y] else BLACK
            pygame.draw.rect(screen, color, (y*cell_width, x*cell_height, cell_width, cell_height))

    for node in path:
        pygame.draw.rect(screen, RED, (node[1]*cell_width, node[0]*cell_height, cell_width, cell_height))

def main():
    pygame.init()
    screen = pygame.display.set_mode((WINDOW_SIZE, WINDOW_SIZE))
    pygame.display.set_caption("Autonomous Pathfinder")

    grid = load_map(MAP_IMAGE_PATH)
    start = (0, 0)
    goal = (GRID_SIZE-1, GRID_SIZE-1)

    path = a_star(grid, start, goal)

    running = True
    while running:
        screen.fill(BLACK)
        draw_grid(screen, grid, path)
        pygame.display.flip()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

    pygame.quit()

if __name__ == '__main__':
    main()
