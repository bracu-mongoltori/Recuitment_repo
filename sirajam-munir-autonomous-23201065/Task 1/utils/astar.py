
import heapq  # For priority queue operations

# Heuristic function using Manhattan Distance
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])  # |x1 - x2| + |y1 - y2|

# A* Algorithm function
def astar(grid, start, goal):
    rows, cols = len(grid), len(grid[0])  # Get number of rows and columns

    open_set = []  # Priority queue (min-heap) for unexplored nodes
    heapq.heappush(open_set, (0, start))  # Push the start node with f = 0

    came_from = {}  # Dictionary to remember the path (backtracking)
    g_score = {start: 0}  # Cost from start to each node; start is 0
    f_score = {start: heuristic(start, goal)}  # Estimated total cost (f = g + h)
    
    explored = []  # List to keep track of explored nodes


    while open_set:  # Keep searching while there are nodes to explore
        current = heapq.heappop(open_set)[1]  # Get node with lowest f-score
        
        if current not in explored:  # If this node hasn't been explored yet
            explored.append(current)  # Add it to the explored list

        if current == goal:  # If goal is reached, reconstruct the path
            path = []  # List to store the path
            while current in came_from:  # Go backwards from goal to start
                path.append(current)
                current = came_from[current]
            path.append(start)  # Add the starting node
            return path[::-1],explored  # Return reversed path from start to goal

        # Check all 4 possible directions (up, down, left, right)
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            neighbor = (current[0] + dx, current[1] + dy)  # Calculate neighbor position

            # Check if neighbor is inside the grid boundaries
            if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols:
                if grid[neighbor[0]][neighbor[1]] == 1:
                    continue  # Skip if it's an obstacle (1)

                tentative_g_score = g_score[current] + 1  # Cost to reach neighbor

                # If this path to neighbor is better (shorter) than any previous one
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current  # Record the path
                    g_score[neighbor] = tentative_g_score  # Update cost to reach neighbor
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)  # f = g + h
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))  # Add neighbor to open list

    return None,explored  # If goal is unreachable