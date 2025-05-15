import heapq
from collections import deque
from heuristics import get_heuristic

dirs = [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]

def a_star(start, goal, heur, GRID_SIZE, buildings):
        """A* pathfinding algorithm"""

        heuristic = get_heuristic(heur)

        open_set = []
        heapq.heappush(open_set, (0 + heuristic(start, goal), 0, start, []))
        visited = set()

        while open_set:
            est_total, cost, current, path = heapq.heappop(open_set)
            
            if current in visited:
                continue
                
            visited.add(current)
            
            if current == goal:
                return path + [current]

            # Explore neighbors (including diagonals)
            for dx, dy in dirs:
                nx, ny = current[0]+dx, current[1]+dy
                
                # Check boundaries and obstacles
                if (0 <= nx < GRID_SIZE and 0 <= ny < GRID_SIZE and 
                    (nx, ny) not in buildings):
                    
                    # Diagonal movement costs more
                    move_cost = 1.4 if dx != 0 and dy != 0 else 1.0
                    heapq.heappush(
                        open_set, 
                        (cost + move_cost + heuristic((nx, ny), goal), 
                         cost + move_cost, 
                         (nx, ny), 
                         path + [current])
                    )

        return []  # No path found

def bfs(start, goal, GRID_SIZE, buildings):
    
    queue = deque([(start, [start])])
    visited = set([start])

    while queue:
        current, path = queue.popleft()
        
        if current == goal:
            return path

        for dx, dy in dirs:
            nx, ny = current[0]+dx, current[1]+dy
            
            if (0 <= nx < GRID_SIZE and 0 <= ny < GRID_SIZE and (nx, ny) not in buildings and (nx, ny) not in visited):
                queue.append(((nx, ny), path + [(nx,ny)]))
                visited.add((nx, ny))

    return []  # No path found

def dfs(start, goal, GRID_SIZE, buildings):

    stack = [(start, [start])]
    visited = set([start])

    while stack:
        current, path = stack.pop()
        
        if current == goal:
            return path

        for dx, dy in dirs:
            nx, ny = current[0]+dx, current[1]+dy
            
            if (0 <= nx < GRID_SIZE and 0 <= ny < GRID_SIZE and (nx, ny) not in buildings and (nx, ny) not in visited):
                stack.append(((nx, ny), path + [(nx,ny)]))
                visited.add((nx, ny))

    return []  # No path found
    
def ucs(start, goal, GRID_SIZE, buildings): 
        # same as a_star implementation but withour using heuristics

        open_set = []
        heapq.heappush(open_set, (0, 0, start, []))
        visited = set()

        while open_set:
            est_total, cost, current, path = heapq.heappop(open_set)
            
            if current in visited:
                continue
                
            visited.add(current)
            
            if current == goal:
                return path + [current]

            # Explore neighbors (including diagonals)
            for dx, dy in dirs:
                nx, ny = current[0]+dx, current[1]+dy
                
                # Check boundaries and obstacles
                if (0 <= nx < GRID_SIZE and 0 <= ny < GRID_SIZE and 
                    (nx, ny) not in buildings):
                    
                    # Diagonal movement costs more
                    move_cost = 1.4 if dx != 0 and dy != 0 else 1.0
                    heapq.heappush(
                        open_set, 
                        (cost + move_cost, 
                         cost + move_cost, 
                         (nx, ny), 
                         path + [current])
                    )

        return []  # No path found


def gbfs(start, goal, heur, GRID_SIZE, buildings):

    heuristic = get_heuristic(heur)
    open_set = []
    heapq.heappush(open_set, (heuristic(start, goal), start, [start]))  
    visited = set()

    while open_set:
        h, current, path = heapq.heappop(open_set)  
        
        if current in visited:
            continue
            
        visited.add(current)
        
        if current == goal:
            return path

        for dx, dy in dirs:
            nx, ny = current[0] + dx, current[1] + dy
            
            if (0 <= nx < GRID_SIZE and 0 <= ny < GRID_SIZE and (nx, ny) not in buildings and (nx, ny) not in visited):
                
                heapq.heappush(open_set, (heuristic((nx, ny), goal), (nx, ny), path + [(nx, ny)] ))

    return []  

def bidirectional(start, goal, GRID_SIZE, buildings):

    if start == goal: 
        return [start]
    
    forward = deque([(start, [start])])
    for_vis = {start: [start]}

    backward = deque([(goal, [goal])])
    back_vis = {goal: [goal]}

    while forward and backward:
        for_cur, for_p = forward.popleft()

        for dx, dy in dirs:
            nx, ny = for_cur[0]+dx, for_cur[1]+dy
            
            if (0 <= nx < GRID_SIZE and 0 <= ny < GRID_SIZE and (nx, ny) not in buildings and (nx, ny) not in for_vis):
                
                if((nx, ny) in back_vis):
                    return for_p + back_vis[(nx, ny)][::-1]
                
                forward.append(((nx, ny), for_p + [(nx,ny)]))
                for_vis[(nx, ny)] = for_p + [(nx,ny)]


        back_cur, back_p = backward.popleft()

        for dx, dy in dirs:
            nx, ny = back_cur[0]+dx, back_cur[1]+dy
            
            if (0 <= nx < GRID_SIZE and 0 <= ny < GRID_SIZE and (nx, ny) not in buildings and (nx, ny) not in back_vis):

                if((nx, ny) in for_vis):
                    return for_vis[(nx, ny)][:-1] + back_p[::-1]

                backward.append(((nx, ny), back_p + [(nx,ny)]))
                back_vis[(nx, ny)] = back_p + [(nx,ny)]

    return []  # No path found
    
