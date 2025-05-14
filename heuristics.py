import math

def manhattenHeuristic(a, b):
        return abs(a[0]-b[0]) + abs(a[1]-b[1])  
    
def nullHeuristic(a, b):
    return 0

def euclideanHeuristic(a, b):
    return ( (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 ) ** 0.5

def chebyshevHeuristic(a, b):
    return max(abs(a[0] - b[0]), abs(a[1] - b[1]))  

def octileHeuristic(a, b):
    dx = abs(a[0] - b[0])
    dy = abs(a[1] - b[1])
    return (dx + dy) + (math.sqrt(2) - 2) * min(dx, dy)

    
def get_heuristic(self, heur='null'):
    if heur == 'Manhatten':
        return manhattenHeuristic
    elif heur == 'Euclidean':
        return euclideanHeuristic
    elif heur == 'Chebyshev':
        return chebyshevHeuristic
    elif heur == 'Octile':
        return octileHeuristic
    else:
        return nullHeuristic