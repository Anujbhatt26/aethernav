import heapq
from typing import Dict, List, Set, Tuple
import numpy as np

class DStarLite:
    def __init__(self, grid_size: Tuple[int, int]):
        self.grid_size = grid_size
        self.obstacles: Set[Tuple[int, int]] = set()
        self.neighbors = [(0, 1), (1, 0), (0, -1), (-1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]
        
        # D* Lite specific attributes
        self.k_m = 0
        self.rhs: Dict[Tuple[int, int], float] = {}
        self.g: Dict[Tuple[int, int], float] = {}
        self.queue: List[Tuple[float, float, Tuple[int, int]]] = []
        
    def initialize(self, start: Tuple[int, int], goal: Tuple[int, int]):
        self.start = start
        self.goal = goal
        self.rhs.clear()
        self.g.clear()
        self.queue.clear()
        self.k_m = 0
        
        for i in range(self.grid_size[0]):
            for j in range(self.grid_size[1]):
                self.rhs[(i, j)] = float('inf')
                self.g[(i, j)] = float('inf')
        
        self.rhs[goal] = 0
        self.queue = [(self.calculate_key(goal), goal)]
        heapq.heapify(self.queue)

    def calculate_key(self, s: Tuple[int, int]) -> Tuple[float, float]:
        h = self.heuristic(self.start, s)
        return (min(self.g[s], self.rhs[s]) + h + self.k_m, 
                min(self.g[s], self.rhs[s]))

    def heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

    def get_neighbors(self, node: Tuple[int, int]) -> List[Tuple[int, int]]:
        neighbors = []
        for dx, dy in self.neighbors:
            new_node = (node[0] + dx, node[1] + dy)
            if (0 <= new_node[0] < self.grid_size[0] and 
                0 <= new_node[1] < self.grid_size[1] and 
                new_node not in self.obstacles):
                neighbors.append(new_node)
        return neighbors

    def cost(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        if b in self.obstacles:
            return float('inf')
        return self.heuristic(a, b)

    def update_vertex(self, u: Tuple[int, int]):
        if u != self.goal:
            self.rhs[u] = min(self.cost(u, s) + self.g[s] 
                            for s in self.get_neighbors(u))
        
        if u in self.queue:
            self.queue.remove(u)
            heapq.heapify(self.queue)
            
        if self.g[u] != self.rhs[u]:
            heapq.heappush(self.queue, (self.calculate_key(u), u))

    def compute_shortest_path(self):
        while (self.queue and 
               (self.calculate_key(self.queue[0][1]) < self.calculate_key(self.start) or 
                self.rhs[self.start] != self.g[self.start])):
            
            k_old = self.calculate_key(self.queue[0][1])
            u = heapq.heappop(self.queue)[1]
            
            if k_old < self.calculate_key(u):
                heapq.heappush(self.queue, (self.calculate_key(u), u))
            elif self.g[u] > self.rhs[u]:
                self.g[u] = self.rhs[u]
                for s in self.get_neighbors(u):
                    self.update_vertex(s)
            else:
                self.g[u] = float('inf')
                for s in self.get_neighbors(u) + [u]:
                    self.update_vertex(s)

    def update_obstacles(self, new_obstacles: Set[Tuple[int, int]]):
        changed_edges = self.obstacles.symmetric_difference(new_obstacles)
        self.obstacles = new_obstacles
        self.k_m += self.heuristic(self.start, self.goal)
        
        for u in changed_edges:
            self.update_vertex(u)
            for s in self.get_neighbors(u):
                self.update_vertex(s)

    def get_path(self) -> List[Tuple[int, int]]:
        if self.g[self.start] == float('inf'):
            return []
            
        path = [self.start]
        current = self.start
        
        while current != self.goal:
            neighbors = self.get_neighbors(current)
            if not neighbors:
                return []
                
            current = min(neighbors, 
                         key=lambda s: self.cost(current, s) + self.g[s])
            path.append(current)
            
            if len(path) > self.grid_size[0] * self.grid_size[1]:
                return []  # Prevent infinite loops
                
        return path