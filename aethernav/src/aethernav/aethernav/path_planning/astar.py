import heapq
import numpy as np
from typing import List, Tuple, Dict, Set

class AStarPlanner:
    def __init__(self, grid_size: Tuple[int, int]):
        self.grid_size = grid_size
        self.obstacles: Set[Tuple[int, int]] = set()
        self.neighbors = [(0, 1), (1, 0), (0, -1), (-1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]

    def set_obstacles(self, obstacles: List[Tuple[int, int]]):
        self.obstacles = set(obstacles)

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

    def find_path(self, start: Tuple[int, int], goal: Tuple[int, int]) -> List[Tuple[int, int]]:
        frontier = []
        heapq.heappush(frontier, (0, start))
        came_from = {start: None}
        cost_so_far = {start: 0}

        while frontier:
            current = heapq.heappop(frontier)[1]

            if current == goal:
                break

            for next_node in self.get_neighbors(current):
                new_cost = cost_so_far[current] + 1
                if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                    cost_so_far[next_node] = new_cost
                    priority = new_cost + self.heuristic(next_node, goal)
                    heapq.heappush(frontier, (priority, next_node))
                    came_from[next_node] = current

        if goal not in came_from:
            return []

        path = []
        current = goal
        while current is not None:
            path.append(current)
            current = came_from[current]
        path.reverse()
        return path