from typing import List, Tuple, Set
from .astar import AStarPlanner
from .dstar_lite import DStarLite

class HybridPlanner:
    def __init__(self, grid_size: Tuple[int, int]):
        self.grid_size = grid_size
        self.astar = AStarPlanner(grid_size)
        self.dstar = DStarLite(grid_size)
        self.current_path = []
        self.replanning_threshold = 0.3  # Trigger replanning if 30% of path is affected
        
    def initialize(self, start: Tuple[int, int], goal: Tuple[int, int]):
        self.start = start
        self.goal = goal
        # Initial path planning with A*
        self.current_path = self.astar.find_path(start, goal)
        # Initialize D* Lite for potential replanning
        self.dstar.initialize(start, goal)
        
    def update_obstacles(self, new_obstacles: Set[Tuple[int, int]]) -> bool:
        """
        Updates obstacle information and determines if replanning is needed
        Returns: True if replanning is needed
        """
        # Check how many points in current path are affected
        if not self.current_path:
            return True
            
        affected_points = sum(1 for point in self.current_path if point in new_obstacles)
        replanning_needed = (affected_points / len(self.current_path)) > self.replanning_threshold
        
        self.astar.set_obstacles(new_obstacles)
        self.dstar.update_obstacles(new_obstacles)
        
        return replanning_needed
        
    def replan(self) -> List[Tuple[int, int]]:
        """
        Uses D* Lite for efficient replanning when obstacles change
        """
        self.dstar.compute_shortest_path()
        new_path = self.dstar.get_path()
        
        # Fallback to A* if D* Lite fails to find a path
        if not new_path:
            new_path = self.astar.find_path(self.start, self.goal)
            
        self.current_path = new_path
        return new_path
        
    def get_current_path(self) -> List[Tuple[int, int]]:
        return self.current_path