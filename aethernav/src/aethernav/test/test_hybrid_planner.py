import unittest
import numpy as np
from aethernav.path_planning.hybrid_planner import HybridPlanner

class TestHybridPlanner(unittest.TestCase):
    def setUp(self):
        self.grid_size = (10, 10)
        self.planner = HybridPlanner(self.grid_size)
        self.start = (0, 0)
        self.goal = (9, 9)
        self.planner.initialize(self.start, self.goal)

    def test_initial_path_using_astar(self):
        """Test that initial path is generated using A*"""
        path = self.planner.get_current_path()
        self.assertIsNotNone(path)
        self.assertEqual(path[0], self.start)
        self.assertEqual(path[-1], self.goal)

    def test_replanning_threshold(self):
        """Test that replanning occurs when threshold is exceeded"""
        initial_path = self.planner.get_current_path()
        
        # Add obstacles affecting less than 30% of the path
        small_change = {(1, 1)}
        needs_replan = self.planner.update_obstacles(small_change)
        self.assertFalse(needs_replan)
        
        # Add obstacles affecting more than 30% of the path
        large_change = set((i, i) for i in range(5))
        needs_replan = self.planner.update_obstacles(large_change)
        self.assertTrue(needs_replan)

    def test_replanning_maintains_endpoints(self):
        """Test that replanning maintains start and goal positions"""
        # Initial path
        initial_path = self.planner.get_current_path()
        
        # Add obstacles requiring replanning
        obstacles = set((5, i) for i in range(4, 7))
        self.planner.update_obstacles(obstacles)
        new_path = self.planner.replan()
        
        self.assertEqual(new_path[0], self.start)
        self.assertEqual(new_path[-1], self.goal)
        self.assertNotEqual(initial_path, new_path)

    def test_fallback_to_astar(self):
        """Test fallback to A* when D* Lite fails"""
        # Create a situation where D* Lite might struggle
        # by adding and removing many obstacles rapidly
        for i in range(5):
            obstacles = set((i, j) for j in range(5))
            self.planner.update_obstacles(obstacles)
            path = self.planner.replan()
            self.assertIsNotNone(path)
            if len(path) > 0:  # If a path exists
                self.assertEqual(path[0], self.start)
                self.assertEqual(path[-1], self.goal)

    def test_path_validity(self):
        """Test that all paths are valid (no obstacles, within bounds)"""
        obstacles = set([(3, 3), (3, 4), (3, 5)])
        self.planner.update_obstacles(obstacles)
        path = self.planner.replan()
        
        # Check path is within bounds
        for x, y in path:
            self.assertTrue(0 <= x < self.grid_size[0])
            self.assertTrue(0 <= y < self.grid_size[1])
            
        # Check path avoids obstacles
        for pos in path:
            self.assertNotIn(pos, obstacles)

if __name__ == '__main__':
    unittest.main()