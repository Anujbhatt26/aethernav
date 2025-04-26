import unittest
import numpy as np
from aethernav.path_planning.dstar_lite import DStarLite

class TestDStarLite(unittest.TestCase):
    def setUp(self):
        self.grid_size = (10, 10)
        self.planner = DStarLite(self.grid_size)
        self.start = (0, 0)
        self.goal = (9, 9)
        self.planner.initialize(self.start, self.goal)

    def test_initial_path(self):
        """Test initial path finding"""
        path = self.planner.get_path()
        self.assertIsNotNone(path)
        self.assertEqual(path[0], self.start)
        self.assertEqual(path[-1], self.goal)

    def test_dynamic_replanning(self):
        """Test replanning when new obstacles appear"""
        # Get initial path
        initial_path = self.planner.get_path()
        
        # Add new obstacles that block the path
        new_obstacles = set([(5, 5), (5, 6), (5, 7)])
        self.planner.update_obstacles(new_obstacles)
        
        # Get new path
        new_path = self.planner.get_path()
        
        self.assertNotEqual(initial_path, new_path)
        self.assertEqual(new_path[0], self.start)
        self.assertEqual(new_path[-1], self.goal)
        for obstacle in new_obstacles:
            self.assertNotIn(obstacle, new_path)

    def test_incremental_updates(self):
        """Test incremental updates to path"""
        # Initial path
        path1 = self.planner.get_path()
        
        # Add single obstacle
        self.planner.update_obstacles({(5, 5)})
        path2 = self.planner.get_path()
        
        # Add another obstacle
        self.planner.update_obstacles({(5, 5), (5, 6)})
        path3 = self.planner.get_path()
        
        self.assertNotEqual(path1, path2)
        self.assertNotEqual(path2, path3)

    def test_no_path_after_update(self):
        """Test when path becomes impossible after adding obstacles"""
        # Create a wall of obstacles that blocks all paths
        obstacles = set([(i, 5) for i in range(10)])
        self.planner.update_obstacles(obstacles)
        
        path = self.planner.get_path()
        self.assertEqual(len(path), 0)

    def test_rhs_values(self):
        """Test rhs values are properly updated"""
        self.assertEqual(self.planner.rhs[self.goal], 0)
        self.assertGreater(self.planner.rhs[self.start], 0)

    def test_key_calculation(self):
        """Test key calculation"""
        key = self.planner.calculate_key(self.start)
        self.assertEqual(len(key), 2)
        self.assertIsInstance(key[0], (int, float))
        self.assertIsInstance(key[1], (int, float))

if __name__ == '__main__':
    unittest.main()