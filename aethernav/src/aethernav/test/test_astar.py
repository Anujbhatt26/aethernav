import unittest
import numpy as np
from aethernav.path_planning.astar import AStarPlanner

class TestAStarPlanner(unittest.TestCase):
    def setUp(self):
        self.grid_size = (10, 10)
        self.planner = AStarPlanner(self.grid_size)

    def test_simple_path(self):
        """Test path finding in an empty grid"""
        start = (0, 0)
        goal = (9, 9)
        path = self.planner.find_path(start, goal)
        self.assertIsNotNone(path)
        self.assertEqual(path[0], start)
        self.assertEqual(path[-1], goal)

    def test_path_with_obstacles(self):
        """Test path finding with obstacles"""
        start = (0, 0)
        goal = (9, 9)
        obstacles = [(1, 1), (1, 2), (1, 3), (2, 1), (3, 1)]
        self.planner.set_obstacles(obstacles)
        path = self.planner.find_path(start, goal)
        self.assertIsNotNone(path)
        self.assertEqual(path[0], start)
        self.assertEqual(path[-1], goal)
        for obstacle in obstacles:
            self.assertNotIn(obstacle, path)

    def test_no_path_exists(self):
        """Test when no path exists due to obstacles"""
        start = (0, 0)
        goal = (2, 2)
        # Create a wall of obstacles
        obstacles = [(1, 0), (1, 1), (1, 2), (0, 1), (2, 1)]
        self.planner.set_obstacles(obstacles)
        path = self.planner.find_path(start, goal)
        self.assertEqual(len(path), 0)

    def test_path_optimality(self):
        """Test if the path found is optimal in terms of length"""
        start = (0, 0)
        goal = (2, 2)
        path = self.planner.find_path(start, goal)
        # Optimal path should be diagonal
        self.assertEqual(len(path), 3)  # [0,0] -> [1,1] -> [2,2]

    def test_neighbors_generation(self):
        """Test neighbor generation"""
        node = (1, 1)
        neighbors = self.planner.get_neighbors(node)
        # Should have 8 neighbors (including diagonals)
        self.assertEqual(len(neighbors), 8)
        
        # Test edge node
        edge_node = (0, 0)
        edge_neighbors = self.planner.get_neighbors(edge_node)
        # Should have 3 neighbors (including diagonal)
        self.assertEqual(len(edge_neighbors), 3)

if __name__ == '__main__':
    unittest.main()