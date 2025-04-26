import numpy as np
from typing import List, Tuple, Dict
from dataclasses import dataclass
import yaml
import random

@dataclass
class ScenarioConfig:
    grid_size: Tuple[int, int]
    num_agents: int
    num_obstacles: int
    obstacle_density: float
    min_agent_distance: float
    scenario_duration: float
    dynamic_obstacles: bool
    obstacle_movement_speed: float

class ScenarioGenerator:
    def __init__(self, config: ScenarioConfig):
        self.config = config
        self.grid = np.zeros(config.grid_size)
        self.agent_positions = []
        self.agent_goals = []
        self.obstacles = set()
        
    def generate_scenario(self) -> Dict:
        """Generate a complete scenario with agents and obstacles"""
        self._place_obstacles()
        self._place_agents()
        return self._create_scenario_dict()
        
    def _place_obstacles(self):
        """Place static and dynamic obstacles"""
        self.obstacles.clear()
        total_cells = self.config.grid_size[0] * self.config.grid_size[1]
        num_obstacles = int(total_cells * self.config.obstacle_density)
        
        for _ in range(num_obstacles):
            while True:
                x = random.randint(0, self.config.grid_size[0] - 1)
                y = random.randint(0, self.config.grid_size[1] - 1)
                if (x, y) not in self.obstacles:
                    self.obstacles.add((x, y))
                    self.grid[x, y] = 1
                    break
                    
    def _place_agents(self):
        """Place agents and their goals with minimum separation"""
        self.agent_positions.clear()
        self.agent_goals.clear()
        
        for _ in range(self.config.num_agents):
            # Place agent
            while True:
                pos = self._get_random_free_position()
                if self._is_valid_position(pos, self.agent_positions):
                    self.agent_positions.append(pos)
                    break
                    
            # Place goal
            while True:
                goal = self._get_random_free_position()
                if self._is_valid_position(goal, self.agent_goals):
                    self.agent_goals.append(goal)
                    break
                    
    def _get_random_free_position(self) -> Tuple[int, int]:
        """Get a random position that's not occupied"""
        while True:
            x = random.randint(0, self.config.grid_size[0] - 1)
            y = random.randint(0, self.config.grid_size[1] - 1)
            if (x, y) not in self.obstacles and self.grid[x, y] == 0:
                return (x, y)
                
    def _is_valid_position(self, pos: Tuple[int, int], 
                          existing_positions: List[Tuple[int, int]]) -> bool:
        """Check if position maintains minimum distance from others"""
        for other_pos in existing_positions:
            dist = np.sqrt((pos[0] - other_pos[0])**2 + 
                         (pos[1] - other_pos[1])**2)
            if dist < self.config.min_agent_distance:
                return False
        return True
        
    def _create_scenario_dict(self) -> Dict:
        """Create a dictionary representation of the scenario"""
        return {
            'config': {
                'grid_size': self.config.grid_size,
                'num_agents': self.config.num_agents,
                'scenario_duration': self.config.scenario_duration,
                'dynamic_obstacles': self.config.dynamic_obstacles,
                'obstacle_movement_speed': self.config.obstacle_movement_speed
            },
            'obstacles': list(self.obstacles),
            'agents': [
                {
                    'id': i,
                    'start': self.agent_positions[i],
                    'goal': self.agent_goals[i]
                }
                for i in range(self.config.num_agents)
            ]
        }
        
    def save_scenario(self, filename: str):
        """Save scenario to YAML file"""
        scenario = self.generate_scenario()
        with open(filename, 'w') as f:
            yaml.dump(scenario, f)
            
    @staticmethod
    def load_scenario(filename: str) -> Dict:
        """Load scenario from YAML file"""
        with open(filename, 'r') as f:
            return yaml.safe_load(f)
            
def create_test_scenarios():
    """Create a set of test scenarios with different configurations"""
    scenarios = []
    
    # Simple scenario (few agents, sparse obstacles)
    simple_config = ScenarioConfig(
        grid_size=(20, 20),
        num_agents=5,
        num_obstacles=20,
        obstacle_density=0.1,
        min_agent_distance=2.0,
        scenario_duration=60.0,
        dynamic_obstacles=False,
        obstacle_movement_speed=0.0
    )
    
    # Complex scenario (many agents, dense obstacles)
    complex_config = ScenarioConfig(
        grid_size=(50, 50),
        num_agents=20,
        num_obstacles=100,
        obstacle_density=0.2,
        min_agent_distance=2.0,
        scenario_duration=120.0,
        dynamic_obstacles=True,
        obstacle_movement_speed=0.5
    )
    
    # Stress test scenario (maximum agents)
    stress_config = ScenarioConfig(
        grid_size=(100, 100),
        num_agents=50,
        num_obstacles=500,
        obstacle_density=0.3,
        min_agent_distance=1.5,
        scenario_duration=300.0,
        dynamic_obstacles=True,
        obstacle_movement_speed=1.0
    )
    
    for config, name in [(simple_config, 'simple'),
                        (complex_config, 'complex'),
                        (stress_config, 'stress')]:
        generator = ScenarioGenerator(config)
        scenario = generator.generate_scenario()
        generator.save_scenario(f'scenarios/{name}_scenario.yaml')
        scenarios.append(scenario)
        
    return scenarios