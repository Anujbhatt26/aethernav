#!/usr/bin/env python3

from aethernav.simulation.scenario_generator import ScenarioConfig, ScenarioGenerator

def main():
    # Create and save all test scenarios
    scenarios = [
        # Simple corridor scenario
        ScenarioConfig(
            grid_size=(30, 10),
            num_agents=3,
            num_obstacles=15,
            obstacle_density=0.15,
            min_agent_distance=2.0,
            scenario_duration=60.0,
            dynamic_obstacles=False,
            obstacle_movement_speed=0.0
        ),
        # Room with doorways scenario
        ScenarioConfig(
            grid_size=(40, 40),
            num_agents=8,
            num_obstacles=80,
            obstacle_density=0.2,
            min_agent_distance=2.0,
            scenario_duration=120.0,
            dynamic_obstacles=True,
            obstacle_movement_speed=0.3
        ),
        # Large open space scenario
        ScenarioConfig(
            grid_size=(80, 80),
            num_agents=15,
            num_obstacles=200,
            obstacle_density=0.1,
            min_agent_distance=3.0,
            scenario_duration=180.0,
            dynamic_obstacles=True,
            obstacle_movement_speed=0.5
        ),
        # High density stress test
        ScenarioConfig(
            grid_size=(100, 100),
            num_agents=50,
            num_obstacles=1000,
            obstacle_density=0.3,
            min_agent_distance=1.5,
            scenario_duration=300.0,
            dynamic_obstacles=True,
            obstacle_movement_speed=1.0
        )
    ]

    for i, config in enumerate(scenarios):
        generator = ScenarioGenerator(config)
        generator.save_scenario(f'scenarios/scenario_{i+1}.yaml')
        print(f'Generated scenario_{i+1}.yaml')

if __name__ == '__main__':
    main()