# AetherNav: Adaptive Multi-Agent Path Planning System

A robust framework for real-time, cooperative path planning in dynamic environments, using a hybrid A* and D*-Lite algorithm implementation.

## Table of Contents
- [Features](#features)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Quick Start](#quick-start)
- [Configuration](#configuration)
- [Running Scenarios](#running-scenarios)
- [Docker Deployment](#docker-deployment)
- [Monitoring & Metrics](#monitoring--metrics)
- [Development Guide](#development-guide)
- [API Reference](#api-reference)
- [Performance Tuning](#performance-tuning)
- [Contributing](#contributing)
- [License](#license)

## Features

- Hybrid path planning combining A* and D*-Lite algorithms
- Real-time dynamic obstacle avoidance
- Multi-agent coordination with ROS2
- Performance monitoring and metrics collection
- Visualization tools for real-time system state
- Scalable from single agent to 50+ agents
- Docker-based deployment
- Built-in scenario generation and testing

## Prerequisites

- ROS2 Humble
- Python 3.8+
- Docker and Docker Compose (for containerized deployment)
- NVIDIA GPU (optional, for large-scale simulations)

## Installation

### Local Development Setup

1. Clone the repository:
```bash
git clone [repository-url]
cd aethernav
```

2. Create and activate a Python virtual environment:
```bash
python -m venv venv
source venv/bin/activate  # Linux/macOS
.\venv\Scripts\activate   # Windows
```

3. Install dependencies:
```bash
pip install -r requirements.txt
```

4. Build the ROS2 package:
```bash
colcon build --symlink-install
source install/setup.bash  # Linux/macOS
call install/setup.bat     # Windows
```

## Quick Start

### 1. Run a Simple Demo

```bash
ros2 launch aethernav multi_agent.launch.py num_agents:=5
```

### 2. Run a Predefined Scenario

```bash
ros2 launch aethernav scenario.launch.py scenario_file:=scenarios/scenario_1.yaml
```

### 3. Docker Deployment

```bash
docker compose up --build
```

## Configuration

### Agent Configuration

Agents can be configured through ROS2 parameters or scenario files. Key parameters include:

- `grid_size`: Size of the navigation grid (default: 100x100)
- `min_agent_distance`: Minimum distance between agents (default: 2.0)
- `replanning_threshold`: Threshold for triggering path replanning (default: 0.3)

Example parameter file (yaml):
```yaml
agent_node:
  ros__parameters:
    grid_size: [100, 100]
    min_agent_distance: 2.0
    replanning_threshold: 0.3
```

### Scenario Configuration

Scenarios are defined in YAML files with the following structure:

```yaml
config:
  grid_size: [50, 50]
  num_agents: 10
  scenario_duration: 120.0
  dynamic_obstacles: true
  obstacle_movement_speed: 0.5

obstacles:
  - [10, 10]
  - [20, 20]

agents:
  - id: 0
    start: [0, 0]
    goal: [49, 49]
```

## Running Scenarios

### 1. Generate Test Scenarios

```bash
python scripts/generate_scenarios.py
```

### 2. Run Specific Scenarios

```bash
# Run simple corridor scenario
ros2 launch aethernav scenario.launch.py scenario_file:=scenarios/scenario_1.yaml

# Run stress test
ros2 launch aethernav scenario.launch.py scenario_file:=scenarios/scenario_4.yaml
```

## Docker Deployment

### Basic Deployment

```bash
# Start all services
docker compose up --build

# Run specific configurations
docker compose up agent-1     # Single agent
docker compose up agent-10    # 10 agents
docker compose up agent-50    # 50 agents
```

### Custom Configurations

1. Modify docker-compose.yml for custom agent counts
2. Add environment variables for specific configurations
3. Mount custom scenario files

Example custom deployment:
```bash
docker compose -f docker-compose.yml -f docker-compose.custom.yml up
```

## Monitoring & Metrics

### Available Metrics

1. Path Planning Performance
   - Path length optimization
   - Replanning frequency
   - Computation time

2. Multi-Agent Coordination
   - Inter-agent distances
   - Collision avoidance success rate
   - Communication latency

3. System Resources
   - CPU usage
   - Memory consumption
   - Network bandwidth

### Accessing Metrics

1. Real-time monitoring:
   ```bash
   ros2 topic echo /aethernav/performance_metrics
   ```

2. Historical data:
   - Metrics are stored in JSON format in the `metrics/` directory
   - Each run creates a timestamped file with comprehensive metrics

### Visualization

1. Launch RViz visualization:
   ```bash
   ros2 run aethernav viz_node
   ```

2. View real-time agent positions, paths, and obstacles

## Development Guide

### Project Structure

```
aethernav/
├── src/aethernav/
│   ├── aethernav/
│   │   ├── path_planning/      # Core algorithms
│   │   ├── visualization/      # Visualization tools
│   │   └── monitoring/        # Performance monitoring
│   ├── launch/                # Launch configurations
│   └── test/                  # Test suite
└── docker/                    # Docker configuration
```

### Testing

1. Run all tests:
```bash
ros2 launch aethernav test.launch.py
```

2. Run specific test categories:
```bash
python -m pytest src/aethernav/test/test_astar.py
python -m pytest src/aethernav/test/test_dstar_lite.py
```

### Performance Tuning

1. Algorithm Parameters
   - Adjust replanning threshold based on environment dynamics
   - Tune heuristic weights for different scenarios

2. System Configuration
   - Optimize ROS2 QoS settings for different network conditions
   - Adjust update frequencies based on computational resources

3. Docker Resource Allocation
   - Configure container resource limits
   - Optimize network settings for multi-container deployment

## API Reference

### Core Classes

1. HybridPlanner
```python
class HybridPlanner:
    def __init__(self, grid_size: Tuple[int, int])
    def initialize(self, start: Tuple[int, int], goal: Tuple[int, int])
    def update_obstacles(self, obstacles: Set[Tuple[int, int]]) -> bool
    def replan(self) -> List[Tuple[int, int]]
```

2. AetherNavAgent
```python
class AetherNavAgent:
    def __init__(self, agent_id: int = 0)
    def update_path(self)
    def load_scenario(self)
```

### ROS2 Topics

1. Agent Topics
   - `/agent_{id}/path`: Published path (nav_msgs/Path)
   - `/agent_{id}/pose`: Current position (geometry_msgs/PoseStamped)
   - `/agent_{id}/goal`: Goal position (geometry_msgs/PoseStamped)

2. System Topics
   - `/map`: Environment map (nav_msgs/OccupancyGrid)
   - `/aethernav/performance_metrics`: System metrics (std_msgs/Float32MultiArray)

## Contributing

1. Fork the repository
2. Create your feature branch
3. Run tests to ensure system stability
4. Submit a pull request with comprehensive documentation

## License

Apache License 2.0