# AetherNav Configuration Guide

This guide details all configuration options and parameters available in the AetherNav system.

## Core System Parameters

### Path Planning Configuration

```yaml
path_planning:
  # A* algorithm parameters
  astar:
    diagonal_movement: true
    heuristic_weight: 1.0
    tie_breaking: true

  # D*-Lite parameters
  dstar:
    key_multiplier: 1000
    update_threshold: 0.1
    max_iterations: 1000

  # Hybrid planner parameters
  hybrid:
    replanning_threshold: 0.3  # Trigger replanning when 30% of path affected
    min_replan_distance: 2.0   # Minimum distance to trigger replanning
    smoothing_factor: 0.1      # Path smoothing coefficient
```

### Multi-Agent Settings

```yaml
agent:
  # Communication settings
  communication:
    position_update_rate: 10  # Hz
    path_broadcast_rate: 2    # Hz
    qos_profile:
      reliability: RELIABLE
      durability: VOLATILE
      history: KEEP_LAST
      depth: 10

  # Collision avoidance
  collision:
    min_separation: 2.0       # Minimum distance between agents
    safety_margin: 1.5        # Additional safety buffer
    dynamic_scaling: true     # Scale parameters with velocity

  # Performance settings
  performance:
    max_compute_time: 100     # ms per planning cycle
    max_path_points: 1000     # Maximum points in path
    cache_size: 1000          # Size of planning cache
```

## ROS2 Configuration

### Node Parameters

1. Agent Node
```yaml
agent_node:
  ros__parameters:
    grid_size: [100, 100]
    update_frequency: 10.0
    goal_tolerance: 0.5
    timeout_duration: 5.0
```

2. Visualization Node
```yaml
viz_node:
  ros__parameters:
    refresh_rate: 30.0
    path_lifetime: 1.0
    marker_scale: 0.5
    use_namespace: true
```

3. Monitor Node
```yaml
monitor_node:
  ros__parameters:
    metrics_update_rate: 1.0
    log_to_file: true
    metrics_directory: "metrics"
    buffer_size: 1000
```

## Docker Configuration

### Resource Limits

```yaml
services:
  agent:
    deploy:
      resources:
        limits:
          cpus: '1.0'
          memory: 1G
        reservations:
          cpus: '0.25'
          memory: 512M
```

### Network Configuration

```yaml
networks:
  ros-net:
    driver: overlay
    driver_opts:
      com.docker.network.driver.mtu: 1500
    ipam:
      driver: default
      config:
        - subnet: 172.28.0.0/16
```

## Scenario Configuration

### Basic Scenario Structure

```yaml
config:
  grid_size: [50, 50]
  num_agents: 10
  scenario_duration: 120.0
  dynamic_obstacles: true
  obstacle_movement_speed: 0.5
  random_seed: 42

environment:
  obstacles:
    static:
      - type: "wall"
        points: [[0, 0], [0, 10]]
      - type: "block"
        center: [25, 25]
        size: [5, 5]
    dynamic:
      - type: "moving"
        path: [[10, 10], [20, 20], [10, 20]]
        speed: 0.5

agents:
  - id: 0
    start: [0, 0]
    goal: [49, 49]
    capabilities:
      max_speed: 1.0
      turn_rate: 0.5
```

## Performance Tuning

### Memory Optimization

```yaml
memory:
  cache_size: 1000           # Number of cached paths
  max_nodes: 10000          # Maximum nodes in planning graph
  pruning_threshold: 1000   # Node count triggering pruning
```

### CPU Optimization

```yaml
cpu:
  max_threads: 4            # Maximum threads per agent
  planning_budget: 50       # ms per planning cycle
  load_balancing: true     # Enable load balancing
```

### Network Optimization

```yaml
network:
  compression: true         # Enable message compression
  batch_updates: true      # Batch position updates
  batch_size: 10           # Updates per batch
  max_latency: 100         # Maximum acceptable latency (ms)
```

## Environment Variables

```bash
# System configuration
AETHERNAV_LOG_LEVEL=INFO
AETHERNAV_METRICS_DIR=/path/to/metrics
AETHERNAV_CONFIG_FILE=/path/to/config.yaml

# ROS2 configuration
ROS_DOMAIN_ID=42
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
CYCLONEDDS_URI=/path/to/cyclonedds.xml

# Performance tuning
AETHERNAV_MAX_AGENTS=50
AETHERNAV_CACHE_SIZE=1000
AETHERNAV_DEBUG=0
```

## Logging Configuration

### Log Levels

```yaml
logging:
  default: INFO
  planning: DEBUG
  communication: INFO
  visualization: WARN
  monitoring: INFO
```

### Log Formatting

```yaml
log_format:
  timestamp: true
  node_name: true
  thread_id: true
  format: "[{timestamp}] [{level}] [{node}]: {message}"
```

### File Output

```yaml
log_output:
  file: true
  directory: "logs"
  max_size: 100M
  backup_count: 5
  compression: true
```

## Custom Extensions

### Plugin Configuration

```yaml
plugins:
  enabled:
    - custom_planner
    - terrain_analyzer
    - traffic_predictor

  custom_planner:
    algorithm: "rrt_star"
    parameters:
      max_iterations: 1000
      step_size: 0.1

  terrain_analyzer:
    update_rate: 1.0
    resolution: 0.1

  traffic_predictor:
    model_path: "models/traffic.pkl"
    prediction_horizon: 5.0
```