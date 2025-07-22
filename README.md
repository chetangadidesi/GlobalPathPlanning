# Global Path Planning Algorithms for Mobile Robots
This project implements a suite of classical global path planning algorithms—A*, Dijkstra’s, Breadth-First Search (BFS), and Depth-First Search (DFS)—in a modular Python framework for mobile robot navigation in a 2D grid-based environment with randomly generated obstacles and goal locations.

It showcases core algorithmic strategies for navigating known environments, forming the backbone of high-level decision-making in autonomous ground robots, warehouse AGVs, and planetary rovers.

# What This Project Demonstrates
- Strong grasp of graph-based search algorithms and their applications in global motion planning.
- Capability to design and implement modular, scalable planners suitable for robotics simulation and real-time deployment.
- Proficiency in Python, data structures, and algorithm optimization in grid-based world models.
- Visual debugging and algorithm comparison through Matplotlib-based trajectory rendering.
- Experience with autonomy pipeline components—goal selection, obstacle modeling, and path extraction.

# Key Features
Multiple Planners in One Framework
- Includes A*, Dijkstra’s, BFS, and DFS, unified under a generic node-expansion architecture for clean benchmarking and testing.

Randomized Scenario Generation
- Every simulation run spawns a new map with different obstacle distributions and goal positions to test algorithm adaptability.

Reusable Base Class for Planners
- All planners inherit from a common abstract interface, supporting cleaner architecture and easier extension to future planners like D* Lite or Hybrid A*.

2D Occupancy Grid World
- Implements a discretized world model for pathfinding and collision detection, suitable for integrating with SLAM or map servers in robotic systems.

Real-Time Visualization
- Dynamically animates planner execution and resulting paths using Matplotlib to analyze search behavior, convergence, and efficiency.

# Background: What Are These Planners?
A*
- A heuristic-guided search that balances optimality and efficiency. It combines the actual cost from the start with an admissible heuristic (e.g., Euclidean distance to goal) to prioritize promising nodes, often used in embedded systems and autonomous vehicles for rapid, near-optimal decisions.

Dijkstra’s Algorithm
- A special case of A* without a heuristic (pure cost-based). It guarantees shortest paths but explores more nodes than A*, making it slower in practice. Still widely used in constrained or fully known environments.

Breadth-First Search (BFS)
- A blind search that expands equally in all directions, ensuring shortest-path discovery in unweighted grids. While simple, it's computationally expensive and impractical for large maps.

Depth-First Search (DFS)
- A non-optimal search useful for completeness but not path quality. It dives deep along one path before backtracking and is included here for contrast and academic comparison.

# Structure

global_planners_compare/
│
├── main.py                    # Entry point to run all planners sequentially
├── astar.py                  # A* planner
├── dijkstra.py               # Dijkstra’s planner
├── bfs.py                    # Breadth-First Search
├── dfs.py                    # Depth-First Search
├── generic_planning.py       # Shared search loop and utilities
├── planner_base.py           # Abstract base class for all planners
├── grid_map_utils.py         # Map generation, obstacle setup, and goal sampling
└── utils.py                  # Helper functions (heuristics, collision check, etc.)

# Use Cases
- Benchmarking classical path planners for mobile robotics
- Integration with SLAM/local planners in autonomy stacks
- Rapid prototyping of global planners in warehouse, outdoor, or exploration scenarios
- Algorithm comparison for performance under sparse vs. dense environments
