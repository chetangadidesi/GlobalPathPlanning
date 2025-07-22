import matplotlib.pyplot as plt
from astar import AStarPlanner
from dijkstra import DijkstraPlanner
from bfs import BFSPlanner
from dfs import DFSPlanner

def main():
    # Start and goal positions
    sx, sy = 0.0, 0.0
    gx, gy = 50.0, 50.0

    grid_size = 2.0
    robot_radius = 1.0

    ox, oy = [], []
    for i in range(-10, 60):
        ox.append(float(i))
        oy.append(-10.0)
    for i in range(-10, 60):
        ox.append(60.0)
        oy.append(float(i))
    for i in range(-10, 61):
        ox.append(float(i))
        oy.append(60.0)
    for i in range(-10, 61):
        ox.append(-10.0)
        oy.append(float(i))
    for i in range(-10, 40):
        ox.append(20.0)
        oy.append(float(i))
    for i in range(0, 40):
        ox.append(40.0)
        oy.append(60.0 - i)

    fig, axs = plt.subplots(2, 2, figsize=(12, 12))
    axs = axs.flatten()

    planners = [
        ("A*", AStarPlanner(ox, oy, grid_size, robot_radius)),
        ("Dijkstra", DijkstraPlanner(ox, oy, grid_size, robot_radius)),
        ("BFS", BFSPlanner(ox, oy, grid_size, robot_radius)),
        ("DFS", DFSPlanner(ox, oy, grid_size, robot_radius)),
    ]

    for ax, (name, planner) in zip(axs, planners):
        ax.set_title(name)
        ax.plot(ox, oy, ".k", markersize=4)
        ax.plot(sx, sy, "og")
        ax.plot(gx, gy, "xb")
        ax.grid(True)
        ax.axis("equal")
        ax.set_xlim(-15, 65)
        ax.set_ylim(-15, 65)

    show_animation = True
    for ax, (name, planner) in zip(axs, planners):
        print(f"Running {name} planner...")
        rx, ry = planner.planning(sx, sy, gx, gy, ax, show_animation)
        if rx and ry:
            ax.plot(rx, ry, "-r", linewidth=2)
        else:
            ax.text(0, 0, "No path found", color="red")

    plt.show()

if __name__ == '__main__':
    main()
