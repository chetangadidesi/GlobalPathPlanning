import math
from planner_base import PlannerBase

class AStarPlanner(PlannerBase):

    def calc_heuristic(self, n1, n2):
        w = 1.0
        return w * math.hypot(n1.x - n2.x, n1.y - n2.y)

    def planning(self, sx, sy, gx, gy, ax=None, show_animation=True):
        start = self.Node(self.calc_xy_index(sx, self.min_x),
                          self.calc_xy_index(sy, self.min_y), 0.0, -1)
        goal = self.Node(self.calc_xy_index(gx, self.min_x),
                         self.calc_xy_index(gy, self.min_y), 0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[self.calc_index(start)] = start

        explored_x, explored_y = [], []
        count = 0

        while open_set:
            c_id = min(open_set,
                       key=lambda o: open_set[o].cost + self.calc_heuristic(goal, open_set[o]))
            current = open_set[c_id]

            if ax and show_animation:
                explored_x.append(self.calc_position(current.x, self.min_x))
                explored_y.append(self.calc_position(current.y, self.min_y))
                count += 1
                if count % 20 == 0:
                    ax.plot(explored_x, explored_y, "xc", markersize=5)
                    import matplotlib.pyplot as plt
                    plt.pause(0.001)
                    explored_x, explored_y = [], []

            if current.x == goal.x and current.y == goal.y:
                goal.parent_index = current.parent_index
                goal.cost = current.cost
                break

            del open_set[c_id]
            closed_set[c_id] = current

            for dx, dy, cost in self.motion:
                node = self.Node(current.x + dx, current.y + dy,
                                 current.cost + cost, c_id)
                n_id = self.calc_index(node)

                if not self.verify_node(node) or n_id in closed_set:
                    continue

                if n_id not in open_set or open_set[n_id].cost > node.cost:
                    open_set[n_id] = node

        if ax and show_animation and explored_x:
            ax.plot(explored_x, explored_y, "xc", markersize=5)
            import matplotlib.pyplot as plt
            plt.pause(0.001)

        return self.calc_final_path(goal, closed_set)

    def calc_final_path(self, goal, closed_set):
        rx, ry = [self.calc_position(goal.x, self.min_x)], [self.calc_position(goal.y, self.min_y)]
        parent = goal.parent_index
        while parent != -1:
            n = closed_set[parent]
            rx.append(self.calc_position(n.x, self.min_x))
            ry.append(self.calc_position(n.y, self.min_y))
            parent = n.parent_index
        return rx[::-1], ry[::-1]
