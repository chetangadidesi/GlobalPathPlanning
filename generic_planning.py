def generic_planning(planner_obj, sx, sy, gx, gy, ax=None, use_heuristic=False):
    start = planner_obj.Node(planner_obj.calc_xy_index(sx, planner_obj.min_x),
                            planner_obj.calc_xy_index(sy, planner_obj.min_y), 0.0, -1)
    goal = planner_obj.Node(planner_obj.calc_xy_index(gx, planner_obj.min_x),
                           planner_obj.calc_xy_index(gy, planner_obj.min_y), 0.0, -1)

    open_set, closed_set = dict(), dict()
    open_set[planner_obj.calc_index(start)] = start

    explored_x, explored_y = [], []
    count = 0

    while open_set:
        if use_heuristic:
            c_id = min(open_set,
                       key=lambda o: open_set[o].cost + planner_obj.calc_heuristic(goal, open_set[o]))
        else:
            c_id = min(open_set, key=lambda o: open_set[o].cost)

        current = open_set[c_id]

        if ax:
            explored_x.append(planner_obj.calc_position(current.x, planner_obj.min_x))
            explored_y.append(planner_obj.calc_position(current.y, planner_obj.min_y))
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

        for dx, dy, cost in planner_obj.motion:
            node = planner_obj.Node(current.x + dx, current.y + dy,
                                   current.cost + cost, c_id)
            n_id = planner_obj.calc_index(node)

            if not planner_obj.verify_node(node) or n_id in closed_set:
                continue

            if n_id not in open_set or open_set[n_id].cost > node.cost:
                open_set[n_id] = node

    if ax and explored_x:
        ax.plot(explored_x, explored_y, "xc", markersize=5)
        import matplotlib.pyplot as plt
        plt.pause(0.001)

    return planner_obj.calc_final_path(goal, closed_set)
