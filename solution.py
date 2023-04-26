# %matplotlib widget

import numpy as np
from utils import (State, ARAStar_Plotter,
                   GRAPH_LARGE, GRAPH_SMALL)

from ara_star import ARAStar_Planner

def initialize(planner: ARAStar_Planner):
    planner.g[planner.goal] = np.inf
    planner.g[planner.start] = 0
    planner.OPEN.clear()
    planner.CLOSED.clear()
    planner.INCONS.clear()
    planner.PARENTS.clear()  #^ new
    planner.OPEN[planner.start] = planner.f(planner.start)
    planner.PARENTS[planner.start] = planner.start  #^ new
    planner.alg_history.clear()  #^ new
    planner.paths_found.clear()  #^ new


#* student implemented
def improve_path(planner: ARAStar_Planner):
    planner.save_alg_state(planner.start)  #^ new
    while planner.f(planner.goal) > min(planner.f(s) for s in planner.OPEN):
        s = planner.get_next_state()
        planner.OPEN.pop(s)
        planner.CLOSED.add(s)

        for sprime in planner.neighbors(s):
            if sprime not in planner.g:  # unvisited
                planner.g[sprime] = np.inf

            new_cost = planner.g[s] + planner.cost(s, sprime)
            if planner.g[sprime] > new_cost:
                planner.g[sprime] = new_cost
                planner.PARENTS[sprime] = s  #^ new

                if sprime not in planner.CLOSED:
                    planner.OPEN[sprime]  = planner.f(sprime)
                else:
                    planner.INCONS.add(sprime)
        planner.save_alg_state(s)  #^ new

#* student implemented
def calc_epsilon_prime(planner: ARAStar_Planner):
    open_plus_incons = list(planner.OPEN.keys()) + list(planner.INCONS)
    if open_plus_incons:
        v = min(planner.g[s] + planner.h(s) for s in open_plus_incons)
    else:
        v = np.inf

    return min(planner.epsilon, planner.g[planner.goal] / v)


#* student implemented
def run(planner: ARAStar_Planner):
    initialize(planner)  #^ new, optional
    improve_path(planner)
    epsilon_prime = calc_epsilon_prime(planner)  #^ new, optional
    planner.publish_path()

    while epsilon_prime > 1:
        planner.epsilon = max(1, planner.epsilon - planner.stepsize)

        while planner.INCONS:
            planner.OPEN[planner.INCONS.pop()] = 0
        planner.OPEN = {s: planner.f(s) for s in planner.OPEN}

        planner.CLOSED.clear()
        improve_path(planner)
        epsilon_prime = calc_epsilon_prime(planner)  #^ new, optional
        planner.publish_path()


if __name__ == '__main__':
    # test = ARAStar_Planner(GRAPH_LARGE, State(24, 4), State(4, 44), 1.5, .2)
    # run(test)
    # print(test.all_path_costs())
    # plotter = ARAStar_Plotter(GRAPH_LARGE, State(24, 4), State(4, 44))

    # test = ARAStar_Planner(GRAPH_SMALL, State(0, 0), State(6, 5), 2.5, 1)
    # plotter = ARAStar_Plotter(GRAPH_SMALL, State(0, 0), State(6, 5))

    # run(test)
    # plotter.plot_paths_found(test.paths_found)

    EPSILON = 1.5
    START = State(24, 4)
    GOAL = State(4, 44)

    planner = ARAStar_Planner(GRAPH_LARGE, START, GOAL, EPSILON, stepsize=0.2)
    plotter = ARAStar_Plotter(GRAPH_LARGE, START, GOAL)
    run(planner)

    plotter.plot_paths_found(planner.paths_found)