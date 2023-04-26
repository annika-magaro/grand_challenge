import os
import sys
import math
import matplotlib.pyplot as plt
import random

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Search_based_Planning/")

import plotting, env


class LparaStar:
    def __init__(self, s_start, s_goal, e, heuristic_type, env_changes):
        self.s_start, self.s_goal = s_start, s_goal
        self.heuristic_type = heuristic_type

        self.Env = env.Env() 
        self.Plot = plotting.Plotting(self.s_start, self.s_goal)  

        self.u_set = self.Env.motions                                       # feasible input set
        self.obs = self.Env.obs                                             # position of obstacles
        self.init_e = e                                                     # initial weight
        self.e = e                                                          # weight (decremented in algorit)

        self.g = dict()                                                     # Cost to come
        self.OPEN = dict()                                                  # priority queue / OPEN set
        self.CLOSED = set()                                                 # CLOSED set
        self.INCONS = {}                                                    # INCONSISTENT set
        self.PARENT = dict()                                                # relations
        self.path = []      
        # TODO: make copy of env
        self.env_change_history = [self.Env.obs]
        self.visited = [] 
        self.count = 0
        self.env_changes = env_changes                                                 # order of visited nodes

        self.rhs, self.U = {}, {}

        for i in range(self.Env.x_range):
            for j in range(self.Env.y_range):
                self.rhs[(i, j)] = float("inf")
                self.g[(i, j)] = float("inf")

        self.rhs[self.s_start] = 0

    def init(self):
        """
        initialize each set.
        """

        # self.Plot.plot_grid("Lifelong Planning Anytime Repairing A*")
        
        self.g[self.s_start] = 0.0
        self.g[self.s_goal] = math.inf
        self.OPEN[self.s_start] = self.CalculateKey(self.s_start)
        self.PARENT[self.s_start] = self.s_start

        # self.fig = plt.figure()

    def ImprovePath(self):
        self.e = self.init_e
        self.MakeConsistent()
        self.path.append(self.extract_path())

        while self.update_e() > 1:                                               # continue condition
            self.e -= 0.4    
            self.OPEN.update(self.INCONS)
            self.OPEN = {s: self.CalculateKey(s) for s in self.OPEN}             # update f_value of OPEN set

            self.INCONS = dict()
            self.CLOSED = set()
            self.path.append(self.extract_path())

    def searching(self):
        self.init()
        # self.ComputeShortestPath() 
        self.ImprovePath()
        self.path.append(self.extract_path())
        self.plot_path(self.extract_path())
        iteration = 0

        while iteration < 3:
            print("NEW ITERATION")   
            self.ImprovePath()
            iteration += 1
            self.plot_path(self.extract_path())

            # change environment here
            if iteration < len(self.env_changes):
                self.change_env(*self.env_changes[iteration])
                self.path.append(self.extract_path())

        plt.show()
        return self.path, self.visited

    def MakeConsistent(self):
        """
        :return: a e'-suboptimal path
        """

        visited_each = []

        while True:
            s, f_small = self.calc_smallest_f()

            # print("IN MAKE CONSISTENT", self.s_goal, f_small)
            if self.CalculateKey(self.s_goal) <= f_small:
                break

            self.OPEN.pop(s)
            self.CLOSED.add(s)

            for s_n in self.get_neighbor(s):
                if s_n in self.obs:
                    continue

                new_cost = self.g[s] + self.cost(s, s_n)

                if s_n not in self.g or new_cost < self.g[s_n]:
                    self.g[s_n] = new_cost
                    self.PARENT[s_n] = s
                    visited_each.append(s_n)

                    if s_n not in self.CLOSED:
                        self.OPEN[s_n] = self.CalculateKey(s_n)
                    else:
                        self.INCONS[s_n] = 0.0

        self.visited.append(visited_each)

    def CalculateKey(self, s):

        return [min(self.g[s], self.rhs[s]) + self.e * self.h(s), # added epsilon
                min(self.g[s], self.rhs[s])]

    def calc_smallest_f(self):
        """
        :return: node with smallest f_value in OPEN set.
        """
        # print("IN CALC SMALLEST", self.OPEN)
        s_small = min(self.OPEN, key=self.OPEN.get)

        return s_small, self.OPEN[s_small]

    def get_neighbor(self, s):
        """
        find neighbors of state s that not in obstacles.
        :param s: state
        :return: neighbors
        """

        return {(s[0] + u[0], s[1] + u[1]) for u in self.u_set}

    def update_e(self):
        v = float("inf")

        if self.OPEN:
            v = min(self.g[s] + self.h(s) for s in self.OPEN)
        if self.INCONS:
            v = min(v, min(self.g[s] + self.h(s) for s in self.INCONS))

        return min(self.e, self.g[self.s_goal] / v)

    def f_value(self, x):
        """
        f = g + e * h
        f = cost-to-come + weight * cost-to-go
        :param x: current state
        :return: f_value
        """

        return self.g[x] + self.e * self.h(x) # could maybe replace with CalculateKey

    def extract_path(self):
        """
        Extract the path based on the PARENT set.
        :return: The planning path
        """

        path = [self.s_goal]
        s = self.s_goal

        while True:
            s = self.PARENT[s]
            path.append(s)

            if s == self.s_start:
                break

        return list(path)

    def h(self, s):
        """
        Calculate heuristic.
        :param s: current node (state)
        :return: heuristic function value
        """

        heuristic_type = self.heuristic_type                                # heuristic type
        goal = self.s_goal                                                  # goal node

        if heuristic_type == "manhattan":
            return abs(goal[0] - s[0]) + abs(goal[1] - s[1])
        else:
            return math.hypot(goal[0] - s[0], goal[1] - s[1])

    def cost(self, s_start, s_goal):
        """
        Calculate Cost for this motion
        :param s_start: starting node
        :param s_goal: end node
        :return:  Cost for this motion
        :note: Cost function could be more complicate!
        """

        if self.is_collision(s_start, s_goal):
            return math.inf

        return math.hypot(s_goal[0] - s_start[0], s_goal[1] - s_start[1])

    def is_collision(self, s_start, s_end):
        """
        check if the line segment (s_start, s_end) is collision.
        :param s_start: start node
        :param s_end: end node
        :return: True: is collision / False: not collision
        """

        if s_start in self.obs or s_end in self.obs:
            return True

        if s_start[0] != s_end[0] and s_start[1] != s_end[1]:
            if s_end[0] - s_start[0] == s_start[1] - s_end[1]:
                s1 = (min(s_start[0], s_end[0]), min(s_start[1], s_end[1]))
                s2 = (max(s_start[0], s_end[0]), max(s_start[1], s_end[1]))
            else:
                s1 = (min(s_start[0], s_end[0]), max(s_start[1], s_end[1]))
                s2 = (max(s_start[0], s_end[0]), min(s_start[1], s_end[1]))

            if s1 in self.obs or s2 in self.obs:
                return True

        return False
    
    def UpdateVertex(self, s):
        """
        update the status and the current cost to come of state s.
        :param s: state s
        """

        if s != self.s_start:

            # Condition: cost of parent of s changed
            # Since we do not record the children of a state, we need to enumerate its neighbors
            self.rhs[s] = min(self.g[s_n] + self.cost(s_n, s)
                              for s_n in self.get_neighbor(s))

        if s in self.OPEN:
            self.OPEN.pop(s)

        if self.g[s] != self.rhs[s]:

            # Condition: current cost to come is different to that of last time
            # state s should be added into OPEN set (set U)
            self.OPEN[s] = self.CalculateKey(s)

    
    def change_env(self, x, y):
        x, y = int(x), int(y)
        print("Change position: s =", x, ",", "y =", y)

        # self.visited = []
        self.count += 1

        if (x, y) not in self.obs:
            self.obs.add((x, y))
        else:
            self.obs.remove((x, y))
            self.UpdateVertex((x, y))

        self.env_change_history.append(self.obs)
        self.Plot.update_obs(self.obs)

        for s_n in self.get_neighbor((x, y)):
            self.UpdateVertex(s_n)

        self.MakeConsistent()
        plt.cla()
        self.Plot.plot_grid("Lifelong Planning Anytime Repairing A*")
        # self.Plot.plot_visited(self.visited)
        print(self.extract_path())
        self.plot_path(self.extract_path())
        self.fig.canvas.draw_idle()
        

    def on_press(self, event):
        x, y = event.xdata, event.ydata
        if x < 0 or x > self.x - 1 or y < 0 or y > self.y - 1:
            print("Please choose right area!")
        else:
            x, y = int(x), int(y)
            print("Change position: s =", x, ",", "y =", y)

            self.visited = []
            self.count += 1

            if (x, y) not in self.obs:
                self.obs.add((x, y))
            else:
                self.obs.remove((x, y))
                self.UpdateVertex((x, y))

            self.Plot.update_obs(self.obs)

            for s_n in self.get_neighbor((x, y)):
                self.UpdateVertex(s_n)

            self.MakeConsistent()

            plt.cla()
            self.Plot.plot_grid("Lifelong Planning A*")
            self.plot_visited(self.visited)
            self.plot_path(self.extract_path())
            self.fig.canvas.draw_idle()

    def plot_path(self, path):
        px = [x[0] for x in path]
        py = [x[1] for x in path]
        plt.plot(px, py, linewidth=2)
        plt.plot(self.s_start[0], self.s_start[1], "bs")
        plt.plot(self.s_goal[0], self.s_goal[1], "gs")

    def plot_visited(self, visited):
        color = ['gainsboro', 'lightgray', 'silver', 'darkgray',
                 'bisque', 'navajowhite', 'moccasin', 'wheat',
                 'powderblue', 'skyblue', 'lightskyblue', 'cornflowerblue']

        if self.count >= len(color) - 1:
            self.count = 0

        for x in visited:
            plt.plot(x[0], x[1], marker='s', color=color[self.count])


def main():
    s_start = (5, 5)
    s_goal = (45, 25)

    lparastar = LparaStar(s_start, s_goal, 2.5, "euclidean", [(6, 6), (8, 14), (44, 22), (34, 6), (3, 3)])
    plot = plotting.Plotting(s_start, s_goal)

    path, visited = lparastar.searching()
    # print(visited)
    plot.animation_lpara_star(path, visited, lparastar.env_change_history, "Lifelong Planning Anytime Repairing A* (LPARA*)")

if __name__ == '__main__':
    main()