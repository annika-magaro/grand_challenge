import os
import sys
import math

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Search_based_Planning/")

from plotting import Plotting
from env import Env
import matplotlib.pyplot as plt

PARAMETERS = {
    "e": 2.5, 
    "connected": 4, 
    "size": 50,
    "clump-size": "medium",
    "coverage": 0.1
}
class LparaStar:
    def __init__(self, s_start, s_goal, e, heuristic_type, connected=8, size=50, coverage=0.1, clump_size='small'):
        self.s_start, self.s_goal = s_start, s_goal
        self.heuristic_type = heuristic_type

        self.e = e
        self.new_env_changes = False
        self.s_changed = set()
        self.connected, self.size, self.coverage, self.clump_size = connected, size, coverage, clump_size

        self.Plot = Plotting(self.s_start, self.s_goal, self.connected, self.size, self.coverage, self.clump_size) 
        self.Env = self.Plot.env
        self.x = self.Env.x_range
        self.y = self.Env.y_range
        self.u_set = self.Env.motions                                       # feasible input set
        self.obs = self.Env.obs   
        self.fig = plt.figure()                                                                                          

        self.colors_visited = Plotting.colors_visited()
        self.colors_path = Plotting.colors_path()
        self.g = dict()                                                     # Cost to come
        self.OPEN = dict()                                                  # priority queue / OPEN set
        self.CLOSED = set()                                                 # CLOSED set
        self.INCONS = {}                                                    # INCONSISTENT set
        self.PARENT = dict()                                                # relations
        self.path = []                                                      # planning path
        self.visited = []                                                   # order of visited nodes

    def init(self):
        """
        initialize each set.
        """

        for i in range(self.Env.x_range):
            for j in range(self.Env.y_range):
                self.g[(i, j)] = math.inf

        self.g[self.s_start] = 0.0
        self.OPEN[self.s_start] = self.f_value(self.s_start)
        self.PARENT[self.s_start] = self.s_start

    def plot_progress(self):
        k = len(self.visited) - 1
        self.Plot.plot_visited(self.visited[k], self.colors_visited[k % len(self.colors_visited)])
        self.Plot.plot_path(self.path[k], self.colors_path[k % len(self.colors_path)], True)
        plt.pause(0.5) 

    def on_press(self, event):
        x, y = event.xdata, event.ydata
        if x < 0 or x > self.x - 1 or y < 0 or y > self.y - 1:
            print("Please choose right area!")
        else:
            x, y = int(x), int(y)
            print("Change position: s =", x, ",", "y =", y)
            self.new_env_changes = True

        if (x, y) not in self.obs:
            for (i, j) in self.get_clump(x, y):
                self.obs.add((i, j))
                self.s_changed.add((i, j))
                plt.plot(i, j, "sk")
        else:
            for (i, j) in self.get_clump(x, y):
                self.obs.remove((i, j))
                self.s_changed.add((i, j))
                plt.plot(i, j, "sw")

        self.Plot.update_obs(self.obs)

    def get_clump(self, x, y):
        clump_dict = {'small': 0, 'medium': 1, 'large': 2}
        delta = clump_dict[self.clump_size]
        for i in range(x - delta, x + delta + 1):
            for j in range(y - delta, y + delta + 1):
                if i >= 0 and i < self.x and j >= 0 and j < self.y:
                    yield (i, j)


    def searching(self):
        self.init()
        self.fig.canvas.mpl_connect('button_press_event', self.on_press)
        self.Plot.plot_grid("LPARA*")
        self.ImprovePath()
        self.path.append(self.extract_path())
        self.plot_progress()

        while self.update_e() > 1:  
        # for i in range(3):                                        # continue condition
            self.e -= 0.1 
            self.OPEN.update(self.INCONS)
            self.OPEN = {s: self.f_value(s) for s in self.OPEN}             # update f_value of OPEN set

            self.INCONS = dict()
            self.CLOSED = set()
            self.ImprovePath()                                    
            self.path.append(self.extract_path())
            self.plot_progress()

        return self.path, self.visited

    def UpdateVertex(self, s):
        """
        update the status and the current cost to come of state s.
        :param s: state s
        """
        if s in self.OPEN:
            self.OPEN.pop(s)
        if s in self.obs:
            self.g[s] = math.inf
            return
        
        if s != self.s_start:

            # Condition: cost of parent of s changed
            # Since we do not record the children of a state, we need to enumerate its neighbors
            rhs = min(self.g[s_n] + self.cost(s_n, s)
                              for s_n in self.get_neighbor(s))
        else:
            rhs = 0

        if self.g[s] != rhs:

            # Condition: current cost to come is different to that of last time
            # state s should be added into OPEN set 
            self.OPEN[s] = self.f_value(s)

    def ImprovePath(self):
        """
        :return: a e'-suboptimal path
        """
        visited_each = []

        while True and len(self.OPEN) != 0:
            if self.new_env_changes:
                for s in self.s_changed:
                    self.UpdateVertex(s)
                
                    for s_n in self.get_neighbor(s):
                        self.UpdateVertex(s_n)

                self.s_changed = set()
                self.new_env_changes = False

            s, f_small = self.calc_smallest_f()

            if self.f_value(self.s_goal) <= f_small:
                break


            self.OPEN.pop(s)
            self.CLOSED.add(s)
            if s in self.obs:
                continue

            for s_n in self.get_neighbor(s):
                if s_n in self.obs:
                    continue

                new_cost = self.g[s] + self.cost(s, s_n)

                if s_n not in self.g or new_cost < self.g[s_n]:
                    self.g[s_n] = new_cost
                    self.PARENT[s_n] = s
                    visited_each.append(s_n)

                    if s_n not in self.CLOSED:
                        self.OPEN[s_n] = self.f_value(s_n)
                    else:
                        self.INCONS[s_n] = 0.0

        self.visited.append(visited_each)

    def calc_smallest_f(self):
        """
        :return: node with smallest f_value in OPEN set.
        """

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

        return self.g[x] + self.e * self.h(x)

    def CalculateKey(self, s):
        return [min(self.g[s], self.rhs(s)) + self.h(s),
                min(self.g[s], self.rhs(s))]
    
    def rhs(self, s):
        return min(self.g[s_n] + self.cost(s_n, s)
                for s_n in self.get_neighbor(s))

    def extract_path(self):
        """
        Extract the path based on the PARENT set.
        :return: The planning path
        """
        parents = self.extract_path_no_recurse()
        path = []
        current = self.s_start
        while current is not None:
            path.append(current)
            current = parents[current]
        return path
    
    def extract_path_better(self, goal, current_path, used, start):
        # go backwards in path but keep track of ties
        # when there are multiple options, try both paths
        # at the end, choose the path that ends with s_start
        g_list = {}
        for x in self.get_neighbor(goal):
            if x not in used and x[0] >= 0 and x[0] < self.x and x[1] >= 0 and x[1] < self.y:
                g_list[x] = self.g[x]
        if len(g_list) > 0 and len(current_path) < 100:
            possible_s = []
            min_val = g_list[min(g_list, key=g_list.get)]
            if min_val == math.inf:
                return [current_path]
            for s in g_list:
                if g_list[s] == min_val:
                    possible_s.append(s)
            if min_val == 0:
                return [current_path + [end] for end in possible_s]
            paths = []
            for possible in possible_s:
                for path in self.extract_path_better(possible, current_path + [possible], used | set([possible]), start):
                    paths.append(path)
                    if path[-1] == start:
                        return paths
            return paths       
        else:
            return [current_path]
        
    def extract_path_no_recurse(self):
        # go backwards in path but keep track of ties
        # when there are multiple options, try both paths
        # at the end, choose the path that ends with s_start
        parents = {self.s_goal: None}
        agenda = [self.s_goal]
        used = {self.s_goal}
        while agenda:
            current = agenda.pop(0)
            if current == self.s_start:
                break
            for x in self.get_neighbor(current):
                if x not in used and not self.is_collision(current, x) and x[0] >= 0 and x[0] < self.x and x[1] >= 0 and x[1] < self.y:
                    agenda.append(x)
                    used.add(x)
                    parents[x] = current
            agenda.sort(key = lambda x: self.g.get(x))
        return parents


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

def parse_arguments():
    if "-e" in sys.argv:
        i = sys.argv.index("-e")
        PARAMETERS["e"] = float(sys.argv[i + 1])
    if "-connected" in sys.argv:
        i = sys.argv.index("-connected")
        PARAMETERS["connected"] = int(sys.argv[i + 1])
    if "-size" in sys.argv:
        i = sys.argv.index("-size")
        PARAMETERS["size"] = int(sys.argv[i + 1])
    if "-clump-size" in sys.argv:
        i = sys.argv.index("-clump-size")
        PARAMETERS["clump-size"] = sys.argv[i + 1]
    if "-coverage" in sys.argv:
        i = sys.argv.index("-coverage")
        PARAMETERS["coverage"] = float(sys.argv[i + 1])

def main():
    s_start = (5, 5)
    s_goal = (45, 45)

    args = parse_arguments()
    lparastar = LparaStar(
        s_start, 
        s_goal, 
        PARAMETERS["e"], 
        "euclidean", 
        connected = PARAMETERS["connected"], 
        size = PARAMETERS["size"], 
        clump_size = PARAMETERS["clump-size"],
        coverage = PARAMETERS["coverage"]
    )

    path, visited = lparastar.searching()

    print("Arguments: ", sys.argv)

if __name__ == '__main__':
    main()