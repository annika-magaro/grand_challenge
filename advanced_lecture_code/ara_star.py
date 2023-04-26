from itertools import tee

import numpy as np

from advanced_lecture_code.utils import State, ARAStar_State


def _pairwise(iterable):
    "s -> (s0,s1), (s1,s2), (s2, s3), ..."
    a, b = tee(iterable)
    next(b, None)
    return zip(a, b)

class ARAStar_Planner:
    """The ARA* planner with needed utilities, along with some additional
    utilities for serializing the state of the algorithm for visualization
    and testing purposes.
    Attributes
    ----------
    graph : np.ndarray, (n x m)
        the graph being searched through, see __init__ for format requirements.
    start, goal : State
        start and goal states in the graph
    epsilon : float
        inflation factor for the heuristic function
    stepsize : float
        amount to decrease epsilon by each iteration of ARA*
    g : dict[State, float]
        dict of costs from the start State to a given State within the graph
    OPEN : dict[State: float]
        dict of locally inconsistent States to respective fvalues, used as
        a priority queue for ARA* search
    CLOSED : set[State]
        set of States which have been expanded
    INCONS : set[State]
        set of locally inconsistent States which have been previously expanded
    PARENTS : dict[State, State]
        dict mapping each expanded State to its predecessor State in the graph
    alg_history : dict[float, List[ARAStar_State]]
        dict mapping an epsilon value to a list of ARAStar_State objects,
        which recreate the state of the algoritm as it progressed through
        its search.
    paths_found : dict[float, List[State]]
        dict mapping epsilon values to the final paths returned for that
        epsilon. Path is represented as a list of States, start to goal.
    """
    def __init__(self, graph: np.ndarray, start: State,
                 goal: State, epsilon: float = 3.0,
                 stepsize: float = 0.4) -> None:
        '''
        Parameters
        ----------
            graph : np.ndarray
                NumPy array representing the graph to search through. Should
                comprise of only 0s and 1s, with 0s representing free space
                and 1s representing obstacles.
            start, goal : State
                start and goal states for the search.
            epsilon : float
                initial inflation factor for the heuristic function
            stepsize : float
                amount to decrease epsilon by each iteration of ARA*
        '''
        self.graph = graph
        self.start = start
        self.goal = goal
        self.epsilon = epsilon
        self.stepsize = stepsize

        self.g = {}
        self.OPEN = {}
        self.CLOSED = set()
        self.INCONS = set()
        self.PARENTS = {}

        self.alg_history = {}
        self.paths_found = {}

    def h(self, state: State) -> int:
        """Euclidean heuristic between goal and state."""
        return np.hypot(self.goal.x - state.x, self.goal.y - state.y)
        # return max(abs(state.x - self.goal.x), abs(state.y - self.goal.y))

    def f(self, state: State) -> float:
        """Combined inflated heuristic."""
        return self.g[state] + self.epsilon * self.h(state)

    def is_clear(self, state: State) -> bool:
        """Returns True if given state does not collide with an obstacle in the graph,
        False otherwise."""
        return self.graph[state] == 0

    def is_obstacle(self, state: State):
        """Returns True if given state collides with an obstacle in the graph,
        False otherwise."""
        return self.graph[state] != 0

    def valid_state(self, state: State) -> bool:
        """Returns True if given state is within bounds of graph and does not
        collide with an obstacle, False otherwise."""
        x, y = state
        x_bound, y_bound = self.graph.shape
        return 0 <= x < x_bound and 0 <= y < y_bound and self.is_clear(state)

    def neighbors(self, state: State) -> list[State]:
        """Returns list of neighbors of the given state which are within the bounds
        of the 8-connected graph and which do not collide with obstacles."""
        x, y = state
        n = [
            (x - 1, y - 1), (x - 1, y), (x - 1, y + 1),
            (x, y - 1), (x, y + 1),
            (x + 1, y - 1), (x + 1, y), (x + 1, y + 1),
        ]
        return [State(*s) for s in n if self.valid_state(s)]

    def cost(self, state1: State, state2: State) -> float:
        """Cost of traversal between two states. Infinite if states are not
        neighbors, else Euclidean distance."""
        if state2 not in self.neighbors(state1):
            return np.inf

        return np.hypot(state2.x - state1.x, state2.y - state1.y)

    def get_next_state(self) -> State:
        """Returns the state from OPEN with the lowest f value, which should
        be expanded next, or None if OPEN is empty."""
        if not self.OPEN:
            return None

        lowest_f = min(self.OPEN.values())
        next_states = {state for state, f in self.OPEN.items() if f == lowest_f}
        return min(next_states)  # breaks ties between states

    def extract_path(self, final_state: State = None) -> list[State]:
        """From PARENTS mapping, returns path to final_state as a list
        of States. If final_state is None, defaults to goal state."""
        if final_state is None:
            final_state = self.goal

        if final_state not in self.PARENTS:  # path not found yet
            return None

        s = final_state  # rename within loop
        path = [s]
        while True:
            s = self.PARENTS[s]
            path.append(s)

            if s == self.start:
                break

        path.reverse()
        return path

    def publish_path(self):
        """Saves current value of epsilon and current path to set of paths found."""
        self.paths_found[self.epsilon] = self.extract_path()

    def save_alg_state(self, current_state: State):
        """Saves current path and values of OPEN, CLOSED, and INCONS states
        to alg history for use in testing and plotting."""
        hist = self.alg_history.setdefault(self.epsilon, [])
        hist.append(ARAStar_State(
                self.OPEN.copy(),
                self.CLOSED.copy(),
                self.INCONS.copy(),
                self.extract_path(current_state)
            ))

    def path_cost(self, epsilon: float) -> float:
        """Calculates the total cost for the path found using the given
        epsilon value. If no path exists for this epsilon, returns 0."""
        path = self.paths_found.get(epsilon)
        if not path:
            return 0

        cost = 0
        for s1, s2 in _pairwise(path):
            cost += self.cost(s1, s2)
        return cost

    def all_path_costs(self) -> dict[float: float]:
        """Returns path costs for all found paths."""
        return {eps: self.path_cost(eps) for eps in self.paths_found}