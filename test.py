import numpy as np

from IPython.display import display_html
from nose.tools import assert_equal, assert_almost_equal

from utils import State, GRAPH_LARGE, GRAPH_SMALL, in_notebook
from ara_star import ARAStar_Planner


def test_ok():
    """If execution gets to this point, print out a happy message."""
    if in_notebook():
        display_html("""<div class="alert alert-success">
        <strong>Tests passed!!</strong>
        </div>""", raw=True)
    else:
        print("Tests passed!!")


def test_initalize(fn):
    start = State(0, 0)
    goal = State(6, 5)
    planner = ARAStar_Planner(GRAPH_SMALL, start, goal, 2.5, stepsize=1)

    # write garbage into planner to make sure it's cleared
    planner.OPEN = {1: 1, 2: 2, 3: 3}
    planner.CLOSED = {1, 2, 3}
    planner.INCONS = {1, 2, 3}
    planner.PARENTS = {1: 1, 2: 2, 3: 3}
    planner.alg_history[1] = [1]
    planner.paths_found = {1: 1, 2: 2, 3: 3}

    fn(planner)

    # verify planner initial state
    assert_equal(planner.g, {goal: np.inf, start: 0})
    assert_almost_equal(planner.OPEN[start], 19.525624189766635)
    assert_equal(len(planner.OPEN), 1)
    assert_equal(planner.CLOSED, set())
    assert_equal(planner.INCONS, set())
    assert_equal(planner.PARENTS, {start: start})
    assert_equal(planner.alg_history, {})
    assert_equal(planner.paths_found, {})

    # verify planner OPEN queue on larger graph
    start = State(24, 4)
    goal = State(4, 44)
    planner = ARAStar_Planner(GRAPH_LARGE, start, goal, 1.5, .2)
    fn(planner)
    assert_almost_equal(planner.OPEN[start], 67.0820393249937)

    test_ok()


def _test_alg_hist(open_user: dict, open_soln: dict, closed_user: set, closed_soln: set):
    # verify fvalues are correct for each state in OPEN queue
    for s, fval in open_soln.items():
        assert_almost_equal(fval, open_user.get(s, -1))
    # verify CLOSED sets match
    assert_equal(closed_user, closed_soln)


def test_improve_path(fn, initialize):
    start = State(0, 0)
    goal = State(6, 5)
    eps = 2.5
    planner = ARAStar_Planner(GRAPH_SMALL, start, goal, eps, stepsize=1)
    initialize(planner)
    fn(planner)
    planner.publish_path()

    # verify final path matches
    path = {eps: [
       State(x=0, y=0),
       State(x=1, y=1),
       State(x=2, y=1),
       State(x=3, y=1),
       State(x=4, y=0),
       State(x=5, y=1),
       State(x=5, y=2),
       State(x=4, y=3),
       State(x=3, y=4),
       State(x=4, y=5),
       State(x=5, y=5),
       State(x=6, y=5)]}
    assert_almost_equal(planner.path_cost(eps), 13.485281374238571)
    assert_equal(planner.paths_found, path)

    # verify user history matches for first step of search
    user_hist = planner.alg_history[eps][1]
    OPEN_soln = {
        State(x=0, y=1): 19.027756377319946,
        State(x=1, y=1): 17.422024155955217,
    }
    CLOSED_soln = {start}
    _test_alg_hist(user_hist.OPEN, OPEN_soln, user_hist.CLOSED, CLOSED_soln)

    # verify user history matches for second step of search
    user_hist = planner.alg_history[eps][2]
    OPEN_soln = {
        State(x=0, y=1): 19.027756377319946,
        State(x=0, y=2): 19.598936955994617,
        State(x=2, y=1): 16.556349186104047
    }
    CLOSED_soln = {start, State(x=1, y=1)}
    _test_alg_hist(user_hist.OPEN, OPEN_soln, user_hist.CLOSED, CLOSED_soln)

    # verify user history matches for third step of search
    user_hist = planner.alg_history[eps][3]
    OPEN_soln = {
        State(x=0, y=1): 19.027756377319946,
        State(x=0, y=2): 19.598936955994617,
        State(x=3, y=0): 18.40580686185944,
        State(x=3, y=1): 15.914213562373096
    }
    CLOSED_soln = {start, State(x=1, y=1), State(x=2, y=1)}
    _test_alg_hist(user_hist.OPEN, OPEN_soln, user_hist.CLOSED, CLOSED_soln)

    # verify user history matches for last step of search
    user_hist = planner.alg_history[eps][-1]
    OPEN_soln = {
        State(x=0, y=1): 19.027756377319946,
        State(x=0, y=2): 19.598936955994617,
        State(x=2, y=5): 21.48528137423857,
        State(x=3, y=0): 18.40580686185944,
        State(x=3, y=3): 18.670732438152353,
        State(x=3, y=5): 18.571067811865476,
        State(x=5, y=0): 18.575975908728154,
        State(x=6, y=5): 13.485281374238571
    }
    CLOSED_soln = {
        State(x=0, y=0),
        State(x=1, y=1),
        State(x=2, y=1),
        State(x=3, y=1),
        State(x=3, y=4),
        State(x=4, y=0),
        State(x=4, y=3),
        State(x=4, y=5),
        State(x=5, y=1),
        State(x=5, y=2),
        State(x=5, y=3),
        State(x=5, y=5)
    }
    _test_alg_hist(user_hist.OPEN, OPEN_soln, user_hist.CLOSED, CLOSED_soln)

    test_ok()


def test_calc_epsilon_prime(fn, improve_path, initialize):
    planner = ARAStar_Planner(GRAPH_SMALL, State(0, 0), State(6, 5), 2.5, stepsize=1)
    initialize(planner)
    improve_path(planner)
    assert_almost_equal(fn(planner), 1.6423228537944068)

    planner = ARAStar_Planner(GRAPH_LARGE, State(24, 4), State(4, 44), 1.5, .2)
    initialize(planner)
    improve_path(planner)
    assert_almost_equal(fn(planner), 1.2514705177523118)

    test_ok()


def test_run(fn):
    planner = ARAStar_Planner(GRAPH_LARGE, State(24, 4), State(4, 44), 1.5, .2)
    fn(planner)

    # verify path costs match for each path found
    path_costs_soln = {
        1.5: 55.11269837220807,
        1.3: 55.11269837220807,
        1.1: 53.455844122715675,
        1: 51.59797974644663}
    for eps, cost in path_costs_soln.items():
        assert_almost_equal(planner.path_cost(eps), cost)

    # verify paths go in expected directions by checking that certain
    # states are included in each path
    paths_found_soln = {
        1.5: State(x=0, y=29),
        1.3: State(x=0, y=29),
        1.1: State(x=15, y=29),
        1: State(x=28, y=19)
    }
    paths_found_user = planner.paths_found
    for eps, s in paths_found_soln.items():
        assert s in paths_found_user[eps]
