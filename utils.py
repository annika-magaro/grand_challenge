from typing import NamedTuple
from dataclasses import dataclass

import matplotlib.pyplot as plt
import numpy as np
# import ipywidgets as widgets

from matplotlib.colors import ListedColormap, BoundaryNorm
from matplotlib.lines import Line2D
from matplotlib.patches import Patch


def in_notebook() -> bool:
    try:
        from IPython import get_ipython  # pylint: disable=import-outside-toplevel
        if 'IPKernelApp' not in get_ipython().config:  # pragma: no cover
            return False
    except ImportError:
        return False
    except AttributeError:
        return False
    return True


GRAPH_SMALL = np.array([
    [0, 0, 0, 0, 0, 0],
    [1, 0, 1, 1, 1, 0],
    [1, 0, 1, 1, 1, 0],
    [0, 0, 1, 0, 0, 0],
    [0, 1, 1, 0, 1, 0],
    [0, 0, 0, 0, 1, 0],
    [1, 1, 1, 1, 1, 0],
])

GRAPH_LARGE = np.zeros((29, 49))
GRAPH_LARGE[14, 9:20] = 1
GRAPH_LARGE[14:28, 19] = 1
GRAPH_LARGE[1:15, 29] = 1
GRAPH_LARGE[14:29, 39] = 1


class State(NamedTuple):
    """A state in a graph represented by a 2D array."""
    x: int
    y: int


@dataclass
class ARAStar_State:
    """A snapshot of the different States being tracked by ARA*,
    as well as the current path being expanded."""
    OPEN: dict
    CLOSED: set
    INCONS: set
    current_path: list[State]


class ARAStar_Plotter:
    CELL_TYPES = ['CLEAR', 'OBSTACLE', 'OPEN', 'INCONS', 'CLOSED']
    CELL_COLORS = ['white', 'black', 'dodgerblue', 'chartreuse', 'silver']
    UNKNOWN_CELL_COLOR = 'red'
    PATH_COLOR = 'red'
    GOAL_COLOR = 'magenta'
    PATH_MARKER = 'x'
    # COLORMAP = ListedColormap(CELL_COLORS, 'ARA*_planner').with_extremes(
    #     under=UNKNOWN_CELL_COLOR, over=UNKNOWN_CELL_COLOR)
    COLORMAP = ListedColormap(CELL_COLORS, 'ARA*_planner')
    COLOR_BOUNDS = BoundaryNorm(np.arange(len(CELL_TYPES) + 1), COLORMAP.N)

    def __init__(self, graph: np.ndarray, start: State, goal: State):
        self.graph = graph
        self.start = start
        self.goal = goal
        self._selected_axes = {}

    def visualize_graph(self):
        fig, ax = plt.subplots()
        fig.canvas.toolbar_visible = False
        fig.canvas.footer_visible = False

        ax.pcolormesh(self.graph, cmap=self.COLORMAP, norm=self.COLOR_BOUNDS,
                      edgecolors='k', linewidth=0.5)
        ax.invert_yaxis()  # do this otherwise graph looks upside down
        ax.xaxis.set_tick_params(labeltop=True, labelbottom=False, bottom=False)

        # plot start and goal markers
        ax.plot(self.start.y + 0.5, self.start.x + 0.5, color=self.PATH_COLOR,
                marker=self.PATH_MARKER, linestyle='none')
        ax.plot(self.goal.y + 0.5, self.goal.x + 0.5, color=self.GOAL_COLOR,
                marker=self.PATH_MARKER, linestyle='none')

        # add start/goal to legend
        handles = []
        handles.append(Line2D([], [], color=self.PATH_COLOR, linestyle='none',
                              marker=self.PATH_MARKER,label='Start'))
        handles.append(Line2D([], [], color=self.GOAL_COLOR, linestyle='none',
                              marker=self.PATH_MARKER, label='Goal'))
        ax.legend(handles=handles)

        plt.show()


    def _add_legend(self, ax: plt.Axes):
        handles = []
        for cell_type, color in zip(self.CELL_TYPES, self.CELL_COLORS):
            p = Patch(facecolor=color, label=cell_type, edgecolor='k')
            handles.append(p)

        handles.append(Line2D([], [], color=self.PATH_COLOR,
                              marker=self.PATH_MARKER,label='Path'))
        handles.append(Line2D([], [], color=self.GOAL_COLOR, linestyle='none',
                              marker=self.PATH_MARKER, label='Goal'))
        ax.legend(handles=handles)

    def _path_to_xyvals(self, path: list[State]) -> tuple[list[float], list[float]]:
        # reverse x/y order since graph is inverted
        xvals, yvals = [], []
        for x, y in path:
            xvals.append(y + 0.5)
            yvals.append(x + 0.5)
        return xvals, yvals

    def _add_path(self, ax: plt.Axes, path: list[State]):
        xvals, yvals = self._path_to_xyvals(path)
        ax.plot(xvals, yvals, color=self.PATH_COLOR)
        ax.plot(xvals[0], yvals[0], xvals[-1], yvals[-1], color=self.PATH_COLOR,
                marker=self.PATH_MARKER, linestyle='none')  # plot path start/end markers
        ax.plot(self.goal.y + 0.5, self.goal.x + 0.5, color=self.GOAL_COLOR,
                marker=self.PATH_MARKER, linestyle='none')  # plot goal marker

    def plot_episode(self, epsilon: float, history: list[ARAStar_State]):
        with plt.ioff():
            fig = plt.figure()
        fig.canvas.toolbar_visible = False
        fig.canvas.footer_visible = False
        axs = []

        for i, alg_state in enumerate(history):
            ax = fig.add_axes([0.125, 0.12, .8, 0.75], label=i, visible=i == 0)
            graph = self.graph.copy()
            for s in alg_state.OPEN:
                graph[s] = self.CELL_TYPES.index('OPEN')
            for s in alg_state.INCONS:
                graph[s] = self.CELL_TYPES.index('INCONS')
            for s in alg_state.CLOSED:
                graph[s] = self.CELL_TYPES.index('CLOSED')

            ax.pcolormesh(graph, cmap=self.COLORMAP, norm=self.COLOR_BOUNDS,
                              edgecolors='k', linewidth=0.5)
            self._add_path(ax, alg_state.current_path)

            ax.set_title(rf'Anytime Repairing A*, $\epsilon={epsilon}$, '
                         f'Iteration {i}')
            ax.invert_yaxis()  # do this otherwise graph looks upside down
            ax.xaxis.set_tick_params(labeltop=True, labelbottom=False, bottom=False)
            self._add_legend(ax)
            axs.append(ax)

        self._selected_axes[epsilon] = 0
        def select_ax(new_ax):
            current_ax = self._selected_axes[epsilon]
            new_ax %= len(history)
            if new_ax != current_ax:
                axs[current_ax].set_visible(False)
                axs[new_ax].set_visible(True)
                self._selected_axes[epsilon] = new_ax
                fig.canvas.draw_idle()

        bforward = widgets.Button(
            disabled=False,
            button_style='',
            icon='caret-right'
        )
        bbackward = widgets.Button(
            disabled=False,
            button_style='',
            icon='caret-left'
        )
        bbackward.on_click(lambda _: select_ax(self._selected_axes[epsilon] - 1))
        bforward.on_click(lambda _: select_ax(self._selected_axes[epsilon] + 1))
        footer = widgets.HBox([bbackward, bforward])
        return widgets.VBox([fig.canvas, footer])

    def plot_final_state(self, epsilon: float, history: list[ARAStar_State]):
        fig, ax = plt.subplots()
        fig.canvas.toolbar_visible = False
        fig.canvas.footer_visible = False

        alg_state = history[-1]  # only care about final expansion
        graph = self.graph.copy()
        for s in alg_state.OPEN:
            graph[s] = self.CELL_TYPES.index('OPEN')
        for s in alg_state.INCONS:
            graph[s] = self.CELL_TYPES.index('INCONS')
        for s in alg_state.CLOSED:
            graph[s] = self.CELL_TYPES.index('CLOSED')

        ax.pcolormesh(graph, cmap=self.COLORMAP, norm=self.COLOR_BOUNDS,
                            edgecolors='k', linewidth=0.5)
        self._add_path(ax, alg_state.current_path)

        ax.set_title(rf'Anytime Repairing A*, $\epsilon={epsilon}$')
        ax.invert_yaxis()  # do this otherwise graph looks upside down
        ax.xaxis.set_tick_params(labeltop=True, labelbottom=False, bottom=False)
        self._add_legend(ax)

        plt.show()

    def plot_paths_found(self, paths_found: dict[float, list[State]]):
        fig, ax = plt.subplots()
        fig.canvas.toolbar_visible = False
        fig.canvas.footer_visible = False

        ax.pcolormesh(self.graph, cmap=self.COLORMAP, norm=self.COLOR_BOUNDS,
                                    edgecolors='k', linewidth=0.5)
        ax.set_title('Anytime Repairing A*: All Paths Found')
        ax.invert_yaxis()  # do this otherwise graph looks upside down
        ax.xaxis.set_tick_params(labeltop=True, labelbottom=False, bottom=False)

        for epsilon, path in paths_found.items():
            xvals, yvals = self._path_to_xyvals(path)
            ax.plot(xvals, yvals, label=rf'$\epsilon={round(epsilon, 2)}$')

        ax.legend()
        plt.show()