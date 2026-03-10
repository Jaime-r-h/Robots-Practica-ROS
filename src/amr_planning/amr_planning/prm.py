import datetime
import numpy as np
import os
import pytz
import random
import time

# This try-except enables local debugging of the PRM class
try:
    from amr_planning.maps import Map
except ImportError:
    from maps import Map

from matplotlib import pyplot as plt


class PRM:
    """Class to plan a path to a given destination using probabilistic roadmaps (PRM)."""

    def __init__(
        self,
        map_path: str,
        obstacle_safety_distance=0.08,
        use_grid: bool = False,
        node_count: int = 50,
        grid_size=0.1,
        connection_distance: float = 0.15,
        sensor_range_max: float = 8.0,
        logger=None,
        simulation: bool = False,
    ):
        """Probabilistic roadmap (PRM) class initializer.

        Args:
            map_path: Path to the map of the environment.
            obstacle_safety_distance: Distance to grow the obstacles by [m].
            use_grid: Sample from a uniform distribution when False.
                Use a fixed step grid layout otherwise.
            node_count: Number of random nodes to generate. Only considered if use_grid is False.
            grid_size: If use_grid is True, distance between consecutive nodes in x and y.
            connection_distance: Maximum distance to consider adding an edge between two nodes [m].
            sensor_range_max: Sensor measurement range [m].
            logger: Logger object to output messages with different severity levels.
            simulation: True if running in simulation, False if running on the real robot.

        """
        self._map: Map = Map(
            map_path,
            sensor_range=sensor_range_max,
            safety_distance=obstacle_safety_distance,
            compiled_intersect=False,
            use_regions=False,
        )

        self._graph: dict[tuple[float, float], list[tuple[float, float]]] = self._create_graph(
            use_grid,
            node_count,
            grid_size,
            connection_distance,
        )

        self._logger = logger
        self._simulation: bool = simulation

        self._figure, self._axes = plt.subplots(1, 1, figsize=(7, 7))
        self._timestamp = datetime.datetime.now(pytz.timezone("Europe/Madrid")).strftime(
            "%Y-%m-%d_%H-%M-%S"
        )

    def find_path(
        self, start: tuple[float, float], goal: tuple[float, float]
    ) -> list[tuple[float, float]]:
        """Computes the shortest path from a start to a goal location using the A* algorithm.

        Args:
            start: Initial location in (x, y) [m] format.
            goal: Destination in (x, y) [m] format.

        Returns:
            Path to the destination. The first value corresponds to the initial location.

        """
        # Check if the goal is valid
        if not self._map.contains(goal):
            raise ValueError("Goal location is outside the environment.")

        ancestors: dict[tuple[float, float], tuple[float, float]] = {}  # {(x, y: (x_prev, y_prev)}

        # TODO: 4.3. Complete the function body (i.e., replace the code below).

        nodes = list(self._graph.keys())
 
        start_node = min(nodes, key=lambda n: np.hypot(n[0]-start[0], n[1]-start[1]))
    
        goal_node = min(nodes, key=lambda n: np.hypot(n[0]-goal[0], n[1]-goal[1]))
    
        open_list = {}
        closed_set = set()

        g_start = np.hypot(start_node[0]-start[0], start_node[1]-start[1])
        h_start = np.hypot(goal_node[0]-start_node[0], goal_node[1]-start_node[1])
    
        open_list[start_node] = (g_start + h_start, g_start)

        ancestors[start_node] = start
    
        while open_list:
    
            node = min(open_list, key=lambda k: open_list.get(k)[0])
            f, g = open_list[node]
    
            del open_list[node]
    
            if node == goal_node:
                ancestors[goal] = node
                return self._reconstruct_path(start, goal, ancestors)
    
            closed_set.add(node)
    
            for neighbor in self._graph[node]:
    
                if neighbor in closed_set:
                    continue
    
                g_new = g + np.hypot(neighbor[0]-node[0], neighbor[1]-node[1])
                h = np.hypot(goal_node[0]-neighbor[0], goal_node[1]-neighbor[1])
                f_new = g_new + h
    
                if neighbor not in open_list:
                    open_list[neighbor] = (f_new, g_new)
                    ancestors[neighbor] = node
    
                else:
                    _, g_existing = open_list[neighbor]
    
                    if g_new < g_existing:
                        open_list[neighbor] = (f_new, g_new)
                        ancestors[neighbor] = node
    
        raise RuntimeError("No path could be found.")
        
    @staticmethod
    def smooth_path(
        path: list[tuple[float, float]],
        data_weight: float = 0.1,
        smooth_weight: float = 0.3,
        additional_smoothing_points: int = 0,
        tolerance: float = 1e-6,
    ) -> list[tuple[float, float]]:
        """Computes a smooth path from a piecewise linear path.

        Args:
            path: Non-smoothed path to the goal (start location first).
            data_weight: The larger, the more similar the output will be to the original path.
            smooth_weight: The larger, the smoother the output path will be.
            additional_smoothing_points: Number of equally spaced intermediate points to add
                between two nodes of the original path.
            tolerance: The algorithm will stop when after an iteration the smoothed path changes
                less than this value.

        Returns: Smoothed path (initial location first) in (x, y) [m] format.

        """
        # TODO: 4.5. Complete the function body (i.e., load smoothed_path).
        smoothed_path: list[tuple[float, float]] = []

        extended_path: list[tuple[float, float]] = []

        for i in range(len(path) - 1):
            p0 = path[i]
            p1 = path[i + 1]
    
            extended_path.append(p0)
    
            for j in range(1, additional_smoothing_points + 1):
                t = j / (additional_smoothing_points + 1)
                x = p0[0] + t * (p1[0] - p0[0])
                y = p0[1] + t * (p1[1] - p0[1])
                extended_path.append((x, y))
    
        extended_path.append(path[-1])
    
        original = [list(point) for point in extended_path]
        smoothed = [list(point) for point in extended_path]
    
        change = tolerance
    
        while change >= tolerance:
            change = 0.0
    
            for i in range(1, len(smoothed) - 1):
                for dim in range(2):
                    old_value = smoothed[i][dim]
    
                    smoothed[i][dim] += (
                        data_weight * (original[i][dim] - smoothed[i][dim])
                        + smooth_weight * (smoothed[i - 1][dim] + smoothed[i + 1][dim] - 2.0 * smoothed[i][dim])
                    )
    
                    change += abs(old_value - smoothed[i][dim])
    
        smoothed_path = [tuple(point) for point in smoothed]
    
        return smoothed_path

    def plot(
        self,
        axes,
        path: list[tuple[float, float]] = (),
        smoothed_path: list[tuple[float, float]] = (),
    ):
        """Draws particles.

        Args:
            axes: Figure axes.
            path: Path (start location first).
            smoothed_path: Smoothed path (start location first).

        Returns:
            axes: Modified axes.

        """
        # Plot the nodes
        x, y = zip(*self._graph.keys())
        axes.plot(list(x), list(y), "co", markersize=1)

        # Plot the edges
        for node, neighbors in self._graph.items():
            x_start, y_start = node

            if neighbors:
                for x_end, y_end in neighbors:
                    axes.plot([x_start, x_end], [y_start, y_end], "c-", linewidth=0.25)

        # Plot the path
        if path:
            x_val = [x[0] for x in path]
            y_val = [x[1] for x in path]

            axes.plot(x_val, y_val)  # Plot the path
            axes.plot(x_val[1:-1], y_val[1:-1], "bo", markersize=4)  # Draw nodes as blue circles

        # Plot the smoothed path
        if smoothed_path:
            x_val = [x[0] for x in smoothed_path]
            y_val = [x[1] for x in smoothed_path]

            axes.plot(x_val, y_val, "y")  # Plot the path
            axes.plot(x_val[1:-1], y_val[1:-1], "yo", markersize=2)  # Draw nodes as yellow circles

        if path or smoothed_path:
            axes.plot(
                x_val[0], y_val[0], "rs", markersize=7
            )  # Draw a red square at the start location
            axes.plot(
                x_val[-1], y_val[-1], "g*", markersize=12
            )  # Draw a green star at the goal location

        return axes

    def show(
        self,
        title: str = "",
        path=(),
        smoothed_path=(),
        display: bool = False,
        block: bool = False,
        save_figure: bool = False,
        save_dir: str = "images",
    ):
        """Displays the current particle set on the map.

        Args:
            title: Plot title.
            path: Path (start location first).
            smoothed_path: Smoothed path (start location first).
            display: True to open a window to visualize the particle filter evolution in real-time.
                Time consuming. Does not work inside a container unless the screen is forwarded.
            block: True to stop program execution until the figure window is closed.
            save_figure: True to save figure to a .png file.
            save_dir: Image save directory.

        """
        figure = self._figure
        axes = self._axes
        axes.clear()

        axes = self._map.plot(axes)
        axes = self.plot(axes, path, smoothed_path)

        axes.set_title(title)
        figure.tight_layout()  # Reduce white margins

        if display:
            plt.show(block=block)
            plt.pause(0.001)  # Wait for 1 ms or the figure won't be displayed

        if display:
            plt.show(block=block)

        if save_figure:
            save_path = os.path.join(os.path.dirname(__file__), "..", save_dir)

            if not os.path.isdir(save_path):
                os.makedirs(save_path)

            file_name = f"{self._timestamp} {title.lower()}.png"
            file_path = os.path.join(save_path, file_name)
            figure.savefig(file_path)

    def _connect_nodes(
        self,
        graph: dict[tuple[float, float], list[tuple[float, float]]],
        connection_distance: float = 0.15,
    ) -> dict[tuple[float, float], list[tuple[float, float]]]:
        """Connects every generated node with all the nodes that are closer than a given threshold.

        Args:
            graph: A dictionary with (x, y) [m] tuples as keys and empty lists as values.
            connection_distance: Maximum distance to consider adding an edge between two nodes [m].

        Returns: A modified graph with lists of connected nodes as values.

        """
        # TODO: 4.2. Complete the missing function body with your code.
        nodes = list(graph.keys())
 
        for i, node_a in enumerate(nodes):
            for node_b in nodes[i + 1:]:
    
                distance = np.hypot(node_a[0] - node_b[0], node_a[1] - node_b[1])
    
                if distance <= connection_distance:
    
                    segment = [node_a, node_b]
    
                    if not self._map.crosses(segment):
                        graph[node_a].append(node_b)
                        graph[node_b].append(node_a)

        return graph

    def _create_graph(
        self,
        use_grid: bool = False,
        node_count: int = 50,
        grid_size=0.1,
        connection_distance: float = 0.15,
    ) -> dict[tuple[float, float], list[tuple[float, float]]]:
        """Creates a roadmap as a graph with edges connecting the closest nodes.

        Args:
            use_grid: Sample from a uniform distribution when False.
                Use a fixed step grid layout otherwise.
            node_count: Number of random nodes to generate. Only considered if use_grid is False.
            grid_size: If use_grid is True, distance between consecutive nodes in x and y.
            connection_distance: Maximum distance to consider adding an edge between two nodes [m].

        Returns: A dictionary with (x, y) [m] tuples as keys and lists of connected nodes as values.
            Key elements are rounded to a fixed number of decimal places to allow comparisons.

        """
        graph = self._generate_nodes(use_grid, node_count, grid_size)
        graph = self._connect_nodes(graph, connection_distance)

        return graph

    def _generate_nodes(
        self, use_grid: bool = False, node_count: int = 50, grid_size=0.1
    ) -> dict[tuple[float, float], list[tuple[float, float]]]:
        """Creates a set of valid nodes to build a roadmap with.

        Args:
            use_grid: Sample from a uniform distribution when False.
                Use a fixed step grid layout otherwise.
            node_count: Number of random nodes to generate. Only considered if use_grid is False.
            grid_size: If use_grid is True, distance between consecutive nodes in x and y.

        Returns: A dictionary with (x, y) [m] tuples as keys and empty lists as values.
            Key elements are rounded to a fixed number of decimal places to allow comparisons.

        """
        graph: dict[tuple[float, float], list[tuple[float, float]]] = {}

        # TODO: 4.1. Complete the missing function body with your code.

        if use_grid:
            xmin, ymin, xmax, ymax  = self._map.bounds()

            x = xmin
            while x <= xmax:
                y = ymin
                while y <= ymax:
                    node = (round(x, 3), round(y, 3))

                    if self._map.contains(node):
                        graph[node] = []

                    y += grid_size
                x += grid_size

        else:
            xmin, xmax = self._map.bounds[0]
            ymin, ymax = self._map.bounds[1]

            while len(graph) < node_count:
                x = random.uniform(xmin, xmax)
                y = random.uniform(ymin, ymax)

                node = (round(x, 3), round(y, 3))

                if self._map.contains(node):
                    graph[node] = []

        return graph

    def _reconstruct_path(
        self,
        start: tuple[float, float],
        goal: tuple[float, float],
        ancestors: dict[tuple[int, int], tuple[int, int]],
    ) -> list[tuple[float, float]]:
        """Computes the path from the start to the goal given the ancestors of a search algorithm.

        Args:
            start: Initial location in (x, y) [m] format.
            goal: Goal location in (x, y) [m] format.
            ancestors: Dictionary with (x, y) [m] tuples as keys and the node (x_prev, y_prev) [m]
                from which it was added to the open list as values.

        Returns: Path to the goal (start location first) in (x, y) [m] format.

        """
        path: list[tuple[float, float]] = []

        # TODO: 4.4. Complete the missing function body with your code.
        current = goal
        path.append(current)
    
        while current != start:
            current = ancestors[current]
            path.append(current)
    
        path.reverse()
    
        return path

if __name__ == "__main__":
    map_name = "project"
    map_path = os.path.realpath(
        os.path.join(os.path.dirname(__file__), "..", "maps", map_name + ".json")
    )

    # Create the roadmap
    start_time = time.perf_counter()
    prm = PRM(map_path, use_grid=True, node_count=250, grid_size=0.1, connection_distance=0.15)
    roadmap_creation_time = time.perf_counter() - start_time

    print(f"Roadmap creation time: {roadmap_creation_time:1.3f} s")

    # Find the path
    start_time = time.perf_counter()
    path = prm.find_path(start=(-1.0, -1.0), goal=(-0.6, 1.0))
    pathfinding_time = time.perf_counter() - start_time

    print(f"Pathfinding time: {pathfinding_time:1.3f} s")

    # Smooth the path
    start_time = time.perf_counter()
    smoothed_path = prm.smooth_path(
        path, data_weight=0.1, smooth_weight=0.3, additional_smoothing_points=3
    )
    smoothing_time = time.perf_counter() - start_time

    print(f"Smoothing time: {smoothing_time:1.3f} s")

    prm.show(path=path, smoothed_path=smoothed_path, save_figure=True)
