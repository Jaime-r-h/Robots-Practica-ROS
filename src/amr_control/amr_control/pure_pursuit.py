import math

class PurePursuit:
    """Class to follow a path using a simple pure pursuit controller."""

    def __init__(
        self,
        dt: float,
        lookahead_distance: float = 0.5,
        logger=None,
        simulation: bool = False,
    ):
        """Pure pursuit class initializer.

        Args:
            dt: Sampling period [s].
            lookahead_distance: Distance to the next target point [m].
            logger: Logger object to output messages with different severity levels.
            simulation: True if running in simulation, False if running on the real robot.

        """
        self._dt: float = dt
        self._logger = logger
        self._lookahead_distance: float = lookahead_distance
        self._path: list[tuple[float, float]] = []
        self._simulation: bool = simulation
        
    def compute_commands(self, x: float, y: float, theta: float) -> tuple[float, float]:
        """Pure pursuit controller implementation.

        Args:
            x: Estimated robot x coordinate [m].
            y: Estimated robot y coordinate [m].
            theta: Estimated robot heading [rad].

        Returns:
            v: Linear velocity [m/s].
            w: Angular velocity [rad/s].

        """
        # TODO: 4.11. Complete the function body with your code (i.e., compute v and w).
        # print(self._path, flush=True)
        if not self._path:
            return 0.0, 0.0
    
        closest_xy, closest_idx = self._find_closest_point(x, y)
        target_xy = self._find_target_point(closest_xy, closest_idx)
    
        goal_xy = self._path[-1]
        goal_distance = math.hypot(goal_xy[0] - x, goal_xy[1] - y)
        if goal_distance < 0.05:
            return 0.0, 0.0
    
        dx = target_xy[0] - x
        dy = target_xy[1] - y
        target_heading = math.atan2(dy, dx)
    
        alpha = target_heading - theta
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))
    
        if abs(alpha) > math.pi / 4:
            v = 0.0
            w = 1.5 * alpha
            return v, w
    
        if self._simulation:
            v = 0.20
        else:
            v = 0.10
    
        if abs(alpha) > math.pi / 8:
            v *= 0.5
    
        w = 2.0 * v * math.sin(alpha) / self._lookahead_distance
    
        return v, w

    @property
    def path(self) -> list[tuple[float, float]]:
        """Path getter."""
        return self._path

    @path.setter
    def path(self, value: list[tuple[float, float]]) -> None:
        """Path setter."""
        self._path = value
        
    def _find_closest_point(self, x: float, y: float) -> tuple[tuple[float, float], int]:
        """Find the closest path point to the current robot pose.

        Args:
            x: Estimated robot x coordinate [m].
            y: Estimated robot y coordinate [m].

        Returns:
            tuple[float, float]: (x, y) coordinates of the closest path point [m].
            int: Index of the path point found.

        """
        # TODO: 4.9. Complete the function body (i.e., find closest_xy and closest_idx).
        closest_idx = 0
        closest_xy = self._path[0]
        min_distance = math.hypot(closest_xy[0] - x, closest_xy[1] - y)
    
        for i, point in enumerate(self._path[1:], start=1):
            distance = math.hypot(point[0] - x, point[1] - y)
    
            if distance < min_distance:
                min_distance = distance
                closest_idx = i
                closest_xy = point
    
        return closest_xy, closest_idx

    def _find_target_point(
        self, origin_xy: tuple[float, float], origin_idx: int
    ) -> tuple[float, float]:
        """Find the destination path point based on the lookahead distance.

        Args:
            origin_xy: Current location of the robot (x, y) [m].
            origin_idx: Index of the current path point.

        Returns:
            tuple[float, float]: (x, y) coordinates of the target point [m].

        """
        # TODO: 4.10. Complete the function body with your code (i.e., determine target_xy).
    
        for i in range(origin_idx, len(self._path)):
            point = self._path[i]
            distance = math.hypot(point[0] - origin_xy[0], point[1] - origin_xy[1])
    
            if distance >= self._lookahead_distance:
                return point
    
        return self._path[-1]