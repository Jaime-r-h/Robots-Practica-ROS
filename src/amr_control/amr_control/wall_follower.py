from enum import Enum


class State(Enum):
    FORWARD = 1
    CHOOSE_TURN = 2
    TURNING = 3


def clamp(x, lo, hi):
    return max(lo, min(x, hi))


class WallFollower:
    """Class to safely explore an environment (without crashing) when the pose is unknown."""

    # Robot limits
    LINEAR_SPEED_MAX = 0.1  # Maximum linear velocity in the abscence of angular velocity [m/s]
    SENSOR_RANGE_MIN = 0.16  # Minimum LiDAR sensor range [m]
    SENSOR_RANGE_MAX = 8.0  # Maximum LiDAR sensor range [m]
    TRACK = 0.16  # Distance between same axle wheels [m]
    WHEEL_RADIUS = 0.033  # Radius of the wheels [m]
    WHEEL_SPEED_MAX = LINEAR_SPEED_MAX / WHEEL_RADIUS  # Maximum motor angular speed [rad/s]
    MAX_ANGULAR_SPEED = 0.8
    STOP_DIST = 0.22
    CLEAR_DIST = 0.8
    KP = 0.6
    KI = 0.05
    KD = 0.35
    SETPOINT = 0.2

    def get_front_left_right(self, zscan):
        front = zscan[0]
        left = zscan[60]
        right = zscan[180]
        return front, left, right

    def __init__(self, dt: float, logger=None, simulation: bool = False) -> None:
        """Wall following class initializer.

        Args:
            dt: Sampling period [s].
            logger: Logger object to output messages with different severity levels.
            simulation: True if running in simulation, False if running on the real robot.

        """
        self._dt: float = dt
        self._logger = logger
        self._simulation: bool = simulation
        self.state = State.FORWARD
        self.turn_dir = 0
        self.prev_error = 0.0
        self.int_error = 0.0
        self.is_reference_wall_right = False

    def compute_commands(self, z_scan: list[float], z_v: float, z_w: float) -> tuple[float, float]:
        """Wall following exploration algorithm.

        Args:
            z_scan: Distance from every LiDAR ray to the closest obstacle [m].
            z_v: Odometric estimate of the linear velocity of the robot center [m/s].
            z_w: Odometric estimate of the angular velocity of the robot center [rad/s].

        Returns:
            The linear and angular velocity commands for the robot. They must
                v: Linear velocity [m/s].
                w: Angular velocity [rad/s].

        """
        # TODO: 2.14. Complete the function body with your code (i.e., compute v and w).
        front, left, right = self.get_front_left_right(z_scan)

        v = 0.0
        w = 0.0

        if self.state == State.FORWARD:
            if front < self.STOP_DIST:
                self.state = State.CHOOSE_TURN
                v = 0.0
                w = 0.0
                self.prev_error = 0.0

            else:
                distancia_lado = right if self.is_reference_wall_right else left
                error = distancia_lado - self.SETPOINT

                # PID
                d_error = (error - self.prev_error) / self._dt
                self.prev_error = error

                self.int_error += error * self._dt
                i_error = clamp(self.int_error, -0.2, 0.2)

                w = (self.KP * error + self.KD * d_error + self.KI * i_error) * (
                    -1 if self.is_reference_wall_right else 1
                )

                w = clamp(w, -0.6, 0.6)
                v = self.LINEAR_SPEED_MAX * (1 - abs(w) / 0.6)

        elif self.state == State.CHOOSE_TURN:
            l = left if left is not None else 0.0
            r = right if right is not None else 0.0
            if l >= r:
                self.turn_dir = 1
            else:
                self.turn_dir = -1

            self.state = State.TURNING

        elif self.state == State.TURNING:
            if front > self.CLEAR_DIST:
                if self.turn_dir == 1:
                    self.is_reference_wall_right = True
                else:
                    self.is_reference_wall_right = False

                self.state = State.FORWARD
                v = 0.0
                w = 0.0
            else:
                v = 0.0
                w = self.MAX_ANGULAR_SPEED * self.turn_dir

        return v, w
