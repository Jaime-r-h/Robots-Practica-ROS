from enum import Enum


class State(Enum):
    FORWARD = 1
    CHOOSE_TURN = 2
    TURNING = 3


def clamp(x, lo, hi):
    return max(lo, min(x, hi))


class WallFollower:
    LINEAR_SPEED_MAX = 0.1
    SENSOR_RANGE_MIN = 0.16
    SENSOR_RANGE_MAX = 8.0
    TRACK = 0.16
    WHEEL_RADIUS = 0.033
    WHEEL_SPEED_MAX = LINEAR_SPEED_MAX / WHEEL_RADIUS
    MAX_ANGULAR_SPEED = 0.8

    STOP_DIST = 0.22
    CLEAR_DIST = 0.6

    KP = 0.6
    KI = 0.05
    KD = 0.35
    SETPOINT = 0.2

    def get_front_left_right(self, zscan):
        front = zscan[0]
        left = zscan[60]
        right = zscan[180]
        front_left = zscan[35]
        front_right = zscan[205]
        return front, left, right, front_left, front_right

    def __init__(self, dt: float, logger=None, simulation: bool = False) -> None:
        self._dt = dt
        self._logger = logger
        self._simulation = simulation

        self.state = State.FORWARD
        self.turn_dir = 0
        self.prev_error = 0.0
        self.int_error = 0.0

        # Empieza siguiendo pared derecha si quieres
        self.is_reference_wall_right = False

    def compute_commands(self, z_scan: list[float], z_v: float, z_w: float) -> tuple[float, float]:
        front, left, right, front_left, front_right = self.get_front_left_right(z_scan)

        v = 0.0
        w = 0.0

        if self.state == State.FORWARD:
            if (
                (front < self.STOP_DIST)
                or (front_left < 0.6 * self.STOP_DIST)
                or (front_right < 0.6 * self.STOP_DIST)
            ):
                self.state = State.CHOOSE_TURN
                self.int_error = 0.0
                self.prev_error = 0.0
                return 0.0, 0.0

            distancia_lado = right if self.is_reference_wall_right else left
            error = distancia_lado - self.SETPOINT

            dt = max(self._dt, 1e-6)
            d_error = (error - self.prev_error) / dt
            d_error = clamp(d_error, -2.0, 2.0)
            self.prev_error = error

            self.int_error += error * dt
            i_error = clamp(self.int_error, -0.2, 0.2)

            sign = -1.0 if self.is_reference_wall_right else 1.0
            w = sign * (self.KP * error + self.KD * d_error + self.KI * i_error)

            w = clamp(w, -0.65, 0.65)

            v = self.LINEAR_SPEED_MAX * (1.0 - 0.7 * abs(w) / 0.65)
            v = clamp(v, 0.03, self.LINEAR_SPEED_MAX)

        elif self.state == State.CHOOSE_TURN:
            self.turn_dir = 1 if left >= right else -1

            self.state = State.TURNING
            v = 0.0
            w = self.MAX_ANGULAR_SPEED * self.turn_dir

        elif self.state == State.TURNING:
            if front > self.CLEAR_DIST:
                self.is_reference_wall_right = self.turn_dir == 1

                self.state = State.FORWARD
                self.int_error = 0.0
                self.prev_error = 0.0
                v = 0.0
                w = 0.0
            else:
                v = 0.0
                w = self.MAX_ANGULAR_SPEED * self.turn_dir

        return v, w
