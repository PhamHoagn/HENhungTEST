import math
from dataclasses import dataclass, field
from typing import Dict, List, Tuple

MAX_SENSOR_RANGE = 3.0
MAX_SPEED_MPS = 0.9
MAX_DT = 0.05


@dataclass
class Obstacle:
    x: float
    y: float
    radius: float

    def check_ray_intersection(self, x0: float, y0: float, angle: float, max_dist: float) -> float:
        dx = math.cos(angle)
        dy = math.sin(angle)
        fx = x0 - self.x
        fy = y0 - self.y

        a = dx * dx + dy * dy
        b = 2 * (fx * dx + fy * dy)
        c = fx * fx + fy * fy - self.radius * self.radius
        discriminant = b * b - 4 * a * c
        if discriminant < 0:
            return max_dist

        root = math.sqrt(discriminant)
        t1 = (-b - root) / (2 * a)
        t2 = (-b + root) / (2 * a)
        candidates = [t for t in (t1, t2) if 0 <= t <= max_dist]
        return min(candidates) if candidates else max_dist


@dataclass
class Car:
    x: float
    y: float
    theta: float
    wheelbase: float = 0.16
    trail: List[Tuple[float, float]] = field(default_factory=list)

    def update(self, vL: float, vR: float, dt: float) -> None:
        dt = min(max(dt, 0.0), MAX_DT)
        vL = max(-1.0, min(1.0, vL))
        vR = max(-1.0, min(1.0, vR))

        left_speed = vL * MAX_SPEED_MPS
        right_speed = vR * MAX_SPEED_MPS

        v = (left_speed + right_speed) * 0.5
        omega = (right_speed - left_speed) / self.wheelbase

        self.theta += omega * dt
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt

        self.trail.append((self.x, self.y))
        if len(self.trail) > 500:
            self.trail.pop(0)

    def _cone_distance(self, center_angle: float, obstacles: List[Obstacle], rays: int = 7, cone_deg: float = 10.0) -> float:
        span = math.radians(cone_deg)
        if rays <= 1:
            ray_angles = [center_angle]
        else:
            ray_angles = [center_angle - span + (2 * span * i / (rays - 1)) for i in range(rays)]

        min_dist = MAX_SENSOR_RANGE
        for ang in ray_angles:
            for obs in obstacles:
                dist = obs.check_ray_intersection(self.x, self.y, self.theta + ang, MAX_SENSOR_RANGE)
                if dist < min_dist:
                    min_dist = dist
        return min_dist

    def get_sensor_readings(self, obstacles: List[Obstacle]) -> Tuple[float, float, float]:
        dF = self._cone_distance(0.0, obstacles)
        dL = self._cone_distance(math.pi / 2, obstacles)
        dR = self._cone_distance(-math.pi / 2, obstacles)
        return dF, dL, dR


class SimulationWorld:
    def __init__(self) -> None:
        self.car = Car(x=0.3, y=0.3, theta=0.0)
        self.obstacles = [
            Obstacle(1.1, 0.8, 0.2),
            Obstacle(2.0, 1.4, 0.25),
            Obstacle(1.8, 0.4, 0.18),
        ]
        self.waypoints = [(0.4, 0.3), (1.0, 0.3), (1.7, 1.0), (2.4, 1.6)]

    def step(self, vL: float, vR: float, dt: float) -> None:
        self.car.update(vL, vR, dt)

    def get_sensor_data(self) -> Dict[str, float]:
        dF, dL, dR = self.car.get_sensor_readings(self.obstacles)
        return {"dF": dF, "dL": dL, "dR": dR}
