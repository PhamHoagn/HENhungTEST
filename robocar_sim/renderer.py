import math
from typing import Dict, Tuple

import pygame

from simulation_engine import MAX_SENSOR_RANGE, SimulationWorld


class Renderer:
    def __init__(self, width: int = 1100, height: int = 720, scale: int = 260) -> None:
        pygame.init()
        self.width = width
        self.height = height
        self.scale = scale
        self.screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption("Robocar HIL Simulator")
        self.font = pygame.font.SysFont("consolas", 18)
        self.small_font = pygame.font.SysFont("consolas", 14)

    def _to_screen(self, x: float, y: float) -> Tuple[int, int]:
        return int(60 + x * self.scale), int(self.height - (60 + y * self.scale))

    def _ray_color(self, d: float) -> Tuple[int, int, int]:
        if d < 0.5:
            return (210, 60, 60)
        if d < 1.5:
            return (230, 200, 80)
        return (70, 200, 110)

    def draw(self, world: SimulationWorld, sensors: Dict[str, float], control: Dict[str, float], state: str, fps: float = 60.0) -> None:
        self.screen.fill((20, 25, 35))
        self._draw_grid()
        self._draw_waypoints(world)
        self._draw_obstacles(world)
        self._draw_trail(world)
        self._draw_car(world)
        self._draw_sensors(world, sensors)
        self._draw_panel(world, sensors, control, state, fps)
        pygame.display.flip()

    def _draw_grid(self) -> None:
        for x in range(60, self.width, self.scale // 2):
            pygame.draw.line(self.screen, (40, 45, 55), (x, 0), (x, self.height), 1)
        for y in range(0, self.height, self.scale // 2):
            pygame.draw.line(self.screen, (40, 45, 55), (0, y), (self.width, y), 1)

    def _draw_waypoints(self, world: SimulationWorld) -> None:
        points = [self._to_screen(x, y) for x, y in world.waypoints]
        if len(points) > 1:
            pygame.draw.lines(self.screen, (120, 150, 240), False, points, 2)
        for p in points:
            pygame.draw.circle(self.screen, (150, 180, 250), p, 5)

    def _draw_obstacles(self, world: SimulationWorld) -> None:
        for obstacle in world.obstacles:
            center = self._to_screen(obstacle.x, obstacle.y)
            radius = int(obstacle.radius * self.scale)
            pygame.draw.circle(self.screen, (160, 90, 100), center, radius)

    def _draw_trail(self, world: SimulationWorld) -> None:
        if len(world.car.trail) < 2:
            return
        pts = [self._to_screen(x, y) for x, y in world.car.trail]
        pygame.draw.lines(self.screen, (90, 220, 240), False, pts, 2)

    def _draw_car(self, world: SimulationWorld) -> None:
        x, y = self._to_screen(world.car.x, world.car.y)
        radius = 14
        pygame.draw.circle(self.screen, (240, 240, 245), (x, y), radius)
        hx = x + int(radius * 1.8 * math.cos(world.car.theta))
        hy = y - int(radius * 1.8 * math.sin(world.car.theta))
        pygame.draw.line(self.screen, (30, 30, 40), (x, y), (hx, hy), 4)

    def _draw_sensors(self, world: SimulationWorld, sensors: Dict[str, float]) -> None:
        base_x, base_y = self._to_screen(world.car.x, world.car.y)
        angles = {"dF": 0.0, "dL": math.pi / 2, "dR": -math.pi / 2}
        for key, rel in angles.items():
            dist = sensors[key]
            ang = world.car.theta + rel
            end_x = world.car.x + dist * math.cos(ang)
            end_y = world.car.y + dist * math.sin(ang)
            sx, sy = self._to_screen(end_x, end_y)
            pygame.draw.line(self.screen, self._ray_color(dist), (base_x, base_y), (sx, sy), 3)

    def _draw_panel(self, world: SimulationWorld, sensors: Dict[str, float], control: Dict[str, float], state: str, fps: float) -> None:
        panel = pygame.Rect(self.width - 330, 20, 300, 230)
        pygame.draw.rect(self.screen, (30, 35, 45), panel, border_radius=8)
        pygame.draw.rect(self.screen, (80, 90, 110), panel, 2, border_radius=8)

        lines = [
            f"FPS: {fps:5.1f}",
            f"State: {state}",
            f"Car x/y: {world.car.x:.2f}, {world.car.y:.2f}",
            f"Heading: {math.degrees(world.car.theta):.1f} deg",
            f"dF: {sensors['dF']:.2f} m",
            f"dL: {sensors['dL']:.2f} m",
            f"dR: {sensors['dR']:.2f} m",
            f"vL: {control['vL']:.2f}",
            f"vR: {control['vR']:.2f}",
            f"Range max: {MAX_SENSOR_RANGE:.1f} m",
        ]
        for i, text in enumerate(lines):
            surface = self.font.render(text, True, (220, 225, 235))
            self.screen.blit(surface, (panel.x + 14, panel.y + 14 + i * 21))


def demo() -> None:
    world = SimulationWorld()
    renderer = Renderer()
    clock = pygame.time.Clock()
    running = True

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        world.step(0.35, 0.45, 1 / 60)
        sensors = world.get_sensor_data()
        renderer.draw(world, sensors, {"vL": 0.35, "vR": 0.45}, "demo", clock.get_fps())
        clock.tick(60)

    pygame.quit()


if __name__ == "__main__":
    demo()
