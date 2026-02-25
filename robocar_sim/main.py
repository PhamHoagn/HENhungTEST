import argparse
import time

import pygame

from renderer import Renderer
from serial_bridge import SerialBridge
from simulation_engine import MAX_DT, SimulationWorld


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Robocar HIL simulator")
    parser.add_argument("--port", default="COM3", help="Serial port (e.g. COM3 or /dev/ttyUSB0)")
    parser.add_argument("--baud", default=115200, type=int, help="Serial baud rate")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    world = SimulationWorld()
    renderer = Renderer()
    clock = pygame.time.Clock()

    bridge = None
    try:
        bridge = SerialBridge(port=args.port, baud=args.baud)
        print(f"Connected to serial {args.port} @ {args.baud}")
    except Exception as exc:
        print(f"[WARN] Serial unavailable: {exc}. Running in dry-run mode.")

    running = True
    last_time = time.time()
    vL, vR, state = 0.0, 0.0, "stop"

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        now = time.time()
        dt = min(now - last_time, MAX_DT)
        last_time = now

        sensors = world.get_sensor_data()

        if bridge:
            try:
                bridge.send_sensor_data(sensors["dF"], sensors["dL"], sensors["dR"], dt)
                vL, vR, state = bridge.receive_control(timeout_ms=30)
            except Exception as exc:
                print(f"[WARN] Serial IO failed: {exc}")
                bridge.close()
                bridge = None
                vL, vR, state = 0.0, 0.0, "timeout"
        else:
            state = "dry-run"

        world.step(vL, vR, dt)
        renderer.draw(world, sensors, {"vL": vL, "vR": vR}, state, clock.get_fps())
        clock.tick(60)

    if bridge:
        bridge.close()
    pygame.quit()


if __name__ == "__main__":
    main()
