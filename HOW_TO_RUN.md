# How to Run

1. Install Python dependencies:
   ```bash
   cd robocar_sim
   pip install -r requirements.txt
   ```
2. Open Wokwi project from `robocar_esp32_wokwi/` and start simulation.
3. Find virtual serial port exposed by Wokwi.
4. Run simulator:
   ```bash
   cd robocar_sim
   python main.py --port COM3 --baud 115200
   ```
5. Verify JSON traffic in Wokwi Serial Monitor and pygame scene behavior.

## Quick checks
- Standalone renderer:
  ```bash
  python renderer.py
  ```
- Sensor output quick test:
  ```bash
  python -c "from simulation_engine import SimulationWorld; print(SimulationWorld().get_sensor_data())"
  ```
