import json
import time
from typing import Tuple

import serial


class SerialBridge:
    def __init__(self, port: str, baud: int = 115200) -> None:
        self.serial = serial.Serial(port, baudrate=baud, timeout=0)
        self.seq = 0
        self.last_received_seq = -1
        self.last_control_time = time.time()
        self.last_vL = 0.0
        self.last_vR = 0.0
        self.last_state = "stop"

    def send_sensor_data(self, dF: float, dL: float, dR: float, dt: float) -> int:
        self.seq += 1
        payload = {
            "seq": self.seq,
            "dF": round(float(dF), 4),
            "dL": round(float(dL), 4),
            "dR": round(float(dR), 4),
            "dt": round(float(dt), 4),
        }
        self.serial.write((json.dumps(payload) + "\n").encode("utf-8"))
        return self.seq

    def receive_control(self, timeout_ms: int = 30) -> Tuple[float, float, str]:
        start = time.time()
        while (time.time() - start) * 1000 < timeout_ms:
            if self.serial.in_waiting <= 0:
                time.sleep(0.001)
                continue

            raw = self.serial.readline().decode("utf-8", errors="ignore").strip()
            if not raw:
                continue

            try:
                data = json.loads(raw)
            except json.JSONDecodeError:
                continue

            if not all(key in data for key in ("seq", "vL", "vR", "state")):
                continue

            seq = int(data["seq"])
            if self.last_received_seq >= 0 and seq != self.last_received_seq + 1:
                print(f"[WARN] sequence gap expected={self.last_received_seq + 1} got={seq}")

            vL = max(-1.0, min(1.0, float(data["vL"])))
            vR = max(-1.0, min(1.0, float(data["vR"])))
            state = str(data["state"])

            self.last_received_seq = seq
            self.last_control_time = time.time()
            self.last_vL, self.last_vR, self.last_state = vL, vR, state
            return vL, vR, state

        silence_ms = (time.time() - self.last_control_time) * 1000
        if silence_ms > 500:
            self.last_vL, self.last_vR, self.last_state = 0.0, 0.0, "timeout"
            return 0.0, 0.0, "timeout"
        if silence_ms > 200:
            return self.last_vL, self.last_vR, "stale"
        return self.last_vL, self.last_vR, self.last_state

    def close(self) -> None:
        if self.serial and self.serial.is_open:
            self.serial.close()
