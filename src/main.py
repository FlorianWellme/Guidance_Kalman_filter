import asyncio
import serial
import serial.tools.list_ports
import threading
import time
from collections import deque
from fastapi import FastAPI, WebSocket
from fastapi.responses import HTMLResponse
from pydantic import BaseModel
from fastapi.staticfiles import StaticFiles
from pathlib import Path

# ---------------- CONFIG ----------------
BAUD_RATE = 115200
MAX_POINTS = 200

# ---------------- STATE ----------------
running = False
serial_port = None
ser = None

buffers = {
    "acc": [deque(maxlen=MAX_POINTS) for _ in range(3)],
    "gyro": [deque(maxlen=MAX_POINTS) for _ in range(3)],
    "angles": [deque(maxlen=MAX_POINTS) for _ in range(5)],
}

# ---------------- SERIAL THREAD ----------------
def serial_loop():
    global ser
    while True:
        if running and ser and ser.is_open:
            try:
                line = ser.readline().decode(errors="ignore").strip()
                values = list(map(float, line.split(",")))
                if len(values) == 12:
                    for i in range(3):
                        buffers["acc"][i].append(values[i])
                        buffers["gyro"][i].append(values[i + 3])
                    buffers["angles"][0].append(values[6])
                    buffers["angles"][1].append(values[9])
                    buffers["angles"][2].append(values[7])
                    buffers["angles"][3].append(values[10])
                    buffers["angles"][4].append(values[8])
            except Exception:
                pass
        time.sleep(0.005)

threading.Thread(target=serial_loop, daemon=True).start()

# ---------------- FASTAPI ----------------
app = FastAPI()

BASE_DIR = Path(__file__).resolve().parent  # = Guidance/src
IMAGES_DIR = BASE_DIR.parent / "images"    # = Guidance/images

print("Static images dir:", IMAGES_DIR)

app.mount("/static", StaticFiles(directory=IMAGES_DIR), name="static")

@app.get("/")
def index():
    return HTMLResponse(open("index.html").read())

@app.get("/ports")
def list_ports():
    return [p.device for p in serial.tools.list_ports.comports()]

class PortConfig(BaseModel):
    port: str

@app.post("/start")
def start(cfg: PortConfig):
    global running, ser, serial_port
    serial_port = cfg.port
    ser = serial.Serial(serial_port, BAUD_RATE, timeout=0.1)
    running = True
    return {"status": "started"}

@app.post("/stop")
def stop():
    global running, ser
    running = False
    if ser:
        ser.close()
    return {"status": "stopped"}

@app.post("/clear")
def clear():
    for group in buffers.values():
        for b in group:
            b.clear()
    return {"status": "cleared"}

@app.websocket("/ws/imu")
async def imu_ws(ws: WebSocket):
    await ws.accept()
    while True:
        await ws.send_json({
            "acc": [list(b) for b in buffers["acc"]],
            "gyro": [list(b) for b in buffers["gyro"]],
            "angles": [list(b) for b in buffers["angles"]],
        })
        await asyncio.sleep(0.05)
