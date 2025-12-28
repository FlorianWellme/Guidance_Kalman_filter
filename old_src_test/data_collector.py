import serial
from collections import deque
from threading import Thread, Lock
import time


class IMUDataCollector:
    """Handles serial communication and data buffering for IMU sensors."""

    def __init__(self, port='/dev/cu.usbmodem142301', baud_rate=115200, max_points=200):
        self.port = port
        self.baud_rate = baud_rate
        self.max_points = max_points

        # Data buffers
        self.acc_x = deque(maxlen=max_points)
        self.acc_y = deque(maxlen=max_points)
        self.acc_z = deque(maxlen=max_points)

        self.gyro_x = deque(maxlen=max_points)
        self.gyro_y = deque(maxlen=max_points)
        self.gyro_z = deque(maxlen=max_points)

        self.roll_acc = deque(maxlen=max_points)
        self.pitch_acc = deque(maxlen=max_points)
        self.yaw = deque(maxlen=max_points)

        self.roll_kal = deque(maxlen=max_points)
        self.pitch_kal = deque(maxlen=max_points)

        self.running = False
        self.thread = None
        self.lock = Lock()
        self.ser = None

    def connect(self):
        """Establish serial connection."""
        try:
            self.ser = serial.Serial(
                self.port,
                self.baud_rate,
                timeout=0.02
            )
            return True
        except Exception as e:
            print(f"[IMU] Serial connection error: {e}")
            self.ser = None
            return False

    def start(self):
        """Start data collection thread."""
        if self.running:
            return True

        if not self.ser:
            if not self.connect():
                return False

        self.running = True
        self.thread = Thread(target=self._read_loop, daemon=True)
        self.thread.start()
        return True

    def stop(self):
        """Stop data collection thread."""
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=0.5)
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.thread = None

    def _read_loop(self):
        """Continuously read complete lines from serial port."""
        while self.running:
            try:
                if not self.ser or not self.ser.is_open:
                    if not self.connect():
                        time.sleep(0.1)
                        continue

                line = self.ser.readline().decode(errors='ignore').strip()
                if line:
                    self._parse_line(line)
            except Exception as e:
                print(f"[IMU] Read error: {e}")
                time.sleep(0.1)

    def _parse_line(self, line):
        """Parse incoming serial data line."""
        try:
            values = list(map(float, line.split(',')))

            if len(values) != 12:
                return

            with self.lock:
                # Accelerometer
                self.acc_x.append(values[0])
                self.acc_y.append(values[1])
                self.acc_z.append(values[2])

                # Gyroscope
                self.gyro_x.append(values[3])
                self.gyro_y.append(values[4])
                self.gyro_z.append(values[5])

                # Angles from accelerometer / gyro
                self.roll_acc.append(values[6])
                self.pitch_acc.append(values[7])
                self.yaw.append(values[8])

                # Kalman filtered angles
                self.roll_kal.append(values[9])
                self.pitch_kal.append(values[10])

        except ValueError:
            pass

    def get_data(self):
        """Return current data snapshot (thread-safe)."""
        with self.lock:
            return {
                'acc': (
                    list(self.acc_x),
                    list(self.acc_y),
                    list(self.acc_z)
                ),
                'gyro': (
                    list(self.gyro_x),
                    list(self.gyro_y),
                    list(self.gyro_z)
                ),
                'angles': {
                    'roll_acc': list(self.roll_acc),
                    'pitch_acc': list(self.pitch_acc),
                    'roll_kal': list(self.roll_kal),
                    'pitch_kal': list(self.pitch_kal),
                    'yaw': list(self.yaw)
                }
            }

    def clear_buffers(self):
        """Clear all data buffers."""
        with self.lock:
            self.acc_x.clear()
            self.acc_y.clear()
            self.acc_z.clear()
            self.gyro_x.clear()
            self.gyro_y.clear()
            self.gyro_z.clear()
            self.roll_acc.clear()
            self.pitch_acc.clear()
            self.roll_kal.clear()
            self.pitch_kal.clear()
            self.yaw.clear()

