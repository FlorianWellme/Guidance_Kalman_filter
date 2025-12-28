"""
IMU Serial Reader
Handles serial communication with Arduino
"""

import serial
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Optional, Callable

@dataclass
class IMUData:
    """Structure to hold IMU sensor data"""
    # Raw accelerometer
    ax: float = 0.0
    ay: float = 0.0
    az: float = 0.0
    
    # Raw gyroscope (deg/s)
    gx: float = 0.0
    gy: float = 0.0
    gz: float = 0.0
    
    # Angles before Kalman
    roll_acc: float = 0.0
    pitch_acc: float = 0.0
    yaw_gyro: float = 0.0
    
    # Angles after Kalman
    roll_kalman: float = 0.0
    pitch_kalman: float = 0.0
    yaw_kalman: float = 0.0
    
    # Timestamp
    timestamp: float = 0.0

class IMUSerialReader:
    def __init__(self, port: str, baudrate: int = 115200, max_points: int = 200):
        self.port = port
        self.baudrate = baudrate
        self.max_points = max_points
        
        self.serial_port: Optional[serial.Serial] = None
        self.running = False
        self.connected = False
        self.thread: Optional[threading.Thread] = None
        
        # Data buffers
        self.data_buffer = deque(maxlen=max_points)
        self.current_data = IMUData()
        
        # Lock for thread-safe access
        self.lock = threading.Lock()
        
        # Callback for data updates
        self.data_callback: Optional[Callable] = None
        
    def connect(self) -> bool:
        """Attempt to connect to the serial port"""
        try:
            self.serial_port = serial.Serial(self.port, self.baudrate, timeout=0.1)
            time.sleep(2)  # Wait for Arduino reset
            
            # Clear initial buffer
            self.serial_port.reset_input_buffer()
            
            self.connected = True
            return True
        except Exception as e:
            print(f"Connection error: {e}")
            self.connected = False
            return False
    
    def disconnect(self):
        """Disconnect from serial port"""
        self.connected = False
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
    
    def start(self):
        """Start reading data in a separate thread"""
        if not self.connected and not self.connect():
            return False
        
        self.running = True
        self.thread = threading.Thread(target=self._read_loop, daemon=True)
        self.thread.start()
        return True
    
    def stop(self):
        """Stop reading data"""
        self.running = False
        if self.thread:
            self.thread.join(timeout=1)
        self.disconnect()
    
    def _read_loop(self):
        """Main reading loop (runs in separate thread)"""
        skip_header = True
        
        while self.running:
            if not self.serial_port or not self.serial_port.is_open:
                break
            
            try:
                if self.serial_port.in_waiting:
                    line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                    
                    # Skip header line
                    if skip_header:
                        skip_header = False
                        continue
                    
                    # Parse data
                    try:
                        values = list(map(float, line.split(',')))
                        if len(values) == 12:
                            data = IMUData(
                                ax=values[0],
                                ay=values[1],
                                az=values[2],
                                gx=values[3],
                                gy=values[4],
                                gz=values[5],
                                roll_acc=values[6],
                                pitch_acc=values[7],
                                yaw_gyro=values[8],
                                roll_kalman=values[9],
                                pitch_kalman=values[10],
                                yaw_kalman=values[11],
                                timestamp=time.time()
                            )
                            
                            with self.lock:
                                self.current_data = data
                                self.data_buffer.append(data)
                            
                            # Call callback if set
                            if self.data_callback:
                                self.data_callback(data)
                                
                    except ValueError:
                        pass  # Skip malformed lines
                        
            except Exception as e:
                print(f"Read error: {e}")
                time.sleep(0.01)
    
    def get_current_data(self) -> IMUData:
        """Get the most recent data point"""
        with self.lock:
            return self.current_data
    
    def get_buffer_data(self) -> list:
        """Get all buffered data"""
        with self.lock:
            return list(self.data_buffer)
    
    def set_data_callback(self, callback: Callable):
        """Set callback function for new data"""
        self.data_callback = callback