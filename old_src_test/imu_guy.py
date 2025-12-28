"""
IMU Visualizer GUI - Using matplotlib FuncAnimation for maximum speed
Modern interface with CustomTkinter
"""

import customtkinter as ctk
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import serial
import time

class IMUVisualizerGUI:
    def __init__(self, root):
        self.root = root
        
        # Serial connection
        self.serial_port = None
        self.is_connected = False
        
        # Configure grid
        self.root.grid_rowconfigure(0, weight=1)
        self.root.grid_columnconfigure(1, weight=1)
        
        # Create UI sections
        self.create_sidebar()
        self.create_main_content()
        
        # Animation object
        self.ani = None
        
        # FPS tracking
        self.last_time = time.time()
        self.frame_count = 0
        
    def create_sidebar(self):
        """Create left sidebar with controls and current values"""
        self.sidebar = ctk.CTkFrame(self.root, width=300, corner_radius=0)
        self.sidebar.grid(row=0, column=0, sticky="nsew", padx=0, pady=0)
        self.sidebar.grid_rowconfigure(8, weight=1)
        
        # Title
        title_label = ctk.CTkLabel(
            self.sidebar,
            text="IMU Monitor",
            font=ctk.CTkFont(size=24, weight="bold")
        )
        title_label.grid(row=0, column=0, padx=20, pady=(20, 10))
        
        # Connection status
        self.status_frame = ctk.CTkFrame(self.sidebar)
        self.status_frame.grid(row=1, column=0, padx=20, pady=10, sticky="ew")
        
        self.status_indicator = ctk.CTkLabel(
            self.status_frame,
            text="●",
            font=ctk.CTkFont(size=20),
            text_color="red"
        )
        self.status_indicator.grid(row=0, column=0, padx=(10, 5), pady=5)
        
        self.status_label = ctk.CTkLabel(
            self.status_frame,
            text="Disconnected",
            font=ctk.CTkFont(size=14)
        )
        self.status_label.grid(row=0, column=1, padx=(0, 10), pady=5)
        
        # Port entry
        port_label = ctk.CTkLabel(
            self.sidebar,
            text="Serial Port:",
            font=ctk.CTkFont(size=12)
        )
        port_label.grid(row=2, column=0, padx=20, pady=(10, 0), sticky="w")
        
        self.port_entry = ctk.CTkEntry(
            self.sidebar,
            placeholder_text="/dev/cu.usbmodem142301"
        )
        self.port_entry.insert(0, "/dev/cu.usbmodem142301")
        self.port_entry.grid(row=3, column=0, padx=20, pady=(0, 10), sticky="ew")
        
        # Connect/Disconnect button
        self.connect_btn = ctk.CTkButton(
            self.sidebar,
            text="Connect",
            command=self.toggle_connection,
            font=ctk.CTkFont(size=14, weight="bold"),
            height=40
        )
        self.connect_btn.grid(row=4, column=0, padx=20, pady=10, sticky="ew")
        
        # Separator
        separator1 = ctk.CTkFrame(self.sidebar, height=2, fg_color="gray30")
        separator1.grid(row=5, column=0, padx=20, pady=10, sticky="ew")
        
        # Current values section
        values_label = ctk.CTkLabel(
            self.sidebar,
            text="Current Orientation",
            font=ctk.CTkFont(size=16, weight="bold")
        )
        values_label.grid(row=6, column=0, padx=20, pady=(10, 5))
        
        # Value cards
        self.value_cards = {}
        angles = [
            ("Roll", "roll", "°"),
            ("Pitch", "pitch", "°"),
            ("Yaw", "yaw", "°")
        ]
        
        for i, (label, key, unit) in enumerate(angles):
            card = self.create_value_card(self.sidebar, label, unit)
            card['frame'].grid(row=7+i, column=0, padx=20, pady=5, sticky="ew")
            self.value_cards[key] = card
        
        # Separator
        separator2 = ctk.CTkFrame(self.sidebar, height=2, fg_color="gray30")
        separator2.grid(row=10, column=0, padx=20, pady=10, sticky="ew")
        
        # FPS Counter
        self.fps_label = ctk.CTkLabel(
            self.sidebar,
            text="FPS: 0",
            font=ctk.CTkFont(size=14, weight="bold"),
            text_color="#4ECDC4"
        )
        self.fps_label.grid(row=11, column=0, padx=20, pady=10)
        
        # Data rate
        self.data_rate_label = ctk.CTkLabel(
            self.sidebar,
            text="Data: 0 Hz",
            font=ctk.CTkFont(size=12),
            text_color="gray60"
        )
        self.data_rate_label.grid(row=12, column=0, padx=20, pady=(0, 10))
        
    def create_value_card(self, parent, label, unit):
        """Create a card to display a sensor value"""
        frame = ctk.CTkFrame(parent, fg_color="gray20", corner_radius=10)
        
        label_widget = ctk.CTkLabel(
            frame,
            text=label,
            font=ctk.CTkFont(size=12),
            text_color="gray60"
        )
        label_widget.grid(row=0, column=0, padx=15, pady=(10, 0), sticky="w")
        
        value_widget = ctk.CTkLabel(
            frame,
            text=f"0.00 {unit}",
            font=ctk.CTkFont(size=20, weight="bold")
        )
        value_widget.grid(row=1, column=0, padx=15, pady=(0, 10), sticky="w")
        
        return {"frame": frame, "label": label_widget, "value": value_widget, "unit": unit}
    
    def create_main_content(self):
        """Create main content area with plots"""
        self.main_frame = ctk.CTkFrame(self.root, corner_radius=0)
        self.main_frame.grid(row=0, column=1, sticky="nsew", padx=0, pady=0)
        self.main_frame.grid_rowconfigure(0, weight=1)
        self.main_frame.grid_columnconfigure(0, weight=1)
        
        # Data buffers - EXACTLY like original code
        self.max_points = 200
        self.acc_x = deque(maxlen=self.max_points)
        self.acc_y = deque(maxlen=self.max_points)
        self.acc_z = deque(maxlen=self.max_points)
        
        self.gyro_x = deque(maxlen=self.max_points)
        self.gyro_y = deque(maxlen=self.max_points)
        self.gyro_z = deque(maxlen=self.max_points)
        
        self.roll_acc = deque(maxlen=self.max_points)
        self.pitch_acc = deque(maxlen=self.max_points)
        self.yaw = deque(maxlen=self.max_points)
        
        self.roll_kal = deque(maxlen=self.max_points)
        self.pitch_kal = deque(maxlen=self.max_points)
        
        # Create matplotlib figure
        plt.style.use('dark_background')
        self.fig, (self.ax1, self.ax2, self.ax3) = plt.subplots(3, 1, figsize=(10, 6))
        self.fig.patch.set_facecolor('#1a1a1a')
        self.fig.subplots_adjust(left=0.08, right=0.95, top=0.95, bottom=0.08, hspace=0.3)
        
        # Accelerometer plot
        self.l_ax, = self.ax1.plot([], [], label='Ax', color='#FF6B6B', linewidth=1.5)
        self.l_ay, = self.ax1.plot([], [], label='Ay', color='#4ECDC4', linewidth=1.5)
        self.l_az, = self.ax1.plot([], [], label='Az', color='#95E1D3', linewidth=1.5)
        self.ax1.set_ylabel("Accel (raw)", color='white')
        self.ax1.set_title("Raw Accelerometer Data", color='white', pad=10)
        self.ax1.legend(loc='upper right', facecolor='#2a2a2a', edgecolor='gray')
        self.ax1.grid(True, alpha=0.2, color='gray')
        self.ax1.set_facecolor('#1a1a1a')
        self.ax1.set_xlim(0, self.max_points)
        self.ax1.set_ylim(-15000, 15000)
        self.ax1.tick_params(colors='white')
        
        # Gyroscope plot
        self.l_gx, = self.ax2.plot([], [], label='Gx', color='#FF6B6B', linewidth=1.5)
        self.l_gy, = self.ax2.plot([], [], label='Gy', color='#4ECDC4', linewidth=1.5)
        self.l_gz, = self.ax2.plot([], [], label='Gz', color='#95E1D3', linewidth=1.5)
        self.ax2.set_ylabel("Gyro (deg/s)", color='white')
        self.ax2.set_title("Raw Gyroscope Data", color='white', pad=10)
        self.ax2.legend(loc='upper right', facecolor='#2a2a2a', edgecolor='gray')
        self.ax2.grid(True, alpha=0.2, color='gray')
        self.ax2.set_facecolor('#1a1a1a')
        self.ax2.set_xlim(0, self.max_points)
        self.ax2.set_ylim(-800, 800)
        self.ax2.tick_params(colors='white')
        
        # Angle comparison plot
        self.l_racc, = self.ax3.plot([], [], '--', label='Roll Acc', color='#FF6B6B', linewidth=1, alpha=0.5)
        self.l_rkal, = self.ax3.plot([], [], label='Roll Kalman', color='#FF6B6B', linewidth=2)
        self.l_pacc, = self.ax3.plot([], [], '--', label='Pitch Acc', color='#4ECDC4', linewidth=1, alpha=0.5)
        self.l_pkal, = self.ax3.plot([], [], label='Pitch Kalman', color='#4ECDC4', linewidth=2)
        self.l_yaw, = self.ax3.plot([], [], ':', label='Yaw (gyro)', color='#95E1D3', linewidth=2)
        self.ax3.set_ylabel("Angle (deg)", color='white')
        self.ax3.set_xlabel("Samples", color='white')
        self.ax3.set_title("Angle Estimation (Kalman Filter)", color='white', pad=10)
        self.ax3.legend(loc="upper left", ncol=2, facecolor='#2a2a2a', edgecolor='gray')
        self.ax3.grid(True, alpha=0.2, color='gray')
        self.ax3.set_facecolor('#1a1a1a')
        self.ax3.set_xlim(0, self.max_points)
        self.ax3.set_ylim(-110, 110)
        self.ax3.tick_params(colors='white')
        
        # Embed figure in tkinter
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.main_frame)
        self.canvas.draw()
        canvas_widget = self.canvas.get_tk_widget()
        canvas_widget.grid(row=0, column=0, sticky="nsew", padx=10, pady=10)
        
        # Counter for updating value cards
        self.update_counter = 0
        self.data_counter = 0
        self.last_data_time = time.time()
        
    def toggle_connection(self):
        """Toggle serial connection"""
        if self.is_connected:
            self.disconnect()
        else:
            self.connect()
    
    def connect(self):
        """Connect to serial port"""
        port = self.port_entry.get()
        try:
            self.serial_port = serial.Serial(port, 115200, timeout=0)
            time.sleep(2)  # Wait for Arduino reset
            self.serial_port.reset_input_buffer()
            
            self.is_connected = True
            
            self.connect_btn.configure(text="Disconnect")
            self.status_indicator.configure(text_color="lime")
            self.status_label.configure(text="Connected")
            
            # Start FuncAnimation - EXACTLY like original code
            self.ani = animation.FuncAnimation(
                self.fig,
                self.update,
                interval=20,  # 20ms = 50 FPS
                blit=True,
                cache_frame_data=False
            )
            
        except Exception as e:
            self.show_error(f"Connection failed: {e}")
    
    def disconnect(self):
        """Disconnect from serial port"""
        self.is_connected = False
        
        # Stop animation
        if self.ani:
            self.ani.event_source.stop()
            self.ani = None
        
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        
        self.connect_btn.configure(text="Connect")
        self.status_indicator.configure(text_color="red")
        self.status_label.configure(text="Disconnected")
    
    def read_serial(self):
        """Read serial data - EXACTLY like original code"""
        if not self.serial_port or not self.serial_port.is_open:
            return
        
        # Read ALL available data in buffer
        while self.serial_port.in_waiting:
            try:
                line = self.serial_port.readline().decode(errors='ignore').strip()
                
                # Skip header
                if 'ax,ay,az' in line.lower():
                    continue
                
                # Parse data
                values = list(map(float, line.split(',')))
                if len(values) == 12:
                    self.acc_x.append(values[0])
                    self.acc_y.append(values[1])
                    self.acc_z.append(values[2])
                    
                    self.gyro_x.append(values[3])
                    self.gyro_y.append(values[4])
                    self.gyro_z.append(values[5])
                    
                    self.roll_acc.append(values[6])
                    self.pitch_acc.append(values[7])
                    self.yaw.append(values[8])
                    
                    self.roll_kal.append(values[9])
                    self.pitch_kal.append(values[10])
                    
                    # Count data rate
                    self.data_counter += 1
                    
            except ValueError:
                pass
    
    def update(self, frame):
        """Update function for FuncAnimation - EXACTLY like original code"""
        # Read all available serial data
        self.read_serial()
        
        # Update plot data
        x = range(len(self.acc_x))
        
        self.l_ax.set_data(x, self.acc_x)
        self.l_ay.set_data(x, self.acc_y)
        self.l_az.set_data(x, self.acc_z)
        
        self.l_gx.set_data(x, self.gyro_x)
        self.l_gy.set_data(x, self.gyro_y)
        self.l_gz.set_data(x, self.gyro_z)
        
        self.l_racc.set_data(x, self.roll_acc)
        self.l_rkal.set_data(x, self.roll_kal)
        
        self.l_pacc.set_data(x, self.pitch_acc)
        self.l_pkal.set_data(x, self.pitch_kal)
        
        self.l_yaw.set_data(x, self.yaw)
        
        # Update value cards every 10 frames
        self.update_counter += 1
        if self.update_counter % 10 == 0:
            if len(self.roll_kal) > 0:
                self.value_cards['roll']['value'].configure(
                    text=f"{self.roll_kal[-1]:.2f}°"
                )
            if len(self.pitch_kal) > 0:
                self.value_cards['pitch']['value'].configure(
                    text=f"{self.pitch_kal[-1]:.2f}°"
                )
            if len(self.yaw) > 0:
                self.value_cards['yaw']['value'].configure(
                    text=f"{self.yaw[-1]:.2f}°"
                )
        
        # Update FPS every 30 frames
        if self.update_counter % 30 == 0:
            current_time = time.time()
            elapsed = current_time - self.last_time
            if elapsed > 0:
                fps = 30 / elapsed
                self.fps_label.configure(text=f"FPS: {fps:.1f}")
            self.last_time = current_time
            
            # Update data rate
            data_elapsed = current_time - self.last_data_time
            if data_elapsed > 0:
                data_rate = self.data_counter / data_elapsed
                self.data_rate_label.configure(text=f"Data: {data_rate:.0f} Hz")
                self.data_counter = 0
                self.last_data_time = current_time
        
        # Return all artists for blitting
        return (
            self.l_ax, self.l_ay, self.l_az,
            self.l_gx, self.l_gy, self.l_gz,
            self.l_racc, self.l_rkal,
            self.l_pacc, self.l_pkal,
            self.l_yaw
        )
    
    def show_error(self, message):
        """Show error dialog"""
        dialog = ctk.CTkToplevel(self.root)
        dialog.title("Error")
        dialog.geometry("400x150")
        
        label = ctk.CTkLabel(
            dialog,
            text=message,
            font=ctk.CTkFont(size=14)
        )
        label.pack(pady=30)
        
        button = ctk.CTkButton(
            dialog,
            text="OK",
            command=dialog.destroy
        )
        button.pack(pady=10)