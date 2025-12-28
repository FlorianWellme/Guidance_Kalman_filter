import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

# -------- Serial configuration --------
SERIAL_PORT = '/dev/cu.usbmodem142301'
BAUD_RATE = 115200
MAX_POINTS = 200

ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0)

# -------- Data buffers --------
acc_x = deque(maxlen=MAX_POINTS)
acc_y = deque(maxlen=MAX_POINTS)
acc_z = deque(maxlen=MAX_POINTS)

gyro_x = deque(maxlen=MAX_POINTS)
gyro_y = deque(maxlen=MAX_POINTS)
gyro_z = deque(maxlen=MAX_POINTS)

roll_acc = deque(maxlen=MAX_POINTS)
pitch_acc = deque(maxlen=MAX_POINTS)
yaw = deque(maxlen=MAX_POINTS)

roll_kal = deque(maxlen=MAX_POINTS)
pitch_kal = deque(maxlen=MAX_POINTS)

# -------- Create figure --------
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 6))

# -------- Accelerometer plot --------
l_ax, = ax1.plot([], [], label='Ax')
l_ay, = ax1.plot([], [], label='Ay')
l_az, = ax1.plot([], [], label='Az')
ax1.set_ylabel("Accel (raw)")
ax1.legend()
ax1.grid()

# -------- Gyroscope plot --------
l_gx, = ax2.plot([], [], label='Gx')
l_gy, = ax2.plot([], [], label='Gy')
l_gz, = ax2.plot([], [], label='Gz')
ax2.set_ylabel("Gyro (deg/s)")
ax2.legend()
ax2.grid()

# -------- Angle comparison plot --------
l_racc, = ax3.plot([], [], '--', label='Roll Acc')
l_rkal, = ax3.plot([], [], label='Roll Kalman')

l_pacc, = ax3.plot([], [], '--', label='Pitch Acc')
l_pkal, = ax3.plot([], [], label='Pitch Kalman')

l_yaw,  = ax3.plot([], [], ':', label='Yaw (gyro)')

ax3.set_ylabel("Angle (deg)")
ax3.set_xlabel("Samples")
ax3.legend(loc="upper right")
ax3.grid()

# -------- Fixed limits --------
ax1.set_xlim(0, MAX_POINTS)
ax1.set_ylim(-15000, 15000)

ax2.set_xlim(0, MAX_POINTS)
ax2.set_ylim(-800, 800)

ax3.set_xlim(0, MAX_POINTS)
ax3.set_ylim(-110, 110)

# -------- Read serial --------
def read_serial():
    while ser.in_waiting:
        line = ser.readline().decode(errors='ignore').strip()
        try:
            values = list(map(float, line.split(',')))
            if len(values) == 12:
                acc_x.append(values[0])
                acc_y.append(values[1])
                acc_z.append(values[2])

                gyro_x.append(values[3])
                gyro_y.append(values[4])
                gyro_z.append(values[5])

                roll_acc.append(values[6])
                pitch_acc.append(values[7])
                yaw.append(values[8])

                roll_kal.append(values[9])
                pitch_kal.append(values[10])
        except ValueError:
            pass

# -------- Update plot --------
def update(frame):
    read_serial()
    x = range(len(acc_x))

    l_ax.set_data(x, acc_x)
    l_ay.set_data(x, acc_y)
    l_az.set_data(x, acc_z)

    l_gx.set_data(x, gyro_x)
    l_gy.set_data(x, gyro_y)
    l_gz.set_data(x, gyro_z)

    l_racc.set_data(x, roll_acc)
    l_rkal.set_data(x, roll_kal)

    l_pacc.set_data(x, pitch_acc)
    l_pkal.set_data(x, pitch_kal)

    l_yaw.set_data(x, yaw)

    return (
        l_ax, l_ay, l_az,
        l_gx, l_gy, l_gz,
        l_racc, l_rkal,
        l_pacc, l_pkal,
        l_yaw
    )

# -------- Animation --------
ani = animation.FuncAnimation(
    fig,
    update,
    interval=20,
    blit=True
)

plt.show()
ser.close()
