import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation

PLOT_LENGTH = 5000

serialInst = serial.Serial()
serialInst.baudrate = 115200
serialInst.port = "/dev/cu.usbserial-0001" #for example, "/dev/cu.usbserial-0001" or "/dev/ttyUSB0"

serialInst.open()

data_dict = {}
x_data=[]
y_data=[]

fig, ax = plt.subplots(figsize=(12, 6))
line, = ax.plot(x_data, x_data, 'o', markersize = 2)
ax.set_ylim(0, 10000)
ax.set_xlim(0, PLOT_LENGTH)
ax.set_xlabel('Frequencies')
ax.set_ylabel('Amplitude')
ax.grid(True)

def update(fragment):
    while serialInst.in_waiting:
        try:
            packed = serialInst.readline()
            new_packed = packed.decode('ascii', errors='ignore').strip()
            if new_packed:
                parts = new_packed.split(',')
                if len(parts) == 2:
                    x_value = float(parts[0])
                    y_value = float(parts[1])
                    data_dict[x_value] = y_value
        except (ValueError, UnicodeDecodeError) as e:
            continue

    x_data = list(data_dict.keys())
    y_data = list(data_dict.values())

    line.set_data(x_data, y_data)
    return line,

ani = animation.FuncAnimation(fig, update, interval=1, blit=True, cache_frame_data=False)
plt.tight_layout()
plt.show()
