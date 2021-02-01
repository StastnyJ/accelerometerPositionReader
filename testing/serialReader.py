#!/usr/bin/python3.7
import serial
import time
import datetime as dt
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import sys

n = 1
yLimMin = -100.0
yLimMax = 100.0

ax = None
z1baudrate = 115200
z1port = '/dev/ttyUSB0' 

def animate(i, xs, ys):
    size = z1serial.inWaiting()
    if size:
        data = str(z1serial.read(size)).replace("b", "").replace("'", "")
        vals = [x.split(',') for x in data.split("\\n") if len(x) > 0]       
        vals = vals[1:-1]
        xs += [xs[-1] + i + 1 for i in range(len(vals))]
        for i in range(n):
            ys[i] += [float(v[i]) for v in vals]

    xs = xs[-1000:]
    ys = [y[-1000:] for y in ys]

    ax.clear()
    for i in range(n):
        ax.plot(xs, ys[i])

    plt.subplots_adjust(bottom=0.30)
    plt.ylim([yLimMin, yLimMax])
    plt.title('Plot')
    plt.ylabel('sensor values')

if __name__ == '__main__':
    if len(sys.argv) > 1:
        n = int(sys.argv[1])
    if len(sys.argv) == 3:
        yLimMin = -float(sys.argv[2])
        yLimMax = float(sys.argv[2])
    if len(sys.argv) > 3:
        yLimMin = float(sys.argv[2])
        yLimMax = float(sys.argv[3])

    z1serial = serial.Serial(port=z1port, baudrate=z1baudrate)
    z1serial.timeout = 2
    if not z1serial.is_open:
        print("Fail to connect")
        sys.exit()

    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)

    xs = [0]
    ys = [[0] for _ in range(n)]

    ani = animation.FuncAnimation(fig, animate, fargs=(xs, ys), interval=300)
    plt.show()