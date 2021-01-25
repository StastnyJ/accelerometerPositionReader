#!/usr/bin/python3.7
import serial
import time
import datetime as dt
import matplotlib.pyplot as plt
import matplotlib.animation as animation


z1baudrate = 115200
z1port = '/dev/ttyUSB0' 

z1serial = serial.Serial(port=z1port, baudrate=z1baudrate)
z1serial.timeout = 2
print(z1serial.is_open)


fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
bx = fig.add_subplot(1,1,1)
cx = fig.add_subplot(1,1,1)

xs = [0]
ys = [[0],[1],[2]]


def animate(i, xs, ys):
    size = z1serial.inWaiting()
    if size:
        data = str(z1serial.read(size)).replace("b", "").replace("'", "")
        vals = [x.split(',') for x in data.split("\\n") if len(x) > 0]       
        vals = vals[1:-1]
        xs += [xs[-1] + i + 1 for i in range(len(vals))]
        for i in range(3):
            ys[i] += [float(v[i]) for v in vals]
    else:
        print('no data')

    xs = xs[-1000:]
    ys = [y[-1000:] for y in ys]

    ax.clear()
    ax.plot(xs, ys[0])
    ax.plot(xs, ys[1])
    ax.plot(xs, ys[2])

    plt.subplots_adjust(bottom=0.30)
    # plt.ylim([-10, 10])
    plt.ylim([-2,2])
    plt.title('Plot')
    plt.ylabel('acceleration (multiplies of g)')

ani = animation.FuncAnimation(fig, animate, fargs=(xs, ys), interval=300)
plt.show()