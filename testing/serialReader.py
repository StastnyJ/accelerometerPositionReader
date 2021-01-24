#!/usr/bin/python3.7
import serial
import time

z1baudrate = 115200
z1port = '/dev/ttyUSB0' 

z1serial = serial.Serial(port=z1port, baudrate=z1baudrate)
z1serial.timeout = 2    
print(z1serial.is_open) 



import datetime as dt
import matplotlib.pyplot as plt
import matplotlib.animation as animation

fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
xs = [0]
ys = [0]


def animate(i, xs, ys):
    size = z1serial.inWaiting()
    if size:
        data = str(z1serial.read(size)).replace("b'", "")
        vals = [float(x.replace("A: ", "").split(', ')[0]) for x in data.split("\r\n") if len(x) > 0]
        xs += [xs[-1] + i + 1 for i in range(len(vals))]
        ys += vals
    else:
        print('no data')

    xs = xs[-1000:]
    ys = ys[-1000:]

    ax.clear()
    ax.plot(xs, ys)

    plt.subplots_adjust(bottom=0.30)
    plt.ylim([-2, 2])
    plt.title('Plot')
    plt.ylabel('acceleration (multiplies of g)')

ani = animation.FuncAnimation(fig, animate, fargs=(xs, ys), interval=300)
plt.show()