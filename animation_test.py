import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

fig, ax = plt.subplots()
xdata, ydata = [], []
xdatab, ydatab = [], []
ln, = plt.plot([], [], 'ro')
ln2, = plt.plot([], [], 'bo')

def init():
    ax.set_xlim(0, 2*np.pi)
    ax.set_ylim(-1, 1)
    return ln,

def update(frame):
    xdata.append(frame)
    ydata.append(np.sin(frame))
    xdatab.append(frame+0.1)
    ydatab.append(np.sin(frame))
    ln.set_data(xdata, ydata)
    ln2.set_data(xdatab, ydatab)

    return ln, ln2

ani = FuncAnimation(fig, update, frames=np.linspace(0, 2*np.pi, 128),
                    init_func=init, blit=True)
plt.show()