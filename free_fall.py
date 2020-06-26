
import numpy
import matplotlib.pyplot


def forward_euler():
    h = 0.1 # s
    g = 9.81 # m / s2

    num_steps = 50

    t = numpy.zeros(num_steps + 1)
    x = numpy.zeros(num_steps + 1)
    v = numpy.zeros(num_steps + 1)

    for step in range(num_steps):
        t[step + 1] = h + t[step]
        x[step + 1] = x[step] + h * v[step]
        v[step + 1] = v[step] - h * g
    return t, x, v

t, x, v = forward_euler()

def plot_me():
    axes_height = matplotlib.pyplot.subplot(211)
    matplotlib.pyplot.plot(t, x)
    axes_velocity = matplotlib.pyplot.subplot(212)
    matplotlib.pyplot.plot(t, v)
    axes_height.set_ylabel('Height in m')
    axes_velocity.set_ylabel('Velocity in m/s')
    axes_velocity.set_xlabel('Time in s')
    # Uncomment the line below when running locally.
    matplotlib.pyplot.show() 

plot_me()
