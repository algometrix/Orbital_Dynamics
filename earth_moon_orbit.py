

import math
from matplotlib import pyplot as plt
import numpy as np

def sin_cos():
    num_points = 50
    x = np.zeros(num_points)
    sin_x = np.zeros(num_points)
    cos_x = np.zeros(num_points)

    for i in range(num_points):
        x[i] = i * (2 * math.pi) / (num_points - 1)
        sin_x[i] = math.sin(x[i])
        cos_x[i] = math.cos(x[i])
    return x, sin_x, cos_x

def plot_graph(x, sin_x, cos_x):
    plt.style.use('ggplot')
    plt.plot(x, cos_x, label = "Moon X axis") # Moon X axis
    plt.plot(x, sin_x, label = "Moon Y axis") # Moon Y axis
    plt.show()

if __name__ == "__main__":
    x, sin_x, cos_x = sin_cos()
    plot_graph(x, sin_x, cos_x)

