# PROBLEM 2
#
# Modify the trajectory function below to 
# plot the trajectory of several particles. 
# Each trajectory starts at the point (0,0) 
# given initial speed in the direction 
# specified by the angle. Use the Forward 
# Euler Method to accomplish this.

import math
import numpy
import matplotlib.pyplot as plt

h = 0.1 # s
g = 9.81 # m / s2
acceleration = numpy.array([0., -g])
initial_speed = 20. # m / s

def trajectory():
    angles = numpy.linspace(20., 70., 6)

    num_steps = 30
    x = numpy.zeros([num_steps + 1, 2])
    v = numpy.zeros([num_steps + 1, 2])

    for angle in angles:
        v[0, 0] = initial_speed * math.cos(angle * math.pi/180)
        v[0, 1] = initial_speed * math.sin(angle * math.pi/180)
        
        for i in range(num_steps):
            v[i + 1] = v[i] + acceleration * h
            x[i + 1] = x[i] + v[i] * h

        plt.plot(x[:, 0], x[:, 1])
    
    plt.axis('equal')
    axes = plt.gca()
    axes.set_xlabel('Horizontal position in m')
    axes.set_ylabel('Vertical position in m')
    return x, v

trajectory()


