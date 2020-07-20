# PROBLEM 1
#
# Modify the orbit function below to model
# one revolution of the moon around the earth,
# assuming that the orbit is circular.
#
# Use the math.cos(angle) and math.sin(angle) 
# functions in order to accomplish this.

import math
import matplotlib.pyplot as plt
import numpy

moon_distance = 384e6 # m

def orbit():
    num_steps = 50
    x = numpy.zeros([num_steps + 1, 2])
    
    for i in range(num_steps + 1):
        temp = (2 * i * (math.pi)) / (num_steps) 
        x[i][0] = moon_distance * math.cos(temp)
        x[i][1] = moon_distance * math.sin(temp)
        
    return x

x = orbit()

def plot_me():
    plt.axis('equal')
    plt.plot(x[:, 0], x[:, 1])
    axes = plt.gca()
    axes.set_xlabel('Longitudinal position in m')
    axes.set_ylabel('Lateral position in m')

plot_me()

