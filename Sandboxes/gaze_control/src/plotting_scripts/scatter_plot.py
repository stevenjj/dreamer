#!/usr/bin/env python
import rospy

import matplotlib.pyplot as plt
from numpy.random import rand


fig, ax = plt.subplots()
for color in ['red', 'green', 'blue']:
    n = 750
    x, y = rand(2, n)
    scale = 20.0 * rand(n)
    ax.scatter(x, y, c=color, s=scale, label=color,
               alpha=0.75, edgecolors='none')

ax.legend()
ax.grid(True)

plt.show()