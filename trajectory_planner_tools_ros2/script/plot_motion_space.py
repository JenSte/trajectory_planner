#!/usr/bin/env python3

import itertools
import math

import matplotlib.pyplot as plt
import numpy as np

# Maximum wheel speed, in m/s.
vmax = 1

# Distance between the wheels, in m.
dist = 0.4

vstep = 0.01


lins = []
rots = []

#vls = np.arange(-vmax, vmax, vstep)
#vrs = np.arange(-vmax, vmax, vstep)
#for vl, vr in itertools.product(vls, vrs):
#    lin = (vr + vl) / 2.0
#    rot = (vr - vl) / dist
#
#    lins.append(lin)
#    rots.append(rot)

for vl in np.arange(-vmax, vmax + vstep, vstep):
    w1 = 2.0 / dist * (vl - math.copysign(vmax, vl))
    w2 = 2.0 / dist * (math.copysign(vmax, vl) - vl)

    lins.append(vl); lins.append(vl)
    rots.append(w1); rots.append(w2)

print(f"rot. vel. max: {2.0 * vmax / dist:.3f} rad/s")

plt.scatter(lins, rots)
plt.show()
