# -*- coding: utf-8 -*-

import numpy as np
import pylab as pl
f = open("../out.txt")
x = []
y = []
for line in f:
    info = line.split()
    y.append(float(info[0]))
    x.append(float(info[1]))

results = {}
coeffs = np.polyfit(x, y, 2)
results = coeffs.tolist()
p = np.poly1d(coeffs)                  
pl.plot(x,y,'o',x,p(x))
pl.show()
