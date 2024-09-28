#!/usr/bin/env python3
import numpy as np
import matplotlib.pylab as plt

data = np.genfromtxt("/tmp/imu_solve.csv", delimiter=",", skip_header=True)

time = data[:, 0] * 1e-9
gnd = data[:, 1:4]
init = data[:, 4:7]
est = data[:, 7:10]

plt.plot(gnd[:, 0], gnd[:, 1], "k--", label="Ground-Truth")
plt.plot(init[:, 0], init[:, 1], "r.", label="Initial")
plt.plot(est[:, 0], est[:, 1], "b-", label="Estimate")

plt.legend(loc=0)
plt.show()
