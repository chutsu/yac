#!/usr/bin/env python3
import numpy as np
import matplotlib.pylab as plt

sim_data = np.genfromtxt("/tmp/sim.csv", delimiter=",", skip_header=True)

time = sim_data[:, 0] * 1e-9
pos = sim_data[:, 1:4]
vel = sim_data[:, 8:11]
acc = sim_data[:, 11:14]
gyr = sim_data[:, 14:17]

# Plot X-Y
plt.figure()
plt.plot(pos[:, 0], pos[:, 1], "k--", label="Ground-Truth")
plt.xlabel("x [m]")
plt.ylabel("y [m]")

# Plot Velocity
plt.figure()
plt.plot(time, vel[:, 0], "r-")
plt.plot(time, vel[:, 1], "g-")
plt.plot(time, vel[:, 2], "b-")
plt.xlabel("Timestamps [s]")
plt.ylabel("Velocity m/s")
plt.xlim([0, np.max(time)])

# Plot Acc
plt.figure()
plt.plot(time, acc[:, 0], "r-")
plt.plot(time, acc[:, 1], "g-")
plt.plot(time, acc[:, 2], "b-")
plt.xlabel("Timestamps [s]")
plt.ylabel("Acc [ms^-2]")
plt.xlim([0, np.max(time)])

# Plot Gyro
plt.figure()
plt.plot(time, gyr[:, 0], "r-")
plt.plot(time, gyr[:, 1], "g-")
plt.plot(time, gyr[:, 2], "b-")
plt.xlabel("Timestamps [s]")
plt.ylabel("AngVel [rad s^-1]")
plt.xlim([0, np.max(time)])

plt.show()
