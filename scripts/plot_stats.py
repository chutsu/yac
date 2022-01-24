""" Plot Stats """
#!/usr/bin/env python3
import numpy as np
import pandas
import matplotlib.pylab as plt
import matplotlib as mpl

# Load statistics
stats_data = pandas.read_csv("/tmp/calib-stats.csv")

# Residuals
rx = stats_data["rx"].to_numpy()
ry = stats_data["ry"].to_numpy()

# Reprojection Errors
reproj_errors = []
for i in range(len(rx)):
  r = np.array([rx[i], ry[i]])
  reproj_errors.append(np.linalg.norm(r))
reproj_errors = np.array(reproj_errors)
reproj_errors_max = np.max(reproj_errors)
reproj_errors_median = np.median(reproj_errors)
reproj_errors_mean = np.mean(reproj_errors)
reproj_errors_stddev = np.std(reproj_errors)

# Histogram - Residuals x-axis
plt.figure()
plt.hist(rx, bins=200)
plt.xlim([-1.0, 1.0])
plt.title("Residuals in x-axis")
plt.xlabel("Residual [px]")
plt.ylabel("Frequency")

# Histogram - Residuals y-axis
plt.figure()
plt.hist(ry, bins=200)
plt.xlim([-1.0, 1.0])
plt.title("Residuals in y-axis")
plt.xlabel("Residual [px]")
plt.ylabel("Frequency")

# Scatter XY Plot
plt.figure()

cmap = plt.cm.get_cmap("jet")
ax = plt.subplot(111)
ax.scatter(rx, ry, c=reproj_errors, cmap=cmap, vmax=2.0)

ax.grid(True)
ax.spines['left'].set_position('zero')
ax.spines['right'].set_color('none')
ax.spines['bottom'].set_position('zero')
ax.spines['top'].set_color('none')

# max_val = reproj_errors_stddev * 10.0
# max_val = reproj_errors_max
# max_val = 5.0
max_val = 3.0
ax.set_xlim([-max_val, max_val])
ax.set_ylim([-max_val, max_val])

plt.show()
