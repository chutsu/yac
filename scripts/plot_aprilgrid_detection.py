#!/usr/bin/env python3
import pandas
import seaborn as sns
import matplotlib.pylab as plt

# Load data
data_full = pandas.read_csv("/tmp/aprilgrid_detect-pthreads_fullsize.csv")
data_50 = pandas.read_csv("/tmp/aprilgrid_detect-pthreads_resized_50.csv")
data_75 = pandas.read_csv("/tmp/aprilgrid_detect-pthreads_resized_75.csv")

# Time series plots
plt.figure()
timestamps = data_full['timestamps'].to_numpy()
time = (timestamps - timestamps[0]) * 1e-9
# -- Plot time vs num_detections
plt.subplot(211)
plt.plot(time, data_full["timings"], label="Full-size")
plt.plot(time, data_75["timings"], label="564x360")
plt.plot(time, data_50["timings"], label="376x240")
plt.xlabel("Time [s]")
plt.ylabel("Timings [s]")
plt.title("Detection Times")
plt.legend(loc=0)
# -- Plot time vs timings
plt.subplot(212)
plt.plot(time, data_full["detections"], label="Full-size")
plt.plot(time, data_75["detections"], label="564x360")
plt.plot(time, data_50["detections"], label="376x240")
plt.xlabel("Time [s]")
plt.ylabel("Num of AprilTags Corners Detected")
plt.title("Num of AprilTag Corners Detected")
plt.legend(loc=0)

# Box plots
plt.figure()
# -- Timings
plt.subplot(211)
timings = [data_full["timings"], data_75["timings"], data_50["timings"]]
sns.boxplot(data=timings)
plt.xticks([0, 1, 2], ["Full-Size", "564x360", "376x240"])
plt.ylabel("Detection Time [s]")
plt.title("Detection Times")
# -- Corners detected
plt.subplot(212)
detections = [data_full["detections"], data_75["detections"], data_50["detections"]]
sns.boxplot(data=detections)
plt.xticks([0, 1, 2], ["Full-Size", "564x360", "376x240"])
plt.ylabel("Num of AprilTag Corners")
plt.title("Num of AprilTag Corners Detected")

plt.show()
