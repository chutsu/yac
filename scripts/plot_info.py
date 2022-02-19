""" Plot INFO """
import pandas
import numpy as np
from numpy.linalg import inv
import matplotlib.pylab as plt
from mpl_toolkits import mplot3d

all_views_csv = "/data/euroc_results/experiments/kalibr_data/data/timestamps.csv"
kalibr_views_csv = "/data/euroc_results/experiments/kalibr_data/data/views.csv"
kalibr_info_csv = "/data/euroc_results/experiments/kalibr_data/data/info.csv"
yac_views_csv = "/tmp/yac_views.csv"
yac_info_csv = "/tmp/yac_info.csv"

# Load all views
all_views = {}
for idx, row in pandas.read_csv(all_views_csv, header=None).iterrows():
  ts = int(row[0] * 1e6)
  all_views[ts] = idx

# Load kalibr views
kalibr_info = []
kalibr_info_gain = []
for idx, row in pandas.read_csv(kalibr_info_csv, header=None).iterrows():
  ts = int(row[0] * 1e6)
  info = row[2]
  info_gain = row[3]
  kalibr_info.append(-1.0 * info)
  kalibr_info_gain.append(info_gain)

# Load Kalibr info
kalibr_views = []
for idx, row in pandas.read_csv(kalibr_views_csv, header=None).iterrows():
  ts = int(row[0] * 1e6)
  kalibr_views.append(all_views[ts])

# Load YAC views
yac_views = []
for idx, row in pandas.read_csv(yac_views_csv, header=None).iterrows():
  ts = row[0]
  yac_views.append(all_views[ts])

# Load YAC info
yac_info = []
yac_info_gain = []
for idx, row in pandas.read_csv(yac_info_csv, header=None).iterrows():
  ts = row[0]
  info = row[1]
  info_gain = row[3]
  yac_info.append(info)
  yac_info_gain.append(info_gain)

# Plot NBVs
lo = [0, 1.0]
c = ["red", "blue"]
plt.figure()
ax = plt.subplot(111)
ax.eventplot([kalibr_views, yac_views], lineoffsets=lo, color=c)
ax.set_xlabel("View Index")
ax.set_yticks(lo)
ax.set_yticklabels(["Kalibr", "YAC"])

# Plot information gain over time
plt.figure()
ax = plt.subplot(111)
ax.plot(yac_info, label="YAC")
ax.plot(kalibr_info, label="Kalibr")
ax.set_xlabel("View Index")
ax.legend(loc=0)
ax.set_title("sum(log(S)) / log(2)")
plt.show()

# from numpy import genfromtxt
# H_marg = genfromtxt('/tmp/H_marg.csv', delimiter=',')
#
# # Decompose H_marg into JtJ
# w, V = np.linalg.eig(H_marg)
# for idx, w_i in enumerate(w):
#   if w_i < 1e-12:
#     w[idx] = 0.0
# S_sqrt = np.diag(w**0.5)
# J = S_sqrt @ V.T
#
# # SVD H
# u, s, vh = np.linalg.svd(H_marg)
#
# plt.figure()
# plt.imshow(u - V)
# plt.colorbar()
#
# print(np.log(w).sum())
# print(np.log(s).sum())
#
# plt.figure()
# plt.imshow(vh.T - V)
# plt.colorbar()
#
# # plt.figure()
# # plt.imshow((u - vh.T))
# # plt.colorbar()
#
# plt.show()

# covar = np.linalg.inv(H_marg)
# w, V = np.linalg.eig(H_marg)

# print(f"S.log().sum(): {-1.0 * np.sum(np.log(w))}")

# print(f"det(covar): {np.linalg.det(covar)}")
# print(f"S.log().sum(): {-1.0 * np.sum(np.log(s))}")

# plt.imshow((J.T @ J) - H_marg)
# plt.colorbar()
# plt.show()
