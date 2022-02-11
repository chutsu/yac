#!/usr/bin/env python3
import numpy as np
import pandas
import matplotlib.pylab as plt

m = 6
H = pandas.read_csv("/tmp/H.csv", header=None).to_numpy()
b = pandas.read_csv("/tmp/b.csv", header=None).to_numpy()

# Schur complement
H_mm = H[0:m, 0:m]
H_mr = H[0:m, m:]
H_rm = H[m:, 0:m]
H_rr = H[m:, m:]

b_mm = b[:m]
b_rr = b[m:]

H_mm = 0.5 * (H_mm + H_mm.T)
w, V = np.linalg.eig(H_mm)
for idx, w_i in enumerate(w):
  if w_i < 1e-12:
    w[idx] = 0.0

Lambda_inv = np.diag(1.0 / w)
H_mm_inv = V @ Lambda_inv @ V.T
H_marg = H_rr - (H_rm @ H_mm_inv @ H_mr)
b_marg = b_rr - (H_rm @ H_mm_inv @ b_mm)
# print(((H_mm @ H_mm_inv) - np.eye(m, m)).sum())

# Decompose H_marg into JtJ
w, V = np.linalg.eig(H_marg)
for idx, w_i in enumerate(w):
  if w_i < 1e-12:
    w[idx] = 0.0

S_sqrt = np.diag(w**0.5)
J = S_sqrt @ V.T
# print(np.max(((J.T @ J) - H_marg)))
# plt.imshow(J.T @ J - H_marg)
# plt.colorbar()
# plt.show()

# plt.imshow(H)
# plt.colorbar()
# plt.show()
