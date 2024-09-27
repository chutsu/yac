#!/usr/bin/env python3
import argparse
import numpy as np
import matplotlib.pylab as plt

if __name__ == "__main__":
  parser = argparse.ArgumentParser(prog='compare_matrices.py')
  parser.add_argument('--mata', required=True)
  parser.add_argument('--matb', required=True)
  args = parser.parse_args()

  A = np.genfromtxt(args.mata, delimiter=",")
  B = np.genfromtxt(args.matb, delimiter=",")
  diff = A - B

  plt.subplot(131)
  plt.imshow(A)
  plt.title("A")
  plt.colorbar()

  plt.subplot(132)
  plt.imshow(B)
  plt.title("B")
  plt.colorbar()

  plt.subplot(133)
  plt.imshow(diff)
  plt.colorbar()
  plt.title("(A - B)")

  plt.show()
