""" Sandbox """
#!/usr/bin/env python3
import numpy as np
import matplotlib.pylab as plt
import scipy
from scipy.stats import chi2


def symdiff():
  """ Symbolic diff """
  from sympy import exp
  from sympy import log
  from sympy import sqrt
  from sympy import symbols
  from sympy import diff

  s = symbols("s")
  k = symbols("k")
  epsilon = symbols("epsilon")

  # Huber Loss
  loss = 2 * k * sqrt(s) - k**2
  loss_deriv = diff(loss, s)
  loss_dderiv = diff(loss_deriv, s)

  print("Huber loss:")
  print(loss)
  print(loss_deriv)
  print(loss_dderiv)

  # Blake-Zisserman Loss
  loss = -log(exp(-1.0 * s) + epsilon)
  loss_deriv = diff(loss, s)
  loss_dderiv = diff(loss_deriv, s)

  print("Blake-Zisserman loss:")
  print(loss)
  print(loss_deriv)
  print(loss_dderiv)


class HuberLoss:
  """HuberLoss"""

  def __init__(self, k):
    self.k = k

  def eval(self, s):
    """ Evaluate """
    k = self.k
    r = np.sqrt(s)
    rho = None
    rho_prime = None
    rho_pprime = None

    if s > k**2:
      # Outlier
      rho = 2.0 * k * r - k**2
      rho_prime = k / np.sqrt(s)
      rho_pprime = -1.0 * rho_prime / (2.0 * s)
    else:
      # Inlier
      rho = s
      rho_prime = 1.0
      rho_pprime = 0.0

    return (rho, rho_prime, rho_pprime)


class BlakeZissermanLoss:
  """BlakeZissermanLoss"""

  def __init__(self, eps):
    self.eps = eps

  def eval(self, s):
    """ Evaluate """
    eps = self.eps

    rho = -np.log(eps + np.exp(-1.0 * s))
    rho_prime = 1.0 / (np.exp(s) + eps + 1.0)
    rho_pprime = (np.exp(s) + eps + 1.0) / (np.exp(s) + eps + 1.0)**2

    return (rho, rho_prime, rho_pprime)


def test_huber_loss():
  """ Test huber loss """
  k = 1.0
  loss = HuberLoss(k)

  residual_vals = []
  cost_vals = []

  for r in np.linspace(-5.0, 5.0, 200):
    s = r * r
    (rho, rho_prime, rho_pprime) = loss.eval(s)
    residual_vals.append(r)
    cost_vals.append(rho)

  plt.plot(residual_vals, cost_vals, "r-", label="Huber Loss")
  plt.show()


def test_blake_zisserman_loss():
  """ Test Blake Zisserman """
  eps = 1.0
  loss = BlakeZissermanLoss(eps)

  residual_vals = []
  cost_vals = []

  for r in np.linspace(-5.0, 5.0, 200):
    s = r * r
    (rho, rho_prime, rho_pprime) = loss.eval(s)
    residual_vals.append(r)
    cost_vals.append(rho)

  plt.plot(residual_vals, cost_vals, "r-", label="Blake-Zisserman Loss")
  plt.show()


if __name__ == "__main__":
  # Plot chi-square distribution - df = 2
  df = 2
  # x = np.linspace(chi2.ppf(0.01, df), chi2.ppf(0.99, df), 100)
  # fig, ax = plt.subplots(1, 1)
  # ax.plot(x, chi2.pdf(x, df), 'r-', label='chi2 pdf')
  # plt.show()
  print(f"0.999: {chi2.ppf(0.99, df)}")

  # test_huber_loss()
  # test_blake_zisserman_loss()
