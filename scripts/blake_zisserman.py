#!/usr/bin/env python3
import numpy as np
import matplotlib.pylab as plt
import sympy


def diff_blake_zisserman():

  s = sympy.Symbol('s')
  eps = sympy.Symbol('eps')

  expr = -sympy.log(sympy.exp(-s) + eps)
  expr_diff = sympy.diff(expr, s)
  expr_ddiff = sympy.diff(expr_diff, s)

  sympy.pprint(expr)
  sympy.pprint(sympy.simplify(expr_diff))
  sympy.pprint(sympy.simplify(expr_ddiff))


diff_blake_zisserman()


def blake_zisserman(s):
  eps = 0.000009
  x = eps * np.exp(s) + 1.0
  rho0 = -np.log(eps + np.exp(-s))
  rho1 = 1.0 / x
  rho2 = (x - x**2) / x**3.0

  return (rho0, rho1, rho2)


r = np.array([0.1, 0.2])
s = np.linalg.norm(r @ r)
h = 1e-8

(rho0, rho1, rho2) = blake_zisserman(s)

(rho0_fwd, rho1_fwd, rho2_fwd) = blake_zisserman(s + h)
(rho0_bwd, rho1_bwd, rho2_bwd) = blake_zisserman(s - h)

J_diff = rho1 - ((rho0_fwd - rho0_bwd) / (2.0 * h))
JJ_diff = rho2 - ((rho1_fwd - rho1_bwd) / (2.0 * h))
print(J_diff)
print(JJ_diff)
