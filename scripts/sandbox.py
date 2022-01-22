#!/usr/bin/env python3
from sympy import exp
from sympy import log
from sympy import sqrt
from sympy import symbols
from sympy import diff

r = symbols("r")
k = symbols("k")
epsilon = symbols("epsilon")

# Blake-Zisserman Loss
loss = -log(exp(-r**2.0) + epsilon)
loss_deriv = diff(loss, r)
loss_dderiv = diff(loss_deriv, r)

# # Huber Loss
# loss = 2.0 * sqrt(s) - 1.0
# loss_deriv = diff(loss, s)
# loss_dderiv = diff(loss_deriv, s)

print(loss)
print(loss_deriv / r)
# print(loss_dderiv)
