import numpy as np

# sampring period
T = 0.05

# -- state space model of LTI system
# x[t+1] = A*x[t] + B*u[t]
A = np.array([[1., T],
              [0, 1.]])
B = np.array([[0.],
              [T]])
