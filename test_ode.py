import numpy as np
from scipy.integrate import solve_ivp

# 定义带有质量矩阵的常微分方程组
def fun(t, y):
    M = np.array([[2, 1], [1, 2]])
    return np.linalg.solve(M, np.array([y[1], -y[0]]))

# 定义初始条件
t0, t1 = 0, 10
y0 = [0, 1]

# 定义质量矩阵
def mass(t, y):
    return np.array([[1, 0], [0, 2]])

# 求解常微分方程组
sol = solve_ivp(fun, (t0, t1), y0, method='Radau', mass=mass)

print(sol.y)
