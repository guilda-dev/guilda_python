import numpy as np
import pycuda.autoinit
import pycuda.driver as drv
from pycuda.compiler import SourceModule
from scipy.integrate import solve_ivp

# 定义ODE
def ode_func(t, y, p):
    y_dot = np.zeros_like(y)
    y_dot[0] = y[1]
    y_dot[1] = p * y[0]
    return y_dot

# 定义CUDA内核
mod = SourceModule("""
    __global__ void ode_kernel(double *y, double *y_dot, double *p)
    {
        int i = threadIdx.x;
        y_dot[i] = 0.0;
        if (i == 0) {
            y_dot[0] = y[1];
        }
        if (i == 1) {
            y_dot[1] = p[0] * y[0];
        }
    }
""")

# 获取内核函数
ode_kernel = mod.get_function("ode_kernel")

# 定义初始值和参数
y0 = np.array([1.0, 0.0])
p = np.array([2.0])

# 定义时间点
t_span = (0.0, 10.0)

# 使用PyCUDA和CUDA内核来求解ODE
def ode_solve_cuda(t, y, p):
    y_dot = np.zeros_like(y)
    ode_kernel(drv.In(y), drv.Out(y_dot), drv.In(p), block=(2,1,1))
    return y_dot

# 使用scipy.integrate.solve_ivp函数来求解ODE
solution = solve_ivp(ode_func, t_span, y0, method='RK45', args=(p,))
solution_cuda = solve_ivp(ode_solve_cuda, t_span, y0, method='RK45', args=(p,))

# 打印结果
print("scipy.integrate.solve_ivp result:")
print(solution.y)
print("PyCUDA result:")
print(solution_cuda.y)
