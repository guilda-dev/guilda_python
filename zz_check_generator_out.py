import pandas as pd
from numpy import pi

from generator.generator_1axis import Generator1Axis

omega0 = 60*2*pi
mac = {'Xd':1.569, 'Xd_prime':0.963, 'Xq':0.963, 'T':5.14, 'M':100, 'D':10}
mac_pd = pd.Series(mac)

component1 = Generator1Axis(omega0, mac_pd)

# ここを変える
V = 3.0 + 0.5j
I = 0.5 + 0.1j


x_st = component1.set_equilibrium(V, I)
print(component1.x_equilibrium)
print(component1.V_equilibrium)
print(component1.system_matrix)
