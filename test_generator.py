from numpy import pi

from guilda.generator.generator_1axis import Generator1Axis
from guilda.generator.types import GeneratorParameters

omega0 = 60*2*pi
mac = {'Xd':1.569, 'Xd_prime':0.963, 'Xq':0.963, 'T':5.14, 'M':100, 'D':10}
mac_pd = GeneratorParameters(**mac)

component1 = Generator1Axis(omega0, mac_pd)

# ここを変える
v_eq = 3.0 + 0.5j
I = 0.5 + 0.1j


component1.set_equilibrium(v_eq, I)
print(component1.x_equilibrium)
print(component1.V_equilibrium)
print(component1.system_matrix)
