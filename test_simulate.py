import numpy as np
from guilda.power_network import SimulationOptions

import guilda.models as sample

import matplotlib.pyplot as plt


np.set_printoptions(
    precision=6, 
    suppress=True,
)

# net2 = sample.IEEE68bus()

net = sample.simple_3_bus_nishino()
net.initialize()

Y = net.get_admittance_matrix()
V, I = net.calculate_power_flow()

net.print_bus_state()

options = SimulationOptions(
    linear=False,
)
options.set_parameter_from_pn(net)

result = net.simulate(
    (0, 10, 20, 60), 
    np.array([
        [0, 0.05, 0.1, 0.1],
        [0,    0,   0,   0],
    ]), 
    [2], 
    options
)

plt.plot(result.t, result.x[0][:, 1], label="rotator angle difference")
plt.plot(result.t, result.x[1][:, 1], label="rotator angle difference")
plt.legend()
plt.show()

print(result)
