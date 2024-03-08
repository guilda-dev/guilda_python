import numpy as np
from guilda.generator import Generator, Generator1Axis
from guilda.power_network import SimulationOptions

import guilda.models as sample

import matplotlib.pyplot as plt

from guilda.power_network.types import BusInput, SimulationScenario


np.set_printoptions(
    precision=6,
    suppress=True,
)

# net2 = sample.IEEE68bus()

net = sample.simple_3_bus_nishino(True, generator_model=Generator1Axis)
net.initialize()

Y = net.get_admittance_matrix()
V, I = net.calculate_power_flow()

net.print_bus_state()

scenario = SimulationScenario(
    tstart=0, 
    tend=60,
    u=[
        BusInput(
            index=3,
            time=[0, 10, 20, 60],
            value=np.array([
                [0, 0.05, 0.1, 0.1],
                [0,    0,   0,   0],
            ]).T,
        )
    ]
)

options = SimulationOptions(
    linear=False,
    rtol=1e-6,
    atol=1e-6,
)

result = net.simulate(
    scenario, 
    options
)

plt.plot(result.t, result.x[0][:, 1], label="rotator angle difference")
plt.plot(result.t, result.x[1][:, 1], label="rotator angle difference")
plt.legend()
plt.show()

print(result)
