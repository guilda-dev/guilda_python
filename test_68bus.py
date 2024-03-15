import numpy as np
import pickle
import matplotlib.pyplot as plt


from guilda.generator import Generator, Generator1Axis
from guilda.power_network import SimulationOptions

import guilda.models as sample

from guilda.power_network.types import BusInput, SimulationScenario


np.set_printoptions(
    precision=6,
    suppress=True,
)
net = sample.IEEE68bus()



scenario = SimulationScenario(
    tstart = 0,
    tend = 20,
    dx_init_sys={
        1: np.array([np.pi / 6, 0, 0, 0, 0, 0, 0]).reshape((-1, 1))
    }
)


net.initialize()

Y = net.get_admittance_matrix()
V, I = net.calculate_power_flow()

net.print_bus_state()


options = SimulationOptions(
    linear=False,
    rtol=1e-6,
    atol=1e-6,
)

result = net.simulate(
    scenario, 
    options
)

plt.plot(result.t, result[1].x[:, 1], label="rotator angle difference")
plt.plot(result.t, result[2].x[:, 1], label="rotator angle difference")
plt.legend()
plt.show()

print(result)

