import numpy as np
from cmath import phase

from guilda.load import LoadCurrent
from guilda.power_network import PowerNetwork, SimulateOptions
from guilda.bus import BusSlack, BusPV, BusPQ
from guilda.branch import BranchPi


import sample



np.set_printoptions(
    precision=6, 
    suppress=True,
)

net = sample.simple_3_bus_nishino()
net.initialize()

Y = net.get_admittance_matrix()
V, I = net.calculate_power_flow()

net.print_bus_state()

options = SimulateOptions(
    
)

result = net.simulate(
    (0, 10), 
    np.array([
        [0, 0],
        [0, 0],
        [0, 0],
        [0, 0],
        [0, 0],
        [0, 0],
    ]), 
    [0, 1, 2], 
    options
)


