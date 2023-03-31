import numpy as np
from cmath import phase

from guilda.load import LoadCurrent
from guilda.power_network import PowerNetwork, SimulateOptions
from guilda.bus import BusSlack, BusPV, BusPQ
from guilda.branch import BranchPi


import sample

net = sample.simple_3_bus()
net.initialize()

Y = net.get_admittance_matrix()
V, I = net.calculate_power_flow()


options = SimulateOptions(
    
)

result = net.simulate(
    (0, 10), 
    [x * 0 for x in net.x_equilibrium], 
    [0, 1, 2], 
    options
)


for idx in range(len(net.a_bus)):
    print(f"bus{idx+1}のVst:{net.a_bus[idx].V_equilibrium}")
    print(f"bus{idx+1}のIst:{net.a_bus[idx].I_equilibrium}")
    print(f"bus{idx+1}のcomponentのIst:{net.a_bus[idx].component.V_equilibrium}")
    print(f"bus{idx+1}のcomponentのIst:{net.a_bus[idx].component.I_equilibrium}")

