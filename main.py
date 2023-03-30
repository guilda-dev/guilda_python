import numpy as np
from cmath import phase

from guilda.load.load_current import LoadCurrent
from guilda.power_network import PowerNetwork
from guilda.bus import BusSlack, BusPV, BusPQ, Bus
from guilda.branch import BranchPi


y12 = 1.3652 - 11.6040j
y23 = -10.5107j

net = PowerNetwork()
net.add_bus(BusSlack(2, 0, 0))
net.add_bus(BusPQ(-3, 0, 0))
net.add_bus(BusPV(0.5, 2, 0))

net.a_bus[0].set_component(LoadCurrent())
net.a_bus[1].set_component(LoadCurrent())
net.a_bus[2].set_component(LoadCurrent())

a = BranchPi(1, 2, 1/y12, 0)

net.add_branch(BranchPi(1, 2, 1/y12, 0))
net.add_branch(BranchPi(2, 3, 1/y23, 0))


# Y, _ = net.get_admittance_matrix()
# V, I = net.calculate_power_flow()

# PQ = V * I.conjugate()
# P  = PQ.real
# Q  = PQ.imag

# print(P)
# print(Q)



net.initialize()
for idx in range(len(net.a_bus)):
    print(f"bus{idx+1}のVst:{net.a_bus[idx].V_equilibrium}")
    print(f"bus{idx+1}のIst:{net.a_bus[idx].I_equilibrium}")
    print(f"bus{idx+1}のcomponentのIst:{net.a_bus[idx].component.V_equilibrium}")
    print(f"bus{idx+1}のcomponentのIst:{net.a_bus[idx].component.I_equilibrium}")

