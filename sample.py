from guilda.load import LoadCurrent
from guilda.power_network import PowerNetwork
from guilda.bus import BusSlack, BusPV, BusPQ
from guilda.branch import BranchPi


def simple_3_bus():
    
    y12 = 1.3652 - 11.6040j
    y23 = -10.5107j

    net = PowerNetwork()
    net.add_bus(BusSlack(2, 0, 0))
    net.add_bus(BusPQ(-3, 0, 0))
    net.add_bus(BusPV(0.5, 2, 0))

    net.a_bus[0].set_component(LoadCurrent())
    net.a_bus[1].set_component(LoadCurrent())
    net.a_bus[2].set_component(LoadCurrent())

    net.add_branch(BranchPi(1, 2, 1/y12, 0))
    net.add_branch(BranchPi(2, 3, 1/y23, 0))
    
    return net
