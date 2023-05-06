import numpy as np

from guilda.load import LoadCurrent, LoadImpedance
from guilda.power_network import PowerNetwork
from guilda.bus import BusSlack, BusPV, BusPQ
from guilda.branch import BranchPi
from guilda.generator import Generator1Axis, MachineParameters


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


def simple_3_bus_nishino():
    
    net = PowerNetwork()
    
    y12 = 0.010 + 0.085j
    y23 = 0.017 + 0.092j

    net.add_branch(BranchPi(1, 2, y12, 0))
    net.add_branch(BranchPi(2, 3, y23, 0))

    shunt = 0
    net.add_bus(BusSlack(2, 0, shunt))
    net.add_bus(BusPV(0.5, 2, shunt))
    net.add_bus(BusPQ(-3, 0, shunt))

    # definition of generators
    
    omega0 = np.pi * 60 * 2
    
    component1 = Generator1Axis(omega0, MachineParameters(
        Xd = 1.569, Xd_prime = 0.963, Xq = 0.963, T = 5.14, M = 100, D = 10,
    ))
    
    component2 = Generator1Axis(omega0, MachineParameters(
        Xd = 1.220, Xd_prime = 0.667, Xq = 0.667, T = 8.97, M = 12, D = 10,
    ))
    
    component3 = LoadImpedance()

    net.a_bus[0].set_component(component1)
    net.a_bus[1].set_component(component2)
    net.a_bus[2].set_component(component3)
    
    return net
