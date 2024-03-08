from typing import Type
import numpy as np

from guilda.controller.controller_broadcast_PI_AGC import ControllerBroadcastPIAGC
from guilda.load import LoadCurrent, LoadImpedance
from guilda.power_network import PowerNetwork
from guilda.bus import BusSlack, BusPV, BusPQ
from guilda.branch import BranchPi
from guilda.generator import Generator, Generator1Axis, GeneratorParameters


def simple_3_bus():
    
    y12 = 1.3652 - 11.6040j
    y23 = -10.5107j

    net = PowerNetwork()
    net.add_bus(BusSlack(2, 0, 0))
    net.add_bus(BusPQ(-3, 0, 0))
    net.add_bus(BusPV(0.5, 2, 0))

    for b in net.a_bus:
        b.set_component(LoadCurrent())

    net.add_branch(BranchPi(1, 2, 1/y12, 0))
    net.add_branch(BranchPi(2, 3, 1/y23, 0))
    
    return net


def simple_3_bus_nishino(controller = False, generator_model: Type[Generator]=Generator1Axis):
    
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
    
    component1 = generator_model(omega0, GeneratorParameters(
        Xd = 1.569, Xd_prime = 0.963, Xq = 0.963, Tdo = 5.14, M = 100, D = 10,
    ))
    
    component2 = generator_model(omega0, GeneratorParameters(
        Xd = 1.220, Xd_prime = 0.667, Xq = 0.667, Tdo = 8.97, M = 12, D = 10,
    ))
    
    component3 = LoadImpedance()

    net.a_bus_dict[1].set_component(component1)
    net.a_bus_dict[2].set_component(component2)
    net.a_bus_dict[3].set_component(component3)
    
    if controller:
        gen_indices = [1, 2]
        ctrl = ControllerBroadcastPIAGC(
            gen_indices, gen_indices, # type: ignore
            -10, -500
        )
        net.add_controller_global(ctrl)
    
    return net
