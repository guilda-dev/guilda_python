from typing import Type
import numpy as np

from guilda.controller.controller_broadcast_PI_AGC import ControllerBroadcastPIAGC
from guilda.load import LoadCurrent, LoadImpedance
from guilda.power_network import PowerNetwork
from guilda.bus import BusSlack, BusPV, BusPQ
from guilda.branch import BranchPi
from guilda.generator import Generator, Generator1Axis, GeneratorParameters


def simple_2_bus_moris(generator_model: Type[Generator] = Generator):

    z12 = 0.010 + 0.085j
    omega0 = np.pi * 60 * 2

    net = PowerNetwork()
    bus1 = net.add_bus(BusSlack(2, 0, 0))
    bus2 = net.add_bus(BusPV(0.5, 2, 0))

    bus1.set_component(generator_model(omega0, GeneratorParameters(
        M=100, D=10, Xd=0.963, Xd_prime=0.963, Xq=0.963, Tdo=5.14, 
    )))
    bus2.set_component(generator_model(omega0, GeneratorParameters(
        M=12, D=10, Xd=0.667, Xd_prime=0.667, Xq=0.667, Tdo=8.97, 
    )))

    net.add_branch(BranchPi(1, 2, z12, 0))

    return net


def simple_3_bus_nishino(controller=False, generator_model: Type[Generator] = Generator1Axis):

    z12 = 0.010 + 0.085j
    z23 = 0.017 + 0.092j

    omega0 = np.pi * 60 * 2
    shunt = 0
    
    net = PowerNetwork()

    bus1 = net.add_bus(BusSlack(2, 0, shunt))
    bus2 = net.add_bus(BusPV(0.5, 2, shunt))
    bus3 = net.add_bus(BusPQ(-3, 0, shunt))

    net.add_branch(BranchPi(1, 2, z12, 0))
    net.add_branch(BranchPi(2, 3, z23, 0))

    gen1 = generator_model(omega0, GeneratorParameters(
        Xd=1.569, Xd_prime=0.963, Xq=0.963, Tdo=5.14, M=100, D=10,
    ))

    gen2 = generator_model(omega0, GeneratorParameters(
        Xd=1.220, Xd_prime=0.667, Xq=0.667, Tdo=8.97, M=12, D=10,
    ))

    load = LoadImpedance()

    bus1.set_component(gen1)
    bus2.set_component(gen2)
    bus3.set_component(load)

    if controller:
        gen_indices = [1, 2]
        ctrl = ControllerBroadcastPIAGC(
            gen_indices, gen_indices,  # type: ignore
            -10, -500
        )
        net.add_controller_global(ctrl)

    return net
