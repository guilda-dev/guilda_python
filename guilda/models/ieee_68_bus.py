from collections import defaultdict
import numpy as np

from typing import Dict, TypeVar, Type, List
from guilda.bus import BusParameters, BusSlack, BusPV, BusPQ
from guilda.branch import BranchParameters, BranchPi, BranchPiTransformer, Branch
from guilda.generator import GeneratorParameters, PssParameters, Generator1Axis, Pss
from guilda.avr import AvrSadamoto2019Parameters, AvrSadamoto2019

from guilda.load import LoadImpedance

from guilda.power_network import PowerNetwork

from guilda.utils import get_resource_path, from_sheet

BASE_PATH = 'static/IEEE68bus/'
T = TypeVar('T')

def _r(type: Type[T], path: str) -> List[T]: 
    return list(from_sheet(type, get_resource_path(BASE_PATH + path)))

buses = _r(BusParameters, 'bus.csv')
branches = _r(BranchParameters, 'branch.csv')
machineries = _r(GeneratorParameters, 'machinery.csv')
excitations = _r(AvrSadamoto2019Parameters, 'excitation.csv')
pss_data = _r(PssParameters,  'pss.csv')

gen_by_bus: Dict[int, List[GeneratorParameters]] = defaultdict(list) # type: ignore
for machinery in machineries:
    gen_by_bus[machinery.No_bus].append(machinery)
    
ex_by_bus: Dict[int, List[AvrSadamoto2019Parameters]] = defaultdict(list) # type: ignore
for ex in excitations:
    ex_by_bus[ex.No_bus].append(ex)
    
pss_by_bus: Dict[int, List[PssParameters]] = defaultdict(list) # type: ignore
for pss in pss_data:
    pss_by_bus[pss.No_bus].append(pss)

OMEGA_0 = 60 * 2 * np.pi

class IEEE68bus(PowerNetwork):
    
    def __init__(self):
        super().__init__()
    
        # add buses
        for bus in buses:
            shunt = bus.G_shunt + 1j * bus.B_shunt
            if bus.type == 1:
                self.add_bus(BusSlack(bus.V_abs, bus.V_angle, shunt))
                self.set_generator(bus.No)
            elif bus.type == 2:
                self.add_bus(BusPV(bus.P_gen, bus.V_abs, shunt))
                self.set_generator(bus.No)
            elif bus.type == 3:
                self.add_bus(BusPQ(-bus.P_load, -bus.Q_load, shunt))
                if bus.P_load != 0 or bus.Q_load != 0:
                    load = LoadImpedance()
                    self.a_bus[-1].set_component(load)
        
        # add branches
        for branch in branches:
            br: Branch
            if branch.tap == 0:
                br = BranchPi(
                    branch.bus_from, 
                    branch.bus_to, 
                    branch.x_real + 1j * branch.x_imag, 
                    branch.y
                )
            else:
                br = BranchPiTransformer(
                    branch.bus_from, 
                    branch.bus_to, 
                    branch.x_real + 1j * branch.x_imag, 
                    branch.y,
                    branch.tap,
                    branch.phase
                )
            self.add_branch(br)
            
        self.initialize()
        
    def set_generator(self, i: int):
        if len(gen_by_bus[i]) == 0:
            return
        generator = gen_by_bus[i][0]
        g = Generator1Axis(OMEGA_0, generator)
        g.set_avr(AvrSadamoto2019(ex_by_bus[i][0]))
        g.set_pss(Pss(pss_by_bus[i][0]))
        self.a_bus[-1].set_component(g)

    
