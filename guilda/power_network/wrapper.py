from typing import Optional, Iterable

from guilda.power_network.types import SimulationOptions
from guilda.power_network.base import _PowerNetwork
from guilda.power_network.simulate import simulate

from guilda.backend import ArrayProtocol

class PowerNetwork(_PowerNetwork):
    
    
    def simulate(
        self, 
        t: Iterable[float], 
        u: Optional[ArrayProtocol] = None, 
        idx_u: Optional[Iterable[int]] = None, 
        options: Optional[SimulationOptions] = None, 
        ):
        
        return simulate(self, t, u, idx_u, options)
    
    def print_bus_state(self) -> None:
        for i, b in enumerate(self.a_bus):
            print(f"Bus #{i + 1} {type(b)}: ")
            print(f"  Vst: {b.V_equilibrium}")
            print(f"  Ist: {b.I_equilibrium}")
            print(f"  component.Vst: {b.component.V_equilibrium}")
            print(f"  component.Ist: {b.component.I_equilibrium}")
