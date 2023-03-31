from typing import Optional, Tuple, List

from guilda.power_network.types import SimulateOptions
from guilda.power_network.base import _PowerNetwork
from guilda.power_network.simulate import simulate

from guilda.utils.typing import FloatArray

class PowerNetwork(_PowerNetwork):
    
    
    def simulate(
        self, 
        t: Tuple[float, float], 
        u: Optional[List[FloatArray]] = None, 
        idx_u: Optional[List[int]] = None, 
        options: Optional[SimulateOptions] = None, 
        ):
        
        return simulate(self, t, u, idx_u, options)
    
    