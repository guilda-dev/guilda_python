from typing import Hashable
from abc import ABC, abstractmethod as AM

from guilda.utils.typing import ComplexArray

class Branch(ABC):
    
    def __init__(self, bus1: Hashable, bus2: Hashable, z: complex = 0, y: float = 0) -> None:
        self.bus1: Hashable = bus1
        self.bus2: Hashable = bus2
        self.z: complex = z
        self.y: float = y
        
    @AM
    def get_admittance_matrix(self) -> ComplexArray:
        '''_summary_

        Returns:
            FloatArray: _description_
        '''        


