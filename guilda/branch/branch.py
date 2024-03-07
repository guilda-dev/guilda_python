from typing import Hashable
from abc import ABC, abstractmethod as AM

from guilda.utils.typing import ComplexArray

class Branch(ABC):
    
    def __init__(self, from_: Hashable, to: Hashable) -> None:
        self.bus1: Hashable = from_
        self.bus2: Hashable = to
        
    @AM
    def get_admittance_matrix(self) -> ComplexArray:
        '''_summary_

        Returns:
            FloatArray: _description_
        '''        


