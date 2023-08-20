from abc import ABC, abstractmethod as AM

from guilda.backend import ArrayProtocol

class Branch(ABC):
    
    def __init__(self, from_: int, to: int) -> None:
        self.from_: int = from_
        self.to: int = to
        
    @AM
    def get_admittance_matrix(self) -> ArrayProtocol:
        '''_summary_

        Returns:
            ArrayProtocol: _description_
        '''        


