from abc import ABC, abstractmethod as AM

from guilda.utils.typing import ComplexArray

class Branch(ABC):
    
    def __init__(self, from_: int, to: int) -> None:
        self.from_: int = from_
        self.to: int = to
        
    @AM
    def get_admittance_matrix(self) -> ComplexArray:
        """_summary_

        Returns:
            FloatArray: _description_
        """        


