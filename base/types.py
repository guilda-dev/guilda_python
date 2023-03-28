import numpy as np
from dataclasses import dataclass
from typing import Annotated, Literal
from numpy.typing import NDArray

from utils.typing import FloatArray


MatXX = Annotated[FloatArray, Literal['n_x', 'n_x']]
MatUU = Annotated[FloatArray, Literal['n_u', 'n_u']]
MatXU = Annotated[FloatArray, Literal['n_x', 'n_u']]
MatUX = Annotated[FloatArray, Literal['n_u', 'n_x']]

READONLY_ZERO_ARRAY = np.zeros((0, 0))
READONLY_ZERO_ARRAY.setflags(write=False)

@dataclass
class StateEquationRecord:
    """_summary_
    """    
    
    n_x: int = 0
    n_u: int = 0
    
    A: MatXX = READONLY_ZERO_ARRAY
    B: MatXU = READONLY_ZERO_ARRAY
    C: MatUX = READONLY_ZERO_ARRAY
    D: MatUU = READONLY_ZERO_ARRAY
    
    BV: MatXU = READONLY_ZERO_ARRAY
    DV: MatUU = READONLY_ZERO_ARRAY
    
    BI: MatXU = READONLY_ZERO_ARRAY
    DI: MatUU = READONLY_ZERO_ARRAY
    
    R: NDArray = READONLY_ZERO_ARRAY
    S: NDArray = READONLY_ZERO_ARRAY
    
    def as_tuple(self):
        """_summary_

        Returns:
            _type_: _description_
        """        
        return (
            self.A, self.B, self.C, self.D, 
            self.BV, self.DV, self.BI, self.DI, 
            self.R, self.S,
        )