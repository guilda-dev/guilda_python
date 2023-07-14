import numpy as np
from dataclasses import dataclass, field
from typing import Annotated, Literal
from numpy.typing import NDArray

from guilda.utils.typing import FloatArray


MatXX = Annotated[FloatArray, Literal['n_x', 'n_x']]
MatUU = Annotated[FloatArray, Literal['n_u', 'n_u']]
MatXU = Annotated[FloatArray, Literal['n_x', 'n_u']]
MatUX = Annotated[FloatArray, Literal['n_u', 'n_x']]

 
@dataclass
class StateEquationRecord:
    '''_summary_
    '''    
    
    nx: int = 0
    nu: int = 0
    
    A: MatXX = field(default_factory=lambda: np.zeros((0, 0)))
    B: MatXU = field(default_factory=lambda: np.zeros((0, 0)))
    C: MatUX = field(default_factory=lambda: np.zeros((0, 0)))
    D: MatUU = field(default_factory=lambda: np.zeros((0, 0)))
    
    BV: MatXU = field(default_factory=lambda: np.zeros((0, 0)))
    DV: MatUU = field(default_factory=lambda: np.zeros((0, 0)))
    
    BI: MatXU = field(default_factory=lambda: np.zeros((0, 0)))
    DI: MatUU = field(default_factory=lambda: np.zeros((0, 0)))
    
    R: NDArray = field(default_factory=lambda: np.zeros((0, 0)))
    S: NDArray = field(default_factory=lambda: np.zeros((0, 0)))
    
    def as_tuple(self):
        '''_summary_

        Returns:
            _type_: _description_
        '''        
        return (
            self.A, self.B, self.C, self.D, 
            self.BV, self.DV, self.BI, self.DI, 
            self.R, self.S,
        )
        
    def copy(self):
        return StateEquationRecord(
            A = self.A, B = self.B, C = self.C, D = self.D, 
            BV = self.BV, DV = self.DV, BI = self.BI, DI = self.DI, 
            R = self.R, S = self.S, 
            nx = self.nx, 
            nu = self.nu,
        )