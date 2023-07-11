from typing import Optional, Tuple

import numpy as np

from guilda.base import Component

from guilda.utils.data import complex_to_col_vec, complex_to_matrix
from guilda.utils.typing import FloatArray

class Load(Component):
    """負荷モデル
      ・状態：なし
      ・入力：２ポート「有効電力の倍率,無効電力の倍率」
              *入力αのとき電力の値は設定値の(1+α)倍となる．
    """
    
    @property
    def x_equilibrium(self) -> FloatArray:
        return np.zeros((0, 1))
      
      
    @property
    def Y_mat(self) -> FloatArray:
        return complex_to_matrix(self.Y)
    
    
    def set_admittance(self, Y: complex):
        self.Y = Y
        
    
    def __init__(self):
        super().__init__()
        self.S: FloatArray = np.zeros((1, 0))
        self.R: FloatArray = np.zeros((0, 0))

        self.Y: complex = 0
        

    def get_dx_constraint_linear(
        self,
        V: complex = 0,
        I: complex = 0,
        x: Optional[FloatArray] = None,
        u: Optional[FloatArray] = None,
        t: float = 0) -> Tuple[FloatArray, FloatArray]:
        assert u is not None
        E = self.get_linear_matrix(V, x)
        dx = np.zeros([0, 1])
        diff_I: FloatArray = complex_to_col_vec(I) - complex_to_col_vec(self.I_equilibrium)
        diff_V: FloatArray = complex_to_col_vec(V) - complex_to_col_vec(self.V_equilibrium)
        constraint: FloatArray = E.D @ u + E.DI @ diff_I + E.DV @ diff_V
        return dx, constraint
      
    @property
    def nx(self):
        return 0

    @property
    def nu(self):
        return 2
    
    @property
    def nl(self):
        return 0


