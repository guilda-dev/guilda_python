from typing import Optional, Tuple

import guilda.backend as G

from guilda.base import Component

from guilda.utils.data import complex_to_col_vec, complex_to_matrix
from guilda.backend import ArrayProtocol

class Load(Component):
    '''負荷モデル
      ・状態：なし
      ・入力：２ポート「有効電力の倍率,無効電力の倍率」
              *入力αのとき電力の値は設定値の(1+α)倍となる．
    '''
    
    @property
    def x_equilibrium(self) -> ArrayProtocol:
        return G.zeros((0, 1))
      
      
    @property
    def Y_mat(self) -> ArrayProtocol:
        return complex_to_matrix(self.Y)
    
    
    def set_admittance(self, Y: complex):
        self.Y = Y
        
    
    def __init__(self):
        super().__init__()
        self.S: ArrayProtocol = G.zeros((1, 0))
        self.R: ArrayProtocol = G.zeros((0, 0))

        self.Y: complex = 0
        

    def get_dx_constraint_linear(
        self,
        V: complex = 0,
        I: complex = 0,
        x: Optional[ArrayProtocol] = None,
        u: Optional[ArrayProtocol] = None,
        t: float = 0) -> Tuple[ArrayProtocol, ArrayProtocol]:
        assert u is not None
        E = self.get_linear_matrix(V, x)
        dx = G.zeros((0, 1))
        diff_I: ArrayProtocol = complex_to_col_vec(I) - complex_to_col_vec(self.I_equilibrium)
        diff_V: ArrayProtocol = complex_to_col_vec(V) - complex_to_col_vec(self.V_equilibrium)
        constraint: ArrayProtocol = E.D @ u + E.DI @ diff_I + E.DV @ diff_V
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


