from typing import Optional, Tuple
from base.component import Component

import numpy as np

from base.types import FloatArray, StateEquationRecord
from utils.data import complex_to_col_vec

class LoadVoltage(Component):
    """モデル ：定電圧負荷モデル
      ・状態：なし
      ・入力：２ポート「電圧フェーザの実部の倍率,電圧フェーザの虚部の倍率」
              *入力αのとき値は設定値の(1+α)倍となる．
親クラス：componentクラス
実行方法：obj = load_voltage()
引数 ：なし
出力 ：componentクラスのインスタンス

    Args:
        Component (_type_): _description_
    """    

    def __init__(self):
        self.x_equilibrium = np.zeros([0, 1])
        self.S = np.array([]).reshape(1, -1)
        self.R = np.zeros([0,0])

        self.V_equilibrium: complex = 0
        self.I_equilibrium: complex = 0
        self.Y = None

    def set_equilibrium(self, V: complex = 0, I: complex = 0) -> None:
        self.V_equilibrium = V
        self.I_equilibrium = I

    def get_dx_constraint(
        self,
        V: complex = 0,
        I: complex = 0,
        x: Optional[FloatArray] = None,
        u: Optional[FloatArray] = None,
        t: float = 0) -> Tuple[FloatArray, FloatArray]:
        assert u is not None
        dx: FloatArray = np.zeros([0, 1])
        constraint: FloatArray = complex_to_col_vec(V) - complex_to_col_vec(self.V_equilibrium) * (1+u[1, 0])
        return (dx, constraint)

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
        constraint = E.D@u + E.DI@diff_I + E.DV@diff_V
        return (dx, constraint)

    def get_nu(self):
        return 2

    def get_linear_matrix(self, V: complex = 0, x: Optional[FloatArray] = None) -> StateEquationRecord:
        A = np.zeros([0, 0])
        B = np.zeros([0, 2])
        C = np.zeros([2, 0])
        D = np.diag([self.I_equilibrium.real, self.I_equilibrium.imag])
        BV = np.zeros([0, 2])
        BI = np.zeros([0, 2])
        DV = np.zeros([2, 2])
        DI = -np.identity(2)
        R = self.R
        S = self.S
    
        return StateEquationRecord(
            n_x = self.get_nx(), n_u = self.get_nu(),
            A = A, B = B, C = C, D = D, 
            BV = BV, DV = DV, BI = BI, DI = DI, 
            R = R, S = S
        )
