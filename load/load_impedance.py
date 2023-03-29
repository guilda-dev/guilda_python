from typing import Optional, Tuple

import numpy as np
from base.types import StateEquationRecord

from load.load import Load
from utils.data import complex_to_col_vec, complex_to_matrix
from utils.typing import FloatArray


class LoadImpedance(Load):
    """モデル ：定インピーダンス付加モデル
          ・状態：なし
          ・入力：２ポート「インピーダンス値の実部の倍率,インピーダンス値の虚部の倍率」
                  *入力αのときインピーダンスの値は設定値の(1+α)倍となる．
    親クラス：componentクラス
    実行方法：obj = load_impedance()
     引数 ：なし
     出力 ：componentクラスのインスタンス
    """    
    
    
    
    def set_equilibrium(self, V: complex, I: complex) -> None:
        super().set_equilibrium(V, I)
        self.set_admittance(I / V)


    def get_dx_constraint(
        self,
        V: complex = 0,
        I: complex = 0,
        x: Optional[FloatArray] = None,
        u: Optional[FloatArray] = None,
        t: float = 0) -> Tuple[FloatArray, FloatArray]:
        assert u is not None
        dx = np.zeros([0, 1])
        Y_vec = complex_to_col_vec(self.Y) * (1 + u[:2, :1])
        Y = Y_vec[0, 0] + 1j * Y_vec[1, 0]
        I_ = Y * V
        constraint = complex_to_col_vec(I - I_)
        return dx, constraint

    def get_linear_matrix(self, V: complex = 0, x: Optional[FloatArray] = None) -> StateEquationRecord:
        if x is None:
            V = self.V_equilibrium

        A = np.zeros([0, 0])
        B = np.zeros([0, 2])
        C = np.zeros([2, 0])
        D1 = complex_to_matrix(self.Y.real) @ complex_to_col_vec(V)
        D2 = complex_to_matrix(
            1j*(self.Y.imag)) @ complex_to_col_vec(V)
        D = np.hstack([D1, D2])
        BV = np.zeros([0, 2])
        DV = self.Y_mat
        R = self.R
        S = self.S
        BI = np.zeros([0, 2])
        DI = -np.identity(2)
        
        return StateEquationRecord(
            n_x=self.get_nx(), n_u=self.get_nu(),
            A=A, B=B, C=C, D=D,
            BV=BV, DV=DV.astype(BV.dtype), BI=BI, DI=DI,
            R=R, S=S
        )
