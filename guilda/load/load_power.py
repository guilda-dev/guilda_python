from typing import Optional, Tuple

import numpy as np

from guilda.base import StateEquationRecord
from guilda.load.load import Load
from guilda.utils.data import complex_to_col_vec
from guilda.utils.typing import FloatArray

class LoadPower(Load):
    """モデル ：定電力負荷モデル
      ・状態：なし
      ・入力：２ポート「有効電力の倍率,無効電力の倍率」
              *入力αのとき電力の値は設定値の(1+α)倍となる．
親クラス：componentクラス
実行方法：obj = load_power()
 引数 ：なし
 出力 ：componentクラスのインスタンス

    Args:
        Component (_type_): _description_
    """
    def __init__(self):
        super().__init__()
        self.P_st: float = 0
        self.Q_st: float = 0


    def set_equilibrium(self, V: complex, I: complex) -> None:
        super().set_equilibrium(V, I)
        self.set_power(V, I)
        
        
    def set_power(self, Veq: complex, Ieq: complex) -> None:
        PQ = Veq * Ieq.conjugate()
        self.P_st = PQ.real
        self.Q_st = PQ.imag
        

    def get_dx_constraint(
        self,
        V: complex = 0,
        I: complex = 0,
        x: Optional[FloatArray] = None,
        u: Optional[FloatArray] = None,
        t: float = 0) -> Tuple[FloatArray, FloatArray]:
        assert u is not None
        dx: FloatArray = np.zeros((0, 1))
        PQ = self.P_st * (1 + u[0, 0]) + 1j * self.Q_st * (1 + u[1, 0])
        I_ = PQ/V
        constraint = complex_to_col_vec(I) - complex_to_col_vec(I_)
        return dx, constraint


    def get_linear_matrix(self, V: complex = 0, x: Optional[FloatArray] = None) -> StateEquationRecord:
        if x is None:
            V = self.V_equilibrium
        
        den = abs(V)**2
        Vr = V.real
        Vi = V.imag
        P = self.P_st
        Q = self.Q_st

        A = np.zeros([0, 0])
        B = np.zeros([0, 2])
        C = np.zeros([2, 0])
        D = np.array([[P*Vr, Q*Vi], [P*Vi, -Q*Vr]])/abs(V)
        BV = np.zeros([0, 2])
        DV = np.array([[P, Q], [-Q, P]]) @ np.array([[(Vi**2 - Vr**2)/den, -2*Vr*Vi/den], [-2*Vr*Vi/den, (Vr**2 - Vi**2)/den]])
        R = self.R
        S = self.S
        BI = np.zeros([0, 2])
        DI = -np.identity(2)

        return StateEquationRecord(
            nx=self.nx, nu=self.nu,
            A=A, B=B, C=C, D=D,
            BV=BV, DV=DV.astype(BV.dtype), BI=BI, DI=DI,
            R=R, S=S
        )

