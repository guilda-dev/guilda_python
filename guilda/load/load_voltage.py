from typing import Optional, Tuple

import numpy as np

from guilda.base import StateEquationRecord
from guilda.load.load import Load
from guilda.utils.data import complex_to_col_vec
from guilda.backend import ArrayProtocol

class LoadVoltage(Load):
    '''モデル：定電圧負荷モデル
      ・状態：なし
      ・入力：２ポート「電圧フェーザの実部の倍率,電圧フェーザの虚部の倍率」
              *入力αのとき値は設定値の(1+α)倍となる．
親クラス：componentクラス
実行方法：obj = load_voltage()
引数：なし
出力：componentクラスのインスタンス

    Args:
        Component (_type_): _description_
    '''

    def get_dx_constraint(
        self,
        V: complex = 0,
        I: complex = 0,
        x: Optional[ArrayProtocol] = None,
        u: Optional[ArrayProtocol] = None,
        t: float = 0) -> Tuple[ArrayProtocol, ArrayProtocol]:
        assert u is not None
        dx: ArrayProtocol = np.zeros((0, 1))
        constraint: ArrayProtocol = complex_to_col_vec(V) - complex_to_col_vec(self.V_equilibrium) * (1 + u[:2, :1])
        return (dx, constraint)

    def get_linear_matrix(self, V: complex = 0, x: Optional[ArrayProtocol] = None) -> StateEquationRecord:
        A = np.zeros((0, 0))
        B = np.zeros((0, 2))
        C = np.zeros((2, 0))
        D = np.diag([self.I_equilibrium.real, self.I_equilibrium.imag])
        BV = np.zeros((0, 2))
        BI = np.zeros((0, 2))
        DV = np.zeros((2, 2))
        DI = -np.identity(2)
        R = self.R
        S = self.S

        return StateEquationRecord(
            nx=self.nx, nu=self.nu,
            A=A, B=B, C=C, D=D,
            BV=BV, DV=DV, BI=BI, DI=DI,
            R=R, S=S
        )
