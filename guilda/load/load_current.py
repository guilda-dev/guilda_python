from typing import Optional, Tuple
import guilda.backend as G

from guilda.base import StateEquationRecord
from guilda.load.load import Load
from guilda.utils.data import complex_to_col_vec
from guilda.backend import ArrayProtocol

class LoadCurrent(Load):
    '''モデル：定電流負荷モデル
      ・状態：なし
      ・入力：２ポート「電流フェーザの実部の倍率,電流フェーザの虚部の倍率」
              *入力αのとき値は設定値の(1+α)倍となる．
親クラス：componentクラス
実行方法：obj = load_current()
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
        
        dx = G.zeros((0, 1))
        constraint = complex_to_col_vec(I) - complex_to_col_vec(self.I_equilibrium) * (1 + u[:2, :1])

        return dx, constraint


    def get_linear_matrix(self, V: complex = 0, x: Optional[ArrayProtocol] = None) -> StateEquationRecord:
        A = G.zeros((0, 0))
        B = G.zeros((0, 2))
        C = G.zeros((2, 0))
        D = G.diag([self.I_equilibrium.real, self.I_equilibrium.imag])
        BV = G.zeros((0, 2))
        BI = G.zeros((0, 2))
        DV = G.zeros((2, 2))
        DI = -G.identity(2)
        R = self.R
        S = self.S

        return StateEquationRecord(
            nx=self.nx, nu=self.nu,
            A=A, B=B, C=C, D=D,
            BV=BV, DV=DV, BI=BI, DI=DI,
            R=R, S=S
        )
