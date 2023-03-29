from typing import Optional, Tuple
import numpy as np

from base.types import StateEquationRecord
from load.load import Load
from utils.data import complex_to_col_vec
from utils.typing import FloatArray

class LoadCurrent(Load):
    """モデル ：定電流負荷モデル
      ・状態：なし
      ・入力：２ポート「電流フェーザの実部の倍率,電流フェーザの虚部の倍率」
              *入力αのとき値は設定値の(1+α)倍となる．
親クラス：componentクラス
実行方法：obj = load_current()
引数 ：なし
出力 ：componentクラスのインスタンス

    Args:
        Component (_type_): _description_
    """


    def get_dx_constraint(
        self,
        V: complex = 0,
        I: complex = 0,
        x: Optional[FloatArray] = None,
        u: Optional[FloatArray] = None,
        t: float = 0) -> Tuple[FloatArray, FloatArray]:
        
        assert u is not None
        
        dx = np.zeros((0, 1))
        constraint = complex_to_col_vec(I) - complex_to_col_vec(self.I_equilibrium) * (1 + u[:2, :1])

        return dx, constraint


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
            n_x=self.get_nx(), n_u=self.get_nu(),
            A=A, B=B, C=C, D=D,
            BV=BV, DV=DV, BI=BI, DI=DI,
            R=R, S=S
        )
