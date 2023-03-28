import numpy as np

from base.component import Component

class LoadCurrent(Component):
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
    def __init__(self):
        self.x_equilibrium = np.zeros([0, 1])
        self.V_equilibrium = None
        self.I_equilibrium = None
        self.Y = None
        self.R = np.zeros([0,0])
        self.S = np.array([]).reshape(1, -1)

    def set_equilibrium(self, v, i):
        # 複素数表記で平衡点を取得
        self.V_equilibrium = v
        self.I_equilibrium = i

    def get_dx_constraint(self, I, u, t=None, x=None, V=None):
        dx = np.zeros([0, 1])
        constraint = np.array([[I.real], [I.imag]]) - np.array([[self.I_equilibrium.real * (1 + u[0, 0])], [self.I_equilibrium.imag * (1 + u[1, 0])]])

        return [dx, constraint]

    def get_dx_constraint_linear(self, x, V, I, u, t=None):
        [A, B, C, D, BV, DV, BI, DI, _, _] = self.get_linear_matrix(V, x)
        dx = np.zeros([0, 1])
        diff_I = np.array([[I.real], [I.imag]]) - np.array([[self.I_equilibrium.real], [self.I_equilibrium.imag]])
        diff_V = np.array([[V.real], [V.imag]]) - np.array([[self.V_equilibrium.real], [self.V_equilibrium.imag]])
        constraint = D@u + DI@diff_I + DV@diff_V

        return [dx, constraint]

    def get_nu(self):
        return 2

    def get_linear_matrix(self, v, x):
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

        return [A, B, C, D, BV, DV, BI, DI, R, S]
