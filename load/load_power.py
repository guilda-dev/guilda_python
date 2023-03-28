from base.component import Component

import numpy as np

class LoadPower(Component):
# モデル ：定電力負荷モデル
#       ・状態：なし
#       ・入力：２ポート「有効電力の倍率,無効電力の倍率」
#               *入力αのとき電力の値は設定値の(1+α)倍となる．
#親クラス：componentクラス
#実行方法：obj = load_power()
#　引数　：なし
#　出力　：componentクラスのインスタンス
    def __init__(self, *args):
        self.x_equilibrium = np.zeros([0, 1])
        self.S = np.array([]).reshape(1, -1)
        self.R = np.zeros([0,0])

        self.V_equilibrium = None
        self.I_equilibrium = None
        self.P_st = None
        self.Q_st = None
        self.Y = None

    def set_equilibrium(self, Veq, Ieq):
        self.V_equilibrium = Veq
        self.I_equilibrium = Ieq
        self.set_power(Veq, Ieq)

    def get_dx_constraint(self, V, I, u, t=None, x=None):
        dx = np.zeros([0, 1])
        PQ = self.P_st*(1+u[0, 0]) + 1j*self.Q_st*(1+u[1, 0])
        I_ = PQ/V
        constraint = np.array([[I.real], [I.imag]]) - np.array([[I_.real], [I_.imag]])
        return [dx, constraint]

    def get_dx_constraint_linear(self, V, I, u, t=None, x=None):
        [A, B, C, D, BV, DV, BI, DI, _, _] = self.get_linear_matrix_(x, V)
        dx = np.zeros([0, 1])
        diff_I = np.array([[I.real], [I.imag]]) - np.array([[self.I_equilibrium.real], [self.I_equilibrium.imag]])
        diff_V = np.array([[V.real], [V.imag]]) - np.array([[self.V_equilibrium.real], [self.V_equilibrium.imag]])
        constraint = D@u + DI@diff_I + DV@diff_V
        return [dx, constraint]

    def get_nu(self):
        return 2

    def set_power(self, Veq, Ieq):
        PQ = Veq * Ieq.conjugate()
        self.P_st = PQ.real
        self.Q_st = PQ.imag

    def get_linear_matrix_(self, V, x=None):
        if x == None:
            [A, B, C, D, BV, DV, BI, DI, R, S] = self.get_linear_matrix_(V=self.V_equilibrium)
        else:
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

        return [A, B, C, D, BV, DV, BI, DI, R, S]

