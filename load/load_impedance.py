from base.component import Component

import numpy as np


class LoadImpedance(Component):
    # モデル ：定インピーダンス付加モデル
    #       ・状態：なし
    #       ・入力：２ポート「インピーダンス値の実部の倍率,インピーダンス値の虚部の倍率」
    #               *入力αのときインピーダンスの値は設定値の(1+α)倍となる．
    #親クラス：componentクラス
    #実行方法：obj = load_impedance()
    #　引数　：なし
    #　出力　：componentクラスのインスタンス
    def __init__(self, *args):
        self.x_equilibrium = np.zeros([0, 1])
        self.S = np.array([]).reshape(1, -1)
        self.R = np.zeros([0, 0])

        self.V_equilibrium = None
        self.I_equilibrium = None
        self.Y = None
        self.Y_mat = None

    def set_equilibrium(self, v: complex, i: complex) -> None:
        self.V_equilibrium = v
        self.I_equilibrium = i
        self.set_admittance(i / v)

    def complex2matrix(self, y):
        r = y.real
        c = y.imag
        return np.array([[r, -c], [c, r]])

    def get_dx_constraint(self, V, I, u, t=None, x=None):
        dx = np.zeros([0, 1])
        Y = (self.Y).real*(1+u[0, 0]) + 1j*(self.Y).imag*(1+u[1, 0])
        I_ = Y*V
        constraint = np.array([[I.real], [I.imag]]) - \
            np.array([[I_.real], [I_.imag]])
        return [dx, constraint]

    def get_nu(self):
        return 2

    def set_admittance(self, y):
        self.Y = y
        self.Y_mat = self.complex2matrix(y)

    def get_linear_matrix(self, V=None, x=None):
        if x == None:
            [A, B, C, D, BV, DV, BI, DI, R, S] = self.get_linear_matrix(
                V=self.V_equilibrium, x=[])
            return [A, B, C, D, BV, DV, BI, DI, R, S]

        A = np.zeros([0, 0])
        B = np.zeros([0, 2])
        C = np.zeros([2, 0])
        D1 = self.complex2matrix(self.Y.real) @ np.array([[V.real], [V.imag]])
        D2 = self.complex2matrix(
            1j*(self.Y.imag)) @ np.array([[V.real], [V.imag]])
        D = np.hstack([D1, D2])
        BV = np.zeros([0, 2])
        DV = self.Y_mat
        R = self.R
        S = self.S
        BI = np.zeros([0, 2])
        DI = -np.identity(2)
        return [A, B, C, D, BV, DV, BI, DI, R, S]
