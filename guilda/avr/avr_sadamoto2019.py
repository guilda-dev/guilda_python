from control import StateSpace as SS
import numpy as np
import pandas as pd

from guilda.avr.avr import Avr

class AvrSadamoto2019(Avr):
# モデル  ：定本先生が2019年の論文で紹介されたモデル
#親クラス：avrクラス
#実行方法：AvrSadamoto2019(avr_pd)
# 引数 ：・avr_pd：pandas.Series型の変数。「'Te','Ka'」を列名として定義
# 出力 ：avrクラスの変数
    def __init__(self, avr_pd):
        self.Vfd_st = None
        self.Vabs_st = None

        self.Te = avr_pd['Te']
        self.Ka = avr_pd['Ka']
        [A, B, C, D] = self.get_linear_matrix()
        sys = SS(A, B, C, D)
        SS.set_inputs(sys, ['Vabs', 'Efd', 'u_avr'])
        SS.set_outputs(sys, ['Vfd'])
        self.sys = sys

    def get_state_name(self):
        return ['Vfield']

    def get_nx(self):
        return 1

    def initialize(self, Vfd, Vabs):
        self.Vfd_st = Vfd
        self.Vabs_st = Vabs
        x = np.array([Vfd]).reshape(-1, 1)
        return x

    def get_Vfd(self, Vfd, Vabs, u, Efd=None):
        Vef = self.Ka*(Vabs - self.Vabs_st + u)
        dVfd = (-Vfd + self.Vfd_st - Vef)/self.Te
        return [dVfd, Vfd]

    def get_Vfd_linear(self, Vfd, Vabs, u, Efd=None):
        return self.get_Vfd(self, Vfd=Vfd, Vabs=Vabs, u=u, Efd=Efd)

    def get_linear_matrix(self):
        A = np.array(-1/self.Te).reshape(1, 1)
        B = -self.Ka/self.Te * np.array([[1, 0, 1]])
        C = np.array(1).reshape(1, 1)
        D = np.zeros([1, 3])
        return [A, B, C, D]

    def get_sys(self):
        return self.sys
