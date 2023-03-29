import pandas as pd
import numpy as np
from control import StateSpace as SS

class Pss():
# モデル ：PSSの実装モデル
#         発電機モデルに付加するために実装されたクラス
#親クラス：handleクラス
#実行方法：obj = pss(parameter)
#　引数　：parameter : pandas.Series型．「'Kpss','Tpss','TL1p','TL1','TL2p','TL2'」を列名として定義
#　出力　：pssクラスのインスタンス
    def __init__(self, pss_in=None):
        self.A = None
        self.B = None
        self.C = None
        self.D = None
        self.nx = 0

        if pss_in == None:
            A = np.array([])
            B = np.array([]).reshape(-1, 1)
            C = np.array([]).reshape(1, -1)
            D = np.zeros([1, 1])
            sys = SS(A, B, C, D)
            SS.set_inputs(sys, ['omega'])
            SS.set_outputs(sys, ['v_pss'])
            self.set_pss(sys)
            self.sys = sys
        else:
            try:
                self.set_pss(pss_in)
                sys = SS(self.A, self.B, self.C, self.D)
                SS.set_inputs(sys, ['omega'])
                SS.set_outputs(sys, ['v_pss'])
                self.sys = sys
            except:
                print("pss_inのパラメータが不足しています")

    def get_nx(self) -> int:
        return self.nx
    
    def get_state_name(self):
        nx = self.get_nx()
        name_tag = []
        if nx != 0:
            name_tag = ['xi' + str(i+1) for i in range(nx)]
        return name_tag
    
    def get_u(self, x_pss, omega):
        dx = self.A@x_pss + self.B@omega
        u = self.C@x_pss + self.D@omega
        return [dx, u]
    
    def initialize(self):
        x = np.zeros([self.get_nx(), 1])
        return x

    def get_sys(self):
        sys = self.sys
        return sys
    
    def set_pss(self, pss):
        if type(pss) == pd.core.series.Series:
            Kpss = pss['Kpss']
            Tpss = pss['Tpss']
            TL1p = pss['TL1p']
            TL1 = pss['TL1']
            TL2p = pss['TL2p']
            TL2 = pss['TL2']

            self.A = np.array([
                    [-1/Tpss, 0, 0],
                    [-Kpss*(1-TL1p/TL1)/(Tpss*TL1), -1/TL1, 0],
                    [-(Kpss*TL1p)*(1-TL2p/TL2)/(Tpss*TL1*TL2), (1-TL2p/TL2)/TL2, -1/TL2]
                    ])

            self.B = np.array([
                    [1/Tpss],
                    [Kpss*(1-TL1p/TL1)/(Tpss*TL1)],
                    [(Kpss*TL1p)*(1-TL2p/TL2)/(Tpss*TL1*TL2)]
                    ])

            self.C = np.array([-(Kpss*TL1p*TL2p)/(Tpss*TL1*TL2), TL2p/TL2, 1]).reshape(1, -1)
            self.D = np.array([(Kpss*TL1p*TL2p)/(Tpss*TL1*TL2)]).reshape(1, -1)
        else:
            # pssが状態空間表現SSオブジェクトであると仮定する
            self.A, self.B, self.C, self.D = pss.A, pss.B, pss.C, pss.D
        self.nx = self.A.shape[0]
