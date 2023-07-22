from dataclasses import dataclass
from control import StateSpace as SS
import numpy as np
import pandas as pd

from guilda.avr.avr import Avr
from guilda.utils.typing import FloatArray


@dataclass
class AvrSadamoto2019Parameters:
    No_bus: int = -1
    
    Te: float = 0
    Ka: float = 0
    
    def __post_init__(self):
        self.No_bus = int(self.No_bus)

class AvrSadamoto2019(Avr):
    '''
    モデル：定本先生が2019年の論文で紹介されたモデル
    親クラス：avrクラス
    実行方法：AvrSadamoto2019(avr_pd)
    引数：・avr_pd：pandas.Series型の変数。「'Te','Ka'」を列名として定義
    出力：avrクラスの変数
    '''
    
    def __init__(self, avr_pd: AvrSadamoto2019Parameters):
        self.Te: float = avr_pd.Te
        self.Ka: float = avr_pd.Ka
        super().__init__()

        

    def get_state_name(self):
        return ['Vfield']

    @property
    def nx(self):
        return 1

    def initialize(self, Vfd: complex, Vabs: float):
        self.Vfd_st = Vfd
        self.Vabs_st = Vabs
        x = np.array([[Vfd]])
        return x

    def get_Vfd(self, u: FloatArray, x_avr: FloatArray, Vabs: float, Efd: complex):
        Vfd: complex = x_avr[0,0]
        Vef: float = self.Ka*(Vabs - self.Vabs_st + u[0, 0])
        dVfd = (-Vfd + self.Vfd_st - Vef)/self.Te
        return np.array([[dVfd]]), Vfd

    def get_Vfd_linear(self, u: FloatArray, x_avr: FloatArray, Vabs: float, Efd: complex):
        return self.get_Vfd(u, x_avr, Vabs, Efd)

    def get_linear_matrix(self):
        A = np.array(-1/self.Te).reshape(1, 1)
        B = -self.Ka/self.Te * np.array([[1, 0, 1]])
        C = np.identity(1)
        D = np.zeros([1, 3])
        return A, B, C, D

