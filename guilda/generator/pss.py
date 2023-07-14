import numpy as np
from control import StateSpace as SS

from typing import Union, Tuple, List

from guilda.generator.types import PssParameters
from guilda.utils.typing import FloatArray


class Pss():
    '''    モデル：PSSの実装モデル
            発電機モデルに付加するために実装されたクラス
    親クラス：handleクラス
    実行方法：obj = pss(parameter)
     引数：parameter : pandas.Series型．「'Kpss','Tpss','TL1p','TL1','TL2p','TL2'」を列名として定義
     出力：pssクラスのインスタンス
    '''
    def __init__(self, pss_in: Union[PssParameters, SS, None] = None):

        self.A: FloatArray = np.zeros((0, 0))
        self.B: FloatArray = np.zeros((0, 1))
        self.C: FloatArray = np.zeros((1, 0))
        self.D: FloatArray = np.zeros([1, 1])
        self._nx: int = 0

        if isinstance(pss_in, PssParameters) or isinstance(pss_in, SS):
            self.set_pss(pss_in)
        else:
            assert pss_in is None, "pss_inのパラメータが不足しています"

        sys = SS(self.A, self.B, self.C, self.D)
        SS.set_inputs(sys, ['omega'])
        SS.set_outputs(sys, ['v_pss'])
        self.set_pss(sys)
        self.sys: SS = sys

    @property
    def nx(self) -> int:
        return self._nx

    def get_state_name(self) -> List[str]:
        nx = self.nx
        name_tag = []
        if nx != 0:
            name_tag = ['xi' + str(i+1) for i in range(nx)]
        return name_tag

    def get_u(self, x_pss: FloatArray, omega: float) -> Tuple[FloatArray, FloatArray]:
        dx = self.A @ x_pss + self.B * omega
        u = self.C @ x_pss + self.D * omega
        return dx, u

    def initialize(self) -> FloatArray:
        x: FloatArray = np.zeros((self.nx, 1))
        return x

    def get_sys(self) -> SS:
        return self.sys

    def set_pss(self, pss: Union[PssParameters, SS]) -> None:
        if isinstance(pss, PssParameters):

            Kpss, Tpss, TL1p, TL1, TL2p, TL2 = \
                pss.Kpss, pss.Tpss, pss.TL1p, pss.TL1, pss.TL2p, pss.TL2

            self.A = np.array(
                [
                    [-1/Tpss, 0., 0.],
                    [-Kpss*(1-TL1p/TL1)/(Tpss*TL1), -1/TL1, 0.],
                    [-(Kpss*TL1p)*(1-TL2p/TL2)/(Tpss*TL1*TL2),
                     (1-TL2p/TL2)/TL2, -1/TL2]
                ])  # (3, 3)

            self.B = np.array(
                [
                    [1/Tpss],
                    [Kpss*(1-TL1p/TL1)/(Tpss*TL1)],
                    [(Kpss*TL1p)*(1-TL2p/TL2)/(Tpss*TL1*TL2)]
                ])  # (3, 1)

            self.C = np.array(
                [
                    [-(Kpss*TL1p*TL2p)/(Tpss*TL1*TL2), TL2p/TL2, 1.],
                ])  # (1, 3)

            self.D = np.array(
                [
                    [(Kpss*TL1p*TL2p)/(Tpss*TL1*TL2)]
                ])  # (1, 1)
        else:
            # pssが状態空間表現SSオブジェクトであると仮定する
            self.A, self.B, self.C, self.D = pss.A, pss.B, pss.C, pss.D  # type: ignore

        self._nx = np.shape(self.A)[0]
