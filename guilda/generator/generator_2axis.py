from typing import List, Optional, Tuple, Union
import numpy as np
from math import sin, cos, atan, atan2, sqrt
from cmath import phase
import control as ct
from control import StateSpace as SS

from guilda.base import StateEquationRecord
from guilda.utils import complex_to_col_vec
from guilda.utils.typing import FloatArray

from guilda.generator.generator import Generator


class Generator2Axis(Generator):
    '''
モデル：同期発電機の2軸モデル
        ・状態：4つ「回転子偏角"δ",周波数偏差"Δω",内部電圧"E"」
              *AVRやPSSが付加されるとそれらの状態も追加される
        ・入力：２ポート「界磁入力"Vfield", 機械入力"Pmech"」
              *定常値からの追加分を指定
親クラス：componentクラス
実行方法：obj = generator_1axis(omega, parameter)
    引数：・omega: float値．系統周波数(50or60*2pi)
     ・parameter : pandas.Series型．「'Xd', 'Xd_prime','Xq','T','M','D'」を列名として定義
    出力：componentクラスのインスタンス

    Args:
        Component (_type_): _description_
    '''

    def get_self_x_name(self) -> List[str]:
        return super().get_self_x_name() + ['Eq', 'Ed']

    @property
    def nx_gen(self):
        return 4

    def get_dx_constraint(
        self,
        V: complex = 0,
        I: complex = 0,
        x: Optional[FloatArray] = None,
        u: Optional[FloatArray] = None,
            t: float = 0) -> Tuple[FloatArray, FloatArray]:

        assert x is not None
        assert u is not None

        Xd = self.parameter.Xd
        Xdp = self.parameter.Xd_prime
        Xq = self.parameter.Xq
        Xqp = self.parameter.Xq_prime
        Tdo = self.parameter.Tdo
        Tqo = self.parameter.Tqo
        M = self.parameter.M
        d = self.parameter.D

        Vabs = abs(V)
        Vangle = atan2(V.imag, V.real)

        delta, omega, Eq, Ed = x[:, 0]

        # Vd, Vqを定義
        Vq = V.real*cos(delta) + V.imag*sin(delta)
        Vd = V.real*sin(delta) - V.imag*cos(delta)

        # Id, Iqを定義
        Iq = -(Ed-Vd)/Xqp
        Id = (Eq-Vq)/Xdp

        # |I|cosI, |I|sinIを逆算
        Ir = Iq*cos(delta)+Id*sin(delta)
        Ii = Iq*sin(delta)-Id*cos(delta)

        con = np.array([[I.real - Ir], [I.imag - Ii]])

        # Efdの修正とEfqの追加
        Efd = Xd*Eq/Xdp - (Xd/Xdp-1)*Vq
        Efq = Xq*Ed/Xqp - (Xq/Xqp-1)*Vd

        # スカラーを返す
        dx_avr, dx_pss, dx_gov, \
            Vfd, P = self.get_components_dx(x, u, omega, Vabs, Efd)

        dEq = (-Efd + Vfd)/Tdo
        dEd = (-Efq)/Tqo
        dDelta: float = self.omega0*omega
        dOmega: float = (P - d*omega - Vq*Iq - Vd*Id)/M

        dx_gen = [[dDelta], [dOmega], [dEq], [dEd]]

        dx = np.vstack((dx_gen, dx_avr, dx_pss, dx_gov))

        return dx, con

    def get_self_equilibrium(self, V: complex, I: complex):

        Vangle = phase(V)
        Vabs = abs(V)
        Iangle = phase(I)
        Iabs = abs(I)

        Pow = I.conjugate() * V
        P = Pow.real
        Q = Pow.imag

        Xd = self.parameter.Xd
        Xdp = self.parameter.Xd_prime
        Xq = self.parameter.Xq
        Xqp = self.parameter.Xq_prime

        delta = Vangle + atan(P/(Q+Vabs**2/Xq))
        Eqnum = P**2*Xdp*Xq + Q**2*Xdp*Xq + Vabs**2*Q*Xq + Vabs**2*Q*Xdp + Vabs**4
        Eqden = Vabs*sqrt(P**2*Xq**2 + Q**2*Xq**2 + 2*Vabs**2*Q*Xq + Vabs**4)
        Eq = Eqnum/Eqden
        Ednum = (Xq-Xqp)*Vabs*P
        Edden = sqrt(P**2*Xq**2 + Q**2*Xq**2 + 2*Vabs**2*Q*Xq + Vabs**4)
        Ed = Ednum/Edden
        Vfd = Eq + (Xd-Xdp)*Iabs*sin(delta-Iangle)

        x_gen = np.array([[delta], [0], [Eq], [Ed]])

        return x_gen, Vfd

    # TODO get linear matrix