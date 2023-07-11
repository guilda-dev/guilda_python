from control import StateSpace as SS
import numpy as np

from guilda.utils.typing import FloatArray

class Avr():
    def __init__(self):
        self.Vfd_st: complex = 0
        self.Vabs_st: float = 0

        # ここではavrのダイナミクスを考慮しない
        A = np.array([])
        B = np.zeros((0, 3))
        C = np.zeros((1, 0))
        D = np.array([0, 0, 1])
        sys = SS(A, B, C, D)
        SS.set_inputs(sys, ['Vabs', 'Efd', 'u_avr'])
        SS.set_outputs(sys, ['Vfd'])

        self.sys = sys

    @property
    def nx(self) -> int:
        # このavrには状態がない
        return 0

    def initialize(self, Vfd: complex, Vabs: float):
        self.Vfd_st = Vfd
        self.Vabs_st = Vabs
        # 状態がない
        x = np.zeros((0, 1))
        return x

    def get_Vfd(self, u: FloatArray, x_avr=None, Vabs=None, Efd=None):
        Vfd = self.Vfd_st + u[0, 0]
        dx = np.zeros((0, 1))
        return dx, Vfd

    def get_sys(self):
        sys = self.sys
        return sys

    def get_state_name(self):
        nx = self.nx
        name_tag = []
        if nx != 0:
            name_tag = ['state_avr' + str(i+1) for i in range(nx)]
        return name_tag
