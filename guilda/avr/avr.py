from control import StateSpace as SS
import numpy as np

class Avr():
    def __init__(self):
        self.Vfd_st = None
        self.Vabs_st = None

        # ここではavrのダイナミクスを考慮しない
        A = np.array([])
        B = np.array([]).reshape(-1, 3)
        C = np.array([]).reshape(1, -1)
        D = np.array([0, 0, 1])
        sys = SS(A, B, C, D)
        SS.set_inputs(sys, ['Vabs', 'Efd', 'u_avr'])
        SS.set_outputs(sys, ['Vfd'])

        self.sys = sys

    def get_nx(self) -> int:
        # このavrには状態がない
        return 0

    def initialize(self, Vfd, Vabs):
        self.Vfd_st = Vfd
        self.Vabs_st = Vabs
        # 状態がない
        x = np.array([]).reshape(-1, 1)
        return x

    def get_Vfd(self, u, x_avr=None, Vabs=None, Efd=None):
        Vfd = self.Vfd_st + u
        dx = []
        return [dx, Vfd]

    def get_sys(self):
        sys = self.sys
        return sys

    def get_state_name(self):
        nx = self.get_nx()
        name_tag = []
        if nx != 0:
            name_tag = ['state_avr' + str(i+1) for i in range(nx)]
        return name_tag
