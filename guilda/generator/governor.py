from control import StateSpace as SS
import numpy as np

from guilda.backend import ArrayProtocol

class Governor(object):
    
    def __init__(self):
        self.P: float = 0
        
        # ここではgovernorのダイナミクスを考慮しない
        A: ArrayProtocol = np.zeros((0, 0))
        B: ArrayProtocol = np.zeros((0, 1))
        C: ArrayProtocol = np.zeros((1, 0))
        D: ArrayProtocol = np.identity(2)
        sys = SS(A, B, C, D)
        SS.set_inputs(sys, ['omega_governor', 'u_governor'])
        SS.set_outputs(sys, ['omega_governor', 'Pmech'])

        self.sys: SS = sys

    def initialize(self, P: float) -> ArrayProtocol:
        self.P = P
        # governorに状態はない
        x: ArrayProtocol = np.zeros((0, 1))
        return x

    @property
    def nx(self) -> int:
        return 0

    def get_P(self, x: ArrayProtocol, u: ArrayProtocol):
        P: float = self.P + u[0, 0]
        dx: ArrayProtocol = np.zeros((0, 1))
        return (dx, P)

    def get_sys(self) -> SS:
        return self.sys
    
    def get_state_name(self):
        nx = self.nx
        name_tag = []
        if nx != 0:
            name_tag = ['state_governor' + str(i+1) for i in range(nx)]
        return name_tag
