from control import StateSpace as SS
import numpy as np

class Governor():
    def __init__(self):
        self.P = None
        
        # ここではgovernorのダイナミクスを考慮しない
        A = np.array([])
        B = np.array([]).reshape(-1, 1)
        C = np.array([]).reshape(1, -1)
        D = np.identity(2)
        sys = SS(A, B, C, D)
        SS.set_inputs(sys, ['omega_governor', 'u_governor'])
        SS.set_outputs(sys, ['omega_governor', 'Pmech'])

        self.sys = sys

    def initialize(self, P):
        self.P = P
        # governorに状態はない
        x = np.array([]).reshape(-1, 1)
        return x

    def get_nx(self) -> int:
        return 0

    def get_P(self, u, x_gov=None, omega=None):
        P = self.P + u
        dx = np.array([])
        return [dx, P]

    def get_sys(self):
        sys = self.sys
        return sys
    
    def get_state_name(self):
        nx = self.get_nx()
        name_tag = []
        if nx != 0:
            name_tag = ['state_governor' + str(i+1) for i in range(nx)]
        return name_tag
