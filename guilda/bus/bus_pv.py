from typing import Hashable

import numpy as np
from numpy.linalg import norm

from guilda.bus.bus import Bus

class BusPV(Bus):
    def __init__(self, P: float, V_abs: float, shunt: complex, index: Hashable = None):
        super().__init__(shunt, index=index)
        self.P: float = P
        self.V_abs: float = V_abs

    def get_constraint(self, Vr: float, Vi: float, P: float, Q: float):
        V_abs = norm([Vr, Vi])
        return np.array([
            [P-self.P],
            [V_abs-self.V_abs]
        ])
