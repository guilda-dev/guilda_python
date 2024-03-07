from typing import Hashable

import numpy as np
from numpy.linalg import norm

from guilda.bus.bus import Bus

class BusPV(Bus):
    def __init__(self, P: float, Vabs: float, shunt: complex, index: Hashable = None):
        super().__init__(shunt, index=index)
        self.P: float = P
        self.Vabs: float = Vabs

    def get_constraint(self, Vr: float, Vi: float, P: float, Q: float):
        Vabs = norm([Vr, Vi])
        return np.array([
            [P-self.P],
            [Vabs-self.Vabs]
        ])
