from typing import Hashable

import numpy as np
from numpy.linalg import norm
from cmath import phase

from guilda.bus.bus import Bus


class BusSlack(Bus):
    def __init__(self, Vabs: float, Vangle: float, shunt: complex, index: Hashable = None):
        super().__init__(shunt, index=index)
        self.Vabs: float = Vabs
        self.Vangle: float = Vangle

    def get_constraint(self, Vr: float, Vi: float, P: float, Q: float):
        Vabs = norm([Vr, Vi])
        Vangle = phase(complex(Vr, Vi))
        return np.array([
            [Vabs-self.Vabs], 
            [Vangle-self.Vangle]
        ])
