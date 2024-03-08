from typing import Hashable

import numpy as np
from numpy.linalg import norm
from cmath import phase

from guilda.bus.bus import Bus


class BusSlack(Bus):
    def __init__(self, V_abs: float, V_angle: float, shunt: complex, index: Hashable = None):
        super().__init__(shunt, index=index)
        self.V_abs: float = V_abs
        self.V_angle: float = V_angle

    def get_constraint(self, Vr: float, Vi: float, P: float, Q: float):
        V_abs = norm([Vr, Vi])
        V_angle = phase(complex(Vr, Vi))
        return np.array([
            [V_abs-self.V_abs], 
            [V_angle-self.V_angle]
        ])
