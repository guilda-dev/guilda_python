from typing import Hashable
import numpy as np

from guilda.bus.bus import Bus



class BusPQ(Bus):
    def __init__(self, P: float, Q: float, shunt: complex, index: Hashable = None):
        super().__init__(shunt, index=index)
        self.P: float = P
        self.Q: float = Q

    def get_constraint(self, Vr: float, Vi: float, P: float, Q: float):
        return np.array([
            [P-self.P],
            [Q-self.Q]
        ])
