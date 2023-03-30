from bus.bus import Bus
import numpy as np
from numpy.linalg import norm


class BusPV(Bus):
    def __init__(self, P: float, Vabs: float, shunt):
        super().__init__(shunt)
        self.P: float = P
        self.Vabs: float = Vabs

    def get_constraint(self, Vr: float, Vi: float, P: float, Q: float):
        Vabs = norm([Vr, Vi])
        return np.array([
            [P-self.P],
            [Vabs-self.Vabs]
        ])
