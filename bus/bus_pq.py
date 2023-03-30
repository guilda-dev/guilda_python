from bus.bus import Bus
import numpy as np


class BusPQ(Bus):
    def __init__(self, P: float, Q: float, shunt):
        super().__init__(shunt)
        self.P: float = P
        self.Q: float = Q

    def get_constraint(self, Vr: float, Vi: float, P: float, Q: float):
        return np.array([
            [P-self.P],
            [Q-self.Q]
        ])
