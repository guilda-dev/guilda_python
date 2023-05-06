from dataclasses import dataclass


@dataclass
class BusParameters:
    No: int = -1
    type: int = -1
    
    V_abs: float = 0
    V_angle: float = 0
    V_angle_deg: float = 0
    
    P_gen: float = 0
    Q_gen: float = 0
    P_load: float = 0
    Q_load: float = 0
    
    G_shunt: float = 0
    B_shunt: float = 0
    
    def __post_init__(self):
        self.No = int(self.No)
        self.type = int(self.type)