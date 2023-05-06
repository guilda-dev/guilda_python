from dataclasses import dataclass

@dataclass
class MachineParameters:
    
    No_machine: int = -1
    No_bus: int = -1
    
    Xd: float = 0
    Xd_prime: float = 0
    
    Xq: float = 0
    Xq_prime: float = 0
    
    T: float = 0
    Tdo: float = 0
    Tqo: float = 0
    
    H: float = 0
    
    M: float = 0
    
    D: float = 0
    
    
    def __post_init__(self):
        self.No_bus = int(self.No_bus)
        self.No_machine = int(self.No_machine)
    
@dataclass
class PssParameters:
    
    No_bus: int = -1
    
    Kpss: float = 0
    Tpss: float = 0
    
    TL1p: float = 0
    TL1: float = 0
    
    TL2p: float = 0
    TL2: float = 0
    
    def __post_init__(self):
        self.No_bus = int(self.No_bus)