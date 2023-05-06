from dataclasses import dataclass


@dataclass
class BranchParameters:
    bus_from: int = 0
    bus_to: int = 0
    
    x_real: float = 0
    x_imag: float = 0
    y: float = 0
    y_: float = 0
    
    tap: float = 0
    phase: float = 0
    Vprime: float = 0
    Vsecond: float = 0
    
    def __post_init__(self):
        self.bus_from = int(self.bus_from)
        self.bus_to = int(self.bus_to)