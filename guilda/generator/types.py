from dataclasses import dataclass

@dataclass
class GeneratorParameters:
    Xd: float = 0
    Xd_prime: float = 0
    Xq: float = 0
    T: float = 0
    M: float = 0
    D: float = 0
    
@dataclass
class PssParameters:
    Kpss: float = 0
    Tpss: float = 0
    TL1p: float = 0
    TL1: float = 0
    TL2p: float = 0
    TL2: float = 0