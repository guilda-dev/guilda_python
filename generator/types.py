from dataclasses import dataclass

@dataclass
class GeneratorParameters:
    Xd: float = 0
    Xd_prime: float = 0
    Xq: float = 0
    T: float = 0
    M: float = 0
    D: float = 0