import numpy as np
from numpy import exp


class Branch(object):
    
    def __init__(self, from_, to) -> None:
        self.from_ = int(from_)
        self.to = int(to)


