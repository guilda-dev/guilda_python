from typing import Set, Optional

from guilda.backend.types import BitLenType, KernelType


import guilda.backend.bn_numpy as __np

__valid_kernels: Set[KernelType] = {'numpy', 'sympy', 'torch', 'cupy'}
    

__valid_bitlens: Set[BitLenType] = {16, 32, 64}

__kernel: KernelType = 'numpy'

__bitlen: BitLenType = 64

def set_config(kernel: Optional[KernelType] = None, bitlen: Optional[BitLenType] = None):
    
    global __kernel, __bitlen
    
    if kernel is not None and kernel not in __valid_kernels:
        raise ValueError(f"Invalid kernel: {kernel}. Valid options are {__valid_kernels}.")
    
    if bitlen is not None and bitlen not in __valid_bitlens:
        raise ValueError(f"Invalid bitlen: {bitlen}. Valid options are {__valid_bitlens}.")
    
    changed = False
    
    if kernel is not None:
        changed = True
        __kernel = kernel
    if bitlen is not None:
        changed = True
        __bitlen = bitlen
        
    init_by_config()
        
def get_config() :
    return __kernel, __bitlen


def init_by_config():
    import guilda.backend as p
    if __kernel == 'numpy':
        __np.init_numpy(__bitlen, p)
