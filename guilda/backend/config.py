from typing import Set, Optional

from guilda.backend.types import BitLenType, KernelType


__valid_kernels: Set[KernelType] = {'numpy', 'sympy', 'torch', 'cupy'}
  

__valid_bitlens: Set[BitLenType] = {16, 32, 64}

__kernel: KernelType = 'numpy'

__bitlen: BitLenType = 64

def set_config(kernel: Optional[KernelType], bitlen: Optional[BitLenType]):
  
  global __kernel, __bitlen
  
  if kernel is not None and kernel not in __valid_kernels:
    raise ValueError(f"Invalid kernel: {kernel}. Valid options are {__valid_kernels}.")
  
  if bitlen is not None and bitlen not in __valid_bitlens:
    raise ValueError(f"Invalid bitlen: {bitlen}. Valid options are {__valid_bitlens}.")
  
  if kernel is not None:
    __kernel = kernel
  if bitlen is not None:
    __bitlen = bitlen
    
def get_config() :
  return __kernel, __bitlen

