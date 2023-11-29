# type:ignore

import torch
from numpy.typing import NDArray
from typing import Iterable

from guilda.backend.protocol import ArrayProtocol
from guilda.backend.types import BitLenType, ArrayLike, KernelType, ShapeLike, ShapeLike1, ShapeLike2, NumberLike

__dtype_selector = {
    16: torch.float16,
    32: torch.float32,
    64: torch.float64,
    128: torch.float64,
}

__dtype_selector_c = {
    16: torch.complex64,
    32: torch.complex64,
    64: torch.complex128,
    128: torch.complex128,
}

class TorchArray(torch.Tensor, ArrayProtocol):
    
    def __new__(cls, o):
        return o
    
_f = lambda x: TorchArray(x)

def __rvec(object: ArrayLike, *args, **kwargs):
    ret = torch.tensor(object, *args, **kwargs)
    return ret.reshape((1, ret.size()[0]))

def __cvec(object: ArrayLike, *args, **kwargs):
    ret = torch.tensor(object, *args, **kwargs)
    return ret.reshape((ret.size()[0], 1))

def __asnp(object: torch.Tensor, *args, **kwargs):
    ret = object.numpy(*args, **kwargs)
    return ret

def init_torch(bitlen: BitLenType, p: 'guilda.backend.protocol'):
    p.dtype = __dtype_selector.get(bitlen, torch.float64)
    p.dtype_c = __dtype_selector_c.get(bitlen, torch.complex128)

    p.array = torch.tensor
    p.zeros = torch.zeros
    p.ones = torch.ones
    p.identity = torch.eye
    p.diag = torch.diag
    p.hstack = torch.hstack
    p.vstack = torch.vstack
    p.dstack = torch.dstack
    p.concatenate = torch.concat
    p.inv = torch.inverse
    p.arange = torch.arange
    
    p.rvec = __rvec
    p.cvec = __cvec
    
    p.asnp = __asnp
    
    
    
    