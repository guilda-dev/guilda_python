# type:ignore

import numpy as np
from numpy.typing import NDArray
from typing import Iterable

from guilda.backend.protocol import ArrayProtocol
from guilda.backend.types import BitLenType, ArrayLike, KernelType, ShapeLike, ShapeLike1, ShapeLike2, NumberLike

__dtype_selector = {
    16: np.float16,
    32: np.float32,
    64: np.float64,
    128: np.float64,
}

__dtype_selector_c = {
    16: np.complex64,
    32: np.complex64,
    64: np.complex128,
    128: np.complex128,
}

class NumPyArray(np.ndarray, ArrayProtocol):
    
    def __new__(cls, o):
        return o
    
_f = lambda x: NumPyArray(x)

def __rvec(object: ArrayLike, *args, **kwargs):
    ret = np.array(object, *args, **kwargs)
    return ret.reshape((1, ret.size))

def __cvec(object: ArrayLike, *args, **kwargs):
    ret = np.array(object, *args, **kwargs)
    return ret.reshape((ret.size, 1))

def init_numpy(bitlen: BitLenType, p: 'guilda.backend.protocol'):
    p.dtype = __dtype_selector.get(bitlen, np.float64)
    p.dtype_c = __dtype_selector_c.get(bitlen, np.complex128)

    p.array = np.array
    p.zeros = np.zeros
    p.ones = np.ones
    p.identity = np.identity
    p.diag = np.diag
    p.hstack = np.hstack
    p.vstack = np.vstack
    p.dstack = np.dstack
    p.concatenate = np.concatenate
    p.inv = np.linalg.inv
    p.arange = np.arange
    
    p.rvec = __rvec
    p.cvec = __cvec
    
    p.asnp = lambda x: x
    