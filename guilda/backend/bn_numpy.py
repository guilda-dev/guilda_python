import numpy as np
from typing import Iterable

from guilda.backend.types import BitLenType, GuildaBackendProtocol, ArrayLike, ArrayProtocol, ShapeLike, ShapeLike1, ShapeLike2, NumberLike

__dtype_selector = {
  16: np.float16,
  32: np.float32,
  64: np.float64,
  128: np.float128,
}

__dtype_selector_c = {
  16: np.complex64,
  32: np.complex64,
  64: np.complex128,
  128: np.complex256,
}

class NumPyArray(ArrayProtocol):
    
    def __new__(cls, o):
        return o
    
_f = lambda x: NumPyArray(x)

class NumPyProtocol(GuildaBackendProtocol):
    
    def __init__(self, bitlen: BitLenType) -> None:
        super().__init__(bitlen)
        self.__dtype = __dtype_selector.get(bitlen, np.float64)
        self.__dtype_c = __dtype_selector_c.get(bitlen, np.complex128)
        
    
    def array(self, object: ArrayLike):
        return np.array(object, dtype=self.__dtype_c if np.iscomplexobj(object) else self.__dtype)
    
    
    def zeros(self, shape: ShapeLike):
        return np.zeros(shape, dtype=self.__dtype)
    
    
    def ones(self, shape: ShapeLike):
        return np.ones(shape, dtype=self.__dtype)
    
    
    def identity(self, shape: int):
        return np.identity(shape, dtype=self.__dtype)
    
    
    def rvec(self, object: Iterable[NumberLike]):
        pass
    
    
    def cvec(self, object: Iterable[NumberLike]):
        pass
    
    
    def diag(self, object: Iterable[NumberLike]):
        pass
    
    
    def hstack(self, tup: Iterable[ArrayLike]):
        pass
    
    
    def vstack(self, tup: Iterable[ArrayLike]):
        pass
    
    
    def dstack(self, tup: Iterable[ArrayLike]):
        pass
    
    
    def concatenate(self, tup: Iterable[ArrayLike], axis: int = -1):
        pass
    
    
    def inv(self, object: ArrayLike):
        return np.linalg.inv(object) # type: ignore
    
    
    def arange(self, start: NumberLike, stop: NumberLike, step: NumberLike):
        pass