from typing import Union, Literal, Any, Iterable, Optional, Tuple
from numpy.typing import ArrayLike
from abc import ABC, abstractmethod as AM

KernelType = Union[
    Literal['numpy'],
    Literal['sympy'],
    Literal['torch'],
    Literal['cupy']
]

BitLenType = Union[
    Literal[16],
    Literal[32],
    Literal[64]
]

ShapeLike = Tuple[int, ...]
ShapeLike1 = Tuple[int]
ShapeLike2 = Tuple[int, int]

NumberLike = Union[bool, int, float, complex]

class GuildaBackendProtocol(ABC):
    
    # cannot change since assigned
    __bitlen: BitLenType
    
    @property
    def bitlen(self):
        return self.__bitlen
    
    def __init__(self, bitlen: BitLenType) -> None:
        super().__init__()
        self.__bitlen = bitlen
        
    @AM
    def array(self, object: ArrayLike) -> 'ArrayProtocol':
        pass
    
    @AM
    def zeros(self, shape: ShapeLike) -> 'ArrayProtocol':
        pass
    
    @AM
    def ones(self, shape: ShapeLike) -> 'ArrayProtocol':
        pass
    
    @AM
    def identity(self, shape: int) -> 'ArrayProtocol':
        pass
    
    @AM
    def rvec(self, object: Iterable[NumberLike]) -> 'ArrayProtocol':
        pass
    
    @AM
    def cvec(self, object: Iterable[NumberLike]) -> 'ArrayProtocol':
        pass
    
    @AM
    def diag(self, object: Iterable[NumberLike]) -> 'ArrayProtocol':
        pass
    
    @AM
    def hstack(self, tup: Iterable[ArrayLike]) -> 'ArrayProtocol':
        pass
    
    @AM
    def vstack(self, tup: Iterable[ArrayLike]) -> 'ArrayProtocol':
        pass
    
    @AM
    def dstack(self, tup: Iterable[ArrayLike]) -> 'ArrayProtocol':
        pass
    
    @AM
    def concatenate(self, tup: Iterable[ArrayLike], axis: int = -1) -> 'ArrayProtocol':
        pass
    
    @AM
    def inv(self, object: ArrayLike) -> 'ArrayProtocol':
        pass
    
    @AM
    def arange(self, start: NumberLike, stop: NumberLike, step: NumberLike) -> 'ArrayProtocol':
        pass

    

# TODO use PYI syntax

class ArrayProtocol:
    
    def __init__(self, iterable: Iterable) -> None:
        raise NotImplementedError()

    def __getitem__(self, index: int) -> Any:
        raise NotImplementedError()

    def __setitem__(self, index: int, value: Any) -> None:
        raise NotImplementedError()

    def __len__(self) -> int:
        raise NotImplementedError()

    def __iter__(self) -> Iterable:
        raise NotImplementedError()

    def shape(self) -> ShapeLike:
        raise NotImplementedError()

    def reshape(self, *shape: Union[int, ShapeLike]) -> 'ArrayProtocol':
        raise NotImplementedError()

    def flatten(self) -> 'ArrayProtocol':
        raise NotImplementedError()

    def transpose(self) -> 'ArrayProtocol':
        raise NotImplementedError()

    def __add__(self, other: 'ArrayProtocol') -> 'ArrayProtocol':
        raise NotImplementedError()

    def __sub__(self, other: 'ArrayProtocol') -> 'ArrayProtocol':
        raise NotImplementedError()

    def __mul__(self, other: 'ArrayProtocol') -> 'ArrayProtocol':
        raise NotImplementedError()

    def __div__(self, other: 'ArrayProtocol') -> 'ArrayProtocol':
        raise NotImplementedError()

    def dot(self, other: 'ArrayProtocol') -> 'ArrayProtocol':
        raise NotImplementedError()

    def __str__(self) -> str:
        raise NotImplementedError()
