from typing import Union, Literal, Any, Iterable, List, Tuple
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

class GuildaBackendProtocol(ABC):
    
    # cannot change since assigned
    __bitlen: BitLenType
    
    @property
    def bitlen(self):
        return self.__bitlen
    
    def __init__(self, bitlen: BitLenType) -> None:
        super().__init__()
        self.__bitlen = bitlen
        

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

    def shape(self) -> Tuple[int, ...]:
        raise NotImplementedError()

    def reshape(self, *shape: int) -> 'ArrayProtocol':
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
