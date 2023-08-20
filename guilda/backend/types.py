from typing import Union, Literal, Any, Iterable, Optional, Tuple
from numpy.typing import ArrayLike
from numpy import floating, complexfloating
from abc import ABC, abstractmethod as AM

KernelType = Literal['numpy', 'sympy', 'torch', 'cupy']
BitLenType = Literal[16, 32, 64, 128]

ShapeLike = Tuple[int, ...]
ShapeLike1 = Tuple[int]
ShapeLike2 = Tuple[int, int]

NumberLike = Union[bool, int, float, complex, floating, complexfloating]

