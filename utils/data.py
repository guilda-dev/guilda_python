import numpy as np
from numpy.typing import NDArray
from typing import TypeVar, List

from utils.typing import FloatArray


def convert_to_complex(obj) -> complex:
    if isinstance(obj, complex):
        return obj
    elif isinstance(obj, (tuple, list)):
        return complex(
            obj[0] if len(obj) > 0 else 0,
            obj[1] if len(obj) > 1 else 0
        )
    elif isinstance(obj, (int, float)):
        return complex(obj, 0)
    elif isinstance(obj, np.number):
        return complex(obj)

    raise TypeError(
        f'{repr(obj)}({type(obj).__name__}) does not have a supported data type.')


def expand_complex_arr(arr: NDArray[np.number], axis=0):
    assert isinstance(arr, np.ndarray)

    shape = list(arr.shape)
    axis = axis % len(shape)
    if axis < 0:
        axis += len(shape)

    shape[axis] *= 2
    ret = np.zeros(shape, dtype=np.float64)

    slices_real = tuple([slice(0, None)] * axis +
                        [slice(0, None, 2), Ellipsis])
    slices_imag = tuple([slice(0, None)] * axis +
                        [slice(1, None, 2), Ellipsis])

    ret[slices_real] = np.real(arr)
    ret[slices_imag] = np.imag(arr)

    return ret


T = TypeVar('T')


def sep_list(lst: List[T], lens: List[int]) -> List[List[T]]:
    idx = 0
    ret: List[List[T]] = []
    for len_ in lens:
        cells = lst[idx: idx + len_]
        idx += len_
        ret.append(cells)
    return ret


def complex_to_matrix(z: complex) -> FloatArray:
    r = z.real
    c = z.imag
    return np.array([[r, -c], [c, r]])


def complex_to_col_vec(z: complex) -> FloatArray:
    r = z.real
    c = z.imag
    return np.array([[r], [c]])

