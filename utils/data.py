from typing import *
from numpy.typing import *
import numpy as np

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
    
    raise TypeError(f'{repr(obj)}({type(obj).__name__}) does not have a supported data type.')


def expand_complex_arr(arr: NDArray[np.number], axis=0):
    assert isinstance(arr, np.ndarray)
    
    shape = list(arr.shape)
    axis = axis % len(shape)
    if axis < 0:
        axis += len(shape)
        
    shape[axis] *= 2
    ret = np.zeros(shape, dtype=np.float64)
    
    slices_real = tuple([slice(0, None)] * axis + [slice(0, None, 2), Ellipsis])
    slices_imag = tuple([slice(0, None)] * axis + [slice(1, None, 2), Ellipsis])
    
    ret[slices_real] = np.real(arr)
    ret[slices_imag] = np.imag(arr)
    
    return ret
    