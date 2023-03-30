import numpy as np

from guilda.utils.typing import ComplexArray, FloatArray


def complex_square_mat_to_float(m: ComplexArray) -> FloatArray:
    """_summary_

    Args:
        m (ComplexArray): _description_

    Returns:
        FloatArray: _description_
    """  
    n = m.shape[0]
    r: FloatArray = np.zeros((2*n, 2*n))
    r[ ::2, ::2] =  m.real
    r[ ::2,1::2] = -m.imag
    r[1::2, ::2] =  m.imag
    r[1::2,1::2] =  m.real
    return r