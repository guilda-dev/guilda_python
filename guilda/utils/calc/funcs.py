import numpy as np

from guilda.backend import ArrayProtocol, ArrayProtocol


def complex_mat_to_float(m: ArrayProtocol) -> ArrayProtocol:
    '''_summary_

    Args:
        m (ArrayProtocol): _description_

    Returns:
        ArrayProtocol: _description_
    '''  
    n, p = m.shape
    r: ArrayProtocol = np.zeros((2*n, 2*p))
    r[ ::2, ::2] =  m.real
    r[ ::2,1::2] = -m.imag
    r[1::2, ::2] =  m.imag
    r[1::2,1::2] =  m.real
    return r