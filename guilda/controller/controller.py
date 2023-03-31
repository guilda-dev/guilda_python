import numpy as np
from typing import Tuple, List
from abc import ABC, abstractmethod as AM

from guilda.utils.typing import FloatArray

class Controller(ABC):
    """_summary_

    Args:
        ABC (_type_): _description_

    Returns:
        _type_: _description_
    """
    
    def __init__(self, index_input: List[int], index_observe: List[int]):
        self.index_input: List[int] = index_input  
        self.index_observe: List[int] = index_observe 

    @AM
    def get_nx(self) -> int:
        """_summary_

        Returns:
            int: _description_
        """        
        
    @AM
    def get_dx_u(self, t, x, X, V, I, U_global) -> Tuple[FloatArray, FloatArray]:
        """_summary_

        Args:
            t (_type_): _description_
            x (_type_): _description_
            X (_type_): _description_
            V (_type_): _description_
            I (_type_): _description_
            U_global (_type_): _description_

        Returns:
            Tuple[FloatArray, FloatArray]: _description_
        """        
        
    @AM
    def get_dx_u_linear(self, t, x, X, V, I, U_global) -> Tuple[FloatArray, FloatArray]:
        """_summary_

        Args:
            t (_type_): _description_
            x (_type_): _description_
            X (_type_): _description_
            V (_type_): _description_
            I (_type_): _description_
            U_global (_type_): _description_

        Returns:
            Tuple[FloatArray, FloatArray]: _description_
        """        

    def get_dx_u_func(self, linear: bool):
        return self.get_dx_u_linear if linear else self.get_dx_u


    def get_x0(self) -> FloatArray:
        out = np.zeros((self.get_nx(), 1))  # controller.m:28
        return out

    @property
    def index_all(self) -> List[int]:
        ret = {*self.index_input, *self.index_observe}
        return sorted(list(ret))

    def get_input_vectorized(self, t, x, X, V, I, U):
        def Xi(i): return tools.cellfun(
            lambda x: x(i, arange()).T, X)  # controller.m:36

        def Vi(i): return tools.hcellfun(
            lambda v: v(i, arange()).T, V)  # controller.m:37

        def Ii(j): return tools.hcellfun(
            lambda i: i(j, arange()).T, I)  # controller.m:38

        def Ui(i): return tools.cellfun(
            lambda u: u(i, arange()).T, U)  # controller.m:39
        
        __, u = self.get_dx_u_func(t(1), x(1, arange()).T, Xi(
            1), Vi(1), Ii(1), Ui(1), nargout=2)  # controller.m:41
        uout = np.zeros(numel(t), numel(u))  # controller.m:43
        uout[1, :] = u.T  # controller.m:44
        for i in arange(2, numel(t)).reshape(-1):
            __, u = self.get_dx_u_func(t(i), x(i, arange()).T, Xi(
                i), Vi(i), Ii(i), Ui(i), nargout=2)  # controller.m:47
            uout[i, arange()] = u.T  # controller.m:48
        return uout
