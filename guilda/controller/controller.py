import numpy as np
from typing import Optional, Tuple, List, Hashable
from abc import ABC, abstractmethod as AM

from guilda.utils.typing import FloatArray



class Controller(ABC):
    '''_summary_

    Args:
        ABC (_type_): _description_

    Returns:
        _type_: _description_
    '''

    def __init__(self, index_input: List[Hashable], index_observe: List[Hashable]):
        self.index_input: List[Hashable] = index_input
        self.index_observe: List[Hashable] = index_observe

    @property
    @AM
    def nx(self) -> int:
        '''_summary_

        Returns:
            int: _description_
        '''

    @AM
    def get_dx_u(
        self,
        V: Optional[FloatArray] = None,
        I: Optional[FloatArray] = None,
        x: Optional[FloatArray] = None,  # self state
        u: Optional[List[FloatArray]] = None,  # observed state (=X)
        U: Optional[List[FloatArray]] = None,  # global input
        t: float = 0
    ) -> Tuple[FloatArray, FloatArray]:
        """_summary_

        Args:
            V (Optional[FloatArray], optional): _description_. Defaults to None.
            I (Optional[FloatArray], optional): _description_. Defaults to None.
            x (Optional[FloatArray], optional): _description_. Defaults to None.

        Returns:
            Tuple[FloatArray, FloatArray]: _description_
        """

    @AM
    def get_dx_u_linear(
        self,
        V: Optional[FloatArray] = None,
        I: Optional[FloatArray] = None,
        x: Optional[FloatArray] = None,  # self state
        # observed state (=X)
        u: Optional[List[FloatArray]] = None,
        U: Optional[List[FloatArray]] = None,  # global input
        t: float = 0
    ) -> Tuple[FloatArray, FloatArray]:
        """_summary_

        Args:
            V (Optional[FloatArray], optional): _description_. Defaults to None.
            I (Optional[FloatArray], optional): _description_. Defaults to None.
            x (Optional[FloatArray], optional): _description_. Defaults to None.

        Returns:
            Tuple[FloatArray, FloatArray]: _description_
        """

    def get_dx_u_func(self, linear: bool):
        return self.get_dx_u_linear if linear else self.get_dx_u

    def get_x0(self) -> FloatArray:
        out = np.zeros((self.nx, 1))  # controller.m:28
        return out

    @property
    def index_all(self) -> List[Hashable]:
        ret = {*self.index_input, *self.index_observe}
        return list(ret)

    def get_input_vectorized(self, t, x, X, V, I, U):
        pass
        # what the hell???
        # def Xi(i): return tools.cellfun(
        #     lambda x: x(i, arange()).T, X)  # controller.m:36

        # def Vi(i): return tools.hcellfun(
        #     lambda v: v(i, arange()).T, V)  # controller.m:37

        # def Ii(j): return tools.hcellfun(
        #     lambda i: i(j, arange()).T, I)  # controller.m:38

        # def Ui(i): return tools.cellfun(
        #     lambda u: u(i, arange()).T, U)  # controller.m:39

        # __, u = self.get_dx_u_func(t(1), x(1, arange()).T, Xi(
        #     1), Vi(1), Ii(1), Ui(1), nargout=2)  # controller.m:41
        # uout = np.zeros(numel(t), numel(u))  # controller.m:43
        # uout[1, :] = u.T  # controller.m:44
        # for i in arange(2, numel(t)).reshape(-1):
        #     __, u = self.get_dx_u_func(t(i), x(i, arange()).T, Xi(
        #         i), Vi(i), Ii(i), Ui(i), nargout=2)  # controller.m:47
        #     uout[i, arange()] = u.T  # controller.m:48
        # return uout
