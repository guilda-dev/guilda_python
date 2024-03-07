import numpy as np

from typing import List, Hashable, Optional, Tuple, cast
from guilda.utils.typing import FloatArray
from guilda.controller.controller import Controller

from scipy.linalg import block_diag


class ControllerBroadcastPIAGC(Controller):

    # restrictions: Access = private

    net = None

    def __init__(
        self, 
        index_input: List[Hashable], index_observe: List[Hashable], 
        Kp: float, Ki: float
    ):

        super().__init__(index_input, index_observe)

        self.Ki = Ki
        self.Kp = Kp
        
        # TODO add weights?

    @property
    def nx(self):
        return 1

    def get_dx_u(
        self,
        V: Optional[FloatArray] = None,
        I: Optional[FloatArray] = None,
        x: Optional[FloatArray] = None,  # self state
        u: Optional[List[FloatArray]] = None,  # observed state (=X)
        U: Optional[List[FloatArray]] = None,  # global input
        t: float = 0
    ) -> Tuple[FloatArray, FloatArray]:
        omega = [_u[1] for _u in u or []]
        omega_mean = np.mean(omega)  # controller_broadcast_PI_AGC.m:31
        dx = np.array([[omega_mean]])  # controller_broadcast_PI_AGC.m:32
        
        input = (self.Kp * omega_mean + self.Ki * cast(FloatArray, x)).flatten()[0]
        u_ret = np.array([
            [0, input] * len(self.index_input)
        ]).T
        return dx, u_ret

    def get_dx_u_linear(self, *args, **kwargs):
        return self.get_dx_u(*args, **kwargs)

    # def get_linear_matrix(self):
    #     A = 0  # controller_broadcast_PI_AGC.m:42
    #     nx = tools.vcellfun(lambda b: b.component.nx, self.net.a_bus(
    #         self.index_observe))  # controller_broadcast_PI_AGC.m:43
    #     nu = tools.vcellfun(lambda b: b.component.nu, self.net.a_bus(
    #         self.index_observe))  # controller_broadcast_PI_AGC.m:44
    #     # controller_broadcast_PI_AGC.m:46
    #     BX = tools.harrayfun(lambda n: concat(
    #         [0, 1, zeros(1, n - 2)]), nx) / numel(nx)
    #     # controller_broadcast_PI_AGC.m:47
    #     DX = zeros(dot(numel(self.K_broadcast), 2), size(BX, 2))
    #     DX[arange(2, end(), 2), arange()] = dot(
    #         dot(self.K_broadcast, BX), self.Kp)  # controller_broadcast_PI_AGC.m:48
    #     # controller_broadcast_PI_AGC.m:50
    #     C = zeros(dot(numel(self.K_broadcast), 2), 1)
    #     C[arange(2, end(), 2), arange()] = dot(self.K_broadcast,
    #                                            self.Ki)  # controller_broadcast_PI_AGC.m:51
    #     # controller_broadcast_PI_AGC.m:53
    #     BV = zeros(1, dot(2, numel(self.index_observe)))
    #     # controller_broadcast_PI_AGC.m:54
    #     BI = zeros(1, dot(2, numel(self.index_observe)))
    #     # controller_broadcast_PI_AGC.m:55
    #     DV = zeros(size(C, 1), dot(2, numel(self.index_observe)))
    #     # controller_broadcast_PI_AGC.m:56
    #     DI = zeros(size(C, 1), dot(2, numel(self.index_observe)))
    #     Bu = zeros(1, sum(nu))  # controller_broadcast_PI_AGC.m:58
    #     Du = zeros(size(C, 1), sum(nu))  # controller_broadcast_PI_AGC.m:59
    #     return A, BX, BV, BI, Bu, C, DX, DV, DI, Du
