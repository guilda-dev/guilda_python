import numpy as np

from typing import List

from guilda.backend import ArrayProtocol
from guilda.controller.controller import Controller


class ControllerBroadcastPIAGC(Controller):

    # restrictions: Access = private

    net = None

    def __init__(self, net, y_idx: List[int], u_idx: List[int], Kp, Ki):
        
        super().__init__(u_idx, y_idx)
        
        self.Ki = Ki  # controller_broadcast_PI_AGC.m:16
        self.Kp = Kp  # controller_broadcast_PI_AGC.m:17
        # controller_broadcast_PI_AGC.m:18
        self.K_broadcast: ArrayProtocol = np.ones((len(self.index_input), 1))
        self.net = net  # controller_broadcast_PI_AGC.m:19

    @property
    def nx(self):
        return 1

    def get_dx_u(self, t, x, X, V, I, u_global):
        omega = np.zeros(numel(X), 1)  # controller_broadcast_PI_AGC.m:27
        for i in np.arange(1, numel(X)).reshape(-1):
            omega[i] = X[i](2)  # controller_broadcast_PI_AGC.m:29
        omega_mean = np.mean(omega)  # controller_broadcast_PI_AGC.m:31
        dx = omega_mean  # controller_broadcast_PI_AGC.m:32
        u = blkdiag(zeros(0, 1), dot(self.K_broadcast, (dot(
            self.Ki, x) + dot(self.Kp, omega_mean)))).T  # controller_broadcast_PI_AGC.m:33
        u = ravel(u)  # controller_broadcast_PI_AGC.m:34
        return dx, u

    def get_dx_u_linear(self, varargin):
        # controller_broadcast_PI_AGC.m:38
        dx, u = self.get_dx_u(varargin[arange()], nargout=2)
        return dx, u

    def get_linear_matrix(self):
        A = 0  # controller_broadcast_PI_AGC.m:42
        nx = tools.vcellfun(lambda b: b.component.nx, self.net.a_bus(
            self.index_observe))  # controller_broadcast_PI_AGC.m:43
        nu = tools.vcellfun(lambda b: b.component.nu, self.net.a_bus(
            self.index_observe))  # controller_broadcast_PI_AGC.m:44
        # controller_broadcast_PI_AGC.m:46
        BX = tools.harrayfun(lambda n: concat(
            [0, 1, zeros(1, n - 2)]), nx) / numel(nx)
        # controller_broadcast_PI_AGC.m:47
        DX = zeros(dot(numel(self.K_broadcast), 2), size(BX, 2))
        DX[arange(2, end(), 2), arange()] = dot(
            dot(self.K_broadcast, BX), self.Kp)  # controller_broadcast_PI_AGC.m:48
        # controller_broadcast_PI_AGC.m:50
        C = zeros(dot(numel(self.K_broadcast), 2), 1)
        C[arange(2, end(), 2), arange()] = dot(self.K_broadcast,
                                               self.Ki)  # controller_broadcast_PI_AGC.m:51
        # controller_broadcast_PI_AGC.m:53
        BV = zeros(1, dot(2, numel(self.index_observe)))
        # controller_broadcast_PI_AGC.m:54
        BI = zeros(1, dot(2, numel(self.index_observe)))
        # controller_broadcast_PI_AGC.m:55
        DV = zeros(size(C, 1), dot(2, numel(self.index_observe)))
        # controller_broadcast_PI_AGC.m:56
        DI = zeros(size(C, 1), dot(2, numel(self.index_observe)))
        Bu = zeros(1, sum(nu))  # controller_broadcast_PI_AGC.m:58
        Du = zeros(size(C, 1), sum(nu))  # controller_broadcast_PI_AGC.m:59
        return A, BX, BV, BI, Bu, C, DX, DV, DI, Du

    def get_signals(self, X, V):
        out = []  # controller_broadcast_PI_AGC.m:63
        return out
