import numpy as np
import guilda.backend as G

from numpy.linalg import inv
from numpy.typing import NDArray

from scipy.optimize import root
from scipy.linalg import block_diag


from typing import Tuple, List, Optional, Callable

from guilda.bus import Bus
from guilda.branch import Branch
from guilda.controller import Controller
from guilda.utils.calc import complex_mat_to_float

from guilda.backend import ArrayProtocol, ArrayProtocol


def f_tmp(t, fs):
    idx = []
    for itr in range(fs):
        idx.append(fs[itr](t))
    return idx


def _sample2f(t: List[float], u: ArrayProtocol) -> Callable[[float], ArrayProtocol]:
    '''_summary_

    Args:
        t (List[float]): _description_
        u (ArrayProtocol): array (2, n)

    Returns:
        _type_: _description_
    '''
    # if not u.any() or not u[1].any():
    #     return lambda _: np.zeros((0, 0))
    
    if u.shape[0] != len(t) and u.shape[1] == len(t):
        u = G.array(u).T

    def ret(T: float) -> ArrayProtocol:
        ind = len(t) - 1 - ([t_ <= T for t_ in t][::-1]).index(True)
        return u[ind: ind + 1].T
    return ret

def _idx2f_inner(t: float, fault_time: List[Tuple[float, float]], idx_fault: List[List[int]]) -> List[int]:
    ret: List[int] = []
    for (ts, i) in zip(fault_time, idx_fault):
        if (ts[0] <= t and t < ts[1]):
            ret.extend(i)
    return ret


def _idx2f(fault_time: List[Tuple[float, float]], idx_fault: List[List[int]]) -> Callable[[float], List[int]]:
    
    if not fault_time:
        return lambda _: []
    
    return lambda t: _idx2f_inner(t, fault_time, idx_fault)


class _PowerNetwork(object):

    def __init__(self):

        self.a_bus: List[Bus] = []
        self.a_branch: List[Branch] = []
        self.a_controller_local: List[Controller] = []
        self.a_controller_global: List[Controller] = []

    @property
    def x_equilibrium(self) -> List[ArrayProtocol]:
        return [b.component.x_equilibrium for b in self.a_bus]

    @property
    def V_equilibrium(self) -> List[complex]:
        return [b.V_equilibrium or 0 for b in self.a_bus]

    @property
    def I_equilibrium(self) -> List[complex]:
        return [b.I_equilibrium or 0 for b in self.a_bus]

    def add_bus(self, *bus: Bus):
        self.a_bus.extend(bus)

    def add_branch(self, *branch: Branch):
        self.a_branch.extend(branch)
        
    

    def get_admittance_matrix(self, a_index_bus: Optional[List[int]] = None) -> ArrayProtocol:
        if not a_index_bus:
            a_index_bus = list(range(len(self.a_bus)))

        n: int = len(self.a_bus)
        Y: ArrayProtocol = G.zeros((n, n), dtype=G.dtype_c)

        for br in self.a_branch:
            if (br.from_-1 in a_index_bus) and (br.to-1 in a_index_bus):
                Y_sub = br.get_admittance_matrix()
                f, t = br.from_-1, br.to-1
                Y[f:f+1, f:f+1] += Y_sub[:1, :1]
                Y[f:f+1, t:t+1] += Y_sub[:1, 1:]
                Y[t:t+1, f:f+1] += Y_sub[1:, :1]
                Y[t:t+1, t:t+1] += Y_sub[1:, 1:]

        for idx in a_index_bus:
            Y[idx, idx] += self.a_bus[idx].shunt

        return Y

    def calculate_power_flow(self) -> Tuple[ArrayProtocol, ArrayProtocol]:
        n: int = len(self.a_bus)

        def func_eq(Y: NDArray, x: NDArray):
            Vr = x[0::2]
            Vi = x[1::2]
            V = Vr + 1j*Vi

            I = Y @ V
            PQhat = V * I.conjugate()
            P = PQhat.real
            Q = PQhat.imag

            out = np.zeros((n * 2, 1))
            for i in range(n):
                bus = self.a_bus[i]
                out_i = bus.get_constraint(V[i].real, V[i].imag, P[i], Q[i])
                out[i * 2: i * 2 + 2, :] = out_i
            return out.flatten()

        Y = self.get_admittance_matrix()
        Y_np = G.asnp(Y)
        x0 = np.array([1, 0] * n).reshape((-1, 1))

        # this one definitely requires numpy backend
        ans = root(lambda x: func_eq(Y_np, x), x0, method="hybr")

        Vrans = np.array([[ans.x[i]] for i in range(0, len(ans.x), 2)])
        Vians = np.array([[ans.x[i]] for i in range(1, len(ans.x), 2)])
        Vans = Vrans + 1j*Vians

        Ians = Y @ Vans
        return G.array(Vans), Ians

    def set_equilibrium(self, V: ArrayProtocol, I: ArrayProtocol):
        for idx in range(len(self.a_bus)):
            self.a_bus[idx].set_equilibrium(V[idx][0], I[idx][0])

    def initialize(self):
        V, I = self.calculate_power_flow()
        self.set_equilibrium(V, I)

    def get_sys(self):
        # A, B, C, D, BV, DV, BI, DI, R, S
        mats = [[] for _ in range(10)]
        for b in self.a_bus:
            mat = b.component.get_linear_matrix().as_tuple()
            for i in range(len(mats)):
                if mat[i].shape == (0, 0):
                    continue
                mats[i].append(mat[i])
        [A, B, C, D, BV, DV, BI, DI, R, S] = list(
            map(lambda mat: block_diag(*mat), mats))
        nI = C.shape[0]
        nx = A.shape[0]

        nV = BV.shape[1]
        nd = R.shape[1]
        nu = B.shape[1]
        nz = S.shape[0]
        Y = self.get_admittance_matrix()
        Ymat = complex_mat_to_float(Y)
        
        # TODO replace backend
        A11 = A
        A12 = np.hstack([BV, BI])
        A21 = np.vstack([C, np.zeros((nI, nx))])
        A22 = np.block([[DV, DI], [Ymat, -np.eye(nI)]])
        B1 = np.hstack([B, R])
        B2 = np.block([[D, np.zeros([nV, nd])], [np.zeros([nI, nu+nd])]])
        C1 = np.vstack([np.eye(nx), S, np.zeros([nI+nV, nx])])
        C2: ArrayProtocol = np.vstack([np.zeros([nx+nz, nV+nI]), np.eye(nV+nI)])

        A_ = A11-A12 @ inv(A22) @ A21
        B_ = B1-A12 @ inv(A22) @ B2
        C_ = C1-C2 @ inv(A22) @ A21
        D_ = 0-C2 @ inv(A22) @ B2
        return [A_, B_, C_, D_]


