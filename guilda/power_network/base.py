from functools import cached_property
from varname import nameof
import numpy as np
from numpy.linalg import inv

from scipy.optimize import root
from scipy.linalg import block_diag


from typing import Tuple, List, Optional, Callable, Dict, Hashable, Iterable

from guilda.bus import Bus
from guilda.branch import Branch
from guilda.controller import Controller
from guilda.utils.calc import complex_mat_to_float
from guilda.utils.runtime import del_cache

from guilda.utils.typing import FloatArray, ComplexArray

_pn_cached_vars: List[str] = []


class _PowerNetwork(object):

    def __init__(self):

        self.a_bus_dict: Dict[Hashable, Bus] = {}

        self.a_branch: List[Branch] = []
        self.a_controller_local: List[Controller] = []
        self.a_controller_global: List[Controller] = []

    @cached_property
    def x_equilibrium(self) -> List[FloatArray]:
        return [self.a_bus_dict[b].component.x_equilibrium for b in self.bus_indices]

    @cached_property
    def V_equilibrium(self) -> List[complex]:
        return [self.a_bus_dict[b].V_equilibrium or 0 for b in self.bus_indices]

    @cached_property
    def I_equilibrium(self) -> List[complex]:
        return [self.a_bus_dict[b].I_equilibrium or 0 for b in self.bus_indices]

    def add_bus(self, *buses: Bus):
        '''
        Add buses and allocate indices for those unassigned.
        '''
        for bus in buses:
            if bus.index is None:
                new_index = len(self.a_bus_dict) + 1
                while new_index in self.a_bus_dict:
                    new_index += 1
                bus.index = new_index
            if bus.index in self.a_bus_dict:
                raise RuntimeError(f'Bus of index {bus.index} already exists.')
            self.a_bus_dict[bus.index] = bus

        for name in _pn_cached_vars:
            del_cache(self, name)

    def add_branch(self, *branch: Branch):
        self.a_branch.extend(branch)

    def add_controller_global(self, *ctrl: Controller):
        self.a_controller_global.extend(ctrl)

    def add_controller(self, *ctrl: Controller):
        self.a_controller_local.extend(ctrl)

    @property
    def a_bus(self):
        return self.a_bus_dict.values()

    @cached_property
    def bus_indices(self):
        return list(self.a_bus_dict.keys())

    @cached_property
    def bus_index_map(self):
        m = {}
        for i in self.bus_indices:
            m[i] = len(m)
        return m

    def sort_buses(self):
        sorted_buses = dict(sorted(self.a_bus_dict.items(),
                            key=lambda item: item[1]))  # type: ignore
        self.a_bus_dict = sorted_buses

    def get_admittance_matrix(self, bus_index_map: Optional[Dict[Hashable, int]] = None) -> ComplexArray:
        if not bus_index_map:
            bus_index_map = self.bus_index_map

        n: int = len(bus_index_map)
        Y: ComplexArray = np.zeros((n, n), dtype=complex)

        for br in self.a_branch:
            if (br.bus1 in bus_index_map) and (br.bus2 in bus_index_map):
                Y_sub = br.get_admittance_matrix()
                f = bus_index_map[br.bus1]
                t = bus_index_map[br.bus2]
                Y[f:f+1, f:f+1] += Y_sub[:1, :1]
                Y[f:f+1, t:t+1] += Y_sub[:1, 1:]
                Y[t:t+1, f:f+1] += Y_sub[1:, :1]
                Y[t:t+1, t:t+1] += Y_sub[1:, 1:]

        for idx in bus_index_map:
            _idx = bus_index_map[idx]
            Y[_idx, _idx] += self.a_bus_dict[idx].shunt

        return Y

    def calculate_power_flow(self) -> Tuple[ComplexArray, ComplexArray]:
        n: int = len(self.a_bus_dict)

        def func_eq(Y: ComplexArray, x: FloatArray):
            Vr = x[0::2]
            Vi = x[1::2]
            V = Vr + 1j*Vi

            I = Y @ V
            PQhat = V * I.conjugate()
            P = PQhat.real
            Q = PQhat.imag

            out = np.zeros((n * 2, 1))
            for index, i in self.bus_index_map.items():
                bus = self.a_bus_dict[index]
                out_i = bus.get_constraint(V[i].real, V[i].imag, P[i], Q[i]) # type: ignore
                out[i * 2: i * 2 + 2, :] = out_i
            return out.flatten()

        Y = self.get_admittance_matrix()
        x0 = np.array([1, 0] * n).reshape((-1, 1))

        # this one definitely requires numpy backend
        ans = root(lambda x: func_eq(Y, x), x0, method="hybr")

        Vrans = np.array([[ans.x[i]] for i in range(0, len(ans.x), 2)])
        Vians = np.array([[ans.x[i]] for i in range(1, len(ans.x), 2)])
        Vans = Vrans + 1j*Vians

        Ians = Y @ Vans
        return Vans, Ians

    def set_equilibrium(self, V: ComplexArray, I: ComplexArray, bus_index_map: Optional[Dict[Hashable, int]] = None):
        if bus_index_map is None:
            bus_index_map = self.bus_index_map
        for index, i in bus_index_map.items():
            self.a_bus_dict[index].set_equilibrium(V[i][0], I[i][0])

    def initialize(self):
        V, I = self.calculate_power_flow()
        self.set_equilibrium(V, I)

    def get_sys(self):

        # A, B, C, D, BV, DV, BI, DI, R, S
        mats = [[np.zeros((0, 0))] * len(self.a_bus_dict) for _ in range(10)]
        for index, i in self.bus_index_map.items():
            b = self.a_bus_dict[index]
            mat = b.component.get_linear_matrix().as_tuple()
            for mi in range(len(mats)):
                # if mat[mi].shape == (0, 0):
                #     continue
                mats[mi][i] = mat[mi]
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

        A11 = A
        A12 = np.hstack([BV, BI])
        A21 = np.vstack([C, np.zeros((nI, nx))])
        A22 = np.block([[DV, DI], [Ymat, -np.eye(nI)]])
        B1 = np.hstack([B, R])
        B2 = np.block([[D, np.zeros([nV, nd])], [np.zeros([nI, nu+nd])]])
        C1 = np.vstack([np.eye(nx), S, np.zeros([nI+nV, nx])])
        C2: FloatArray = np.vstack([np.zeros([nx+nz, nV+nI]), np.eye(nV+nI)])

        A_ = A11-A12 @ inv(A22) @ A21
        B_ = B1-A12 @ inv(A22) @ B2
        C_ = C1-C2 @ inv(A22) @ A21
        D_ = 0-C2 @ inv(A22) @ B2
        return [A_, B_, C_, D_]


_pn_cached_vars += [
    nameof(_PowerNetwork.bus_index_map),
    nameof(_PowerNetwork.bus_indices),
    nameof(_PowerNetwork.x_equilibrium),
    nameof(_PowerNetwork.V_equilibrium),
    nameof(_PowerNetwork.I_equilibrium),
]
