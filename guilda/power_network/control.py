from typing import List
import numpy as np

from guilda.bus import Bus
from guilda.controller import Controller
from guilda.utils.data import sep_col_vec
from guilda.utils.typing import FloatArray

def get_dx_con(
    linear: bool,
    bus: List[Bus], 
    controllers_global: List[Controller], 
    controllers: List[Controller], 
    Ymat: FloatArray, 
    nx_bus: List[int], 
    nx_controller_global: List[int], 
    nx_controller: List[int], 
    nu_bus: List[int], 
    t: float, 
    x_all: FloatArray, 
    u: FloatArray, 
    idx_u: List[int],
    idx_fault: List[int], 
    simulated_bus: List[int]):

    # print(t)

    n1 = np.sum([nx_bus[b] for b in simulated_bus], dtype=int)
    n2 = np.sum(nx_controller_global, dtype=int)
    n3 = np.sum(nx_controller, dtype=int)

    n4 = 2 * len(simulated_bus)
    n5 = 2 * len(idx_fault)

    x = x_all[0: n1]
    xkg = x_all[n1: n1 + n2]
    xk = x_all[n1 + n2: n1 + n2 + n3]

    ns = n1 + n2 + n3
    V0 = x_all[ns: ns + n4]
    V = np.reshape(V0, (-1, 2)).T
    I_fault = np.reshape(x_all[ns + n4: ns + n4 + n5], (-1, 2)).T

    I = np.reshape(Ymat @ V0, (-1, 2)).T
    I[:, idx_fault] = I_fault

    Vall = np.zeros((2, len(bus)))
    Iall = np.zeros((2, len(bus)))

    Vall[:, simulated_bus] = V
    Iall[:, simulated_bus] = I

    #

    x_bus = sep_col_vec(x, [nx_bus[b] for b in simulated_bus])

    U_bus = [np.zeros((0, 0))] * len(bus)
    for b in simulated_bus:
        U_bus[b] = np.zeros((nu_bus[b], 1))

    xkg_cell = sep_col_vec(xkg, nx_controller_global)
    xk_cell = sep_col_vec(xk, nx_controller)

    #

    dxkg = [np.zeros((0, 0))] * len(controllers_global)

    for i, c in enumerate(controllers_global):
        f = c.get_dx_u_func(linear)
        dxkg[i], ug_ = f(
            t, xkg_cell[i], [x_bus[i] for i in c.index_observe], 
            Vall[:, c.index_observe], Iall[:, c.index_observe], [])

        idx = 0
        for i_input in np.array(c.index_input, dtype=int).flatten():
            U_bus[i_input] += ug_[idx: idx + nu_bus[i_input]]
            idx = idx + nu_bus[i_input]

    # 
    U_global = list(U_bus)

    dxk = [np.zeros((0, 0))] * len(controllers)

    for i, c in enumerate(controllers):
        f = c.get_dx_u_func(linear)
        dxk[i], u_ = f(
            t, xk_cell[i], [x_bus[i] for i in c.index_observe], 
            Vall[:, c.index_observe], Iall[:, c.index_observe], 
            [U_global[i] for i in c.index_observe])

        idx = 0
        for i_input in np.array(c.index_input, dtype=int).flatten():
            U_bus[i_input] += u_[idx: idx + nu_bus[i_input]]
            idx = idx + nu_bus[i_input]



    idx = 0
    for i in idx_u:
        U_bus[i] += u[idx:idx+nu_bus[i]]
        idx += nu_bus[i]


    dx_component: List[FloatArray] = []
    constraint: List[FloatArray] = []

    for idx in simulated_bus:
        f = bus[idx].component.get_dx_con_func(linear)
        v = Vall[0, idx] + 1j * Vall[1, idx]
        i = Iall[0, idx] + 1j * Iall[1, idx]
        dx_i, cs_i = f(
            v,
            i,
            x_bus[idx], 
            U_bus[idx],
            t, 
            )
        dx_component.append(dx_i)
        constraint.append(cs_i)

    dx_algebraic = np.vstack([
        *constraint, 
        np.reshape(Vall[:, idx_fault], (-1, 1))
        ])
    dx = np.vstack([
        *dx_component, 
        *dxkg, 
        *dxk, 
    ])

    return dx, dx_algebraic


