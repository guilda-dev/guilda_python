from typing import Callable, List, Dict, Hashable, Tuple
import numpy as np

from guilda.bus import Bus
from guilda.controller import Controller
from guilda.utils.data import sep_col_vec
from guilda.utils.typing import FloatArray


def get_dx_con(
    linear: bool,

    buses: List[Bus],
    
    ctrls_global: List[Controller],
    ctrls: List[Controller],
    ctrls_global_indices: List[Tuple[List[int], List[int]]],
    ctrls_indices: List[Tuple[List[int], List[int]]],

    reduced_admittance: FloatArray,

    nx_bus: List[int],
    nx_ctrl_global: List[int],
    nx_ctrl: List[int],
    nu_bus: List[int],

    t: float,
    xVI_all: FloatArray,
    u_all: Callable[[float, int], FloatArray],
    u_indices: List[int],  # only these inputs are not 0

    fault_buses: List[int],
    simulated_buses: List[int],
):

    # separate x, V, I
    
    n1 = np.sum([nx_bus[b] for b in simulated_buses], dtype=int)
    n2 = np.sum(nx_ctrl_global, dtype=int)
    n3 = np.sum(nx_ctrl, dtype=int)

    n_v_sim_buses = 2 * len(simulated_buses)
    n_i_fault_buses = 2 * len(fault_buses)

    x = xVI_all[0: n1]
    xkg = xVI_all[n1: n1 + n2]
    xk = xVI_all[n1 + n2: n1 + n2 + n3]

    nx_buses_and_ctrls = n1 + n2 + n3
    
    V_flattened = xVI_all[nx_buses_and_ctrls: nx_buses_and_ctrls + n_v_sim_buses]
    
    V_arr = np.reshape(V_flattened, (-1, 2)).T # each row is [real, imag]
    
    I_fault_arr = np.reshape(
        xVI_all[nx_buses_and_ctrls + n_v_sim_buses: nx_buses_and_ctrls + n_v_sim_buses + n_i_fault_buses], 
        (-1, 2)
    ).T

    I_flattened = np.reshape(reduced_admittance @ V_flattened, (-1, 2)).T # each row is [real, imag]
    I_flattened[:, fault_buses] = I_fault_arr

    V_all = np.zeros((2, len(buses)))
    I_all = np.zeros((2, len(buses)))

    V_all[:, simulated_buses] = V_arr
    I_all[:, simulated_buses] = I_flattened

    # split bus and controller states

    x_buses = sep_col_vec(x, [nx_bus[b] for b in simulated_buses])

    x_ctrls_global = sep_col_vec(xkg, nx_ctrl_global)
    x_ctrls = sep_col_vec(xk, nx_ctrl)
    
    # calculate inputs
    
    u_buses = [np.zeros((0, 1))] * len(buses)
    for b in simulated_buses:
        u_buses[b] = np.zeros((nu_bus[b], 1))

    # calculate dx of global controllers

    u_global = dict()
    dx_ctrls_global = [np.zeros((0, 0))] * len(ctrls_global)

    for i, c in enumerate(ctrls_global):
        i_observe, i_input = ctrls_global_indices[i]
        # pass data
        f = c.get_dx_u_func(linear)
        dx_ctrls_global[i], u_ctrl_global = f(
            V_all[:, i_observe], I_all[:, i_input], 
            x_ctrls_global[i], 
            [x_buses[i] for i in i_observe], None, t
        )

        idx = 0
        for i_input2 in i_input:
            u_global[i_input2] = u_ctrl_global[idx: idx + nu_bus[i_input2]]
            idx = idx + nu_bus[i_input2]

    # apply inputs from global controllers
    for i, u_ctrl_g in u_global.items():
        u_buses[i] += u_ctrl_g

    # calculate dx of local controllers
    
    u_local = dict()
    dx_ctrls = [np.zeros((0, 0))] * len(ctrls)

    for i, c in enumerate(ctrls):
        i_observe, i_input = ctrls_global_indices[i]
        f = c.get_dx_u_func(linear)
        dx_ctrls[i], u_ctrl = f(
            V_all[:, i_observe], I_all[:, i_observe],
            x_ctrls[i], 
            [x_buses[i] for i in i_observe], [u_buses[i] for i in i_observe], t
        )

        idx = 0
        for i_input2 in i_input:
            u_local[i_input2] = u_ctrl[idx: idx + nu_bus[i_input2]]
            idx = idx + nu_bus[i_input2]
            
    # apply inputs from local controllers
    for i, u_ctrl_l in u_local.items():
        u_buses[i] += u_ctrl_l
        
    
    # apply inputs from simulation scenario
    idx = 0
    for i in u_indices:
        u_buses[i] += u_all(t, i)
        idx += nu_bus[i]
        
        
    # calculate DAE residues of network components
        
    dx_component: List[FloatArray] = []
    constraint_component: List[FloatArray] = []

    for idx in simulated_buses:
        f = buses[idx].component.get_dx_con_func(linear)
        v = V_all[0, idx] + 1j * V_all[1, idx]
        i = I_all[0, idx] + 1j * I_all[1, idx]
        dx_i, cs_i = f(
            v,
            i,
            x_buses[idx],
            u_buses[idx],
            t,
        )
        dx_component.append(dx_i)
        constraint_component.append(cs_i)

    # concatenate results

    algebraic_constraint = np.vstack([
        *constraint_component,
        np.reshape(V_all[:, fault_buses], (-1, 1))
    ])
    dx = np.vstack([
        *dx_component,
        *dx_ctrls_global,
        *dx_ctrls,
    ])

    return dx, algebraic_constraint
