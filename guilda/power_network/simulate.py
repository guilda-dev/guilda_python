# pylint: disable=W0640

import numpy as np

from functools import reduce

from typing import Tuple, List, Callable, Optional, Iterable, Hashable, Dict

from tqdm import tqdm
from guilda.bus.bus import Bus
from guilda.controller.controller import Controller

from guilda.power_network.base import _PowerNetwork, _sample2f, _idx2f

from guilda.power_network.types import SimulationOptions, SimulationResult, SimulationSegment
from guilda.power_network.control import get_dx_con

from guilda.base import ComponentEmpty

from guilda.utils.calc import complex_mat_to_float
from guilda.utils.data import complex_arr_to_col_vec
from guilda.utils.runtime import suppress_stdout
from guilda.utils.typing import ComplexArray, FloatArray

from assimulo.problem import Implicit_Problem
from assimulo.solvers import IDA


def get_t_simulated(
        t_cand: List[float],
        uf: Callable[[float], FloatArray],
        fault_f: Callable[[float], List[int]]):
    has_difference = np.ones((len(t_cand), 1), dtype=bool)
    u: FloatArray = np.array([[np.nan]])
    f: List[int] = [-1]
    for i in range(len(t_cand) - 1):
        unew = uf((t_cand[i] + t_cand[i + 1]) / 2)
        fnew = fault_f((t_cand[i] + t_cand[i + 1]) / 2)
        if any(unew != u) \
                or len(f) != len(fnew) \
                or any([f[i] != fnew[i] for i in range(len(f))]):
            u = unew
            f = fnew
        else:
            has_difference[i] = False
    t_simulated = [t_cand[i] for i in range(len(t_cand)) if has_difference[i]]
    return t_simulated


def reduce_admittance_matrix(Y: ComplexArray, index: Iterable[int]) -> Tuple[
    ComplexArray,
    FloatArray,
    ComplexArray,
    FloatArray
]:

    n_bus = Y.shape[0]
    reduced = np.array([i not in index for i in range(n_bus)])
    n_reduced = np.logical_not(reduced)

    Y11 = Y[n_reduced][:, n_reduced]
    Y12 = Y[n_reduced][:, reduced]
    Y21 = Y[reduced][:, n_reduced]
    Y22 = Y[reduced][:, reduced]

    Y_reduced = Y11 - Y12 @ np.linalg.inv(Y22) @ Y21
    Ymat_reduced = complex_mat_to_float(Y_reduced)

    nr_n_reduced = int(np.sum(n_reduced))

    A_reproduce: ComplexArray = np.zeros((n_bus, nr_n_reduced), dtype=complex)
    A_reproduce[n_reduced] = np.eye(nr_n_reduced)
    A_reproduce[reduced] = -np.linalg.inv(Y22) @ Y21

    Amat_reproduce = complex_mat_to_float(A_reproduce)

    return Y_reduced, Ymat_reduced, A_reproduce, Amat_reproduce


def simulate(
    self: _PowerNetwork,
    t: Iterable[float],
    u: Optional[FloatArray] = None,  # (n_u, n_t)
    idx_u: Optional[Iterable[int]] = None,
    options: Optional[SimulationOptions] = None,
):

    # process options
    if options is None:
        options = SimulationOptions()

    options.set_parameter_from_pn(self)
    
    # build bus list
    bus_index_map = self.bus_index_map
    bus = [self.a_bus_dict[i] for i in bus_index_map]

    # process u and index of u

    if idx_u is None:
        idx_u = []

    t_list = list(t)
    n_t = len(t_list)
    n_u = np.sum([x.nu for x in bus])

    if u is None:
        u = np.zeros((n_u, n_t))

    if u is None or len(u.shape) != 2:
        raise TypeError(
            f'u must be 2-dimensional array of shape {(n_u, n_t)}, but got {u.shape}.')

    controllers_global = self.a_controller_global
    controllers = self.a_controller_local
    Y = self.get_admittance_matrix()

    out = solve_odes(
        bus,
        bus_index_map,
        controllers_global,
        controllers,
        Y,
        t_list,
        u,
        idx_u if isinstance(idx_u, list) else list(idx_u),
        options.fault,
        options.x0_sys,
        options.x0_con_global,
        options.x0_con_local,
        options.V0,
        options.I0,
        options.linear,
        options
    )

    return out


def solve_odes(
    buses: List[Bus],
    bus_index_map: Dict[Hashable, int],
    ctrls_global: List[Controller],
    ctrls: List[Controller],
    system_admittance: ComplexArray,

    t: List[float],
    u: FloatArray,  # (n_u, n_t)
    u_indices: List[int],
    fault: List[Tuple[Tuple[float, float], List[int]]],

    x_init_bus: List[FloatArray],
    x_init_kg: List[FloatArray],
    x_init_k: List[FloatArray],
    V_init: List[complex],
    I_init: List[complex],

    linear: bool,
    options: SimulationOptions  # only used for initializing solver
):

    # transform input data

    fault_times: List[Tuple[float, float]] = [x[0] for x in fault]
    fault_indices: List[List[int]] = [x[1] for x in fault]

    uf = _sample2f(t, u)
    fault_f = _idx2f(fault_times, fault_indices)

    t_cand = sorted(list(set(
        np.array(fault_times).flatten().tolist() + t
    )))
    
    # build controller index map
    ctrls_global_indices = [
        (
            [bus_index_map[x] for x in c.index_observe],
            [bus_index_map[x] for x in c.index_input]
        ) for c in ctrls_global
    ]
    ctrls_indices = [
        (
            [bus_index_map[x] for x in c.index_observe],
            [bus_index_map[x] for x in c.index_input]
        ) for c in ctrls
    ]

    # :27

    nx_bus = [b.nx for b in buses]
    nu_bus = [b.nu for b in buses]
    nx_kg = [c.nx for c in ctrls_global]
    nx_k = [c.nx for c in ctrls]

    idx_empty_buses = [i for i, b in enumerate(
        buses) if isinstance(b.component, ComponentEmpty)]

    idx_controlled_buses: List[int] = sorted(list(set(reduce(
        lambda x, y: x + y[0] + y[1], 
        ctrls_global_indices + ctrls_indices,
        []
    ))))
    
    # idx_controlled_buses: unique pairs of observe-input indices

    system_admittance_float = complex_mat_to_float(system_admittance)

    t_simulated = t_cand
    if options.method.lower() == 'zoh':
        t_simulated = get_t_simulated(t_cand, uf, fault_f)

    # :45
    # store simulation result

    sols: List[Tuple[FloatArray, FloatArray,
                     FloatArray, FloatArray]] = []  # (t, x)[]
    metas: List[SimulationSegment] = []

    # initial condition

    x_k: FloatArray = np.vstack(x_init_bus + x_init_kg + x_init_k)
    V_k: FloatArray = complex_arr_to_col_vec(np.array(V_init))
    I_k: FloatArray = complex_arr_to_col_vec(np.array(I_init))

    # shape etc.

    nx = x_k.size

    # :88

    pbar = tqdm(total=1)

    pbar.update(0)
    ti = t_simulated[0]
    tf = t_simulated[-1]

    # in each time span
    for i in range(len(t_simulated) - 1):

        tstart, tend = t_simulated[i: i + 2]

        cur_fault_buses = fault_f((tstart + tend) / 2)

        must_include_buses = set(list(cur_fault_buses) + idx_controlled_buses)

        # (all buses that are empty but neither faulted nor controlled) are excluded
        # TODO assume that this step is only for the reduction of computation...
        cur_sim_buses = sorted(set(range(len(buses))) -
                               (set(idx_empty_buses) - must_include_buses))

        _, Ymat, __, Ymat_reproduce \
            = reduce_admittance_matrix(system_admittance, cur_sim_buses)

        meta = SimulationSegment(
            time_start=tstart,
            time_end=tend,
            simulated_buses=cur_sim_buses,
            fault_buses=cur_fault_buses,
            admittance=Ymat_reproduce,
        )

        idx_sim_buses: List[int] = [
            2 * x for x in cur_sim_buses] + [2 * x + 1 for x in cur_sim_buses]
        idx_fault_buses: List[int] = reduce(
            lambda x, y: [*x, *y], [[x * 2, x * 2 + 1] for x in cur_fault_buses] + [[]])

        # x_k: the states of all buses
        # V_k, I_k: the states of needed buses only
        x_dae = np.vstack([x_k, V_k[idx_sim_buses], I_k[idx_fault_buses]])

        # :128
        nVI = x_dae.shape[0] - nx
        nI = len(cur_fault_buses) * 2
        nV = nVI - nI

        if i == 0:
            # add initial value records
            _X = x_dae[:nx, :].T
            _V = x_dae[nx: nx + nV, :].T @ Ymat_reproduce.T
            _I = _V @ system_admittance_float.T
            sols.append((
                np.array([tstart]),
                # x,
                _X,
                _V,
                _I,
            ))

        # mass = block_diag(np.eye(nx), np.zeros((nVI, nVI)))

        # define the equation

        def func_(
            t: float,
            x: FloatArray,
            u: Callable[[float], FloatArray],
            xdot: FloatArray,
            # res: FloatArray,
        ):

            dx, con = get_dx_con(
                linear,
                buses, ctrls_global, ctrls, ctrls_global_indices, ctrls_indices, 
                Ymat,
                nx_bus, nx_kg, nx_k, nu_bus,
                t, x.reshape(
                    (-1, 1)), u(t), u_indices, cur_fault_buses, cur_sim_buses
            )

            n = dx.size
            # res[:n] = dx.flatten() - xdot[:n]
            # res[n:] = con.flatten()

            pbar_val = (t - ti) / (tf - ti)
            pbar.update(pbar_val - pbar.n)

            ret = np.concatenate([dx.flatten() - xdot[:n], con.flatten()])
            return ret
            # TODO add reporter?

        # interpolate input
        if options.method.lower() == 'zoh':
            u_ = uf((tstart + tend) / 2)
            def func(t, x, dx): return func_(t, x, lambda _: u_, dx)
        else:
            us_ = uf(tstart)
            ue_ = uf(tend)
            def u__(t): return (ue_ * (t - tstart) +
                                us_ * (tend - t)) / (tend - tstart)

            def func(t, x, dx): return func_(t, x, u__, dx)

        # :138

        # solver = dae(
        #     options.solver_method, func,
        #     compute_initcond='yp0', first_step_size=1e-18,
        #     atol=options.atol, rtol=options.rtol,
        #     algebraic_vars_idx=list(range(nx, x_dae.size)),
        # )

        x_0 = x_dae.flatten()
        dx_0 = x_0 * 0  # this will be computed by the solver

        model = Implicit_Problem(func, x_0, dx_0, tstart)
        sim = IDA(model)
        
        sim.rtol = options.rtol
        sim.atol = options.atol

        sim.algvar = [True] * nx + [False] * nVI
        sim.make_consistent('IDA_YA_YDP_INIT')
        sim.display_progress = False # this one is useless, dunno if it is buggy of my fault
        
        @suppress_stdout
        def s():
            return sim.simulate(tend)
        
        t_sol, y_orig, dy = s()
        y = y_orig.T
        
        
        # :143~148

        X = y[:nx, :].T
        V = y[nx: nx + nV, :].T @ Ymat_reproduce.T
        I = V @ system_admittance_float.T

        x_k = X[-1:].T
        V_k = V[-1:].T
        I_k = I[-1:].T

        ifault = np.array([
            [x * 2, x * 2 + 1] for x in cur_fault_buses
        ], dtype=int)  # (n_fault, 2)
        I[:, ifault.flatten()] = y[nx + nV:, :].T

        if options.save_solution:
            meta.solution = (t_sol, y_orig, dy)

        metas.append(meta)
        sols.append((t_sol[0:], X, V, I))

    t_all, x_all, V_all, I_all = [
        np.concatenate([x[i] for x in sols]) if i == 0 else np.vstack(
            [x[i] for x in sols])
        for i in range(4)
    ]

    pbar.update(1)
    pbar.close()

    x_part: List[FloatArray] = []
    V_part: List[FloatArray] = []
    I_part: List[FloatArray] = []

    idx = 0
    for nx in nx_bus:
        idx_end = idx + nx
        x_part.append(x_all[:, idx: idx_end])
        V_part.append(V_all[:, idx: idx_end])
        I_part.append(I_all[:, idx: idx_end])
        idx = idx_end

    out = SimulationResult(
        t=t_all,
        nx_bus=nx_bus,
        nu_bus=nu_bus,
        segments=metas,
        linear=linear,
        x=x_part,
        V=V_part,
        I=I_part,
    )

    # TODO add controller data

    return out
