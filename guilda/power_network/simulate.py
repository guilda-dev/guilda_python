# pylint: disable=W0640

import numpy as np

from functools import reduce

from typing import Tuple, List, Callable, Optional, Iterable

from tqdm import tqdm

from guilda.power_network.base import _PowerNetwork, _sample2f, _idx2f

from guilda.power_network.types import SimulationOptions, SimulationResult, SimulationSegment
from guilda.power_network.control import get_dx_con

from guilda.base import ComponentEmpty

from guilda.utils.math import complex_mat_to_float
from guilda.utils.data import complex_arr_to_col_vec
from guilda.utils.typing import ComplexArray, FloatArray

from scikits.odes.dae import dae


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
    u: Optional[FloatArray] = None, # (n_u, n_t)
    idx_u: Optional[Iterable[int]] = None, 
    options: Optional[SimulationOptions] = None, 
    ):

    # process options
    if options is None:
        options = SimulationOptions()

    options.set_parameter_from_pn(self)

    # process u and index of u

    if idx_u is None:
        idx_u = []

    t_list = list(t)
    n_t = len(t_list)
    n_u = np.sum([x.get_nu() for x in self.a_bus])

    if u is None:
        u = np.zeros((n_u, n_t))

    if u is None or len(u.shape) != 2:
        raise TypeError(f'u must be 2-dimensional array of shape {(n_u, n_t)}, but got {u.shape}.')

    out = solve_odes(
        self, 
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
    self: _PowerNetwork, 
    t: List[float], 
    u: FloatArray, # (n_u, n_t)
    idx_u: List[int],
    fault: List[Tuple[Tuple[float, float], List[int]]], 
    x_in: List[FloatArray], 
    xkg: List[FloatArray],
    xk: List[FloatArray],
    V0_in: List[complex], 
    I0_in: List[complex], 
    linear: bool, 
    options: SimulationOptions):

    bus = self.a_bus
    controllers_global = self.a_controller_global
    controllers = self.a_controller_local

    fault_time: List[Tuple[float, float]] = [x[0] for x in fault]
    idx_fault: List[List[int]] = [x[1] for x in fault]

    uf = _sample2f(t, u)
    fault_f = _idx2f(fault_time, idx_fault)

    t_cand = sorted(list(set(
        np.array(fault_time).flatten().tolist() + t
    )))


    # :27

    nx_bus = [b.get_nx() for b in bus]
    nu_bus = [b.get_nu() for b in bus]
    nx_kg = [c.get_nx() for c in controllers_global]
    nx_k = [c.get_nx() for c in controllers]

    idx_non_unit = [i for i, b in enumerate(bus) if isinstance(b.component, ComponentEmpty)]
    _idx_controller = [
        c.index_observe + c.index_input
        for c in controllers + controllers_global
    ]
    idx_controller: List[int] = sorted(list(set(reduce(lambda x, y: [*x, *y], _idx_controller)))) if _idx_controller else [] 
    # idx_controller: unique pairs of observe-input indices

    Y = self.get_admittance_matrix()
    Ymat_all = complex_mat_to_float(Y)

    t_simulated = t_cand
    if options.method.lower() == 'zoh':
        t_simulated = get_t_simulated(t_cand, uf, fault_f)

    # :45
    # restore simulation result

    sols: List[Tuple[FloatArray, FloatArray, FloatArray, FloatArray]] = [] # (t, x)[]
    metas: List[SimulationSegment] = []
    # TODO add reporter

    # initial condition

    x_k: FloatArray = np.vstack(x_in + xkg + xk)
    V_k: FloatArray = complex_arr_to_col_vec(np.array(V0_in))
    I_k: FloatArray = complex_arr_to_col_vec(np.array(I0_in))

    # shape etc.

    nx = x_k.size


    # :88


    pbar = tqdm(total=1)

    pbar.update(0)
    ti = t_simulated[0]
    tf = t_simulated[-1]

    for i in range(len(t_simulated) - 1):

        tstart, tend = t_simulated[i: i + 2]

        f_ = fault_f((tstart + tend) / 2)

        except_ = set(list(f_) + idx_controller)
        simulated_bus = sorted(set(range(len(bus))) - (set(idx_non_unit) - except_))
        _, Ymat, __, Ymat_reproduce \
            = reduce_admittance_matrix(Y, simulated_bus)

        meta = SimulationSegment(
            time_start = tstart, 
            time_end = tend, 
            simulated_bus = simulated_bus, 
            fault_bus = f_, 
            impedance_matrix = Ymat_reproduce,
        )

        idx_simulated_bus: List[int] = [2 * x for x in simulated_bus] + [2 * x + 1 for x in simulated_bus]
        idx_fault_bus: List[int] = reduce(lambda x, y: [*x, *y], [[x * 2, x * 2 + 1] for x in f_] + [[]])


        x = np.vstack([x_k, V_k[idx_simulated_bus], I_k[idx_fault_bus]])


        # :128
        nVI = x.size - nx
        nI = len(f_) * 2
        nV = nVI - nI

        if i == 0:
            # add initial value records
            _X = x[:nx, :].T
            _V = x[nx: nx + nV, :].T @ Ymat_reproduce.T
            _I = _V @ Ymat_all.T
            sols.append((
                np.array([tstart]), 
                # x,
                _X,
                _V,
                _I,
            ))

        # mass = block_diag(np.eye(nx), np.zeros((nVI, nVI)))

        # define the equation

        def func_(t: float, x: FloatArray, u: Callable[[float], FloatArray]):
            pbar_val = (t - ti) / (tf - ti)
            pbar.update(pbar_val - pbar.n)

            dx, con = get_dx_con(
                linear,
                bus, controllers_global, controllers, Ymat,
                nx_bus, nx_kg, nx_k, nu_bus,
                t, x.reshape((-1, 1)), u(t), idx_u, f_, simulated_bus
            )

            # print(ret.var())

            return dx.flatten(), con.flatten()


        if options.method.lower() == 'zoh':
            u_ = uf((tstart + tend) / 2)
            func = lambda t, x: func_(t, x, lambda _: u_)
        else:
            us_ = uf(tstart)
            ue_ = uf(tend)
            u__ = lambda t: (ue_ * (t - tstart) + us_ * (tend - t)) / (tend - tstart)
            func = lambda t, x: func_(t, x, u__)


        def func_dae_rhs(t, y, ydot, res):
            dy, con = func(t, y)
            n = dy.size
            res[:n] = dy - ydot[:n]
            res[n:] = con

            print(res)


        # :138
        # This uses Dormand-Prince method instead of ode15s. s
        # To use ode15s, one must write his own integration.

        # solver_method = RadauDAE#  if options.solver_method == 'RadauDAE' else options.solver_method

        # sol = solve_ivp_custom(lambda t, x: np.concatenate(func(t, x)), t_simulated[i: i + 2], x.flatten(),
        #     method=RadauDAE, # order=options.solver_order, # type: ignore
        #     atol=options.atol, rtol=options.rtol, mass_matrix=mass
        # ) # (n_x, 2)

        solver = dae(
            'ida', func_dae_rhs,
            compute_initcond='yp0', first_step_size=1e-18,
            atol=options.atol, rtol=options.rtol,
            algebraic_vars_idx=list(range(nx, x.size)),
        )

        dx0_, con0_ = func(0, x.flatten())
        dx0 = np.concatenate([dx0_, con0_ * 0])
        sol = solver.solve(t_simulated[i: i + 2], x.flatten(), dx0)

        # :143~148

        y = sol.values.y.T[:, 1:] # get the end solution
        V = y[nx: nx + len(idx_simulated_bus)]

        # calculate conditions for the next iteration
        x_k = y[0: nx]
        V_k = Ymat_reproduce @ V
        I_k = Ymat_all @ V_k

        X = y[:nx, :].T
        V = y[nx: nx + nV, :].T @ Ymat_reproduce.T
        I = V @ Ymat_all.T

        ifault = np.array([
            [x * 2, x * 2 + 1] for x in f_
        ], dtype=int) # (n_fault, 2)
        I[:, ifault.flatten()] = y[nx + nV:, :].T

        if options.save_solution:
            meta.solution = sol

        metas.append(meta)
        sols.append((sol.values.t[1:], X, V, I))

    t_all, x_all, V_all, I_all = [
        np.concatenate([x[i] for x in sols]) if i == 0 else np.vstack([x[i] for x in sols])
        for i in range(4)
    ]

    pbar.update(1)
    pbar.close()

    # return (len(t_simulated), t_all, s_all, x_all, V_all, I_all), metas

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
