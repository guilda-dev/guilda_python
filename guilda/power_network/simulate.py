# pylint: disable=W0640

import numpy as np

from functools import reduce

from typing import Set, Tuple, List, Callable, Optional, Iterable, Hashable, Dict

from tqdm import tqdm
from guilda.bus.bus import Bus
from guilda.controller.controller import Controller

from guilda.power_network.base import _PowerNetwork

from guilda.power_network.types import BusConnect, BusFault, BusInput, SimulationOptions, SimulationResult, SimulationSegment, SimulationScenario
from guilda.power_network.control import get_dx_con

from guilda.base import ComponentEmpty

from guilda.utils.calc import complex_mat_to_float
from guilda.utils.data import complex_arr_to_col_vec
from guilda.utils.runtime import suppress_stdout
from guilda.utils.typing import ComplexArray, FloatArray

from assimulo.problem import Implicit_Problem
from assimulo.solvers import IDA

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
    Y_mat_reduced = complex_mat_to_float(Y_reduced)

    nr_n_reduced = int(np.sum(n_reduced))

    A_reproduce: ComplexArray = np.zeros((n_bus, nr_n_reduced), dtype=complex)
    A_reproduce[n_reduced] = np.eye(nr_n_reduced)
    A_reproduce[reduced] = -np.linalg.inv(Y22) @ Y21

    A_mat_reproduce = complex_mat_to_float(A_reproduce)

    return Y_reduced, Y_mat_reduced, A_reproduce, A_mat_reproduce


def simulate(
    self: _PowerNetwork,
    scenario: SimulationScenario,
    options: Optional[SimulationOptions] = None,
):

    # process options
    if options is None:
        options = SimulationOptions()

    bus_index_map, init_states, timestamps, events = scenario.parse(self)

    bus = [self.a_bus_dict[i] for i in bus_index_map]

    controllers_global = self.a_controller_global
    controllers = self.a_controller_local
    Y = self.get_admittance_matrix()

    out = solve_odes(
        bus,
        bus_index_map,
        controllers_global,
        controllers,
        Y,
        timestamps,
        *init_states,
        events,
        scenario,
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
    
    x_init_bus: List[FloatArray],
    x_init_kg: List[FloatArray],
    x_init_k: List[FloatArray],
    V_init: List[complex],
    I_init: List[complex],

    events: Dict[float, List[Tuple[object, int, bool]]],
    scenario: SimulationScenario, 
    options: SimulationOptions  # only used for initializing solver
):
    
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

    t_simulated = t

    # :45
    # store simulation result

    sol_list: List[Tuple[FloatArray, FloatArray,
                     FloatArray, FloatArray]] = []  # (t, x)[]
    meta_list: List[SimulationSegment] = []

    # initial condition

    x_k: FloatArray = np.vstack(x_init_bus + x_init_kg + x_init_k)
    V_k: FloatArray = complex_arr_to_col_vec(np.array(V_init))
    I_k: FloatArray = complex_arr_to_col_vec(np.array(I_init))

    # shape etc.

    nx = x_k.size

    # define p-bar

    progress_bar = tqdm(total=1)

    progress_bar.update(0)
    ti = t_simulated[0]
    tf = t_simulated[-1]
    
    # event recorders
    idx_with_input: Set[int] = set()
    input_functions: Dict[int, Callable[[float], FloatArray]] = {}
    idx_with_fault: Set[int] = set()
    idx_disconnected: Set[int] = set()

    def get_input(t: float, i: int):
        ret = input_functions[i](t).reshape((-1, 1))
        return ret

    # in each time span
    for i in range(len(t_simulated) - 1):

        tstart, tend = t_simulated[i: i + 2]
        
        # handle events
        for e in events[tstart]:
            obj, index, flag = e
            
            # input
            if isinstance(obj, BusInput):
                b_index = bus_index_map[obj.index]
                if flag:
                    idx_with_input.add(b_index)
                    if callable(obj.value):
                        input_functions[b_index] = obj.value
                    else:
                        input_functions[b_index] = obj.get_interp()
                elif b_index in idx_with_input:
                    idx_with_input.remove(b_index)
                    
            # fault
            if isinstance(obj, BusFault):
                b_index = bus_index_map[obj.index]
                if flag:
                    idx_with_fault.add(b_index)
                elif b_index in idx_with_fault:
                    idx_with_fault.remove(b_index)
                    
            # connection
            # TODO install
            if isinstance(obj, BusConnect):
                b_index = bus_index_map[obj.index]
                if flag:
                    idx_disconnected.add(b_index)
                elif b_index in idx_disconnected:
                    idx_disconnected.remove(b_index)
                

        cur_fault_buses = sorted(list(idx_with_fault))
        u_indices = sorted(list(idx_with_input))

        must_include_buses = set(list(cur_fault_buses) + idx_controlled_buses)

        # (all buses that are empty but neither faulted nor controlled) are excluded
        # TODO assume that this step is only for the reduction of computation...
        cur_sim_buses = sorted(set(range(len(buses))) -
                               (set(idx_empty_buses) - must_include_buses))

        _, admittance_reduced, __, admittance_reproduce \
            = reduce_admittance_matrix(system_admittance, cur_sim_buses)

        meta = SimulationSegment(
            time_start=tstart,
            time_end=tend,
            simulated_buses=cur_sim_buses,
            fault_buses=cur_fault_buses,
            admittance=admittance_reproduce,
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
            _V = x_dae[nx: nx + nV, :].T @ admittance_reproduce.T
            _I = _V @ system_admittance_float.T
            sol_list.append((
                np.array([tstart]),
                # x,
                _X,
                _V,
                _I,
            ))

        # define the equation

        def func(
            t: float,
            x: FloatArray,
            dx: FloatArray,
        ):

            dx_calc, con = get_dx_con(
                options.linear,
                buses, ctrls_global, ctrls, ctrls_global_indices, ctrls_indices, 
                admittance_reduced,
                nx_bus, nx_kg, nx_k, nu_bus,
                t, x.reshape(
                    (-1, 1)), get_input, u_indices, cur_fault_buses, cur_sim_buses
            )

            n = dx_calc.size

            # TODO add reporter?
            progress_val = (t - ti) / (tf - ti)
            progress_bar.update(progress_val - progress_bar.n)

            ret = np.concatenate([dx_calc.flatten() - dx[:n], con.flatten()])
            return ret

        # solve the equation

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
        
        
        # concatenate results

        X = y[:nx, :].T
        V = y[nx: nx + nV, :].T @ admittance_reproduce.T
        I = V @ system_admittance_float.T

        x_k = X[-1:].T
        V_k = V[-1:].T
        I_k = I[-1:].T

        i_fault = np.array([
            [x * 2, x * 2 + 1] for x in cur_fault_buses
        ], dtype=int)  # (n_fault, 2)
        I[:, i_fault.flatten()] = y[nx + nV:, :].T

        if options.save_solution:
            meta.solution = (t_sol, y_orig, dy)

        meta_list.append(meta)
        sol_list.append((t_sol[0:], X, V, I))

    t_all, x_all, V_all, I_all = [
        np.concatenate([x[i] for x in sol_list]) if i == 0 else np.vstack(
            [x[i] for x in sol_list])
        for i in range(4)
    ]

    progress_bar.update(1)
    progress_bar.close()

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
        segments=meta_list,
        linear=options.linear,
        x=x_part,
        V=V_part,
        I=I_part,
    )

    # TODO add controller data

    return out
