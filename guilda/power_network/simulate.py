# pylint: disable=W0640

from collections import defaultdict
import numpy as np

from functools import reduce

from typing import Set, Tuple, List, Callable, Optional, Iterable, Hashable, Dict

from tqdm import tqdm
from guilda.bus.bus import Bus
from guilda.controller.controller import Controller

from guilda.power_network.base import _PowerNetwork
from guilda.power_network.segment import gen_segments, parse_scenario

from guilda.power_network.types import BusConnect, BusEvent, BusFault, BusInput, SimulationMetadata, SimulationOptions, SimulationResult, SimulationResultComponent, SimulationSegment, SimulationScenario
from guilda.power_network.dae import get_dx_con

from guilda.base import ComponentEmpty

from guilda.utils.calc import complex_mat_to_float
from guilda.utils.data import complex_arr_to_col_vec, sep_col_vec
from guilda.utils.runtime import suppress_stdout
from guilda.utils.typing import ComplexArray, FloatArray

from assimulo.problem import Implicit_Problem
from assimulo.solvers import IDA



def augment_2(input_lst: List[int]):
    return reduce(
        lambda x, y: [*x, *y], 
        [[x * 2, x * 2 + 1] for x in input_lst],
        []
    )

def solve_dae(
    segment: SimulationSegment,
    meta: SimulationMetadata,
    options: SimulationOptions,
    
    x_init: FloatArray, # col vec
    V_init: FloatArray, # col vec
    I_init: FloatArray, # col vec
    
    dy_init: Optional[FloatArray] = None,
    e: Optional[Callable[[float], None]] = None,
):
    
    idx_sim_buses = augment_2(segment.buses_simulated)
    idx_fault_buses = augment_2(segment.buses_fault)

    nx = x_init.shape[0]
    nV = len(idx_sim_buses)
    nI = len(idx_fault_buses)
    nVI = nV + nI
    
    y_init = np.vstack([
        x_init,
        V_init[idx_sim_buses],
        I_init[idx_fault_buses],
    ]).flatten()

    # define the equation

    def func(
        t: float,
        y: FloatArray,
        dy: FloatArray,
    ):

        dx_calc, con = get_dx_con(
            
            t, 
            y.reshape((-1, 1)), 
            
            options.linear,
            meta.buses, meta.ctrls_global, meta.ctrls, 
            meta.ctrls_global_indices, meta.ctrls_indices,
            
            meta.nx_bus, meta.nx_ctrl_global, meta.nx_ctrl, meta.nu_bus,
            
            
            lambda t, i: segment.buses_input[i](t).reshape((-1, 1)), 
            list(segment.buses_input.keys()), 
            segment.buses_fault,
            segment.buses_simulated,
            
            segment.admittance_reduced,
        )

        n = dx_calc.size

        # event reporter
        if e:
            e(t)

        ret = np.concatenate([dx_calc.flatten() - dy[:n], con.flatten()])
        return ret

    # solve the equation
    if dy_init is None:
        dy_init = func(segment.time_start, y_init, np.zeros(y_init.shape))
    # this will partially be computed by the solver

    model = Implicit_Problem(func, y_init, dy_init, segment.time_start)
    sim = IDA(model)

    sim.rtol = options.rtol
    sim.atol = options.atol

    sim.algvar = [True] * nx + [False] * nVI
    con = sim.make_consistent('IDA_YA_YDP_INIT')
    sim.display_progress = False  # this one is useless, dunno if it is buggy of my fault
    
    ncp_list = None
    if options.t_interval > 0:
        ss, se = (segment.time_start, segment.time_end)
        ncp_list = np.arange(ss, se, options.t_interval)

    @suppress_stdout
    def s():
        return sim.simulate(segment.time_end, 0, ncp_list)

    t_sol, y_orig, dy = s()
    y = y_orig.T

    # concatenate results

    t = t_sol[0:]
    X = y[:nx, :].T
    V = y[nx: nx + nV, :].T @ segment.admittance_reproduce.T
    I = V @ meta.system_admittance_f.T
    
    I[:, idx_fault_buses] = y[nx + nV:, :].T
    
    solution = (t, X, V, I)

    # prepare for the next scenario

    x_k = X[-1:].T
    V_k = V[-1:].T
    I_k = I[-1:].T
    
    sol_end = (x_k, V_k, I_k)
    
    return solution, sol_end


def simulate(
    self: _PowerNetwork,
    scenario: SimulationScenario,
    options: Optional[SimulationOptions] = None,
):

    # process options
    if options is None:
        options = SimulationOptions()
    
    meta, init_states, timestamps, events = parse_scenario(scenario, self)
    # TODO process timestamps
    
    segments = gen_segments(meta, timestamps, events)
    
    
    # solve
    

    sol_list: List[Tuple[FloatArray, FloatArray,
                         FloatArray, FloatArray]] = []  # (t, x)[]
    
    x_init_bus, x_init_kg, x_init_k, V_init, I_init = init_states
    x_k: FloatArray = np.vstack(x_init_bus + x_init_kg + x_init_k)
    V_k: FloatArray = complex_arr_to_col_vec(np.array(V_init))
    I_k: FloatArray = complex_arr_to_col_vec(np.array(I_init))
    
    # add init condition
    sol_list.append((
        np.array([segments[0].time_start if segments else 0,]),
        x_k.T,
        V_k.T,
        I_k.T,
    ))
    
    progress_bar = tqdm(total = 1)
    min_time = np.min(timestamps)
    max_time = np.max(timestamps)
    
    def set_progress_bar(t: float):
        val = (t - min_time) / (max_time - min_time)
        progress_bar.update(val - progress_bar.n)
    
    for segment in segments:
        
        # solve
        solution, sol_end = solve_dae(
            segment,
            meta,
            options,
            x_k,
            V_k,
            I_k,
            
            e = set_progress_bar,
        )
        
        # post process
        x_k, V_k, I_k = sol_end
        set_progress_bar(segment.time_end)
        sol_list.append(solution)
        
    progress_bar.close()
    
    # concatenate solutions
    
    t_all, x_all, V_all, I_all = [
        np.concatenate([x[i] for x in sol_list]) if i == 0 else np.vstack(
            [x[i] for x in sol_list])
        for i in range(4)
    ]
    t_filter = np.concatenate([
        [True],
        t_all[1:] > t_all[:1]
    ])
    t_all = t_all[t_filter]
    x_all = x_all[t_filter, :]
    V_all = V_all[t_filter, :]
    I_all = I_all[t_filter, :]

    x_part_all = sep_col_vec(x_all.T, meta.nx_bus + meta.nx_ctrl_global + meta.nx_ctrl)
    i1 = len(meta.nx_bus)
    i2 = len(meta.nx_bus) + len(meta.nx_ctrl_global)
    x_part_sys = x_part_all[:i1]
    x_part_ctrl_global = x_part_all[i1:i2]
    x_part_ctrl = x_part_all[i2:]
    V_part = V_all.reshape((t_all.size, -1, 2))
    I_part = I_all.reshape((t_all.size, -1, 2))


    res_dict: Dict[Hashable, SimulationResultComponent] = {}
    for key, index in meta.bus_index_map.items():
        res_dict[key] = SimulationResultComponent(
            x=x_part_sys[index].T,
            V=V_part[:, index],
            I=I_part[:, index],
        )

    out = SimulationResult(
        options=options,
        meta=meta,
        segments=segments,
        t=t_all,
        components=res_dict,
        ctrls_global=x_part_ctrl_global,
        ctrls=x_part_ctrl
    )

    return out

