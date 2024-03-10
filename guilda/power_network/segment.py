# pylint: disable=W0640

from collections import defaultdict
import numpy as np

from functools import reduce

from typing import Set, Tuple, List, Callable, Iterable, Dict


from guilda.power_network.base import _PowerNetwork

from guilda.power_network.types import BusConnect, BusEvent, BusFault, BusInput, SimulationMetadata, SimulationSegment, SimulationScenario

from guilda.base import ComponentEmpty

from guilda.utils.calc import complex_mat_to_float
from guilda.utils.typing import ComplexArray, FloatArray



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


def parse_scenario(s: SimulationScenario, n: _PowerNetwork):

    bus_index_map = n.bus_index_map

    # get default states from network

    x0_sys = n.x_equilibrium or []
    V0 = n.V_equilibrium or []
    I0 = n.I_equilibrium or []

    x0_con_local = [c.get_x0() for c in n.a_controller_local]
    x0_con_global = [c.get_x0() for c in n.a_controller_global]

    # apply custom states

    for index, x in s.x_init_sys.items():
        x0_sys[bus_index_map[index]] = x

    for index, x in s.V_init.items():
        V0[bus_index_map[index]] = x

    for index, x in s.I_init.items():
        I0[bus_index_map[index]] = x

    for index, x in s.x_init_ctrl_global.items():
        x0_con_global[index] = x

    for index, x in s.x_init_ctrl.items():
        x0_con_local[index] = x

    # apply custom difference

    for index, x in s.dx_init_sys.items():
        x0_sys[bus_index_map[index]] += x

    for index, x in s.dx_init_ctrl_global.items():
        x0_con_global[index] += x

    for index, x in s.dx_init_ctrl.items():
        x0_con_local[index] += x

    # check timestamps and build event record

    timestamps = set([s.tstart, s.tend])
    events: Dict[float, List[Tuple[BusEvent, int, bool]]] = defaultdict(list)
    events[s.tstart] = []
    events[s.tend] = []

    # input
    for i, u in enumerate(s.u):
        t_count = 0
        t_min = np.Infinity
        t_max = -np.Infinity
        for _t in u.time:
            timestamps.add(_t)
            events[_t] = []
            t_count += 1
            t_min = min(t_min, _t)
            t_max = max(t_max, _t)
        # san check
        if t_count < 2:
            raise RuntimeError('Invalid input time duration.')
        if u.value is None:
            raise RuntimeError('Empty input record.')
        events[t_min].append((u, i, True))
        events[t_max].append((u, i, False))

    # fault
    for i, f in enumerate(s.fault):
        t_min, t_max = f.time
        if t_max <= t_min:
            raise RuntimeError('Invalid input time duration.')

        events[t_min].append((f, i, True))
        events[t_max].append((f, i, False))

    for i, c in enumerate(s.conn):
        events[c.time].append((c, i, not c.disconnect))

    timestamp_list = list(timestamps)
    timestamp_list.sort()

    # form metadata

    # build controller & index map
    
    ctrls_global = list(n.a_controller_global)
    ctrls_global_indices = [
        (
            [bus_index_map[x] for x in c.index_observe],
            [bus_index_map[x] for x in c.index_input]
        ) for c in ctrls_global
    ]
    ctrls = list(n.a_controller_local)
    ctrls_indices = [
        (
            [bus_index_map[x] for x in c.index_observe],
            [bus_index_map[x] for x in c.index_input]
        ) for c in ctrls
    ]
    
    buses = [n.a_bus_dict[i] for i in bus_index_map]
    
    nx_bus = [b.nx for b in buses]
    nu_bus = [b.nu for b in buses]
    nx_ctrl_global = [c.nx for c in ctrls_global]
    nx_ctrl = [c.nx for c in ctrls]

    Y = n.get_admittance_matrix(bus_index_map)

    meta = SimulationMetadata(
        buses=buses,
        bus_index_map=bus_index_map,
        ctrls_global=ctrls_global,
        ctrls=ctrls,
        ctrls_global_indices=ctrls_global_indices,
        ctrls_indices=ctrls_indices,
        nx_bus=nx_bus,
        nu_bus=nu_bus,
        nx_ctrl_global=nx_ctrl_global,
        nx_ctrl=nx_ctrl,
        system_admittance=Y,
        system_admittance_f=complex_mat_to_float(Y),
    )
    
    init_cond = (x0_sys, x0_con_global, x0_con_local, V0, I0)

    return meta, init_cond, timestamp_list, dict(events)


def gen_segments(

    m: SimulationMetadata,

    important_timestamps: List[float],
    events: Dict[float, List[Tuple[BusEvent, int, bool]]],

):

    # :27

    idx_empty_buses = [i for i, b in enumerate(
        m.buses) if isinstance(b.component, ComponentEmpty)]

    idx_controlled_buses: List[int] = sorted(list(set(reduce(
        lambda x, y: x + y[0] + y[1],
        m.ctrls_global_indices + m.ctrls_indices,
        []
    ))))

    # idx_controlled_buses: unique pairs of observe-input indices

    t_simulated = sorted(important_timestamps)

    # event recorders
    idx_with_input: Set[int] = set()
    input_functions: Dict[int, Callable[[float], FloatArray]] = {}
    idx_with_fault: Set[int] = set()
    idx_disconnected: Set[int] = set()

    segments: List[SimulationSegment] = []

    # in each time span
    for i in range(len(t_simulated) - 1):

        tstart, tend = t_simulated[i: i + 2]

        # handle events
        for e in events[tstart]:
            obj, index, flag = e

            # input
            if isinstance(obj, BusInput):
                b_index = m.bus_index_map[obj.index]
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
                b_index = m.bus_index_map[obj.index]
                if flag:
                    idx_with_fault.add(b_index)
                elif b_index in idx_with_fault:
                    idx_with_fault.remove(b_index)

            # connection
            # TODO install
            if isinstance(obj, BusConnect):
                b_index = m.bus_index_map[obj.index]
                if flag:
                    idx_disconnected.add(b_index)
                elif b_index in idx_disconnected:
                    idx_disconnected.remove(b_index)

        # get all buses needed to handle
        buses_fault = sorted(list(idx_with_fault))
        buses_input_list = sorted(list(idx_with_input))
        buses_disconnect = sorted(list(idx_disconnected))

        buses_input: Dict[int, Callable[[float], FloatArray]] = {}
        for b in buses_input_list:
            buses_input[b] = input_functions[b]

        must_include_buses = set(list(buses_fault) + idx_controlled_buses)

        # (all buses that are empty but neither faulted nor controlled) are excluded
        # TODO assume that this step is only for the reduction of computation...
        cur_sim_buses = sorted(set(range(len(m.buses))) -
                               (set(idx_empty_buses) - must_include_buses))

        # get reduced and reproduce admittance matrices
        _, admittance_reduced, __, admittance_reproduce \
            = reduce_admittance_matrix(m.system_admittance, cur_sim_buses)

        # create segment record
        segment = SimulationSegment(
            time_start=tstart,
            time_end=tend,

            buses_simulated=cur_sim_buses,
            buses_fault=buses_fault,
            buses_input=buses_input,
            buses_disconnect=buses_disconnect,

            admittance_reduced=admittance_reduced,
            admittance_reproduce=admittance_reproduce,
        )

        segments.append(segment)

    return segments
