
from dataclasses import dataclass, field
from collections import defaultdict
from typing import List, Tuple, Union, Literal, Any, Dict, Hashable, Callable, Optional
import numpy as np
from scipy.interpolate import interp1d

from guilda.power_network.base import _PowerNetwork
from guilda.utils.typing import FloatArray, ComplexArray


@dataclass
class BusInput:
    
    index: Hashable = None
    time: List[float] = field(default_factory=list)
    type: Optional[str] = None
    value: FloatArray | Callable[[float], FloatArray] = field(default_factory=lambda: np.zeros((0, 0)))
    
    def get_interp(self):
        # san check
        if not isinstance(self.value, np.ndarray):
            raise RuntimeError('This bus input record is provided as a pure function, but not a group of interpolation points.')
        if self.value.shape[0] != len(self.time):
            raise RuntimeError('Value points and time stamps have different counts.')
        if len(self.time) == 0 or self.value.shape[0] == 0:
            raise RuntimeError('No value or time data provided.')
        
        err = (self.value[0], self.value[-1])
        kind = self.type or 'previous'
        if kind == 'zoh':
            kind = 'previous'
        if kind == 'foh':
            kind = 'linear'
        return interp1d(self.time, self.value, kind, axis=0, fill_value=err) # type: ignore
        
    
    
@dataclass
class BusFault:
    
    index: Hashable = None
    time: Tuple[float, float] = (0, 0)
    
@dataclass
class BusConnect:
    
    index: Hashable = None
    time: float = 0
    disconnect: bool = False

EVENT_INPUT = 0
EVENT_FAULT = 1
EVENT_CONNECTION = 2

@dataclass
class SimulationScenario:
    
    tstart: float = 0
    tend: float = 1
    
    V_init: Dict[Hashable, complex] = field(default_factory=dict)
    I_init: Dict[Hashable, complex] = field(default_factory=dict)
    
    x_init_sys: Dict[Hashable, FloatArray] = field(default_factory=dict)
    x_init_ctrl_global: Dict[int, FloatArray] = field(default_factory=dict)
    x_init_ctrl: Dict[int, FloatArray] = field(default_factory=dict)
    
    u: List[BusInput] = field(default_factory=list)
    fault: List[BusFault] = field(default_factory=list)
    conn: List[BusConnect] = field(default_factory=list)
    
    
    def parse(self, pn: _PowerNetwork):
        
        bus_index_map = pn.bus_index_map
        
        # get default states from network
        
        x0_sys = pn.x_equilibrium or []
        V0 = pn.V_equilibrium or []
        I0 = pn.I_equilibrium or []
        
        x0_con_local = [c.get_x0() for c in pn.a_controller_local]
        x0_con_global = [c.get_x0() for c in pn.a_controller_global]
        
        # apply custom states
        
        for index, x in self.x_init_sys.items():
            x0_sys[bus_index_map[index]] = x
            
        for index, x in self.V_init.items():
            V0[bus_index_map[index]] = x
            
        for index, x in self.I_init.items():
            I0[bus_index_map[index]] = x
            
        for index, x in self.x_init_ctrl_global.items():
            x0_con_global[index] = x
            
        for index, x in self.x_init_ctrl.items():
            x0_con_local[index] = x
        
        # check timestamps and build event record
        
        timestamps = set([self.tstart, self.tend])
        events: Dict[float, List[Tuple[object, int, bool]]] = defaultdict(list)
        events[self.tstart] = []
        events[self.tend] = []
        
        # input
        for i, u in enumerate(self.u):
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
        for i, f in enumerate(self.fault):
            t_min, t_max = f.time
            if t_max <= t_min:
                raise RuntimeError('Invalid input time duration.')
                
            events[t_min].append((f, i, True))
            events[t_max].append((f, i, False))
            
        for i, c in enumerate(self.conn):
            events[c.time].append((c, i, not c.disconnect))
            
        timestamp_list = list(timestamps)
        timestamp_list.sort()
        
        return bus_index_map, (x0_sys, x0_con_global, x0_con_local, V0, I0), timestamp_list, dict(events)
            
            
    


@dataclass
class SimulationOptions:
    
    linear: bool = False
    solver_method: str = 'ida'
    
    atol: float = 1e-8
    rtol: float = 1e-8
    
    do_report: bool = False
    do_retry: bool = True
    reset_time: float = np.inf
    OutputFcn: List[Any] = field(default_factory = list)
    
    tools: bool = False
    save_solution: bool = False
    
        
@dataclass
class SimulationSegment:
    
    time_start: float = 0
    time_end: float = 0
    
    simulated_buses: List[int] = field(default_factory=list)
    fault_buses: List[int] = field(default_factory=list)
    
    admittance: FloatArray = field(default_factory=lambda: np.zeros((0, 0)))
    
    solution: Any = None
    
        
EMPTY_ARR = np.zeros((0, 0))
EMPTY_ARR.setflags(write=False)

@dataclass
class SimulationResult:
    
    linear: bool = True
    
    segments: List[SimulationSegment] = field(default_factory=list)
    
    t: FloatArray = field(default_factory=lambda: np.zeros((0,)))
    nx_bus: List[int] = field(default_factory=list)
    nu_bus: List[int] = field(default_factory=list)
    
    x: List[FloatArray] = field(default_factory=list)
    V: List[FloatArray] = field(default_factory=list)
    I: List[FloatArray] = field(default_factory=list)
    
    
        