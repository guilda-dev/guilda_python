
from dataclasses import dataclass, field
from typing import List, Tuple, Union, Literal, Any
import numpy as np

from guilda.power_network.base import _PowerNetwork
from guilda.backend import ArrayProtocol

@dataclass
class SimulationOptions:
    
    linear: bool = False
    fault: List[Tuple[Tuple[float, float], List[int]]] = field(default_factory = list)
    
    x0_sys: List[ArrayProtocol] = field(default_factory = list)
    V0: List[complex] = field(default_factory = list)
    I0: List[complex] = field(default_factory = list)
    
    x0_con_local: List[ArrayProtocol] = field(default_factory = list)
    x0_con_global: List[ArrayProtocol] = field(default_factory = list)
    
    method: Union[Literal['zoh'], Literal['foh']] = 'zoh'
    solver_method: str = 'ida'
    
    atol: float = 1e-8
    rtol: float = 1e-8
    
    do_report: bool = False
    do_retry: bool = True
    reset_time: float = np.inf
    OutputFcn: List[Any] = field(default_factory = list)
    
    tools: bool = False
    save_solution: bool = False
    
    def set_parameter_from_pn(self, pn: _PowerNetwork):
        
        self.x0_sys = pn.x_equilibrium or []
        self.V0 = pn.V_equilibrium or []
        self.I0 = pn.I_equilibrium or []
        
        self.x0_con_local = [c.get_x0() for c in pn.a_controller_local]
        self.x0_con_global = [c.get_x0() for c in pn.a_controller_global]
        
        # TODO controller
        
@dataclass
class SimulationSegment:
    
    time_start: float = 0
    time_end: float = 0
    
    simulated_bus: List[int] = field(default_factory=list)
    fault_bus: List[int] = field(default_factory=list)
    
    impedance_matrix: ArrayProtocol = field(default_factory=lambda: np.zeros((0, 0)))
    
    solution: Any = None
    
        
EMPTY_ARR = np.zeros((0, 0))
EMPTY_ARR.setflags(write=False)

@dataclass
class SimulationResult:
    
    linear: bool = True
    
    segments: List[SimulationSegment] = field(default_factory=list)
    
    t: ArrayProtocol = field(default_factory=lambda: np.zeros((0,)))
    nx_bus: List[int] = field(default_factory=list)
    nu_bus: List[int] = field(default_factory=list)
    
    x: List[ArrayProtocol] = field(default_factory=list)
    V: List[ArrayProtocol] = field(default_factory=list)
    I: List[ArrayProtocol] = field(default_factory=list)
    
    
        