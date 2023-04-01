
from dataclasses import dataclass, field
from typing import List, Tuple, Union, Literal, Any
import numpy as np

from guilda.power_network.base import _PowerNetwork
from guilda.utils.typing import ComplexArray, FloatArray

@dataclass
class SimulateOptions:
    
    linear: bool = False
    fault: List[Tuple[Tuple[float, float], List[int]]] = field(default_factory = list)
    
    x0_sys: List[FloatArray] = field(default_factory = list)
    V0: List[complex] = field(default_factory = list)
    I0: List[complex] = field(default_factory = list)
    
    x0_con_local: List[FloatArray] = field(default_factory = list)
    x0_con_global: List[FloatArray] = field(default_factory = list)
    
    method: Union[Literal['zoh'], Literal['foh']] = 'zoh'
    AbsTol: float = 1e-8
    RelTol: float = 1e-8
    
    do_report: bool = False
    do_retry: bool = True
    reset_time: float = np.inf
    OutputFcn: List[Any] = field(default_factory = list)
    tools: bool = False
    
    def set_parameter_from_pn(self, pn: _PowerNetwork):
        
        self.x0_sys = pn.x_equilibrium or []
        self.V0 = pn.V_equilibrium or []
        self.I0 = pn.I_equilibrium or []
        
        self.x0_con_local = [c.get_x0() for c in pn.a_controller_local]
        self.x0_con_global = [c.get_x0() for c in pn.a_controller_global]
        
        # TODO controller
        

        
EMPTY_ARR = np.zeros((0, 0))
EMPTY_ARR.setflags(write=False)
class SimulateResult:
    
    def __init__(self, len_t_simulated: int):
        
        
        self.simulated_bus: List[List[int]] = [[] for _ in range(len_t_simulated - 1)]
        self.fault_bus: List[List[int]] = [[] for _ in range(len_t_simulated - 1)]
        self.Ymat_reproduce: List[FloatArray] = [EMPTY_ARR for _ in range(len_t_simulated - 1)]
        self.t = None
        self.X = self.V = self.I = None
        
        