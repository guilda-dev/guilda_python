
from dataclasses import dataclass, field
from collections import defaultdict
from typing import List, Tuple, Union, Literal, Any, Dict, Hashable, Callable, Optional, cast
import numpy as np
from scipy.interpolate import interp1d
from guilda.bus.bus import Bus
from guilda.controller.controller import Controller

from guilda.power_network.base import _PowerNetwork
from guilda.utils.typing import FloatArray, ComplexArray


@dataclass
class BusEvent:
    index: Hashable = None


@dataclass
class BusInput(BusEvent):

    time: List[float] = field(default_factory=list)
    type: Optional[str] = None
    value: FloatArray | Callable[[float], FloatArray] = field(
        default_factory=lambda: np.zeros((0, 0)))

    def get_interp(self):
        # san check
        if not isinstance(self.value, np.ndarray):
            raise RuntimeError(
                'This bus input record is provided as a pure function, but not a group of interpolation points.')
        if self.value.shape[0] != len(self.time):
            raise RuntimeError(
                'Value points and time stamps have different counts.')
        if len(self.time) == 0 or self.value.shape[0] == 0:
            raise RuntimeError('No value or time data provided.')

        err = cast(float, (self.value[0], self.value[-1]))
        kind = self.type or 'previous'
        if kind == 'zoh':
            kind = 'previous'
        if kind == 'foh':
            kind = 'linear'
        return interp1d(self.time, self.value, kind, axis=0, fill_value=err)


@dataclass
class BusFault(BusEvent):

    time: Tuple[float, float] = (0, 0)


@dataclass
class BusConnect(BusEvent):

    time: float = 0
    disconnect: bool = False


@dataclass
class SimulationScenario:

    tstart: float = 0
    tend: float = 1

    V_init: Dict[Hashable, complex] = field(default_factory=dict)
    I_init: Dict[Hashable, complex] = field(default_factory=dict)

    x_init_sys: Dict[Hashable, FloatArray] = field(default_factory=dict)
    x_init_ctrl_global: Dict[int, FloatArray] = field(default_factory=dict)
    x_init_ctrl: Dict[int, FloatArray] = field(default_factory=dict)

    dx_init_sys: Dict[Hashable, FloatArray] = field(default_factory=dict)
    dx_init_ctrl_global: Dict[int, FloatArray] = field(default_factory=dict)
    dx_init_ctrl: Dict[int, FloatArray] = field(default_factory=dict)

    u: List[BusInput] = field(default_factory=list)
    fault: List[BusFault] = field(default_factory=list)
    conn: List[BusConnect] = field(default_factory=list)


@dataclass
class SimulationOptions:

    linear: bool = False
    strict_duration: bool = False  # TODO
    solver_method: str = 'ida'

    atol: float = 1e-8
    rtol: float = 1e-8
    t_interval: float = -1

    do_report: bool = False
    do_retry: bool = True
    reset_time: float = np.inf
    OutputFcn: List[Any] = field(default_factory=list)

    tools: bool = False
    save_solution: bool = False


@dataclass
class SimulationMetadata:

    buses: List[Bus]
    bus_index_map: Dict[Hashable, int]

    ctrls_global: List[Controller]
    ctrls: List[Controller]
    ctrls_global_indices: List[Tuple[List[int], List[int]]]
    ctrls_indices: List[Tuple[List[int], List[int]]]

    nx_bus: List[int]
    nu_bus: List[int]
    nx_ctrl_global: List[int]
    nx_ctrl: List[int]

    system_admittance: ComplexArray
    system_admittance_f: FloatArray


@dataclass
class SimulationSegment:

    time_start: float
    time_end: float

    buses_simulated: List[int]
    buses_fault: List[int]
    buses_input: Dict[int, Callable[[float], FloatArray]]
    buses_disconnect: List[int]  # TODO

    admittance_reduced: FloatArray
    admittance_reproduce: FloatArray


@dataclass
class SimulationResultComponent:
    x: FloatArray = field(default_factory=lambda: np.zeros((0, 0)))
    V: FloatArray = field(default_factory=lambda: np.zeros((0, 0)))
    I: FloatArray = field(default_factory=lambda: np.zeros((0, 0)))


@dataclass
class SimulationResult:

    options: SimulationOptions

    meta: SimulationMetadata
    segments: List[SimulationSegment]

    t: FloatArray
    components: Dict[Hashable, SimulationResultComponent]
    ctrls_global: List[FloatArray]
    ctrls: List[FloatArray]

    def __getitem__(self, x: Hashable):
        return self.components[x]
