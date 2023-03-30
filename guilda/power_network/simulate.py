import numpy as np

from cmath import phase
from functools import reduce

from typing import Tuple, List, Union, Callable, Any, Optional

from scipy.integrate import ode, odeint

from guilda.power_network.base import _PowerNetwork, _sample2f, _idx2f

from guilda.power_network.types import SimulateOptions, SimulateResult
from guilda.power_network.control import get_dx

from guilda.base import ComponentEmpty

from guilda.utils.io import from_dict
from guilda.utils.data import expand_complex_arr
from guilda.utils.math import complex_square_mat_to_float
from guilda.utils.typing import ComplexArray, FloatArray


def get_t_simulated(
    t_cand, 
    uf: Callable[[float], FloatArray], 
    fault_f: Callable[[float], List[int]]):
    has_difference = np.ones((len(t_cand), 1), dtype=bool) 
    u: FloatArray = np.zeros((0, 0)) 
    f: List[int] = [] 
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


def simulate(
    self: _PowerNetwork, 
    t: Tuple[float, float], 
    u: Optional[List[float]] = None, 
    idx_u: Optional[List[int]] = None, 
    options: Optional[SimulateOptions] = None, 
    ):
    
    # process options
    if options is None:
        options = SimulateOptions()
        
    options.set_parameter_from_pn(self)
    
    # process u and index of u
    if u is None:
        u = []
    if idx_u is None:
        idx_u = []
    
    out = solve_odes(
        self, t, u, idx_u,
        options.fault,
        options.x0_sys,
        options.x0_con_global,
        options.x0_con_local,
        expand_complex_arr(options.V0),
        expand_complex_arr(options.I0),
        options.linear,
        options
    )
    
    return out
        
def solve_odes(self: _PowerNetwork, 
                t: Tuple[float, float], u, idx_u, 
                fault: List[Tuple[Tuple[float, float], List[int]]], 
                x: complex, xkg, xk, 
                V0: ComplexArray, I0: ComplexArray, 
                linear: bool, 
                options: SimulateOptions):
    
    bus = self.a_bus
    controllers_global = self.a_controller_global
    controllers = self.a_controller_local
    
    fault_time: List[Tuple[float, float]] = [x[0] for x in fault]
    idx_fault: List[List[int]] = [x[1] for x in fault]
    
    uf = _sample2f(t, u)
    fault_f = _idx2f(fault_time, idx_fault)
    
    t_cand: List[Tuple[float, float]] = [t] + fault_time
    t_cand = sorted(list(set(t_cand)))
    
    
    # :27
    
    nx_bus = [b.get_nx() for b in bus]
    nu_bus = [b.get_nu() for b in bus]
    nx_kg = [c.get_nx() for c in controllers_global]
    nx_k = [c.get_nx() for c in controllers]
    
    idx_non_unit = [b for b in bus if isinstance(b.component, ComponentEmpty)]
    idx_controller: List[Tuple[int, int]] = sorted(list(set(reduce(lambda x, y: [*x, *y], [
        list(zip(c.index_observe, c.index_input))
        for c in controllers + controllers_global
    ])))) # unique pairs of observe-input indices
    
    Y = self.get_admittance_matrix()
    Ymat_all = complex_square_mat_to_float(Y)
    
    t_simulated = t_cand
    if options.method == 'zoh':
        t_simulated = get_t_simulated(t_cand, uf, fault_f)
    
    # :45
    
    sols = [None] * (len(t_simulated) - 1)
    reporter = tools.Reporter(
        t_simulated[0], t_simulated[-1], 
        options.do_report, options.OutputFcn)
    out_X = [None] * (len(t_simulated) - 1)
    out_V = [None] * (len(t_simulated) - 1)
    out_I = [None] * (len(t_simulated) - 1)
    x0 = np.hstack([x, xkg, xk], dtype=np.float64)
    
    for c in controllers_global + controllers:
        c.get_dx_u_func = c.get_dx_u_linear if linear else c.get_dx_u
        
    for b in bus:
        c = bus[i].component
        c.get_dx_con_func = c.get_dx_constraint_linear \
            if linear else c.get_dx_constraint
        
    # :88
    out = SimulateResult(len_t_simulated=len(t_simulated))
    
    for i in range(len(t_simulated) - 1):
        f_ = fault_f((t_simulated[i] + t_simulated[i + 1]) / 2)
        except_ = set(list(f_) + idx_controller)
        simulated_bus = set(range(len(bus))) - (set(idx_non_unit) - except_)
        _, Ymat, __, Ymat_reproduce = self.reduce_admittance_matrix(Y, simulated_bus)
        out.simulated_bus[i] = simulated_bus
        out.fault_bus[i] = f_
        out.Ymat_reproduce[i] = Ymat_reproduce
        
        idx_simulated_bus = [2 * simulated_bus - 1, 2 * simulated_bus]
        idx_fault_bus = [[x * 2 - 1, x * 2] for x in f_]
        idx_fault_bus = reduce(lambda x, y: x + y, idx_fault_bus)
        
        x_func = lambda x0: np.hstack([x0, V0[simulated_bus], I0[idx_fault_bus]], dtype=np.float64)
        x = x0
        
        if options.method == 'zoh':
            u_ = uf((t_simulated[i] + t_simulated[i + 1]) / 2)
            func = lambda t, x: get_dx(
                bus, controllers_global, controllers, Ymat,
                nx_bus, nx_kg, nx_k, nu_bus,
                t, x_func(x), u_, idx_u, f_, simulated_bus
            )
        else:
            us_ = uf(t_simulated[i])
            ue_ = uf(t_simulated[i + 1])
            u_ = lambda t: (ue_ * (t - t_simulated[i]) + us_ * (t_simulated[i + 1] - t)) / (t_simulated[i + 1] - t_simulated[i])
            func = lambda t, x: get_dx(
                bus, controllers_global, controllers, Ymat,
                nx_bus, nx_kg, nx_k, nu_bus,
                t, x_func(x), u_(t), idx_u, f_, simulated_bus
            )
            
        # :128
        nx = x0.size
        nVI = x.size - nx
        nI = f_.size * 2
        nV = nVI - nI
        # Mf = block_diag(np.eye(nx), np.zeros(nVI))
        # TODO MASS
        
        t_now = time.time()
        r = lambda t, y, flag: reporter.report(t, y, flag, options.reset_time, t_now)
        
        # :138
        sol = odeint(func, x, t_simulated[i: i + 2],
            method='bdf', order=15,
            atol=options.AbsTol, rtol=options.RelTol, 
        )
        tend = t_simulated[i + 1]
        
        # :143~148
        
        y = sol[:, -1:]
        V = y[nx: nx + len(idx_simulated_bus)]
        x0 = y[0: nx]
        V0 = Ymat_reproduce @ V
        I0 = Ymat_all @ V0
        sols[i] = sol
        
        X = y[:nx, :].T
        V = y[nx: nx + nV, :].T @ Ymat_reproduce.T
        I = V @ Ymat_all.T
        
        ifault = np.hstack([f_.flatten() * 2 - 1, f_.flatten() * 2], dtype=np.int)
        I[:, ifault.flatten()] = y[nx + nv:, :].T
        
        out_X[i] = X
        out_V[i] = V
        out_I[i] = I
        
    # TODO maybe bug
    out.t = t_simulated[i: i + 2]
    X_all = np.vstack(out_X)
    V_all = np.vstack(out_V)
    I_all = np.vstack(out_I)
    
    out.X = [None] * len(self.a_bus)
    out.V = [V_all[:, i * 2 - 1: i * 2 + 1] for i in range(len(self.a_bus))]
    out.I = [I_all[:, i * 2 - 1: i * 2 + 1] for i in range(len(self.a_bus))]
    
    # :170
    