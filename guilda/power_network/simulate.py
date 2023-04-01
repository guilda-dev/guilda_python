import numpy as np

from cmath import phase
from functools import reduce

from typing import Tuple, List, Union, Callable, Any, Optional, Iterable

from scipy.integrate import ode, odeint

from guilda.power_network.base import _PowerNetwork, _sample2f, _idx2f

from guilda.power_network.types import SimulateOptions, SimulateResult
from guilda.power_network.control import get_dx

from guilda.base import ComponentEmpty

from guilda.utils.math import complex_mat_to_float
from guilda.utils.data import complex_arr_to_col_vec
from guilda.utils.typing import ComplexArray, FloatArray


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
    

# Stability not tested!!!
def solve_ode15s(
    func: Callable[[FloatArray, float], FloatArray],
    x0: FloatArray,
    t_simulated: Tuple[float, float],
    atol: float = 1e-10, 
    rtol: float = 1e-10,
):
    solver = ode(func).set_integrator('vode', method='bdf', order=15, atol=atol, rtol=rtol)
    solver.set_initial_value(x0, t_simulated[0])
    x = x0
    while solver.successful() and solver.t < t_simulated[1]:
        x = solver.integrate(t_simulated[1])
    return x, solver.t


def simulate(
    self: _PowerNetwork, 
    t: Iterable[float], 
    u: Optional[FloatArray] = None, # (n_u, n_t)
    idx_u: Optional[Iterable[int]] = None, 
    options: Optional[SimulateOptions] = None, 
    ):
    
    # process options
    if options is None:
        options = SimulateOptions()
        
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
    options: SimulateOptions):
    
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
    if options.method == 'zoh':
        t_simulated = get_t_simulated(t_cand, uf, fault_f)
    
    # :45
    # restore simulation result
    
    sols: List[Tuple[float, FloatArray, FloatArray, FloatArray, FloatArray]] = [] # (t, x)[]
    metas: List[Tuple[float, float, List[int], List[int], FloatArray]] = []
    # TODO add reporter
    
    # initial condition
    
    x_k: FloatArray = np.vstack(x_in + xkg + xk)
    V_k: FloatArray = complex_arr_to_col_vec(np.array(V0_in))
    I_k: FloatArray = complex_arr_to_col_vec(np.array(I0_in))
    
    # add initian state as result
    
        
    # :88
    
    for i in range(len(t_simulated) - 1):
        
        tstart, tend = t_simulated[i: i + 2]
        
        f_ = fault_f((tstart + tend) / 2)
        
        except_ = set(list(f_) + idx_controller)
        simulated_bus = sorted(set(range(len(bus))) - (set(idx_non_unit) - except_))
        _, Ymat, __, Ymat_reproduce \
            = reduce_admittance_matrix(Y, simulated_bus)
        
        metas.append((
            tstart, 
            tend, 
            simulated_bus, 
            f_, 
            Ymat_reproduce
        ))
        
        idx_simulated_bus: List[int] = [2 * x - 1 for x in simulated_bus] + [ 2 * x for x in simulated_bus]
        idx_fault_bus: List[int] = reduce(lambda x, y: [*x, *y], [[x * 2 - 1, x * 2] for x in f_] + [[]])
        
        
        x = np.vstack([x_k, V_k[idx_simulated_bus], I_k[idx_fault_bus]])
        

        if options.method.lower() == 'zoh':
            u_ = uf((tstart + tend) / 2)
            func = lambda x, t: get_dx(
                linear,
                bus, controllers_global, controllers, Ymat,
                nx_bus, nx_kg, nx_k, nu_bus,
                t, x.reshape((-1, 1)), u_, idx_u, f_, simulated_bus
            ).flatten()
        else:
            us_ = uf(tstart)
            ue_ = uf(tend)
            u__ = lambda t: (ue_ * (t - tstart) + us_ * (tend - t)) / (tend - tstart)
            func = lambda x, t: get_dx(
                linear,
                bus, controllers_global, controllers, Ymat,
                nx_bus, nx_kg, nx_k, nu_bus,
                t, x.reshape((-1, 1)), u__(t), idx_u, f_, simulated_bus
            ).flatten()
            
        # :128
        nx = x_k.size
        nVI = x.size - nx
        nI = len(f_) * 2
        nV = nVI - nI
        
        if i == 0:
            
            _X = x[:nx, :].T
            _V = x[nx: nx + nV, :].T @ Ymat_reproduce.T
            _I = _V @ Ymat_all.T
            sols.append((
                tstart, 
                x,
                _X,
                _V,
                _I,
            ))
        
        
        # Mf = block_diag(np.eye(nx), np.zeros(nVI))
        
        
        # :138
        # This uses Dormand-Prince method instead of ode15s. 
        # To use ode15s, one must write his own integration.
        # TODO Mass should be added as parameter as well
        sol = odeint(func, x.flatten(), t_simulated[i: i + 2],
            # method='bdf', order=15,
            atol=options.AbsTol, rtol=options.RelTol, 
        ) # (n_x, 2)
        
        # :143~148
        
        y = sol[-1:].T # get the end solution
        V = y[nx: nx + len(idx_simulated_bus)]
        
        # calculate conditions for the next iteration
        x_k = y[0: nx]
        V_k = Ymat_reproduce @ V
        I_k = Ymat_all @ V_k
        
        X = y[:nx, :].T
        V = y[nx: nx + nV, :].T @ Ymat_reproduce.T
        I = V @ Ymat_all.T
        
        ifault = np.array([
            [x * 2 - 1, x * 2] for x in f_
        ], dtype=int) # (n_fault, 2)
        I[:, ifault.flatten()] = y[nx + nV:, :].T
        
        
        sols.append((tend, y, X, V, I))
        
    t_all, s_all, x_all, V_all, I_all = [
        np.array([x[i] for x in sols]) if i == 0 else np.hstack([x[i] for x in sols])
        for i in range(5)
    ]
    
    return (len(t_simulated), t_all, s_all, x_all, V_all, I_all), metas
    
    # out = SimulateResult(len_t_simulated=len(t_simulated))
    
        
    # # TODO maybe bug
    # out.t = t_simulated[i: i + 2]
    # X_all = np.vstack(out_X)
    # V_all = np.vstack(out_V)
    # I_all = np.vstack(out_I)
    
    # out.X = [None] * len(self.a_bus)
    # out.V = [V_all[:, i * 2 - 1: i * 2 + 1] for i in range(len(self.a_bus))]
    # out.I = [I_all[:, i * 2 - 1: i * 2 + 1] for i in range(len(self.a_bus))]
    
    # # :170
    
    
    # return out
    