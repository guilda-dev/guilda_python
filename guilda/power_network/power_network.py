import numpy as np
from numpy.linalg import inv

from scipy.optimize import root
from scipy.integrate import ode, odeint
from scipy.linalg import block_diag

from  cmath import phase
from functools import reduce

from typing import *
from numpy.typing import *

from guilda.bus.bus import Bus
from guilda.branch.branch import Branch

from guilda.power_network.types import SimulateOptions, SimulateResult
from guilda.power_network.control import get_dx
from guilda.utils.io import from_dict
from guilda.utils.data import expand_complex_arr

import time

def f_tmp(t, fs):
    idx = []
    for itr in range(fs):
        idx.append( fs[itr](t))
    return idx

def select_value(isA, A, B):
    if isA:
        return A
    else:
        return B


class PowerNetwork(object):
    
    def __init__(self):
        
        self.a_bus: List[Bus] = []
        self.a_branch: List[Branch] = []
        self.a_controller_local = []
        self.a_controller_global = []

    @property
    def x_equilibrium(self):
        return [b.component.x_equilibrium for b in self.a_bus]
    
    @property
    def V_equilibrium(self):
        return [b.V_equilibrium for b in self.a_bus]
    
    @property
    def I_equilibrium(self):
        return [b.I_equilibrium for b in self.a_bus]
    

    def add_bus(self, bus):
        if type(bus) != list:
            bus = [bus]

        for b in bus:
            if isinstance(b, Bus):
                self.a_bus.append(b)
            else:
                print("Type must a child of Bus")

    def add_branch(self, branch):
        if type(branch) != list:
            branch = [branch]

        for b in branch:
            if isinstance(b, Branch):
                self.a_branch.append(b)
            else:
                print("Type must a child of Branch")


    def get_admittance_matrix(self, a_index_bus=None):
        if not a_index_bus:
            a_index_bus = [i for i in range(len(self.a_bus))]

        n = len(self.a_bus)
        Y = np.zeros((n, n), dtype=complex)

        for br in self.a_branch:
            if (br.from_-1 in a_index_bus) and (br.to-1 in a_index_bus):
                Y_sub = br.get_admittance_matrix()
                Y[br.from_-1, br.from_-1] += Y_sub[0, 0]
                Y[br.from_-1, br.to-1]    += Y_sub[0, 1]
                Y[br.to-1, br.from_-1]    += Y_sub[1, 0]
                Y[br.to-1, br.to-1]       += Y_sub[1, 1]

        for idx in a_index_bus:
            Y[idx, idx] += self.a_bus[idx].shunt

        Ymat = np.zeros((2*n, 2*n))
        Ymat[ ::2, ::2] =  Y.real
        Ymat[ ::2,1::2] = -Y.imag
        Ymat[1::2, ::2] =  Y.imag
        Ymat[1::2,1::2] =  Y.real

        return [Y, Ymat]

    def calculate_power_flow(self):
        n = len(self.a_bus)

        def func_eq(Y, x):
            Vr = np.array([[x[i]] for i in range(0, len(x), 2)])
            Vi = np.array([[x[i]] for i in range(1, len(x), 2)])
            V = Vr + 1j*Vi

            I = Y @ V
            PQhat = V * I.conjugate()
            P  = PQhat.real
            Q  = PQhat.imag

            out = []
            for i in range(n):
                bus = self.a_bus[i]
                out_i = bus.get_constraint(V[i].real, V[i].imag, P[i], Q[i])
                out.extend(out_i[:, 0].tolist())
            return out

        Y, _ = self.get_admittance_matrix()
        x0 = [1, 0] * n

        ans = root(lambda x: func_eq(Y, x), x0, method="hybr")

        Vrans = np.array([[ans.x[i]] for i in range(0, len(ans.x), 2)])
        Vians = np.array([[ans.x[i]] for i in range(1, len(ans.x), 2)])
        Vans = Vrans + 1j*Vians

        Ians = Y @ Vans
        return [Vans, Ians]

    def set_equilibrium(self, V, I):
        for idx in range(len(self.a_bus)):
            self.a_bus[idx].set_equilibrium(V[idx][0],I[idx][0])

    def initialize(self):
        V, I = self.calculate_power_flow()
        self.set_equilibrium(V, I)

    def get_sys(self):
        #A, B, C, D, BV, DV, BI, DI, R, S
        mats = [[] for _ in range(10)]
        for b in self.a_bus:
            mat = b.component.get_linear_matrix()
            for i in range(len(mats)):
                if mat[i].shape == (0,0):
                    continue
                mats[i].append(mat[i])
        [A, B, C, D, BV, DV, BI, DI, R, S] = list(map(lambda mat: block_diag(*mat), mats))
        nI = C.shape[0]
        nx = A.shape[0]


        nV = BV.shape[1]
        nd = R.shape[1]
        nu = B.shape[1]
        nz = S.shape[0]
        [_, Ymat] = self.get_admittance_matrix()

        A11 = A
        A12 = np.hstack([BV, BI])
        A21 = np.vstack([C, np.zeros((nI, nx))])
        A22 = np.block([[DV, DI], [Ymat, -np.eye(nI)]])
        B1  = np.hstack([B, R])
        B2  = np.block([[D, np.zeros([nV, nd])], [np.zeros([nI, nu+nd])]])
        C1  = np.vstack([np.eye(nx), S, np.zeros([nI+nV, nx])])
        C2  = np.vstack([np.zeros([nx+nz, nV+nI]), np.eye(nV+nI)])

        A_  = A11-A12 @ inv(A22) @ A21
        B_  = B1-A12 @ inv(A22) @ B2
        C_  = C1-C2 @ inv(A22) @ A21
        D_  = -C2 @ inv(A22) @ B2
        return [A_, B_, C_, D_]


    @classmethod
    def __sample2f(t, u):
        if not u or not u[1]:
            return lambda _: np.array([]).reshape((0, 0))
        else:
            if u.shape[0] != len(t) and u.shape[1] == len(t):
                u = np.array(u).T
                
            def ret(T):
                ind = len(t) - 1 - ([t_ <= T for t_ in t][::-1]).index(True)
                return u[ind: ind + 1].T
            return ret
        
        
    def __idx2f(self, fault_time, idx_fault):
        if not fault_time:
            return lambda _: np.array([]).reshape((0, 0))
        else:
            if np.ndim(fault_time) != 2:
                return lambda t: select_value( (fault_time[0]<=t and t<fault_time[1]) , idx_fault, np.array([]).reshape((0, 0)) )
            else:
                fs = list( map(lambda time,idx: self.__idx2f(time, idx), fault_time, idx_fault) )
                # fault_timeとidx_faultはlist型
                return lambda t: f_tmp(t, fs)
        
                

    def simulate(
        self, 
        t: Tuple[float, float], 
        u: List[float] = None, 
        idx_u: List[int] = None, 
        options: Union[SimulateOptions, Dict[str, Any]] = {}, 
        **kwargs):
        
        # process options
        if isinstance(options, dict):
            options = from_dict(SimulateOptions, {**options, **kwargs})
        elif (isinstance(options, SimulateOptions)):
            for k, v in kwargs.items:
                options.__setattr__(k, v)
        else:
            raise TypeError()
        options.set_parameter_from_pn(self)
        
        # process u and index of u
        if u is None:
            u = []
        if idx_u is None:
            idx_u = []
        
        out = self.solve_odes(t, u, idx_u,
                                options.fault,
                                options.x0_sys,
                                options.x0_con_global,
                                options.x0_con_local,
                                expand_complex_arr(options.V0),
                                expand_complex_arr(options.I0),
                                options.linear,
                                options)
            
    def solve_odes(self, 
                   t, u, idx_u, 
                   fault: List[Tuple[Tuple[float, float], List[int]]], 
                   x: complex, xkg, xk, 
                   V0: NDArray[np.float64], I0: NDArray[np.float64], 
                   linear: bool, 
                   options: SimulateOptions):
        
        bus = self.a_bus
        controllers_global = self.a_controller_global
        controllers = self.a_controller_local
        
        fault_time = [x[1] for x in fault]
        idx_fault: List[List[int]] = [x[2] for x in fault]
        
        uf = PowerNetwork.__sample2f(t, u)
        fault_f = PowerNetwork.__idx2f(fault_time, idx_fault)
        
        t_cand = [t] + fault_time
        
        t_cand_unique = []
        t_cand_set2 = set()
        for tt in t_cand:
            if tt not in t_cand_set2:
                t_cand_unique.append(tt)
                t_cand_set2.add(tt)
        
        t_cand = t_cand_unique
        
        # :27
        
        nx_bus = [b.get_nx() for b in bus]
        nu_bus = [b.get_nu() for b in bus]
        nx_kg = [c.get_nx() for c in controllers_global]
        nx_k = [c.get_nx() for c in controllers]
        
        idx_non_unit = [b for b in bus if isinstance(b.component, ComponentEmpty)]
        idx_controller: List[Tuple[int, int]] = sorted(list(set([
            (c.index_observe, c.index_input) 
            for c in controllers + controllers_global
        ])))
        
        Y, Ymat_all = self.get_admittance_matrix()
        
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
        
            
        
        
        
        
                    
            
        
        
        
        
        
        
        
        
        
        
        
        
    

    