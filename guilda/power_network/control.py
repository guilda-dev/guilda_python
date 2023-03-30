from typing import *
from numpy.typing import *
import numpy as np

from guilda.bus.bus import Bus
from guilda.utils.data import sep_list

def get_dx(bus: Bus, controllers_global, controllers, Ymat: NDArray, 
           nx_bus, nx_controller_global, nx_controller, nu_bus, 
           t, x_all, u, idx_u, 
           idx_fault: List[int], simulated_bus: List[int]):
       
    n1 = np.sum([nx_bus[b] for b in simulated_bus], dtype=int)
    n2 = np.sum(nx_controller_global, dtype=int)
    n3 = np.sum(nx_controller, dtype=int)
    
    n4 = 2 * len(simulated_bus)
    n5 = 2 * len(idx_fault)
    
    x = x_all[0: n1]
    xkg = x_all[n1: n1 + n2]
    xk = x_all[n1 + n2: n1 + n2 + n3]
    
    ns = n1 + n2 + n3
    V0 = x_all[ns: ns + n4]
    V = np.reshape(V0, (2, -1))
    I_fault = np.reshape(x_all[ns + n4: ns + n4 + n5], (2, -1))
    
    I = np.reshape(Ymat @ V0, (2, -1))
    I[:, idx_fault] = I_fault
    
    Vall = np.zeros((2, len(bus)))
    Iall = np.zeros((2, len(bus)))
    
    Vall[:, simulated_bus] = V
    Iall[:, simulated_bus] = I
    
    #
    
    x_bus = [None] * len(bus)
    x_bus = sep_list(x, [nx_bus[b] for b in simulated_bus])
    
    U_bus = [None] * len(bus)
    for b in simulated_bus:
        U_bus[b] = np.zeros((nu_bus[b], 1))
        
    xkg_cell = sep_list(xkg, nx_controller_global)
    xk_cell = sep_list(xk, nx_controller)
        
    #
    
    dxkg = [None] * len(controllers_global)
    
    for i, c in enumerate(controllers_global):
        dxkg[i], ug_ = c.get_dx_u_func(
            t, xkg_cell[i], x_bus[c.index_observe], 
            Vall[:, c.index_observe], Iall[:, c.index_observe], [])
        
        idx = 0
        for i_input in np.array(c.index_input, dtype=int).flatten():
            U_bus[i_input] += ug_[idx: idx + nu_bus[i_input]]
            idx = idx + nu_bus[i_input]
            
    # 
    U_global = list(U_bus)
    
    dxk = [None] * len(controllers)
    
    for i, c in enumerate(controllers):
        dxk[i], u_ = c.get_dx_u_func(
            t, xk_cell[i], x_bus[c.index_observe], 
            Vall[:, c.index_observe], Iall[:, c.index_observe], 
            U_global[c.index_observe])
        
        idx = 0
        for i_input in np.array(c.index_input, dtype=int).flatten():
            U_bus[i_input] += u_[idx: idx + nu_bus[i_input]]
            idx = idx + nu_bus[i_input]
            
    
    
    idx = 0
    for i in idx_u:
       U_bus[i] += u[idx:idx+nu_bus[i]]
       idx += nu_bus[i]
        
    
    dx_component = [None] * len(simulated_bus)
    constraint = [None] * len(simulated_bus)
    
    for  i, idx in enumerate(simulated_bus):
        dx_component[i], constraint[i] = bus[idx].component.get_dx_con_func(
            t, 
            x_bus[idx], 
            Vall[:,idx],
            Iall[:,idx],
            U_bus[idx]
            )
        
    dx_algebraic = np.vstack([
        [constraint], 
        np.reshape(Vall[:, idx_fault], (-1, 1))
        ], dtype=np.float64)
    dx = np.vstack([
        [dx_component], 
        [dxkg], 
        [dxk], 
        dx_algebraic,
    ], dtype=np.float64)

    return dx
       
    
    