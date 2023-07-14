from typing import Optional, Tuple, List
import numpy as np
from math import sin, cos, atan, atan2
from cmath import phase

from guilda.base import Component, StateEquationRecord
from guilda.avr import Avr
from guilda.utils import as_dict
from guilda.utils.data import complex_to_col_vec
from guilda.utils.typing import FloatArray

from guilda.generator.pss import Pss
from guilda.generator.governor import Governor
from guilda.generator.types import GeneratorParameters

class Generator(Component):
    '''
モデル：同期発電機
Actually, it is the classical model
    '''
    
    
    @property
    def x_equilibrium(self) -> FloatArray:
        return self._x_eq
    

    def __init__(self, omega: float, parameter: GeneratorParameters):
        super().__init__()
        
        self.parameter: GeneratorParameters = GeneratorParameters(**as_dict(parameter))

        self._x_eq: FloatArray = np.zeros((0, 0))
        
        self.alpha_st: FloatArray = np.zeros((0, 0))
        self.avr = Avr()
        self.governor = Governor()
        self.pss = Pss()
        self.omega0: float = omega

        # 近似線形システムの行列を辞書タイプで収納
        self.system_matrix: StateEquationRecord = StateEquationRecord()
        
    # states
        
    def get_components_x_name(self) -> List[str]:
        avr_state = self.avr.get_state_name()
        pss_state = self.pss.get_state_name()
        governor_state = self.governor.get_state_name()
        return avr_state + pss_state + governor_state
    
    def get_self_x_name(self) -> List[str]:
        return ['delta', 'omega']
    
    def get_x_name(self) -> List[str]:
        return self.get_self_x_name() + self.get_components_x_name()

    # inputs
    def get_port_name(self):
        return ['Vfd', 'Pm']
    
    # components

    def set_avr(self, input_avr: Avr):
        assert isinstance(input_avr, Avr), 'input_avr must be subclass of Avr class'
        self.avr = input_avr

    def set_pss(self, input_pss: Pss):
        assert isinstance(input_pss, Pss), 'input_pss must be subclass of Pss class'
        self.pss = input_pss

    def set_governor(self, input_gov: Governor):
        assert isinstance(input_gov, Governor), 'input_gov must be subclass of Governor class'
        self.governor = input_gov



    def set_linear_matrix(self, Veq: complex = 0, xeq: Optional[FloatArray] = None):
        if self.omega0 == None:
            return
        
        E: StateEquationRecord = self.get_linear_matrix(Veq, xeq)
        self.system_matrix = E.copy()


    @property
    def nx_gen(self):
        return 2

    @property
    def nx(self):
        out = self.nx_gen + self.avr.nx + self.pss.nx + self.governor.nx
        return out

    @property
    def nu(self):
        return 2
    
    @property
    def nl(self):
        return 0

    def get_components_dx(self, x: FloatArray, u: FloatArray, omega: float, Vabs: float, Efd: complex):
        
        nx = self.nx_gen
        
        nx_avr = self.avr.nx
        nx_pss = self.pss.nx
        nx_gov = self.governor.nx

        x_avr: FloatArray = x[nx:nx+nx_avr]
        x_pss: FloatArray = x[nx+nx_avr:nx+nx_avr+nx_pss]
        x_gov: FloatArray = x[nx+nx_avr+nx_pss:nx+nx_avr+nx_pss+nx_gov]
        
        dx_pss, v = self.pss.get_u(x_pss, omega)
        dx_avr, Vfd = self.avr.get_Vfd(
            x_avr=x_avr, Vabs=Vabs, Efd=Efd, u=u[0:1, :]-v)
        dx_gov, P = self.governor.get_P(x_gov, u[1:2, :])
        
        return dx_avr, dx_pss, dx_gov, Vfd, P
        

    def get_dx_constraint(
        self,
        V: complex = 0,
        I: complex = 0,
        x: Optional[FloatArray] = None,
        u: Optional[FloatArray] = None,
        t: float = 0
    ) -> Tuple[FloatArray, FloatArray]:
        
        assert x is not None
        assert u is not None

        Vabs = abs(V)
        Vangle = atan2(V.imag, V.real)

        delta: float = x[0, 0]
        omega: float = x[1, 0]
        
        Efd = 0
        
        dx_avr, dx_pss, dx_gov, \
        Vfd, P = self.get_components_dx(x, u, omega, Vabs, Efd)
        
        Xd = self.parameter.Xd
        Xq = self.parameter.Xq
        M = self.parameter.M
        d = self.parameter.D
        
        Vd = V.real * sin(delta) - V.imag * cos(delta)
        Vq = V.real * cos(delta) + V.imag * sin(delta)
        Id = (Vfd - Vq) / Xd
        Iq = Vd / Xq
        
        Ir = Id * sin(delta) + Iq * cos(delta)
        Ii = -Id * cos(delta) + Iq * sin(delta)
        
        dDelta = self.omega0 * omega
        dOmega = P - d * omega - Vq * Iq - Vd * Id
        
        con = np.array([[I.real - Ir], [I.imag - Ii]])
        
        dx_gen = [[dDelta], [dOmega]]

        dx = np.vstack((dx_gen, dx_avr, dx_pss, dx_gov))

        return dx, con

    def get_dx_constraint_linear(
        self,
        V: complex = 0,
        I: complex = 0,
        x: Optional[FloatArray] = None,
        u: Optional[FloatArray] = None,
        t: float = 0) -> Tuple[FloatArray, FloatArray]:
        assert x is not None
        assert u is not None
        
        A = self.system_matrix.A 
        B = self.system_matrix.B 
        C = self.system_matrix.C 
        D = self.system_matrix.D 
        BV = self.system_matrix.BV 
        DV = self.system_matrix.DV 
        BI = self.system_matrix.BI 
        DI = self.system_matrix.DI 
        dx = A @ (x-self.x_equilibrium) + B @ u + \
            BV @ complex_to_col_vec(V-self.V_equilibrium) + BI @ complex_to_col_vec(I-self.I_equilibrium)
        con = C @ (x-self.x_equilibrium) + D @ u + \
            DV @ complex_to_col_vec(V-self.V_equilibrium) + DI @ complex_to_col_vec(I-self.I_equilibrium)

        return dx, con

    def get_linear_matrix(
        self, 
        V: complex = 0, 
        x: Optional[FloatArray] = None) -> StateEquationRecord:
       
        raise RuntimeError("Not Implemented")

    def get_self_equilibrium(self, V: complex, I: complex):
        
        Vabs = abs(V)
        Vangle = phase(V)
        Pow = I.conjugate() * V
        P = Pow.real
        Q = Pow.imag
        
        Xd: float = self.parameter.Xd 
        Xq: float = self.parameter.Xq 
        
        delta: float = Vangle + atan(P/(Q+(Vabs**2)/Xq))
        
        Id: complex = (1j * I * (cos(delta) - 1j * sin(delta)))
        Vq: complex = (1j * V * (cos(delta) - 1j * sin(delta)))

        Vfd = Id * Xd + Vq
        
        x_gen = np.array([[delta], [0]])
        
        return x_gen, Vfd

    def set_equilibrium(self, V: complex, I: complex) -> None:
        super().set_equilibrium(V, I)
        
        Vabs = abs(V)
        Pow = I.conjugate() * V
        P = Pow.real
        
        x_gen, Vfd = self.get_self_equilibrium(V, I)
        x_avr = self.avr.initialize(Vfd, Vabs)
        x_gov = self.governor.initialize(P)
        x_pss = self.pss.initialize()
        x_st = np.vstack((x_gen, x_avr, x_gov, x_pss))
        self.alpha_st = np.array([P, Vfd, Vabs]).reshape(-1, 1)
        
        self._x_eq = x_st
        self.set_linear_matrix(V, x_st)
        