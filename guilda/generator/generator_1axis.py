from typing import Optional, Tuple, Union
import numpy as np
from math import sin, cos, atan, atan2, sqrt
from cmath import phase
import control as ct
from control import StateSpace as SS

from guilda.base import Component, StateEquationRecord
from guilda.avr import Avr
from guilda.utils.data import complex_to_col_vec
from guilda.utils.typing import FloatArray

from guilda.generator.pss import Pss
from guilda.generator.governor import Governor
from guilda.generator.types import MachineParameters

class Generator1Axis(Component):
    """
モデル  ：同期発電機の一軸モデル
        ・状態：３つ「回転子偏角"δ",周波数偏差"Δω",内部電圧"E"」
              *AVRやPSSが付加されるとそれらの状態も追加される
        ・入力：２ポート「界磁入力"Vfield", 機械入力"Pmech"」
              *定常値からの追加分を指定
親クラス：componentクラス
実行方法：obj = generator_1axis(omega, parameter)
 引数 ：・omega     : float値．系統周波数(50or60*2pi)
     ・parameter : pandas.Series型．「'Xd', 'Xd_prime','Xq','T','M','D'」を列名として定義
 出力 ：componentクラスのインスタンス

    Args:
        Component (_type_): _description_
    """
    
    
    
    @property
    def x_equilibrium(self) -> FloatArray:
        return self.__x_eq
    

    def __init__(self, omega: float, parameter: MachineParameters):
        super().__init__()
        
        self.parameter: MachineParameters = parameter

        self.__x_eq: FloatArray = np.zeros((0, 0))
        
        self.alpha_st: FloatArray = np.zeros((0, 0))
        self.avr = Avr()
        self.governor = Governor()
        self.pss = Pss()
        self.omega0: float = omega

        # 近似線形システムの行列を辞書タイプで収納
        self.system_matrix: StateEquationRecord = StateEquationRecord()

    def get_x_name(self):
        gen_state = ['delta', 'omega', 'Ed']
        avr_state = self.avr.get_state_name()
        pss_state = self.pss.get_state_name()
        governor_state = self.governor.get_state_name()
        name_tag = gen_state + avr_state + pss_state + governor_state
        return name_tag

    def get_port_name(self):
        return ['Vfd', 'Pm']

    def get_nx(self):
        out = 3 + self.avr.get_nx() + self.pss.get_nx() + self.governor.get_nx()
        return out

    def get_nu(self):
        return 2

    def get_dx_constraint(
        self,
        V: complex = 0,
        I: complex = 0,
        x: Optional[FloatArray] = None,
        u: Optional[FloatArray] = None,
        t: float = 0) -> Tuple[FloatArray, FloatArray]:
        
        assert x is not None
        assert u is not None
        
        omega0 = self.omega0
        
        Xd = self.parameter.Xd
        Xdp = self.parameter.Xd_prime
        Xq = self.parameter.Xq
        Tdo = self.parameter.T
        M = self.parameter.M
        d = self.parameter.D
        
        nx = 3
        
        nx_avr = self.avr.get_nx()
        nx_pss = self.pss.get_nx()
        nx_gov = self.governor.get_nx()

        x_gen = x[0:nx]
        x_avr = x[nx:nx+nx_avr]
        x_pss = x[nx+nx_avr:nx+nx_avr+nx_pss]
        x_gov: FloatArray = x[nx+nx_avr+nx_pss:nx+nx_avr+nx_pss+nx_gov]

        Vabs = abs(V)
        Vangle = atan2(V.imag, V.real)

        delta: float = x_gen[0, 0]
        omega: float = x_gen[1, 0]
        E = x_gen[2, 0]

        Vabscos = V.real*cos(delta) + V.imag*sin(delta)
        Vabssin = V.real*sin(delta) - V.imag*cos(delta)

        Ir = (E-Vabscos)*sin(delta)/Xdp + Vabssin*cos(delta)/Xq
        Ii = -(E-Vabscos)*cos(delta)/Xdp + Vabssin*sin(delta)/Xq

        con = np.array([[I.real], [I.imag]]) - np.array([[Ir], [Ii]])

        Efd = Xd*E/Xdp - (Xd/Xdp - 1)*Vabscos

        # スカラーを返す
        [dx_pss, v] = self.pss.get_u(x_pss, omega)
        [dx_avr, Vfd] = self.avr.get_Vfd(
            x_avr=x_avr, Vabs=Vabs, Efd=Efd, u=u[0, 0]-v)
        [dx_gov, P] = self.governor.get_P(u=u[1, 0])

        dE = (-Efd + Vfd)/Tdo
        ddelta_: float = omega0*omega
        domega_: float = (P - d*omega - Vabs*E*sin(delta-Vangle)/Xdp +
                  Vabs**2*(1/Xdp-1/Xq)*sin(2*(delta-Vangle))/2)/M

        dE = np.array(dE).reshape(-1, 1)
        ddelta = np.array(ddelta_).reshape(-1, 1)
        domega = np.array(domega_).reshape(-1, 1)
        dx_pss = np.array(dx_pss).reshape(-1, 1)
        dx_avr = np.array(dx_avr).reshape(-1, 1)
        dx_gov = np.array(dx_gov).reshape(-1, 1)

        dx = np.vstack((ddelta, domega, dE, dx_avr, dx_pss, dx_gov))

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

    def get_linear_matrix(self, V: complex = 0, x: Optional[FloatArray] = None) -> StateEquationRecord:
        if (x is None or not any(x)) and V is None:
            return self.system_matrix.copy()

        if x is None or not any(x):
            x = self.x_equilibrium

        if V is None:
            V = self.V_equilibrium

        omega0 = self.omega0
        Xd = self.parameter.Xd
        Xdp = self.parameter.Xd_prime 
        Xq = self.parameter.Xq 
        Tdo = self.parameter.T 
        M = self.parameter.M 
        d = self.parameter.D 

        A_swing = np.array([[0, omega0, 0],
                            [0, -d/M,   0],
                            [0, 0,      0]])
        # u1 = Pmech
        # u2 = Vfd
        # u3 = Pout
        # u4 = Efd
        B_swing = np.array([[0,   0,     0,    0],
                            [1/M, 0,     -1/M, 0],
                            [0,   1/Tdo, 0,    -1/Tdo]])

        # y1 = delta
        # y2 = omega
        # y3 = E
        C_swing = np.identity(3)

        D_swing = np.zeros([3, 4])

        sys_swing = SS(A_swing, B_swing, C_swing, D_swing)

        swing_inputs = ['Pmech', 'Vfd', 'Pout', 'Efd_swing']
        swing_outputs = ['delta', 'omega', 'E']
        SS.set_inputs(sys_swing, swing_inputs)
        SS.set_outputs(sys_swing, swing_outputs)

        #状態の平衡点を取得
        delta = x[0, 0]
        E = x[2, 0]

        # 以下、行ベクトル
        dVabscos_dV = np.array([cos(delta), sin(delta)]).reshape(1, -1)
        dVabssin_dV = np.array([sin(delta), -cos(delta)]).reshape(1, -1)
        dIr_dV = -dVabscos_dV*sin(delta)/Xdp + dVabssin_dV*cos(delta)/Xq
        dIi_dV = dVabscos_dV*cos(delta)/Xdp + dVabssin_dV*sin(delta)/Xq

        # 以下、スカラー
        Vabscos = V.real*cos(delta) + V.imag*sin(delta)
        Vabssin = V.real*sin(delta) - V.imag*cos(delta)
        dVabscos = -Vabssin
        dVabssin = Vabscos

        # 以下、行ベクトル
        vec1 = np.concatenate(
            [np.array([[dVabscos]]), np.array([[0]]), dVabscos_dV], axis=1)
        vec2 = np.array([0, Xd/Xdp, 0, 0]).reshape(1, -1)
        dEfd = -vec1*(Xd/Xdp-1) + vec2

        # 以下、スカラー
        dIr_dd = (-dVabscos*sin(delta)+(E-Vabscos)*cos(delta)) / \
            Xdp + (dVabssin*cos(delta)-Vabssin*sin(delta))/Xq
        dIi_dd = (dVabscos*cos(delta)+(E-Vabscos)*sin(delta)) / \
            Xdp + (dVabssin*sin(delta)+Vabssin*cos(delta))/Xq

        Ieq_vec = np.array([[(E-Vabscos)*sin(delta)/Xdp + Vabssin*cos(delta)/Xq],
                            [-(E-Vabscos)*cos(delta)/Xdp + Vabssin*sin(delta)/Xq]])

        # (delta, E, V) => (Ir, Ii)
        KI_vec1 = np.concatenate(
            [np.array([[dIr_dd]]), np.array([[sin(delta)/Xdp]]), dIr_dV], axis=1)
        KI_vec2 = np.concatenate(
            [np.array([[dIi_dd]]), np.array([[-cos(delta)/Xdp]]), dIi_dV], axis=1)
        KI = np.concatenate([KI_vec1, KI_vec2], axis=0)

        vec3 = np.concatenate([np.zeros([2, 2]), np.identity(2)], axis=1)
        dP = np.array([V.real, V.imag]).reshape(1, -1) @ KI + Ieq_vec.T @ vec3

        # sys_fbの直達行列(4×4)
        D_fb = np.concatenate([dP, dEfd, KI], axis=0)
        A_fb = np.array([]).reshape(1, -1)
        B_fb = np.array([]).reshape(-1, 4)
        C_fb = np.array([]).reshape(4, -1)
        sys_fb = SS(A_fb, B_fb, C_fb, D_fb)

        fb_inputs = ['delta', 'E', 'Vr', 'Vi']
        fb_outputs = ['P', 'Efd', 'Ir', 'Ii']
        SS.set_inputs(sys_fb, fb_inputs)
        SS.set_outputs(sys_fb, fb_outputs)

        Vabs = abs(V)
        # sys_Vの直達行列(3×2)
        D_V = np.concatenate([np.identity(2), np.array(
            [V.real, V.imag]).reshape(1, -1)/Vabs], axis=0)
        A_V = np.array([]).reshape(1, -1)
        B_V = np.array([]).reshape(-1, 2)
        C_V = np.array([]).reshape(3, -1)
        sys_V = SS(A_V, B_V, C_V, D_V)

        V_inputs = ['Vrin', 'Viin']
        V_outputs = ['Vr', 'Vi', 'Vabs']
        SS.set_inputs(sys_V, V_inputs)
        SS.set_outputs(sys_V, V_outputs)

        sys_avr = self.avr.get_sys()
        sys_pss = self.pss.get_sys()
        sys_gov = self.governor.get_sys()

        #interconnectを使うためにはIOSystemでなくてはいけない
        io_swing = ct.ss2io(sys_swing, name='sys_swing')
        io_fb = ct.ss2io(sys_fb, name='sys_fb')
        io_V = ct.ss2io(sys_V, name='sys_V')
        io_avr = ct.ss2io(sys_avr, name='sys_avr')
        io_pss = ct.ss2io(sys_pss, name='sys_pss')
        io_gov = ct.ss2io(sys_gov, name='sys_gov')

        # 内部のつながりと外部の入出力を設定
        io_closed = ct.interconnect(
            [io_swing, io_fb, io_V, io_avr, io_pss, io_gov],
            connections=[
                ['sys_swing.Pout', 'sys_fb.P'],
                ['sys_avr.Efd', 'sys_fb.Efd'],
                ['sys_swing.Efd_swing', 'sys_fb.Efd'],
                ['sys_fb.delta', 'sys_swing.delta'],
                ['sys_fb.E', 'sys_swing.E'],
                ['sys_fb.Vr', 'sys_V.Vr'],
                ['sys_fb.Vi', 'sys_V.Vi'],
                ['sys_avr.Vabs', 'sys_V.Vabs'],
                ['sys_swing.Vfd', 'sys_avr.Vfd'],
                ['sys_avr.u_avr', 'sys_pss.v_pss'],
                ['sys_pss.omega', 'sys_swing.omega'],
                ['sys_gov.omega_governor', 'sys_swing.omega'],
                ['sys_swing.Pmech', 'sys_gov.Pmech']
            ],
            inplist=['sys_avr.u_avr', 'sys_gov.u_governor',
                     'sys_V.Vrin', 'sys_V.Viin'],
            outlist=['sys_fb.Ir', 'sys_fb.Ii']
        )

        # InputOutputオブジェクトをStateSpaceオブジェクトに変換
        ss_closed = SS(io_closed)
        # 入出力名を新たにつける
        system_inputs = ['u_avr', 'u_governor', 'Vrin', 'Viin']
        system_outputs = ['Ir', 'Ii']
        SS.set_inputs(ss_closed, system_inputs)
        SS.set_outputs(ss_closed, system_outputs)

        # ret_uの作成
        
        idx_u_avr: Union[int, None] = SS.find_input(ss_closed, 'u_avr')
        idx_u_gov: Union[int, None] = SS.find_input(ss_closed, 'u_governor')
        idx_u_vrin: Union[int, None] = SS.find_input(ss_closed, 'Vrin')
        idx_u_viin: Union[int, None] = SS.find_input(ss_closed, 'Viin')
        
        assert idx_u_avr is not None
        assert idx_u_gov is not None
        assert idx_u_viin is not None
        assert idx_u_vrin is not None
        
        _A: FloatArray = ss_closed.A # type: ignore
        _B: FloatArray = ss_closed.B # type: ignore
        _C: FloatArray = ss_closed.C # type: ignore
        _D: FloatArray = ss_closed.D # type: ignore
        
        A = _A
        B = _B[:, idx_u_avr: idx_u_gov + 1]
        C = _C
        D = _D[:, idx_u_avr: idx_u_gov + 1]
        
        BV = _B[:, idx_u_vrin: idx_u_viin + 1]
        DV = _D[:, idx_u_vrin: idx_u_viin + 1]
        
        BI = np.zeros([A.shape[0], 2])
        DI = -np.identity(2)
        R = np.array([[]]).reshape(self.get_nx(), -1)
        S = np.array([[]]).reshape(-1, self.get_nx())

        return StateEquationRecord(
            n_x=self.get_nx(), n_u=self.get_nu(),
            A=A, B=B, C=C, D=D,
            BV=BV, DV=DV, BI=BI, DI=DI,
            R=R, S=S
        )

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


    def set_equilibrium(self, V: complex, I: complex) -> None:
        super().set_equilibrium(V, I)
        Vabs = abs(V)
        Vangle = phase(V)
        Pow = I.conjugate() * V
        P = Pow.real
        Q = Pow.imag

        Xd = self.parameter.Xd 
        Xdp = self.parameter.Xd_prime 
        Xq = self.parameter.Xq 

        delta = Vangle + atan(P/(Q+(Vabs**2)/Xq))
        Enum = Vabs**4 + Q**2*Xdp*Xq + Q*Vabs**2*Xdp + Q*Vabs**2*Xq + P**2*Xdp*Xq
        Eden = Vabs*sqrt(P**2*Xq**2 + Q**2*Xq**2 + 2*Q*Vabs**2*Xq + Vabs**4)
        E = Enum/Eden

        Vfd = Xd*E/Xdp - (Xd/Xdp-1)*Vabs*cos(delta-Vangle)
        x_gen = np.array([[delta], [0], [E]])
        x_avr = self.avr.initialize(Vfd, Vabs)
        x_gov = self.governor.initialize(P)
        x_pss = self.pss.initialize()
        x_st = np.vstack((x_gen, x_avr, x_gov, x_pss))
        self.alpha_st = np.array([P, Vfd, Vabs]).reshape(-1, 1)
        
        self.__x_eq = x_st
        self.set_linear_matrix(V, x_st)
        
        # return x_st
