from abc import ABC, abstractmethod as AM
from typing import Optional, Tuple
import numpy as np

from utils.typing import FloatArray
from base.types import StateEquationRecord


class Component(ABC):
    """_summary_

    Args:
        ABC (_type_): _description_
    """

    @AM
    def set_equilibrium(self, V: complex, I: complex) -> None:
        """_summary_

        Args:
            V_eq (_type_): _description_
            I_eq (_type_): _description_
        """

    @AM
    def get_nx(self) -> int:
        """Returns the dimension of the component's state.
        """

    @AM
    def get_nu(self) -> int:
        """Returns the dimension of the component's input.
        """

    @AM
    def get_linear_matrix(self, V: complex = 0, x: Optional[FloatArray] = None) -> StateEquationRecord:
        """_summary_

        Args:
            v: voltage at equilibrium.
            x: state at equilibrium.
        """

    @AM
    def get_dx_constraint(
        self,
        V: complex = 0,
        I: complex = 0,
        x: Optional[FloatArray] = None,
        u: Optional[FloatArray] = None,
        t: float = 0) -> Tuple[FloatArray, FloatArray]:
        """_summary_

        Args:
            V (complex, optional): _description_. Defaults to 0.
            I (complex, optional): _description_. Defaults to 0.
            x (Optional[NDArray], optional): _description_. Defaults to None.
            u (Optional[NDArray], optional): _description_. Defaults to None.
            t (float, optional): _description_. Defaults to 0.

        Returns:
            _type_: _description_
        """
    @AM
    def get_dx_constraint_linear(
        self,
        V: complex = 0,
        I: complex = 0,
        x: Optional[FloatArray] = None,
        u: Optional[FloatArray] = None,
        t: float = 0) -> Tuple[FloatArray, FloatArray]:
        """_summary_

        Args:
            V (complex, optional): _description_. Defaults to 0.
            I (complex, optional): _description_. Defaults to 0.
            x (Optional[NDArray], optional): _description_. Defaults to None.
            u (Optional[NDArray], optional): _description_. Defaults to None.
            t (float, optional): _description_. Defaults to 0.

        Returns:
            _type_: _description_
        """


class ComponentEmpty(Component):

    def __init__(self):
        self.x_equilibrium = None

    def set_equilibrium(self, V, I):
        self.x_equilibrium = np.array([]).reshape(-1, 1)

    def get_nx(self) -> int:
        return 0

    def get_nu(self) -> int:
        return 0

    def get_dx_constraint(
        self,
        V: complex = 0,
        I: complex = 0,
        x: Optional[FloatArray] = None,
        u: Optional[FloatArray] = None,
        t: float = 0) -> Tuple[FloatArray, FloatArray]:
        dx: FloatArray = np.array([], dtype=np.float64).reshape(-1, 1)
        constraint: FloatArray = np.zeros([2, 1], dtype=np.float64)
        return (dx, constraint)

    def get_dx_constraint_linear(
        self,
        V: complex = 0,
        I: complex = 0,
        x: Optional[FloatArray] = None,
        u: Optional[FloatArray] = None,
        t: float = 0) -> Tuple[FloatArray, FloatArray]:
        return self.get_dx_constraint(V, I, x, u, t)

    def get_linear_matrix(self, V: complex = 0, x: Optional[FloatArray] = None) -> StateEquationRecord:
        A = np.array([]).reshape(-1, 1)
        B = np.array([]).reshape(-1, 1)
        C = np.zeros([2, 0])
        D = np.zeros([2, 0])
        BV = np.zeros([0, 2])
        DV = np.zeros([2, 2])
        R = np.array([]).reshape(-1, 1)
        S = np.array([]).reshape(-1, 1)
        DI = -np.identity(2)
        BI = np.zeros([0, 2])

        return StateEquationRecord(
            n_x=self.get_nx(), n_u=self.get_nu(),
            A=A, B=B, C=C, D=D,
            BV=BV, DV=DV, BI=BI, DI=DI,
            R=R, S=S
        )
