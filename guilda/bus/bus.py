from typing import Optional

from abc import ABC, abstractmethod as AM

from guilda.base.component import Component, ComponentEmpty
from guilda.utils.typing import FloatArray

# ComplexOrNumList = Union[complex, List[Union[int, float]]]


class Bus(ABC):

    def __init__(self, shunt: complex):
        self.V_equilibrium: Optional[complex] = None
        self.I_equilibrium: Optional[complex] = None
        self.component: Component = None # type: ignore
        self.set_component(ComponentEmpty())
        self.shunt: complex = 0
        self.set_shunt(shunt)

    @property
    def nx(self):
        return self.component.nx

    @property
    def nu(self):
        return self.component.nu

    def set_equilibrium(self, Veq: complex, Ieq: complex):
        self.V_equilibrium = Veq
        self.I_equilibrium = Ieq
        self.component.set_equilibrium(Veq, Ieq)

    def set_component(self, component):
    
        if not isinstance(component, Component):
            raise TypeError("must be a child of component")
        
        self.component = component
        if not self.V_equilibrium or not self.I_equilibrium:
            return
        
        self.component.set_equilibrium(self.V_equilibrium, self.I_equilibrium)

            
    def set_shunt(self, shunt: complex):
        self.shunt = shunt
        
    @AM
    def get_constraint(self, Vr: float, Vi: float, P: float, Q: float) -> FloatArray:
        '''_summary_

        Args:
            Vr (_type_): _description_
            Vi (_type_): _description_
            P (_type_): _description_
            Q (_type_): _description_
        '''        
