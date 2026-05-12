from abc import ABC, abstractmethod

from g1_playground.action import JointAction
from g1_playground.state import G1State


class Policy(ABC):
    """Strategy interface for deciding the next action from the current state."""

    @property
    @abstractmethod
    def dt(self) -> float:
        """Policy step period in seconds (e.g. 0.1 for 10 Hz)."""

    @abstractmethod
    def reset(self, state: G1State) -> None:
        """Called once after the first state arrives, before any step()."""

    @abstractmethod
    def step(self, state: G1State) -> JointAction:
        """Compute the next action from the current state."""