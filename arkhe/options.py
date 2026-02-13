import numpy as np
from typing import List, Dict, Any, Callable

class OptionModel:
    """
    [Γ_∞+46] Option models to predict the outcomes of macro actions.
    Facilitates planning in semi-Markov decision processes.
    """
    def __init__(self, name: str, initiation_set: Callable[[float], bool]):
        self.name = name
        self.initiation_set = initiation_set # states where it can start

    def predict_outcome(self, state: float) -> float:
        """
        Predicts terminal state omega.
        """
        if self.name == "ascensão": return 0.07
        if self.name == "descida": return 0.00
        return state

    def predict_reward(self, state: float) -> float:
        """
        Predicts syzygy gain.
        """
        target = self.predict_outcome(state)
        return 1.0 - abs(0.00 - target)

class HierarchicalValueFunction:
    """
    [Γ_∞+46] Evaluates both primitive and macro actions.
    Propagates rewards across levels for coherent optimization.
    """
    def __init__(self):
        self.values: Dict[str, float] = {}

    def evaluate(self, action_name: str, current_syzygy: float, reward: float) -> float:
        """
        Value update based on syzygy and reward.
        """
        alpha = 0.1
        old_val = self.values.get(action_name, current_syzygy)
        new_val = old_val + alpha * (reward - old_val)
        self.values[action_name] = new_val
        return new_val

    def propagate_rewards(self, macro_reward: float, sub_actions: List[str]):
        """
        Distributes macro rewards to sub-actions.
        """
        share = macro_reward / len(sub_actions)
        for action in sub_actions:
            val = self.values.get(action, 0.5)
            self.values[action] = val + 0.05 * share
