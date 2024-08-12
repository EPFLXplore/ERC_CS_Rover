from abc import ABC, abstractmethod

# Abstract Parent Class
class SubSystem(ABC):
    
    @abstractmethod
    def make_action(self, goal_handle):
        pass
    
    @abstractmethod
    def action_status(sel, goal):
        pass