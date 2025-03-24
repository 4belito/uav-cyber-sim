from __future__ import annotations
from typing import Callable, Optional
from pymavlink import mavutil
from typing import List,Union




class State:
    NOT_STARTED = "NOT_STARTED"
    IN_PROGRESS = "IN_PROGRESS"
    DONE = "DONE"
    FAILED = "FAILED"


class StepFailed(Exception):
    """Raised when a step fails due to known issues (like battery too low, GPS not ready, etc)."""
    pass



class MissionElement:
    def __init__(self, 
                 name: str, 
                 check_fn: Callable[[mavutil.mavlink_connection], bool], 
                 exec_fn: Optional[Callable[[mavutil.mavlink_connection], None]] = None
                ) -> None:
        self.name = name
        self.check_fn = check_fn
        self.exec_fn = exec_fn or (lambda conn: None)
        self.next = None
        self.state = State.NOT_STARTED
        self.conn = None
        print(f"{self.__class__.__name__} '{self.name}' created — no connection yet 🧩")


    def execute(self,conn:mavutil.mavlink_connection)->None:
        class_name = self.__class__.__name__
        try:
            self.bind_connection(conn)
            self.exec_fn(self.conn)
            print(f"▶️ Starting {class_name}: {self.name}")
            self.state = State.IN_PROGRESS
        except Exception as e:
            print(f"❌ {class_name} '{self.name}' execution failed: {e}")
            self.state = State.FAILED

    def check(self)->None:
        class_name = self.__class__.__name__
        try:
            if self.check_fn(self.conn):
                print(f"✅ {class_name}: {self.name} is done")
                self.state = State.DONE
        except StepFailed as e:
            print(f"❌ {class_name} '{self.name}' check failed: {e}")
            self.state = State.FAILED

    def rest(self):
        self.state = State.NOT_STARTED

    def __repr__(self)-> str:
        return f"<{self.__class__.__name__} '{self.name}' — State: {self.state}>"

    def bind_connection(self, connection: mavutil.mavlink_connection) -> None:
        self.conn = connection  # Set later from the parent Action
        print(f"{self.__class__.__name__} '{self.name}' is now connected ✅🔗")

class Step(MissionElement):
    def __init__(self, name: str, 
                 check_fn: Callable[[mavutil.mavlink_connection, bool], bool],
                 exec_fn: Optional[Callable[[mavutil.mavlink_connection, bool], None]] = None
                ) -> None:
        # No connection here
        super().__init__(name=name, check_fn=check_fn, exec_fn=exec_fn)


class Action(MissionElement):
    def __init__(self, name: str)-> None:
        self.steps: List[Union[Step, Action]] = []
        self.current: Optional[Union[Step, Action]]= None
        super().__init__(name=name,  check_fn=self.run)  # ✅ no-op

    def add(self, step: Step) -> None:
        """
        Adds a Step or Action to this Action/Plan.
        Maintains chaining via `next` and updates current element.
        """
        if self.steps:
            self.steps[-1].next = step
        self.steps.append(step)
        if not self.current:
            self.current = step

    def run(self,connection):     
        step=self.current
        if step.state == State.NOT_STARTED:
            step.execute(connection)
        elif step.state == State.IN_PROGRESS:
            step.check()
        elif step.state == State.DONE: # This create an extra time for introducing other actions
            #step_class_name = step.__class__.__name__
            if step.next is None:
                self.state = State.DONE
                return True
            else:
                self.current = step.next 
        elif step.state == State.FAILED:
            step_class_name = step.__class__.__name__
            print(f"❌ {step_class_name}: '{step.name}' previously failed")
        return False
        
    def run_all(self,connection):
        while self.state!=State.DONE and self.state != State.FAILED:
            self.run(connection)

    def reset(self)-> None:
        for step in self.steps:
            step.reset()
        self.current = self.steps[0] if self.steps else None
        super().reset()

class Plan(Action):
    def __init__(self, name: str) -> None:
        super().__init__(name)