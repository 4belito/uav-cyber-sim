from typing import Callable, Optional
from pymavlink import mavutil
from typing import List


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
                 check_fn: Callable[[mavutil.mavlink_connection,bool], bool], 
                 exec_fn: Optional[Callable[[mavutil.mavlink_connection,bool], None]] = None
                ) -> None:
        self.name = name
        self.check_fn = check_fn
        self.exec_fn = exec_fn or (lambda conn, blocking: None)
        self.next = None
        self.state = State.NOT_STARTED
        self.conn = None
        print(f"{self.__class__.__name__} '{self.name}' created — no connection yet 🧩")


    def execute(self,conn:mavutil.mavlink_connection,blocking:bool)->None:
        class_name = self.__class__.__name__
        try:
            self.bind_connection(conn)
            self.exec_fn(self.conn, blocking)
            print(f"▶️ Starting {class_name}: {self.name}")
            self.state = State.IN_PROGRESS
        except StepFailed as e:
            print(f"❌ {class_name} '{self.name}' execution failed: {e}")
            self.state = State.FAILED

    def check(self,blocking:bool)->None:
        class_name = self.__class__.__name__
        try:
            if self.check_fn(self.conn, blocking):
                print(f"✅ {class_name}: {self.name} is done")
                self.state = State.DONE
        except StepFailed as e:
            print(f"❌ {class_name} '{self.name}' check failed: {e}")
            self.state = State.FAILED


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
        self.steps: List[Step] = []
        self.current_step: Optional[Step]= None
        super().__init__(name=name,  check_fn=self._check_step)  # ✅ no-op

    def add_step(self, step: Step) -> None:
        if self.steps:
            self.steps[-1].next = step
        self.steps.append(step)
        if not self.current_step:
            self.current_step = step

    def _check_step(self,connection,blocking=False):     
        step=self.current_step
        if step.state == State.NOT_STARTED:
            step.execute(connection,blocking)
        elif step.state == State.IN_PROGRESS:
            step.check(blocking)
        elif step.state == State.DONE: # This create an extra time for introducing other actions
            step_class_name = step.__class__.__name__
            if step.next == None:
                self.state = State.DONE
            else:
                self.current_step = step.next 
        elif step.state == State.FAILED:
            step_class_name = step.__class__.__name__
            print(f"⚠️ {step_class_name}: '{step.name}' previously failed")
        

    # This is temporal until plan class
    def run(self,connection:mavutil.mavlink_connection,blocking:str=True)-> bool:
        """Runs the action by executing and checking all steps."""    
        if self.current_step is None:
            raise RuntimeError(f"❌ Action '{self.name}' has no steps to run!")
        self.execute(connection,blocking=blocking)
        print(f"🚀 Running Action: {self.name}")
        if blocking:
            while self.state == State.IN_PROGRESS:
                self.check(blocking=False)
        else:
            self.check(blocking=False)
        return self.state == State.DONE   # All done

    def reset(self)-> None:
        for step in self.steps:
            step.state = State.NOT_STARTED
        self.current_step = self.steps[0] if self.steps else None
        self.state = State.NOT_STARTED


