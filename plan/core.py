from __future__ import annotations
from typing import Callable, Optional,List,Union
from pymavlink import mavutil


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
                name: str= 'action name', verbose:bool = False
                ) -> None:
        self.name = name
        self.prev = None
        self.next = None
        self.state = State.NOT_STARTED
        self.conn = None
        self.verbose = verbose
        if verbose:
            print(f"{self.__class__.__name__} '{self.name}' created — no connection yet 🧩")

    def act(self):
        pass

    def reset(self):
        self.state = State.NOT_STARTED
        
    def start(self,connection):
        self.state = State.IN_PROGRESS
        self.bind_connection(connection) 

    def __repr__(self) -> str:
        state_symbols = {
            State.NOT_STARTED: "🕓",
            State.IN_PROGRESS: "🚀",
            State.DONE: "✅",
            State.FAILED: "❌",
        }
        symbol = state_symbols.get(self.state, "❔")
        return f"{symbol} <{self.__class__.__name__} '{self.name}'>"

    def bind_connection(self, connection: mavutil.mavlink_connection) -> None:
        self.conn = connection  # Set later from the parent Action
        if self.verbose:
            print(f"Vehicle {self.conn.target_system}: {self.__class__.__name__} '{self.name}' is now connected ✅🔗")


class Step(MissionElement):
    def __init__(self, name: str, 
                 check_fn: Callable[[mavutil.mavlink_connection, bool], bool],
                 exec_fn: Optional[Callable[[mavutil.mavlink_connection, bool], None]] = None,
                 verbose:bool = False
                ) -> None:
        # No connection here
        self.exec_fn = exec_fn or (lambda conn: None)
        self.check_fn = check_fn 
        super().__init__(name=name,verbose = verbose)


    def execute(self)->None:
        class_name = self.__class__.__name__
        try:
            self.exec_fn(self.conn)
            print(f"Vehicle {self.conn.target_system}: ▶️ Starting {class_name}: {self.name}")
            self.state = State.IN_PROGRESS
        except Exception as e:
            print(f"Vehicle {self.conn.target_system}: ❌ {class_name} '{self.name}' execution failed: {e}")
            self.state = State.FAILED

    def check(self)->None:
        class_name = self.__class__.__name__
        try:
            if self.check_fn(self.conn):
                print(f"Vehicle {self.conn.target_system}: ✅ {class_name}: {self.name} is done")
                self.state = State.DONE
        except StepFailed as e:
            print(f"Vehicle {self.conn.target_system}: ❌ {class_name} '{self.name}' check failed: {e}")
            self.state = State.FAILED

    def act(self):
        if self.state == State.NOT_STARTED:
            self.execute()
        elif self.state == State.IN_PROGRESS:
            self.check()
        elif self.state == State.DONE:
            print("⚠️ Already done!. Cannot perform this step again!")
        elif self.state == State.FAILED:
            print("⚠️ Already failed!. Cannot perform this step again!")


class Action(MissionElement):
    def __init__(self, name: str,verbose:bool = False)-> None:
        self.steps: List[Union[Step, Action]] = []
        self.current: Optional[Union[Step, Action]]= None
        super().__init__(name=name,verbose=verbose)  # ✅ no-op

    def add(self, step: Union[Step, Action]) -> None:
        """
        Adds a Step or Action to this Action/Plan.
        Maintains chaining via `next` and updates current element.
        """
        if self.steps:
            self.steps[-1].next = step
            step.prev = self.steps[-1] 
        self.steps.append(step)
        if not self.current:
            self.current = step

    def act(self):   
        if self.state in [State.NOT_STARTED,State.IN_PROGRESS]:
            self.state = State.IN_PROGRESS
            step = self.current
            if step is None:
                self.state = State.DONE
            elif step.state==State.DONE:
                self.current = step.next 
                if self.current is None:
                    self.state = State.DONE
            elif step.state == State.FAILED:
                self.state = State.FAILED
                print("⚠️ Already failed!. Cannot perform this again!")
            else:
                step.act()
        elif self.state == State.DONE:
            print("⚠️ Already done!. Cannot perform this again!")
        elif self.state == State.FAILED:
            print("⚠️ Already failed!. Cannot perform this again!")
        

    def run_all(self):
        while self.state!=State.DONE and self.state != State.FAILED:
            self.run()

    def reset(self)-> None:
        # Change the stated of the steps to no started
        for step in self.steps:
            step.reset()
        # change current to the first step
        self.current = self.steps[0] if self.steps else None
        # change the action state to no statrted
        super().reset()

    def bind_connection(self, connection: mavutil.mavlink_connection) -> None:
        for step in self.steps:
            step.bind_connection(connection)
        super().bind_connection(connection)
        if self.verbose:
            print(f"Vehicle {self.conn.target_system}: {self.__class__.__name__} '{self.name}' is now connected ✅🔗")

    def __repr__(self) -> str:
        output = [super().__repr__()]
        for step in self.steps:
            indented = '\n'.join("  " + line for line in repr(step).splitlines())
            output.append(indented)
        return '\n'.join(output)


    def add_next(self, new_step: Union[Step, Action]) -> None:
        """
        Inserts a new step/action immediately after the current step.
        Maintains chaining and updates the 'next' pointers accordingly.
        current<new<next
        """
        if self.current is None:
            # If no current step exists, treat it like a normal add
            self.add(new_step)
            return

        next_step = self.current.next # save next step
        self.current.next = new_step  # current -> new
        new_step.prev = self.current  # current <- new
        new_step.next = next_step     #     new -> next
        if next_step:
            next_step.prev = new_step  #    new <- next 


        # Insert in the list just after the current step
        current_index = self.steps.index(self.current)
        self.steps.insert(current_index + 1, new_step)

    def add_now(self, new_step: Union[Step, Action]) -> None:
        """
        Inserts a new step/action immediately before the current step.
        Maintains chaining and updates the 'next' pointers accordingly.
        prev<new<current
        """
        if self.current is None:
            # If no current step exists, treat it like a normal add
            self.add(new_step)
            return

        prev_step = self.current.prev   # save prev_step
        new_step.next = self.current    # new -> current
        new_step.prev = prev_step       # prev <- new
        self.current.prev = new_step    # new <- current
        if prev_step:
            prev_step.next = new_step   # prev -> new

        current_index = self.steps.index(self.current)
        self.steps.insert(current_index, new_step)
        
        self.current.reset()
        self.current = new_step