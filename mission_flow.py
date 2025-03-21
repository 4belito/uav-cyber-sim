from typing import Callable, Optional
from abc import ABC, abstractmethod


NOT_STARTED = "NOT_STARTED"
IN_PROGRESS = "IN_PROGRESS"
DONE = "DONE"
FAILED = "FAILED"


class StepFailed(Exception):
    """Raised when a step fails due to known issues (like battery too low, GPS not ready, etc)."""
    pass



class MissionElement:
    def __init__(self, name: str, check_fn: Callable[[], bool], exec_fn: Optional[Callable[[bool], None]] = None):
        self.name = name
        self.check = check_fn
        self.exec = exec_fn or (lambda blocking: None)
        self.next = None
        self.state = NOT_STARTED

    def run(self, blocking=False):
        class_name = self.__class__.__name__

        if self.state == NOT_STARTED:
            try:
                self.exec(blocking)
                print(f"▶️ Starting {class_name}: {self.name}")
                self.state = IN_PROGRESS
            except StepFailed as e:
                print(f"❌ {class_name} '{self.name}' execution failed: {e}")
                self.state = FAILED

        elif self.state == IN_PROGRESS:
            try:
                if self.check(blocking):
                    print(f"✅ {class_name}: {self.name} is done")
                    self.state = DONE
            except StepFailed as e:
                print(f"❌ {class_name} '{self.name}' check failed: {e}")
                self.state = FAILED

        elif self.state == DONE:
            print(f"ℹ️ {class_name}: '{self.name}' already done")

        elif self.state == FAILED:
            print(f"⚠️ {class_name}: '{self.name}' previously failed")

    def __repr__(self):
        return f"<{self.__class__.__name__} '{self.name}' — State: {self.state}>"


class Step(MissionElement):
    def __init__(self, name: str, check_fn, exec_fn=None):
        super().__init__(name=name, check_fn=check_fn, exec_fn=exec_fn)

class Action(MissionElement):
    def __init__(self, name: str):
        self.steps = []
        self.current_step = None
        super().__init__(name=name, check_fn=self._run_steps, exec_fn=self._initiate_action)  # ✅ no-op

    def add_step(self, step: Step):
        if self.steps:
            self.steps[-1].next = step
        self.steps.append(step)
        if not self.current_step:
            self.current_step = step

    def _run_steps(self,blocking):
        while self.current_step:
            step = self.current_step
            step.run(blocking=blocking)

            if step.state == DONE:
                self.current_step = step.next
            elif step.state == FAILED:
                raise StepFailed(f"Step '{step.name}' failed in Action '{self.name}'")
            else:
                return False  # Still waiting

        return True  # All done

    def _initiate_action(self,blocking=False):
        self.state = IN_PROGRESS        

    def reset(self):
        for step in self.steps:
            step.state = NOT_STARTED
        self.current_step = self.steps[0] if self.steps else None
        self.state = NOT_STARTED


# class Step:
#     def __init__(self, name: str, check_fn: Callable[[], bool], exec_fn: Optional[Callable[[bool], None]] = None):
#         self.name = name
#         self.check = check_fn
#         self.exec = exec_fn or (lambda blocking: None)
#         self.next = None
#         self.state = "NOT_STARTED"


#     def run(self, blocking=False):
#         if self.state == NOT_STARTED:
#             try:
#                 print(f"▶️  Starting step: {self.name}")
#                 self.exec(blocking)
#                 self.state = IN_PROGRESS
#             except StepFailed as e:
#                 print(f"❌ Execution failed: {e}")
#                 self.state = FAILED

#         elif self.state == IN_PROGRESS:
#             try:
#                 if self.check():
#                     print(f"✅ Step: {self.name} is done")
#                     self.state = DONE
#             except StepFailed as e:
#                 print(f"❌ Check failed: {e}")
#                 self.state = FAILED

#         elif self.state == DONE:
#             print(f"ℹ️ Step: '{self.name}' already DONE.")

#         elif self.state == FAILED:
#             print(f"⚠️ Step: '{self.name}' is in FAILED state.")

#     def __repr__(self):
#         next_step = self.next.name if self.next else "None"
#         return f"<UAVStep name='{self.name}', state={self.state}, next='{next_step}'>"
