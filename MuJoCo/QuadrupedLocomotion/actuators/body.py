from .arm import Arm


class Body:
    def __init__(self, arm1: Arm, arm2: Arm, arm3: Arm, arm4: Arm):
        self.arm1 = arm1
        self.arm2 = arm2
        self.arm3 = arm3
        self.arm4 = arm4
