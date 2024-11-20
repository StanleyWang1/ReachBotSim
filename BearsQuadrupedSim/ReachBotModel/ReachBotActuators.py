class Motor:
    def __init__(self, id, ctrl_limits):
        self.id = id
        self.ctrl_limits = ctrl_limits


class Arm:
    def __init__(self, motor1, revolute1, motor2, revolute2, motor3, prismatic):
        self.motor1 = motor1
        self.motor2 = motor2
        self.motor3 = motor3
        self.revolute1 = revolute1
        self.revolute2 = revolute2
        self.prismatic = prismatic

class Body:
    def __init__(self, arm1, arm2, arm3, arm4):
        self.arm1 = arm1
        self.arm2 = arm2
        self.arm3 = arm3
        self.arm4 = arm4

